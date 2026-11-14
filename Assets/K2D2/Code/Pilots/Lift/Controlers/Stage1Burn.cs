using KSP.Modules;
using KSP.Sim.impl;
using KSP.Sim.Maneuver;
using KSP2FlightAssistant.MathLibrary;
using KTools;
using UnityEngine.UIElements;
using K2D2.UI;
using K2D2.Controller;
using K2D2.Node;
using K2D2.KSPService;
using K2D2.Landing;
using K2D2.Lift.Stage1;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift
{
    // Stage-1 disposal, after Coasting has warped clear of the atmosphere: one prograde node at
    // apoapsis for stage 1 - enough to lift periapsis to the cap, or (most of) what stage 1 has
    // left - flown with the same Turn/Warp/Burn pipeline as FinalCircularize, then stage until the
    // marked separator has fired. Does nothing unless disposal mode is on.
    public class Stage1Burn : ExecuteController
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Lift.Stage1Burn");
        LiftSettings lift_settings;
        KSPVessel current_vessel;
        LiftPilot lift;

        public enum Mode { Settle, Turn, Warp, Burn, Separate, Ignite }
        public Mode mode;
        TurnTo turn = new TurnTo();
        WarpTo warp = new WarpTo();
        BurnManeuver burn = new BurnManeuver();
        ManeuverCreator maneuver_creator = new ManeuverCreator();
        public SingleExecuteController current_executor = new SingleExecuteController();
        ManeuverNodeData node;
        PartComponent separator;
        // seconds before the node to light stage 1 (the node sits on apoapsis, see CreateNode)
        double ignite_lead_s;
        const double LateTurnMaxDeg = 5;
        double settle_until, next_stage_ut;
        int stage_attempts;
        string status_line = "";
        // set when the run ends needing the player (the upper engine wouldn't light safely)
        public string problem;

        // Never ask stage 1 for all of its Δv: if it ran dry mid-burn the burn would never finish.
        const double UsableFraction = 0.97;
        const double SettleSeconds = 2;
        const double SeparationGap = 4;  // s from stage-1 cut-off to upper ignition
        const double MinLead = 8;        // s to turn onto the node

        // seconds to burn dv from mass m0 (t) at thrust (kN) and Isp (s)
        public static double BurnTime(double m0, double thrustKN, double isp, double dv)
        {
            if (dv <= 0 || thrustKN <= 0 || isp <= 0) return 0;
            double ve = isp * 9.80665;
            return m0 * ve / thrustKN * (1 - System.Math.Exp(-dv / ve));
        }

        public Stage1Burn(LiftPilot lift, LiftSettings lift_settings)
        {
            current_vessel = K2D2_Plugin.Instance.current_vessel;
            this.lift = lift;
            this.lift_settings = lift_settings;
            sub_contollers.Add(current_executor);
        }

        public override void Start()
        {
            base.Start();
            finished = false;
            problem = null;
            node = null;
            var vessel = current_vessel?.VesselComponent;
            separator = Stage1Model.FindMarkedSeparator(vessel);
            if (!lift_settings.Stage1BurnOn(lift.mode) || vessel == null || separator == null)
            {
                if (lift.mode == LiftMode.New && lift_settings.next_s1_ap_burn.V && vessel != null && separator == null)
                    logger.LogWarning("[Stage1Burn] no stage-1 separator marked (StageGroup) - skipping the stage-1 apoapsis burn");
                finished = true;
                return;
            }
            // Coasting may have left warp on: stop it and give the game a moment before planning.
            TimeWarpTools.SetRateIndex(0, false);
            current_vessel.SetThrottle(0);
            settle_until = GeneralTools.Current_UT + SettleSeconds;
            mode = Mode.Settle;
            status_line = "Stage 1: settling after warp";
        }

        bool SeparatorAttached()
        {
            var parts = current_vessel.VesselComponent?.SimulationObject?.PartOwner?.Parts;
            if (separator == null || parts == null) return false;
            foreach (var p in parts)
                if (ReferenceEquals(p, separator)) return true;
            return false;
        }

        void CreateNode()
        {
            var vessel = current_vessel.VesselComponent;
            PatchedConicsOrbit orbit = vessel.Orbit;
            var body = orbit.referenceBody;
            double now = GeneralTools.Current_UT;
            Vector3d r_now = orbit.GetRelativePositionAtUTZup(now);
            Vector3d v_now = orbit.GetOrbitalVelocityAtUTZup(now);
            LandingTargeting.OrbitalElementsFromStateVectors(r_now, v_now, body.gravParameter,
                out _, out double apR, out double peR);
            double period = LandingTargeting.OrbitalPeriodFromStateVectors(r_now, v_now, body.gravParameter);
            double burn_UT = now + LandingTargeting.TimeToNextApoapsis(r_now, v_now, body.gravParameter, period);

            double cap_km = lift_settings.Stage1CapKm(lift.mode, body.hasAtmosphere ? body.atmosphereDepth / 1000 : 0);
            double capR = System.Math.Min(body.radius + cap_km * 1000.0, apR);
            double v_ap = VisVivaEquation.CalculateVelocity(apR, apR, peR, body.gravParameter);
            double v_cap = VisVivaEquation.CalculateVelocity(apR, apR, capR, body.gravParameter);
            double dv_cap = System.Math.Max(0, v_cap - v_ap);

            // what stage 1 has left (its last burn phase's Isp, down to its end mass)
            double dv_avail = double.PositiveInfinity;
            double m = vessel.totalMass;
            var model = Stage1Model.Build(vessel);
            Stage1Model.BurnPhase last = null, upper = null;
            if (model.valid)
            {
                last = model.phases[model.phases.Count - 1];
                upper = model.upperPhases[0];
                dv_avail = m > last.endMass ? last.ispVac * 9.80665 * System.Math.Log(m / last.endMass) : 0;
            }
            double dv = System.Math.Min(dv_cap, UsableFraction * dv_avail);

            // Centre the whole apoapsis burn - stage 1 plus whatever the upper stage still has to
            // add to circularise - on apoapsis. BurnManeuver ignites AT the node time, so on
            // 2026-10-20 (polar, stage 1 short: Pe -248 km at burnout) all 40 s of stage 1 burned
            // after apoapsis and the low-TWR upper stage started its 370 m/s falling at 74 km.
            // The node stays ON apoapsis (prograde is horizontal only there) and the engines are lit
            // early instead: moving the node itself early with a pure-prograde Δv plans an
            // eccentric orbit that the burn then steers to (FinalCircularize, 2026-11-13).
            double t1 = 0, t2 = 0;
            ignite_lead_s = 0;
            if (last != null)
            {
                double v_circ = VisVivaEquation.CalculateVelocity(apR, apR, apR, body.gravParameter);
                double dv2 = System.Math.Max(0, v_circ - v_ap - dv);
                t1 = BurnTime(m, last.thrustVac, last.ispVac, dv);
                t2 = BurnTime(upper.startMass, upper.thrustVac, upper.ispVac, dv2);
                ignite_lead_s = System.Math.Max(0, System.Math.Min((t1 + SeparationGap + t2) / 2, burn_UT - now - MinLead));
            }
            burn_UT = System.Math.Max(burn_UT, now + MinLead);
            lift.last_s1_burn_dv = dv;
            logger.LogInfo($"[Stage1Burn] Ap {(apR - body.radius) / 1000:n1} km, Pe {(peR - body.radius) / 1000:n1} km: " +
                           $"{dv_cap:n0} m/s to the {cap_km:n0} km cap, stage 1 has {dv_avail:n0} m/s " +
                           $"-> node {dv:n1} m/s on Ap at T+{burn_UT - now:n0} s, ignition {ignite_lead_s:n0} s early (stage 1 {t1:n0} s + upper {t2:n0} s, centred on Ap)");

            if (dv < 1)
            {
                StartSeparation("Stage 1: nothing to burn - separating");
                return;
            }
            status_line = $"Stage 1 apoapsis burn: {dv:n0} m/s";
            maneuver_creator.Update();
            maneuver_creator.RemoveAllNodesThenCreate(burn_UT, dv, created_node =>
            {
                node = created_node;
                mode = Mode.Turn;
                current_executor.setController(turn);
                turn.StartManeuver(node);
            });
        }

        // true if any engine left on board is lit; otherwise reports the activation stage of the
        // next engine to fire and whether a decoupler fires before or with it (higher stage index
        // fires first)
        bool UpperEngineIgnited(out int engine_stage, out bool decoupler_first)
        {
            engine_stage = -1;
            decoupler_first = false;
            var parts = current_vessel.VesselComponent?.SimulationObject?.PartOwner?.Parts;
            if (parts == null) return false;
            foreach (var p in parts)
            {
                if (!p.TryGetModule<PartComponentModule_Engine>(out var engine)) continue;
                if (engine.EngineIgnited) return true;
                engine_stage = System.Math.Max(engine_stage, p.ActivationStage);
            }
            foreach (var p in parts)
                if (p.HasModule<PartComponentModule_Decouple>() && p.ActivationStage >= engine_stage)
                    decoupler_first = true;
            return false;
        }

        void StartSeparation(string status)
        {
            var o = current_vessel?.VesselComponent?.Orbit;
            if (o != null)
            {
                lift.last_s1_sep_ap = (o.Apoapsis - o.referenceBody.radius) / 1000;
                lift.last_s1_sep_pe = (o.Periapsis - o.referenceBody.radius) / 1000;
            }
            status_line = status;
            current_vessel.SetThrottle(0);
            mode = Mode.Separate;
            stage_attempts = 0;
            next_stage_ut = GeneralTools.Current_UT + 0.3;
        }

        public override void Update()
        {
            if (finished) return;
            double now = GeneralTools.Current_UT;
            switch (mode)
            {
                case Mode.Settle:
                    maneuver_creator.Update();
                    if (now >= settle_until && node == null)
                    {
                        CreateNode();
                        settle_until = double.PositiveInfinity; // once
                    }
                    return;

                case Mode.Separate:
                    // stage until the marked separator has fired (stage 1 may still have booster stages)
                    if (SeparatorAttached() && stage_attempts < 6)
                    {
                        if (now >= next_stage_ut)
                        {
                            current_vessel.VesselComponent.ActivateNextStage();
                            stage_attempts++;
                            next_stage_ut = now + 0.5;
                        }
                        return;
                    }
                    logger.LogInfo($"[Stage1Burn] separated after {stage_attempts} staging(s)");
                    K2D2.Lift.Next.DebrisTracker.RegisterNew();
                    status_line = "Stage 1 separated - lighting the upper stage";
                    mode = Mode.Ignite;
                    stage_attempts = 0;
                    next_stage_ut = now + 0.5;
                    return;

                case Mode.Ignite:
                    // Separation alone leaves the upper engine off unless it shares the separator's
                    // stage; stage on until it lights - but never fire a stage that would also
                    // release a decoupler (e.g. a satellite).
                    if (now < next_stage_ut) return;
                    if (!UpperEngineIgnited(out int engine_stage, out bool decoupler_first) && stage_attempts < 3)
                    {
                        if (engine_stage < 0 || decoupler_first)
                        {
                            logger.LogWarning($"[Stage1Burn] upper engine not lit and not safe to stage " +
                                              $"(engine stage {engine_stage}, decoupler first {decoupler_first}) - light it by hand");
                            status_line = "Upper engine is off - stage it by hand";
                            problem = status_line;
                            finished = true;
                            return;
                        }
                        current_vessel.VesselComponent.ActivateNextStage();
                        stage_attempts++;
                        next_stage_ut = now + 0.5;
                        return;
                    }
                    logger.LogInfo($"[Stage1Burn] upper engine lit after {stage_attempts} staging(s)");
                    status_line = "Stage 1 separated, upper engine lit";
                    finished = true;
                    return;
            }

            if (node == null) return;
            base.Update();

            // TurnTo waits for 0.5° and settled rotation; on 2026-11-13 run 2 that took ~48 s and the
            // burn lit 10 s late, so the upper stage's burn ended up entirely after apoapsis
            // (Ap 110 / Pe 66 km). At the ignition time, close enough is good enough: SAS keeps
            // steering onto the node during the burn.
            if (mode == Mode.Turn && node != null && GeneralTools.Current_UT >= node.Time - ignite_lead_s - 0.5 &&
                turn.angle < LateTurnMaxDeg)
            {
                logger.LogInfo($"[Stage1Burn] ignition time reached while still lining up ({turn.angle:n1}° off) - burning, SAS keeps turning");
                mode = Mode.Burn;
                current_executor.setController(burn);
                burn.StartManeuver(node);
                burn.UT = node.Time - ignite_lead_s;
                return;
            }

            if (!current_executor.finished) return;
            switch (mode)
            {
                case Mode.Turn:
                    mode = Mode.Warp;
                    current_executor.setController(warp);
                    warp.StartManeuver(node);
                    warp.UT = node.Time - ignite_lead_s;
                    break;
                case Mode.Warp:
                    mode = Mode.Burn;
                    current_executor.setController(burn);
                    burn.StartManeuver(node);
                    burn.UT = node.Time - ignite_lead_s;
                    break;
                case Mode.Burn:
                    maneuver_creator.Update();
                    maneuver_creator.RemoveNode(node);
                    node = null;
                    var orbit = current_vessel.VesselComponent.Orbit;
                    logger.LogInfo($"[Stage1Burn] burn complete: Ap {(orbit.Apoapsis - orbit.referenceBody.radius) / 1000:n1} km, " +
                                   $"Pe {(orbit.Periapsis - orbit.referenceBody.radius) / 1000:n1} km, mass {current_vessel.VesselComponent.totalMass:n2} t");
                    StartSeparation("Stage 1 burn complete - separating");
                    break;
            }
        }

        public override void updateUI(VisualElement root_el, FullStatus st)
        {
            st.Status(string.IsNullOrEmpty(status_line) ? "Stage 1 burn" : status_line);
        }

        public override void UpdateInfoRows(System.Action<string, string> addRow)
        {
            addRow("Stage 1", status_line);
            addRow("Step", mode.ToString());
        }
    }
}
