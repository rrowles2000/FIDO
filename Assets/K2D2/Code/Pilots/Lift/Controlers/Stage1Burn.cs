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

        public enum Mode { Settle, Turn, Warp, Burn, Separate }
        public Mode mode;
        TurnTo turn = new TurnTo();
        WarpTo warp = new WarpTo();
        BurnManeuver burn = new BurnManeuver();
        ManeuverCreator maneuver_creator = new ManeuverCreator();
        public SingleExecuteController current_executor = new SingleExecuteController();
        ManeuverNodeData node;
        PartComponent separator;
        double settle_until, next_stage_ut;
        int stage_attempts;
        string status_line = "";

        // Never ask stage 1 for all of its Δv: if it ran dry mid-burn the burn would never finish.
        const double UsableFraction = 0.97;
        const double SettleSeconds = 2;

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
            node = null;
            var vessel = current_vessel?.VesselComponent;
            separator = Stage1Model.FindMarkedSeparator(vessel);
            if (!lift_settings.disposal_enabled.V || vessel == null || separator == null)
            {
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

            double capR = System.Math.Min(body.radius + lift_settings.disposal_pe_km.V * 1000.0, apR);
            double v_ap = VisVivaEquation.CalculateVelocity(apR, apR, peR, body.gravParameter);
            double v_cap = VisVivaEquation.CalculateVelocity(apR, apR, capR, body.gravParameter);
            double dv_cap = System.Math.Max(0, v_cap - v_ap);

            // what stage 1 has left (its last burn phase's Isp, down to its end mass)
            double dv_avail = double.PositiveInfinity;
            var model = Stage1Model.Build(vessel);
            if (model.valid)
            {
                var last = model.phases[model.phases.Count - 1];
                double m = vessel.totalMass;
                dv_avail = m > last.endMass ? last.ispVac * 9.80665 * System.Math.Log(m / last.endMass) : 0;
            }
            double dv = System.Math.Min(dv_cap, UsableFraction * dv_avail);
            logger.LogInfo($"[Stage1Burn] Ap {(apR - body.radius) / 1000:n1} km, Pe {(peR - body.radius) / 1000:n1} km: " +
                           $"{dv_cap:n0} m/s to the {lift_settings.disposal_pe_km.V:n0} km cap, stage 1 has {dv_avail:n0} m/s " +
                           $"-> node {dv:n1} m/s at T+{burn_UT - now:n0} s");

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

        void StartSeparation(string status)
        {
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
                    status_line = "Stage 1 separated";
                    finished = true;
                    return;
            }

            if (node == null) return;
            base.Update();
            if (!current_executor.finished) return;
            switch (mode)
            {
                case Mode.Turn:
                    mode = Mode.Warp;
                    current_executor.setController(warp);
                    warp.StartManeuver(node);
                    break;
                case Mode.Warp:
                    mode = Mode.Burn;
                    current_executor.setController(burn);
                    burn.StartManeuver(node);
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
