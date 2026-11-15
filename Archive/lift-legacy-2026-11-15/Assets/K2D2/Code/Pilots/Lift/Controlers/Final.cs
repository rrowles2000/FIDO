using KSP.Sim.impl;
using KSP.Sim;
using KSP.Sim.DeltaV;
using KSP.Sim.Maneuver;
using KSP2FlightAssistant.MathLibrary;
using KTools;
using UnityEngine.UIElements;
using K2D2.UI;
using K2D2.Controller;
using K2D2.Node;
using K2D2.KSPService;
using K2D2.Landing;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift
{
    // Final ascent phase: circularizes the orbit at apoapsis, using the same state-vector orbital
    // math and node/execution pipeline as precision landing's Circularize/DeorbitBurn
    // (LandingTargeting for the orbital elements, ManeuverCreator.CreateManeuverNodeAtUT for the
    // node, Turn/Warp/Burn for execution). Unlike Landing's Circularize, there is no "close enough,
    // skip" or "too high, refuse" gating: Adjust's fine-tuning burn always leaves periapsis
    // low/suborbital, so this phase always has real circularizing work to do.
    public class FinalCircularize : ExecuteController
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Lift.FinalCircularize");

        LiftSettings lift_settings = null;
        KSPVessel current_vessel;
        LiftPilot lift;

        public enum Mode { Turn, Warp, Burn }
        public Mode mode = Mode.Turn;

        TurnTo turn = new TurnTo();
        WarpTo warp = new WarpTo();
        BurnManeuver burn = new BurnManeuver();
        ManeuverCreator maneuver_creator = new ManeuverCreator();

        public SingleExecuteController current_executor = new SingleExecuteController();
        ManeuverNodeData node;

        // seconds before the node to light the engines (half the burn, so it is centred on the
        // node); 0 = ignite at the node, as before
        double ignite_lead_s;
        const double LateTurnMaxDeg = 5;

        public FinalCircularize(LiftPilot lift, LiftSettings lift_settings)
        {
            current_vessel = K2D2_Plugin.Instance.current_vessel;
            this.lift = lift;
            this.lift_settings = lift_settings;
            sub_contollers.Add(current_executor);
            closed_loop = new CircClosedLoop(current_vessel);
        }

        public override void Start()
        {
            base.Start();
            finished = false;
            node = null;
            closed = false;

            if (current_vessel == null)
            {
                lift.EndLiftPilot(false, "No vessel");
                return;
            }

            // Zero throttle unconditionally on entry - Adjust's own finish branches already do
            // this, but re-zeroing here guarantees a clean start regardless of which phase ran
            // before, and avoids an uncommanded burn while TurnTo reorients toward the node.
            current_vessel.SetThrottle(0);

            PatchedConicsOrbit orbit = current_vessel.VesselComponent.Orbit;
            var body = orbit.referenceBody;

            double now = GeneralTools.Current_UT;
            Vector3d r_now = orbit.GetRelativePositionAtUTZup(now);
            Vector3d v_now = orbit.GetOrbitalVelocityAtUTZup(now);

            LandingTargeting.OrbitalElementsFromStateVectors(r_now, v_now, body.gravParameter,
                out _, out double apoapsisRadius, out double periapsisRadius);

            double period = LandingTargeting.OrbitalPeriodFromStateVectors(r_now, v_now, body.gravParameter);
            double time_to_apoapsis = LandingTargeting.TimeToNextApoapsis(r_now, v_now, body.gravParameter, period);
            double burn_UT = now + time_to_apoapsis;

            // Circularize at apoapsis: raise periapsis to meet the current apoapsis. Same
            // vis-viva math as Landing's Circularize.cs / ManeuverCreator.CircularizeOrbitApoapsis.
            double v_apoapsis = VisVivaEquation.CalculateVelocity(apoapsisRadius, apoapsisRadius, periapsisRadius, body.gravParameter);
            double v_circular = VisVivaEquation.CalculateVelocity(apoapsisRadius, apoapsisRadius, apoapsisRadius, body.gravParameter);
            double deltaV = v_circular - v_apoapsis;

            // Just past apoapsis (e.g. after stage-1 disposal's apoapsis burn): waiting for the next
            // apoapsis means first passing periapsis - on 2026-10-20 that was 49 km, inside the
            // atmosphere, and the upper stage burned up. If periapsis comes first, circularise
            // shortly instead, at the radius the vessel will be at then.
            //
            // Otherwise the node goes ON apoapsis, where prograde is horizontal, and the engines are
            // lit half the burn early (ignite_lead_s) so the burn is centred on it. Until 2026-11-13
            // the node itself was moved half a burn early with the apoapsis Δv as pure prograde: there
            // the path is still climbing (8-14° on the New system's lofted ascents), so the planned
            // orbit - which the burn then steers to - was eccentric: Ap 125 / Pe 39 km and
            // Ap 214 / Pe -54 km instead of 90 km circular.
            double t_burn = BurnTime(current_vessel.VesselComponent, deltaV);
            const double lead_s = 5; // time to turn onto the node; the vessel is already near prograde
            ignite_lead_s = 0;
            if (time_to_apoapsis <= period / 2)
                // 2 s, not lead_s: after Stage1Burn the vessel is already on the node's direction,
                // and the late-turn rule in Update covers a slow turn
                ignite_lead_s = System.Math.Max(0, System.Math.Min(t_burn / 2, time_to_apoapsis - 2));

            if (time_to_apoapsis > period / 2)
            {
                burn_UT = now + lead_s;
                Vector3d r_b = orbit.GetRelativePositionAtUTZup(burn_UT);
                Vector3d v_b = orbit.GetOrbitalVelocityAtUTZup(burn_UT);
                double v_circ_b = System.Math.Sqrt(body.gravParameter / r_b.magnitude);
                deltaV = System.Math.Max(0, v_circ_b - v_b.magnitude);
                logger.LogInfo($"[FinalCircularize] past apoapsis (next one in {time_to_apoapsis:n0} s, periapsis first) - " +
                               $"circularising in {lead_s:n0} s at {(r_b.magnitude - body.radius) / 1000:n1} km instead");
            }

            lift.last_circ_dv = deltaV;
            logger.LogInfo($"[FinalCircularize] now={now:n1} apoapsis={apoapsisRadius:n0} periapsis={periapsisRadius:n0} " +
                $"body={body.Name} period={period:n1}s time_to_apoapsis={time_to_apoapsis:n1}s | " +
                $"burn_UT={burn_UT:n1} (T+{burn_UT - now:n1}s) deltaV={deltaV:n2}m/s burn~{t_burn:n0}s, ignition {ignite_lead_s:n0}s before the node");

            // New lift system: closed-loop burn (CircClosedLoop) instead of a node - a node burn
            // aimed at one instant's plan left weak upper stages at Ap 97 / Pe 85 km
            if (lift.mode == LiftMode.New)
            {
                closed = true;
                closed_phase = ClosedPhase.Align;
                ignite_ut = time_to_apoapsis > period / 2 ? now + lead_s : burn_UT - ignite_lead_s;
                closed_loop.Start(FirstStageThrust(current_vessel.VesselComponent));
                logger.LogInfo($"[FinalCircularize] closed loop: ~{deltaV:n0} m/s, burn ~{t_burn:n0} s, ignition in {ignite_ut - now:n0} s");
                status_line = $"Circularizing (closed loop): {deltaV:n1} m/s";
                return;
            }
            closed = false;

            maneuver_creator.Update();
            // Clear any existing node before adding ours - AddNodeToVessel only appends, so a
            // player-made node or a leftover from an aborted run would otherwise leave two nodes on
            // the plan. Uses the remove-then-create-next-frame helper (see ManeuverCreator.cs):
            // removing and creating in the same frame can leave the new node broken.
            status_line = $"Circularizing: {deltaV:n1} m/s";
            maneuver_creator.RemoveAllNodesThenCreate(burn_UT, deltaV, created_node =>
            {
                node = created_node;
                mode = Mode.Turn;
                current_executor.setController(turn);
                turn.StartManeuver(node);
            });
        }

        // burn time for dv across the stages that will fire, in firing order, from the game's Δv
        // data (0 if unknown). Stage 1 often still has fuel at the New system's cut-off and is
        // burned dry during this burn before the upper stage takes over (2026-11-13), so counting
        // only the first stage got the centring wrong.
        static double BurnTime(VesselComponent vessel, double dv)
        {
            var info = vessel?.VesselDeltaV?.StageInfo;
            if (info == null) return 0;
            var stages = new System.Collections.Generic.List<DeltaVStageInfo>();
            foreach (var s in info)
                if (s != null && s.ThrustVac > 0 && s.IspVac > 0 && s.StartMass > s.EndMass) stages.Add(s);
            stages.Sort((a, b) => b.Stage.CompareTo(a.Stage)); // highest stage number fires first
            double t = 0, left = dv;
            for (int i = 0; i < stages.Count && left > 0; i++)
            {
                var s = stages[i];
                double m0 = i == 0 ? vessel.totalMass : s.StartMass;
                if (m0 <= s.EndMass) continue;
                double ve = s.IspVac * 9.80665;
                double stage_dv = ve * System.Math.Log(m0 / s.EndMass);
                double use = System.Math.Min(left, stage_dv);
                t += Stage1Burn.BurnTime(m0, s.ThrustVac, s.IspVac, use);
                left -= use;
            }
            return t;
        }

        // ---- closed loop (New lift system)
        enum ClosedPhase { Align, Warp, Burn }
        bool closed;
        ClosedPhase closed_phase;
        double ignite_ut;
        CircClosedLoop closed_loop;

        static double FirstStageThrust(VesselComponent vessel)
        {
            var info = vessel?.VesselDeltaV?.StageInfo;
            if (info == null) return 0;
            DeltaVStageInfo first = null;
            foreach (var s in info)
                if (s != null && s.ThrustVac > 0 && (first == null || s.Stage > first.Stage)) first = s;
            return first?.ThrustVac ?? 0;
        }

        void UpdateClosed()
        {
            double now = GeneralTools.Current_UT;
            switch (closed_phase)
            {
                case ClosedPhase.Align:
                {
                    double angle = closed_loop.Point(out _);
                    status_line = $"Lining up for circularisation: {angle:n1}°, ignition in {ignite_ut - now:n0} s";
                    if (angle < 5 || now >= ignite_ut - 2)
                    {
                        closed_phase = ClosedPhase.Warp;
                        warp.Start_Ut(ignite_ut - 2);
                    }
                    break;
                }
                case ClosedPhase.Warp:
                    closed_loop.Point(out _);
                    warp.Update();
                    status_line = $"Circularisation in {ignite_ut - now:n0} s";
                    if (now >= ignite_ut)
                    {
                        TimeWarpTools.SetRateIndex(0, false);
                        closed_phase = ClosedPhase.Burn;
                    }
                    break;
                case ClosedPhase.Burn:
                    closed_loop.Update();
                    status_line = closed_loop.status;
                    if (closed_loop.finished)
                    {
                        finished = true;
                        lift.EndLiftPilot(true, "Circularized - ascent complete");
                    }
                    break;
            }
        }

        public override void Update()
        {
            if (finished) return;
            if (closed) { UpdateClosed(); return; }
            if (node == null) return;

            base.Update();

            // TurnTo waits for 0.5° and settled rotation; on 2026-11-13 run 2 that took ~48 s and the
            // burn lit 10 s late, so the upper stage's burn ended up entirely after apoapsis
            // (Ap 110 / Pe 66 km). At the ignition time, close enough is good enough: SAS keeps
            // steering onto the node during the burn.
            if (mode == Mode.Turn && node != null && GeneralTools.Current_UT >= node.Time - ignite_lead_s - 0.5 &&
                turn.angle < LateTurnMaxDeg)
            {
                logger.LogInfo($"[FinalCircularize] ignition time reached while still lining up ({turn.angle:n1}° off) - burning, SAS keeps turning");
                mode = Mode.Burn;
                current_executor.setController(burn);
                burn.StartManeuver(node);
                burn.UT = node.Time - ignite_lead_s;
                return;
            }

            if (!current_executor.finished)
                return;

            switch (mode)
            {
                case Mode.Turn:
                    mode = Mode.Warp;
                    current_executor.setController(warp);
                    // Same timewarp-spam fix as DeorbitBurn/Landing's Circularize: no
                    // check_direction flag on StartManeuver.
                    warp.StartManeuver(node);
                    warp.UT = node.Time - ignite_lead_s; // stop warping before the early ignition
                    break;
                case Mode.Warp:
                    mode = Mode.Burn;
                    current_executor.setController(burn);
                    burn.StartManeuver(node);
                    burn.UT = node.Time - ignite_lead_s; // centre the burn on the node
                    break;
                case Mode.Burn:
                    status_line = "Circularize burn complete";
                    finished = true;

                    // Auto-delete the circularize node once its burn is done - nothing else needs
                    // it, and leaving it on the plan would sit there for the rest of the flight.
                    // Uses RemoveNode, not RemoveAllNodes: this is the only node FinalCircularize
                    // ever creates.
                    maneuver_creator.Update();
                    maneuver_creator.RemoveNode(node);

                    lift.EndLiftPilot(true, "Circularized - ascent complete");
                    break;
            }
        }

        public override void updateUI(VisualElement root_el, FullStatus st)
        {
            st.Status(string.IsNullOrEmpty(status_line) ? "Circularizing..." : status_line);
        }
    }
}
