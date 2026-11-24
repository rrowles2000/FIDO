using KSP.Sim.impl;
using KSP.Sim;
using KSP.Sim.DeltaV;
using KSP.Sim.Maneuver;
using KSP2FlightAssistant.MathLibrary;
using KTools;
using UnityEngine.UIElements;
using Fido.UI;
using Fido.Controller;
using Fido.Node;
using Fido.KSPService;
using Fido.Landing;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido.Lift
{
    // Final ascent phase: circularizes the orbit at apoapsis with a closed-loop burn
    // (CircClosedLoop), planned with the same state-vector orbital math as precision landing's
    // Circularize (LandingTargeting). The node-based burn the Classic and Optimiser systems used
    // was removed with them on 2026-11-15 (Archive/lift-legacy-2026-11-15/). There is no "close
    // enough, skip" gating: Adjust's trim burn always leaves periapsis low, so there is real work.
    public class FinalCircularize : ExecuteController
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.Lift.FinalCircularize");

        LiftSettings lift_settings = null;
        KSPVessel current_vessel;
        LiftPilot lift;

        WarpTo warp = new WarpTo();

        // seconds before apoapsis to light the engines (half the burn, so it is centred on it)
        double ignite_lead_s;

        public FinalCircularize(LiftPilot lift, LiftSettings lift_settings)
        {
            current_vessel = FidoPlugin.Instance.current_vessel;
            this.lift = lift;
            this.lift_settings = lift_settings;
            closed_loop = new CircClosedLoop(current_vessel);
        }

        public override void Start()
        {
            base.Start();
            finished = false;

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
                $"burn_UT={burn_UT:n1} (T+{burn_UT - now:n1}s) deltaV={deltaV:n2}m/s burn~{t_burn:n0}s, ignition {ignite_lead_s:n0}s before apoapsis");

            // closed-loop burn (CircClosedLoop) rather than a node: a node burn aimed at one
            // instant's plan left weak upper stages at Ap 97 / Pe 85 km
            closed_phase = ClosedPhase.Align;
            ignite_ut = time_to_apoapsis > period / 2 ? now + lead_s : burn_UT - ignite_lead_s;
            closed_loop.Start(FirstStageThrust(current_vessel.VesselComponent));
            logger.LogInfo($"[FinalCircularize] closed loop: ~{deltaV:n0} m/s, burn ~{t_burn:n0} s, ignition in {ignite_ut - now:n0} s");
            status_line = $"Circularizing (closed loop): {deltaV:n1} m/s";
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

        // ---- closed loop
        enum ClosedPhase { Align, Warp, Burn }
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
            UpdateClosed();
        }

        public override void updateUI(VisualElement root_el, FullStatus st)
        {
            st.Status(string.IsNullOrEmpty(status_line) ? "Circularizing..." : status_line);
        }
    }
}
