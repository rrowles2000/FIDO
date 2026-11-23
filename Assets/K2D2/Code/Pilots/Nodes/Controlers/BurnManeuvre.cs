using System;
using K2D2.KSPService;
using KSP.Sim;
using KSP.Sim.Maneuver;
using KTools;
using K2D2.UI;

// using KTools.UI;
using UnityEngine;
using UnityEngine.UIElements;
using K2D2.Controller;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Node
{
    class BurnManeuverSettings
    {
        public static Setting<float> burn_adjust = new ("node_ex.burn.burn_adjust", 1.5f);
        public static Setting<float> max_dv_error = new ("node_ex.burn.max_dv_error", 0.1f);
        // Default flipped to true (was false): keeping SAS locked onto the live maneuver vector
        // for the whole burn, instead of dropping to plain stability-hold the moment burning
        // starts, is what Node/Lift/Landing's burns should all do out of the box - see the
        // Mode.Waiting -> Mode.Burning transition below. This is a single global static shared by
        // every BurnManeuver in the mod (Node's own burns, Lift's final circularize, and Landing's
        // Circularize/DeorbitBurn/MidCourseCorrection all construct their burns through this same
        // class), so this one flip changes all of them at once - intentionally, since more
        // accurate burns are wanted everywhere, not just here. The toggle itself (Node tab's
        // Experimental section) is left in place for anyone who wants the old fixed-orientation
        // behavior back.
        public static Setting<bool> rotate_during_burn = new ("node_ex.burn.rotate_during_burn", true);
        // K2D2 owns the burn (2026-11-23, default on): "remaining" = node Δv minus the Δv the engines
        // have delivered along the node direction, counted every frame from their real thrust, the
        // vessel's mass and the nose's angle to the node - see BurnManeuver.OwnRemaining. Off = the
        // old way: the game's maneuver solver's "velocity after the burn" minus the current velocity,
        // which the game models over its own BurnDuration (811 s for a 72 s burn on a constellation
        // carrier: the burn stopped half done). Global, like rotate_during_burn.
        public static Setting<bool> own_burn = new ("node_ex.burn.own_burn", true);
    }

    public class BurnManeuver : ExecuteController
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.BurnManeuver");

        BurndV burn_dV = new BurndV();
        KSPVessel current_vessel;

        public BurnManeuver()
        {
            current_vessel = K2D2_Plugin.Instance.current_vessel;
        }

        double start_dt;

        public enum Mode
        {
            Waiting,
            Burning
        }

        Mode mode = Mode.Waiting;

        public double remaining_dv;
        public double last_remaining_dv = -1;


        public float needed_throttle = 0;

        // m/s²; >0 caps the throttle so a light vessel isn't shoved at full thrust (Constellation
        // sets it - the carrier gets lighter with every satellite released and starts to wobble)
        public double max_accel = 0;
        public float remaining_full_burn_time = 0;

        // Opt-in: end the burn when the engines stop producing thrust while throttled up (the
        // stage ran dry). Off by default because the Node pilot relies on StagingPilot staging
        // through a flameout mid-burn. Stage1Burn turns it on so stage 1 can be asked for nearly
        // all its Δv: if it runs dry the burn ends and separation carries on (2026-11-15: a 3%
        // reserve, held back only to avoid a burn that never finishes, cost ~21 km of Pe).
        public bool end_on_flameout = false;
        const double FlameoutGraceS = 2;    // after ignition, for the engines to spool up
        const double FlameoutHoldS = 0.5;   // no thrust for this long = flamed out
        const float FlameoutMinThrottle = 0.05f;
        double no_thrust_since = -1;

        ManeuverNodeData maneuver;

        public void StartManeuver(ManeuverNodeData node)
        {
            maneuver = node;
            UT = node.Time;
            Start();
        }

        public override void Start()
        {
            finished = false;
            mode = Mode.Waiting;
            remaining_dv = 0;
            nan_since = -1;
            no_thrust_since = -1;
            last_remaining_dv = -1;

            if (current_vessel == null) return;

            SASTool.setAutoPilot(AutopilotMode.Maneuver);

            // compute initial direction

            double ut;

            Vector velocity_after_maneuver = current_vessel.VesselComponent.Orbiter.ManeuverPlanSolver.GetVelocityAfterFirstManeuver(out ut);
            var current_vel = current_vessel.VesselComponent.Orbit.GetOrbitalVelocityAtUTZup(ut);
            initial_dir = velocity_after_maneuver.vector - current_vel;
            delivered_dv = 0;
            LogGeometry("start", velocity_after_maneuver, ut, current_vel);
        }

        // ---------------------------------------------------------------- own burn (2026-11-23)
        // Δv delivered along the node direction since ignition: thrust acceleration (engines' real
        // thrust / mass) x cos(nose to node direction) x game time, summed every frame. Frame-free
        // and the same for prograde, normal and radial nodes; independent of the game's solver.
        double delivered_dv, last_count_ut;
        bool owning; // own_burn latched at ignition

        double OwnRemaining(double now_ut)
        {
            double dt = now_ut - last_count_ut;
            last_count_ut = now_ut;
            if (dt > 0 && dt < 5)
            {
                double a = NodeAngleDeg();
                double cos = double.IsNaN(a) ? 1 : Math.Max(0, Math.Cos(a * Math.PI / 180));
                delivered_dv += burn_dV.actual_dv * cos * dt;
            }
            return maneuver.BurnRequiredDV - delivered_dv;
        }

        /// <summary>Angle (°) of the vessel's nose (control part up) to the game's node direction; NaN without one.</summary>
        double NodeAngleDeg()
        {
            try
            {
                var tel = SASTool.getTelemetry();
                if (tel == null || !tel.HasManeuver) return double.NaN;
                var dir = tel.ManeuverDirection;
                var r = Rotation.Reframed(current_vessel.GetRotation(), dir.coordinateSystem);
                Vector3d fwd = (r.localRotation * Vector3.up).normalized;
                return Vector3d.Angle(dir.vector, fwd);
            }
            catch { return double.NaN; }
        }

        /// <summary>Burn time at the capped acceleration (max_accel when > 0), from the node Δv - K2D2's own, not the game's.</summary>
        double OwnBurnTime()
        {
            burn_dV.LateUpdate();
            double a = burn_dV.full_dv;
            if (!(a > 0)) return double.NaN;
            if (max_accel > 0) a = Math.Min(a, max_accel);
            return maneuver.BurnRequiredDV / a;
        }

        // ---------------------------------------------------------------- diagnostics (2026-11-23)
        // A constellation altitude burn on an 81° orbit started with 978 m/s "remaining" for a 359 m/s
        // node and then burned the wrong way for two minutes. These lines show, at start, ignition and
        // every LogPeriodS while burning, whether the node, the target velocity or the pointing is off:
        //   node      the game's node: time, required Δv, raw BurnVector
        //   target    the frame of the solver's after-burn velocity, its speed minus the current
        //             speed at the node (frame-independent: ~node Δv for a pure prograde burn) and
        //             the vector difference K2D2 steers by (they disagree if the frames don't match)
        //   split     that difference along prograde / normal / radial-out of the current orbit at
        //             the node (Zup, the frame GetOrbitalVelocityAtUTZup uses)
        //   pointing  angle of the vessel's nose (control part 'up') to the game's own node direction,
        //             to prograde and to retrograde, all in the game's frames

        static string V3(Vector3d v) => $"({v.x:0.0}, {v.y:0.0}, {v.z:0.0})";

        void LogGeometry(string what, Vector velocity_after, double ut, Vector3d current_vel)
        {
            try
            {
                var vc = current_vessel?.VesselComponent;
                if (vc == null) return;
                var delta = velocity_after.vector - current_vel;
                string node = maneuver == null ? "none" :
                    $"T {maneuver.Time - GeneralTools.Game.UniverseModel.UniverseTime:n1}s, Δv {maneuver.BurnRequiredDV:n1}, BurnVector {V3(maneuver.BurnVector)}";
                string frame = velocity_after.coordinateSystem?.ToString() ?? "null";

                var pos = vc.Orbit.GetRelativePositionAtUTZup(ut);
                var pro = current_vel.normalized;
                var nrm = Vector3d.Cross(pos, current_vel).normalized;
                var rad = Vector3d.Cross(current_vel, nrm).normalized; // radial-out, orthogonal to prograde

                logger.LogInfo($"[Burn] {what} geometry: {(BurnManeuverSettings.own_burn.V ? "K2D2 owns the burn" : "solver burn")} | node {node} | target frame {frame}, |after| {velocity_after.vector.magnitude:n1} - |now| {current_vel.magnitude:n1} = " +
                               $"{velocity_after.vector.magnitude - current_vel.magnitude:n1} m/s, |after - now| {delta.magnitude:n1} m/s | " +
                               $"split prograde {Vector3d.Dot(delta, pro):n1} normal {Vector3d.Dot(delta, nrm):n1} radial-out {Vector3d.Dot(delta, rad):n1} | {Pointing()}");
            }
            catch (Exception e) { logger.LogWarning("[Burn] geometry log failed: " + e.Message); }
        }

        string Pointing()
        {
            try
            {
                var tel = SASTool.getTelemetry();
                if (tel == null) return "pointing: no telemetry";
                Rotation rot = current_vessel.GetRotation();
                double Ang(Vector dir)
                {
                    var r = Rotation.Reframed(rot, dir.coordinateSystem);
                    Vector3d fwd = (r.localRotation * Vector3.up).normalized;
                    return Vector3d.Angle(dir.vector, fwd);
                }
                string man = tel.HasManeuver ? $"{Ang(tel.ManeuverDirection):n1}°" : "no node";
                var pro = tel.OrbitMovementPrograde;
                return $"pointing: nose to node direction {man}, to prograde {Ang(pro):n1}°, to retrograde {180 - Ang(pro):n1}°, autopilot {SASTool.currentAutoPilot()?.AutopilotMode}";
            }
            catch (Exception e) { return "pointing: " + e.Message; }
        }

        Vector3d initial_dir = Vector3d.zero;

        // The game's maneuver solver can hand back NaN (2026-11-14: a stage-1 apoapsis burn sat at
        // throttle 0 for 76 s, Math.Sign throwing every frame, because the burn's reference
        // direction was taken once at Start while it was NaN). Now the reference is re-taken from
        // the first finite value, NaN frames are skipped, and after NaNTimeoutS the burn ends so
        // the pipeline can carry on (Stage1Burn separates, FinalCircularize re-plans).
        const double NaNTimeoutS = 3;
        double nan_since = -1;
        static bool Finite(Vector3d v) =>
            !(double.IsNaN(v.x) || double.IsNaN(v.y) || double.IsNaN(v.z) ||
              double.IsInfinity(v.x) || double.IsInfinity(v.y) || double.IsInfinity(v.z));
        int sign = 0;
        float angle;
        bool log_geometry_at_ignition;
        double solver_remaining = double.NaN;

        public double UT = 0;

        // [Burn] log: ignition, every LogPeriodS while burning, and the end - nothing about the
        // burn reached the log before 2026-11-13, when two circularisations went badly wrong
        const double LogPeriodS = 2.0;
        double next_log_ut;
        double ignition_ut;

        void LogBurn(string what)
        {
            var v = current_vessel?.VesselComponent;
            var o = v?.Orbit;
            if (o == null) return;
            double R = o.referenceBody.radius;
            double now = GeneralTools.Game.UniverseModel.UniverseTime;
            logger.LogInfo($"[Burn] {what}: {(now - UT >= 0 ? "+" : "")}{now - UT:n1}s from planned ignition, remaining {remaining_dv:n1} m/s" +
                           (owning ? $" (delivered {delivered_dv:n1}, game solver says {solver_remaining:n1})" : "") + ", " +
                           $"{(owning ? "nose off node" : "angle off start")} {angle:n1}°, throttle {last_throttle:n2}, Ap {(o.Apoapsis - R) / 1000:n1} Pe {(o.Periapsis - R) / 1000:n1} km, " +
                           $"inc {o.inclination:n2}°, mass {v.totalMass:n2} t");
        }

        public override void Update()
        {
            if (maneuver == null) return;

            if (finished)
                return;

            if (mode == Mode.Waiting)
            {
                start_dt = UT - GeneralTools.Game.UniverseModel.UniverseTime;
                if (start_dt > 0)
                {
                    status_line = $"Ignition in {StrTool.DurationToString(start_dt)}";
                    // K2D2's own burn time (the game's timer can be wildly off, and assumes full thrust)
                    double own_t = OwnBurnTime();
                    if (!double.IsNaN(own_t))
                        status_line += $" - burn {StrTool.DurationToString(own_t)}" + (max_accel > 0 && burn_dV.full_dv > max_accel ? $" at {max_accel:0.#} m/s²" : "");
                    set_throttle(0);
                    return;
                }
                else
                {
                    mode = Mode.Burning;
                    ignition_ut = GeneralTools.Game.UniverseModel.UniverseTime;
                    next_log_ut = 0;
                    owning = BurnManeuverSettings.own_burn.V;
                    delivered_dv = 0;
                    last_count_ut = ignition_ut;
                    remaining_dv = maneuver.BurnRequiredDV;
                    LogBurn($"ignition (node Δv {maneuver.BurnRequiredDV:n1} m/s, K2D2 burn time {OwnBurnTime():n1} s, game's {maneuver.BurnDuration:n1} s" +
                            $"{(owning ? "" : ", solver burn")})");
                    log_geometry_at_ignition = true;

                    if (BurnManeuverSettings.rotate_during_burn.V)
                        SASTool.setAutoPilot(AutopilotMode.Maneuver);
                    else
                        SASTool.setAutoPilot(AutopilotMode.StabilityAssist);

                }
            }

            if (mode == Mode.Burning)
            {
                burn_dV.LateUpdate();

                if (maneuver == null) return;

                // ActiveVessel = GameManager.Instance?.Game?.ViewController?.GetActiveVehicle(true)?.GetSimVessel(true);
                double ut = 0;

                Vector velocity_after_maneuver = current_vessel.VesselComponent.Orbiter.ManeuverPlanSolver.GetVelocityAfterFirstManeuver(out ut);

                double dt = ut - GeneralTools.Game.UniverseModel.UniverseTime;

                // allow time warp up to what the auto warp can do.
                int max_warp_index = WarpToSettings.compute_wanted_warp_index(dt);
                if (max_warp_index == 1)
                    max_warp_index = 0; // no x2 speed

                if (max_warp_index < TimeWarpTools.CurrentRateIndex)
                    TimeWarpTools.SetRateIndex(max_warp_index, false);

                var current_vel = current_vessel.VesselComponent.Orbit.GetOrbitalVelocityAtUTZup(ut);
                double now_ut = GeneralTools.Game.UniverseModel.UniverseTime;
                if (log_geometry_at_ignition)
                {
                    log_geometry_at_ignition = false;
                    LogGeometry("ignition", velocity_after_maneuver, ut, current_vel);
                }
                var delta_speed_vector = velocity_after_maneuver.vector - current_vel;
                // the solver's view, kept for the log (and used when own_burn is off)
                solver_remaining = Finite(delta_speed_vector)
                    ? (Finite(initial_dir) && initial_dir.sqrMagnitude > 1e-6 ? Math.Sign(Vector3d.Dot(initial_dir, delta_speed_vector)) : 1) * delta_speed_vector.magnitude
                    : double.NaN;

                if (owning)
                {
                    remaining_dv = OwnRemaining(now_ut);
                    angle = (float)NodeAngleDeg();
                }
                else if (!Finite(delta_speed_vector))
                {
                    if (nan_since < 0)
                    {
                        nan_since = now_ut;
                        logger.LogWarning("[Burn] maneuver solver returned NaN - holding until it recovers");
                    }
                    if (now_ut - nan_since > NaNTimeoutS)
                    {
                        logger.LogWarning($"[Burn] maneuver solver NaN for {NaNTimeoutS:n0} s - ending this burn so the flight can carry on");
                        Finished();
                    }
                    else if (now_ut - nan_since > 0.5) set_throttle(0);
                    return;
                }
                if (nan_since >= 0)
                {
                    logger.LogInfo($"[Burn] maneuver solver recovered after {now_ut - nan_since:n1} s");
                    nan_since = -1;
                }
                if (!owning)
                {
                    if (!Finite(initial_dir) || initial_dir.sqrMagnitude < 1e-6) initial_dir = delta_speed_vector;
                    sign = Math.Sign(Vector3d.Dot(initial_dir, delta_speed_vector));

                    angle = (float)Vector3d.Angle(initial_dir, delta_speed_vector);
                    remaining_dv = sign * delta_speed_vector.magnitude;

                    if (last_remaining_dv > 0 && last_remaining_dv < 1 && remaining_dv > last_remaining_dv)
                    {
                        Finished();
                        return;
                    }
                }

                last_remaining_dv = remaining_dv;

                if (end_on_flameout && now_ut - ignition_ut > FlameoutGraceS)
                {
                    bool no_thrust = last_throttle > FlameoutMinThrottle && burn_dV.actual_dv < 0.01f;
                    if (!no_thrust) no_thrust_since = -1;
                    else if (no_thrust_since < 0) no_thrust_since = now_ut;
                    else if (now_ut - no_thrust_since > FlameoutHoldS)
                    {
                        logger.LogInfo($"[Burn] flameout with {remaining_dv:n1} m/s left - ending the burn");
                        Finished();
                        return;
                    }
                }

                // var required_dv = maneuver.BurnRequiredDV;
                // remaining_dv = required_dv - burn_dV.burned_dV;
                if (remaining_dv <= BurnManeuverSettings.max_dv_error.V)
                {
                    Finished();
                    return;
                }
                else
                {
                    compute_throttle();
                    set_throttle(needed_throttle);
                    status_line = $"{remaining_dv:n2} m/s remaining";
                    if (now_ut >= next_log_ut)
                    {
                        next_log_ut = now_ut + LogPeriodS;
                        LogBurn("burning");
                        logger.LogInfo($"[Burn]   {Pointing()}");
                    }
                }
            }
        }

        void Finished()
        {
            status_line = $"Burn complete - {remaining_dv:n2} m/s error";
            set_throttle(0);
            finished = true;
            LogBurn($"complete after {GeneralTools.Game.UniverseModel.UniverseTime - ignition_ut:n1}s");
        }

        float last_throttle = -1;

        public void set_throttle(float throttle)
        {
            throttle = Mathf.Clamp01(throttle);
            current_vessel.SetThrottle(throttle);
            last_throttle = throttle;
        }

        public void compute_throttle()
        {
            if (remaining_dv <= 0)
            {
                needed_throttle = 0;
                return;
            }

            remaining_full_burn_time = (float)(remaining_dv / burn_dV.full_dv);
            if (remaining_full_burn_time >= 1)
            {
                needed_throttle = 1;
                CapThrottle();
                return;
            }

            needed_throttle = remaining_full_burn_time * BurnManeuverSettings.burn_adjust.V;
            CapThrottle();
        }

        void CapThrottle()
        {
            if (max_accel > 0 && burn_dV.full_dv > max_accel)
                needed_throttle = Mathf.Min(needed_throttle, (float)(max_accel / burn_dV.full_dv));
        }

        public override void updateUI(VisualElement root_el, FullStatus st)
        {
            switch (mode)
            {
                case Mode.Waiting:
                    st.Status("Standing by...");
                    st.Console(status_line);
                    break;
                case Mode.Burning:
                    st.Warning("Burning!");
                    if (maneuver.BurnRequiredDV >= 0)
                        st.Progress(remaining_dv / maneuver.BurnRequiredDV, $"{remaining_dv:n1} m/s");

                    // dV remaining and time remaining on one line so they read side by side
                    // instead of stacked as two separate console lines.
                    st.Console($"{status_line}    |    {StrTool.DurationToString(remaining_full_burn_time)}");
                    break;
            }

            if (K2D2Settings.debug_mode.V)
            {
                if (maneuver == null) return;
                st.Console("-----------------");
                //st.Console($"start_dt {Tools.remainingStartTime(maneuver)}");
                //st.Console($"end_dt {Tools.remainingEndTime(maneuver)}");

                st.Console($"BurnRequiredDV {maneuver.BurnRequiredDV}");
                st.Console($"angle {angle}");
                st.Console($"sign {sign}");
                st.Console($"remaining_dv {remaining_dv}");
                st.Console($"remaining_full_burn_time {remaining_full_burn_time}");
                st.Console($"needed_throttle {needed_throttle}");

                //burn_dV.onGUI();
            }
        }
    }
}
