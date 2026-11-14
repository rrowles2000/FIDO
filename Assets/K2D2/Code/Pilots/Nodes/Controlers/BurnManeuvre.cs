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
            last_remaining_dv = -1;

            if (current_vessel == null) return;

            SASTool.setAutoPilot(AutopilotMode.Maneuver);

            // compute initial direction

            double ut;

            Vector velocity_after_maneuver = current_vessel.VesselComponent.Orbiter.ManeuverPlanSolver.GetVelocityAfterFirstManeuver(out ut);
            var current_vel = current_vessel.VesselComponent.Orbit.GetOrbitalVelocityAtUTZup(ut);
            initial_dir = velocity_after_maneuver.vector - current_vel;
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
            logger.LogInfo($"[Burn] {what}: {(now - UT >= 0 ? "+" : "")}{now - UT:n1}s from planned ignition, remaining {remaining_dv:n1} m/s, " +
                           $"angle off start {angle:n1}°, throttle {last_throttle:n2}, Ap {(o.Apoapsis - R) / 1000:n1} Pe {(o.Periapsis - R) / 1000:n1} km, " +
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
                    // the game's burn timer assumes full thrust; say how long the capped burn really takes
                    if (max_accel > 0)
                    {
                        burn_dV.LateUpdate();
                        if (burn_dV.full_dv > max_accel)
                            status_line += $" - burn {StrTool.DurationToString(maneuver.BurnDuration * burn_dV.full_dv / max_accel)}" +
                                           $" at {max_accel:0.#} m/s² (game timer assumes full thrust)";
                    }
                    set_throttle(0);
                    return;
                }
                else
                {
                    mode = Mode.Burning;
                    ignition_ut = GeneralTools.Game.UniverseModel.UniverseTime;
                    next_log_ut = 0;
                    LogBurn($"ignition (node Δv {maneuver.BurnRequiredDV:n1} m/s)");

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
                var delta_speed_vector = velocity_after_maneuver.vector - current_vel;
                double now_ut = GeneralTools.Game.UniverseModel.UniverseTime;
                if (!Finite(delta_speed_vector))
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
                if (!Finite(initial_dir) || initial_dir.sqrMagnitude < 1e-6) initial_dir = delta_speed_vector;
                sign = Math.Sign(Vector3d.Dot(initial_dir, delta_speed_vector));

                angle = (float)Vector3d.Angle(initial_dir, delta_speed_vector);
                remaining_dv = sign * delta_speed_vector.magnitude;

                if (last_remaining_dv > 0 && last_remaining_dv < 1 && remaining_dv > last_remaining_dv)
                {
                    Finished();
                    return;
                }

                last_remaining_dv = remaining_dv;

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
