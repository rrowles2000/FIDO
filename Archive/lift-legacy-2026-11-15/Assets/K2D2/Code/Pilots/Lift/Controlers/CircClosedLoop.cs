using System;
using K2D2.KSPService;
using KSP.Modules;
using KSP.Sim;
using KSP.Sim.impl;
using KTools;
using UnityEngine;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift
{
    /// <summary>
    /// Closed-loop circularisation for the New lift system (2026-11-14).
    ///
    /// The node burn aims at the velocity a node planned at one instant asks for. With a weak upper
    /// stage (the 13 t test vessel: 60 kN on 2.9 t, a 39 s burn) the burn spans a long arc and every
    /// flight ended at Ap 97 / Pe 85 km instead of 90 circular. This steers continuously instead:
    ///
    ///   direction  horizontal prograde (in the orbit plane), pitched by p where
    ///              sin p = a_v / a,  a_v = -v_r / t_go + (g - v_h^2 / r)
    ///              i.e. take the vertical speed to zero by the end of the burn while holding
    ///              against what gravity isn't yet balanced by the horizontal speed
    ///   throttle   full until the last ~1.5 s of t_go, then easing off
    ///   end        horizontal speed at circular speed for the current radius, vertical speed ~0
    ///
    /// t_go = (v_circ - v_h) / a, with a from thrust / mass (thrust estimated as thrust / throttle,
    /// starting from the game's Δv data). Ignition is half the burn before apoapsis. A stage that
    /// runs dry mid-burn is staged (twice at most).
    /// </summary>
    public class CircClosedLoop
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Lift.Circ");

        public const double MaxPitchSin = 0.6;
        public const double EndDv = 0.3, EndVr = 3.0;
        public const double LogPeriodS = 2.0;

        readonly KSPVessel current_vessel;
        public bool finished;
        public string status = "";
        double full_thrust_est, start_ut, next_log_ut, no_thrust_since = -1, last_stage_ut;
        int stagings;
        bool started, warp_logged;
        double pitch_cmd = double.NaN;
        public const double PitchRateDegS = 3;

        public CircClosedLoop(KSPVessel vessel) { current_vessel = vessel; }

        public void Start(double initialThrustKN)
        {
            finished = false;
            started = false;
            warp_logged = false;
            pitch_cmd = double.NaN;
            full_thrust_est = initialThrustKN;
            next_log_ut = 0;
            no_thrust_since = -1;
            stagings = 0;
            status = "Closed-loop circularisation";
        }

        /// <summary>Hold the attitude for the burn without thrust (before ignition).</summary>
        public double Point(out Vector3d noseErrDir)
        {
            Compute(out var dir, out _, out _, out _, out _, out _);
            noseErrDir = dir;
            Steer(dir);
            current_vessel.SetThrottle(0);
            return NoseAngle(dir);
        }

        public void Update()
        {
            if (finished) return;
            var vessel = current_vessel.VesselComponent;
            if (vessel == null) { finished = true; return; }
            double now = GeneralTools.Current_UT;
            if (!started) { started = true; start_ut = now; }

            // No time warp during the burn: on 2026-11-14 the game went to 4x warp 14 s in, the vessel
            // went on rails, its attitude froze while the engine kept thrusting and the orbit ended
            // at Ap 111 / Pe 74 km.
            if (TimeWarpTools.CurrentRateIndex > 0)
            {
                TimeWarpTools.SetRateIndex(0, false);
                if (!warp_logged) logger.LogWarning("[Circ] time warp switched off - the burn needs real-time physics to steer");
                warp_logged = true;
            }

            Compute(out var dir, out double dv_h, out double vr, out double a, out double pitch, out double rr);
            Steer(dir);

            double t_go = a > 0 ? dv_h / a : double.PositiveInfinity;
            float throttle = dv_h <= 0 ? 0f : (float)Mathf.Clamp((float)(t_go / 1.5), 0.05f, 1f);
            // off the wanted direction: ease the throttle off smoothly (it used to go to 0 above 10°,
            // which with a lagging nose stalled the burn)
            double off = NoseAngle(dir);
            if (off > 5) throttle *= (float)Math.Max(0.2, Math.Cos(Math.Min(80, (off - 5) * 3) * Math.PI / 180));
            current_vessel.SetThrottle(throttle);

            // thrust estimate and dry-stage handling
            double thrust = Thrust(vessel);
            if (current_vessel.GetThrottle() > 0.1 && thrust > 0.5)
            {
                double full = thrust / current_vessel.GetThrottle();
                full_thrust_est = full_thrust_est <= 0 ? full : full_thrust_est + 0.2 * (full - full_thrust_est);
                no_thrust_since = -1;
            }
            else if (throttle > 0.1 && thrust < 0.5)
            {
                if (no_thrust_since < 0) no_thrust_since = now;
                if (now - no_thrust_since > 1 && stagings < 2 && now - last_stage_ut > 1)
                {
                    logger.LogInfo("[Circ] no thrust at full throttle - staging");
                    vessel.ActivateNextStage();
                    stagings++;
                    last_stage_ut = now;
                }
            }

            var orbit = vessel.Orbit;
            double R = orbit.referenceBody.radius;
            if (now >= next_log_ut)
            {
                next_log_ut = now + LogPeriodS;
                logger.LogInfo($"[Circ] +{now - start_ut:n1}s: {dv_h:n1} m/s to circular, v_r {vr:+0.0;-0.0} m/s, pitch {pitch:+0.0;-0.0}°, " +
                               $"throttle {throttle:n2}, a {a:n2} m/s², Ap {(orbit.Apoapsis - R) / 1000:n1} Pe {(orbit.Periapsis - R) / 1000:n1} km");
            }
            status = $"Circularising: {dv_h:n1} m/s to go, vertical {vr:+0.0;-0.0} m/s";

            if ((dv_h <= EndDv && System.Math.Abs(vr) < EndVr) || dv_h < -2)
            {
                current_vessel.SetThrottle(0);
                finished = true;
                logger.LogInfo($"[Circ] done after {now - start_ut:n1}s: {dv_h:n1} m/s left, v_r {vr:+0.0;-0.0} m/s, " +
                               $"Ap {(orbit.Apoapsis - R) / 1000:n1} Pe {(orbit.Periapsis - R) / 1000:n1} km, ecc {orbit.eccentricity:n4}");
            }
        }

        void Compute(out Vector3d dir_world, out double dv_h, out double vr, out double a, out double pitch, out double rr)
        {
            var vessel = current_vessel.VesselComponent;
            var orbit = vessel.Orbit;
            double mu = orbit.referenceBody.gravParameter;
            double now = GeneralTools.Current_UT;
            Vector3d r = orbit.GetRelativePositionAtUTZup(now);
            Vector3d v = orbit.GetOrbitalVelocityAtUTZup(now);
            rr = r.magnitude;
            vr = Vector3d.Dot(v, r) / rr;
            double vh = System.Math.Sqrt(System.Math.Max(0, v.sqrMagnitude - vr * vr));
            double vc = System.Math.Sqrt(mu / rr);
            dv_h = vc - vh;
            double g = mu / (rr * rr);
            double mass = vessel.totalMass;
            a = mass > 0 ? full_thrust_est / mass : 0;
            double t_go = a > 0 ? System.Math.Max(1, dv_h / a) : 1;
            double a_v = -vr / t_go + (g - vh * vh / rr);
            double sinp = a > 0 ? System.Math.Max(-MaxPitchSin, System.Math.Min(MaxPitchSin, a_v / a)) : 0;
            pitch = System.Math.Asin(sinp) * 180 / System.Math.PI;
            // pitch command rate-limited so the nose can follow it
            if (double.IsNaN(pitch_cmd)) pitch_cmd = pitch;
            double step = PitchRateDegS * UnityEngine.Time.deltaTime;
            pitch_cmd += System.Math.Max(-step, System.Math.Min(step, pitch - pitch_cmd));
            pitch = pitch_cmd;
            sinp = System.Math.Sin(pitch * System.Math.PI / 180);

            // direction in the vehicle's up frame: horizontal prograde, tilted by the pitch
            var vehicle = current_vessel.VesselVehicle;
            var up_coords = vehicle.Up.coordinateSystem;
            Vector3d up = vehicle.Up.vector.normalized;
            Vector3d vo = Vector.Reframed(vessel.OrbitalVelocity, up_coords).vector;
            Vector3d h = vo - up * Vector3d.Dot(vo, up);
            h = h.magnitude > 1e-3 ? h.normalized : Vector3d.zero;
            dir_world = (h * System.Math.Cos(pitch * System.Math.PI / 180) + up * sinp).normalized;
        }

        void Steer(Vector3d dir)
        {
            var autopilot = current_vessel.Autopilot;
            if (autopilot == null) return;
            autopilot.Enabled = true;
            autopilot.SAS.lockedMode = false;
            autopilot.SAS.SetTargetOrientation(new Vector(current_vessel.VesselVehicle.Up.coordinateSystem, dir), false);
        }

        double NoseAngle(Vector3d dir)
        {
            var up_coords = current_vessel.VesselVehicle.Up.coordinateSystem;
            Rotation rot = Rotation.Reframed(current_vessel.GetRotation(), up_coords);
            Vector3d nose = (rot.localRotation * Vector3.up).normalized;
            return Vector3d.Angle(nose, dir);
        }

        static double Thrust(VesselComponent vessel)
        {
            double t = 0;
            var engines = vessel.VesselDeltaV?.EngineInfo;
            if (engines != null)
                foreach (var e in engines)
                    if (e?.Engine != null && e.Engine.EngineIgnited) t += e.Engine.FinalThrustValue;
            return t;
        }
    }
}
