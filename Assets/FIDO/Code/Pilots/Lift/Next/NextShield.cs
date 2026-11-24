using System;
using Fido.Lift.Stage1;
using UnityEngine;

namespace Fido.Lift.Next
{
    /// <summary>
    /// Layer L1 of the New lift system (NN ascent report, Section 3.1): every command passes
    /// through here before it reaches the vessel, in flight and in the LaunchCheck simulation alike.
    ///
    ///   1. NaN guard           keep the last good command
    ///   2. hard ranges         elevation -10..90°, throttle 0..max
    ///   3. pitch smoother      rate / acceleration limits
    ///   4. loss of control     reported for the arbiter
    /// plus LimitHeading for the heading correction.
    ///
    /// The AoA limit (nose within Q·AoA / q of the airflow), the max-Q throttle-back and drift
    /// protection belonged to the pre-behaviour-model path and were removed 2026-11-15
    /// (Archive/lift-legacy-2026-11-15/): the behaviour model keeps the nose near the airflow as a
    /// weighted vote and never throttles back for q.
    ///
    /// Runs on the control tick only (dt = tick length), so smoothing doesn't depend on frame rate.
    /// </summary>
    public class SafetyShield
    {
        public const double LossOfControlDeg = 30;
        public const double MinElevationDeg = -10, MaxElevationDeg = 90;
        public const double MinAoADeg = 0.5;

        // heading correction: none below this ground speed (heading is noise near vertical),
        // and at most these rates
        public const double HeadingMinGroundSpeed = 50;
        public const double HeadingRateDegS = 2.0, HeadingRateHighQDegS = 1.0;

        readonly Stage1Guidance.PitchSmoother smoother = new();
        AscentCommand last_good = new AscentCommand(90, 0);
        double heading_correction;

        public ShieldReport report;

        public void Reset()
        {
            smoother.Reset();
            last_good = new AscentCommand(90, 0);
            heading_correction = 0;
            report = default;
        }

        /// <summary>Sideways AoA allowed (deg) at dynamic pressure q for a Q·AoA limit in kPa·deg.</summary>
        public static double MaxAoADeg(double qKPa, double qAlphaKPaDeg)
        {
            if (qKPa * 90 <= qAlphaKPaDeg) return 90;
            return Math.Max(MinAoADeg, Math.Min(90, qAlphaKPaDeg / qKPa));
        }

        public AscentCommand Apply(AscentCommand cmd, in AscentObservation obs, in PolicyContext ctx, double dt)
        {
            var rep = new ShieldReport();

            // 1. NaN / infinity: keep the last good command rather than steer on garbage
            if (!cmd.Finite)
            {
                rep.rejected = true;
                cmd = last_good;
            }

            // 2. hard ranges
            cmd.elevationDeg = Math.Max(MinElevationDeg, Math.Min(MaxElevationDeg, cmd.elevationDeg));
            cmd.throttle = Mathf.Clamp(cmd.throttle, 0f, Mathf.Clamp01(ctx.maxThrottle));

            // 3. pitch rate / acceleration
            double smoothed = smoother.Next(cmd.elevationDeg, dt, obs.qKPa);
            rep.rateLimited = Math.Abs(smoothed - cmd.elevationDeg) > 0.01;
            cmd.elevationDeg = smoothed;

            // 4. loss of control: reported, not acted on - the arbiter decides
            rep.lossOfControl = obs.hasPath && obs.qKPa > 1 && Math.Abs(obs.noseDeg - obs.airPathDeg) > LossOfControlDeg;

            if (!rep.rejected) last_good = cmd;
            rep.headingLimited = report.headingLimited; // LimitHeading's flag, kept for this tick's report
            report = rep;
            return cmd;
        }

        /// <summary>
        /// The heading correction (steering the ground track back onto the launch heading): off
        /// until there's real ground speed, rate-limited, and never pointing the nose further
        /// sideways off the airflow than Q·AoA / q allows. A heading change of h at elevation e moves
        /// the nose about h·cos(e) sideways.
        /// </summary>
        public float LimitHeading(double rawCorrectionDeg, in AscentObservation obs, double elevationDeg,
                                  in PolicyContext ctx, double dt)
        {
            double target = Math.Max(-45, Math.Min(45, rawCorrectionDeg));
            if (obs.horizontalSpeed < HeadingMinGroundSpeed) target = 0;
            if (obs.hasPath)
            {
                double cos_e = Math.Max(0.05, Math.Cos(elevationDeg * Math.PI / 180));
                double max_heading = MaxAoADeg(obs.qKPa, ctx.qAlphaKPaDeg) / cos_e;
                target = Math.Max(-max_heading, Math.Min(max_heading, target));
            }
            double rate = obs.qKPa > Stage1Guidance.HighQKPa ? HeadingRateHighQDegS : HeadingRateDegS;
            double step = Math.Max(-rate * dt, Math.Min(rate * dt, target - heading_correction));
            heading_correction += step;
            report.headingLimited = Math.Abs(heading_correction - Math.Max(-45, Math.Min(45, rawCorrectionDeg))) > 0.5;
            return (float)heading_correction;
        }
    }
}
