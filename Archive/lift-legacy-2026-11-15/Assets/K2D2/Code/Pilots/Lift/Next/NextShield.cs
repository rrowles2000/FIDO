using System;
using K2D2.Lift.Stage1;
using UnityEngine;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Layer L1 of the New lift system (NN ascent report, Section 3.1): every command passes
    /// through here before it reaches the vessel, whichever policy or behaviour produced it, in
    /// flight and in the LaunchCheck simulation alike.
    ///
    ///   1. NaN guard           keep the last good command
    ///   2. hard ranges         elevation -10..90°, throttle 0..max
    ///   3. AoA limit           nose within Q·AoA / q of the airflow (setting, default 4 kPa·deg)
    ///   4. pitch smoother      the Optimiser's rate / acceleration limits
    ///   5. max-Q throttle-back against the max-Q setting (default 30 kPa), suspended while the
    ///                          nose is off its command - with hysteresis, see below
    ///   6. loss of control     reported for the arbiter
    /// plus LimitHeading for the heading correction, which used to bypass all of this.
    ///
    /// 2026-11-13 run 1 (83 t, lost control at 4.9 km, q 13.8 kPa): the throttle-back switched on
    /// and off every second (thrust 1,141 -> 994 -> 1,151 -> 762 -> 1,160 kN in 6 s) because its
    /// suspend rule had no hysteresis, and the heading correction swung -8° -> +14° -> +44° in 2 s
    /// (ground-track heading is noise near vertical) with nothing limiting it.
    ///
    /// Runs on the control tick only (dt = tick length), so smoothing doesn't depend on frame rate.
    /// </summary>
    public class SafetyShield
    {
        public const double LossOfControlDeg = 30;
        public const double MinElevationDeg = -10, MaxElevationDeg = 90;
        public const double MinAoADeg = 0.5;

        // throttle-back suspension: start above SuspendDeg of attitude error, end only after
        // ResumeCalmS below ResumeDeg
        public const double SuspendDeg = 1.5, ResumeDeg = 0.75, ResumeCalmS = 2.0;

        // heading correction: none below this ground speed (heading is noise near vertical),
        // and at most these rates
        public const double HeadingMinGroundSpeed = 50;
        public const double HeadingRateDegS = 2.0, HeadingRateHighQDegS = 1.0;

        readonly Stage1Guidance.PitchSmoother smoother = new();
        float bucket = 1f;
        bool bucket_suspended;
        double calm_s;
        AscentCommand last_good = new AscentCommand(90, 0);
        double clamped_seconds;
        double heading_correction;

        public ShieldReport report;

        // ---- max-Q hold (step 5). Until 2026-11-13 this used the Optimiser's bucket curve: full
        // thrust to 85% of the limit, 40% at the limit. That is a 4x gain - 1.5 kPa of q swung the
        // throttle 20% - so the 47 t test vessel cycled 0.72 -> 0.53 -> 0.78 through Mach 1.4 as
        // thrust and q chased each other. Now the throttle is steered to hold q at QHoldFraction
        // of the limit, looking QLeadS ahead on the q trend, and moving at most QDownRate / QUpRate
        // per second (down faster than up, so it can't overshoot back into the limit).
        // 0.87, not 0.95 (2026-11-14): the launch check now wants a plan's peak q under 90% of max Q
        // (LaunchCheck.LimitMargin), so the hold sits just below that
        public const double QHoldFraction = 0.84, QLeadS = 2.0, QGain = 2.0; // 0.84: the sim overshoots the hold by ~5%
        public const float QDownRate = 0.35f, QUpRate = 0.15f, QMinThrottle = 0.4f;
        double q_prev = -1, dq_filtered;

        // the learned transonic cap starts this much Mach early, so q is already down at the band
        public const double CapLeadMach = 0.15;

        /// <summary>The q to hold now: 95% of max Q, or the learned cap inside its Mach band.</summary>
        public static double QTarget(in AscentObservation obs, in PolicyContext ctx, out bool capped)
        {
            double target = (ctx.qHoldFraction > 0 ? ctx.qHoldFraction : QHoldFraction) * ctx.maxQKPa;
            capped = ctx.qCapKPa > 0 && obs.mach >= ctx.qCapMachLo - CapLeadMach && obs.mach <= ctx.qCapMachHi &&
                     ctx.qCapKPa < target;
            return capped ? ctx.qCapKPa : target;
        }

        /// <summary>
        /// Drift protection (AscentNext sets it while the nose is leaving the airflow): full thrust
        /// for gimbal authority - cutting thrust loses authority at once while q only falls slowly
        /// - unless q is over the hard max Q, and the heading correction frozen (on 2026-11-13 it
        /// was steering sideways while the vessel flipped).
        /// </summary>
        public bool protect;

        float QHoldStep(float current, double qKPa, double dt, double targetKPa)
        {
            if (q_prev >= 0 && dt > 0)
                dq_filtered += 0.3 * ((qKPa - q_prev) / dt - dq_filtered);
            q_prev = qKPa;
            double target = targetKPa;
            double predicted = qKPa + Math.Max(0, dq_filtered) * QLeadS; // only anticipate a rise
            double error = (target - predicted) / target;                // > 0: room to spare
            double rate = Math.Max(-QDownRate, Math.Min(QUpRate, QGain * error));
            return Mathf.Clamp((float)(current + rate * dt), QMinThrottle, 1f);
        }

        public void Reset()
        {
            smoother.Reset();
            bucket = 1f;
            bucket_suspended = false;
            calm_s = 0;
            last_good = new AscentCommand(90, 0);
            clamped_seconds = 0;
            heading_correction = 0;
            q_prev = -1;
            dq_filtered = 0;
            report = default;
            report.bucket = 1f;
        }

        /// <summary>AoA limit (deg) at dynamic pressure q for a Q·AoA limit in kPa·deg.</summary>
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

            // 3. nose near the airflow while the air is thick (not for the behaviour model: its
            //    "maintain trajectory" behaviour does this as a weighted vote, not a hard limit)
            if (obs.hasPath && !ctx.blend)
            {
                double max = MaxAoADeg(obs.qKPa, ctx.qAlphaKPaDeg);
                double limited = Math.Max(obs.pathDeg - max, Math.Min(obs.pathDeg + max, cmd.elevationDeg));
                rep.aoaLimited = Math.Abs(limited - cmd.elevationDeg) > 0.01;
                clamped_seconds = Math.Abs(limited - cmd.elevationDeg) > 2 ? clamped_seconds + dt : 0;
                cmd.elevationDeg = limited;
            }
            else
                clamped_seconds = 0;
            rep.clampedSeconds = clamped_seconds;

            // 4. pitch rate / acceleration
            double smoothed = smoother.Next(cmd.elevationDeg, dt, obs.qKPa);
            rep.rateLimited = Math.Abs(smoothed - cmd.elevationDeg) > 0.01;
            cmd.elevationDeg = smoothed;

            // 5. max-Q throttle-back, suspended (full thrust, for gimbal authority) while the nose is
            //    off its command - entered above SuspendDeg, left only after ResumeCalmS below ResumeDeg
            double attitude_error = Math.Abs(obs.noseDeg - cmd.elevationDeg);
            if (!bucket_suspended && attitude_error > SuspendDeg && obs.qKPa > 1)
            {
                bucket_suspended = true;
                calm_s = 0;
            }
            else if (bucket_suspended)
            {
                calm_s = attitude_error < ResumeDeg ? calm_s + dt : 0;
                if (calm_s >= ResumeCalmS || obs.qKPa <= 1) bucket_suspended = false;
            }
            double q_target = QTarget(obs, ctx, out rep.qCapped);
            if (ctx.blend)
                bucket = 1f; // behaviour model: no max-Q throttle-back (never reduce speed)
            else if (protect && obs.qKPa < ctx.maxQKPa)
                bucket = Mathf.MoveTowards(bucket, 1f, (float)(2.0 * dt));
            else if (bucket_suspended && !rep.qCapped)
                bucket = Mathf.MoveTowards(bucket, 1f, (float)(2.0 * dt));
            else
                bucket = QHoldStep(bucket, obs.qKPa, dt, q_target);
            rep.protect = protect;
            rep.bucketSuspended = bucket_suspended;
            rep.bucket = bucket;
            cmd.throttle *= bucket;

            // 6. loss of control: reported, not acted on - the arbiter decides
            rep.lossOfControl = obs.hasPath && obs.qKPa > 1 && Math.Abs(obs.noseDeg - obs.airPathDeg) > LossOfControlDeg;

            if (!rep.rejected) last_good = cmd;
            rep.headingLimited = report.headingLimited; // LimitHeading's flag, kept for this tick's report
            report = rep;
            return cmd;
        }

        /// <summary>
        /// The heading correction (steering the ground track back onto the launch heading) through
        /// the same limits as pitch: off until there's real ground speed, rate-limited, and never
        /// pointing the nose further sideways off the airflow than the AoA limit allows. A heading
        /// change of h at elevation e moves the nose about h·cos(e) sideways.
        /// </summary>
        public float LimitHeading(double rawCorrectionDeg, in AscentObservation obs, double elevationDeg,
                                  in PolicyContext ctx, double dt)
        {
            double target = Math.Max(-45, Math.Min(45, rawCorrectionDeg));
            if (obs.horizontalSpeed < HeadingMinGroundSpeed) target = 0;
            if (protect) target = heading_correction; // frozen while the nose is drifting
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
