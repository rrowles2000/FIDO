using System;
using K2D2.Lift.Stage1;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// The trainable numbers of the behaviour model - each behaviour's weight is a smooth curve of
    /// one flight input. Hand-picked defaults for now (2026-11-14, from a point-mass sweep of the
    /// 83 t vessel); training in the simulator tunes them (spec: notes/20261114-224500_spec-change_behaviour-model-ascent.md).
    /// </summary>
    public struct BehaviourWeights
    {
        // 1 never reduce speed: on when along-path acceleration falls under aMin (m/s²)
        public double aMin, speedGain;       // speedGain: degrees of pitch-down per m/s² short
        // 2 maintain trajectory and course: weight 0.2 + (q / qScale)²
        public double qScale;                // kPa
        // 3 reach Ap with the most equal Pe: path target 90° x (1 - v / v_orbit)^k, steered with
        //   gain Kp (deg of AoA per deg of path error, at most maxAoA); weight rises past vMid
        public double k, Kp, maxAoA, vMid, vSteep;

        public static BehaviourWeights Default => new BehaviourWeights
        {
            aMin = 2, speedGain = 3,
            qScale = 3,
            k = 1.4, Kp = 2, maxAoA = 10, vMid = 30, vSteep = 10,
        };

        public override string ToString() =>
            $"speed aMin {aMin:n1} m/s² gain {speedGain:n1}; course qScale {qScale:n1} kPa; goal k {k:n2} Kp {Kp:n1} maxAoA {maxAoA:n0}° from {vMid:n0}±{vSteep:n0} m/s";
    }

    /// <summary>
    /// The behaviour model (spec 2026-11-14): three behaviours run every tick, each proposes a
    /// pitch and a throttle, and the output is their weighted blend. No kick, no plan grid, no
    /// throttle holds: the gravity turn comes out of the blend as the goal behaviour's weight rises
    /// at low speed and the course behaviour's weight rises with q.
    ///   1 Never reduce speed           (go forward)       pitch below the path if acceleration drops; full throttle
    ///   2 Maintain trajectory & course (avoid obstacles)  nose on the flight path; full throttle (thrust = steering)
    ///   3 Reach Ap, most equal Pe      (goal)             bend the path toward the target path; throttle eases near the target Ap
    /// Pure like every policy: the same code flies the simulator and the vessel.
    /// </summary>
    public class BehaviourPolicy : IAscentPolicy
    {
        public string Name => "Behaviours";

        // the last tick's weights and goal path, for the log only
        public double lastW1, lastW2, lastW3, lastGoalPath;

        static double Sig(double x) => 1 / (1 + Math.Exp(-Math.Max(-40, Math.Min(40, x))));
        static double Clamp(double x, double lo, double hi) => Math.Max(lo, Math.Min(hi, x));

        public AscentCommand Decide(in AscentObservation obs, in BodyDescriptor body, in PolicyContext ctx)
        {
            var w = ctx.weights;
            double path = obs.hasPath ? obs.pathDeg : 90;
            float full = ctx.maxThrottle;

            // 1 never reduce speed
            double deficit = w.aMin - obs.speedRate;
            double e1 = path - Clamp(deficit * w.speedGain, 0, 10);
            double w1 = obs.hasPath ? Sig(deficit) : 0;

            // 2 maintain trajectory and course
            double e2 = path;
            double w2 = 0.2 + (obs.qKPa / w.qScale) * (obs.qKPa / w.qScale);

            // 3 reach Ap with the most equal Pe
            double v_frac = body.orbitSpeedAtTarget > 0 ? Clamp(obs.airSpeed / body.orbitSpeedAtTarget, 0, 1) : 0;
            double goal_path = 90 * Math.Pow(1 - v_frac, w.k);
            double e3 = path + Clamp(w.Kp * (goal_path - path), -w.maxAoA, w.maxAoA);
            double w3 = obs.hasPath ? Sig((obs.airSpeed - w.vMid) / Math.Max(1, w.vSteep)) : 0;
            float t3 = full * Stage1Guidance.ApThrottle(obs.apR, body.targetApR);

            double sum = w1 + w2 + w3;
            double elev = (w1 * e1 + w2 * e2 + w3 * e3) / sum;
            float throttle = (float)((w1 * full + w2 * full + w3 * t3) / sum);

            lastW1 = w1; lastW2 = w2; lastW3 = w3; lastGoalPath = goal_path;
            return new AscentCommand(elev, throttle);
        }

        public string WeightsText() => $"w speed {lastW1:n2} course {lastW2:n2} goal {lastW3:n2} (goal path {lastGoalPath:n1}°)";
    }
}
