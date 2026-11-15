using System;
using System.Collections.Generic;
using System.Diagnostics;
using K2D2.Lift.Stage1;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Trains the behaviour model's weights in the calibrated launch-check simulator (spec
    /// 2026-11-14, step 3). Runs on the pad before every launch and from the Check launch button:
    /// a simple evolution strategy - mutate the best weights, keep a child if it scores higher,
    /// widen the search after a success and narrow it after a miss - over the five numbers that
    /// shape the trajectory. Failures in the simulator are free, so it may push to the edge; the
    /// real flights then correct the simulator (Calibrator) and the next launch retrains in it.
    ///
    /// Score (same units as the spec, km of periapsis):
    ///     Pe at stage-1 separation, capped at the debris cap (55 km)
    ///     - 0.5 per km of apoapsis miss
    ///     - 0.5 per kPa·° of peak Q·AoA over QAoARef (the simulator's stand-in for drift risk)
    ///     - 2 per kPa of peak q over max Q, - 1 per % of heating over the limit
    ///     - 1000 if it doesn't reach orbit
    ///     + 0.001 per m/s of upper-stage Δv to spare (tie-break once the cap is reached)
    /// </summary>
    public static class BehaviourTrainer
    {
        public const double QAoARef = 10, FailScore = -1000;
        const int Generations = 40, Children = 8;
        const double StartSigma = 0.15, MinSigma = 0.01;
        const long MaxMillis = 4000; // stop early rather than freeze the pad for long

        // the trained numbers, mapped to 0..1 (log scale for the scale-like ones)
        static readonly (string name, double lo, double hi, bool log)[] Params =
        {
            ("qScale", 1, 30, true),
            ("k", 0.5, 6, false), // was 0.5-3: training sat at 3.0 by the 4th flight of 2026-11-14
            ("Kp", 0.3, 6, true),
            ("vMid", 15, 200, false),
            ("maxAoA", 2, 20, false),
        };

        static double Get(in BehaviourWeights w, int i) => i switch { 0 => w.qScale, 1 => w.k, 2 => w.Kp, 3 => w.vMid, _ => w.maxAoA };
        static void Set(ref BehaviourWeights w, int i, double v)
        {
            switch (i) { case 0: w.qScale = v; break; case 1: w.k = v; break; case 2: w.Kp = v; break; case 3: w.vMid = v; break; default: w.maxAoA = v; break; }
        }
        static double ToUnit(int i, double v)
        {
            var p = Params[i];
            v = Math.Max(p.lo, Math.Min(p.hi, v));
            return p.log ? Math.Log(v / p.lo) / Math.Log(p.hi / p.lo) : (v - p.lo) / (p.hi - p.lo);
        }
        static double FromUnit(int i, double u)
        {
            var p = Params[i];
            u = Math.Max(0, Math.Min(1, u));
            return p.log ? p.lo * Math.Exp(u * Math.Log(p.hi / p.lo)) : p.lo + u * (p.hi - p.lo);
        }

        /// <summary>The simulator's score for one simulated flight.</summary>
        public static double Score(LaunchCheck.Result r, in PolicyContext ctx, double capKm)
        {
            if (r == null || r.verdict != LaunchCheck.Verdict.Ok) return FailScore;
            double pe_sep = r.s1ApBurnDv > 0 && !double.IsNaN(r.debrisPeKm) ? r.debrisPeKm : r.cutPeKm;
            if (double.IsInfinity(pe_sep)) pe_sep = capKm;
            double s = Math.Min(pe_sep, capKm);
            s -= 0.5 * Math.Abs(r.apKm - r.targetApKm);
            s -= 0.5 * Math.Max(0, r.maxQAoA - QAoARef);
            if (ctx.maxQKPa > 0) s -= 2 * Math.Max(0, r.maxQKPa - ctx.maxQKPa);
            s -= Math.Max(0, r.maxHeatPct - 100);
            // margin (2026-11-14): every trajectory reaching the cap scored the same, so training
            // drifted to ones that only just reached it in the sim - and the real stage 1 then fell
            // 17-26 m/s short (27 km of Pe). Reaching the cap with fuel to spare now scores more.
            if (pe_sep >= capKm - 0.01) s += MarginKmPerDv * Math.Min(MarginCapDv, r.s1UnusedDv);
            s += 0.001 * r.spareDv;
            return s;
        }

        public const double MarginKmPerDv = 0.05, MarginCapDv = 150;

        /// <summary>A real flight's score: measured Pe at separation, drift instead of Q·AoA.</summary>
        public static double RealScore(bool ok, double sepPeKm, double capKm, double driftPeakDeg, double maxQ, in PolicyContext ctx,
                                       double unusedDv = 0)
        {
            if (!ok || double.IsNaN(sepPeKm)) return FailScore;
            double s = Math.Min(sepPeKm, capKm);
            if (sepPeKm >= capKm - 0.5) s += MarginKmPerDv * Math.Min(MarginCapDv, Math.Max(0, unusedDv));
            s -= 2 * Math.Max(0, driftPeakDeg - 1.5);
            if (ctx.maxQKPa > 0) s -= 2 * Math.Max(0, maxQ - ctx.maxQKPa);
            return s;
        }

        public class Outcome
        {
            public BehaviourWeights weights;
            public double startScore, score;
            public int sims;
            public long millis;
            public LaunchCheck.Result best;
            public string Summary() => $"score {startScore:n1} -> {score:n1} in {sims} simulated flights ({millis} ms); " +
                                       (best == null ? "" : $"cut-off {best.altKm:n0} km at path {best.cutPathDeg:n0}°, Pe at separation {PeSep(best):n0} km " +
                                                            $"(stage 1 {best.s1UnusedDv:n0} m/s to spare), " +
                                                            $"max Q {best.maxQKPa:n1} kPa, Q·AoA {best.maxQAoA:n1}, spare {best.spareDv:n0} m/s");
        }

        public static double PeSep(LaunchCheck.Result r) => r.s1ApBurnDv > 0 && !double.IsNaN(r.debrisPeKm) ? r.debrisPeKm : r.cutPeKm;

        public static Outcome Train(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm,
                                    LaunchCheck.Calibration calibration, BehaviourWeights start, double capKm, ILogger log)
        {
            var clock = Stopwatch.StartNew();
            var o = new Outcome();
            int sims = 0;
            LaunchCheck.Result Fly(BehaviourWeights w)
            {
                var c = ctx;
                c.weights = w;
                sims++;
                return LaunchCheck.Simulate(model, c, headingDeg, targetApKm, calibration);
            }

            var best_w = start;
            var best_r = Fly(best_w);
            double best_s = Score(best_r, ctx, capKm);
            o.startScore = best_s;
            // the hand-picked defaults compete too, in case the stored weights went somewhere bad
            var d = BehaviourWeights.Default;
            var dr = Fly(d);
            double ds = Score(dr, ctx, capKm);
            if (ds > best_s) { best_w = d; best_r = dr; best_s = ds; }

            var rng = new Random(model?.phases?.Count > 0 ? (int)(model.phases[0].startMass * 1000) : 1);
            double sigma = StartSigma;
            for (int g = 0; g < Generations && sigma >= MinSigma && clock.ElapsedMilliseconds < MaxMillis; g++)
            {
                bool improved = false;
                var parent = best_w;
                for (int c = 0; c < Children; c++)
                {
                    var w = parent;
                    for (int i = 0; i < Params.Length; i++)
                        Set(ref w, i, FromUnit(i, ToUnit(i, Get(parent, i)) + sigma * Gauss(rng)));
                    var r = Fly(w);
                    double s = Score(r, ctx, capKm);
                    if (s > best_s + 1e-6) { best_s = s; best_w = w; best_r = r; improved = true; }
                }
                sigma *= improved ? 1.3 : 0.8;
            }

            o.weights = best_w;
            o.score = best_s;
            o.best = best_r;
            o.sims = sims;
            o.millis = clock.ElapsedMilliseconds;
            log?.LogInfo($"[Train] {o.Summary()}");
            log?.LogInfo($"[Train] weights: {best_w}");
            return o;
        }

        /// <summary>
        /// Before a launch (and the Check launch button): train from this design's stored weights
        /// (or the defaults), store the result and put it in ctx.
        /// </summary>
        public static Outcome TrainForLaunch(Stage1Model model, ref PolicyContext ctx, double headingDeg, double targetApKm,
                                             LaunchCheck.Calibration calibration, double capKm, string name, ILogger log)
        {
            if (model == null || !model.valid) return null;
            var start = CraftMemory.GetWeights(model) ?? BehaviourWeights.Default;
            var o = Train(model, ctx, headingDeg, targetApKm, calibration, start, capKm, log);
            ctx.weights = o.weights;
            CraftMemory.SaveTraining(model, name, o.weights, o.score);
            return o;
        }

        static double Gauss(Random rng)
        {
            double u1 = 1 - rng.NextDouble(), u2 = rng.NextDouble();
            return Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2);
        }
    }
}
