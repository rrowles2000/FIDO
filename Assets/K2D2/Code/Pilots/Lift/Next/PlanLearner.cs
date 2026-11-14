using System;
using System.Collections.Generic;
using System.Linq;
using K2D2.Lift.Stage1;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Chooses the ascent plan from what flights actually achieved, not only from what the
    /// simulator predicts (Bayesian optimisation, 2026-11-14).
    ///
    /// Why: six training flights of the 83 t design kept swapping between plans that looked best
    /// in the sim; the plan that really did best (12° kick, TWR 1.6 hold: 6.75 t in orbit, twice)
    /// was dropped for ones that did up to 230 m/s worse. The calibration alone can't fix that -
    /// each plan family kept the same error flight after flight.
    ///
    /// How: for every safe plan the launch check found (LaunchCheck.Result.eligible)
    ///     measured Δv left  ≈  sim spare Δv  +  b  +  GP(plan)
    /// b is the sim's average miss on this design, and GP is a Gaussian process over the plan
    /// settings (kick, kick speed, throttle hold, pull-down) fitted to each flown plan's miss.
    /// Near flown plans the estimate follows the measurements; far from them it falls back to the
    /// sim, with more uncertainty. Each past flight's sim prediction is re-run with today's
    /// calibration so its miss stays comparable.
    ///   - explore (training flights before the last): the plan with the highest expected
    ///     improvement over the best measured result - tries plans that could beat it;
    ///   - exploit (the last training flight, every normal launch): the highest estimate.
    /// The choice stays inside the launch check's safety rules: only eligible plans are considered.
    ///
    /// The measurement is Δv left in orbit, worked out from the vessel's mass after
    /// circularising and the upper stages' Isp - real, unlike the sim's "spare".
    /// </summary>
    public static class PlanLearner
    {
        /// <summary>One pad launch and what it achieved.</summary>
        public class Obs
        {
            public string when;
            public double kick, speed, holdTwr, holdKm, pull;
            public double targetApKm, headingDeg;
            public bool ok;
            public double dvLeft = double.NaN; // measured, m/s (NaN when the flight failed)
            public double massT;               // in orbit (or when it ended)
            public double predSpare;           // the sim's spare Δv for this plan when it was flown
            public string note = "";
        }

        // GP settings: how far the real result can stray from the sim (m/s), repeat-flight
        // noise (the 83 t repeats agreed within ~5 m/s), and how far a lesson carries in plan
        // settings (scaled so 1 = the whole search range)
        const double SignalSd = 80, NoiseSd = 15, Length = 0.25;
        // a flight that failed counts as this far below the sim's usual miss
        const double FailPenalty = 300;
        // expected-improvement margin, m/s
        const double Xi = 5;
        const double G0 = 9.80665;

        static double[] Features(double kick, double speed, double holdTwr, double holdKm, double pull) => new[]
        {
            kick / 22.0,
            speed / 90.0,
            holdTwr > 0 ? (3 - holdTwr) / 3 : 0,  // no hold = 0; a tighter hold (lower TWR) = bigger
            holdTwr > 0 ? holdKm / 20.0 : 0,
            pull / 12.0,
        };

        static double[] Features(LaunchCheck.Result r) => Features(r.kickDeg, r.kickSpeed, r.holdTwr, r.holdKm, r.pullDownDeg);
        static double[] Features(Obs o) => Features(o.kick, o.speed, o.holdTwr, o.holdKm, o.pull);

        static double Kernel(double[] a, double[] b)
        {
            double d2 = 0;
            for (int i = 0; i < a.Length; i++) { double d = (a[i] - b[i]) / Length; d2 += d * d; }
            return SignalSd * SignalSd * Math.Exp(-0.5 * d2);
        }

        static string PlanKey(double kick, double speed, double holdTwr, double holdKm, double pull) =>
            $"{kick:0.0}@{speed:0}" + (holdTwr > 0 ? $" TWR {holdTwr:0.0} to {holdKm:0} km" : "") + (pull > 0 ? $" pull {pull:0}°" : "");

        public static string PlanText(LaunchCheck.Result r) => PlanKey(r.kickDeg, r.kickSpeed, r.holdTwr, r.holdKm, r.pullDownDeg);
        public static string PlanText(Obs o) => PlanKey(o.kick, o.speed, o.holdTwr, o.holdKm, o.pull);

        static bool SameMission(Obs o, double targetApKm, double headingDeg) =>
            Math.Abs(o.targetApKm - targetApKm) <= 1 && Math.Abs(Math.IEEERemainder(o.headingDeg - headingDeg, 360)) <= 1;

        /// <summary>Δv left in the upper stages at this mass (vacuum), from the pad model's upper phases.</summary>
        public static double MeasuredDvLeft(Stage1Model model, double massT)
        {
            if (model?.upperPhases == null || model.upperPhases.Count == 0) return double.NaN;
            // still carrying stage 1 (or more than the upper stages' full mass): can't measure
            if (massT > model.upperPhases[0].startMass + 0.05) return double.NaN;
            double dv = 0;
            bool found = false;
            foreach (var p in model.upperPhases)
            {
                if (!found)
                {
                    if (massT <= p.endMass + 1e-3) continue; // this phase is used up
                    found = true;
                    double m = Math.Min(massT, p.startMass);
                    dv += p.ispVac * G0 * Math.Log(m / p.endMass);
                }
                else dv += p.deltaVVac;
            }
            return dv;
        }

        // ---------------------------------------------------------------- recording

        /// <summary>After a pad launch: store the plan and what it achieved. Stopped-by-hand flights aren't stored.</summary>
        public static void Record(Stage1Model model, string name, PolicyContext ctx, double targetApKm, double headingDeg,
                                  bool ok, string state, double massT, double predSpare, ILogger log)
        {
            if (model == null || !model.valid) return;
            if (!ok && state.Contains("stopped by hand")) return;
            var o = new Obs
            {
                when = DateTime.Now.ToString("yyyy-MM-dd HH:mm"),
                kick = ctx.kickDeg, speed = ctx.kickSpeed, holdTwr = ctx.holdTwr, holdKm = ctx.holdKm, pull = ctx.pullDownDeg,
                targetApKm = targetApKm, headingDeg = headingDeg, ok = ok, massT = massT, predSpare = predSpare, note = state,
            };
            if (ok)
            {
                o.dvLeft = MeasuredDvLeft(model, massT);
                if (double.IsNaN(o.dvLeft))
                {
                    log?.LogInfo($"[Learn] not recorded: can't measure Δv left at {massT:n2} t (stage 1 still on?)");
                    return;
                }
            }
            CraftMemory.AddPlanResult(model, name, o);
            log?.LogInfo($"[Learn] RECORD {PlanText(o)}: " + (ok ? $"{o.dvLeft:n0} m/s left in orbit ({massT:n2} t), sim said spare {predSpare:n0}" : $"FAILED ({state})") +
                         $" - {CraftMemory.PlanResults(model).Count} plan result(s) for this design");
        }

        // ---------------------------------------------------------------- choosing

        class Estimate
        {
            public LaunchCheck.Result plan;
            public double mean, sd, ei;
        }

        /// <summary>
        /// Picks among check.eligible. Returns check itself when it keeps the launch check's own
        /// choice, otherwise the chosen plan re-simulated with its trace (for the plan log and the
        /// predicted-vs-actual comparison).
        /// </summary>
        public static LaunchCheck.Result Choose(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm,
                                                LaunchCheck.Result check, LaunchCheck.Calibration calibration, bool explore, ILogger log)
        {
            if (check == null || check.verdict != LaunchCheck.Verdict.Ok) return check;
            var obs = CraftMemory.PlanResults(model).Where(o => SameMission(o, targetApKm, headingDeg)).ToList();
            var cands = check.eligible ?? new List<LaunchCheck.Result>();
            string mode = explore ? "explore" : "best";
            if (cands.Count < 2)
            {
                check.learnerNote = cands.Count == 0 ? "no safe plan with margin - nothing to choose" : "only one safe plan";
                log?.LogInfo($"[Learn] {mode}: {check.learnerNote}; {obs.Count} measured flight(s) of this mission");
                return check;
            }
            if (obs.Count(o => o.ok) == 0)
            {
                check.learnerNote = "no measured flights yet - flying the sim's best";
                log?.LogInfo($"[Learn] {mode}: {check.learnerNote}");
                return check;
            }

            // each flown plan's sim prediction with today's calibration (one sim per distinct plan)
            var pred = new Dictionary<string, double>();
            double Pred(Obs o)
            {
                string k = PlanText(o);
                if (pred.TryGetValue(k, out double v)) return v;
                var c = ctx;
                c.kickDeg = o.kick; c.kickSpeed = o.speed; c.holdTwr = o.holdTwr; c.holdKm = o.holdKm; c.pullDownDeg = o.pull;
                v = LaunchCheck.Simulate(model, c, headingDeg, targetApKm, calibration).spareDv;
                pred[k] = v;
                return v;
            }

            var ok_obs = obs.Where(o => o.ok).ToList();
            double b = ok_obs.Average(o => o.dvLeft - Pred(o));
            int n = obs.Count;
            var X = obs.Select(Features).ToArray();
            var y = obs.Select(o => o.ok ? o.dvLeft - Pred(o) - b : -FailPenalty).ToArray();

            // K + noise, Cholesky, alpha = K^-1 y
            var K = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    K[i, j] = Kernel(X[i], X[j]) + (i == j ? NoiseSd * NoiseSd : 0);
            var L = Cholesky(K, n);
            var alpha = SolveUpper(L, SolveLower(L, y, n), n);

            double best_measured = ok_obs.Max(o => o.dvLeft);
            var est = new List<Estimate>();
            foreach (var r in cands)
            {
                var x = Features(r);
                var ks = new double[n];
                for (int i = 0; i < n; i++) ks[i] = Kernel(x, X[i]);
                double mean = r.spareDv + b;
                for (int i = 0; i < n; i++) mean += ks[i] * alpha[i];
                var v = SolveLower(L, ks, n);
                double var_ = SignalSd * SignalSd;
                for (int i = 0; i < n; i++) var_ -= v[i] * v[i];
                double sd = Math.Sqrt(Math.Max(1e-6, var_));
                double imp = mean - best_measured - Xi, z = imp / sd;
                est.Add(new Estimate { plan = r, mean = mean, sd = sd, ei = imp * Phi(z) + sd * PhiDensity(z) });
            }

            var chosen = explore ? est.OrderByDescending(e => e.ei).First() : est.OrderByDescending(e => e.mean).First();
            var top_measured = ok_obs.OrderByDescending(o => o.dvLeft).First();
            log?.LogInfo($"[Learn] {mode}: {ok_obs.Count} measured flight(s) (+{n - ok_obs.Count} failed); best measured {best_measured:n0} m/s " +
                         $"({PlanText(top_measured)}); sim misses by {b:+0;-0} m/s on average; {cands.Count} safe plan(s)");
            foreach (var e in (explore ? est.OrderByDescending(e => e.ei) : est.OrderByDescending(e => e.mean)).Take(5))
                log?.LogInfo($"[Learn]   {PlanText(e.plan),-36} sim {e.plan.spareDv,6:n0}  estimate {e.mean,6:n0} ± {e.sd,3:n0}  " +
                             $"expected gain {e.ei,5:n1}{(e == chosen ? "  <- chosen" : "")}");

            string note = explore
                ? $"explore: {PlanText(chosen.plan)} - estimate {chosen.mean:n0} ± {chosen.sd:n0} m/s vs best measured {best_measured:n0}"
                : $"best: {PlanText(chosen.plan)} - estimate {chosen.mean:n0} ± {chosen.sd:n0} m/s (best measured {best_measured:n0})";

            if (PlanText(chosen.plan) == PlanText(check))
            {
                check.learnerNote = note;
                return check;
            }
            var pc = ctx;
            pc.kickDeg = chosen.plan.kickDeg; pc.kickSpeed = chosen.plan.kickSpeed;
            pc.holdTwr = chosen.plan.holdTwr; pc.holdKm = chosen.plan.holdKm; pc.pullDownDeg = chosen.plan.pullDownDeg;
            var res = LaunchCheck.Simulate(model, pc, headingDeg, targetApKm, calibration);
            res.trials = check.trials;
            res.eligible = check.eligible;
            res.kicksTried = check.kicksTried;
            res.authorityShort = check.authorityShort;
            res.learnerNote = note;
            return res;
        }

        /// <summary>For the Training tab: best measured result of this design for this mission.</summary>
        public static string Describe(Stage1Model model, double targetApKm, double headingDeg)
        {
            var obs = CraftMemory.PlanResults(model).Where(o => SameMission(o, targetApKm, headingDeg)).ToList();
            var ok = obs.Where(o => o.ok).ToList();
            if (ok.Count == 0) return obs.Count == 0 ? "no measured flights for this mission yet" : $"{obs.Count} flight(s), none reached orbit";
            var best = ok.OrderByDescending(o => o.dvLeft).First();
            var first = ok[0];
            return $"best {best.dvLeft:n0} m/s left in orbit ({PlanText(best)}), first flight {first.dvLeft:n0} - " +
                   $"{ok.Count} measured flight(s)" + (obs.Count > ok.Count ? $", {obs.Count - ok.Count} failed" : "");
        }

        // ---------------------------------------------------------------- maths

        static double[,] Cholesky(double[,] A, int n)
        {
            var L = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j <= i; j++)
                {
                    double s = A[i, j];
                    for (int k = 0; k < j; k++) s -= L[i, k] * L[j, k];
                    L[i, j] = i == j ? Math.Sqrt(Math.Max(s, 1e-9)) : s / L[j, j];
                }
            return L;
        }

        static double[] SolveLower(double[,] L, double[] b, int n)
        {
            var x = new double[n];
            for (int i = 0; i < n; i++)
            {
                double s = b[i];
                for (int k = 0; k < i; k++) s -= L[i, k] * x[k];
                x[i] = s / L[i, i];
            }
            return x;
        }

        static double[] SolveUpper(double[,] L, double[] b, int n) // solves L^T x = b
        {
            var x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double s = b[i];
                for (int k = i + 1; k < n; k++) s -= L[k, i] * x[k];
                x[i] = s / L[i, i];
            }
            return x;
        }

        static double PhiDensity(double z) => Math.Exp(-0.5 * z * z) / Math.Sqrt(2 * Math.PI);

        static double Phi(double z) => 0.5 * (1 + Erf(z / Math.Sqrt(2)));

        static double Erf(double x) // Abramowitz & Stegun 7.1.26
        {
            double t = 1 / (1 + 0.3275911 * Math.Abs(x));
            double y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.Exp(-x * x);
            return x >= 0 ? y : -y;
        }
    }
}
