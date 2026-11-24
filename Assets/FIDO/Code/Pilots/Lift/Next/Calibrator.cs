using System;
using System.Collections.Generic;
using Fido.Lift.Stage1;

namespace Fido.Lift.Next
{
    /// <summary>
    /// Automatic calibration of the launch-check model from flights (2026-11-14).
    ///
    /// After each flight that reached cut-off, the flown plan (same kick, kick speed, hold) is
    /// re-simulated and three model factors are adjusted so the simulation lands where the flight
    /// did at cut-off - time, speed and flight-path angle:
    ///
    ///   dragScale    drag multiplier                       step x1.15
    ///   thrustScale  thrust at the same propellant flow    step +0.02
    ///   aoaBiasDeg   nose held above its command (flying)  step +0.25°
    ///   aoaGain      real AoA / commanded AoA              step x1.15 (added 2026-11-14)
    ///
    /// Residuals: the flight's speed, flight path and altitude every LaunchCheck.PlanStepS up to
    /// cut-off against the re-simulated plan at the same times (weights 40 m/s, 1.5°, 1 km) - the
    /// whole climb, not just cut-off, so drag (speed) and nose bias (path) separate, and it still
    /// works when plan and flight end differently.
    ///
    /// One Gauss-Newton step: the sensitivity of (T, v, path) to one step of each factor comes
    /// from re-running the sim, residuals are weighted (5 s, 50 m/s, 2°), the solve is damped
    /// (Levenberg-Marquardt, lambda 0.3), each factor moves at most 2 steps per flight and 70% of
    /// the solved step is taken, inside bounds. Each flight refines the design's factors; new
    /// designs start from the average of every calibrated design (CraftMemory).
    ///
    /// Why: every low-TWR flight so far cut off 6-10° steeper, ~7 s earlier and 150-200 m/s
    /// slower than predicted, and the across-plane rotation velocity (the first suspect) turned
    /// out to be worth only ~10 m/s.
    /// </summary>
    public static class Calibrator
    {
        const double DragStep = 1.15, ThrustStep = 0.02, BiasStep = 0.25, GainStep = 1.15;
        const int N = 5; // drag, thrust, nose bias, AoA gain, gimbal trim
        const double TrimStep = 0.25;
        const double Lambda = 0.3, MaxSteps = 2, Damping = 0.7;
        static readonly double[] Weight = { 1 / 40.0, 1 / 1.5, 1 / 1.0 }; // v, path, alt (km)

        public struct Sample { public double t, v, path, altKm; }

        public class Fit
        {
            public bool ok;
            public string why = "";
            public LaunchCheck.Calibration before, after;
            public double[] actual = new double[3], predBefore = new double[3], predAfter = new double[3];
            public double rmsBefore, rmsAfter; // weighted trajectory mismatch (1 = 40 m/s / 1.5° / 1 km)

            public string Summary() => !ok ? "calibration skipped: " + why :
                $"trajectory mismatch rms {rmsBefore:n2} -> {rmsAfter:n2}; cut-off T/v/path actual {actual[0]:n0} s / {actual[1]:n0} m/s / {actual[2]:n1}° - predicted with " +
                $"{before} {predBefore[0]:n0} / {predBefore[1]:n0} / {predBefore[2]:n1}, with the new {after} " +
                $"{predAfter[0]:n0} / {predAfter[1]:n0} / {predAfter[2]:n1}";
        }

        static double[] Observe(LaunchCheck.Result r) => new[] { r.cutT, r.cutSpeed, r.cutPathDeg };

        static LaunchCheck.Calibration Apply(LaunchCheck.Calibration c, double[] d)
        {
            var n = c.Clone();
            n.dragScale = Clamp(c.dragScale * Math.Pow(DragStep, d[0]), 0.5, 2.5);
            n.thrustScale = Clamp(c.thrustScale + ThrustStep * d[1], 0.90, 1.08);
            n.aoaBiasDeg = Clamp(c.aoaBiasDeg + BiasStep * d[2], -3, 3);
            n.aoaGain = Clamp(c.aoaGain * Math.Pow(GainStep, d[3]), 0.4, 3.0);
            n.trimDeg = Clamp(c.trimDeg + TrimStep * d[4], -3, 5);
            return n;
        }

        static double[] Unit(int k) { var d = new double[N]; d[k] = 1; return d; }

        static double Clamp(double v, double lo, double hi) => Math.Max(lo, Math.Min(hi, v));

        // residual vector over the flight samples (weighted), against a simulated trace
        static double[] Residuals(List<Sample> flight, List<LaunchCheck.TracePoint> plan, double tEnd)
        {
            var res = new List<double>();
            foreach (var fs in flight)
            {
                if (fs.t > tEnd || fs.t < 5) continue;
                if (!Interp(plan, fs.t, out double v, out double path, out double alt)) continue;
                res.Add((fs.v - v) * Weight[0]);
                res.Add((fs.path - path) * Weight[1]);
                res.Add((fs.altKm - alt) * Weight[2]);
            }
            return res.ToArray();
        }

        static bool Interp(List<LaunchCheck.TracePoint> p, double t, out double v, out double path, out double alt)
        {
            v = path = alt = 0;
            if (p == null || p.Count < 2 || t < p[0].t || t > p[p.Count - 1].t) return false;
            for (int i = 1; i < p.Count; i++)
                if (t <= p[i].t)
                {
                    double k = p[i].t > p[i - 1].t ? (t - p[i - 1].t) / (p[i].t - p[i - 1].t) : 0;
                    v = p[i - 1].v + (p[i].v - p[i - 1].v) * k;
                    path = p[i - 1].path + (p[i].path - p[i - 1].path) * k;
                    alt = p[i - 1].altKm + (p[i].altKm - p[i - 1].altKm) * k;
                    return true;
                }
            return false;
        }

        static double Rms(double[] r) { double s = 0; foreach (var x in r) s += x * x; return r.Length > 0 ? Math.Sqrt(s / r.Length) : 0; }

        /// <param name="flown">the context the flight used (kick, kick speed, hold as chosen)</param>
        /// <param name="flight">the flight's own samples every PlanStepS up to cut-off</param>
        public static Fit Run(Stage1Model model, PolicyContext flown, double headingDeg, double targetApKm,
                              LaunchCheck.Calibration current, List<Sample> flight, double tCut,
                              double actT, double actV, double actPath)
        {
            var f = new Fit { before = current.Clone(), actual = new[] { actT, actV, actPath } };
            try
            {
                if (model == null || !model.valid) { f.why = "no vessel model"; return f; }
                if (flight == null || flight.Count < 4) { f.why = "too few flight samples"; return f; }
                var b = LaunchCheck.Simulate(model, flown, headingDeg, targetApKm, current);
                var sims = new LaunchCheck.Result[N];
                for (int k = 0; k < N; k++) sims[k] = LaunchCheck.Simulate(model, flown, headingDeg, targetApKm, Apply(current, Unit(k)));
                // compare over the span every run covers (they cut off a few seconds apart)
                double tEnd = tCut;
                foreach (var x in new List<LaunchCheck.Result>(sims) { b })
                    if (x.plan != null && x.plan.Count > 0) tEnd = Math.Min(tEnd, x.plan[x.plan.Count - 1].t);
                var r0 = Residuals(flight, b.plan, tEnd);
                if (r0.Length < 9) { f.why = "plan and flight overlap too little"; return f; }
                f.predBefore = new[] { b.cutT, b.cutSpeed, b.cutPathDeg };
                f.rmsBefore = Rms(r0);

                var cols = new double[N][];
                for (int k = 0; k < N; k++)
                {
                    var res_k = Residuals(flight, sims[k].plan, tEnd);
                    if (res_k.Length != r0.Length) { f.why = $"sensitivity run {k} covers a different span"; return f; }
                    cols[k] = new double[r0.Length];
                    // residual = flight - sim, so the sim's sensitivity is minus the residual change
                    for (int i = 0; i < r0.Length; i++) cols[k][i] = r0[i] - res_k[i];
                }

                var A = new double[N, N];
                var g = new double[N];
                for (int a = 0; a < N; a++)
                {
                    for (int c = 0; c < N; c++)
                    {
                        double sum = 0;
                        for (int i = 0; i < r0.Length; i++) sum += cols[a][i] * cols[c][i];
                        A[a, c] = sum + (a == c ? Lambda * r0.Length / 9.0 : 0);
                    }
                    double gs = 0;
                    for (int i = 0; i < r0.Length; i++) gs += cols[a][i] * r0[i];
                    g[a] = gs;
                }
                var d = SolveN(A, g);
                if (d == null) { f.why = "singular fit"; return f; }
                for (int k = 0; k < N; k++) d[k] = Damping * Clamp(d[k], -MaxSteps, MaxSteps);

                f.after = Apply(current, d);
                f.after.source = "fitted";
                var a2 = LaunchCheck.Simulate(model, flown, headingDeg, targetApKm, f.after);
                var r2 = Residuals(flight, a2.plan, tEnd);
                f.predAfter = new[] { a2.cutT, a2.cutSpeed, a2.cutPathDeg };
                f.rmsAfter = r2.Length == r0.Length ? Rms(r2) : double.PositiveInfinity;
                if (f.rmsAfter >= f.rmsBefore)
                {
                    f.why = $"the fitted factors didn't improve the match (rms {f.rmsBefore:n2} -> {f.rmsAfter:n2})";
                    return f;
                }
                f.ok = true;
                return f;
            }
            catch (Exception ex)
            {
                f.why = "failed: " + ex.Message;
                return f;
            }
        }

        static double[] SolveN(double[,] A, double[] b)
        {
            int n = b.Length;
            var m = new double[n, n + 1];
            for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) m[i, j] = A[i, j]; m[i, n] = b[i]; }
            for (int c = 0; c < n; c++)
            {
                int p = c;
                for (int r = c + 1; r < n; r++) if (Math.Abs(m[r, c]) > Math.Abs(m[p, c])) p = r;
                if (Math.Abs(m[p, c]) < 1e-12) return null;
                if (p != c) for (int j = 0; j <= n; j++) { var t = m[c, j]; m[c, j] = m[p, j]; m[p, j] = t; }
                for (int r = 0; r < n; r++)
                {
                    if (r == c) continue;
                    double k = m[r, c] / m[c, c];
                    for (int j = c; j <= n; j++) m[r, j] -= k * m[c, j];
                }
            }
            var x = new double[n];
            for (int i = 0; i < n; i++) x[i] = m[i, n] / m[i, i];
            return x;
        }
    }
}
