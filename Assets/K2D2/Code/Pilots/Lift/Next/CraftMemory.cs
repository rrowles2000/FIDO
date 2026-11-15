using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using K2D2.Lift.Stage1;
using Newtonsoft.Json;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// What the New lift system has learned about each vessel design, kept between flights in
    /// k2d2_craft_memory.json next to the DLL.
    ///
    /// 2026-11-14 revision: learned now is steering AUTHORITY - thrust per unit of dynamic pressure -
    /// not a q cap. A q cap flown with the throttle halved the 83 t vessel's thrust at Mach 0.63
    /// and it flipped earlier than ever. Every drift onset so far was at 42-56 kN/kPa, so a design
    /// that drifted must keep 1.3 x its worst onset ratio; the launch check only accepts ascents
    /// that do (at full thrust that means a loftier climb), throttle holds are off and the AoA limit
    /// is tight. Clean flights relax the floor 7%, drifts raise it. (Original note follows.)
    ///
    /// The one thing learned so far was a transonic dynamic-pressure cap. On 2026-11-13 the 83 t
    /// test vessel (body flaps top and bottom) flew cleanly to Mach 0.95, then its nose left the
    /// airflow at Mach 0.98 / q 25 kPa and it flipped within 3 s - too fast to rescue in flight.
    /// What helps is crossing that Mach band at lower q, which has to be planned before launch.
    /// So a drift (or a loss of control) at Mach m and q sets, for that design:
    ///     q cap = 0.7 x q at onset, across Mach m - 0.2 .. m + 0.3
    /// The shield holds q under the cap in that band (looking ahead), the launch check simulates
    /// it and picks the kick / throttle hold around it, and every clean flight relaxes the cap by
    /// 10% while another drift tightens it - so it settles at the design's real limit.
    ///
    /// A design is identified by stage 1's liftoff mass and thrust (rounded) - the same numbers in
    /// the VAB and on the pad.
    /// </summary>
    public static class CraftMemory
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.CraftMemory");

        public const double CapFraction = 0.7, RelaxFactor = 1.10, MinCapKPa = 8;
        public const double BandBelow = 0.2, BandAbove = 0.3;

        public class Event
        {
            public string when;
            public string kind;   // drift, lost-control, clean
            public double mach, q, altKm;
            public double thrustKN;      // at the onset - authority = thrust / q
        }

        /// <summary>
        /// One pad launch as the Training tab charts it (2026-11-15): the real score against the
        /// trained sim's prediction, Pe at separation against the plan, spare Δv against the plan,
        /// and how well the calibration fitted the flight. NaN where a value wasn't measured.
        /// </summary>
        public class FlightRecord
        {
            public string when;
            public bool ok;
            public double realScore = double.NaN, simScore = double.NaN;
            public double sepPeKm = double.NaN, plannedPeKm = double.NaN, capKm = double.NaN;
            public double spareDv = double.NaN, plannedSpareDv = double.NaN;
            public double driftDeg = double.NaN, maxQKPa = double.NaN;
            public double calRmsBefore = double.NaN, calRmsAfter = double.NaN;
        }

        public class Record
        {
            public string key, name;
            public string designName;          // the VAB design's name: the flight name without its launch counter (2026-11-15)
            public double s1MassT, s1ThrustKN; // what the design is matched on (within MatchTolerance)
            public int finsAhead = -1, finsBehind = -1; // ... and exactly on these when both sides know them
            public int flights;
            public double minThrustPerQ;      // kN per kPa the plan must keep (0 = none learned)
            // fitted model factors (Calibrator)
            public double calDrag = 1, calThrust = 1, calAoA = 0, calAoAGain = 1, calTrim = 0;
            public double calS1Loss;          // stage-1 apoapsis burn loss, m/s (LearnS1BurnLoss)
            public int s1LossFlights;
            public double calS1Left;          // stage-1 Δv left at cut-off, real minus sim, m/s: Kalman mean (LearnS1Left)
            public double calS1LeftVar;       // ... and its variance, (m/s)² (0 in records from before 2026-11-15 pm)
            public int s1LeftFlights;
            public BehaviourWeights? flownWeights; // the weights the last pad launch actually flew (trust-region anchor)
            public int calFlights;
            public bool fragile;              // has drifted outside the kick
            public double capKPa;             // superseded 2026-11-14 (a throttle q cap cut steering authority); ignored
            public double capMachLo, capMachHi;
            public string capReason = "";
            public List<Event> events = new();
            // (PlanLearner's "plans" and the max-Q "qHold" were dropped 2026-11-15 with the
            // pre-behaviour-model path; older files still load - unknown fields are skipped)
            // behaviour model: the weights training settled on, its simulated score, and the real flights' scores
            public BehaviourWeights? weights;
            public double simScore;
            public List<double> realScores = new();
            public List<FlightRecord> history = new(); // every pad launch since 2026-11-15, for the Training chart
        }

        static Dictionary<string, Record> records;
        static string path;

        static string FilePath => path ??= Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location) ?? ".",
                                                        "k2d2_craft_memory.json");

        static Dictionary<string, Record> All()
        {
            if (records != null) return records;
            try
            {
                records = File.Exists(FilePath)
                    ? JsonConvert.DeserializeObject<Dictionary<string, Record>>(File.ReadAllText(FilePath)) ?? new()
                    : new();
            }
            catch (Exception e)
            {
                logger.LogWarning($"[Memory] couldn't read {FilePath}: {e.Message} - starting empty");
                records = new();
            }
            return records;
        }

        static void Save()
        {
            try { File.WriteAllText(FilePath, JsonConvert.SerializeObject(records, Formatting.Indented)); }
            catch (Exception e) { logger.LogWarning($"[Memory] couldn't write {FilePath}: {e.Message}"); }
        }

        /// <summary>Design key: stage 1 liftoff mass to 0.5 t and vacuum thrust to 10 kN.</summary>
        public static string Key(Stage1Model model)
        {
            if (model?.phases == null || model.phases.Count == 0) return null;
            var p = model.phases[0];
            string k = $"{Math.Round(p.startMass * 2) / 2:0.0}t-{Math.Round(p.thrustVac / 10) * 10:0}kN";
            return model.finsAhead >= 0 ? $"{k}-s{model.finsAhead}/{model.finsBehind}" : k;
        }

        // Matching within 2% rather than on the rounded key alone: 1,505.4 kN rounds to 1510 while a
        // hand-written 1500 missed it (2026-11-14), and the VAB and pad numbers differ slightly.
        public const double MatchTolerance = 0.02;

        static bool Matches(Record r, double mass, double thrust, int ahead, int behind)
        {
            if (ahead >= 0 && r.finsAhead >= 0 && (ahead != r.finsAhead || behind != r.finsBehind)) return false;
            double rm = r.s1MassT, rt = r.s1ThrustKN;
            if (rm <= 0 || rt <= 0)
            {
                // older records: read them back from the key ("83.0t-1500kN")
                var m = System.Text.RegularExpressions.Regex.Match(r.key ?? "", @"^([\d.]+)t-([\d.]+)kN");
                if (!m.Success) return false;
                rm = double.Parse(m.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
                rt = double.Parse(m.Groups[2].Value, System.Globalization.CultureInfo.InvariantCulture);
            }
            return Math.Abs(rm - mass) <= MatchTolerance * mass && Math.Abs(rt - thrust) <= MatchTolerance * thrust;
        }

        public static Record Get(Stage1Model model)
        {
            string key = Key(model);
            if (key == null) return null;
            var all = All();
            if (all.TryGetValue(key, out var exact) &&
                (model.finsAhead < 0 || exact.finsAhead < 0 || (exact.finsAhead == model.finsAhead && exact.finsBehind == model.finsBehind)))
                return exact;
            var p = model.phases[0];
            Record best = null;
            foreach (var r in all.Values)
                if (Matches(r, p.startMass, p.thrustVac, model.finsAhead, model.finsBehind) && (best == null || r.flights > best.flights)) best = r;
            return best;
        }

        /// <summary>Puts the design's learned limits into the context the shield and simulator use.</summary>
        public const double AuthorityMargin = 1.3, AuthorityRelax = 0.93, FragileQAlpha = 2;

        /// <summary>Puts the design's learned limits into the context the shield and simulator use.</summary>
        public static void ApplyTo(ref PolicyContext ctx, Stage1Model model)
        {
            var r = Get(model);
            if (r == null) return;
            if (r.fragile)
            {
                ctx.fragile = true;
                ctx.qAlphaKPaDeg = Math.Min(ctx.qAlphaKPaDeg, FragileQAlpha);
            }
        }

        /// <summary>
        /// The model corrections to use for this design: its own once it has been calibrated,
        /// otherwise the average over every calibrated design, otherwise none.
        /// </summary>
        public static LaunchCheck.Calibration GetCalibration(Stage1Model model)
        {
            var r = Get(model);
            if (r != null && r.calFlights > 0)
                return WithS1Left(new LaunchCheck.Calibration { dragScale = r.calDrag, thrustScale = r.calThrust, aoaBiasDeg = r.calAoA, aoaGain = r.calAoAGain > 0 ? r.calAoAGain : 1, trimDeg = r.calTrim, s1BurnLossDv = r.calS1Loss,
                                                     source = $"this design, {r.calFlights} flight(s)" }, r);
            double w = 0, d = 0, t = 0, k = 0, gsum = 0, trs = 0;
            foreach (var x in All().Values)
                if (x.calFlights > 0)
                {
                    double wi = Math.Min(5, x.calFlights);
                    w += wi; d += wi * Math.Log(x.calDrag); t += wi * x.calThrust; k += wi * x.calAoA; gsum += wi * Math.Log(x.calAoAGain > 0 ? x.calAoAGain : 1); trs += wi * x.calTrim;
                }
            if (w <= 0) return WithS1Left(new LaunchCheck.Calibration(), r);
            return WithS1Left(new LaunchCheck.Calibration { dragScale = Math.Exp(d / w), thrustScale = t / w, aoaBiasDeg = k / w, aoaGain = Math.Exp(gsum / w), trimDeg = trs / w,
                                                 source = "average of calibrated designs" }, r);
        }

                // ---------------------------------------------------------------- names

        /// <summary>
        /// The VAB design a flight came from: the vessel name without a hand-written "(...)" note and
        /// without the "-N" launch counter the game adds ("R2-1-2" -> "R2-1", "test002-1" -> "test002").
        /// </summary>
        public static string DesignName(string vesselName)
        {
            if (string.IsNullOrWhiteSpace(vesselName)) return null;
            string n = System.Text.RegularExpressions.Regex.Replace(vesselName, @"\s*\(.*\)\s*$", "").Trim();
            n = System.Text.RegularExpressions.Regex.Replace(n, @"-\d+$", "");
            return n.Length > 0 ? n : vesselName.Trim();
        }

        static void Named(Record r, string name)
        {
            if (r == null || string.IsNullOrWhiteSpace(name)) return;
            r.name = name;
            r.designName = DesignName(name);
        }

        /// <summary>The design's display name ("R2-1"), else its key.</summary>
        public static string DisplayName(Record r) => r == null ? null : r.designName ?? DesignName(r.name) ?? r.key;

        static bool NameMatches(Record r, string workspace)
        {
            string w = workspace.Trim();
            if (string.Equals(DisplayName(r), w, StringComparison.OrdinalIgnoreCase)) return true;
            // a flight of that workspace: "<workspace>-<digits>" (older records have only the flight name)
            string n = System.Text.RegularExpressions.Regex.Replace(r.name ?? "", @"\s*\(.*\)\s*$", "").Trim();
            return n.StartsWith(w + "-", StringComparison.OrdinalIgnoreCase) &&
                   System.Text.RegularExpressions.Regex.IsMatch(n.Substring(w.Length + 1), @"^\d+$");
        }

        /// <summary>For the VAB's open dialog: the most-trained record flown from a workspace of this name.</summary>
        public static Record FindByDesignName(string workspaceName)
        {
            if (string.IsNullOrWhiteSpace(workspaceName)) return null;
            Record best = null;
            foreach (var r in All().Values)
            {
                if (!NameMatches(r, workspaceName)) continue;
                if (best == null) { best = r; continue; }
                bool r_trained = r.weights != null, best_trained = best.weights != null;
                // a trained record beats an untrained one; between equals, the one flown most
                if (r_trained != best_trained ? r_trained : r.flights > best.flights) best = r;
            }
            return best;
        }

        /// <summary>One line for the VAB's open dialog: whether the design is trained.</summary>
        public static string TrainingStatus(Record r)
        {
            if (r == null) return "K2D2: not trained";
            var real = r.realScores ?? new List<double>();
            if (r.weights == null) return $"K2D2: not trained ({r.flights} flight(s) logged)";
            if (real.Count == 0) return "K2D2: trained in the simulator, not flown yet";
            double best = double.NegativeInfinity;
            foreach (var v in real) if (v > best) best = v;
            return $"K2D2: trained - {real.Count} scored flight(s), latest score {real[real.Count - 1]:0.#}, best {best:0.#}";
        }

        public static void SaveCalibration(Stage1Model model, string name, LaunchCheck.Calibration c)
        {
            string key = Key(model);
            if (key == null) return;
            var all = All();
            var r = Get(model);
            if (r == null) all[key] = r = new Record { key = key, s1MassT = model.phases[0].startMass, s1ThrustKN = model.phases[0].thrustVac,
                                                       finsAhead = model.finsAhead, finsBehind = model.finsBehind };
            Named(r, name);
            r.calDrag = c.dragScale;
            r.calThrust = c.thrustScale;
            r.calAoA = c.aoaBiasDeg;
            r.calAoAGain = c.aoaGain;
            r.calTrim = c.trimDeg;
            r.calFlights++;
            Save();
        }

        public static BehaviourWeights? GetWeights(Stage1Model model) => Get(model)?.weights;

        static Record GetOrAdd(Stage1Model model, string name)
        {
            string key = Key(model);
            if (key == null) return null;
            var r = Get(model);
            if (r == null) All()[key] = r = new Record { key = key, s1MassT = model.phases[0].startMass, s1ThrustKN = model.phases[0].thrustVac,
                                                         finsAhead = model.finsAhead, finsBehind = model.finsBehind };
            Named(r, name);
            return r;
        }

        public static void SaveTraining(Stage1Model model, string name, BehaviourWeights w, double simScore)
        {
            var r = GetOrAdd(model, name);
            if (r == null) return;
            r.weights = w;
            r.simScore = simScore;
            Save();
        }

        /// <summary>After a separation: the real burn's loss against an instant burn (averaged, newest weighted 1/2).</summary>
        public static double LearnS1BurnLoss(Stage1Model model, string name, double lossDv)
        {
            var r = GetOrAdd(model, name);
            if (r == null) return 0;
            lossDv = Math.Max(-50, Math.Min(300, lossDv));
            r.calS1Loss = r.s1LossFlights == 0 ? lossDv : 0.5 * r.calS1Loss + 0.5 * lossDv;
            r.s1LossFlights++;
            Save();
            return r.calS1Loss;
        }

        // ---- stage-1 leftover correction: a one-state Kalman filter (2026-11-15 pm).
        // The first version moved half way to each flight's error, and three flights of R2-1 swung
        // it -45 -> -1 -> -28 m/s (errors -45, +42, -55: the error depends on the ascent flown).
        // Now each flight is a noisy measurement: the gain is large while the estimate is uncertain
        // and small once flights agree, and a little process noise lets it follow a changed rocket.
        // The sim plans with mean - 1 sd (PessimisticSd), so the margin to the debris cap follows
        // how sure the estimate is instead of being one fixed number.
        public const double S1LeftPriorSd = 40;     // m/s, before any flight
        public const double S1LeftMeasureSd = 35;   // m/s, one flight's scatter (seen: -45, +42, -55)
        public const double S1LeftDriftSd = 5;      // m/s per flight, the rocket/model changing
        public const double PessimisticSd = 1.0;    // plan with mean - this x sd

        static void S1LeftState(Record r, out double mean, out double var)
        {
            mean = r?.calS1Left ?? 0;
            int n = r?.s1LeftFlights ?? 0;
            if (r == null || n == 0) { mean = 0; var = S1LeftPriorSd * S1LeftPriorSd; return; }
            // records from the half-way version have a mean but no variance: n flights' worth
            var = r.calS1LeftVar > 0 ? r.calS1LeftVar : S1LeftMeasureSd * S1LeftMeasureSd / n;
        }

        static LaunchCheck.Calibration WithS1Left(LaunchCheck.Calibration c, Record r)
        {
            S1LeftState(r, out double mean, out double var);
            c.s1LeftMeanDv = mean;
            c.s1LeftSdDv = Math.Sqrt(var);
            c.s1LeftBiasDv = mean - PessimisticSd * c.s1LeftSdDv;
            return c;
        }

        /// <summary>
        /// After a cut-off: stage 1's real Δv left minus the sim's (unbiased) prediction, folded into
        /// the Kalman estimate. Returns the new mean and standard deviation.
        /// </summary>
        public static (double mean, double sd) LearnS1Left(Stage1Model model, string name, double errorDv)
        {
            var r = GetOrAdd(model, name);
            if (r == null) return (0, S1LeftPriorSd);
            errorDv = Math.Max(-300, Math.Min(300, errorDv));
            S1LeftState(r, out double mean, out double var);
            var += S1LeftDriftSd * S1LeftDriftSd;                          // predict
            double gain = var / (var + S1LeftMeasureSd * S1LeftMeasureSd); // update
            mean += gain * (errorDv - mean);
            var *= 1 - gain;
            r.calS1Left = mean;
            r.calS1LeftVar = var;
            r.s1LeftFlights++;
            Save();
            return (mean, Math.Sqrt(var));
        }

        /// <summary>The weights the last pad launch of this design flew, for training's trust region.</summary>
        public static BehaviourWeights? GetFlownWeights(Stage1Model model)
        {
            var r = Get(model);
            if (r == null) return null;
            // records from before flownWeights: the stored weights are the ones the last launch flew
            return r.flownWeights ?? ((r.realScores?.Count ?? 0) > 0 ? r.weights : null);
        }

        public static void RecordFlownWeights(Stage1Model model, string name, BehaviourWeights w)
        {
            var r = GetOrAdd(model, name);
            if (r == null) return;
            r.flownWeights = w;
            Save();
        }

        public static void RecordRealScore(Stage1Model model, string name, double score)
        {
            var r = GetOrAdd(model, name);
            if (r == null) return;
            r.realScores ??= new List<double>();
            r.realScores.Add(score);
            Save();
        }

        public static void RecordFlight(Stage1Model model, string name, FlightRecord f)
        {
            var r = GetOrAdd(model, name);
            if (r == null || f == null) return;
            r.history ??= new List<FlightRecord>();
            r.history.Add(f);
            if (r.history.Count > 200) r.history.RemoveRange(0, r.history.Count - 200);
            Save();
        }

        /// <summary>For the Training tab: the behaviour model's training state for this design.</summary>
        public static string DescribeTraining(Stage1Model model)
        {
            var r = Get(model);
            if (r?.weights == null) return "not trained yet - the launch check trains it";
            var real = r.realScores ?? new List<double>();
            string rs = real.Count == 0 ? "no real flights yet"
                : $"real flights: first {real[0]:n1}, last {real[real.Count - 1]:n1}, best {System.Linq.Enumerable.Max(real):n1} ({real.Count})";
            return $"sim score {r.simScore:n1}; {rs} (score = Pe at stage-1 separation, km, minus penalties)";
        }

                public static string Describe(Stage1Model model)
        {
            var r = Get(model);
            if (r == null) return "new design";
            if (!r.fragile && r.minThrustPerQ <= 0) return $"{r.flights} flight(s), no limit learned";
            return $"FRAGILE: heading correction Q·AoA ≤ {FragileQAlpha:n0}, drifted at thrust/q {r.minThrustPerQ / AuthorityMargin:n0} kN/kPa ({r.capReason}; {r.flights} flight(s))";
        }

        /// <summary>
        /// After a flight. onsetMach/onsetQ: the last drift onset outside the kick (NaN if none);
        /// lost: the run ended in loss of control (locMach/locQ where); flewBand: the flight got
        /// past the cap's Mach band (a clean pass counts as evidence to relax).
        /// </summary>
        // 2026-11-14: one recovered 1.7° wobble at Mach 1.54 marked a design FRAGILE for good, and it
        // then flew with the fragile limits for no reason. Now only a real departure counts at once;
        // small recovered drifts count when they keep happening; clean flights clear the flag.
        public const double MajorDriftDeg = 3;
        public const int ClearAfterClean = 3;

        public static void Learn(Stage1Model model, string name, double onsetMach, double onsetQ, double onsetThrust, double onsetPeakDeg,
                                 bool lost, double locMach, double locQ, double locThrust, double locAltKm, double maxMach, bool completed)
        {
            string key = Key(model);
            if (key == null) return;
            var all = All();
            var r = Get(model);
            if (r == null) all[key] = r = new Record { key = key };
            Named(r, name);
            r.s1MassT = model.phases[0].startMass;
            r.s1ThrustKN = model.phases[0].thrustVac;
            if (model.finsAhead >= 0) { r.finsAhead = model.finsAhead; r.finsBehind = model.finsBehind; }
            r.flights++;
            r.capKPa = 0; // the old q cap is not used any more
            string when = DateTime.Now.ToString("yyyy-MM-dd HH:mm");
            string before = Describe(model);

            bool have_onset = !double.IsNaN(onsetMach) && onsetQ > 0;
            double m = have_onset ? onsetMach : lost ? locMach : double.NaN;
            double q = have_onset ? onsetQ : lost ? locQ : double.NaN;
            double t = have_onset ? onsetThrust : lost ? locThrust : double.NaN;
            bool major = lost || (have_onset && onsetPeakDeg >= MajorDriftDeg);
            if (!double.IsNaN(m) && q > 0 && !major)
            {
                // small, recovered: note it; only repeated ones make the design fragile
                r.events.Add(new Event { when = when, kind = "minor-drift", mach = m, q = q, thrustKN = t, altKm = locAltKm });
                int recent = 0, n = 0;
                for (int i = r.events.Count - 1; i >= 0 && n < 3; i--, n++)
                    if (r.events[i].kind == "minor-drift") recent++;
                if (recent >= 2) major = true;
            }
            if (major && !double.IsNaN(m) && q > 0)
            {
                if (r.events.Count == 0 || r.events[r.events.Count - 1].when != when)
                    r.events.Add(new Event { when = when, kind = lost ? "lost-control" : "drift", mach = m, q = q, thrustKN = t, altKm = locAltKm });
                r.fragile = true;
                if (t > 0)
                    r.minThrustPerQ = Math.Max(r.minThrustPerQ, AuthorityMargin * t / q);
                r.capReason = $"{(lost ? "lost control" : "drift")} at Mach {m:n2}, q {q:n1} kPa, thrust {t:n0} kN (thrust/q {t / q:n0}) on {when}";
            }
            else if (!double.IsNaN(m) && q > 0)
            {
                // minor drift recorded above; nothing else changes
            }
            else
            {
                r.events.Add(new Event { when = when, kind = "clean", mach = maxMach });
                if (completed && r.minThrustPerQ > 0)
                {
                    r.minThrustPerQ *= AuthorityRelax;
                    r.capReason += $"; relaxed after a clean flight {when}";
                }
                // enough clean flights in a row: not fragile any more
                int clean_run = 0;
                for (int i = r.events.Count - 1; i >= 0 && r.events[i].kind == "clean"; i--) clean_run++;
                if (completed && r.fragile && clean_run >= ClearAfterClean)
                {
                    r.fragile = false;
                    r.minThrustPerQ = 0;
                    r.capReason = $"fragile flag cleared after {clean_run} clean flights ({when})";
                }
            }
            if (r.events.Count > 40) r.events.RemoveRange(0, r.events.Count - 40);
            Save();
            logger.LogInfo($"[Memory] {key} '{name}': {before} -> {Describe(model)}");
        }
    }
}
