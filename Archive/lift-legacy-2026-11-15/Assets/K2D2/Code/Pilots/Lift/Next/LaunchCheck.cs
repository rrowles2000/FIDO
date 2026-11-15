using System;
using System.Collections.Generic;
using K2D2.Lift.Stage1;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Pre-launch check for the New lift system (NN ascent report, Sections 9 and 10.2): flies
    /// stage 1 in Stage1Sim under the SAME arbiter, policy and shield the flight uses (not a copy),
    /// for a range of pitch-kick angles, and reports whether stage 1 can reach the target apoapsis
    /// and whether what's left can circularise there. The best kick becomes the flight's kick when
    /// the kick is on auto.
    ///
    /// Used on the pad (AscentNext.Start, the NEW SYSTEM foldout's Check button) with the vessel's
    /// drag table, and in the VAB (VabAdvisor) without drag - the VAB has no drag cubes, so its
    /// verdict is optimistic and says so.
    ///
    /// 2026-11-13 run 2 (8.85 t -> 4.85 t, mass ratio 1.82) burnt out at 25 km with Ap 45 km of 90;
    /// nothing checked beforehand that stage 1 could get there.
    /// </summary>
    public static class LaunchCheck
    {
        /// <summary>
        /// Model corrections fitted from flights (Calibrator): drag, thrust at the same propellant
        /// flow, and the steady angle the real nose holds above the commanded direction while flying
        /// (2026-11-14: the 83 t vessel held +0.3..1.0° all the way up, which turns it less - the
        /// first version fitted a kick gain instead and could only explain part of it).
        /// </summary>
        public class Calibration
        {
            public double dragScale = 1, thrustScale = 1, aoaBiasDeg = 0;
            // the real AoA = aoaGain x the commanded AoA (+ aoaBiasDeg): the nose over-shoots a
            // commanded AoA (kick, pull-down) - flights with different kicks turned more AND less
            // than planned, which a bias alone can't do
            public double aoaGain = 1;
            // gimbal trim: thrust points trimDeg x q / (thrust per mass) below the nose - SAS
            // holds the nose against the aerodynamic load by deflecting the engine, which bends
            // the path more at low throttle and high q (2026-11-14: TWR-hold plans turned 9° more
            // than planned, full-throttle plans 4° less - constant factors can't do both)
            public double trimDeg = 0;
            // stage 1's apoapsis burn needs this much more than an instant burn (finite burn, coast
            // drag) - learned from real separations (2026-11-14: 9-16 m/s on the 83 t vessel)
            public double s1BurnLossDv = 0;
            public string source = "default";
            public Calibration Clone() => (Calibration)MemberwiseClone();
            public override string ToString() => $"drag ×{dragScale:n2}, thrust ×{thrustScale:n3}, nose {aoaBiasDeg:+0.00;-0.00}°, AoA ×{aoaGain:n2}, trim {trimDeg:+0.00;-0.00}, S1 burn loss {s1BurnLossDv:n0} m/s ({source})";
        }

        /// <summary>One point of the simulated ascent, for the [Next] PLAN log and the model export.</summary>
        public struct TracePoint
        {
            public double t, altKm, v, path, q, thrustKN, mass, cmd, apKm, peKm, throttle;
        }

        // the calibration the current Run uses (set for the duration of Run / Simulate)
        static Calibration cal = new Calibration();
        public enum Verdict { Unknown, Ok, NoCircularise, Short }

        public class Result
        {
            public Verdict verdict = Verdict.Unknown;
            public string problem = "";   // why the check couldn't run
            public bool withDrag;
            public double kickDeg;
            public double kickSpeed;      // m/s, where the kick starts
            public double holdTwr, holdKm; // throttle hold (holdTwr 0 = none)
            public double pullDownDeg;    // nose below the flight path after max Q (0 = pure gravity turn)
            public bool reachedTarget;    // apoapsis reached the target while stage 1 burned
            public double apKm, altKm;    // at cut-off / burnout
            public double circDv;         // to circularise at that apoapsis (stage-1 burn + upper)
            public double remainingDv;    // what pays for upperCircDv: upper stages (+ stage 1 if no stage-1 burn)
            public double upperCircDv;    // the part of circDv the upper stage has to give
            public double s1ApBurnDv;     // stage 1's apoapsis burn (0 = none), as an instant burn
            public double s1UnusedDv;     // stage 1's Δv still left after its apoapsis burn reached the cap (margin)
            public double debrisPeKm = double.NaN; // stage 1's periapsis when dropped
            public bool overheats => maxHeatPct > 100;
            public double maxQLimitKPa;   // the max-Q setting this was planned against
            // keeps the margin the player asked for (2026-11-14): peak q and heating both under
            // LimitMargin of their limits - a plan at 30 kPa / 88% heat wobbled at Mach 1.54
            public bool withinMargins => maxHeatPct <= LimitMargin * 100 && (maxQLimitKPa <= 0 || maxQKPa <= (LimitMargin + 0.01) * maxQLimitKPa);
            public double minThrustPerQ = double.PositiveInfinity; // lowest thrust/q while flying (q > 3 kPa)
            public double neededThrustPerQ;   // the learned floor this was checked against (0 = none)
            public bool authorityOk => neededThrustPerQ <= 0 || minThrustPerQ >= neededThrustPerQ;
            public bool authorityShort;       // the chosen plan (and every other) is under the floor
            public bool marginal;         // no kick with a safe neighbour - the chosen one sits next to a failure
            internal bool noMargin;       // the next-bigger kick (same speed and hold) isn't safe
            public double spareDv => remainingDv - upperCircDv;
            public double maxQKPa, maxHeatPct;
            public double maxQAoA;        // peak q x |nose - path| while flying (kPa·°) - the trainer's drift-risk proxy
            public double liftoffTwr;     // at maxThrottle
            public double maxThrottle;
            // the simulated state at cut-off / burnout, kept to compare with the flight
            public double cutSpeed, cutPathDeg, cutPeKm, cutMass, cutT;
            public double stage1LeftDv;   // stage 1's own Δv left at cut-off (vacuum)
            public List<Result> trials;   // every kick simulated (best first is not implied)
            // every plan that passed the safety rules with a safe bigger-kick neighbour - the
            // plans PlanLearner may choose between (empty: nothing safe, the choice is marginal)
            public List<Result> eligible = new();
            public string learnerNote = "";  // why PlanLearner picked this plan ("" = it didn't run)
            public List<TracePoint> plan; // the chosen ascent, every PlanStepS
            public Calibration calibration;
            public double stage1Dv;       // vacuum, from the game's Δv data
            public double targetApKm;
            public int kicksTried;

            /// <summary>One line for the log / LIFT tab.</summary>
            public string Summary()
            {
                if (verdict == Verdict.Unknown) return "Launch check: " + problem;
                string drag = (withDrag ? "" : " (no drag - VAB estimate)") +
                              (overheats ? " - OVERHEATS" : !withinMargins ? " - NEAR LIMITS (over 90% of max Q or heating)" : "") +
                              (marginal ? " - MARGINAL: a slightly bigger kick fails" : "") +
                              (neededThrustPerQ > 0 ? $" - steering: thrust/q {minThrustPerQ:n0} (needs {neededThrustPerQ:n0})" : "") +
                              (authorityShort ? " - UNSAFE: no ascent keeps enough steering authority" : "");
                string head = verdict switch
                {
                    Verdict.Ok => $"OK: Ap {apKm:n0} km at {altKm:n0} km, {spareDv:n0} m/s spare after circularising ({circDv:n0} m/s{S1Burn()})",
                    Verdict.NoCircularise => $"Can't circularise: Ap {apKm:n0} km but {upperCircDv:n0} m/s needed from the upper stage, {remainingDv:n0} m/s left (short {-spareDv:n0})",
                    _ => $"SHORT: stage 1 only reaches Ap {apKm:n0} km of {targetApKm:n0} km",
                };
                return $"{head}; {KickText()}, {Twr()}, stage-1 Δv {stage1Dv:n0} m/s, max Q {maxQKPa:n1} kPa, heat {maxHeatPct:n0}%{drag}";
            }

            public bool blend;            // flown by the behaviour model (no kick / hold / pull-down)
            public string KickText() => blend ? "behaviour model" : $"kick {kickDeg:n1}° at {kickSpeed:n0} m/s" +
                (holdTwr > 0 ? $", TWR held at {holdTwr:n1} to {holdKm:n0} km" : "") +
                (pullDownDeg > 0 ? $", pull-down {pullDownDeg:n0}° after max Q" : "");

            string S1Burn() => s1ApBurnDv > 0
                ? $"; stage 1 {s1ApBurnDv:n0} at Ap, dropped at Pe {debrisPeKm:n0} km, upper {upperCircDv:n0}"
                : "";

            /// <summary>TWR at liftoff, labelled with the throttle cap it assumes when that isn't 100%
            /// (the game's VAB readout is always at full throttle).</summary>
            public string Twr() => maxThrottle > 0 && maxThrottle < 0.995
                ? $"TWR {liftoffTwr:n2} @ {maxThrottle * 100:n0}%"
                : $"TWR {liftoffTwr:n2}";

            /// <summary>
            /// The search, for the log: a kick-angle x kick-speed grid of spare Δv (no throttle hold;
            /// SHORT / NOCIRC / HOT where it fails), then the throttle holds tried on the best cells.
            /// [x] marks the choice, * a cell whose next-bigger kick fails (no safety margin).
            /// </summary>
            public IEnumerable<string> Table()
            {
                if (trials == null || trials.Count < 2) yield break;
                var speeds = new List<double>();
                var kicks = new List<double>();
                foreach (var t in trials)
                    if (t.holdTwr <= 0 && t.pullDownDeg <= 0)
                    {
                        if (!speeds.Contains(t.kickSpeed)) speeds.Add(t.kickSpeed);
                        if (!kicks.Contains(t.kickDeg)) kicks.Add(t.kickDeg);
                    }
                speeds.Sort();
                kicks.Sort();
                string head = "kick \\ start speed";
                foreach (var sp in speeds) head += $" {sp,8:n0}";
                yield return head + "   (spare Δv, m/s; no throttle hold)";
                foreach (var k in kicks)
                {
                    string line = $"{k,10:n1}°        ";
                    foreach (var sp in speeds)
                    {
                        var t = trials.Find(x => x.holdTwr <= 0 && x.pullDownDeg <= 0 && x.kickDeg == k && x.kickSpeed == sp);
                        string cell = t == null ? "-" : CellText(t);
                        if (t != null && holdTwr <= 0 && pullDownDeg <= 0 && t.kickDeg == kickDeg && t.kickSpeed == kickSpeed) cell = "[" + cell + "]";
                        line += $" {cell,8}";
                    }
                    yield return line;
                }
                foreach (var t in trials)
                {
                    if (t.holdTwr <= 0 && t.pullDownDeg <= 0) continue;
                    string mark = t.kickDeg == kickDeg && t.kickSpeed == kickSpeed && t.holdTwr == holdTwr && t.holdKm == holdKm &&
                                  t.pullDownDeg == pullDownDeg ? " <- chosen" : "";
                    yield return $"{t.KickText()}: {CellText(t).TrimEnd('*')}" +
                                 $" (cut-off {t.altKm:n0} km, path {t.cutPathDeg:n0}°, max Q {t.maxQKPa:n1}, heat {t.maxHeatPct:n0}%){mark}";
                }
                yield return $"chosen: {KickText()} - cut-off T+{cutT:n0} {altKm:n1} km, v {cutSpeed:n0} m/s, path {cutPathDeg:n1}°, " +
                             $"Pe {cutPeKm:n0} km, S1 left {stage1LeftDv:n0}, S1@Ap {s1ApBurnDv:n0}, upper {upperCircDv:n0}, spare {spareDv:n0} m/s, " +
                             $"max Q {maxQKPa:n1} kPa, heat {maxHeatPct:n0}%";
            }

            string CellText(Result t)
            {
                if (t.verdict == Verdict.Short) return "SHORT";
                if (t.verdict == Verdict.NoCircularise) return "NOCIRC";
                if (t.verdict == Verdict.Unknown) return "?";
                return (t.overheats ? "HOT" : t.spareDv.ToString("n0")) + (t.noMargin ? "*" : "");
            }

            /// <summary>Short enough for a part-window line.</summary>
            public string Short()
            {
                string drag = withDrag ? "" : "*";
                return verdict switch
                {
                    Verdict.Ok => $"orbit OK{(overheats ? " but OVERHEATS" : "")}{(marginal ? " (marginal kick)" : "")}, {spareDv:n0} m/s spare{drag} · cut-off {altKm:n0} km, " +
                                  (s1ApBurnDv > 0 ? $"S1 {s1ApBurnDv:n0} at Ap → Pe {debrisPeKm:n0}" : $"S1 left {stage1LeftDv:n0}") +
                                  $" · {Twr()} · max Q {maxQKPa:n0} kPa",
                    Verdict.NoCircularise => $"reaches Ap, short {-spareDv:n0} m/s to circularise{drag} · {Twr()}",
                    Verdict.Short => $"SHORT: Ap {apKm:n0}/{targetApKm:n0} km, S1 Δv {stage1Dv:n0}{drag} · {Twr()}",
                    _ => problem,
                };
            }
        }

        // 18 and 22 added 2026-11-13: two of three vessels picked 15, the top, where the safety rule
        // had no bigger neighbour to test. The top candidate itself is never chosen (see Run).
        static readonly double[] KickCandidates = { 0.5, 1, 1.5, 2, 3, 4, 5, 6, 8, 10, 12, 15, 18, 22 };
        // where the kick starts (surface m/s); 20 was the only value before 2026-11-13
        static readonly double[] KickSpeeds = { 20, 40, 60, 90 };
        // throttle holds tried on the best HoldTopCells pass-1 cells: TWR cap, up to altitude (km)
        static readonly (double twr, double km)[] Holds = { (1.6, 10), (1.6, 20), (2.0, 10), (2.0, 20) };
        const int HoldTopCells = 3;
        // nose below the flight path after max Q, deg (0 = pure gravity turn); the AoA limit caps it
        static readonly double[] PullDowns = { 0, 3, 6, 12 };
        const double MaxSimSeconds = 900;

        /// <param name="fixedKickDeg">null: try the candidates and keep the best; otherwise just this kick</param>
        public const double PlanStepS = 5;
        public const double LimitMargin = 0.90;

        /// <summary>One plan exactly as given (no search), with its trace - the calibration fit uses this.</summary>
        public static Result Simulate(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm, Calibration c)
        {
            var keep = cal;
            cal = c ?? new Calibration();
            try
            {
                var trace = new List<TracePoint>();
                var r = RunOne(model, ctx, headingDeg, targetApKm, trace);
                r.plan = trace;
                r.calibration = cal;
                return r;
            }
            finally { cal = keep; }
        }

        public static Result Run(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm, double? fixedKickDeg,
                                 Calibration calibration = null)
        {
            var keep = cal;
            cal = calibration ?? new Calibration();
            try
            {
                var r = RunSearch(model, ctx, headingDeg, targetApKm, fixedKickDeg);
                if (r.verdict != Verdict.Unknown && model != null && model.valid)
                {
                    // re-fly the choice with a trace, for the plan log
                    var c = ctx;
                    c.kickDeg = r.kickDeg;
                    c.kickSpeed = r.kickSpeed;
                    c.holdTwr = r.holdTwr;
                    c.holdKm = r.holdKm;
                    c.pullDownDeg = r.pullDownDeg;
                    var trace = new List<TracePoint>();
                    RunOne(model, c, headingDeg, targetApKm, trace);
                    r.plan = trace;
                }
                r.calibration = cal;
                return r;
            }
            finally { cal = keep; }
        }

        static Result RunSearch(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm, double? fixedKickDeg)
        {
            var best = new Result { targetApKm = targetApKm };
            if (model == null || !model.valid)
            {
                best.problem = model == null ? "no vessel model" : "can't model the vessel: " + model.problem;
                return best;
            }

            int tried = 0;
            var trials = new List<Result>();
            Result Sim(double kick, double speed, double hold_twr, double hold_km, double pull = 0)
            {
                var c = ctx;
                c.kickDeg = kick;
                c.kickSpeed = speed;
                c.holdTwr = hold_twr;
                c.holdKm = hold_km;
                c.pullDownDeg = pull;
                var r = RunOne(model, c, headingDeg, targetApKm);
                tried++;
                trials.Add(r);
                return r;
            }

            if (ctx.blend)
            {
                // behaviour model: no plan to search - one flight of the blend, for the prediction
                best = Sim(0, 0, 0, 0);
                best.kicksTried = tried;
                return best;
            }

            if (fixedKickDeg.HasValue)
            {
                // kick set by hand: fly exactly the settings, no search
                best = Sim(fixedKickDeg.Value, ctx.kickSpeed > 0 ? ctx.kickSpeed : GravityTurnPolicy.DefaultKickSpeed, 0, 0);
                best.kicksTried = tried;
                return best;
            }

            // Pass 1: every kick angle at every kick start speed, no throttle hold.
            // Safety margin (2026-11-13): the best kick was regularly the last one before a cliff -
            // 2° OK and 3° crashing, 4° OK, 5° at 175% heat, 6° crashing - so a small model error
            // flips the flight over the edge. A cell only counts if the next-bigger kick at the same
            // speed is safe too (a bigger kick is the flatter, riskier side).
            var grid = new Result[KickCandidates.Length, KickSpeeds.Length];
            for (int si = 0; si < KickSpeeds.Length; si++)
                for (int ki = 0; ki < KickCandidates.Length; ki++)
                    grid[ki, si] = Sim(KickCandidates[ki], KickSpeeds[si], 0, 0);

            var good = new List<(int ki, int si)>();
            Result plain_best = null;
            for (int si = 0; si < KickSpeeds.Length; si++)
                for (int ki = 0; ki < KickCandidates.Length; ki++)
                {
                    var r = grid[ki, si];
                    if (plain_best == null || Better(r, plain_best)) plain_best = r;
                    if (!Safe(r)) continue;
                    // no bigger neighbour simulated = no evidence of margin
                    if (ki + 1 >= KickCandidates.Length || !Safe(grid[ki + 1, si])) { r.noMargin = true; continue; }
                    good.Add((ki, si));
                }
            good.Sort((a, b) => BetterChoice(grid[a.ki, a.si], grid[b.ki, b.si]) ? -1 : BetterChoice(grid[b.ki, b.si], grid[a.ki, a.si]) ? 1 : 0);
            if (good.Count == 0)
            {
                best = plain_best;
                if (best.verdict == Verdict.Ok) best.marginal = true;
                if (ctx.minThrustPerQ > 0 && !best.authorityOk) best.authorityShort = true;
                best.kicksTried = tried;
                best.trials = trials;
                return best;
            }
            best = grid[good[0].ki, good[0].si];
            var eligible = new List<Result>();
            foreach (var (ki, si) in good) eligible.Add(grid[ki, si]);

            // Pass 2: on the best few cells, every throttle hold (and none) x every pull-down angle,
            // each with its safety neighbour. A fragile design keeps full thrust and no pull-down:
            // both take steering authority or add AoA.
            var hold_options = new List<(double twr, double km)> { (0, 0) };
            if (!ctx.fragile) hold_options.AddRange(Holds);
            var pulls = ctx.fragile ? new[] { 0.0 } : PullDowns;
            for (int n = 0; n < Math.Min(HoldTopCells, good.Count); n++)
            {
                var (ki, si) = good[n];
                foreach (var (twr, km) in hold_options)
                    foreach (double pull in pulls)
                    {
                        if (twr == 0 && pull == 0) continue; // that's the pass-1 cell itself
                        // (a cap above the liftoff TWR still matters: TWR climbs as stage 1 burns off)
                        var r = Sim(KickCandidates[ki], KickSpeeds[si], twr, km, pull);
                        if (!Safe(r)) continue;
                        // every safe plan's margin is checked now, not just the best's: PlanLearner chooses among them
                        if (ki + 1 >= KickCandidates.Length || !Safe(Sim(KickCandidates[ki + 1], KickSpeeds[si], twr, km, pull))) { r.noMargin = true; continue; }
                        eligible.Add(r);
                        if (BetterChoice(r, best)) best = r;
                    }
            }
            best.kicksTried = tried;
            best.trials = trials;
            best.eligible = eligible;
            return best;
        }

        // Between safe plans: the most spare Δv; within SpareTieDv of each other (e.g. several
        // plans that all take stage 1 to the debris cap) the one that leaves stage 1 the most
        // unused Δv (margin against model error), then the lower peak q.
        public const double SpareTieDv = 5;
        static double S1Unused(Result r) => Math.Max(0, r.stage1LeftDv - r.s1ApBurnDv);
        static bool BetterChoice(Result a, Result b)
        {
            if (Math.Abs(a.spareDv - b.spareDv) > SpareTieDv) return a.spareDv > b.spareDv;
            if (Math.Abs(S1Unused(a) - S1Unused(b)) > 5) return S1Unused(a) > S1Unused(b);
            return a.maxQKPa < b.maxQKPa;
        }

        static bool Safe(Result r) => r.verdict == Verdict.Ok && !r.overheats && r.authorityOk && r.withinMargins;

        // Ok (not overheating) beats Ok-but-overheating and NoCircularise, which beat Short; then
        // the most spare Δv (or the highest Ap when short).
        static bool Better(Result a, Result b)
        {
            double Rank(Result r) => r.verdict == Verdict.Ok ? (r.overheats || !r.authorityOk ? 2 : !r.withinMargins ? 2.5 : 3) : r.verdict == Verdict.NoCircularise ? 2 : r.verdict == Verdict.Short ? 1 : 0;
            if (Rank(a) != Rank(b)) return Rank(a) > Rank(b);
            return a.verdict == Verdict.Short ? a.apKm > b.apKm : a.spareDv > b.spareDv;
        }

        static Result RunOne(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm,
                             List<TracePoint> trace = null)
        {
            var body = BodyDescriptor.From(model.body, targetApKm);
            var sim = new Stage1Sim(model) { limitAoAInStep = false, dragScale = cal.dragScale, thrustScale = cal.thrustScale };
            double nominal_kick = ctx.kickDeg;
            double next_trace = 0;
            IAscentPolicy policy = ctx.blend ? new BehaviourPolicy() : new GravityTurnPolicy();
            double prev_v = 0, prev_t = -1, speed_rate = 0;
            var shield = new SafetyShield();
            shield.Reset();
            var arbiter = new Arbiter();
            arbiter.Reset();

            bool handed_off = false;
            double max_q = 0, last_elev = 90, full_thrust = 0, min_tpq = double.PositiveInfinity, max_qa = 0;
            float last_thr = 0;
            double dt = Stage1Sim.StepS;

            var res = sim.RunControlled((in Stage1Sim.Sample s, out double elev, out float thr) =>
            {
                max_q = Math.Max(max_q, s.qKPa);
                if (prev_t >= 0 && s.t > prev_t) speed_rate += 0.2 * ((s.airSpeed - prev_v) / (s.t - prev_t) - speed_rate);
                prev_v = s.airSpeed; prev_t = s.t;
                double path = s.pathDeg;
                if (last_thr > 0.05f) full_thrust = s.thrustKN / last_thr; // same estimate as the flight
                var obs = new AscentObservation
                {
                    valid = true, ut = s.t, landed = s.t < 0.5,
                    altitude = s.altitude, r = s.r, vr = s.vr, vt = s.vt, apR = s.apR, peR = s.peR,
                    airSpeed = s.airSpeed, speedRate = speed_rate, hasPath = s.airSpeed >= 10, pathDeg = path, airPathDeg = path,
                    horizontalSpeed = s.airSpeed * Math.Cos(path * Math.PI / 180), maxQSoFar = max_q,
                    qKPa = s.qKPa, mach = s.mach, heat = s.heat,
                    noseDeg = last_elev, // the simulator flies the command exactly
                    mass = s.mass, enginesLit = true, thrustKN = s.thrustKN, throttle = last_thr,
                    fullThrustKN = full_thrust,
                };
                if (arbiter.behaviour == Behaviour.Fly && s.qKPa > 3 && s.thrustKN > 0)
                    min_tpq = Math.Min(min_tpq, s.thrustKN / s.qKPa);
                var proposal = policy.Decide(obs, body, ctx);
                var action = arbiter.Decide(obs, body, ctx, proposal, shield.report.lossOfControl, false, out var wanted);
                var safe = shield.Apply(wanted, obs, ctx, dt);
                elev = safe.elevationDeg;
                // the real nose sits aoaBiasDeg above its command while flying in air (calibrated)
                if (arbiter.behaviour == Behaviour.Fly && s.qKPa > 1)
                {
                    double accel = s.mass > 0 ? s.thrustKN / s.mass : 0; // m/s²
                    double trim = accel > 0.5 ? Math.Max(-5, Math.Min(5, cal.trimDeg * s.qKPa / accel)) : 0;
                    elev = path + cal.aoaGain * (elev - path) + cal.aoaBiasDeg - trim;
                }
                thr = safe.throttle;
                if (trace != null && (s.t >= next_trace || action == ArbiterAction.HandOff || action == ArbiterAction.EndRun))
                {
                    next_trace = s.t + PlanStepS;
                    trace.Add(new TracePoint
                    {
                        t = s.t, altKm = s.altitude / 1000, v = s.airSpeed, path = s.pathDeg, q = s.qKPa, thrustKN = s.thrustKN,
                        mass = s.mass, cmd = elev, throttle = thr,
                        apKm = double.IsInfinity(s.apR) ? 1e9 : (s.apR - model.body.radius) / 1000, peKm = (s.peR - model.body.radius) / 1000,
                    });
                }
                last_elev = elev;
                if (arbiter.behaviour == Behaviour.Fly && s.qKPa > 1) max_qa = Math.Max(max_qa, s.qKPa * Math.Abs(elev - path));
                last_thr = thr;
                if (action == ArbiterAction.HandOff) { handed_off = true; thr = 0; return false; }
                return action != ArbiterAction.EndRun;
            }, headingDeg, MaxSimSeconds, out var last);

            var r = new Result
            {
                withDrag = model.dragValid,
                blend = ctx.blend,
                kickDeg = nominal_kick,
                kickSpeed = ctx.kickSpeed > 0 ? ctx.kickSpeed : GravityTurnPolicy.DefaultKickSpeed,
                holdTwr = ctx.holdTwr,
                holdKm = ctx.holdKm,
                pullDownDeg = ctx.pullDownDeg,
                targetApKm = targetApKm,
                apKm = res.apKm,
                altKm = res.altKm,
                maxQKPa = res.maxQkPa,
                maxQAoA = max_qa,
                maxHeatPct = res.maxHeat / Stage1Optimiser.HeatLimit * 100,
                stage1Dv = model.TotalDeltaVVac,
                liftoffTwr = LiftoffTwr(model, ctx.maxThrottle),
                maxThrottle = ctx.maxThrottle,
                reachedTarget = handed_off && arbiter.behaviour == Behaviour.CutOff,
                cutSpeed = last.airSpeed,
                cutPathDeg = last.pathDeg,
                cutPeKm = double.IsInfinity(last.peR) ? 0 : (last.peR - model.body.radius) / 1000,
                cutMass = last.mass,
                cutT = last.t,
                stage1LeftDv = res.leftoverDv,
                minThrustPerQ = min_tpq,
                maxQLimitKPa = ctx.maxQKPa,
                neededThrustPerQ = ctx.minThrustPerQ,
            };
            if (res.crashed || double.IsInfinity(last.apR) || last.apR <= 0)
            {
                r.verdict = res.crashed ? Verdict.Short : Verdict.Ok;
                return r;
            }

            // circularise at the apoapsis it has; the upper stages and stage 1's leftovers pay for it
            double ap = last.apR, pe = Math.Max(last.peR, 0);
            double v_ap = Math.Sqrt(Math.Max(0, body.mu * (2 / ap - 2 / (ap + pe))));
            double v_circ = Math.Sqrt(body.mu / ap);
            r.circDv = Math.Max(0, v_circ - v_ap);
            r.upperCircDv = r.circDv;
            r.remainingDv = res.leftoverDv + model.UpperDeltaVVac;

            // Stage1Burn: stage 1's leftover at apoapsis up to the periapsis cap (less Stage1Burn.ReserveDv, as
            // Stage1Burn plans it), then dropped - anything above the cap is thrown away with it
            if (ctx.s1ApBurn && res.leftoverDv > 0)
            {
                double atmo_km = body.hasAtmosphere ? body.atmosphereDepth / 1000 : 0;
                double cap_km = atmo_km > 0 ? Math.Min(ctx.s1CapPeKm, atmo_km - 2) : ctx.s1CapPeKm;
                double cap_r = Math.Min(body.radius + cap_km * 1000, ap);
                double v_cap = Math.Sqrt(Math.Max(0, body.mu * (2 / ap - 2 / (ap + cap_r))));
                double usable = Math.Max(0, res.leftoverDv - K2D2.Lift.Stage1Burn.ReserveDv - cal.s1BurnLossDv);
                r.s1ApBurnDv = Math.Max(0, Math.Min(v_cap - v_ap, usable));
                r.s1UnusedDv = usable - r.s1ApBurnDv;
                double v1 = v_ap + r.s1ApBurnDv;
                double energy = v1 * v1 / 2 - body.mu / ap;
                r.debrisPeKm = energy < 0 ? (-body.mu / energy - ap - body.radius) / 1000 : double.PositiveInfinity;
                r.upperCircDv = Math.Max(0, v_circ - v1);
                r.remainingDv = model.UpperDeltaVVac;
            }

            bool ap_clear = !body.hasAtmosphere || ap > body.radius + body.atmosphereDepth;
            if (!r.reachedTarget && !ap_clear) r.verdict = Verdict.Short;
            else r.verdict = r.spareDv >= 0 ? Verdict.Ok : Verdict.NoCircularise;
            return r;
        }

        public static double LiftoffTwr(Stage1Model model, float maxThrottle)
        {
            if (model?.phases == null || model.phases.Count == 0 || model.body == null) return 0;
            var ph = model.phases[0];
            double g = model.body.gravParameter / (model.body.radius * model.body.radius);
            double thrust_asl = ph.thrustVac * ph.ispASL / ph.ispVac;
            return thrust_asl * maxThrottle / (ph.startMass * g);
        }
    }
}
