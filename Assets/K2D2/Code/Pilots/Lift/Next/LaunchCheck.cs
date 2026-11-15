using System;
using System.Collections.Generic;
using K2D2.Lift.Stage1;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Pre-launch check for the New lift system (NN ascent report, Sections 9 and 10.2): flies
    /// stage 1 in Stage1Sim under the SAME arbiter, behaviour policy and shield the flight uses (not
    /// a copy) and reports whether stage 1 can reach the target apoapsis and whether what's left can
    /// circularise there. BehaviourTrainer uses the same simulation (Simulate) to train the weights.
    /// The kick / throttle-hold / pull-down grid search of the pre-behaviour-model path was removed
    /// 2026-11-15 (Archive/lift-legacy-2026-11-15/).
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
            // stage 1's real Δv left at cut-off minus the sim's (2026-11-15: -20..-34 m/s on the 83 t
            // design - ~0.4 t more fuel burned on the way up than modelled). Added to the sim's
            // leftover, so training keeps a real margin to the debris cap instead of the sim's 2-4 m/s.
            public double s1LeftBiasDv = 0;
            public string source = "default";
            public Calibration Clone() => (Calibration)MemberwiseClone();
            public override string ToString() => $"drag ×{dragScale:n2}, thrust ×{thrustScale:n3}, nose {aoaBiasDeg:+0.00;-0.00}°, AoA ×{aoaGain:n2}, trim {trimDeg:+0.00;-0.00}, S1 burn loss {s1BurnLossDv:n0} m/s, S1 left {s1LeftBiasDv:+0;-0;0} m/s ({source})";
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
            public double spareDv => remainingDv - upperCircDv;
            public double maxQKPa, maxHeatPct;
            public double maxQAoA;        // peak q x |nose - path| while flying (kPa·°) - the trainer's drift-risk proxy
            public double liftoffTwr;     // at maxThrottle
            public double maxThrottle;
            // the simulated state at cut-off / burnout, kept to compare with the flight
            public double cutSpeed, cutPathDeg, cutPeKm, cutMass, cutT;
            public double stage1LeftDv;   // stage 1's own Δv left at cut-off (vacuum), with the learned bias
            public double stage1LeftRawDv; // ... as simulated, before the bias (what the bias is learned against)
            public List<TracePoint> plan; // the chosen ascent, every PlanStepS
            public Calibration calibration;
            public double stage1Dv;       // vacuum, from the game's Δv data
            public double targetApKm;

            /// <summary>One line for the log / LIFT tab.</summary>
            public string Summary()
            {
                if (verdict == Verdict.Unknown) return "Launch check: " + problem;
                string drag = (withDrag ? "" : " (no drag - VAB estimate)") +
                              (overheats ? " - OVERHEATS" : !withinMargins ? " - NEAR LIMITS (over 90% of max Q or heating)" : "");
                string head = verdict switch
                {
                    Verdict.Ok => $"OK: Ap {apKm:n0} km at {altKm:n0} km, {spareDv:n0} m/s spare after circularising ({circDv:n0} m/s{S1Burn()})",
                    Verdict.NoCircularise => $"Can't circularise: Ap {apKm:n0} km but {upperCircDv:n0} m/s needed from the upper stage, {remainingDv:n0} m/s left (short {-spareDv:n0})",
                    _ => $"SHORT: stage 1 only reaches Ap {apKm:n0} km of {targetApKm:n0} km",
                };
                return $"{head}; behaviour model, {Twr()}, stage-1 Δv {stage1Dv:n0} m/s, max Q {maxQKPa:n1} kPa, heat {maxHeatPct:n0}%{drag}";
            }

            string S1Burn() => s1ApBurnDv > 0
                ? $"; stage 1 {s1ApBurnDv:n0} at Ap, dropped at Pe {debrisPeKm:n0} km, upper {upperCircDv:n0}"
                : "";

            /// <summary>TWR at liftoff, labelled with the throttle cap it assumes when that isn't 100%
            /// (the game's VAB readout is always at full throttle).</summary>
            public string Twr() => maxThrottle > 0 && maxThrottle < 0.995
                ? $"TWR {liftoffTwr:n2} @ {maxThrottle * 100:n0}%"
                : $"TWR {liftoffTwr:n2}";

            /// <summary>Short enough for a part-window line.</summary>
            public string Short()
            {
                string drag = withDrag ? "" : "*";
                return verdict switch
                {
                    Verdict.Ok => $"orbit OK{(overheats ? " but OVERHEATS" : "")}, {spareDv:n0} m/s spare{drag} · cut-off {altKm:n0} km, " +
                                  (s1ApBurnDv > 0 ? $"S1 {s1ApBurnDv:n0} at Ap → Pe {debrisPeKm:n0}" : $"S1 left {stage1LeftDv:n0}") +
                                  $" · {Twr()} · max Q {maxQKPa:n0} kPa",
                    Verdict.NoCircularise => $"reaches Ap, short {-spareDv:n0} m/s to circularise{drag} · {Twr()}",
                    Verdict.Short => $"SHORT: Ap {apKm:n0}/{targetApKm:n0} km, S1 Δv {stage1Dv:n0}{drag} · {Twr()}",
                    _ => problem,
                };
            }
        }

        const double MaxSimSeconds = 900;

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

        /// <summary>The launch check: one simulated flight of the behaviour model with its trace.</summary>
        public static Result Run(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm,
                                 Calibration calibration = null)
        {
            if (model == null || !model.valid)
                return new Result { targetApKm = targetApKm, problem = model == null ? "no vessel model" : "can't model the vessel: " + model.problem };
            return Simulate(model, ctx, headingDeg, targetApKm, calibration);
        }

        static Result RunOne(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm,
                             List<TracePoint> trace = null)
        {
            var body = BodyDescriptor.From(model.body, targetApKm);
            var sim = new Stage1Sim(model) { limitAoAInStep = false, dragScale = cal.dragScale, thrustScale = cal.thrustScale };
            double next_trace = 0;
            IAscentPolicy policy = new BehaviourPolicy();
            double prev_v = 0, prev_t = -1, speed_rate = 0;
            var shield = new SafetyShield();
            shield.Reset();
            var arbiter = new Arbiter();
            arbiter.Reset();

            bool handed_off = false;
            double max_q = 0, last_elev = 90, full_thrust = 0, max_qa = 0;
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
                targetApKm = targetApKm,
                apKm = res.apKm,
                altKm = res.altKm,
                maxQKPa = res.maxQkPa,
                maxQAoA = max_qa,
                maxHeatPct = res.maxHeat / Stage1Sim.HeatLimit * 100,
                stage1Dv = model.TotalDeltaVVac,
                liftoffTwr = LiftoffTwr(model, ctx.maxThrottle),
                maxThrottle = ctx.maxThrottle,
                reachedTarget = handed_off && arbiter.behaviour == Behaviour.CutOff,
                cutSpeed = last.airSpeed,
                cutPathDeg = last.pathDeg,
                cutPeKm = double.IsInfinity(last.peR) ? 0 : (last.peR - model.body.radius) / 1000,
                cutMass = last.mass,
                cutT = last.t,
                stage1LeftRawDv = res.leftoverDv,
                maxQLimitKPa = ctx.maxQKPa,
            };
            // stage 1's leftover with the learned bias (Calibration.s1LeftBiasDv)
            double left = Math.Max(0, res.leftoverDv + cal.s1LeftBiasDv);
            r.stage1LeftDv = left;
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
            r.remainingDv = left + model.UpperDeltaVVac;

            // Stage1Burn: stage 1's leftover at apoapsis up to the periapsis cap (less Stage1Burn.ReserveDv, as
            // Stage1Burn plans it), then dropped - anything above the cap is thrown away with it
            if (ctx.s1ApBurn && left > 0)
            {
                double atmo_km = body.hasAtmosphere ? body.atmosphereDepth / 1000 : 0;
                double cap_km = atmo_km > 0 ? Math.Min(ctx.s1CapPeKm, atmo_km - 2) : ctx.s1CapPeKm;
                double cap_r = Math.Min(body.radius + cap_km * 1000, ap);
                double v_cap = Math.Sqrt(Math.Max(0, body.mu * (2 / ap - 2 / (ap + cap_r))));
                double usable = Math.Max(0, left - K2D2.Lift.Stage1Burn.ReserveDv - cal.s1BurnLossDv);
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
