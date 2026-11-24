using System;
using System.Collections.Generic;
using System.Linq;
using Fido.Constellation;
using Fido.Controller;
using Fido.KSPService;
using Fido.Lift.Stage1;
using Fido.Node;
using KSP.Game;
using KSP.Sim.DeltaV;
using KSP.Sim.impl;
using KSP.Sim.Maneuver;
using KTools;
using UnityEngine;

namespace Fido.Mission
{
    // ==================================================================== stage budget

    /// <summary>
    /// The stages a mission has to fly on, in firing order, with Δv taken off as steps use it - for
    /// "is there enough left after the circularise burn, and in which stage does each burn fall".
    /// VAB and pad: the stages above the stage-1 separator (stage 1 is used up and dropped on the way
    /// up). In flight after launch: the vessel's stages as they are now.
    /// </summary>
    public class StageBudget
    {
        public class Stage
        {
            public int stage;
            public double dv;       // m/s, vacuum
            public double thrust;   // kN, vacuum
            public double m0, m1;   // t, start and end of the stage
            public double isp;      // s, vacuum
        }

        public class Use
        {
            public double needed, left_before, left_after, burn_time_s;
            public int first_stage = -1, last_stage = -1; // game stage numbers
            public bool short_of_dv;
            public bool Crosses => first_stage != last_stage;
        }

        public readonly List<Stage> stages = new();
        public string source = "";
        int index;
        double used_in_stage;

        const double G0 = 9.80665;

        public double Left => index >= stages.Count ? 0 : stages.Skip(index).Sum(s => s.dv) - used_in_stage;
        public double LeftInStage => index >= stages.Count ? 0 : stages[index].dv - used_in_stage;
        public bool Empty => stages.Count == 0;

        public StageBudget Copy()
        {
            var b = new StageBudget { source = source };
            b.stages.AddRange(stages);
            return b;
        }

        /// <summary>Takes dv off, stage by stage; reports which stages it spans and how long it burns.</summary>
        public Use Burn(double dv)
        {
            var use = new Use { needed = dv, left_before = Left };
            double left = dv;
            while (left > 1e-6 && index < stages.Count)
            {
                var s = stages[index];
                if (use.first_stage < 0) use.first_stage = s.stage;
                use.last_stage = s.stage;
                double here = Math.Min(left, s.dv - used_in_stage);
                double ve = s.isp * G0;
                double m = s.m0 * Math.Exp(-used_in_stage / ve);
                if (s.thrust > 0)
                    use.burn_time_s += m * ve / s.thrust * (1 - Math.Exp(-here / ve));
                left -= here;
                used_in_stage += here;
                if (s.dv - used_in_stage <= 1e-6)
                {
                    index++;
                    used_in_stage = 0;
                }
            }
            use.short_of_dv = left > 1e-6;
            use.left_after = Left;
            return use;
        }

        static Stage From(DeltaVStageInfo s) => new Stage
        {
            stage = s.Stage, dv = s.DeltaVinVac, thrust = s.ThrustVac, m0 = s.StartMass, m1 = s.EndMass, isp = s.IspVac,
        };

        static Stage From(Stage1Model.BurnPhase p) => new Stage
        {
            stage = p.stage, dv = p.deltaVVac, thrust = p.thrustVac, m0 = p.startMass, m1 = p.endMass, isp = p.ispVac,
        };

        /// <summary>Stages above stage 1 from a stage-1 model (VAB or pad).</summary>
        public static StageBudget AfterStage1(Stage1Model model)
        {
            if (model == null || !model.valid) return null;
            var b = new StageBudget
            {
                source = model.separatorMarked ? $"stages above '{model.separatorName}'" : "stages after the first engine stage (no stage-1 separator marked)",
            };
            b.stages.AddRange(model.upperPhases.Where(p => p.deltaVVac > 0.5).Select(From));
            return b;
        }

        /// <summary>The vessel's stages as they stand (in flight).</summary>
        public static StageBudget Live(VesselComponent vessel)
        {
            var info = vessel?.VesselDeltaV?.StageInfo;
            if (info == null) return null;
            var b = new StageBudget { source = "the vessel's stages now" };
            b.stages.AddRange(info
                .Where(s => s != null && s.ThrustVac > 0 && s.StartMass > s.EndMass && s.IspVac > 0 && s.DeltaVinVac > 0.5)
                .OrderByDescending(s => s.Stage)
                .Select(From));
            return b;
        }

        public string Describe() =>
            Empty ? "no engine stages" : string.Join(", ", stages.Select(s => $"S{s.stage} {s.dv:n0} m/s"));
    }

    // ==================================================================== transfer targets and checks

    public static class TransferChecks
    {
        static UniverseModel Universe => GameManager.Instance?.Game?.UniverseModel;

        public static CelestialBodyComponent Body(string name)
        {
            if (string.IsNullOrEmpty(name)) return null;
            try { return Universe?.FindCelestialBodyByName(name); }
            catch (Exception) { return null; }
        }

        public static CelestialBodyComponent Home
        {
            get
            {
                try { return Universe?.HomeWorld; }
                catch (Exception) { return null; }
            }
        }

        /// <summary>Moons the transfer step offers: the home world's, and the current body's in flight.</summary>
        public static List<string> TargetChoices()
        {
            var names = new List<string>();
            void Add(CelestialBodyComponent parent)
            {
                if (parent?.orbitingBodies == null) return;
                foreach (var b in parent.orbitingBodies)
                    if (b != null && !names.Contains(b.bodyName)) names.Add(b.bodyName);
            }
            Add(Home);
            Add(FidoPlugin.Instance?.current_vessel?.VesselComponent?.mainBody);
            return names;
        }

        static bool Same(CelestialBodyComponent a, CelestialBodyComponent b) => a != null && b != null && a.bodyName == b.bodyName;

        public static bool IsTransfer(MissionStepData s) => s.type == StepCatalog.Transfer || s.type == StepCatalog.Correction;

        /// <summary>Steps that place a node for the Execute node step after them.</summary>
        public static bool PlacesNode(MissionStepData s) => IsTransfer(s) || s.type == StepCatalog.Capture;

        /// <summary>What a transfer-type step aims for at the target: the orbit of the Capture step after it, if there is one.</summary>
        public struct Arrival
        {
            public double alt_km;
            public TransferPlanner.ArrivalPlane plane;
            public int capture;   // index of that Capture step, -1 = none (the step's own arrival periapsis)
            public string Describe(string target) => capture >= 0
                ? $"a {alt_km:0.#} km {TransferPlanner.PlaneName(plane)} orbit at {target} (step {capture + 1})"
                : $"periapsis {alt_km:0.#} km at {target}";
        }

        public static Arrival ArrivalFor(List<MissionStepData> steps, MissionStepData s)
        {
            int i = steps?.IndexOf(s) ?? -1;
            if (s.type == StepCatalog.Capture)
                return new Arrival { alt_km = s.GetFloat("alt_km"), plane = TransferPlanner.ParsePlane(s.Get("orbit")), capture = i };
            if (i >= 0)
            {
                for (int j = i + 1; j < steps.Count; j++)
                {
                    var x = steps[j];
                    if (x.Get("target") != s.Get("target"))
                    {
                        if (x.type == StepCatalog.Transfer) break; // off somewhere else first
                        continue;
                    }
                    if (x.type == StepCatalog.Capture)
                        return new Arrival { alt_km = x.GetFloat("alt_km"), plane = TransferPlanner.ParsePlane(x.Get("orbit")), capture = j };
                }
            }
            return new Arrival { alt_km = s.GetFloat("pe_km"), plane = TransferPlanner.ArrivalPlane.Any, capture = -1 };
        }

        /// <summary>Rough Δv of a course correction: more the further the moon's plane is off the parking orbit.</summary>
        public static double CorrectionEstimate(double rel_inc_deg) => 10 + 600 * Math.Sin(Math.Abs(rel_inc_deg) * Math.PI / 180);

        /// <summary>The step order and target checks (no stage data needed).</summary>
        public static void CheckOrder(List<MissionStepData> steps, int i, List<MissionValidator.Issue> issues)
        {
            var s = steps[i];
            if (!PlacesNode(s)) return;
            string n = $"Step {i + 1}";
            var target = Body(s.Get("target"));
            var home = Home;
            if (target == null)
                issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: pick a target body" });
            else if (target.referenceBody == null || target.IsStar)
                issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: {target.bodyName} isn't a moon - transfers to planets aren't done yet" });
            else if (home != null && !Same(target.referenceBody, home) && !Same(target.referenceBody, FidoPlugin.Instance?.current_vessel?.VesselComponent?.mainBody))
                issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: {target.bodyName} orbits {target.referenceBody.bodyName} - only moons of the body the vessel is orbiting for now" });
            var arrival = ArrivalFor(steps, s);
            if (s.type == StepCatalog.Capture)
            {
                double alt_m = arrival.alt_km * 1000;
                if (alt_m <= 0)
                    issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: orbit altitude must be above the surface" });
                else if (target != null && target.radius + alt_m >= 0.9 * target.sphereOfInfluence)
                    issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: {arrival.alt_km:n0} km is outside (or at the edge of) {target.bodyName}'s sphere of influence ({(target.sphereOfInfluence - target.radius) / 1000:n0} km)" });
                else if (target != null && target.hasAtmosphere && alt_m < target.atmosphereDepth)
                    issues.Add(new MissionValidator.Issue { step = i, text = $"{n}: the orbit is inside {target.bodyName}'s atmosphere" });
                bool arrives = steps.Take(i).Any(x => IsTransfer(x) && x.Get("target") == s.Get("target"));
                if (!arrives)
                    issues.Add(new MissionValidator.Issue { step = i, text = $"{n}: no Transfer to {s.Get("target")} before it - the vessel has to be on its way there" });
            }
            else if (arrival.capture < 0)
            {
                if (s.GetFloat("pe_km") <= 0)
                    issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: arrival periapsis must be above the surface" });
                else if (target != null && target.hasAtmosphere && s.GetFloat("pe_km") * 1000 < target.atmosphereDepth)
                    issues.Add(new MissionValidator.Issue { step = i, text = $"{n}: arrival periapsis is inside {target.bodyName}'s atmosphere" });
            }

            int launch = steps.FindIndex(x => x.type == StepCatalog.Launch);
            if (launch >= 0 && i <= launch + 1)
                issues.Add(new MissionValidator.Issue { step = i, error = true, text = $"{n}: must come after Launch and Circularise - the transfer starts from the parking orbit" });

            var next = i + 1 < steps.Count ? steps[i + 1] : null;
            if (next == null || next.type != StepCatalog.Node)
                issues.Add(new MissionValidator.Issue { step = i, text = $"{n}: only places the node - add 'Execute node' straight after it to fly the burn" });

            if (s.type == StepCatalog.Correction)
            {
                bool has_transfer = steps.Take(i).Any(x => x.type == StepCatalog.Transfer && x.Get("target") == s.Get("target"));
                if (!has_transfer)
                    issues.Add(new MissionValidator.Issue { step = i, text = $"{n}: no Transfer to {s.Get("target")} before it - a correction trims an existing transfer" });
            }
            else if (s.type == StepCatalog.Transfer && target?.Orbit != null)
            {
                bool corrected = steps.Skip(i + 1).TakeWhile(x => x.type != StepCatalog.Capture)
                    .Any(x => x.type == StepCatalog.Correction && x.Get("target") == s.Get("target"));
                string why = target.Orbit.inclination > 1.5
                    ? $"{target.bodyName}'s orbit is tilted {target.Orbit.inclination:0.#}° - the departure burn usually misses it"
                    : arrival.plane == TransferPlanner.ArrivalPlane.Polar
                        ? $"a polar orbit at {target.bodyName} means passing over its pole, which the departure burn can't aim for"
                        : arrival.plane == TransferPlanner.ArrivalPlane.Equatorial && Math.Abs(ParkingInclination(steps, i)) > 1.5
                            ? $"the parking orbit is inclined, so an equatorial orbit at {target.bodyName} needs the arrival moved"
                            : null;
                if (why != null && !corrected)
                    issues.Add(new MissionValidator.Issue
                    {
                        step = i,
                        text = $"{n}: {why}; add Execute node, Course correction, Execute node after it",
                    });
            }

            // the capture step needs the transfer aimed at its altitude: it's taken from there, but say so if they differ
            if (IsTransfer(s) && arrival.capture >= 0 && Math.Abs(s.GetFloat("pe_km") - arrival.alt_km) > 0.5)
                issues.Add(new MissionValidator.Issue
                {
                    step = i,
                    text = $"{n}: aims for step {arrival.capture + 1}'s {arrival.alt_km:0.#} km {TransferPlanner.PlaneName(arrival.plane)} orbit (its own {s.GetFloat("pe_km"):0.#} km arrival periapsis isn't used)",
                });
        }

        /// <summary>Inclination of the plan's orbit before step i (Launch, then any Plane change steps).</summary>
        static double ParkingInclination(List<MissionStepData> steps, int i)
        {
            double inc = 0;
            for (int j = 0; j < i && j < steps.Count; j++)
            {
                if (steps[j].type == StepCatalog.Launch || steps[j].type == StepCatalog.Plane) inc = steps[j].GetFloat("inclination");
                if (steps[j].type == StepCatalog.Transfer) break;
            }
            return inc;
        }

        /// <summary>
        /// Walks the plan from step 'from' taking each step's rough Δv off the budget: is there enough
        /// for each burn, does a transfer burn run across a staging (auto-staging off), how long does it
        /// burn, and what's left to capture at the target.
        /// </summary>
        public static void CheckStages(MissionPlanData plan, StageBudget budget, List<double> dv, int from,
            List<MissionValidator.Issue> issues, double parking_period)
        {
            if (budget == null || plan == null || dv == null) return;
            var steps = plan.steps;
            bool has_burns = Enumerable.Range(Math.Max(0, from), Math.Max(0, steps.Count - from))
                .Any(i => i < dv.Count && !double.IsNaN(dv[i]) && dv[i] > 0.5);
            if (!has_burns) return;
            if (budget.Empty)
            {
                issues.Add(new MissionValidator.Issue { step = -1, error = true, text = $"No engine stages left after stage 1 ({budget.source}) for the burns after launch" });
                return;
            }
            var b = budget.Copy();
            bool auto_staging = StagingSettings.auto_staging.V;
            for (int i = Math.Max(0, from); i < steps.Count && i < dv.Count; i++)
            {
                double need = dv[i];
                if (double.IsNaN(need) || need <= 0.5) continue;
                var s = steps[i];
                string n = $"Step {i + 1}";
                double in_stage = b.LeftInStage;
                var use = b.Burn(need);
                if (use.short_of_dv)
                {
                    // rough numbers: a clear shortfall blocks, a close one warns
                    bool clear = use.left_before < 0.9 * need;
                    issues.Add(new MissionValidator.Issue
                    {
                        step = i,
                        error = clear,
                        text = $"{n}: needs ~{need:n0} m/s but only ~{use.left_before:n0} m/s is left in {b.source}",
                    });
                    return; // nothing sensible to say about the steps after
                }
                if (!PlacesNode(s)) continue;

                if (use.Crosses && !auto_staging)
                    issues.Add(new MissionValidator.Issue
                    {
                        step = i,
                        text = $"{n}: the burn runs out of stage {use.first_stage} part-way (~{in_stage:n0} of {need:n0} m/s in that stage) and auto-staging is off - turn on Auto staging in FIDO's settings or the burn stops when the stage runs dry",
                    });
                if (parking_period > 0 && use.burn_time_s > 0.15 * parking_period && s.type == StepCatalog.Transfer)
                    issues.Add(new MissionValidator.Issue
                    {
                        step = i,
                        text = $"{n}: a long burn (~{use.burn_time_s:n0} s, {use.burn_time_s / parking_period * 100:n0}% of an orbit) - expect to need a Course correction",
                    });

                // the next transfer-type step for the same target (if any) uses this budget too; only warn about capture after the last
                bool last_for_target = IsTransfer(s) && !steps.Skip(i + 1).Any(x => PlacesNode(x) && x.Get("target") == s.Get("target"));
                if (last_for_target)
                {
                    double capture = CaptureEstimate(s.Get("target"), s.GetFloat("pe_km") * 1000);
                    if (!double.IsNaN(capture) && use.left_after < capture)
                        issues.Add(new MissionValidator.Issue
                        {
                            step = i,
                            text = $"{n}: ~{use.left_after:n0} m/s left on arrival; capturing into a {s.GetFloat("pe_km"):0.#} km orbit at {s.Get("target")} needs ~{capture:n0} m/s",
                        });
                }
            }
        }

        /// <summary>Capture Δv into a circular orbit at alt_m above the target, after a Hohmann from a low orbit of its parent.</summary>
        public static double CaptureEstimate(string target_name, double alt_m)
        {
            var target = Body(target_name);
            var parent = target?.referenceBody;
            if (target?.Orbit == null || parent == null) return double.NaN;
            double r1 = parent.radius + (parent.hasAtmosphere ? parent.atmosphereDepth : 0) + 20000;
            double v_inf = TransferPlanner.ArrivalVInf(parent.gravParameter, r1, target.Orbit.semiMajorAxis);
            return TransferPlanner.CaptureDv(target.gravParameter, target.radius + alt_m, v_inf);
        }

        /// <summary>Rough Δv of a transfer-type step from a circular parking orbit at alt_km, inclination inc (home body).</summary>
        public static double Estimate(List<MissionStepData> steps, MissionStepData s, double mu, double radius, double alt_km, double inc)
        {
            var target = Body(s.Get("target"));
            if (target?.Orbit == null) return double.NaN;
            if (s.type == StepCatalog.Capture) return CaptureEstimate(s.Get("target"), s.GetFloat("alt_km") * 1000);
            double rel_inc = double.IsNaN(inc) ? target.Orbit.inclination : Math.Abs(target.Orbit.inclination - inc);
            if (s.type == StepCatalog.Correction)
            {
                // passing over the pole: the miss moves by about the impact parameter, a few tens of m/s more
                bool polar = ArrivalFor(steps, s).plane == TransferPlanner.ArrivalPlane.Polar;
                return CorrectionEstimate(rel_inc) + (polar ? 30 : 0);
            }
            if (double.IsNaN(alt_km)) return double.NaN;
            return TransferPlanner.DepartureDv(mu, radius + alt_km * 1000, target.Orbit.semiMajorAxis);
        }
    }

    // ==================================================================== runners

    /// <summary>
    /// Transfer / Course correction: works out the burn (TransferPlanner), checks the stages can fly
    /// it, puts the node on the plan and checks the game's own prediction agrees. Doesn't burn - an
    /// "Execute node" step after it does.
    /// </summary>
    abstract class TransferNodeRunner : StepRunner
    {
        protected abstract bool IsCorrection { get; }

        // the game's node frame signs for normal / radial, learned from its own prediction
        // (same approach as ConstellationPilot.polar_normal_sign)
        static double normal_sign = 1, radial_sign = 1;

        enum Phase { Idle, WaitNode, Verify, Done, Failed }
        Phase phase;
        TransferPlanner.Plan plan;
        double plan_made_at = -1;
        float plan_made_real = -1000;
        double plan_lead = TransferPlanner.DefaultLead;
        bool in_soi;
        ManeuverNodeData node;
        int frames, attempt;
        float verify_until;
        readonly ManeuverCreator creator = new();
        string advice;
        float advice_at = -10;

        public override string Tab => "node";

        CelestialBodyComponent Target => TransferChecks.Body(step.Get("target"));
        TransferChecks.Arrival Arrival => TransferChecks.ArrivalFor(mission?.steps, step);
        double PeAlt => Arrival.alt_km * 1000.0;
        TransferPlanner.ArrivalPlane Plane => Arrival.plane;
        string planned_for = "";

        string Situation(out VesselComponent vessel, out CelestialBodyComponent target)
        {
            vessel = Vessel?.VesselComponent;
            target = Target;
            if (vessel?.Orbit == null) return "No vessel";
            if (target?.Orbit == null) return $"Unknown target '{step.Get("target")}'";
            var here = vessel.mainBody;
            if (here?.bodyName == target.bodyName) return $"Already in {target.bodyName}'s sphere of influence";
            if (target.referenceBody?.bodyName != here?.bodyName)
                return $"The vessel is orbiting {here?.bodyName}; {target.bodyName} orbits {target.referenceBody?.bodyName}";
            if (!IsCorrection)
            {
                if (vessel.Orbit.eccentricity >= 1) return "The vessel isn't in a closed orbit";
                double pe_alt = vessel.Orbit.PeriapsisArl;
                if (here.hasAtmosphere && pe_alt < here.atmosphereDepth)
                    return $"Periapsis {pe_alt / 1000:n0} km is in the atmosphere - circularise first";
            }
            return null;
        }

        TransferPlanner.Plan MakePlan(bool fresh)
        {
            double now = GeneralTools.Current_UT;
            // a plan stays good until its burn gets close, for two minutes at most (real time, so time
            // warp doesn't replan every frame); "nothing to correct" for half a minute
            float age = Time.unscaledTime - plan_made_real;
            var arrival = Arrival;
            string wanted = $"{arrival.alt_km}|{arrival.plane}";
            bool fresh_enough = plan != null && plan_made_at > 0 && wanted == planned_for &&
                                (plan.ok && plan.Dv < 0.05 ? age < 30 : age < 120 && plan.UT - now > plan_lead * 0.8);
            if (!fresh && fresh_enough) return plan;
            planned_for = wanted;
            in_soi = false;
            if (IsCorrection && Vessel?.VesselComponent?.mainBody?.bodyName == step.Get("target"))
            {
                // already there: nothing left to correct (a capture step will come later)
                in_soi = true;
                plan = new TransferPlanner.Plan { ok = true, encounter = true };
                plan_made_at = now;
                plan_made_real = Time.unscaledTime;
                return plan;
            }
            string problem = Situation(out var vessel, out var target);
            if (problem != null)
            {
                plan = new TransferPlanner.Plan { problem = problem };
                plan_made_at = -1;
                return plan;
            }
            var orbit = vessel.Orbit;
            var r = orbit.GetRelativePositionAtUTZup(now);
            var v = orbit.GetOrbitalVelocityAtUTZup(now);
            var moon = new TransferPlanner.Moon
            {
                r = target.Orbit.GetRelativePositionAtUTZup(now),
                v = target.Orbit.GetOrbitalVelocityAtUTZup(now),
                mu = target.gravParameter,
                radius = target.radius,
                soi = target.sphereOfInfluence,
            };
            double mu = vessel.mainBody.gravParameter;
            // time to turn and warp, and to light up half the burn before the node
            double lead = Math.Max(TransferPlanner.DefaultLead, BurnTime(EstimateDv()) / 2 + 90);
            plan_lead = lead;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            plan = IsCorrection
                ? TransferPlanner.Correct(r, v, mu, now, moon, PeAlt, lead, Plane)
                : TransferPlanner.Compute(r, v, mu, now, moon, PeAlt, lead, Plane);
            plan_made_at = now;
            plan_made_real = Time.unscaledTime;
            FidoPlugin.logger.LogInfo(
                $"[Mission] {(IsCorrection ? "correction" : "transfer")} to {target.bodyName}: ok={plan.ok} encounter={plan.encounter} '{plan.problem}' " +
                $"T+{plan.UT - now:n0}s prograde {plan.prograde:n1} normal {plan.normal:n1} radial {plan.radial:n1} (|Δv| {plan.Dv:n1}) " +
                $"coast {plan.tof:n0}s miss {plan.miss_m / 1000:n1} km (aim {plan.aim_m / 1000:n1}, off-plane {plan.off_plane_m / 1000:n0}) " +
                $"Pe {plan.pe_alt_m / 1000:n1} km (want {PeAlt / 1000:n1}) inc {plan.arrival_inc:n1}° (want {TransferPlanner.PlaneName(Plane)}) " +
                $"v_inf {plan.v_inf:n0} capture {plan.capture_dv:n0} correction~{plan.correction_dv:n0} " +
                $"| moon at {moon.r.magnitude / 1000:n0} km (sma {target.Orbit.semiMajorAxis / 1000:n0}) | {sw.ElapsedMilliseconds} ms");
            return plan;
        }

        double EstimateDv()
        {
            if (IsCorrection) return 50;
            var vessel = Vessel?.VesselComponent;
            var target = Target;
            if (vessel?.Orbit == null || target?.Orbit == null) return 0;
            return TransferPlanner.DepartureDv(vessel.mainBody.gravParameter, vessel.Orbit.semiMajorAxis, target.Orbit.semiMajorAxis);
        }

        internal static double BurnTime(double dv)
        {
            var budget = StageBudget.Live(Vessel?.VesselComponent);
            return budget == null || budget.Empty ? 0 : budget.Burn(dv).burn_time_s;
        }

        /// <summary>Errors that stop the step, from the real orbit and the vessel's stages as they are now.</summary>
        public override string Check()
        {
            var p = MakePlan(false);
            if (!p.ok) return p.problem;
            if (p.Dv < 0.05) return null; // a correction that isn't needed
            var budget = StageBudget.Live(Vessel?.VesselComponent);
            if (budget == null) return "No Δv data for the vessel yet";
            if (budget.Empty) return "No engine stages with fuel left";
            var use = budget.Copy().Burn(p.Dv);
            if (use.short_of_dv)
                return $"The burn needs {p.Dv:n0} m/s but the vessel has {use.left_before:n0} m/s (vac)";
            if (use.Crosses && !StagingSettings.auto_staging.V)
                return $"The burn needs {p.Dv:n0} m/s but stage {use.first_stage} has {budget.LeftInStage:n0} m/s and auto-staging is off - " +
                       "turn on Auto staging in FIDO's settings so the burn carries on into the next stage";
            return null;
        }

        /// <summary>Warnings worth seeing before Go (shown on the Mission tab, never block).</summary>
        public override string Advice()
        {
            if (Time.unscaledTime - advice_at < 2) return advice;
            advice_at = Time.unscaledTime;
            try { advice = MakeAdvice(); }
            catch (Exception e) { advice = "Couldn't plan: " + e.Message; }
            return advice;
        }

        string MakeAdvice()
        {
            var p = MakePlan(false);
            if (!p.ok) return null;
            var lines = new List<string>();
            var vessel = Vessel?.VesselComponent;
            string target = step.Get("target");
            if (in_soi) return $"Already inside {target}'s sphere of influence - nothing to correct";
            var arrival = Arrival;
            if (p.Dv < 0.05) return $"On course for {arrival.Describe(target)}: periapsis ~{p.pe_alt_m / 1000:n0} km, inclination ~{p.arrival_inc:n0}° - no correction needed";
            lines.Add($"Aiming for {arrival.Describe(target)}");
            lines.Add($"Burn {p.Dv:n0} m/s in {FormatTime(p.UT - GeneralTools.Current_UT)}, arrive {FormatTime(p.UT + p.tof - GeneralTools.Current_UT)} from now" +
                      (p.encounter ? $", periapsis ~{p.pe_alt_m / 1000:n0} km, inclination ~{p.arrival_inc:n0}°" : ""));
            if (!p.encounter && !double.IsNaN(p.correction_dv))
                lines.Add("⚠ " + MissReason(p, target) + $" - a Course correction on the way (~{p.correction_dv:n0} m/s) brings it in");
            var budget = StageBudget.Live(vessel);
            if (budget != null && !budget.Empty)
            {
                var use = budget.Copy().Burn(p.Dv);
                double period = vessel?.Orbit?.period ?? 0;
                if (!IsCorrection && period > 0 && use.burn_time_s > 0.15 * period)
                    lines.Add($"⚠ Long burn: ~{use.burn_time_s:n0} s ({use.burn_time_s / period * 100:n0}% of an orbit) - expect to need a Course correction");
                double after = use.left_after - (double.IsNaN(p.correction_dv) ? 0 : p.correction_dv);
                if (arrival.capture < 0 && p.capture_dv > 0 && after < p.capture_dv)
                    lines.Add($"⚠ ~{Math.Max(0, after):n0} m/s left on arrival; capturing into a {PeAlt / 1000:n0} km orbit needs ~{p.capture_dv:n0} m/s");
                if (use.Crosses && StagingSettings.auto_staging.V)
                    lines.Add($"Stages during the burn (stage {use.first_stage} → {use.last_stage}) - auto-staging is on");
            }
            return string.Join("\n", lines);
        }

        static string MissReason(TransferPlanner.Plan p, string target) =>
            !p.pe_ok
                ? $"{target} is {Math.Abs(p.off_plane_m) / 1000:n0} km off this orbit's plane at arrival"
                : $"this path arrives at {p.arrival_inc:n0}° to {target}'s equator, not {TransferPlanner.PlaneName(p.plane)}";

        internal static string FormatTime(double s)
        {
            if (s < 0) s = 0;
            var t = TimeSpan.FromSeconds(s);
            if (t.TotalHours >= 6) return $"{(int)(t.TotalHours / 6)}d {(int)(t.TotalHours % 6)}h"; // Kerbin days are 6 h
            if (t.TotalHours >= 1) return $"{(int)t.TotalHours}h {t.Minutes}m";
            return $"{t.Minutes}m {t.Seconds}s";
        }

        public override void Start()
        {
            attempt = 0;
            var p = MakePlan(false);
            if (!p.ok)
            {
                phase = Phase.Failed;
                status = p.problem;
                return;
            }
            if (in_soi)
            {
                phase = Phase.Done;
                status = $"Already inside {step.Get("target")}'s sphere of influence";
                return;
            }
            if (p.Dv < 0.05)
            {
                phase = Phase.Done;
                status = $"No correction needed - periapsis at {step.Get("target")} ~{p.pe_alt_m / 1000:n0} km";
                return;
            }
            PlaceNode();
        }

        // attempt 0..3: the learned signs, then normal flipped, radial flipped, both
        double NormalSign => normal_sign * ((attempt & 1) == 1 ? -1 : 1);
        double RadialSign => radial_sign * ((attempt & 2) == 2 ? -1 : 1);

        void PlaceNode()
        {
            node = null;
            frames = 0;
            phase = Phase.WaitNode;
            status = "Placing the node";
            creator.Update();
            creator.RemoveAllNodesThenCreate(plan.UT, plan.prograde, n => node = n, plan.normal * NormalSign, plan.radial * RadialSign);
        }

        bool CanFlip(int next) =>
            next < 4 && ((next & 1) == 0 || Math.Abs(plan.normal) > 1) && ((next & 2) == 0 || Math.Abs(plan.radial) > 1);

        public override StepResult Update()
        {
            switch (phase)
            {
                case Phase.Done: return StepResult.Done;
                case Phase.Failed: return StepResult.Failed;
                case Phase.WaitNode:
                    if (node == null) return StepResult.Running;
                    phase = Phase.Verify;
                    frames = 0;
                    verify_until = Time.unscaledTime + 3;
                    status = "Checking the game's prediction";
                    return StepResult.Running;
                case Phase.Verify:
                    return Verify();
            }
            return StepResult.Running;
        }

        StepResult Verify()
        {
            if (++frames < 10) return StepResult.Running;
            var target = Target;
            string target_name = step.Get("target");
            bool read = GamePrediction(target, out bool encounter, out double game_pe, out double game_inc);
            bool timed_out = Time.unscaledTime > verify_until;
            double tol = Math.Max(30000, 3 * TransferPlanner.PeTolerance(PeAlt));
            // the plane only tells the signs apart when one was aimed for (a mirrored normal keeps the periapsis)
            bool plane_agrees = plan.plane == TransferPlanner.ArrivalPlane.Any || double.IsNaN(plan.arrival_inc) ||
                                Math.Abs(game_inc - plan.arrival_inc) < 10;
            bool agrees = encounter && Math.Abs(game_pe - plan.pe_alt_m) < tol && plane_agrees;

            if (plan.encounter && agrees)
            {
                normal_sign = NormalSign;
                radial_sign = RadialSign;
                status = $"Node placed: {plan.Dv:n0} m/s, game predicts periapsis {game_pe / 1000:n0} km, inclination {game_inc:n0}° at {target_name}";
                FidoPlugin.logger.LogInfo($"[Mission] {status} (planned {plan.pe_alt_m / 1000:n1} km {plan.arrival_inc:n1}°, signs n{normal_sign:+0;-0} r{radial_sign:+0;-0})");
                return StepResult.Done;
            }
            if (!plan.encounter && read && !timed_out && !encounter) return StepResult.Running; // give a late encounter a chance
            if (!timed_out && (!read || plan.encounter)) return StepResult.Running;

            if (!plan.encounter)
            {
                // a departure that needs the correction: no encounter is what was planned
                status = encounter
                    ? $"Node placed: {plan.Dv:n0} m/s, game predicts periapsis {game_pe / 1000:n0} km at {target_name}"
                    : $"Node placed: {plan.Dv:n0} m/s - {MissReason(plan, target_name)}; Course correction (~{plan.correction_dv:n0} m/s) brings it in";
                FidoPlugin.logger.LogInfo($"[Mission] {status}");
                return StepResult.Done;
            }
            if (!read)
            {
                status = $"Node placed: {plan.Dv:n0} m/s for periapsis ~{plan.pe_alt_m / 1000:n0} km - couldn't read the game's prediction, check it in map view";
                FidoPlugin.logger.LogWarning("[Mission] " + status);
                return StepResult.Done;
            }
            FidoPlugin.logger.LogWarning($"[Mission] game prediction disagrees (attempt {attempt}): " +
                                          (encounter ? $"Pe {game_pe / 1000:n1} km inc {game_inc:n1}°" : "no encounter") + $", planned {plan.pe_alt_m / 1000:n1} km {plan.arrival_inc:n1}°");
            int next = attempt + 1;
            while (next < 4 && !CanFlip(next)) next++;
            if (next < 4)
            {
                attempt = next;
                PlaceNode();
                return StepResult.Running;
            }
            status = (encounter ? $"Game predicts periapsis {game_pe / 1000:n0} km, inclination {game_inc:n0}°" : $"Game predicts no encounter with {target_name}") +
                     $" (planned {plan.pe_alt_m / 1000:n0} km, {plan.arrival_inc:n0}°) - the node is left on the plan; check it in map view";
            return StepResult.Failed;
        }

        /// <summary>Reads the patched-conic trajectory after the node: is there a patch around the target, and its periapsis altitude.</summary>
        bool GamePrediction(CelestialBodyComponent target, out bool encounter, out double pe_alt, out double inc)
        {
            encounter = false;
            pe_alt = double.NaN;
            inc = double.NaN;
            try
            {
                var trajectory = Vessel?.VesselComponent?.Orbiter?.ManeuverPlanSolver?.ManeuverTrajectory;
                if (trajectory == null || trajectory.Count == 0 || node == null) return false;
                var after = trajectory.OfType<PatchedConicsOrbit>().Where(p => p.StartUT >= node.Time - 5).ToList();
                if (after.Count == 0) return false;
                var at = after.FirstOrDefault(p => p.referenceBody?.bodyName == target?.bodyName);
                if (at == null) return true;
                encounter = true;
                pe_alt = at.PeriapsisArl;
                inc = at.inclination;
                if (frames % 30 == 10)
                    FidoPlugin.logger.LogInfo($"[Mission] game patch at {target.bodyName}: Periapsis {at.Periapsis / 1000:n1} km, PeriapsisArl {at.PeriapsisArl / 1000:n1} km, ecc {at.eccentricity:n3}, inc {at.inclination:n1}°");
                return true;
            }
            catch (Exception e)
            {
                FidoPlugin.logger.LogWarning("[Mission] reading the game's prediction: " + e.Message);
                return false;
            }
        }

        public override void Stop()
        {
            phase = Phase.Idle;
        }
    }

    class TransferRunner : TransferNodeRunner
    {
        protected override bool IsCorrection => false;
    }

    class CorrectionRunner : TransferNodeRunner
    {
        protected override bool IsCorrection => true;
    }
    /// <summary>
    /// Capture at moon: once inside the target's sphere of influence (time-warping there first if the
    /// vessel is still on its way), places the capture node at periapsis - a circular orbit at the
    /// step's altitude. If the arrival periapsis isn't at that altitude the burn makes an ellipse that
    /// reaches it instead, and running Capture again (after Execute node) circularises there. The
    /// orbit's plane is set by the Transfer and Course correction steps, which aim for this step's
    /// altitude and plane; this step only says if it came out wrong. Doesn't burn - Execute node does.
    /// </summary>
    class CaptureRunner : StepRunner
    {
        enum Phase { Idle, Warp, Settle, WaitNode, Verify, Done, Failed }
        Phase phase;
        TransferPlanner.CapturePlan cplan;
        double soi_ut = double.NaN;
        float settle_until, verify_until, warp_again_at;
        int frames;
        ManeuverNodeData node;
        readonly ManeuverCreator creator = new();
        string advice;
        float advice_at = -10;

        public override string Tab => "node";

        CelestialBodyComponent Target => TransferChecks.Body(step.Get("target"));
        double Alt => step.GetFloat("alt_km") * 1000.0;
        TransferPlanner.ArrivalPlane Plane => TransferPlanner.ParsePlane(step.Get("orbit"));

        static bool InSoi(VesselComponent v, CelestialBodyComponent t) => v?.mainBody != null && t != null && v.mainBody.bodyName == t.bodyName;

        /// <summary>The patch around the target on the vessel's path as it is now (no nodes), or null.</summary>
        static PatchedConicsOrbit EncounterPatch(VesselComponent vessel, CelestialBodyComponent target)
        {
            var trajectory = vessel?.Orbiter?.PatchedConicSolver?.CurrentTrajectory;
            if (trajectory == null || target == null) return null;
            return trajectory.OfType<PatchedConicsOrbit>().FirstOrDefault(p => p.referenceBody?.bodyName == target.bodyName);
        }

        TransferPlanner.CapturePlan MakePlan()
        {
            var vessel = Vessel?.VesselComponent;
            var target = Target;
            if (vessel?.Orbit == null) return new TransferPlanner.CapturePlan { problem = "No vessel" };
            if (target == null) return new TransferPlanner.CapturePlan { problem = $"Unknown target '{step.Get("target")}'" };
            double now = GeneralTools.Current_UT;
            PatchedConicsOrbit orbit;
            double t0;
            if (InSoi(vessel, target))
            {
                orbit = vessel.Orbit;
                t0 = now;
                soi_ut = double.NaN;
            }
            else
            {
                orbit = EncounterPatch(vessel, target);
                if (orbit == null)
                    return new TransferPlanner.CapturePlan { problem = $"No encounter with {target.bodyName} on the current path - fly the Transfer (and Course correction) first" };
                soi_ut = orbit.StartUT;
                t0 = Math.Max(now, orbit.StartUT + 1);
            }
            var r = orbit.GetRelativePositionAtUTZup(t0);
            var v = orbit.GetOrbitalVelocityAtUTZup(t0);
            double lead = InSoi(vessel, target) ? TransferPlanner.DefaultLead : 0;
            var c = TransferPlanner.Capture(r, v, target.gravParameter, target.radius, target.sphereOfInfluence, t0, Alt, lead);
            if (c.ok && !c.done && InSoi(vessel, target))
            {
                // time to turn and to light up half the burn before the node
                double need = Math.Max(TransferPlanner.DefaultLead, TransferNodeRunner.BurnTime(Math.Abs(c.prograde)) / 2 + 90);
                if (need > lead) c = TransferPlanner.Capture(r, v, target.gravParameter, target.radius, target.sphereOfInfluence, t0, Alt, need);
            }
            return c;
        }

        public override string Check()
        {
            var c = MakePlan();
            if (!c.ok) return c.problem;
            if (c.done) return null;
            var budget = StageBudget.Live(Vessel?.VesselComponent);
            if (budget == null) return "No Δv data for the vessel yet";
            if (budget.Empty) return "No engine stages with fuel left";
            double dv = Math.Abs(c.prograde);
            var use = budget.Copy().Burn(dv);
            if (use.short_of_dv)
                return $"The capture burn needs {dv:n0} m/s but the vessel has {use.left_before:n0} m/s (vac)";
            if (use.Crosses && !StagingSettings.auto_staging.V)
                return $"The capture burn needs {dv:n0} m/s but stage {use.first_stage} has {budget.LeftInStage:n0} m/s and auto-staging is off - " +
                       "turn on Auto staging in FIDO's settings so the burn carries on into the next stage";
            return null;
        }

        public override string Advice()
        {
            if (Time.unscaledTime - advice_at < 2) return advice;
            advice_at = Time.unscaledTime;
            try { advice = MakeAdvice(); }
            catch (Exception e) { advice = "Couldn't plan the capture: " + e.Message; }
            return advice;
        }

        string MakeAdvice()
        {
            var c = MakePlan();
            if (!c.ok) return null;
            string target = step.Get("target");
            var vessel = Vessel?.VesselComponent;
            var lines = new List<string>();
            if (c.done) lines.Add($"Already in a {c.pe_alt_m / 1000:n0} x {c.ap_alt_m / 1000:n0} km orbit of {target} - nothing to do");
            else
            {
                double now = GeneralTools.Current_UT;
                if (!double.IsNaN(soi_ut) && !InSoi(vessel, Target))
                    lines.Add($"Warps to {target}'s sphere of influence first ({TransferNodeRunner.FormatTime(soi_ut - now)})");
                string where = c.at_apoapsis ? "apoapsis" : "periapsis";
                lines.Add($"Burn {Math.Abs(c.prograde):n0} m/s {(c.prograde < 0 ? "retrograde" : "prograde")} at {where} ({c.burn_alt_m / 1000:n0} km) in {TransferNodeRunner.FormatTime(c.UT - now)}" +
                          (c.circular
                              ? $" → circular {c.burn_alt_m / 1000:n0} km orbit"
                              : $" → {Math.Min(c.burn_alt_m, c.other_alt_m) / 1000:n0} x {Math.Max(c.burn_alt_m, c.other_alt_m) / 1000:n0} km orbit; run Capture again after Execute node to circularise at {Alt / 1000:n0} km"));
                var budget = StageBudget.Live(vessel);
                if (budget != null && !budget.Empty)
                {
                    var use = budget.Copy().Burn(Math.Abs(c.prograde));
                    if (use.Crosses && StagingSettings.auto_staging.V)
                        lines.Add($"Stages during the burn (stage {use.first_stage} → {use.last_stage}) - auto-staging is on");
                }
            }
            var plane = Plane;
            double off = plane == TransferPlanner.ArrivalPlane.Equatorial ? c.inc : plane == TransferPlanner.ArrivalPlane.Polar ? Math.Abs(c.inc - 90) : 0;
            if (off > 5)
            {
                var body = Target;
                double v_circ = body != null ? Math.Sqrt(body.gravParameter / (body.radius + Alt)) : 0;
                double fix = OrbitMath.PlaneChangeDv(v_circ, off);
                lines.Add($"⚠ The orbit will be inclined {c.inc:n0}°, not {TransferPlanner.PlaneName(plane)} - a Plane change step after it can fix that (~{fix:n0} m/s); " +
                          "a Course correction on the way there costs much less");
            }
            return string.Join("\n", lines);
        }

        public override void Start()
        {
            cplan = MakePlan();
            if (!cplan.ok) { phase = Phase.Failed; status = cplan.problem; return; }
            if (cplan.done) { phase = Phase.Done; status = $"Already in a {cplan.pe_alt_m / 1000:n0} x {cplan.ap_alt_m / 1000:n0} km orbit"; return; }
            var vessel = Vessel?.VesselComponent;
            if (!InSoi(vessel, Target))
            {
                phase = Phase.Warp;
                warp_again_at = Time.unscaledTime + 3;
                TimeWarpTools.KspWarpTo(soi_ut + 5);
                status = $"Coasting to {step.Get("target")}";
                FidoPlugin.logger.LogInfo($"[Mission] capture: warping to {step.Get("target")}'s SOI at T+{soi_ut - GeneralTools.Current_UT:n0}s");
                return;
            }
            PlaceNode();
        }

        void PlaceNode()
        {
            cplan = MakePlan();
            if (!cplan.ok) { phase = Phase.Failed; status = cplan.problem; return; }
            if (cplan.done) { phase = Phase.Done; status = $"Already in a {cplan.pe_alt_m / 1000:n0} x {cplan.ap_alt_m / 1000:n0} km orbit"; return; }
            FidoPlugin.logger.LogInfo(
                $"[Mission] capture at {step.Get("target")}: {cplan.prograde:n1} m/s at {(cplan.at_apoapsis ? "Ap" : "Pe")} T+{cplan.UT - GeneralTools.Current_UT:n0}s, " +
                $"orbit now Pe {cplan.pe_alt_m / 1000:n1} km Ap {cplan.ap_alt_m / 1000:n1} km inc {cplan.inc:n1}° -> {cplan.burn_alt_m / 1000:n1} x {cplan.other_alt_m / 1000:n1} km (want {Alt / 1000:n1} {TransferPlanner.PlaneName(Plane)})");
            node = null;
            frames = 0;
            phase = Phase.WaitNode;
            status = "Placing the capture node";
            creator.Update();
            creator.RemoveAllNodesThenCreate(cplan.UT, cplan.prograde, n => node = n);
        }

        public override StepResult Update()
        {
            switch (phase)
            {
                case Phase.Done: return StepResult.Done;
                case Phase.Failed: return StepResult.Failed;
                case Phase.Warp:
                {
                    var vessel = Vessel?.VesselComponent;
                    double now = GeneralTools.Current_UT;
                    if (InSoi(vessel, Target))
                    {
                        phase = Phase.Settle;
                        settle_until = Time.unscaledTime + 2;
                        status = $"In {step.Get("target")}'s sphere of influence";
                        return StepResult.Running;
                    }
                    if (!double.IsNaN(soi_ut) && now > soi_ut + 120)
                    {
                        status = $"Didn't enter {step.Get("target")}'s sphere of influence when expected - check the path in map view";
                        phase = Phase.Failed;
                        return StepResult.Failed;
                    }
                    // warp stopped early (the game drops out of warp for some events): ask again
                    if (TimeWarpTools.CurrentRateIndex == 0 && Time.unscaledTime > warp_again_at)
                    {
                        warp_again_at = Time.unscaledTime + 3;
                        var patch = EncounterPatch(vessel, Target);
                        if (patch != null) soi_ut = patch.StartUT;
                        TimeWarpTools.KspWarpTo(soi_ut + 5);
                    }
                    status = $"Coasting to {step.Get("target")}: {TransferNodeRunner.FormatTime(soi_ut - now)}";
                    return StepResult.Running;
                }
                case Phase.Settle:
                    if (Time.unscaledTime < settle_until || TimeWarpTools.CurrentRateIndex != 0) return StepResult.Running;
                    PlaceNode();
                    return phase == Phase.Failed ? StepResult.Failed : phase == Phase.Done ? StepResult.Done : StepResult.Running;
                case Phase.WaitNode:
                    if (node == null) return StepResult.Running;
                    phase = Phase.Verify;
                    frames = 0;
                    verify_until = Time.unscaledTime + 3;
                    status = "Checking the game's prediction";
                    return StepResult.Running;
                case Phase.Verify:
                    return Verify();
            }
            return StepResult.Running;
        }

        StepResult Verify()
        {
            if (++frames < 10) return StepResult.Running;
            PatchedConicsOrbit after = null;
            try
            {
                var trajectory = Vessel?.VesselComponent?.Orbiter?.ManeuverPlanSolver?.ManeuverTrajectory;
                after = trajectory?.OfType<PatchedConicsOrbit>().FirstOrDefault(p => node != null && p.StartUT >= node.Time - 5);
            }
            catch (Exception e) { FidoPlugin.logger.LogWarning("[Mission] reading the game's prediction: " + e.Message); }
            bool timed_out = Time.unscaledTime > verify_until;
            if (after == null && !timed_out) return StepResult.Running;
            string target = step.Get("target");
            if (after == null)
                status = $"Capture node placed: {Math.Abs(cplan.prograde):n0} m/s - couldn't read the game's prediction, check it in map view";
            else if (after.referenceBody?.bodyName != target || after.eccentricity >= 1)
                status = $"Capture node placed: {Math.Abs(cplan.prograde):n0} m/s, but the game predicts the vessel still leaves {target} - check it in map view";
            else
                status = $"Capture node placed: {Math.Abs(cplan.prograde):n0} m/s, game predicts a {after.PeriapsisArl / 1000:n0} x {after.ApoapsisArl / 1000:n0} km orbit, inclination {after.inclination:n0}°";
            FidoPlugin.logger.LogInfo("[Mission] " + status);
            phase = Phase.Done;
            return StepResult.Done;
        }

        public override void Stop()
        {
            if (phase == Phase.Warp && TimeWarpTools.CurrentRateIndex != 0) TimeWarpTools.SetRateIndex(0, false);
            phase = Phase.Idle;
        }
    }
}
