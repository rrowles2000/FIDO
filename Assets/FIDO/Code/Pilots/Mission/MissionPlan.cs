using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Fido.Constellation;
using Newtonsoft.Json;

namespace Fido.Mission
{
    /// <summary>One step of a mission plan, as stored on the craft.</summary>
    public class MissionStepData
    {
        public string type = "";
        // run straight on from the previous step instead of stopping on this step's settings
        public bool auto;
        public Dictionary<string, string> p = new();
        // "", "done", "skipped", "failed"
        public string state = "";
        public string note = "";
        public string title = "";                       // user's name for the step; "" = the generated summary
        public List<PartActionData> actions = new();    // "Part actions" steps only

        // Computed properties must stay out of the JSON: on load Json.NET populates a get-only
        // reference property in place, which appended the catalog's parameters again on every
        // save/reload (Launch settings repeated in the editor) and could overwrite the current step.
        [JsonIgnore]
        public StepType Type => StepCatalog.Get(type);

        public string Get(string key)
        {
            if (p.TryGetValue(key, out var v)) return v;
            return Type?.Param(key)?.def ?? "";
        }

        public float GetFloat(string key) =>
            float.TryParse(Get(key), NumberStyles.Float, CultureInfo.InvariantCulture, out var f) ? f : 0;

        public int GetInt(string key) =>
            int.TryParse(Get(key), NumberStyles.Integer, CultureInfo.InvariantCulture, out var i) ? i : 0;

        public bool GetBool(string key) => Get(key) == "true";

        public void Set(string key, float v) => p[key] = v.ToString("0.###", CultureInfo.InvariantCulture);
        public void Set(string key, int v) => p[key] = v.ToString(CultureInfo.InvariantCulture);
        public void Set(string key, bool v) => p[key] = v ? "true" : "false";

        [JsonIgnore]
        public string Summary => !string.IsNullOrWhiteSpace(title) ? title : DefaultSummary;
        [JsonIgnore]
        public string DefaultSummary => Type?.summary(this) ?? $"Unknown step '{type}'";
        [JsonIgnore]
        public string Label => Type?.label ?? type;
    }

    /// <summary>
    /// A craft's mission: the step list plus where it has got to. Lives as JSON on one command part
    /// (Module_FidoMission), so it is saved with the craft in the VAB and with the vessel in saves.
    /// </summary>
    public class MissionPlanData
    {
        public int version = 1;
        public List<MissionStepData> steps = new();
        // the step in progress, or the next one to run
        public int current;
        // "", "running", "paused", "done"
        public string status = "";

        [JsonIgnore]
        public bool IsEmpty => steps.Count == 0;
        [JsonIgnore]
        public bool Started => current > 0 || status != "" || steps.Any(s => s.state != "");
        [JsonIgnore]
        public MissionStepData Current => current >= 0 && current < steps.Count ? steps[current] : null;

        public static MissionPlanData FromJson(string json)
        {
            if (string.IsNullOrEmpty(json)) return null;
            try { return JsonConvert.DeserializeObject<MissionPlanData>(json); }
            catch (Exception e)
            {
                FidoPlugin.logger?.LogError("[Mission] unreadable plan: " + e.Message);
                return null;
            }
        }

        public string ToJson() => JsonConvert.SerializeObject(this, Formatting.None);

        public void ResetProgress()
        {
            current = 0;
            status = "";
            foreach (var s in steps)
            {
                s.state = "";
                s.note = "";
            }
        }
    }

    public enum ParamKind { Float, Int, Bool, Choice }

    public class ParamDef
    {
        public string key, label, def, hint;
        public ParamKind kind;
        public Func<List<string>> choices; // Choice only

        public ParamDef(string key, string label, ParamKind kind, string def, string hint = null)
        {
            this.key = key;
            this.label = label;
            this.kind = kind;
            this.def = def;
            this.hint = hint;
        }
    }

    /// <summary>A named set of parameter values the editor can apply in one go (e.g. "South polar").</summary>
    public class StepPreset
    {
        public string label;
        public Dictionary<string, string> values;

        public StepPreset(string label, Dictionary<string, string> values)
        {
            this.label = label;
            this.values = values;
        }

        public bool Matches(MissionStepData s) => values.All(kv => s.Get(kv.Key) == kv.Value);
    }

    public class StepType
    {
        public string id, label;
        public bool default_auto;
        public List<ParamDef> parameters = new();
        public List<StepPreset> presets = new();
        public Func<MissionStepData, string> summary;
        public Func<StepRunner> create;

        public ParamDef Param(string key) => parameters.FirstOrDefault(p => p.key == key);

        public MissionStepData NewStep()
        {
            var step = new MissionStepData { type = id, auto = default_auto };
            foreach (var p in parameters) step.p[p.key] = p.def;
            return step;
        }
    }

    public static class StepCatalog
    {
        public const string Quicksave = "quicksave";
        public const string Launch = "launch";
        public const string Circularise = "circularise";
        public const string Fairings = "fairings";
        public const string Plane = "plane";
        public const string Altitude = "altitude";
        public const string Deploy = "deploy";
        public const string Node = "node";
        public const string PartActions = "part_actions";
        public const string Transfer = "transfer";
        public const string Correction = "correction";
        public const string Capture = "capture";

        static string Km(MissionStepData s, string key) => $"{s.GetFloat(key):0.#} km";

        public static readonly List<StepType> All = new()
        {
            new StepType
            {
                id = Quicksave, label = "Quicksave", default_auto = true,
                summary = s => "Quicksave",
                create = () => new QuicksaveRunner(),
            },
            new StepType
            {
                id = Launch, label = "Launch",
                parameters =
                {
                    new ParamDef("ap_km", "Target apoapsis (km)", ParamKind.Float, "85"),
                    new ParamDef("inclination", "Inclination (°)", ParamKind.Float, "0", "90 = polar"),
                    new ParamDef("south", "Launch southward", ParamKind.Bool, "false"),
                    new ParamDef("optimiser", "Burn stage 1's leftover at apoapsis, then drop it", ParamKind.Bool, "true",
                        "needs a separator marked 'Stage 1 separator'"),
                    new ParamDef("pe_cap_km", "Stage 1 max periapsis (km)", ParamKind.Float, "55"),
                },
                // the Lift panel's direction buttons, same names and order (Fido.Lift.LaunchDirections)
                presets = Fido.Lift.LaunchDirections.All.Select(d => new StepPreset(d.PresetLabel, new()
                {
                    ["inclination"] = d.inclination.ToString("0", System.Globalization.CultureInfo.InvariantCulture),
                    ["south"] = d.south ? "true" : "false",
                })).ToList(),
                summary = s => $"Launch to {Km(s, "ap_km")}, " +
                               (Fido.Lift.LaunchDirections.Match(s.GetFloat("inclination"), s.GetBool("south")) is { } dir
                                   ? dir.PresetLabel
                                   : $"{s.GetFloat("inclination"):0.#}°" + (s.GetBool("south") ? " south" : "")) +
                               (s.GetBool("optimiser") ? ", stage 1 disposal" : ""),
                create = () => new LaunchRunner(),
            },
            new StepType
            {
                id = Circularise, label = "Circularise", default_auto = true,
                summary = s => "Circularise at apoapsis",
                create = () => new CirculariseRunner(),
            },
            new StepType
            {
                id = Fairings, label = "Release fairings",
                parameters =
                {
                    new ParamDef("radial_in", "Point radial-in first", ParamKind.Bool, "true"),
                },
                summary = s => "Release fairings" + (s.GetBool("radial_in") ? " (radial-in)" : ""),
                create = () => new FairingsRunner(),
            },
            new StepType
            {
                id = PartActions, label = "Part actions", default_auto = true,
                parameters =
                {
                    new ParamDef("radial_in", "Point radial-in first", ParamKind.Bool, "false"),
                    new ParamDef("delay", "Pause between actions (s)", ParamKind.Float, "1"),
                },
                summary = s => s.actions.Count == 0 ? "Part actions (none yet)"
                    : s.actions.Count == 1 ? PartActionCatalog.Describe(s.actions[0])
                    : $"{s.actions.Count} part actions: {PartActionCatalog.Describe(s.actions[0])}, ...",
                create = () => new PartActionsRunner(),
            },
            new StepType
            {
                id = Plane, label = "Plane change",
                parameters =
                {
                    new ParamDef("inclination", "Target inclination (°)", ParamKind.Float, "90", "burns at the next node"),
                },
                presets =
                {
                    new StepPreset("Equatorial (0°)", new() { ["inclination"] = "0" }),
                    new StepPreset("Polar (90°)", new() { ["inclination"] = "90" }),
                    new StepPreset("Retrograde (180°)", new() { ["inclination"] = "180" }),
                },
                summary = s => $"Plane change to {s.GetFloat("inclination"):0.#}°",
                create = () => new PlaneRunner(),
            },
            new StepType
            {
                id = Altitude, label = "Change altitude",
                parameters =
                {
                    new ParamDef("alt_km", "Circular altitude (km)", ParamKind.Float, "150"),
                },
                summary = s => $"Change altitude to {Km(s, "alt_km")}",
                create = () => new AltitudeRunner(),
            },
            new StepType
            {
                id = Deploy, label = "Deploy constellation",
                parameters =
                {
                    new ParamDef("alt_km", "Altitude (km, 0 = suggested)", ParamKind.Float, "0"),
                    new ParamDef("separation", "Separation (°, 0 = auto)", ParamKind.Float, "0"),
                    new ParamDef("phasing", "Phasing orbits (0 = auto)", ParamKind.Int, "0"),
                    new ParamDef("phase_budget", "Phasing Δv budget (m/s)", ParamKind.Float, "150",
                        "Auto picks the fewest phasing orbits whose burns fit this"),
                    new ParamDef("auto_dish", "Auto dish switch on", ParamKind.Bool, "true"),
                    new ParamDef("move_fuel", "Recover fuel", ParamKind.Bool, "true"),
                },
                summary = s => "Deploy constellation" +
                               (s.GetFloat("alt_km") > 0 ? $" at {Km(s, "alt_km")}" : " at the suggested altitude"),
                create = () => new DeployRunner(),
            },
            new StepType
            {
                id = Transfer, label = "Transfer to moon",
                parameters =
                {
                    new ParamDef("target", "Target", ParamKind.Choice, "Mun") { choices = TransferChecks.TargetChoices },
                    new ParamDef("pe_km", "Arrival periapsis (km)", ParamKind.Float, "50",
                        "Places the departure node only (replacing any nodes on the plan) - add Execute node after it. " +
                        "With a Capture step later, that step's altitude and orbit are aimed for instead"),
                },
                summary = s => $"Transfer to {s.Get("target")} (Pe {Km(s, "pe_km")})",
                create = () => new TransferRunner(),
            },
            new StepType
            {
                id = Correction, label = "Course correction",
                parameters =
                {
                    new ParamDef("target", "Target", ParamKind.Choice, "Mun") { choices = TransferChecks.TargetChoices },
                    new ParamDef("pe_km", "Arrival periapsis (km)", ParamKind.Float, "50",
                        "On the way out: places a small node that puts the arrival back on target - add Execute node after it. " +
                        "With a Capture step later, that step's altitude and orbit are aimed for instead"),
                },
                summary = s => $"Course correction to {s.Get("target")} (Pe {Km(s, "pe_km")})",
                create = () => new CorrectionRunner(),
            },
            new StepType
            {
                id = Capture, label = "Capture at moon",
                parameters =
                {
                    new ParamDef("target", "Target", ParamKind.Choice, "Mun") { choices = TransferChecks.TargetChoices },
                    new ParamDef("alt_km", "Orbit altitude (km)", ParamKind.Float, "50",
                        "The Transfer and Course correction steps before it aim the arrival for this altitude and plane"),
                    new ParamDef("orbit", "Orbit", ParamKind.Choice, TransferPlanner.PlaneEquatorial,
                        "Equatorial or polar needs a Course correction on the way unless the departure already lines up")
                        { choices = TransferPlanner.PlaneChoices },
                },
                summary = s => $"Capture at {s.Get("target")}: {Km(s, "alt_km")} {TransferPlanner.PlaneName(TransferPlanner.ParsePlane(s.Get("orbit")))} orbit",
                create = () => new CaptureRunner(),
            },
            new StepType
            {
                id = Node, label = "Execute node",
                summary = s => "Execute the next manoeuvre node",
                create = () => new NodeRunner(),
            },
        };

        public static StepType Get(string id) => All.FirstOrDefault(t => t.id == id);
    }

    /// <summary>Plan checks shared by the VAB editor and the flight tab.</summary>
    public static class MissionValidator
    {
        public class Issue
        {
            public int step; // -1 = the whole plan
            public bool error;
            public string text;
        }

        public static List<Issue> Check(MissionPlanData plan)
        {
            var issues = new List<Issue>();
            if (plan == null || plan.IsEmpty) return issues;
            var steps = plan.steps;
            for (int i = 0; i < steps.Count; i++)
            {
                var s = steps[i];
                if (s.Type == null)
                {
                    issues.Add(new Issue { step = i, error = true, text = $"Step {i + 1}: unknown step type '{s.type}'" });
                    continue;
                }
                if (s.type == StepCatalog.Launch)
                {
                    bool first = steps.Take(i).All(x => x.type == StepCatalog.Quicksave);
                    if (!first)
                        issues.Add(new Issue { step = i, error = true, text = $"Step {i + 1}: Launch must come before every other step except quicksaves" });
                    var next = i + 1 < steps.Count ? steps[i + 1] : null;
                    if (next == null || next.type != StepCatalog.Circularise)
                        issues.Add(new Issue { step = i, error = true, text = $"Step {i + 2}: Launch must be followed straight away by Circularise - the stage 1 burn leaves periapsis in the atmosphere" });
                    if (s.GetBool("optimiser") && s.GetFloat("pe_cap_km") >= 70)
                        issues.Add(new Issue { step = i, text = $"Step {i + 1}: stage 1 max periapsis is at or above Kerbin's atmosphere (70 km) - stage 1 may not decay" });
                }
                if (s.type == StepCatalog.Circularise && i > 0 && steps[i - 1].type == StepCatalog.Launch && !s.auto)
                    issues.Add(new Issue { step = i, text = $"Step {i + 1}: runs straight after Launch whatever its auto-continue setting" });
                if (s.type == StepCatalog.PartActions && s.actions.Count == 0)
                    issues.Add(new Issue { step = i, error = true, text = $"Step {i + 1}: no part actions - click 'Add action to mission' in a part's window" });
                if (s.type == StepCatalog.Altitude && s.GetFloat("alt_km") < 71)
                    issues.Add(new Issue { step = i, error = true, text = $"Step {i + 1}: altitude must be above the atmosphere (70 km)" });
                TransferChecks.CheckOrder(steps, i, issues);
            }
            return issues;
        }

        /// <summary>
        /// Check() plus the stage checks: the plan's burns from step 'from' on taken off 'budget'
        /// (the stages left after stage 1, or the vessel's stages in flight). parking_period: the
        /// parking orbit's period, for the long-burn warning (0 = don't know).
        /// </summary>
        public static List<Issue> Check(MissionPlanData plan, StageBudget budget, List<double> dv, int from, double parking_period)
        {
            var issues = Check(plan);
            if (plan == null || plan.IsEmpty || budget == null || dv == null) return issues;
            TransferChecks.CheckStages(plan, budget, dv, from, issues, parking_period);
            return issues;
        }

        /// <summary>
        /// Where the stage walk starts: the step after Launch (Circularise) while Launch is still to fly -
        /// the budget is then the stages above stage 1 - else the current step.
        /// </summary>
        public static int BudgetFrom(MissionPlanData plan, out bool before_launch)
        {
            before_launch = false;
            if (plan == null) return 0;
            int launch = plan.steps.FindIndex(x => x.type == StepCatalog.Launch);
            if (launch >= 0 && plan.steps[launch].state != "done" && launch >= plan.current)
            {
                before_launch = true;
                return launch + 1;
            }
            int from = Math.Max(0, plan.current);
            // a transfer's node is placed but not flown yet: its burn still comes out of the stages
            if (from > 0 && from < plan.steps.Count && plan.steps[from].type == StepCatalog.Node &&
                TransferChecks.PlacesNode(plan.steps[from - 1]) && plan.steps[from - 1].state == "done")
                from--;
            return from;
        }

        /// <summary>Period of the plan's parking orbit (the Launch step's apoapsis, circular), or 0.</summary>
        public static double ParkingPeriod(MissionPlanData plan, double mu, double radius)
        {
            var launch = plan?.steps.FirstOrDefault(x => x.type == StepCatalog.Launch);
            if (launch == null) return 0;
            return OrbitMath.Period(mu, radius + launch.GetFloat("ap_km") * 1000);
        }

        /// <summary>Rough Δv per step, from the planned orbits alone (circular orbits around the home body). NaN = can't say.</summary>
        public static List<double> Estimates(MissionPlanData plan, double mu, double radius)
        {
            var result = new List<double>();
            double alt_km = double.NaN, inc = double.NaN;
            MissionStepData launch = null;
            foreach (var s in plan.steps)
            {
                double dv = double.NaN;
                switch (s.type)
                {
                    case StepCatalog.Launch:
                        alt_km = s.GetFloat("ap_km");
                        inc = s.GetFloat("inclination");
                        launch = s;
                        break;
                    case StepCatalog.Circularise:
                        // the upper stage raises periapsis from where stage 1 left it (its max periapsis with
                        // disposal on; without, assume from the surface - a generous figure)
                        if (launch != null && !double.IsNaN(alt_km))
                        {
                            double pe_km = launch.GetBool("optimiser") ? Math.Min(launch.GetFloat("pe_cap_km"), (float)alt_km) : 0;
                            double ap = radius + alt_km * 1000;
                            dv = OrbitMath.CircularizeToDv(mu, radius + pe_km * 1000, ap, ap);
                        }
                        launch = null;
                        break;
                    case StepCatalog.Transfer:
                    case StepCatalog.Correction:
                    case StepCatalog.Capture:
                        dv = TransferChecks.Estimate(plan.steps, s, mu, radius, alt_km, inc);
                        // after the transfer the orbit isn't the home body's any more
                        if (s.type == StepCatalog.Transfer) alt_km = double.NaN;
                        break;
                    case StepCatalog.Plane:
                        if (!double.IsNaN(alt_km))
                            dv = OrbitMath.PlaneChangeDv(OrbitMath.CircularSpeed(mu, radius + alt_km * 1000), s.GetFloat("inclination") - inc);
                        inc = s.GetFloat("inclination");
                        break;
                    case StepCatalog.Altitude:
                        dv = Hohmann(mu, radius, alt_km, s.GetFloat("alt_km"));
                        alt_km = s.GetFloat("alt_km");
                        break;
                    case StepCatalog.Deploy:
                        if (s.GetFloat("alt_km") > 0)
                        {
                            dv = Hohmann(mu, radius, alt_km, s.GetFloat("alt_km"));
                            alt_km = s.GetFloat("alt_km");
                        }
                        break;
                    case StepCatalog.Quicksave:
                    case StepCatalog.Fairings:
                    case StepCatalog.PartActions:
                        dv = 0;
                        break;
                }
                result.Add(dv);
            }
            return result;
        }

        internal static double Hohmann(double mu, double radius, double from_km, double to_km)
        {
            if (double.IsNaN(from_km)) return double.NaN;
            double r1 = radius + from_km * 1000, r2 = radius + to_km * 1000;
            double a = (r1 + r2) / 2;
            return Math.Abs(OrbitMath.SpeedAt(mu, r1, a) - OrbitMath.CircularSpeed(mu, r1)) +
                   Math.Abs(OrbitMath.CircularSpeed(mu, r2) - OrbitMath.SpeedAt(mu, r2, a));
        }
    }
}
