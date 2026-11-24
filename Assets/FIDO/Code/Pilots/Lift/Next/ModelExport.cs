using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using Fido.Lift.Stage1;
using KSP.Sim;
using Newtonsoft.Json;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido.Lift.Next
{
    /// <summary>
    /// Writes everything the launch check simulated with, at each New-system launch, to
    /// mods/FIDO/fido_models/&lt;time&gt;_&lt;design&gt;.json: the stage model, drag table, atmosphere
    /// table and drag constants, the settings and learned limits, the calibration, the launch
    /// check's verdict and its planned trajectory. With the flight log this lets a flight be
    /// replayed and compared outside the game (2026-11-14: to find why low-TWR flights fly
    /// 6-10° steeper than planned). The newest 60 files are kept.
    /// </summary>
    public static class ModelExport
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.ModelExport");
        const int Keep = 60;

        static string Dir => Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location) ?? ".", "fido_models");

        public static void Write(Stage1Model model, PolicyContext ctx, double headingDeg, double targetApKm,
                                 LaunchCheck.Result check, LaunchCheck.Calibration calibration, string vesselName)
        {
            try
            {
                if (model == null || !model.valid || model.body == null) return;
                var body = model.body;
                var sim = new Stage1Sim(model);
                var reynolds = new List<double[]>();
                foreach (var x in new[] { 0.1, 1, 3, 10, 30, 100, 300, 1000, 3000, 10000, 30000, 100000 })
                    reynolds.Add(new[] { x, (double)PhysicsSettings.DragCurvePseudoReynolds.Evaluate((float)x) });

                var data = new Dictionary<string, object>
                {
                    ["when"] = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
                    ["vessel"] = vesselName,
                    ["design"] = CraftMemory.Key(model),
                    ["body"] = new Dictionary<string, object>
                    {
                        ["name"] = body.Name, ["mu"] = body.gravParameter, ["radius"] = body.radius,
                        ["rotationPeriod"] = body.rotationPeriod, ["atmosphereDepth"] = body.hasAtmosphere ? body.atmosphereDepth : 0,
                        ["adiabaticIndex"] = body.atmosphereAdiabaticIndex,
                    },
                    ["launch"] = new Dictionary<string, object>
                    {
                        ["latitudeDeg"] = model.latitudeDeg, ["altitudeM"] = model.launchAltitude,
                        ["headingDeg"] = headingDeg, ["targetApKm"] = targetApKm,
                    },
                    ["stage1"] = model.phases,
                    ["upper"] = model.upperPhases,
                    ["separator"] = new Dictionary<string, object> { ["name"] = model.separatorName, ["stage"] = model.separatorStage },
                    ["surfaces"] = new Dictionary<string, object> { ["ahead"] = model.finsAhead, ["behind"] = model.finsBehind },
                    ["drag"] = new Dictionary<string, object>
                    {
                        ["valid"] = model.dragValid, ["mach"] = model.machSamples, ["areaDragNose"] = model.areaDrag,
                        ["areaDragSide"] = model.areaDragSide, ["noseFactor"] = Stage1Model.NoseFactor, ["sideShare"] = Stage1Model.SideShare,
                        ["dragCubeMultiplier"] = PhysicsSettings.DragCubeMultiplier, ["dragMultiplier"] = PhysicsSettings.DragMultiplier,
                        ["pseudoReynolds"] = reynolds,
                        ["transonic"] = new[] { 0.8, 0.9, 1.0, 1.1, 1.2, 1.4, 1.6 }.ToDictionaryBy(m => m.ToString("0.0"), Stage1Model.TransonicFactor),
                    },
                    ["atmosphere"] = new Dictionary<string, object>
                    {
                        ["stepM"] = sim.AtmoStepM, ["depthM"] = sim.AtmoDepthM,
                        ["pressureKPa"] = sim.AtmoPressureKPa, ["density"] = sim.AtmoDensity, ["soundSpeed"] = sim.AtmoSoundSpeed,
                    },
                    ["context"] = ctx,
                    ["calibration"] = calibration,
                    ["check"] = check == null ? null : new Dictionary<string, object>
                    {
                        ["summary"] = check.Summary(), ["verdict"] = check.verdict.ToString(),
                        ["cutT"] = check.cutT, ["cutAltKm"] = check.altKm, ["cutSpeed"] = check.cutSpeed, ["cutPathDeg"] = check.cutPathDeg,
                        ["spareDv"] = check.spareDv, ["plan"] = check.plan,
                    },
                };

                Directory.CreateDirectory(Dir);
                string safe = (CraftMemory.Key(model) ?? "unknown").Replace('/', '_');
                string file = Path.Combine(Dir, $"{DateTime.Now:yyyyMMdd-HHmmss}_{safe}.json");
                File.WriteAllText(file, JsonConvert.SerializeObject(data, Formatting.Indented,
                    new JsonSerializerSettings { ReferenceLoopHandling = ReferenceLoopHandling.Ignore }));
                logger.LogInfo($"[Export] model and plan -> {file}");

                var files = new List<string>(Directory.GetFiles(Dir, "*.json"));
                files.Sort();
                for (int i = 0; i < files.Count - Keep; i++) File.Delete(files[i]);
            }
            catch (Exception e)
            {
                logger.LogWarning("[Export] failed: " + e.Message);
            }
        }

        static Dictionary<string, double> ToDictionaryBy(this double[] xs, Func<double, string> key, Func<double, double> value)
        {
            var d = new Dictionary<string, double>();
            foreach (var x in xs) d[key(x)] = value(x);
            return d;
        }
    }
}
