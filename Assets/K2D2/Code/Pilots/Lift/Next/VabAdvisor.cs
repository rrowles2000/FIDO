using System;
using System.Collections.Generic;
using System.Reflection;
using K2D2.Lift.Stage1;
using KSP.Game;
using KSP.Messages;
using KSP.Sim.DeltaV;
using UnityEngine;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// VAB launch advisory (NN ascent report, Section 9): once a separator is marked as the stage-1
    /// separator (StageGroup), runs the New lift system's LaunchCheck on the craft as built and
    /// shows the verdict as the "Stage 1 check" line in that separator's part window, e.g.
    /// "SHORT: Ap 45/90 km, S1 Δv 1,820* · TWR 1.30". A trailing * means drag was left out (the VAB
    /// has no drag cubes to read), so the real flight will do somewhat worse; the pad check at
    /// launch includes drag and is the one that refuses.
    ///
    /// Data: the game publishes VesselDeltaVCalculationMessage (carrying the VesselDeltaVComponent)
    /// every time it recalculates the assembly's Δv - after any edit, including StageGroup's
    /// regroup - so the check reruns on each one (at most once a second). Planet: the home world,
    /// launching from the equator (the VAB doesn't know the launch site). Lift settings: target Ap,
    /// heading, max throttle and the New system's limits; weights untrained (the pad check trains).
    ///
    /// StageGroup is found by reflection (Module_StageOneSeparator.MarkedSeparatorPart / SetAdvice),
    /// so neither mod needs the other installed.
    ///
    /// Each verdict change is logged as [VAB] with the game's own first-stage TWR next to the
    /// check's, to help track down the VAB TWR readout that didn't seem to update (2026-11-13).
    /// </summary>
    public static class VabAdvisor
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.VabAdvisor");

        const float CheckPeriodS = 1f;    // at most one check per second
        const float RefreshPeriodS = 10f; // and at least every 10 s while a separator is marked
        const double LaunchAltitudeM = 70;

        static bool subscribed;
        static readonly List<VesselDeltaVComponent> recent = new();
        static bool dirty;
        static float next_check, next_refresh;
        static object last_part;
        static string last_text;
        // A check that can't model the craft (mid-edit: "no engines fire before the decoupler")
        // only replaces a good verdict once it has persisted this long - 2026-11-13 flashed it five
        // times while parts were being moved.
        const float UnknownHoldS = 3f;
        static float unknown_since = -1;
        static bool last_was_good;

        static bool api_looked;
        static MethodInfo marked_method, set_method;
        static bool error_logged;

        /// <summary>Called every frame from K2D2_Plugin.Update; does nothing outside the VAB.</summary>
        public static void Update()
        {
            try
            {
                if (!InVab()) { last_part = null; return; }
                if (!subscribed) Subscribe();
                if (Time.unscaledTime < next_check) return;
                next_check = Time.unscaledTime + CheckPeriodS;
                if (!FindApi()) return;

                object part = marked_method.Invoke(null, null);
                if (part == null) { last_part = null; return; }
                if (!dirty && ReferenceEquals(part, last_part) && Time.unscaledTime < next_refresh) return;
                dirty = false;
                last_part = part;
                next_refresh = Time.unscaledTime + RefreshPeriodS;

                string text = Check(part, out string detail, out bool good);
                if (good) unknown_since = -1;
                else if (last_was_good)
                {
                    if (unknown_since < 0) unknown_since = Time.unscaledTime;
                    if (Time.unscaledTime - unknown_since < UnknownHoldS)
                    {
                        dirty = true; // look again next second
                        return;
                    }
                }
                last_was_good = good;
                set_method.Invoke(null, new[] { part, text });
                if (text != last_text)
                {
                    last_text = text;
                    logger.LogInfo("[VAB] stage-1 check: " + detail);
                }
            }
            catch (Exception e)
            {
                if (!error_logged) logger.LogError("[VAB] advisory failed: " + e);
                error_logged = true;
            }
        }

        static string Check(object part, out string detail, out bool good)
        {
            good = false;
            var dv = FindDeltaV(part);
            if (dv == null) { detail = "no Δv data yet"; return "no Δv data yet - edit the craft to refresh"; }

            var body = GameManager.Instance?.Game?.UniverseModel?.HomeWorld;
            var settings = LiftPilot.Instance?.settings;
            if (body == null || settings == null) { detail = "no home planet or Lift settings"; return "K2-D2 not ready"; }

            var model = Stage1Model.FromDeltaV(dv, body, 0, LaunchAltitudeM, part as IDeltaVPart, allowNoUpper: true);
            var stability = StabilityCheck.ForAssembly(K2D2.Mission.MissionStore.MainAssembly, part);
            model?.SetSurfaces(stability);
            var ctx = PolicyContext.From(settings);
            CraftMemory.ApplyTo(ref ctx, model);
            var r = LaunchCheck.Run(model, ctx, settings.heading.V, settings.destination_Ap_km.V, CraftMemory.GetCalibration(model));
            var memory = CraftMemory.Get(model);
            detail = $"{r.Summary()} | {stability.Summary()} | memory {CraftMemory.Key(model)}: {CraftMemory.Describe(model)} | game first-stage TWR ASL {GameFirstStageTwr(dv):n2}, " +
                     $"{dv.StageInfo?.Count ?? 0} Δv stages, separator activation stage {model.separatorStage}";
            good = r.verdict != LaunchCheck.Verdict.Unknown;
            string learned = memory != null && memory.fragile ? " · FRAGILE design (has drifted before)" : "";
            return good ? r.Short() + " · " + stability.Short() + learned : r.problem;
        }

        // the Δv component whose stages contain the marked separator (else the latest one seen)
        static VesselDeltaVComponent FindDeltaV(object part)
        {
            foreach (var dv in recent)
            {
                var stages = dv?.StageInfo;
                if (stages == null) continue;
                foreach (var stage in stages)
                {
                    if (stage?.Parts == null) continue;
                    foreach (var info in stage.Parts)
                        if (ReferenceEquals(info?.PartRef, part)) return dv;
                }
            }
            return recent.Count > 0 ? recent[0] : null;
        }

        static float GameFirstStageTwr(VesselDeltaVComponent dv)
        {
            DeltaVStageInfo first = null;
            if (dv?.StageInfo != null)
                foreach (var s in dv.StageInfo)
                    if (s != null && s.ThrustVac > 0 && (first == null || s.Stage > first.Stage)) first = s;
            return first?.TWRASL ?? 0;
        }

        static void Subscribe()
        {
            var messages = GameManager.Instance?.Game?.Messages;
            if (messages == null) return;
            messages.Subscribe<VesselDeltaVCalculationMessage>(msg =>
            {
                var comp = (msg as VesselDeltaVCalculationMessage)?.DeltaVComponent;
                if (comp == null) return;
                recent.Remove(comp);
                recent.Insert(0, comp);
                if (recent.Count > 6) recent.RemoveAt(recent.Count - 1);
                dirty = true;
            });
            subscribed = true;
        }

        static bool FindApi()
        {
            if (api_looked) return marked_method != null && set_method != null;
            api_looked = true;
            Type t = null;
            foreach (var asm in AppDomain.CurrentDomain.GetAssemblies())
            {
                t = asm.GetType("StageGroup.Module_StageOneSeparator", false);
                if (t != null) break;
            }
            marked_method = t?.GetMethod("MarkedSeparatorPart", BindingFlags.Public | BindingFlags.Static);
            set_method = t?.GetMethod("SetAdvice", BindingFlags.Public | BindingFlags.Static);
            if (marked_method == null || set_method == null)
                logger.LogInfo("[VAB] StageGroup advisory API not found - VAB stage-1 check off (StageGroup missing or older)");
            return marked_method != null && set_method != null;
        }

        static bool InVab()
        {
            try { return GameManager.Instance.Game.GlobalGameState.GetGameState().GameState == GameState.VehicleAssemblyBuilder; }
            catch (Exception) { return false; }
        }
    }
}
