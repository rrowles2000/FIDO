using System;
using HarmonyLib;
using KSP.Api.CoreTypes;
using KSP.OAB;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// The VAB's "open design" dialog (2026-11-15): each saved design's entry gets a K2D2 line saying
    /// whether the lift system has trained it - on the parts/stages line under the name and at the
    /// top of the description. Designs are matched to CraftMemory by name (the flight name minus its
    /// launch counter, CraftMemory.DesignName), since the dialog only has the saved metadata, not
    /// the stage-1 mass and thrust the memory is keyed on.
    /// </summary>
    public static class VabLoadDialogTag
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.VabLoadDialog");
        static bool error_logged;

        public static void Apply(Harmony harmony)
        {
            try
            {
                var target = AccessTools.Method(typeof(OABLoadDialogExistingSave), "Initialize");
                if (target == null) { logger.LogWarning("[VAB] OABLoadDialogExistingSave.Initialize not found - no training tags in the open dialog"); return; }
                harmony.Patch(target, postfix: new HarmonyMethod(typeof(VabLoadDialogTag), nameof(InitializePostfix)));
            }
            catch (Exception e) { logger.LogError("[VAB] patching the open dialog: " + e); }
        }

        static void InitializePostfix(OABLoadDialogExistingSave __instance, OABSavedWorkspaceMetaInfo __2, bool __4)
        {
            try
            {
                if (__4 || __2 == null) return; // stock craft: never trained by K2D2
                string name = !string.IsNullOrWhiteSpace(__2.Name) ? __2.Name : __2.VehicleName;
                var record = CraftMemory.FindByDesignName(name) ?? CraftMemory.FindByDesignName(__2.VehicleName);
                string status = CraftMemory.TrainingStatus(record);

                var parts = AccessTools.Field(typeof(OABLoadDialogExistingSave), "_vehiclePartsAndStages")?.GetValue(__instance) as Property<string>;
                if (parts != null)
                {
                    string short_status = record?.weights != null ? "K2D2 trained" : "K2D2 not trained";
                    parts.SetValue($"{parts.GetValue()}  ·  {short_status}");
                }
                var desc = AccessTools.Field(typeof(OABLoadDialogExistingSave), "_vehicleDescription")?.GetValue(__instance) as Property<string>;
                if (desc != null)
                {
                    string d = desc.GetValue();
                    desc.SetValue(string.IsNullOrWhiteSpace(d) ? status : status + "\n" + d);
                }
            }
            catch (Exception e)
            {
                if (!error_logged) logger.LogWarning("[VAB] training tag failed: " + e.Message);
                error_logged = true;
            }
        }
    }
}
