using System;
using HarmonyLib;
using KSP.Api.CoreTypes;
using KSP.OAB;
using UnityEngine;
using UnityEngine.UI;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido.Lift.Next
{
    /// <summary>
    /// The VAB's "open design" dialog (2026-11-15): each saved design's entry gets a FIDO line saying
    /// whether the lift system has trained it - on the parts/stages line under the name and at the
    /// top of the description. Designs are matched to CraftMemory by name (the flight name minus its
    /// launch counter, CraftMemory.DesignName), since the dialog only has the saved metadata, not
    /// the stage-1 mass and thrust the memory is keyed on.
    ///
    /// 2026-11-23: a "Reset FIDO training" button beside the stock Delete button. It is a copy of
    /// Delete with the game's data binding stripped, enabled when the selected design has a FIDO
    /// record; the first click asks to confirm, the second deletes the design's records
    /// (CraftMemory.ResetByDesignName) and refreshes the list so the tags update.
    /// </summary>
    public static class VabLoadDialogTag
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.VabLoadDialog");
        static bool error_logged;

        const string ResetButtonName = "FidoResetTraining";
        const string ResetLabel = "Reset FIDO training", ConfirmLabel = "Click again to reset";

        static Button reset_button;
        static TMPro.TMP_Text reset_text;
        static bool confirming; // armed by the first click, cleared by the second or a new selection

        public static void Apply(Harmony harmony)
        {
            try
            {
                var target = AccessTools.Method(typeof(OABLoadDialogExistingSave), "Initialize");
                if (target == null) logger.LogWarning("[VAB] OABLoadDialogExistingSave.Initialize not found - no training tags in the open dialog");
                else harmony.Patch(target, postfix: new HarmonyMethod(typeof(VabLoadDialogTag), nameof(InitializePostfix)));

                var show = AccessTools.Method(typeof(OABLoadDialog), "ShowDialog");
                var select = AccessTools.Method(typeof(OABLoadDialog), "SelectExistingSaveEntry");
                if (show == null || select == null) { logger.LogWarning("[VAB] OABLoadDialog.ShowDialog/SelectExistingSaveEntry not found - no reset button"); return; }
                harmony.Patch(show, postfix: new HarmonyMethod(typeof(VabLoadDialogTag), nameof(ShowDialogPostfix)));
                harmony.Patch(select, postfix: new HarmonyMethod(typeof(VabLoadDialogTag), nameof(SelectPostfix)));
            }
            catch (Exception e) { logger.LogError("[VAB] patching the open dialog: " + e); }
        }

        static void InitializePostfix(OABLoadDialogExistingSave __instance, OABSavedWorkspaceMetaInfo __2, bool __4)
        {
            try
            {
                if (__4 || __2 == null) return; // stock craft: never trained by FIDO
                var record = Find(__2);
                string status = CraftMemory.TrainingStatus(record);

                var parts = AccessTools.Field(typeof(OABLoadDialogExistingSave), "_vehiclePartsAndStages")?.GetValue(__instance) as Property<string>;
                if (parts != null)
                {
                    string short_status = record?.weights != null ? "FIDO trained" : "FIDO not trained";
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

        static string[] Names(OABSavedWorkspaceMetaInfo info) =>
            new[] { !string.IsNullOrWhiteSpace(info.Name) ? info.Name : info.VehicleName, info.VehicleName };

        static CraftMemory.Record Find(OABSavedWorkspaceMetaInfo info)
        {
            var n = Names(info);
            return CraftMemory.FindByDesignName(n[0]) ?? CraftMemory.FindByDesignName(n[1]);
        }

        // ---------------------------------------------------------------- reset button

        static OABLoadDialogExistingSave Selected(OABLoadDialog dialog) =>
            AccessTools.Field(typeof(OABLoadDialog), "_selectedEntry")?.GetValue(dialog) as OABLoadDialogExistingSave;

        static OABSavedWorkspaceMetaInfo Meta(OABLoadDialogExistingSave entry) =>
            entry == null ? null : AccessTools.Field(typeof(OABLoadDialogExistingSave), "_workspaceMetaInfo")?.GetValue(entry) as OABSavedWorkspaceMetaInfo;

        static void ShowDialogPostfix(OABLoadDialog __instance)
        {
            try
            {
                if (reset_button == null) CreateButton(__instance);
                confirming = false;
                SetLabel(ResetLabel);
                Refresh(__instance);
            }
            catch (Exception e) { logger.LogWarning("[VAB] reset button: " + e); }
        }

        static void SelectPostfix(OABLoadDialog __instance)
        {
            try
            {
                confirming = false;
                SetLabel(ResetLabel);
                Refresh(__instance);
            }
            catch (Exception e) { logger.LogWarning("[VAB] reset button refresh: " + e.Message); }
        }

        static void Refresh(OABLoadDialog dialog)
        {
            if (reset_button == null) return;
            var entry = Selected(dialog);
            var meta = Meta(entry);
            reset_button.interactable = entry != null && !entry.IsStock && meta != null && Find(meta) != null;
        }

        static void SetLabel(string s)
        {
            if (reset_text != null) reset_text.text = s;
        }

        static void CreateButton(OABLoadDialog dialog)
        {
            var buttons = AccessTools.Field(typeof(OABLoadDialog), "_interactableButtons")?.GetValue(dialog) as ButtonExtended[];
            if (buttons == null || buttons.Length == 0) { logger.LogWarning("[VAB] no dialog buttons to copy - no reset button"); return; }
            ButtonExtended source = null;
            foreach (var b in buttons)
                if (b != null && b.name.IndexOf("delete", StringComparison.OrdinalIgnoreCase) >= 0) { source = b; break; }
            if (source == null) source = buttons[buttons.Length - 1];
            logger.LogInfo($"[VAB] dialog buttons: {string.Join(", ", Array.ConvertAll(buttons, b => b == null ? "null" : b.name))}; copying '{source.name}' under '{source.transform.parent?.name}'");

            // copy it while the source is inactive, so the copy's Awake doesn't run before the
            // game's binding components (which would wire it to deleteWorkspace) are removed
            var src_go = source.gameObject;
            bool was_active = src_go.activeSelf;
            src_go.SetActive(false);
            GameObject go;
            try { go = UnityEngine.Object.Instantiate(src_go, src_go.transform.parent, false); }
            finally { src_go.SetActive(was_active); }
            go.name = ResetButtonName;
            // Whitelist, not blacklist (2026-11-23): the copy must not keep ANY of the game's components.
            // The stock Delete button is wired to deleteWorkspace by UIAction_Void_Button (global namespace,
            // no "Bind" in its name - the old name filter kept it) and labelled by value binders; keep only
            // Unity's own UI types (Image, Button/ButtonExtended, layout, TMP text) and drop everything else.
            var kept = new System.Collections.Generic.List<string>();
            var dropped = new System.Collections.Generic.List<string>();
            foreach (var c in go.GetComponentsInChildren<MonoBehaviour>(true))
            {
                var ns = c.GetType().Namespace ?? "";
                if (ns.StartsWith("UnityEngine", StringComparison.Ordinal) || ns.StartsWith("TMPro", StringComparison.Ordinal))
                    kept.Add(c.GetType().Name);
                else
                {
                    dropped.Add(c.GetType().FullName);
                    UnityEngine.Object.DestroyImmediate(c);
                }
            }
            logger.LogInfo($"[VAB] reset button: kept {string.Join(", ", kept)}; removed {string.Join(", ", dropped)}");
            reset_button = go.GetComponent<Button>();
            // every click event the copy carries is replaced, including ButtonExtended's own (it keeps the
            // serialized persistent listeners of the Delete button otherwise)
            reset_button.onClick = new Button.ButtonClickedEvent();
            reset_button.onClick.AddListener(() => OnClick(dialog));
            if (reset_button is ButtonExtended ext)
            {
                ext.onLeftClick = new Button.ButtonClickedEvent();
                ext.onRightClick = new Button.ButtonClickedEvent();
                ext.onMiddleClick = new Button.ButtonClickedEvent();
                ext.onLeftMouseHold = new UnityEngine.Events.UnityEvent();
                ext.onRightMouseHold = new UnityEngine.Events.UnityEvent();
                ext.onMiddleMouseHold = new UnityEngine.Events.UnityEvent();
                ext.onLeftClick.AddListener(() => OnClick(dialog)); // whichever event the click raises; OnClick ignores a repeat in the same frame
            }
            reset_text = go.GetComponentInChildren<TMPro.TMP_Text>(true);
            go.transform.SetSiblingIndex(src_go.transform.GetSiblingIndex() + 1);
            if (go.transform.parent != null && go.transform.parent.GetComponent<LayoutGroup>() == null)
            {
                // no layout group to place it: put it just left of the source
                var rt = (RectTransform)go.transform;
                rt.anchoredPosition -= new Vector2(rt.rect.width + 8, 0);
            }
            go.SetActive(was_active);
        }

        static int last_click_frame = -1;

        static void OnClick(OABLoadDialog dialog)
        {
            try
            {
                if (Time.frameCount == last_click_frame) return; // onClick and onLeftClick from the same press
                last_click_frame = Time.frameCount;
                var entry = Selected(dialog);
                var meta = Meta(entry);
                if (entry == null || entry.IsStock || meta == null) return;
                if (!confirming)
                {
                    confirming = true;
                    SetLabel(ConfirmLabel);
                    return;
                }
                confirming = false;
                SetLabel(ResetLabel);
                int n = CraftMemory.ResetByDesignName(Names(meta));
                logger.LogInfo($"[VAB] reset FIDO training for '{meta.Name}' ({n} record(s))");
                dialog.PopulateEntries(); // rebuild the entries so their tags show "not trained"
                Refresh(dialog);
            }
            catch (Exception e) { logger.LogWarning("[VAB] reset training: " + e); }
        }
    }
}
