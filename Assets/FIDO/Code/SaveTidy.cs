using System;
using System.IO;
using System.Linq;
using System.Reflection;
using HarmonyLib;
using KSP.Game;
using KSP.Sim;
using UnityEngine;
using UnityEngine.UI;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido.Saves
{
    /// <summary>
    /// Save housekeeping (2026-11-15):
    ///  1. Training flights don't autosave. Every pad landing, splashdown and timer tick wrote a
    ///     ~3 MB autosave the campaign screen then listed; the flights are already in the log.
    ///  2. A "Clear autosaves" button on the campaign screen, next to the game's Delete button:
    ///     deletes the selected campaign's autosave_N and autosaveevent_* files (manual and quick
    ///     saves are kept), with a second click to confirm.
    /// </summary>
    public static class SaveTidy
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.SaveTidy");

        public static void Apply(Harmony harmony)
        {
            Patch(harmony, typeof(SaveLoadManager), "SaveInternal", nameof(SaveInternalPrefix), prefix: true);
            Patch(harmony, typeof(CampaignMenu), "Initialize", nameof(CampaignMenuInitializePostfix), prefix: false);
        }

        static void Patch(Harmony harmony, Type type, string method, string patch, bool prefix)
        {
            try
            {
                var target = AccessTools.Method(type, method);
                if (target == null) { logger.LogWarning($"[SaveTidy] {type.Name}.{method} not found - not patched"); return; }
                var hm = new HarmonyMethod(typeof(SaveTidy), patch);
                if (prefix) harmony.Patch(target, prefix: hm); else harmony.Patch(target, postfix: hm);
            }
            catch (Exception e) { logger.LogError($"[SaveTidy] patching {type.Name}.{method}: {e}"); }
        }

        // ============================================================ 1. no autosaves in training

        static int skipped_autosaves;

        // SaveInternal(SavedGameType saveType, OnLoadOrSaveCampaignFinishedCallback callback, AutoSaveEvents event):
        // the timer's autosave and AutoSave(event) both land here with saveType Auto. Autosaves that
        // pass a callback (a flow waiting on the save, e.g. leaving to the main menu) still run.
        static bool SaveInternalPrefix(SavedGameType __0, OnLoadOrSaveCampaignFinishedCallback __1, SaveLoadManager.AutoSaveEvents __2)
        {
            if (__0 != SavedGameType.Auto || __1 != null) return true;
            var training = Fido.Training.TrainingPilot.Instance;
            if (training == null || !training.isRunning) return true;
            skipped_autosaves++;
            logger.LogInfo($"[SaveTidy] training: autosave ({__2}) skipped - {skipped_autosaves} so far");
            return false;
        }

        // ============================================================ 2. "Clear autosaves" button

        const string ButtonName = "Fido_ClearAutosavesButton";

        static void CampaignMenuInitializePostfix(CampaignMenu __instance)
        {
            try { AddButton(__instance); }
            catch (Exception e) { logger.LogError("[SaveTidy] adding the Clear autosaves button: " + e); }
        }

        static void AddButton(CampaignMenu menu)
        {
            var delete = AccessTools.Field(typeof(CampaignMenu), "_deleteCampaignButton")?.GetValue(menu) as Button;
            if (delete == null) { logger.LogWarning("[SaveTidy] campaign Delete button not found - no Clear autosaves button"); return; }
            var parent = delete.transform.parent;
            if (parent.Find(ButtonName) != null) return;

            // Clone under an inactive holder so none of the copy's Awake/OnEnable run, then strip the
            // game's data binding (which would wire the copy to "delete campaign") and localisation
            // (which would put the Delete label back) before it goes live.
            var holder = new GameObject("Fido_CloneHolder");
            holder.SetActive(false);
            var go = UnityEngine.Object.Instantiate(delete.gameObject, holder.transform, false);
            go.name = ButtonName;
            foreach (var mb in go.GetComponentsInChildren<MonoBehaviour>(true))
            {
                string ns = mb.GetType().Namespace ?? "";
                if (ns.StartsWith("KSP.UI.Binding") || ns.StartsWith("I2.Loc") || mb.GetType().Name.Contains("Localiz"))
                    UnityEngine.Object.DestroyImmediate(mb);
            }
            var button = go.GetComponent<Button>();
            button.onClick = new Button.ButtonClickedEvent();
            var clearer = go.AddComponent<AutosaveClearer>();
            clearer.Init(menu, button);

            go.transform.SetParent(parent, false);
            go.transform.SetSiblingIndex(delete.transform.GetSiblingIndex() + 1);
            UnityEngine.Object.Destroy(holder);
            // no layout group to place it: sit it just left of Delete
            if (parent.GetComponent<LayoutGroup>() == null)
            {
                var src = (RectTransform)delete.transform;
                var rt = (RectTransform)go.transform;
                rt.anchoredPosition = src.anchoredPosition - new Vector2(src.rect.width + 12, 0);
            }
            go.SetActive(true);
            logger.LogInfo($"[SaveTidy] Clear autosaves button added under '{parent.name}' " +
                           $"(layout group: {(parent.GetComponent<LayoutGroup>() != null ? "yes" : "no")})");
        }

        /// <summary>Autosave files (json/meta/jpg) in a campaign folder: the game's autosave_N and autosaveevent_* names.</summary>
        public static string[] AutosaveFiles(string folder)
        {
            if (string.IsNullOrEmpty(folder) || !Directory.Exists(folder)) return new string[0];
            return Directory.GetFiles(folder)
                .Where(f => Path.GetFileName(f).StartsWith("autosave", StringComparison.OrdinalIgnoreCase))
                .ToArray();
        }

        /// <summary>
        /// Deletes the folder's autosaves. If nothing but autosaves is there, the newest is kept so
        /// the campaign still has something to load. Returns the number of saves deleted.
        /// </summary>
        public static int ClearAutosaves(string folder)
        {
            var files = AutosaveFiles(folder);
            var saves = files.Where(f => f.EndsWith(".json", StringComparison.OrdinalIgnoreCase))
                             .OrderByDescending(File.GetLastWriteTimeUtc).ToList();
            bool other_saves = Directory.GetFiles(folder, "*.json")
                .Any(f => !Path.GetFileName(f).StartsWith("autosave", StringComparison.OrdinalIgnoreCase));
            string keep = !other_saves && saves.Count > 0 ? Path.GetFileNameWithoutExtension(saves[0]) : null;

            int deleted = 0;
            foreach (var f in files)
            {
                if (keep != null && Path.GetFileNameWithoutExtension(f) == keep) continue;
                try
                {
                    File.Delete(f);
                    if (f.EndsWith(".json", StringComparison.OrdinalIgnoreCase)) deleted++;
                }
                catch (Exception e) { logger.LogWarning($"[SaveTidy] couldn't delete {Path.GetFileName(f)}: {e.Message}"); }
            }
            logger.LogInfo($"[SaveTidy] cleared {deleted} autosave(s) in {folder}" + (keep != null ? $", kept {keep} (no other saves)" : ""));
            return deleted;
        }

        class AutosaveClearer : MonoBehaviour
        {
            const float ConfirmSeconds = 4;
            CampaignMenu menu;
            TMPro.TMP_Text label;
            float confirm_until = -1;
            string pending_folder;

            public void Init(CampaignMenu menu, Button button)
            {
                this.menu = menu;
                label = GetComponentInChildren<TMPro.TMP_Text>(true);
                button.interactable = true;
                button.onClick.AddListener(OnClick);
                SetLabel("Clear autosaves");
            }

            void SetLabel(string text) { if (label != null) label.text = text; }

            string SelectedFolder()
            {
                var path = menu.CurrentSelectedTileEntry?.FileInfo?.Info.FilePath;
                return string.IsNullOrEmpty(path) ? null : Path.GetDirectoryName(path);
            }

            void OnClick()
            {
                string folder = SelectedFolder();
                if (folder == null) { SetLabel("Pick a campaign first"); confirm_until = Time.unscaledTime + 2; pending_folder = null; return; }
                int n = AutosaveFiles(folder).Count(f => f.EndsWith(".json", StringComparison.OrdinalIgnoreCase));

                if (pending_folder != folder || Time.unscaledTime > confirm_until)
                {
                    if (n == 0) { SetLabel("No autosaves"); confirm_until = Time.unscaledTime + 2; pending_folder = null; return; }
                    pending_folder = folder;
                    confirm_until = Time.unscaledTime + ConfirmSeconds;
                    SetLabel($"Delete {n} autosaves?");
                    return;
                }

                pending_folder = null;
                confirm_until = -1;
                int deleted = ClearAutosaves(folder);
                SetLabel($"Deleted {deleted}");
                confirm_until = Time.unscaledTime + 2;
                // the load menu re-reads the active campaign's folder from disk
                try
                {
                    var load = AccessTools.Field(typeof(CampaignMenu), "_campaignLoadMenu")?.GetValue(menu) as CampaignLoadMenu;
                    load?.SyncVisuals();
                }
                catch (Exception e) { logger.LogWarning("[SaveTidy] refreshing the save list: " + e.Message); }
            }

            void Update()
            {
                if (confirm_until > 0 && Time.unscaledTime > confirm_until)
                {
                    confirm_until = -1;
                    pending_folder = null;
                    SetLabel("Clear autosaves");
                }
            }
        }
    }
}
