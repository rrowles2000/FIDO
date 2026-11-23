using System;
using System.Collections.Generic;
using System.Linq;
using K2UI;
using KSP.Game;
using KSP.UI.Binding;
using UitkForKsp2.API;
using UnityEngine;
using UnityEngine.UIElements;

namespace K2D2.Mission
{
    /// <summary>
    /// The VAB's "K2-D2 Mission" window (OAB app bar button): edits the plan stored on the craft's
    /// command part, so it is saved with the craft. Built in code; borrows the main window's styles.
    /// </summary>
    public class MissionVabWindow : MonoBehaviour
    {
        public const string ButtonId = "BTN-K2D2Mission";

        VisualElement root, shell;
        Label info_label, dv_label;
        Button clear_button;
        MissionEditor editor;
        bool styled;
        K2D2.UI.ResizeManipulator resize;

        PlanHolder holder;
        Data_K2Mission bound_data;
        MissionPlanData plan;
        float next_scan;

        bool _open;
        public bool IsOpen
        {
            get => _open;
            set
            {
                _open = value;
                if (value) Scan(true);
                root?.Show(value);
                GameObject.Find(ButtonId)?.GetComponent<UIValue_WriteBool_Toggle>()?.SetValue(value);
            }
        }

        public static MissionVabWindow Instance { get; private set; }

        /// <summary>Opens the window (the part window's "Edit mission" button).</summary>
        public void Open() => IsOpen = true;

        public void Rescan() => Scan(true);

        public static MissionVabWindow Create()
        {
            var options = new WindowOptions
            {
                WindowId = "K2D2Mission",
                UseStockScale = true,
                Parent = null,
                IsHidingEnabled = true,
                DisableGameInputForTextFields = true,
                // moved by DragManipulator below, like the main window
                MoveOptions = new MoveOptions { IsMovingEnabled = false, CheckScreenBounds = true },
            };
            var ui = new VisualElement();
            var document = Window.Create(options, ui);
            var window = document.gameObject.AddComponent<MissionVabWindow>();
            window.Build(ui);
            Instance = window;
            return window;
        }

        void Build(VisualElement ui)
        {
            root = ui;
            K2UI.HoverTip.StartSweeping(ui);
            shell = new UitkForKsp2.Controls.AppShell { Title = "K2-D2 Mission" };
            shell.style.position = Position.Absolute;
            shell.style.left = 380;
            shell.style.top = 80;
            shell.style.width = 380;
            shell.style.height = 560;
            ((UitkForKsp2.Controls.AppShell)shell).CloseClicked += () => IsOpen = false;
            root.Add(shell);

            var scroll = new ScrollView();
            scroll.style.flexGrow = 1;
            shell.Add(scroll);

            info_label = new Label();
            info_label.style.whiteSpace = WhiteSpace.Normal;
            info_label.AddToClassList("k2-slider-value-label");
            scroll.Add(info_label);
            dv_label = new Label();
            dv_label.AddToClassList("k2-slider-value-label");
            scroll.Add(dv_label);

            editor = new MissionEditor
            {
                get_plan = () => plan,
                on_changed = _ => Save(),
                estimates = Estimates,
                part_count = Module_K2ActionTarget.CountInVab,
                sat_count = Module_K2ActionTarget.MarkedSatellitesInVab,
                context_signature = () => Module_K2ActionTarget.MarkedSatellitesInVab().ToString(),
                launch_first = () => true, // a VAB craft is still to launch
            };
            scroll.Add(editor.Build());

            clear_button = new Button(ClearPlan) { text = "Clear the whole plan" };
            clear_button.style.height = 26;
            clear_button.style.marginTop = 10;
            scroll.Add(clear_button);

            // same drag/resize as the main window, with its own saved position and size
            shell.style.minWidth = 300;
            shell.style.minHeight = 250;
            shell.AddManipulator(new K2D2.UI.DragManipulator(false, "mission_window_pos"));
            var handle = new VisualElement { name = "resize-handle", pickingMode = PickingMode.Position };
            handle.AddToClassList("window-resize-handle");
            var icon = new VisualElement { name = "resize-handle-icon", pickingMode = PickingMode.Ignore };
            icon.AddToClassList("window-resize-handle-icon");
            handle.Add(icon);
            shell.Add(handle);
            resize = new K2D2.UI.ResizeManipulator(shell, "mission_window_size");
            handle.AddManipulator(resize);

            root.Show(false);
        }

        // the main window's UXML carries the K2-D2 / Kerbal UI stylesheets - reuse them
        void BorrowStyles()
        {
            if (styled) return;
            var main = K2D2.UI.K2D2Window.Instance?.GetComponent<UIDocument>()?.rootVisualElement;
            if (main == null) return;
            var sheets = new List<StyleSheet>();
            foreach (var el in main.Query<VisualElement>().ToList().Prepend(main))
                for (int i = 0; i < el.styleSheets.count; i++)
                    if (!sheets.Contains(el.styleSheets[i]))
                        sheets.Add(el.styleSheets[i]);
            foreach (var sheet in sheets)
                shell.styleSheets.Add(sheet);
            styled = true;
        }

        static List<double> Estimates(MissionPlanData plan)
        {
            double mu = 3.5316e12, radius = 600000;
            try
            {
                var home = GameManager.Instance?.Game?.UniverseModel?.HomeWorld;
                if (home != null)
                {
                    mu = home.gravParameter;
                    radius = home.radius;
                }
            }
            catch (Exception) { }
            return MissionValidator.Estimates(plan, mu, radius);
        }

        // ---------------------------------------------------------------- craft

        void Scan(bool force)
        {
            holder = MissionStore.FindInVab(out bool has_plan);
            if (!force && holder?.data == bound_data) return;
            bound_data = holder?.data;
            plan = has_plan ? holder.Read() : null;
            if (plan != null && plan.Started)
            {
                // a craft file is a fresh mission - drop any progress it was saved with
                plan.ResetProgress();
                Save();
            }
            if (plan == null && holder != null) plan = new MissionPlanData();
            editor?.Refresh(true);
        }

        void Save() => holder?.Write(plan);

        /// <summary>A part window's "Add action to mission" (see Module_K2ActionTarget).</summary>
        public void AddPartAction(PartActionData action)
        {
            Scan(true);
            if (plan == null)
            {
                Debug.LogWarning("[K2D2 Mission] can't add a part action: the craft has no command part to carry the mission");
                return;
            }
            if (!_open) IsOpen = true;
            editor.AddPartAction(action);
        }

        void ClearPlan()
        {
            if (plan == null) return;
            plan.steps.Clear();
            plan.ResetProgress();
            Save();
            editor.Refresh(true);
        }

        static bool InVab()
        {
            try { return GameManager.Instance.Game.GlobalGameState.GetGameState().GameState == GameState.VehicleAssemblyBuilder; }
            catch (Exception) { return false; }
        }

        void Update()
        {
            if (!_open) return;
            if (!InVab())
            {
                IsOpen = false;
                return;
            }
            BorrowStyles();
            // see K2D2Window.Update: ends a resize drag the UI events failed to finish
            resize?.Tick();
            if (Time.time >= next_scan)
            {
                next_scan = Time.time + 0.5f;
                Scan(false);
            }

            if (holder == null)
                info_label.text = "Add a command part (pod or probe core) - the mission is stored on it.";
            else
                info_label.text = $"Saved on {holder.PartName} with the craft - to store it on another command part, " +
                                  "tick 'Carries the mission' in that part's window. Steps run from the Mission tab in " +
                                  "flight; tick a step to run it straight on from the one before.";
            double dv = MissionStore.MainAssembly?.VesselDeltaV?.TotalDeltaVVac ?? double.NaN;
            dv_label.text = double.IsNaN(dv) ? "" : $"Craft Δv (vac, all stages): {dv:n0} m/s";
            clear_button.Show(plan != null && !plan.IsEmpty);
            editor.Refresh();
        }
    }
}
