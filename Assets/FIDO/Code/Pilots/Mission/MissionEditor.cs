using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using K2UI;
using UitkForKsp2.API;
using UnityEngine;
using UnityEngine.UIElements;

namespace Fido.Mission
{
    /// <summary>
    /// The step list editor, shared by the VAB Mission window and the flight Mission tab: add, remove,
    /// reorder, auto-continue and per-step settings, plus the plan checks and rough Δv estimates.
    /// </summary>
    public class MissionEditor
    {
        public Func<MissionPlanData> get_plan;
        // called after every edit with the index of the edited step (-1 = list structure)
        public Action<int> on_changed;
        // which steps may be edited (flight: not the running one or any already flown)
        public Func<int, bool> editable = _ => true;
        // the step to mark as current (flight only)
        public Func<int> current_index = () => -1;
        // rough Δv per step, NaN where there's no estimate
        public Func<MissionPlanData, List<double>> estimates;
        // how many parts carry a part-action tag (VAB craft or flight vessel); null = don't check
        public Func<string, int> part_count;
        // satellites on the craft for the Deploy plan, and the body in flight (null in the VAB)
        public Func<int> sat_count;
        public Func<KSP.Sim.impl.CelestialBodyComponent> current_body;
        // anything outside the plan that changes what the view shows (e.g. satellite marks)
        public Func<string> context_signature;
        // the craft is still to launch (VAB, or in flight on the pad / landed): the plan must start with
        // Launch + Circularise, so Go on the pad never runs another step (e.g. a decouple) first
        public Func<bool> launch_first = () => false;
        // the stages the plan's burns come out of (see MissionValidator.BudgetFrom); null = no stage checks
        public Func<MissionPlanData, StageBudget> stage_budget;
        // the parking orbit's period, for the long-burn warning
        public Func<MissionPlanData, double> parking_period;
        // the stage budget changes without the plan changing (fuel burned, a stage dropped): re-read it
        // every couple of seconds and rebuild only if it changed (a rebuild takes focus off a text field)
        float next_stage_check;
        StageBudget budget;
        string budget_sig = "";

        public VisualElement root;
        VisualElement list_el, issues_el, add_row;
        DropdownField type_dropdown;

        // the last step clicked (Add step inserts after it) and the step whose settings are open
        // (concertina: opening one closes the others, so it holds at most one index)
        int selected = -1;
        readonly HashSet<int> expanded = new();
        string shown_signature = "";

        public VisualElement Build()
        {
            root = new VisualElement { name = "mission-editor" };

            list_el = new VisualElement();
            list_el.AddToClassList("advanced-info-table");
            root.Add(list_el);

            add_row = new VisualElement();
            add_row.style.flexDirection = FlexDirection.Row;
            add_row.style.alignItems = Align.Center;
            add_row.style.marginTop = 4;
            type_dropdown = new DropdownField(StepCatalog.All.Select(t => t.label).ToList(), 0);
            type_dropdown.style.flexGrow = 1;
            add_row.Add(type_dropdown);
            var add = new Button(AddStep) { text = "Add step" };
            add.style.height = 24;
            add_row.Add(add);
            root.Add(add_row);

            issues_el = new VisualElement();
            issues_el.style.marginTop = 6;
            root.Add(issues_el);

            Refresh(true);
            return root;
        }

        public int Selected => selected;

        // ================================================================ edits

        /// <summary>
        /// For a craft still to launch (launch_first): puts Launch + Circularise at the top of a plan that
        /// has no Launch step (after any leading quicksaves), and returns the first index a new step may
        /// take - after that pair. Otherwise returns 0. (2026-11-23: a plan that started with a part
        /// action decoupled on the pad when Go was pressed.)
        /// </summary>
        int EnsureLaunchFirst(MissionPlanData plan, bool adding_launch)
        {
            if (!launch_first()) return 0;
            int lead = 0;
            while (lead < plan.steps.Count && plan.steps[lead].type == StepCatalog.Quicksave) lead++;
            int launch = plan.steps.FindIndex(x => x.type == StepCatalog.Launch);
            if (launch < 0)
            {
                if (adding_launch) return lead; // the step being added is the Launch: it goes first
                ShiftExpanded(lead, 2);
                plan.steps.Insert(lead, StepCatalog.Get(StepCatalog.Launch).NewStep());
                plan.steps.Insert(lead + 1, StepCatalog.Get(StepCatalog.Circularise).NewStep());
                if (selected >= lead) selected += 2;
                launch = lead;
            }
            bool circ = launch + 1 < plan.steps.Count && plan.steps[launch + 1].type == StepCatalog.Circularise;
            return launch + (circ ? 2 : 1);
        }

        void AddStep()
        {
            var plan = get_plan();
            if (plan == null) return;
            var type = StepCatalog.All.FirstOrDefault(t => t.label == type_dropdown.value);
            if (type == null) return;
            int after_launch = EnsureLaunchFirst(plan, type.id == StepCatalog.Launch);
            int first_editable = Enumerable.Range(0, plan.steps.Count + 1).FirstOrDefault(i => i >= plan.steps.Count || editable(i));
            int at = selected >= 0 && selected < plan.steps.Count ? selected + 1 : plan.steps.Count;
            at = Math.Max(at, first_editable);
            if (type.id != StepCatalog.Launch) at = Math.Max(at, after_launch);
            else if (launch_first() && !plan.steps.Any(x => x.type == StepCatalog.Launch)) at = after_launch; // first, after quicksaves
            ShiftExpanded(at, 1);
            var added = type.NewStep();
            // a correction (or another transfer) is usually for the transfer before it: copy its target
            if (type.id == StepCatalog.Correction || type.id == StepCatalog.Transfer || type.id == StepCatalog.Capture)
            {
                var before = plan.steps.Take(at).LastOrDefault(TransferChecks.PlacesNode);
                if (before != null)
                {
                    added.p["target"] = before.Get("target");
                    if (type.id == StepCatalog.Capture)
                    {
                        if (before.type == StepCatalog.Capture)
                        {
                            added.p["alt_km"] = before.Get("alt_km");
                            added.p["orbit"] = before.Get("orbit");
                        }
                        else added.p["alt_km"] = before.Get("pe_km");
                    }
                    else added.p["pe_km"] = before.type == StepCatalog.Capture ? before.Get("alt_km") : before.Get("pe_km");
                }
            }
            plan.steps.Insert(at, added);
            // Circularise has to follow Launch, so it comes with it
            if (type.id == StepCatalog.Launch &&
                (at + 1 >= plan.steps.Count || plan.steps[at + 1].type != StepCatalog.Circularise))
            {
                ShiftExpanded(at + 1, 1);
                plan.steps.Insert(at + 1, StepCatalog.Get(StepCatalog.Circularise).NewStep());
            }
            selected = at;
            Open(at);
            Changed(-1);
        }

        /// <summary>
        /// The part window's "Add action to mission": appends to the selected Part actions step,
        /// otherwise adds a new Part actions step after the selected step.
        /// </summary>
        public void AddPartAction(PartActionData action)
        {
            var plan = get_plan();
            if (plan == null) return;
            if (selected >= 0 && selected < plan.steps.Count && editable(selected) &&
                plan.steps[selected].type == StepCatalog.PartActions)
            {
                plan.steps[selected].actions.Add(action);
                Open(selected);
                Changed(selected);
                return;
            }
            int after_launch = EnsureLaunchFirst(plan, false);
            int first_editable = Enumerable.Range(0, plan.steps.Count + 1).FirstOrDefault(i => i >= plan.steps.Count || editable(i));
            int at = Math.Max(selected >= 0 && selected < plan.steps.Count ? selected + 1 : plan.steps.Count, first_editable);
            at = Math.Max(at, after_launch);
            var step = StepCatalog.Get(StepCatalog.PartActions).NewStep();
            step.actions.Add(action);
            ShiftExpanded(at, 1);
            plan.steps.Insert(at, step);
            selected = at;
            Open(at);
            Changed(-1);
        }

        void Remove(int i)
        {
            var plan = get_plan();
            if (plan == null || !editable(i)) return;
            plan.steps.RemoveAt(i);
            expanded.Remove(i);
            ShiftExpanded(i + 1, -1);
            if (selected >= plan.steps.Count) selected = plan.steps.Count - 1;
            Changed(-1);
        }

        void Move(int i, int delta)
        {
            var plan = get_plan();
            int j = i + delta;
            if (plan == null || j < 0 || j >= plan.steps.Count || !editable(i) || !editable(j)) return;
            (plan.steps[i], plan.steps[j]) = (plan.steps[j], plan.steps[i]);
            if (selected == i) selected = j;
            else if (selected == j) selected = i;
            bool open_i = expanded.Remove(i), open_j = expanded.Remove(j);
            if (open_i) expanded.Add(j);
            if (open_j) expanded.Add(i);
            Changed(-1);
        }

        void Open(int i)
        {
            expanded.Clear();
            expanded.Add(i);
        }

        // keeps open steps open when steps from 'from' onwards move by 'delta'
        void ShiftExpanded(int from, int delta)
        {
            var moved = expanded.Where(k => k >= from).ToList();
            foreach (var k in moved) expanded.Remove(k);
            foreach (var k in moved) expanded.Add(k + delta);
        }

        void Changed(int index)
        {
            on_changed?.Invoke(index);
            Refresh(true);
        }

        // ================================================================ view

        static string Signature(MissionPlanData plan, int current) =>
            (plan == null ? "none" : plan.ToJson()) + "|" + current;

        bool refreshing, refresh_again;

        /// <summary>Rebuilds the view if the plan changed (cheap to call every frame).</summary>
        public void Refresh(bool force = false)
        {
            // Clearing the old view can take focus off a text field, which commits its value and
            // calls back in here mid-rebuild; the two rebuilds then stacked their settings on top of
            // each other (Launch's settings shown three times). Never nest: rebuild again after.
            if (refreshing)
            {
                refresh_again = true;
                return;
            }
            refreshing = true;
            try
            {
                Rebuild(force);
                for (int guard = 0; refresh_again && guard < 3; guard++)
                {
                    refresh_again = false;
                    Rebuild(true);
                }
            }
            finally
            {
                refreshing = false;
                refresh_again = false;
            }
        }

        void Rebuild(bool force)
        {
            var plan = get_plan();
            int current = current_index();
            string sig = Signature(plan, current) + "|" + selected + "|" + string.Join(",", expanded.OrderBy(k => k)) +
                         "|" + context_signature?.Invoke();
            if (stage_budget != null && (force || Time.unscaledTime >= next_stage_check))
            {
                next_stage_check = Time.unscaledTime + 2;
                try { budget = plan == null ? null : stage_budget(plan); }
                catch (Exception e)
                {
                    budget = null;
                    Debug.LogWarning("[FIDO Mission] stage budget: " + e.Message);
                }
                budget_sig = budget == null ? "" : budget.source + budget.Describe() + StagingSettingsSig();
            }
            sig += "|" + budget_sig;
            if (!force && sig == shown_signature) return;
            shown_signature = sig;

            list_el.Clear();
            issues_el.Clear();
            add_row.Show(plan != null);
            if (plan == null) return;

            if (plan.steps.Count == 0)
                list_el.Add(new Label("No steps yet - pick one below and press Add step"));
            if (selected >= plan.steps.Count) selected = -1;
            expanded.RemoveWhere(k => k >= plan.steps.Count);

            var dv = estimates?.Invoke(plan);
            for (int i = 0; i < plan.steps.Count; i++)
            {
                list_el.Add(StepRow(plan, i, current, dv != null && i < dv.Count ? dv[i] : double.NaN));
                if (expanded.Contains(i))
                    list_el.Add(StepParams(plan, i));
            }

            if (dv != null)
            {
                var known = dv.Where(d => !double.IsNaN(d)).ToList();
                if (known.Count > 0)
                {
                    var total = new Label($"Δv after launch (rough): {known.Sum():n0} m/s");
                    total.AddToClassList("k2-slider-value-label");
                    total.style.marginTop = 2;
                    list_el.Add(total);
                }
            }

            var checks = budget == null
                ? MissionValidator.Check(plan)
                : MissionValidator.Check(plan, budget, dv, MissionValidator.BudgetFrom(plan, out _), parking_period?.Invoke(plan) ?? 0);
            if (budget != null && plan.steps.Any(TransferChecks.PlacesNode))
            {
                var l = new Label($"Stages for the burns: {budget.Describe()} ({budget.source})");
                l.AddToClassList("k2-slider-value-label");
                l.style.whiteSpace = WhiteSpace.Normal;
                issues_el.Add(l);
            }
            foreach (var issue in checks)
            {
                var l = new Label((issue.error ? "✖ " : "⚠ ") + issue.text);
                l.style.whiteSpace = WhiteSpace.Normal;
                l.style.color = issue.error ? new Color(1f, 0.45f, 0.4f) : new Color(1f, 0.8f, 0.35f);
                issues_el.Add(l);
            }
        }

        static string StagingSettingsSig() => Fido.Controller.StagingSettings.auto_staging.V ? "|as" : "|-";

        static string StateMark(MissionStepData s, bool is_current) =>
            s.state == "done" ? "✔" : s.state == "skipped" ? "↷" : s.state == "failed" ? "✖" : is_current ? "▶" : "·";

        VisualElement StepRow(MissionPlanData plan, int i, int current, double dv)
        {
            var s = plan.steps[i];
            bool can_edit = editable(i);

            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            row.style.alignItems = Align.Center;
            row.style.paddingTop = 1;
            row.style.paddingBottom = 1;
            if (i == selected)
                row.style.backgroundColor = new Color(1, 1, 1, 0.08f);

            var text = $"{(expanded.Contains(i) ? "▾" : "▸")} {StateMark(s, i == current)} {i + 1}. {s.Summary}";
            if (!double.IsNaN(dv) && dv > 0.5) text += $"  ~{dv:n0} m/s";
            var label = new Label(text);
            label.style.flexGrow = 1;
            label.style.flexShrink = 1;
            label.style.whiteSpace = WhiteSpace.Normal;
            if (s.state == "done" || s.state == "skipped") label.style.opacity = 0.6f;
            label.tooltip = string.IsNullOrEmpty(s.note) ? "Click to show or hide this step's settings" : s.note;
            void ToggleOpen()
            {
                selected = i;
                if (!expanded.Remove(i)) Open(i);
                Refresh(true);
            }
            label.RegisterCallback<ClickEvent>(_ => ToggleOpen());
            row.Add(label);

            // 2026-11-26: clicking the text was the only way in and the user couldn't find it - a visible button
            bool is_open = expanded.Contains(i);
            var edit = new Button(ToggleOpen)
            {
                text = is_open ? "Close" : can_edit ? "Edit" : "View",
                tooltip = is_open ? "Hide this step's settings" : can_edit ? "Edit this step's settings" : "Show this step's settings (locked)",
            };
            edit.style.height = 22;
            edit.style.minWidth = 44;
            row.Add(edit);

            var auto = new Toggle { text = "auto", value = s.auto, tooltip = "Auto-continue: run straight on from the previous step" };
            auto.SetEnabled(can_edit);
            auto.RegisterValueChangedCallback(e =>
            {
                s.auto = e.newValue;
                Changed(i);
            });
            row.Add(auto);

            var up = new Button(() => Move(i, -1)) { text = "▲" };
            var down = new Button(() => Move(i, 1)) { text = "▼" };
            var del = new Button(() => Remove(i)) { text = "✕", tooltip = "Remove this step" };
            up.SetEnabled(can_edit && i > 0 && editable(i - 1));
            down.SetEnabled(can_edit && i + 1 < plan.steps.Count);
            del.SetEnabled(can_edit);
            foreach (var b in new[] { up, down, del })
            {
                b.style.width = 22;
                b.style.height = 22;
                row.Add(b);
            }
            return row;
        }

        // the settings of one step, shown indented under its row
        VisualElement StepParams(MissionPlanData plan, int i)
        {
            var s = plan.steps[i];
            bool can_edit = editable(i);
            var params_el = new VisualElement();
            params_el.style.marginLeft = 14;
            params_el.style.marginBottom = 6;
            params_el.style.paddingLeft = 6;
            params_el.style.borderLeftWidth = 1;
            params_el.style.borderLeftColor = new Color(1, 1, 1, 0.25f);
            if (!can_edit)
            {
                var flown = new Label("Already flown - settings locked");
                flown.AddToClassList("k2-slider-value-label");
                params_el.Add(flown);
            }
            if (!string.IsNullOrEmpty(s.note))
            {
                var note = new Label(s.note);
                note.AddToClassList("k2-slider-value-label");
                note.style.whiteSpace = WhiteSpace.Normal;
                params_el.Add(note);
            }
            params_el.Add(TitleField(s, i, can_edit));
            var type = s.Type;
            if (type == null) return params_el;
            if (type.presets.Count > 0)
                params_el.Add(PresetRow(s, type, i, can_edit));
            if (type.parameters.Count == 0 && type.id != StepCatalog.PartActions)
            {
                var none = new Label("No settings");
                none.AddToClassList("k2-slider-value-label");
                params_el.Add(none);
            }
            foreach (var p in type.parameters)
            {
                VisualElement field = ParamField(s, p, i);
                field.SetEnabled(can_edit);
                params_el.Add(field);
                if (!string.IsNullOrEmpty(p.hint))
                {
                    var hint = new Label(p.hint);
                    hint.AddToClassList("k2-slider-value-label");
                    hint.style.whiteSpace = WhiteSpace.Normal;
                    params_el.Add(hint);
                }
            }
            if (type.id == StepCatalog.PartActions)
                params_el.Add(ActionList(s, i, can_edit));
            if (type.id == StepCatalog.Deploy)
                params_el.Add(DeployPlanner.Build(s, sat_count?.Invoke() ?? 0, current_body?.Invoke(), can_edit, () => Changed(i)));
            return params_el;
        }

        VisualElement TitleField(MissionStepData s, int i, bool can_edit)
        {
            var box = new VisualElement();
            var f = new TextField("Title") { isDelayed = true, value = s.title ?? "" };
            f.SetEnabled(can_edit);
            f.RegisterValueChangedCallback(e =>
            {
                s.title = e.newValue?.Trim() ?? "";
                Changed(i);
            });
            f.DisableGameInputOnFocus();
            box.Add(f);
            var hint = new Label($"blank = \"{s.DefaultSummary}\"");
            hint.AddToClassList("k2-slider-value-label");
            hint.style.whiteSpace = WhiteSpace.Normal;
            box.Add(hint);
            return box;
        }

        VisualElement ActionList(MissionStepData s, int i, bool can_edit)
        {
            var box = new VisualElement();
            box.style.marginTop = 4;
            var head = new Label(s.actions.Count == 0
                ? "No actions yet - in the VAB, click 'Add action to mission' in a part's window"
                : "Actions, in order:");
            head.style.whiteSpace = WhiteSpace.Normal;
            box.Add(head);
            for (int k = 0; k < s.actions.Count; k++)
            {
                var a = s.actions[k];
                int index = k;
                var row = new VisualElement();
                row.style.flexDirection = FlexDirection.Row;
                row.style.alignItems = Align.Center;

                int count = part_count?.Invoke(a.tag) ?? -1;
                var name = new Label($"{k + 1}. {a.part} #{a.tag}" +
                                     (count == 0 ? "  ✖ not on craft" : count > 1 ? $"  ×{count}" : ""));
                name.style.flexGrow = 1;
                name.style.flexShrink = 1;
                name.style.whiteSpace = WhiteSpace.Normal;
                if (count == 0) name.style.color = new Color(1f, 0.45f, 0.4f);
                if (count > 1) name.tooltip = "Several parts share this tag (symmetry or copies) - the action applies to all of them";
                row.Add(name);

                var options = PartActionCatalog.ActionsFor(a.kinds);
                var labels = options.Select(o => o.label).ToList();
                if (labels.Count > 0)
                {
                    var pick = new DropdownField(labels, Math.Max(0, options.FindIndex(o => o.id == a.action)));
                    pick.style.width = 130;
                    pick.SetEnabled(can_edit);
                    pick.RegisterValueChangedCallback(e =>
                    {
                        a.action = options.FirstOrDefault(o => o.label == e.newValue).id ?? a.action;
                        Changed(i);
                    });
                    row.Add(pick);
                }

                var up = new Button(() => { MoveAction(s, index, -1); Changed(i); }) { text = "▲" };
                var down = new Button(() => { MoveAction(s, index, 1); Changed(i); }) { text = "▼" };
                var del = new Button(() => { s.actions.RemoveAt(index); Changed(i); }) { text = "✕", tooltip = "Remove this action" };
                up.SetEnabled(can_edit && k > 0);
                down.SetEnabled(can_edit && k + 1 < s.actions.Count);
                del.SetEnabled(can_edit);
                foreach (var b in new[] { up, down, del })
                {
                    b.style.width = 22;
                    b.style.height = 22;
                    row.Add(b);
                }
                box.Add(row);
            }
            return box;
        }

        static void MoveAction(MissionStepData s, int k, int delta)
        {
            int j = k + delta;
            if (j < 0 || j >= s.actions.Count) return;
            (s.actions[k], s.actions[j]) = (s.actions[j], s.actions[k]);
        }

        // "Preset: [dropdown]" - picking a preset writes all of its values at once (2026-11-23: it used to
        // need a separate Set button, so picking North polar left the inclination at 0°, and the dropdown
        // showed the first preset even when the settings matched none of them)
        const string CustomPreset = "Custom";

        VisualElement PresetRow(MissionStepData s, StepType type, int i, bool can_edit)
        {
            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            row.style.alignItems = Align.Center;
            row.style.marginBottom = 2;
            var labels = type.presets.Select(p => p.label).ToList();
            int current = type.presets.FindIndex(p => p.Matches(s));
            if (current < 0) labels.Insert(0, CustomPreset);
            var dropdown = new DropdownField("Preset", labels, Math.Max(0, current));
            dropdown.style.flexGrow = 1;
            dropdown.tooltip = current >= 0 ? "Current settings match this preset" : "Current settings match no preset";
            dropdown.RegisterValueChangedCallback(e =>
            {
                var preset = type.presets.FirstOrDefault(p => p.label == e.newValue);
                if (preset == null) return; // "Custom"
                foreach (var kv in preset.values) s.p[kv.Key] = kv.Value;
                // rebuild after the dropdown's own callback has finished (the rebuild replaces it)
                root.schedule.Execute(() => Changed(i));
            });
            dropdown.SetEnabled(can_edit);
            row.Add(dropdown);
            return row;
        }

        VisualElement ParamField(MissionStepData s, ParamDef p, int i)
        {
            switch (p.kind)
            {
                case ParamKind.Choice:
                {
                    var options = p.choices?.Invoke() ?? new List<string>();
                    string value = s.Get(p.key);
                    if (!string.IsNullOrEmpty(value) && !options.Contains(value)) options.Insert(0, value);
                    if (options.Count == 0) options.Add(value ?? "");
                    var d = new DropdownField(p.label, options, Math.Max(0, options.IndexOf(value)));
                    d.RegisterValueChangedCallback(e =>
                    {
                        s.p[p.key] = e.newValue;
                        // rebuild after the dropdown's own callback has finished (the rebuild replaces it)
                        root.schedule.Execute(() => Changed(i));
                    });
                    return d;
                }
                case ParamKind.Bool:
                {
                    var t = new K2Toggle(p.label);
                    t.SetValueWithoutNotify(s.GetBool(p.key));
                    t.RegisterValueChangedCallback(e =>
                    {
                        s.Set(p.key, e.newValue);
                        Changed(i);
                    });
                    return t;
                }
                case ParamKind.Int:
                {
                    var f = new IntegerField(p.label) { isDelayed = true, value = s.GetInt(p.key) };
                    f.RegisterValueChangedCallback(e =>
                    {
                        s.Set(p.key, e.newValue);
                        Changed(i);
                    });
                    f.DisableGameInputOnFocus();
                    return f;
                }
                default:
                {
                    var f = new FloatField(p.label) { isDelayed = true, value = s.GetFloat(p.key) };
                    f.RegisterValueChangedCallback(e =>
                    {
                        s.Set(p.key, e.newValue);
                        Changed(i);
                    });
                    f.DisableGameInputOnFocus();
                    return f;
                }
            }
        }
    }
}
