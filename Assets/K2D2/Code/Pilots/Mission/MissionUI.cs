using System;
using System.Collections.Generic;
using K2D2.Constellation;
using K2D2.Lift;
using K2D2.UI;
using K2UI;
using K2UI.Tabs;
using UnityEngine.UIElements;

namespace K2D2.Mission
{
    /// <summary>
    /// The "Mission" tab (first tab): the craft's step list, the current step's confirm screen
    /// (Go / Skip / manual control) and the plan editor. Built in code like the Sats tab.
    /// </summary>
    class MissionUI : K2Page
    {
        public const string Code = "mission";

        readonly MissionPilot pilot;

        public MissionUI(MissionPilot pilot)
        {
            this.pilot = pilot;
            code = Code;
        }

        public static TabPage CreateTabPage() => new TabPage { name = Code, label = "Mission" };

        FullStatus status_bar;
        VisualElement no_plan_el, plan_el, current_el;
        Label current_title, current_detail, dv_label, advice_label;
        ToggleButton run_button;
        Button skip_button, manual_button, reset_button;
        MissionEditor editor;

        public override bool onInit()
        {
            var page = new VisualElement { name = "page" };
            page.style.flexGrow = 1;
            panel.Add(page);

            var title = new Label("Mission");
            title.AddToClassList("unselectedContent");
            title.style.width = Length.Percent(100);
            title.style.unityTextAlign = UnityEngine.TextAnchor.UpperCenter;
            page.Add(title);

            // ---- no plan
            no_plan_el = new VisualElement();
            var none = new Label("This craft has no mission plan. Plans are made in the VAB (K2-D2 Mission " +
                                 "button) and saved with the craft, or you can start one here.");
            none.style.whiteSpace = WhiteSpace.Normal;
            no_plan_el.Add(none);
            no_plan_el.Add(SmallButton("Create a mission here", () =>
            {
                pilot.CreatePlan();
                pilot.PlanChanged();
            }));
            page.Add(no_plan_el);

            plan_el = new VisualElement();
            page.Add(plan_el);

            // ---- current step
            current_el = new VisualElement();
            current_el.style.marginBottom = 6;
            current_title = new Label();
            current_title.style.unityFontStyleAndWeight = UnityEngine.FontStyle.Bold;
            current_title.style.whiteSpace = WhiteSpace.Normal;
            current_el.Add(current_title);
            current_detail = new Label();
            current_detail.AddToClassList("k2-slider-value-label");
            current_detail.style.whiteSpace = WhiteSpace.Normal;
            current_el.Add(current_detail);
            dv_label = new Label();
            dv_label.AddToClassList("k2-slider-value-label");
            current_el.Add(dv_label);
            advice_label = new Label();
            advice_label.AddToClassList("k2-slider-value-label");
            advice_label.style.whiteSpace = WhiteSpace.Normal;
            advice_label.style.color = new UnityEngine.Color(1f, 0.85f, 0.5f);
            current_el.Add(advice_label);


            run_button = new ToggleButton { label = "Go", name = "run" };
            run_button.AddToClassList("big");
            run_button.style.marginTop = 8;
            run_button.style.marginBottom = 4;
            run_button.listeners += v =>
            {
                if (v && !pilot.isRunning) pilot.Go();
                else if (!v && pilot.isRunning) pilot.Stop();
                run_button.Value = pilot.isRunning;
                run_button.label = pilot.isRunning ? "Stop" : "Go";
            };
            pilot.is_running_event += running =>
            {
                run_button.Value = running;
                run_button.label = running ? "Stop" : "Go";
                status_bar?.avatar?.SetRunning(running);
            };
            current_el.Add(run_button);

            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            skip_button = SmallButton("Skip step", pilot.Skip);
            manual_button = SmallButton("Manual control", OpenStepTab);
            manual_button.tooltip = "Open the tab that flies this step";
            row.Add(skip_button);
            row.Add(manual_button);
            current_el.Add(row);
            plan_el.Add(current_el);

            plan_el.Add(ConstellationUI.StatusGroup());

            // ---- steps
            var steps = new Foldout { text = "STEPS", value = true };
            editor = new MissionEditor
            {
                get_plan = () => pilot.plan,
                on_changed = i => pilot.PlanChanged(i),
                editable = i => pilot.Editable(i),
                current_index = () => pilot.plan == null || pilot.IsDone ? -1 : pilot.plan.current,
                estimates = MissionPilot.Estimates,
                stage_budget = pilot.Budget,
                parking_period = MissionPilot.ParkingPeriod,
                part_count = tag => PartActionCatalog.FindInFlight(K2D2_Plugin.Instance.current_vessel?.VesselComponent, tag).Count,
                sat_count = () => K2D2.Constellation.ConstellationVessel.FindSatellites(K2D2_Plugin.Instance.current_vessel?.VesselComponent).Count,
                current_body = () => K2D2_Plugin.Instance.current_vessel?.VesselComponent?.mainBody,
                launch_first = MissionPilot.OnGround,
            };
            steps.Add(editor.Build());
            plan_el.Add(steps);

            reset_button = SmallButton("Reset mission to step 1", pilot.ResetMission);
            reset_button.style.marginTop = 8;
            plan_el.Add(reset_button);

            status_bar = new FullStatus(plan_el);
            return true;
        }

        static Button SmallButton(string text, Action click)
        {
            var b = new Button(click) { text = text };
            b.style.height = 26;
            b.style.flexGrow = 1;
            return b;
        }

        void OpenStepTab()
        {
            string tab = pilot.Runner?.Tab;
            if (tab == null) return;
            if (pilot.isRunning) pilot.Stop();
            K2D2Window.Instance?.SelectTab(tab);
        }

        public override bool onUpdateUI()
        {
            if (!base.onUpdateUI())
                return false;

            var plan = pilot.plan;
            no_plan_el.Show(plan == null);
            plan_el.Show(plan != null);
            status_bar.Reset();
            if (plan == null) return true;

            editor.Refresh();

            var step = plan.Current;
            bool running = pilot.isRunning;
            if (step == null || pilot.IsDone)
            {
                current_title.text = plan.IsEmpty ? "Add some steps below" : "Mission complete";
                current_detail.text = "";
                dv_label.text = "";
                advice_label.text = "";
                advice_label.Show(false);
                run_button.Show(false);
                skip_button.Show(false);
                manual_button.Show(false);
            }
            else
            {
                current_title.text = $"Step {plan.current + 1} of {plan.steps.Count}: {step.Summary}";
                current_detail.text = running ? pilot.Runner?.status ?? "" : step.auto ? "Auto-continue" : "Waiting for Go";
                var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
                double dv = vessel?.VesselDeltaV?.TotalDeltaVVac ?? 0;
                dv_label.text = $"Vessel Δv (vac): {dv:n0} m/s";
                string advice = null;
                if (!running)
                {
                    try { advice = pilot.Runner?.Advice(); }
                    catch (Exception e) { advice = "Couldn't check this step: " + e.Message; }
                }
                advice_label.text = advice ?? "";
                advice_label.Show(!string.IsNullOrEmpty(advice));
                run_button.Show(true);
                run_button.SetEnabled(running || pilot.GoProblem() == null);
                skip_button.Show(true);
                manual_button.Show(pilot.Runner?.Tab != null);
            }
            reset_button.Show(!running && plan.Started);

            if (!string.IsNullOrEmpty(pilot.status_line))
                status_bar.Status(pilot.status_line, pilot.status_ok ? StatusLine.Level.Normal : StatusLine.Level.Warning);
            else if (step != null && !pilot.IsDone)
                status_bar.Status($"Ready: step {plan.current + 1} - press Go");
            if (!pilot.CanStoreOnVessel)
                status_bar.Console("No command part with the mission module on this vessel - changes won't be saved");
            return true;
        }
    }
}
