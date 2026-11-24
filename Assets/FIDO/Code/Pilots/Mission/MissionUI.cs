using System;
using System.Collections.Generic;
using Fido.Constellation;
using Fido.Lift;
using Fido.UI;
using K2UI;
using K2UI.Tabs;
using UnityEngine.UIElements;

namespace Fido.Mission
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
        Label current_title, current_detail, advice_label, head_note;
        Pill state_pill;
        Readout dv_readout, step_readout;
        ToggleButton run_button;
        Button skip_button, manual_button, reset_button;
        MissionEditor editor;

        public override bool onInit()
        {
            var page = new VisualElement { name = "page" };
            page.style.flexGrow = 1;
            panel.Add(page);

            // ---- header: state pill + where we are
            var head = FidoKit.Row(page, 6);
            head.style.alignItems = Align.Center;
            head.style.flexWrap = Wrap.NoWrap;
            state_pill = new Pill(head);
            head_note = FidoKit.Hint(head);
            head_note.style.flexShrink = 1;

            // ---- no plan
            no_plan_el = FidoKit.Panel(page, "Mission");
            FidoKit.Text(no_plan_el, "This craft has no mission plan. Plans are made in the VAB (FIDO Mission " +
                                     "button) and saved with the craft, or you can start one here.");
            no_plan_el.Add(SmallButton("Create a mission here", () =>
            {
                pilot.CreatePlan();
                pilot.PlanChanged();
            }));

            plan_el = new VisualElement();
            page.Add(plan_el);

            // ---- current step
            current_el = FidoKit.Panel(null, "Current step");
            current_title = FidoKit.Text(current_el, "", bold: true);
            current_detail = FidoKit.Hint(current_el);
            var readouts = FidoKit.Row(current_el, 0);
            readouts.style.marginTop = 4;
            step_readout = new Readout("Step", readouts);
            dv_readout = new Readout("Vessel Δv (vac)", readouts);
            advice_label = FidoKit.Text(current_el);
            FidoKit.SetTone(advice_label, Tone.Warn);


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
            var steps_panel = FidoKit.Panel(null);
            var steps = new Foldout { text = "STEPS", value = true };
            steps_panel.Add(steps);
            editor = new MissionEditor
            {
                get_plan = () => pilot.plan,
                on_changed = i => pilot.PlanChanged(i),
                editable = i => pilot.Editable(i),
                current_index = () => pilot.plan == null || pilot.IsDone ? -1 : pilot.plan.current,
                estimates = MissionPilot.Estimates,
                stage_budget = pilot.Budget,
                parking_period = MissionPilot.ParkingPeriod,
                part_count = tag => PartActionCatalog.FindInFlight(FidoPlugin.Instance.current_vessel?.VesselComponent, tag).Count,
                sat_count = () => Fido.Constellation.ConstellationVessel.FindSatellites(FidoPlugin.Instance.current_vessel?.VesselComponent).Count,
                current_body = () => FidoPlugin.Instance.current_vessel?.VesselComponent?.mainBody,
                launch_first = MissionPilot.OnGround,
            };
            steps.Add(editor.Build());
            plan_el.Add(steps_panel);

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
            FidoWindow.Instance?.SelectTab(tab);
        }

        public override bool onUpdateUI()
        {
            if (!base.onUpdateUI())
                return false;

            var plan = pilot.plan;
            no_plan_el.Show(plan == null);
            plan_el.Show(plan != null);
            status_bar.Reset();
            if (plan == null)
            {
                state_pill.Set("No plan", Tone.Muted);
                head_note.text = "";
                return true;
            }

            editor.Refresh();

            var step = plan.Current;
            bool running = pilot.isRunning;
            if (step == null || pilot.IsDone)
            {
                current_title.text = plan.IsEmpty ? "Add some steps below" : "Mission complete";
                current_detail.text = "";
                state_pill.Set(plan.IsEmpty ? "Empty" : "Complete", plan.IsEmpty ? Tone.Muted : Tone.Ok);
                head_note.text = $"{plan.steps.Count} step(s)";
                step_readout.Set(plan.IsEmpty ? "-" : $"{plan.steps.Count} / {plan.steps.Count}");
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
                var vessel = FidoPlugin.Instance.current_vessel?.VesselComponent;
                double dv = vessel?.VesselDeltaV?.TotalDeltaVVac ?? 0;
                dv_readout.Set($"{dv:n0} m/s");
                step_readout.Set($"{plan.current + 1} / {plan.steps.Count}", running ? Tone.Run : Tone.Normal);
                bool blocked = !running && pilot.GoProblem() != null;
                state_pill.Set(running ? "Running" : blocked ? "Blocked" : "Ready",
                               running ? Tone.Run : blocked ? Tone.Warn : Tone.Normal);
                head_note.text = step.Summary;
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
