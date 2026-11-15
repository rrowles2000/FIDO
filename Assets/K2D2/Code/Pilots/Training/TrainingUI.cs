using System;
using System.Collections.Generic;
using K2D2.Constellation;
using K2D2.Lift.Next;
using K2D2.Lift.Stage1;
using K2D2.UI;
using K2UI;
using K2UI.Tabs;
using UnityEngine.UIElements;

namespace K2D2.Training
{
    /// <summary>
    /// The "Training" tab: how many flights, Start/Stop, progress. Built in code like the Mission tab.
    /// </summary>
    class TrainingUI : K2Page
    {
        public const string Code = "training";

        readonly TrainingPilot pilot;

        public TrainingUI(TrainingPilot pilot)
        {
            this.pilot = pilot;
            code = Code;
        }

        public const string LiveViewUrl = "http://localhost:8642/";

        public static TabPage CreateTabPage() => new TabPage { name = Code, label = "Training" };

        FullStatus status_bar;
        Label vehicle_label, results_label;
        IntegerField flights_field;
        ToggleButton run_button;

        public override bool onInit()
        {
            var page = new VisualElement { name = "page" };
            page.style.flexGrow = 1;
            panel.Add(page);

            var title = new Label("Training");
            title.AddToClassList("unselectedContent");
            title.style.width = Length.Percent(100);
            title.style.unityTextAlign = UnityEngine.TextAnchor.UpperCenter;
            page.Add(title);

            Hint(page).text = "Flies this vehicle from the pad to a circular orbit, reverts to launch and " +
                              "goes again, so the lift system can learn it.";

            vehicle_label = new Label();
            vehicle_label.style.unityFontStyleAndWeight = UnityEngine.FontStyle.Bold;
            vehicle_label.style.whiteSpace = WhiteSpace.Normal;
            vehicle_label.style.marginTop = 6;
            page.Add(vehicle_label);

            flights_field = new IntegerField("Flights") { isDelayed = true };
            flights_field.Bind(pilot.flights);
            flights_field.style.marginTop = 6;
            page.Add(flights_field);

            run_button = new ToggleButton { label = "Start", name = "run" };
            run_button.AddToClassList("big");
            run_button.style.marginTop = 8;
            run_button.style.marginBottom = 4;
            run_button.listeners += v =>
            {
                if (v && !pilot.isRunning) pilot.Begin();
                else if (!v && pilot.isRunning) pilot.Stop();
                SyncButton(pilot.isRunning);
            };
            pilot.is_running_event += running =>
            {
                SyncButton(running);
                status_bar?.avatar?.SetRunning(running);
            };
            page.Add(run_button);

            // the live view: a page served outside the game by ksp2-redux/tools/liveview/liveview.py
            // from Ksp2.log, for a second screen. The game (under Proton) can open a URL but can't
            // reliably start a Linux program, so the server is started outside it.
            var live = new Button(() => UnityEngine.Application.OpenURL(LiveViewUrl)) { text = "Open live view in browser" };
            live.style.height = 24;
            page.Add(live);
            Hint(page).text = $"Live flight and progress page at {LiveViewUrl} - start it first with " +
                              "python3 tools/liveview/liveview.py (in ksp2-redux).";

            page.Add(ConstellationUI.StatusGroup());

            results_label = Hint(page);
            results_label.style.marginTop = 6;

            status_bar = new FullStatus(page);
            return true;
        }

        void SyncButton(bool running)
        {
            run_button.Value = running;
            run_button.label = running ? "Stop" : "Start";
            flights_field.SetEnabled(!running);
        }

        static Label Hint(VisualElement parent)
        {
            var l = new Label();
            l.AddToClassList("k2-slider-value-label");
            l.style.whiteSpace = WhiteSpace.Normal;
            parent.Add(l);
            return l;
        }

        // what the lift system has learned about this design (rebuilt at most once a second)
        float last_vehicle_check = -10;

        void UpdateVehicle()
        {
            if (UnityEngine.Time.unscaledTime - last_vehicle_check < 1f) return;
            last_vehicle_check = UnityEngine.Time.unscaledTime;
            try
            {
                var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
                if (vessel == null) { vehicle_label.text = "No vessel"; return; }
                if (!vessel.Landed && !pilot.isRunning) { vehicle_label.text = "Put the vessel on the launch pad"; return; }
                if (!vessel.Landed) return;  // keep the pad reading during a flight
                var model = Stage1Model.Build(vessel, allowNoUpper: true);
                model?.SetSurfaces(StabilityCheck.ForVessel(vessel));
                var r = model == null ? null : CraftMemory.Get(model);
                vehicle_label.text = r == null
                    ? "This vehicle: not flown yet"
                    : $"{CraftMemory.DisplayName(r)}: {r.flights} flight(s) learned, model calibrated on {r.calFlights}";
                vehicle_label.text += "\nResults: " + CraftMemory.DescribeTraining(model);
            }
            catch (Exception e)
            {
                vehicle_label.text = "This vehicle: " + e.Message;
            }
        }

        public override bool onUpdateUI()
        {
            if (!base.onUpdateUI())
                return false;

            UpdateVehicle();
            status_bar.Reset();

            if (pilot.isRunning)
                status_bar.Progress((double)pilot.done / Math.Max(1, pilot.target), $"{pilot.done} / {pilot.target}");

            if (!string.IsNullOrEmpty(pilot.status_line))
                status_bar.Status(pilot.status_line, pilot.status_ok ? StatusLine.Level.Normal : StatusLine.Level.Warning);
            else
                status_bar.Status("Ready - set the number of flights and press Start");

            // newest first, last 8
            var lines = pilot.results;
            var text = new System.Text.StringBuilder();
            for (int i = lines.Count - 1; i >= 0 && i >= lines.Count - 8; i--)
                text.AppendLine(lines[i]);
            results_label.text = text.ToString();
            return true;
        }
    }
}
