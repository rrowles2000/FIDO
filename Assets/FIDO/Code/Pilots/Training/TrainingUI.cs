using System;
using System.Collections.Generic;
using Fido.Constellation;
using Fido.Lift.Next;
using Fido.Lift.Stage1;
using Fido.UI;
using K2UI;
using K2UI.Tabs;
using UnityEngine.UIElements;

namespace Fido.Training
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
        Label vehicle_label, trained_label;
        Readout learned_readout, cal_readout, ok_readout;
        Pill state_pill;
        Label head_note;
        EventList results_list;
        IntegerField flights_field;
        ToggleButton run_button;

        public override bool onInit()
        {
            var page = new VisualElement { name = "page" };
            page.style.flexGrow = 1;
            panel.Add(page);

            // ---- header: state pill + progress note
            var head = FidoKit.Row(page, 6);
            head.style.alignItems = Align.Center;
            head.style.flexWrap = Wrap.NoWrap;
            state_pill = new Pill(head);
            head_note = FidoKit.Hint(head);
            head_note.style.flexShrink = 1;

            // ---- this vehicle and what the lift system has learned about it
            var vehicle = FidoKit.Panel(page, "Vehicle");
            vehicle_label = FidoKit.Text(vehicle, "", bold: true);
            var readouts = FidoKit.Row(vehicle, 0);
            readouts.style.marginTop = 4;
            learned_readout = new Readout("Flights learned", readouts);
            cal_readout = new Readout("Calibrated on", readouts);
            trained_label = FidoKit.Hint(vehicle);

            // ---- the run
            var run = FidoKit.Panel(page, "Run");
            FidoKit.Hint(run, "Flies this vehicle from the pad to a circular orbit, reverts to launch and " +
                              "goes again, so the lift system can learn it.");
            flights_field = new IntegerField("Flights") { isDelayed = true };
            flights_field.Bind(pilot.flights);
            flights_field.style.marginTop = 6;
            run.Add(flights_field);

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
            run.Add(run_button);
            ok_readout = new Readout("Reached orbit", FidoKit.Row(run, 0));

            page.Add(ConstellationUI.StatusGroup());

            // ---- results, newest first
            var results = FidoKit.Panel(page, "Recent flights");
            results_list = new EventList(results);

            // ---- VRAM after reverts (notes/20261123-211500_bug_gpu-hang-after-reverts.md)
            var memory = FidoKit.Panel(page, "Memory");
            var unload_before = new K2Toggle("Unload unused assets before each revert");
            unload_before.Bind(pilot.unload_before_revert);
            memory.Add(unload_before);
            var unload = new K2Toggle("Unload unused assets after each revert");
            unload.Bind(pilot.unload_on_revert);
            memory.Add(unload);
            var census = new K2Toggle("Log an asset census (FIDO.Census)");
            census.Bind(pilot.asset_census);
            memory.Add(census);

            // the live view: a page served outside the game by ksp2-redux/tools/liveview/liveview.py
            // from Ksp2.log, for a second screen. The game (under Proton) can open a URL but can't
            // reliably start a Linux program, so the server is started outside it.
            var liveview = FidoKit.Panel(page, "Live view");
            var live = new Button(() => UnityEngine.Application.OpenURL(LiveViewUrl)) { text = "Open live view in browser" };
            live.style.height = 24;
            liveview.Add(live);
            FidoKit.Hint(liveview, $"Live flight and progress page at {LiveViewUrl} (the fido-liveview service; " +
                                   "or python3 tools/liveview/liveview.py in ksp2-redux).");

            status_bar = new FullStatus(page);
            return true;
        }

        void SyncButton(bool running)
        {
            run_button.Value = running;
            run_button.label = running ? "Stop" : "Start";
            flights_field.SetEnabled(!running);
        }

        // what the lift system has learned about this design (rebuilt at most once a second)
        float last_vehicle_check = -10;

        void UpdateVehicle()
        {
            if (UnityEngine.Time.unscaledTime - last_vehicle_check < 1f) return;
            last_vehicle_check = UnityEngine.Time.unscaledTime;
            try
            {
                var vessel = FidoPlugin.Instance.current_vessel?.VesselComponent;
                if (vessel == null) { vehicle_label.text = "No vessel"; return; }
                if (!vessel.Landed && !pilot.isRunning) { vehicle_label.text = "Put the vessel on the launch pad"; return; }
                if (!vessel.Landed) return;  // keep the pad reading during a flight
                var model = Stage1Model.Build(vessel, allowNoUpper: true);
                model?.SetSurfaces(StabilityCheck.ForVessel(vessel));
                var r = model == null ? null : CraftMemory.Get(model);
                vehicle_label.text = r == null ? "This vehicle: not flown yet" : CraftMemory.DisplayName(r);
                learned_readout.Set(r == null ? "0" : r.flights.ToString(), r == null ? Tone.Muted : Tone.Normal);
                cal_readout.Set(r == null ? "-" : $"{r.calFlights} flight(s)", r == null ? Tone.Muted : Tone.Normal);
                trained_label.text = "Results: " + CraftMemory.DescribeTraining(model);
            }
            catch (Exception e)
            {
                vehicle_label.text = "This vehicle: " + e.Message;
            }
        }

        readonly List<(string, Tone)> results = new List<(string, Tone)>();

        public override bool onUpdateUI()
        {
            if (!base.onUpdateUI())
                return false;

            UpdateVehicle();
            status_bar.Reset();

            int ok_count = 0;
            foreach (var line in pilot.results) if (line.Contains(": orbit")) ok_count++;
            int flown = pilot.results.Count;
            ok_readout.Set(flown == 0 ? "-" : $"{ok_count} / {flown}",
                           flown == 0 ? Tone.Muted : ok_count == flown ? Tone.Ok : Tone.Warn);

            if (pilot.isRunning)
            {
                status_bar.Progress((double)pilot.done / Math.Max(1, pilot.target), $"{pilot.done} / {pilot.target}");
                state_pill.Set("Training", Tone.Run);
                head_note.text = $"flight {Math.Min(pilot.done + 1, pilot.target)} of {pilot.target}";
            }
            else
            {
                state_pill.Set(flown == 0 ? "Ready" : "Stopped", flown == 0 ? Tone.Normal : Tone.Muted);
                head_note.text = flown == 0 ? "" : $"{flown} flight(s) this session";
            }

            if (!string.IsNullOrEmpty(pilot.status_line))
                status_bar.Status(pilot.status_line, pilot.status_ok ? StatusLine.Level.Normal : StatusLine.Level.Warning);
            else
                status_bar.Status("Ready - set the number of flights and press Start");

            // newest first, last 8
            var lines = pilot.results;
            results.Clear();
            for (int i = lines.Count - 1; i >= 0 && i >= lines.Count - 8; i--)
                results.Add((lines[i], lines[i].Contains(": orbit") ? Tone.Ok : lines[i].Contains(": failed") ? Tone.Bad : Tone.Normal));
            if (results.Count == 0) results.Add(("No flights yet", Tone.Muted));
            results_list.Set(results);
            return true;
        }
    }
}
