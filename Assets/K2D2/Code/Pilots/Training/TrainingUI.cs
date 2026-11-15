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

            flights_field = new IntegerField($"Flights (max {TrainingPilot.MaxFlights})") { isDelayed = true };
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

            BuildProgress(page);

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

        // ---------------------------------------------------------------- training progress

        Foldout progress;
        Label gains_label;
        TrainingChart score_chart, pe_chart, spare_chart, calib_chart;
        string shown_key;
        int shown_count = -1;

        static readonly UnityEngine.Color Real = new UnityEngine.Color(0.35f, 0.75f, 1f);
        static readonly UnityEngine.Color Planned = new UnityEngine.Color(0.85f, 0.85f, 0.85f, 0.75f);
        static readonly UnityEngine.Color After = new UnityEngine.Color(1f, 0.7f, 0.3f);

        void BuildProgress(VisualElement page)
        {
            progress = new Foldout { text = "TRAINING PROGRESS", value = true };
            progress.style.marginTop = 6;
            page.Add(progress);
            gains_label = Hint(progress);
            score_chart = new TrainingChart("Score per flight (km of Pe at separation, less penalties)");
            pe_chart = new TrainingChart("Stage-1 Pe at separation");
            spare_chart = new TrainingChart("Upper-stage Δv to spare after circularising");
            calib_chart = new TrainingChart("Sim vs real trajectory mismatch (calibration, lower is better)", 80);
            progress.Add(score_chart);
            progress.Add(pe_chart);
            progress.Add(spare_chart);
            progress.Add(calib_chart);
        }

        static double[] Pick(List<CraftMemory.FlightRecord> h, Func<CraftMemory.FlightRecord, double> f)
        {
            var a = new double[h.Count];
            for (int i = 0; i < h.Count; i++) a[i] = f(h[i]);
            return a;
        }

        static double FirstGood(double[] a, bool last = false)
        {
            for (int k = 0; k < a.Length; k++)
            {
                double v = a[last ? a.Length - 1 - k : k];
                if (!double.IsNaN(v) && v > TrainingChart.FailBelow) return v;
            }
            return double.NaN;
        }

        static string Change(string what, double[] a, string unit, string fmt = "0.#", bool lowerIsBetter = false)
        {
            double first = FirstGood(a), last = FirstGood(a, true);
            if (double.IsNaN(first)) return null;
            if (a.Length < 2 || double.IsNaN(last)) return $"{what} {first.ToString(fmt)}{unit}";
            double d = last - first;
            string sign = d >= 0 ? "+" : "−";
            bool better = lowerIsBetter ? d < 0 : d > 0;
            return $"{what} {first.ToString(fmt)} → {last.ToString(fmt)}{unit} ({sign}{Math.Abs(d).ToString(fmt)}{(Math.Abs(d) < 1e-9 ? "" : better ? ", better" : ", worse")})";
        }

        void UpdateProgress(CraftMemory.Record r, string key)
        {
            var h = r?.history ?? new List<CraftMemory.FlightRecord>();
            int n = h.Count + (r?.realScores?.Count ?? 0);
            if (key == shown_key && n == shown_count) return;
            shown_key = key;
            shown_count = n;

            // flights before 2026-11-15 only kept their real score: chart them as score-only points
            int old = Math.Max(0, (r?.realScores?.Count ?? 0) - h.Count);
            var real = new List<double>();
            var sim = new List<double>();
            for (int i = 0; i < old; i++) { real.Add(r.realScores[i]); sim.Add(double.NaN); }
            foreach (var f in h) { real.Add(f.realScore); sim.Add(f.simScore); }

            double cap = h.Count > 0 ? h[h.Count - 1].capKm : double.NaN;
            score_chart.SetData(new[]
            {
                new TrainingChart.Series { name = "real flight", color = Real, values = real.ToArray() },
                new TrainingChart.Series { name = "sim prediction", color = Planned, dashed = true, values = sim.ToArray() },
            }, "", cap, double.IsNaN(cap) ? null : "debris cap");

            var pe = Pick(h, f => f.sepPeKm);
            var pe_plan = Pick(h, f => f.plannedPeKm);
            pe_chart.SetData(new[]
            {
                new TrainingChart.Series { name = "actual", color = Real, values = pe },
                new TrainingChart.Series { name = "planned", color = Planned, dashed = true, values = pe_plan },
            }, " km", cap, double.IsNaN(cap) ? null : "cap");

            var spare = Pick(h, f => f.spareDv);
            var spare_plan = Pick(h, f => f.plannedSpareDv);
            spare_chart.SetData(new[]
            {
                new TrainingChart.Series { name = "actual", color = Real, values = spare },
                new TrainingChart.Series { name = "predicted", color = Planned, dashed = true, values = spare_plan },
            }, "");

            var rms_before = Pick(h, f => f.calRmsBefore);
            var rms_after = Pick(h, f => f.calRmsAfter);
            calib_chart.SetData(new[]
            {
                new TrainingChart.Series { name = "before refit", color = Planned, dashed = true, values = rms_before },
                new TrainingChart.Series { name = "after refit", color = After, values = rms_after },
            }, "");

            // gains: first flight against the latest
            var gaps = Pick(h, f => double.IsNaN(f.spareDv) || double.IsNaN(f.plannedSpareDv) ? double.NaN : Math.Abs(f.spareDv - f.plannedSpareDv));
            var lines = new List<string>();
            int orbit = 0;
            foreach (var f in h) if (f.ok) orbit++;
            lines.Add(real.Count == 0 ? "No training flights recorded for this design yet."
                                      : $"{real.Count} flight(s)" + (h.Count > 0 ? $", {orbit} of the last {h.Count} reached orbit" : ""));
            var realArr = real.ToArray();
            foreach (var line in new[]
                     {
                         Change("Real score", realArr, ""),
                         Change("Pe at separation", pe, " km"),
                         Change("Spare Δv", spare, " m/s", "n0"),
                         Change("Prediction error (spare Δv)", gaps, " m/s", "n0", lowerIsBetter: true),
                         Change("Calibration mismatch", rms_after, "", "0.00", lowerIsBetter: true),
                     })
                if (line != null) lines.Add(line);
            double best = double.NegativeInfinity;
            foreach (var v in realArr) if (!double.IsNaN(v)) best = Math.Max(best, v);
            if (!double.IsInfinity(best)) lines.Add($"Best real score {best:0.#}" + (r?.weights != null ? $" · trained sim score {r.simScore:0.#}" : ""));
            gains_label.text = string.Join("\n", lines);
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
                    : $"This vehicle: {r.flights} flight(s) learned, model calibrated on {r.calFlights}";
                vehicle_label.text += "\nResults: " + CraftMemory.DescribeTraining(model);
                UpdateProgress(r, CraftMemory.Key(model));
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
