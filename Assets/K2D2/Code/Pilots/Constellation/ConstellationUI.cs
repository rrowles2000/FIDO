using System;
using K2UI;
using K2UI.Tabs;
using K2D2.UI;
using KTools;
using UnityEngine.UIElements;

namespace K2D2.Constellation
{
    /// <summary>
    /// The "Sats" tab. Unlike the other tabs it has no UXML in the bundle - the whole page is built
    /// here, and CreateTabPage() is inserted into the TabbedPage by K2D2Window before Init.
    /// </summary>
    class ConstellationUI : K2Page
    {
        public const string Code = "constellation";

        readonly ConstellationPilot pilot;

        public ConstellationUI(ConstellationPilot pilot)
        {
            this.pilot = pilot;
            code = Code;
        }

        public static TabPage CreateTabPage() => new TabPage { name = Code, label = "Sats" };

        FullStatus status_bar;
        VisualElement infos_el, plan_el, order_el;
        FloatField altitude_field, separation_field;
        IntegerField phasing_field;
        Label altitude_hint, separation_hint, phasing_hint, polar_label;
        Button polar_button;
        ToggleButton run_button;
        Button discard_button;

        string RunLabel() => pilot.isRunning ? "Stop" : pilot.IsPaused ? "Resume" : "Start";

        public override bool onInit()
        {
            var page = new VisualElement { name = "page" };
            page.style.flexGrow = 1;
            panel.Add(page);

            var title = new Label("Satellite Deployer");
            title.AddToClassList("unselectedContent");
            title.style.width = Length.Percent(100);
            title.style.unityTextAlign = UnityEngine.TextAnchor.UpperCenter;
            page.Add(title);

            infos_el = InfoTable(page);

            // ---- orbit
            var orbit = new Foldout { text = "ORBIT", value = true };
            page.Add(orbit);

            altitude_field = new FloatField("Altitude (km)") { isDelayed = true };
            altitude_field.RegisterCallback<ChangeEvent<float>>(e => pilot.settings.target_altitude_km.V = Math.Max(0, e.newValue));
            orbit.Add(altitude_field);
            altitude_hint = Hint(orbit);
            orbit.Add(SmallButton("Use suggested altitude", () => pilot.settings.target_altitude_km.V = 0));

            polar_button = SmallButton("Click for polar orbit!", () => pilot.StartPolar());
            polar_button.style.marginTop = 6;
            orbit.Add(polar_button);
            polar_label = Hint(orbit);

            // ---- deployment
            var deploy = new Foldout { text = "DEPLOYMENT", value = true };
            page.Add(deploy);

            separation_field = new FloatField("Separation (°)") { isDelayed = true };
            separation_field.RegisterCallback<ChangeEvent<float>>(e => pilot.settings.separation_deg.V = Math.Max(0, e.newValue));
            deploy.Add(separation_field);
            separation_hint = Hint(deploy);
            deploy.Add(SmallButton("Use suggested separation", () => pilot.settings.separation_deg.V = 0));

            phasing_field = new IntegerField("Phasing orbits (0 = auto)") { isDelayed = true };
            phasing_field.value = pilot.settings.phasing_orbits.V;
            phasing_field.RegisterCallback<ChangeEvent<int>>(e => pilot.settings.phasing_orbits.V = Math.Max(0, e.newValue));
            pilot.settings.phasing_orbits.listeners += v => phasing_field.SetValueWithoutNotify(v);
            deploy.Add(phasing_field);
            phasing_hint = Hint(deploy);

            var reserve = new K2Slider { Label = "Fuel reserve (%)", printValue = true };
            reserve.InitValues(pilot.settings.reserve_pc.V, 0, 50);
            reserve.Bind(pilot.settings.reserve_pc);
            deploy.Add(reserve);

            var accel = new K2Slider { Label = "Max burn acceleration (m/s²)", printValue = true };
            accel.InitValues(pilot.settings.max_accel.V, 1, 30);
            accel.Bind(pilot.settings.max_accel);
            accel.tooltip = "Throttle cap for the carrier's burns - lower stops the lightened carrier wobbling";
            deploy.Add(accel);

            deploy.Add(Toggle("Auto dish switch on", pilot.settings.auto_dish));
            deploy.Add(Toggle("Recover fuel", pilot.settings.move_fuel));

            // ---- release order
            var order = new Foldout { text = "RELEASE ORDER", value = true };
            page.Add(order);
            order_el = new VisualElement();
            order.Add(order_el);
            order.Add(SmallButton("Rescan vessel", () => pilot.Rescan()));

            // ---- plan
            var plan = new Foldout { text = "PLAN", value = true };
            page.Add(plan);
            plan_el = new VisualElement();
            plan_el.AddToClassList("advanced-info-table");
            plan.Add(plan_el);

            run_button = new ToggleButton { label = "Start", name = "run" };
            run_button.AddToClassList("big");
            run_button.style.marginTop = 10;
            run_button.style.marginBottom = 10;
            run_button.listeners += v =>
            {
                if (v && !pilot.isRunning)
                {
                    pilot.StartDeploy();
                    discard_button?.Show(pilot.IsPaused);
                    // refused before it began (nothing to release, no carrier) - pop the button back up
                    if (!pilot.isRunning) run_button.Value = false;
                }
                else if (!v && pilot.isRunning) pilot.isRunning = false;
                run_button.label = RunLabel();
            };
            pilot.is_running_event += running =>
            {
                run_button.Value = running;
                run_button.label = RunLabel();
                discard_button?.Show(!running && pilot.IsPaused);
                status_bar?.avatar?.SetRunning(running);
                RebuildOrder();
            };
            page.Add(run_button);

            discard_button = SmallButton("Discard paused deployment", () =>
            {
                pilot.DiscardPaused();
                run_button.label = RunLabel();
                discard_button.Show(false);
            });
            discard_button.Show(false);
            page.Add(discard_button);

            page.Add(StatusGroup());
            status_bar = new FullStatus(page);

            pilot.Rescan();
            return true;
        }

        // ================================================================ builders

        static VisualElement InfoTable(VisualElement parent)
        {
            var el = new VisualElement();
            el.AddToClassList("advanced-info-table");
            parent.Add(el);
            return el;
        }

        static Label Hint(VisualElement parent)
        {
            var l = new Label();
            l.AddToClassList("k2-slider-value-label");
            l.style.whiteSpace = WhiteSpace.Normal;
            l.style.marginBottom = 4;
            parent.Add(l);
            return l;
        }

        static Button SmallButton(string text, Action click)
        {
            var b = new Button(click) { text = text };
            b.style.height = 26;
            return b;
        }

        static K2Toggle Toggle(string label, Setting<bool> setting)
        {
            var t = new K2Toggle(label);
            t.Bind(setting);
            return t;
        }

        internal static VisualElement StatusGroup()
        {
            var group = new VisualElement { name = "status_progress_group" };
            group.style.marginTop = 10;
            group.style.marginBottom = 12;

            var row = new VisualElement { name = "k2_status_row" };
            row.AddToClassList("k2-status-row");
            row.Add(new K2Avatar { name = "k2_avatar" });

            var speech = new VisualElement { name = "k2_status_speech" };
            speech.AddToClassList("k2-status-speech");
            speech.Add(new StatusLine { name = "status_pilot" });
            speech.Add(new K2UI.Console { name = "pilot_console" });
            row.Add(speech);
            group.Add(row);

            var progress = new K2ProgressBar { name = "progress" };
            progress.style.height = 25;
            group.Add(progress);
            return group;
        }

        static void Row(VisualElement table, string label, string value)
        {
            var row = new VisualElement();
            row.AddToClassList("advanced-info-row");
            var l = new Label(label);
            l.AddToClassList("advanced-info-row-label");
            row.Add(l);
            var v = new Label(value);
            v.AddToClassList("advanced-info-row-value");
            row.Add(v);
            table.Add(row);
        }

        public void RebuildOrder()
        {
            if (order_el == null) return;
            order_el.Clear();

            if (pilot.satellites.Count == 0)
            {
                order_el.Add(new Label("No releasable satellites on this vessel"));
                return;
            }

            int index = 1;
            foreach (var sat in pilot.satellites)
            {
                var s = sat;
                var row = new VisualElement();
                row.style.flexDirection = FlexDirection.Row;
                row.style.alignItems = Align.Center;

                var toggle = new Toggle { value = s.enabled };
                toggle.SetEnabled(!s.released && !pilot.isRunning);
                toggle.RegisterValueChangedCallback(e => s.enabled = e.newValue);
                row.Add(toggle);

                var label = new Label($"{index}. {s.Label}{(s.released ? " (released)" : "")}");
                label.style.flexGrow = 1;
                row.Add(label);

                var up = new Button(() => pilot.Move(s, -1)) { text = "▲" };
                var down = new Button(() => pilot.Move(s, 1)) { text = "▼" };
                foreach (var b in new[] { up, down })
                {
                    b.style.width = 24;
                    b.style.height = 22;
                    b.SetEnabled(!pilot.isRunning);
                    row.Add(b);
                }
                order_el.Add(row);
                index++;
            }
            var carrier = new Label($"{index}. Carrier (keeps the engine) - last");
            carrier.style.marginLeft = 22;
            order_el.Add(carrier);
        }

        // ================================================================ refresh

        static string Km(double m) => $"{m / 1000.0:n0} km";

        public override bool onUpdateUI()
        {
            if (!base.onUpdateUI())
                return false;

            var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
            var body = pilot.Body;
            infos_el.Clear();
            plan_el.Clear();
            status_bar.Reset();

            if (vessel == null || body == null)
            {
                status_bar.Status("No active vessel");
                return true;
            }

            var orbit = vessel.Orbit;
            Row(infos_el, "Body", body.Name);
            Row(infos_el, "Ap / Pe", $"{Km(orbit.ApoapsisArl)} / {Km(orbit.PeriapsisArl)}");
            Row(infos_el, "Inclination", $"{orbit.inclination:n2}°");
            Row(infos_el, "Satellites", $"{pilot.EnabledCount} to release + carrier");

            // ---- orbit values
            double suggested = pilot.SuggestedAltitude(out double min, out double max);
            if (!altitude_field.focusController?.focusedElement?.Equals(altitude_field) ?? true)
                altitude_field.SetValueWithoutNotify((float)Math.Round(pilot.TargetAltitude / 1000.0, 1));
            altitude_hint.text = $"Suggested {Km(suggested)} from {pilot.suggestion_source} (scan band {Km(min)} - {Km(max)})";
            double target = pilot.TargetAltitude;
            if (target < min || target > max)
                altitude_hint.text += " - outside the scan band!";

            if (pilot.PolarEstimate(out double polar_dv, out double inc, out string problem))
            {
                double frac = pilot.FuelFraction(polar_dv);
                string fuel = double.IsNaN(frac) ? "" : $", about {frac * 100:n0}% of the fuel";
                polar_label.text = polar_dv < 0.5
                    ? "Already polar"
                    : $"Needs {polar_dv:n0} m/s{fuel} (have {pilot.AvailableDv:n0} m/s)";
                polar_button.SetEnabled(!pilot.isRunning && polar_dv >= 0.5 && polar_dv < pilot.AvailableDv);
            }
            else
            {
                polar_label.text = problem;
                polar_button.SetEnabled(false);
            }

            // ---- deployment values
            if (!separation_field.focusController?.focusedElement?.Equals(separation_field) ?? true)
                separation_field.SetValueWithoutNotify((float)Math.Round(pilot.Separation, 1));
            separation_hint.text = $"Suggested {pilot.SuggestedSeparation:n1}° (360° / {pilot.EnabledCount + 1} spacecraft)";

            int n = pilot.PhasingOrbits(out bool fits);
            phasing_hint.text = pilot.settings.phasing_orbits.V > 0 ? $"Manual: {n} orbit(s) per satellite" : $"Auto: {n} orbit(s) per satellite";

            // ---- plan
            double alt_dv = pilot.AltitudeChangeDv();
            double total = pilot.TotalMissionDv();
            Row(plan_el, "Altitude change", alt_dv > 0 ? $"{alt_dv:n1} m/s" : "none needed");
            Row(plan_el, "Phasing", $"{pilot.EnabledCount} x {pilot.PhasingDv(n):n1} m/s");
            Row(plan_el, "Total dv", $"{total:n0} of {pilot.AvailableDv:n0} m/s");
            double frac_total = pilot.FuelFraction(total);
            if (!double.IsNaN(frac_total))
                Row(plan_el, "Fuel used", $"about {frac_total * 100:n0}%");
            Row(plan_el, "Deploy time", StrTool.DurationToString(pilot.MissionDuration()));

            // ---- status
            if (pilot.isRunning)
            {
                status_bar.Warning(pilot.step_status);
                string detail = pilot.CurrentStepStatus;
                if (!string.IsNullOrEmpty(detail))
                    status_bar.Console(detail);
            }
            else if (!string.IsNullOrEmpty(pilot.end_status))
                status_bar.Status(pilot.end_status, pilot.result_ok ? StatusLine.Level.Normal : StatusLine.Level.Warning);
            else if (!fits)
                status_bar.Warning("Not enough fuel for this plan - lower the reserve or add phasing orbits");
            else if (pilot.EnabledCount == 0)
                status_bar.Status("No satellites selected");
            else
                status_bar.Status("Ready to deploy");

            return true;
        }
    }
}
