using K2UI.Tabs;
using K2UI;
using UnityEngine.UIElements;
using Fido.UI;

namespace Fido.Lift
{
    class LiftUI : K2Page
    {
        LiftPilot pilot;
        public LiftUI(LiftPilot pilot)
        {
            this.pilot = pilot;
            code = "lift";
        }

        public FullStatus status_bar;

        Label heading_label;

        VisualElement final_grp;

        // Live LIFT INFO readout, same table pattern as NodeExUI's node_infos_el.
        VisualElement lift_infos_el;

        public override bool onInit()
        {
            LiftSettings settings = pilot.settings;

            status_bar = new FullStatus(panel);
            panel.Q<IntegerField>("destination_Ap_km").Bind(settings.destination_Ap_km);

            // The bundle's Lift.uxml still has the Classic/Optimiser controls (removed 2026-11-15,
            // Archive/lift-legacy-2026-11-15/): the profile altitudes and its graph, the roll
            // program and the old throttle cap. The bundle can't be rebuilt here, so hide them.
            foreach (var name in new[] { "start_altitude_km", "mid_rotate_ratio", "end_rotate_ratio", "graph",
                                         "roll_program", "roll_program_settings", "max_throttle" })
                panel.Q<VisualElement>(name)?.Show(false);

            lift_infos_el = panel.Q<VisualElement>("lift_infos");

            final_grp = panel.Q<VisualElement>("final_grp");

            heading_label = panel.Q<Label>("heading_label");
            var heading = panel.Q<K2Compass>("heading").Bind(settings.heading);

            BuildPolarLaunch(panel.Q<Foldout>("profile"), settings);
            BuildSystemSection(panel, settings);

            var run_button = panel.Q<ToggleButton>("run");
            pilot.is_running_event += is_running => run_button.Value = is_running;
            // Same "give him a little life" touch as the Node tab: K2's 3 grille lines cascade
            // on/off with the autopilot, via the same is_running_event run_button already
            // listens to above.
            pilot.is_running_event += is_running => status_bar.avatar?.SetRunning(is_running);
            run_button.listeners +=  v =>
            {
                pilot.isRunning = v;
                run_button.label = v ? "Stop" : "Start";
            };

            // The old "settings" page's controls now live inside the ADVANCED foldout on "page"
            // instead of the separate hidden "settings" VisualElement, so setupUI needs to query
            // the whole panel to find them (same move as NodeExUI.cs).
            settings.setupUI(panel);

            // Reset button moved down into the ADVANCED foldout itself, now that the settings
            // page it used to live on is on its way out (replaced by an info button to the About
            // tab once the restyle is done).
            addResetButton(panel.Q<Foldout>("advanced_foldout"), "lift");

            return true;
        }

        Fido.Lift.Next.NextUI next_ui;

        // LIFT SYSTEM foldout (Next/NextUI) under the tab title. Until 2026-11-15 a selector here
        // chose between the Classic, Optimiser and New systems; only the New one is left.
        void BuildSystemSection(VisualElement panel, LiftSettings settings)
        {
            var title = panel.Q<Label>("Title");
            var parent = title?.parent ?? panel;
            int index = title != null ? parent.IndexOf(title) + 1 : 0;
            next_ui = new Fido.Lift.Next.NextUI(settings);
            next_ui.Build(parent, index);
        }

        Label polar_hint;

        // HEADING BUTTONS: the Heading compass is the launch setting and reads like the navball
        // (0 north, 90 east, 180 south, 270 west). East/West/North/South set those headings; Polar
        // works out the heading for a 90° (polar) orbit going north - a little west of north, to
        // cancel the eastward speed the planet's spin gives at launch. Built in code since the
        // bundle's Lift.uxml has no slot for it. (Replaced the POLAR LAUNCH inclination calculator,
        // 2026-11-24: the inclination box didn't steer by itself, which read as "inclination ignored".)
        void BuildPolarLaunch(Foldout profile, LiftSettings settings)
        {
            if (profile == null) return;

            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            row.style.marginTop = 4;
            row.style.marginBottom = 2;
            VisualElement compass = profile.Q<K2Compass>("heading");
            var parent = compass?.parent ?? profile.contentContainer;
            int index = compass != null ? parent.IndexOf(compass) + 1 : -1;
            if (index >= 0) parent.Insert(index, row);
            else profile.Add(row);

            void Add(string text, System.Action click)
            {
                var b = new Button(click) { text = text };
                b.style.height = 26;
                b.style.flexGrow = 1;
                b.style.marginLeft = b.style.marginRight = 1;
                row.Add(b);
            }
            Add("North", () => settings.heading.V = 0);
            Add("East", () => settings.heading.V = 90);
            Add("South", () => settings.heading.V = 180);
            Add("West", () => settings.heading.V = 270);
            Add("Polar", () => { if (PolarHeading(settings, out double h)) settings.heading.V = (float)h; });

            polar_hint = new Label();
            polar_hint.AddToClassList("k2-slider-value-label");
            polar_hint.style.whiteSpace = WhiteSpace.Normal;
            parent.Insert(index >= 0 ? index + 1 : parent.childCount, polar_hint);
        }

        // heading for a 90° orbit, northward, from this pad at the target altitude
        bool PolarHeading(LiftSettings settings, out double heading)
        {
            heading = 0;
            var vessel = FidoPlugin.Instance.current_vessel?.VesselComponent;
            var body = vessel?.mainBody;
            if (body == null) return false;

            double orbit_speed = Fido.Constellation.OrbitMath.CircularSpeed(body.gravParameter,
                body.radius + settings.destination_Ap_km.V * 1000.0);
            double surface_speed = Fido.Constellation.OrbitMath.EquatorialSurfaceSpeed(body);
            Fido.Constellation.OrbitMath.LaunchHeading(90, vessel.Latitude, orbit_speed, surface_speed, true, out heading);
            return true;
        }

        // navball-style name for a heading: North, North-east, ... (within 2° of a cardinal = that name)
        static string HeadingName(float h)
        {
            h = ((h % 360) + 360) % 360;
            string[] names = { "North", "North-east", "East", "South-east", "South", "South-west", "West", "North-west" };
            foreach (var c in new[] { 0, 90, 180, 270, 360 })
                if (System.Math.Abs(h - c) <= 2) return names[(c % 360) / 45];
            return names[(int)System.Math.Round(h / 45) % 8];
        }

        void AddInfoRow(string label, string value)
        {
            var row = new VisualElement();
            row.AddToClassList("advanced-info-row");

            var label_el = new Label(label);
            label_el.AddToClassList("advanced-info-row-label");
            row.Add(label_el);

            var value_el = new Label(value);
            value_el.AddToClassList("advanced-info-row-value");
            row.Add(value_el);

            lift_infos_el.Add(row);
        }

        void UpdateLiftInfos()
        {
            lift_infos_el.Clear();

            if (pilot.current_subpilot == null)
            {
                AddInfoRow("Status", "Not running");
                return;
            }

            pilot.current_subpilot.UpdateInfoRows(AddInfoRow);
        }

        public override bool onUpdateUI()
        {
            if (!base.onUpdateUI())
                return false;

            LiftSettings settings = pilot.settings;

            heading_label.text = $"{settings.heading.V:n1}° {HeadingName(settings.heading.V)}";

            if (polar_hint != null)
            {
                polar_hint.text = PolarHeading(settings, out double polar_heading)
                    ? $"Polar (90° orbit, going north) = heading {polar_heading:n1}° at {settings.destination_Ap_km.V} km"
                    : "";
            }

            final_grp.Show(false);

            UpdateLiftInfos();

            status_bar.Reset();
            if (pilot.isRunning)
            {
                status_bar.Warning($"Status : {pilot.status}");

                if (pilot.current_subpilot != null)
                    pilot.current_subpilot.updateUI(panel, status_bar);
            }
            else
            {
                if (!string.IsNullOrEmpty(pilot.end_status))
                    status_bar.Status("Final status : " + pilot.end_status,
                        pilot.result_ok ? StatusLine.Level.Normal : StatusLine.Level.Warning);
                else
                    // Idle placeholder, same idea as Node's "No Node Created"/"Ready to execute
                    // node!" - K2 has something to say here instead of the status readout just
                    // sitting empty before the pilot's ever been run.
                    status_bar.Status("Lift autopilot not enabled");
            }

            return true;
        }



    }
}