using K2UI.Tabs;
using K2UI;
using UnityEngine.UIElements;
using K2D2.UI;

namespace K2D2.Lift
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

        K2Slider end_rotate_ratio, mid_rotate_ratio;

        Label heading_label;

        VisualElement final_grp;

        // Live LIFT INFO readout, same table pattern as NodeExUI's node_infos_el.
        VisualElement lift_infos_el;

        public override bool onInit()
        {
            LiftSettings settings = pilot.settings;

            status_bar = new FullStatus(panel);
            panel.Q<IntegerField>("start_altitude_km").Bind(settings.start_altitude_km);
            mid_rotate_ratio = panel.Q<K2Slider>("mid_rotate_ratio").Bind(settings.mid_rotate_ratio);
            end_rotate_ratio = panel.Q<K2Slider>("end_rotate_ratio").Bind(settings.end_rotate_ratio);
            panel.Q<IntegerField>("destination_Ap_km").Bind(settings.destination_Ap_km);

            lift_infos_el = panel.Q<VisualElement>("lift_infos");

            final_grp = panel.Q<VisualElement>("final_grp");

            settings.mid_rotate_ratio.listeners += v =>
            {
                if (end_rotate_ratio.value < v)
                    end_rotate_ratio.value = v;
            };

            settings.end_rotate_ratio.listeners += v =>
            {
                if (mid_rotate_ratio.value > v)
                    mid_rotate_ratio.value = v;
            };

            heading_label = panel.Q<Label>("heading_label");
            var heading = panel.Q<K2Compass>("heading").Bind(settings.heading);

            BuildPolarLaunch(panel.Q<Foldout>("profile"), settings);
            stage1_ui = new K2D2.Lift.Stage1.Stage1UI(settings);
            stage1_ui.Build(panel);
            BuildModeSelector(panel, settings);

            var graph = panel.Q<VisualElement>("graph");
            pilot.ascent_path.InitUI(graph);

            var max_throttle = panel.Q<K2Slider>("max_throttle").Bind(settings.max_throttle);

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

        K2D2.Lift.Stage1.Stage1UI stage1_ui;
        K2D2.Lift.Next.NextUI next_ui;

        // LIFT SYSTEM: picks which ascent system flies (LiftSettings.LiftMode). Built in code and
        // placed under the tab title, above the Stage 1 optimiser controls (which only the
        // Optimiser system uses).
        void BuildModeSelector(VisualElement panel, LiftSettings settings)
        {
            var box = new VisualElement();
            box.style.marginBottom = 6;

            var row = new VisualElement();
            row.AddToClassList("k2-slider-label-row");
            row.Add(new Label("Lift System") { style = { fontSize = 13 } });
            box.Add(row);

            var selector = new InlineEnum();
            selector.Bind(settings.mode, "Classic;Optimiser;New");
            box.Add(selector);

            var hint = new Label();
            hint.AddToClassList("k2-slider-value-label");
            hint.style.whiteSpace = WhiteSpace.Normal;
            box.Add(hint);

            var title = panel.Q<Label>("Title");
            var parent = title?.parent;
            if (parent != null) parent.Insert(parent.IndexOf(title) + 1, box);
            else panel.Insert(0, box);

            // the New system's own settings sit right under the selector
            next_ui = new K2D2.Lift.Next.NextUI(settings);
            next_ui.Build(box.parent ?? panel, (box.parent ?? panel).IndexOf(box) + 1);

            settings.mode.listen(m =>
            {
                hint.text = m switch
                {
                    LiftMode.Classic => "Original K2-D2 ascent: profile climb, coast, adjust, circularise.",
                    LiftMode.Optimiser => "Our ascent: adds stage-1 disposal and the optimiser below.",
                    _ => "New lift system (Phase 0.1): 10 Hz layered loop and safety shield, pitch kick + gravity turn, launch check on the pad. No network yet.",
                };
                stage1_ui.SetVisible(m == LiftMode.Optimiser);
                next_ui.SetVisible(m == LiftMode.New);
            });

            // the run keeps the system it started with - lock the selector while flying
            pilot.is_running_event += running => selector.SetEnabled(!running);
        }

        Label polar_hint;

        // POLAR LAUNCH: works out the launch heading for a chosen inclination (90° = polar) from
        // the pad's latitude, the target orbit speed and the planet's rotation, and writes it into
        // the Heading compass above. Built in code since the bundle's Lift.uxml has no slot for it.
        void BuildPolarLaunch(Foldout profile, LiftSettings settings)
        {
            if (profile == null) return;

            var polar = new Foldout { text = "POLAR LAUNCH", value = false };
            var heading_row = profile.Q<VisualElement>("heading_row");
            int index = heading_row != null ? profile.contentContainer.IndexOf(heading_row) : -1;
            if (index >= 0) profile.contentContainer.Insert(index, polar);
            else profile.Add(polar);

            var inc_field = new FloatField("Inclination (°)") { isDelayed = true };
            inc_field.Bind(settings.polar_inclination);
            polar.Add(inc_field);

            var south = new K2Toggle("Launch southward");
            south.Bind(settings.polar_south);
            polar.Add(south);

            var set_button = new Button(() => ApplyPolarHeading(settings)) { text = "Set launch heading" };
            set_button.style.height = 26;
            polar.Add(set_button);

            polar_hint = new Label();
            polar_hint.AddToClassList("k2-slider-value-label");
            polar_hint.style.whiteSpace = WhiteSpace.Normal;
            polar.Add(polar_hint);
        }

        bool PolarHeading(LiftSettings settings, out double heading, out bool reachable)
        {
            heading = 0;
            reachable = false;
            var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
            var body = vessel?.mainBody;
            if (body == null) return false;

            double orbit_speed = K2D2.Constellation.OrbitMath.CircularSpeed(body.gravParameter,
                body.radius + settings.destination_Ap_km.V * 1000.0);
            double surface_speed = K2D2.Constellation.OrbitMath.EquatorialSurfaceSpeed(body);
            reachable = K2D2.Constellation.OrbitMath.LaunchHeading(settings.polar_inclination.V, vessel.Latitude,
                orbit_speed, surface_speed, !settings.polar_south.V, out heading);
            return true;
        }

        void ApplyPolarHeading(LiftSettings settings)
        {
            if (PolarHeading(settings, out double heading, out _))
                settings.heading.V = (float)heading;
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

            pilot.ascent_path.updateProfile(pilot.ascent.current_altitude_km);

            end_rotate_ratio.ValueText = $"{settings.end_rotate_altitude_km:n0} km";
            mid_rotate_ratio.ValueText = $"{settings.mid_rotate_altitude_km:n0} km";
            heading_label.text = $"{settings.heading.V:n1} °";

            if (polar_hint != null)
            {
                if (PolarHeading(settings, out double polar_heading, out bool reachable))
                    polar_hint.text = $"Heading {polar_heading:n1}° for {settings.polar_inclination.V:n1}° at {settings.destination_Ap_km.V} km" +
                        (reachable ? "" : " (closest reachable from this latitude)");
                else
                    polar_hint.text = "";
            }

            final_grp.Show(false);

            UpdateLiftInfos();

            status_bar.Reset();
            if (pilot.isRunning)
            {
                status_bar.Warning($"Status : {pilot.status} ({pilot.mode})");

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