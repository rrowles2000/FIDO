using K2UI;
using KTools;
using UnityEngine;
using UnityEngine.UIElements;

namespace Fido.Lift
{
    public class LiftSettings
    {
        public ClampSetting<float> heading = new ("lift.heading", 90,  0 , 360);

        // Polar launch helper (LiftUI's POLAR LAUNCH foldout): target inclination and which way to go
        public ClampSetting<float> polar_inclination = new ("lift.polar_inclination", 90, 0, 180);
        public Setting<bool> polar_south = new ("lift.polar_south", false);
 
        public Setting<int> destination_Ap_km = new("lift.destination_Ap_km", 100);

        public ClampSetting<float> end_ascent_pc = new ("lift.end_ascent_pc", 0.1f, 0, 1);

        public float end_ascent_error
        {
            get => destination_Ap_km.V * end_ascent_pc.V/100;
        }

        // LIFT SYSTEM (Next/, LiftUI's LIFT foldout). The Classic and Optimiser systems and their
        // settings (profile altitudes, max_throttle, disposal_*, roll program, pitch kick, the
        // behaviour-model toggle) were removed 2026-11-15 - see Archive/lift-legacy-2026-11-15/.
        // Max Q: real launchers fly 30-35 kPa. Since the behaviour model it is no longer a
        // throttle-back limit - training and the real-flight score are penalised above it.
        public ClampSetting<float> next_max_q_kpa = new ("lift.next.max_q_kpa", 30, 5, 80);
        // Heading-correction limit, as Q x AoA (kPa x deg): the ground-track correction may turn the
        // nose at most this / q degrees sideways off the airflow (SafetyShield.LimitHeading).
        public ClampSetting<float> next_q_alpha = new ("lift.next.q_alpha", 4, 1, 60);
        // Throttle cap for the whole lift (ascent and the Adjust trim burn).
        public ClampSetting<float> next_max_throttle = new ("lift.next.max_throttle", 1f, 0.1f, 1f);
        // Stage 1's leftover fuel after cut-off: burned at apoapsis (where raising periapsis costs
        // least) until stage 1 is dry or periapsis reaches next_s1_pe_km, then stage 1 is dropped
        // and the upper stage circularises. Before 2026-11-13 the New system cut off and let the
        // circularisation burn use stage 1 until dry, so where the debris ended up was luck.
        public Setting<bool> next_s1_ap_burn = new ("lift.next.s1_ap_burn", true);
        // Highest periapsis stage 1 may be left on. Kept below the atmosphere top (Kerbin 70 km) so
        // the debris comes down within an orbit or two - see Stage1CapKm.
        public ClampSetting<float> next_s1_pe_km = new ("lift.next.s1_pe_km", 55, 0, 1000);
        // Refuse to launch when the launch check says stage 1 can't get there.
        public Setting<bool> next_refuse_short = new ("lift.next.refuse_short", true);

        /// <summary>
        /// Stage 1's periapsis cap (km), kept at least 2 km inside the atmosphere so stage 1 never
        /// stays in orbit.
        /// </summary>
        public double Stage1CapKm(double atmosphereKm)
        {
            double cap = next_s1_pe_km.V;
            if (atmosphereKm > 0) cap = System.Math.Min(cap, atmosphereKm - 2);
            return cap;
        }

        public Setting<bool> coasting_warp = new ("lift.coasting_warp", true);
        public Setting<bool> adjust = new ("lift.adjust", true);
        public ClampSetting<float> end_adjust_pc = new ("lift.end_adjust_pc", 0.01f, 0.001f, 0.5f);

        public float end_adjust_error
        {
            get => destination_Ap_km.V * end_adjust_pc.V/100;
        }

        // Default on: run straight through ascent -> atmosphere clear -> circularize with no
        // pause, since FinalCircularize (Controlers/Final.cs) now creates and flies the
        // circularize burn itself instead of waiting on a manually- or FlightPlan-created node.
        // Turning this off falls back to the old flow - Coasting.cs only actually pauses on
        // pause_on_final below when this is off.
        public Setting<bool> auto_circularize = new ("lift.auto_circularize", true);

        public Setting<bool> pause_on_final = new ("lift.pause_on_final", true);

        public Setting<bool> heading_correction = new ("lift.heading_correction", true);

        Label end_ascent_alt;
        Label end_adjust_alt;

        Group adjust_group;

        void setLabels()
        {
            end_ascent_alt.text = $"End Ascent Alt. : {destination_Ap_km.V - end_ascent_error:n2} km";
            end_adjust_alt.text = $"Final Adjust Alt : {destination_Ap_km.V - end_adjust_error:n2} km";
        }

        public void setupUI(VisualElement root)
        {
            end_ascent_alt =  root.Q<Label>("end_ascent_alt");
            end_adjust_alt = root.Q<Label>("end_adjust_alt");
            adjust_group = root.Q<K2UI.Group>("adjust_group");

            root.Q<K2Toggle>("heading_correction").Bind(heading_correction);  

            // bind to settings and add listener on the same line !
            root.Q<K2Slider>("end_ascent_pc").Bind(end_ascent_pc);
            end_ascent_pc.listen(v => setLabels());

            root.Q<K2Toggle>("coasting_warp").Bind(coasting_warp);

            // adjust is binded to setting and to show adjust group 
            root.Q<K2Toggle>("adjust").Bind(adjust);
            adjust.listen(v => adjust_group.Show(v));

            root.Q<K2Slider>("end_adjust_pc").Bind(end_adjust_pc);
            end_adjust_pc.listen(v => setLabels());

            root.Q<K2Toggle>("auto_circularize").Bind(auto_circularize);

            root.Q<K2Toggle>("pause_on_final").Bind(pause_on_final);

        }
    }
}
