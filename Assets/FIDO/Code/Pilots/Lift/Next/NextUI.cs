using K2UI;
using KTools;
using UnityEngine.UIElements;

namespace Fido.Lift.Next
{
    /// <summary>
    /// "LIFT SYSTEM" foldout on the Lift tab: the launch check and the overrides. Built in code (the
    /// bundle's Lift.uxml has no slot for it).
    /// </summary>
    public class NextUI
    {
        readonly LiftSettings settings;
        Foldout section;
        Label result;

        public NextUI(LiftSettings settings) { this.settings = settings; }

        public void Build(VisualElement parent, int index)
        {
            section = new Foldout { text = "LIFT SYSTEM", value = true };
            if (index >= 0 && index <= parent.childCount) parent.Insert(index, section);
            else parent.Add(section);

            // Everything is automatic now; the settings are overrides kept for calibration flights
            // (2026-11-13: aiming at a fully automated system - see `ksplog dataset` for the
            // evidence each one needs before it can go).
            Hint(section, "Automatic: the behaviour model is trained for this vessel in the calibrated simulator on the pad, " +
                          "stage 1's leftover is burned at apoapsis and dropped on a re-entry orbit. Check below to see its plan.");

            var button = new Button(CheckNow) { text = "Check launch" };
            button.style.height = 26;
            section.Add(button);
            result = Hint(section, "");

            var o = new Foldout { text = "OVERRIDES (calibration only)", value = false };
            section.Add(o);
            Hint(o, "Leave these at their defaults unless a test needs them.");

            var max_thr = new FloatField("Max throttle (0.1-1)") { isDelayed = true };
            max_thr.Bind(settings.next_max_throttle);
            o.Add(max_thr);

            var max_q = new FloatField("Max Q (kPa)") { isDelayed = true };
            max_q.Bind(settings.next_max_q_kpa);
            o.Add(max_q);
            Hint(o, "Training and the flight score are penalised above it; the throttle is never cut back for it.");

            var q_alpha = new FloatField("Heading Q·AoA limit (kPa·°)") { isDelayed = true };
            q_alpha.Bind(settings.next_q_alpha);
            o.Add(q_alpha);
            Hint(o, "The ground-track heading correction turns the nose at most this ÷ q degrees sideways off the airflow.");

            var s1_burn = new K2Toggle("Burn stage 1's leftover at apoapsis, then drop it");
            s1_burn.Bind(settings.next_s1_ap_burn);
            o.Add(s1_burn);

            var s1_pe = new FloatField("Stage-1 debris periapsis (km)") { isDelayed = true };
            s1_pe.Bind(settings.next_s1_pe_km);
            o.Add(s1_pe);
            settings.next_s1_ap_burn.listen(v => s1_pe.Show(v));
            Hint(o, "Kept at least 2 km inside the atmosphere. Needs the stage-1 separator marked in the VAB.");

            var refuse = new K2Toggle("Refuse if stage 1 is short");
            refuse.Bind(settings.next_refuse_short);
            o.Add(refuse);
        }

        float last_check_time = -10;

        void CheckNow()
        {
            // one check per second: 2026-11-13 logged ten identical checks in 2 s
            if (UnityEngine.Time.unscaledTime - last_check_time < 1f) return;
            last_check_time = UnityEngine.Time.unscaledTime;

            var vessel = FidoPlugin.Instance.current_vessel?.VesselComponent;
            if (vessel == null) { result.text = "No vessel"; return; }
            var ctx = PolicyContext.From(settings);
            var model = Fido.Lift.Stage1.Stage1Model.Build(vessel, allowNoUpper: true);
            if (vessel.Landed) model?.SetSurfaces(StabilityCheck.ForVessel(vessel));
            CraftMemory.ApplyTo(ref ctx, model);
            var calib = CraftMemory.GetCalibration(model);
            string train = "";
            if (model != null && model.valid)
            {
                var b = vessel.mainBody;
                double cap = settings.Stage1CapKm(b.hasAtmosphere ? b.atmosphereDepth / 1000 : 0);
                var o = BehaviourTrainer.TrainForLaunch(model, ref ctx, settings.heading.V, settings.destination_Ap_km.V, calib, cap, vessel.Name,
                                                        LiftPilot.Instance?.logger);
                if (o != null) train = "\nTraining: " + o.Summary() + "\nWeights: " + o.weights;
            }
            var r = LaunchCheck.Run(model, ctx, settings.heading.V, settings.destination_Ap_km.V, calib);
            result.text = r.Summary() + (vessel.Landed ? "" : " (not on the pad - simulated from here as if launching)") +
                          "\nThis design: " + CraftMemory.Describe(model) + "\nModel: " + calib + train;
            var log = LiftPilot.Instance?.logger;
            log?.LogInfo("[Next] launch check (button): " + r.Summary());
        }

        static Label Hint(VisualElement parent, string text)
        {
            var l = new Label(text);
            l.AddToClassList("k2-slider-value-label");
            l.style.whiteSpace = WhiteSpace.Normal;
            parent.Add(l);
            return l;
        }
    }
}
