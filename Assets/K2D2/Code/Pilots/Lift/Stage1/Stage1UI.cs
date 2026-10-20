using System;
using K2UI;
using KTools;
using UnityEngine.UIElements;

namespace K2D2.Lift.Stage1
{
    // "STAGE 1 OPTIMISER" foldout at the top of the Lift tab. Built in code - the bundle's Lift.uxml
    // has no slot for it (same approach as LiftUI.BuildPolarLaunch).
    public class Stage1UI
    {
        readonly LiftSettings settings;
        Label info_hint;
        VisualElement result_table;
        Button apply_button;
        Stage1Optimiser.Outcome last;
        bool has_result;

        public Stage1UI(LiftSettings settings)
        {
            this.settings = settings;
        }

        public void Build(VisualElement panel)
        {
            var section = new Foldout { text = "STAGE 1 OPTIMISER", value = true };
            var profile = panel.Q<Foldout>("profile");
            var parent = profile?.parent;
            if (parent != null) parent.Insert(parent.IndexOf(profile), section);
            else panel.Insert(0, section);

            var enabled = new K2Toggle("Stage 1 disposal ascent");
            enabled.Bind(settings.disposal_enabled);
            section.Add(enabled);

            var pe_field = new FloatField("Stage 1 max periapsis (km)") { isDelayed = true };
            pe_field.Bind(settings.disposal_pe_km);
            section.Add(pe_field);

            var thr = new K2Toggle("Also optimise thrust limit");
            thr.Bind(settings.optimise_throttle);
            section.Add(thr);

            info_hint = Hint(section);
            info_hint.text = "Finds the Lift profile that leaves the upper stage the least Δv to spend reaching a circular " +
                             "orbit at the Lift 'Destination' altitude. Stage 1 = everything below the separator marked " +
                             "'Stage 1 separator' in the VAB; it cuts off early if its periapsis reaches the max, so it still decays.";

            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            row.Add(SmallButton("Optimise", Optimise));
            row.Add(SmallButton("Predict current", PredictCurrent));
            apply_button = SmallButton("Apply", Apply);
            apply_button.SetEnabled(false);
            row.Add(apply_button);
            section.Add(row);

            result_table = new VisualElement();
            result_table.AddToClassList("advanced-info-table");
            section.Add(result_table);
        }

        // ---------------------------------------------------------------- actions

        Stage1Model ReadVessel()
        {
            var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
            if (vessel == null) { info_hint.text = "No active vessel."; return null; }
            if (!vessel.Landed)
            {
                info_hint.text = "Optimise on the launch pad - the simulation starts from the ground.";
                return null;
            }
            var model = Stage1Model.Build(vessel);
            if (!model.valid)
            {
                info_hint.text = "Can't model stage 1: " + model.problem;
                return null;
            }
            info_hint.text = $"Stage 1: below {model.separatorName}, {model.phases.Count} burn stage(s), " +
                             $"{model.TotalDeltaVVac:n0} m/s vac, {model.phases[0].startMass:n1} t at liftoff. " +
                             $"Upper stage: {model.UpperDeltaVVac:n0} m/s vac" +
                             (model.dragValid ? "" : " (no drag data - vacuum estimate)");
            return model;
        }

        SimParams CurrentParams() => new SimParams
        {
            startKm = settings.start_altitude_km.V,
            midRatio = settings.mid_rotate_ratio.V,
            endRatio = settings.end_rotate_ratio.V,
            throttle = settings.max_throttle.V,
            targetApKm = settings.destination_Ap_km.V,
            disposalPeKm = settings.disposal_pe_km.V,
            endAscentErrorKm = settings.end_ascent_error,
            headingDeg = settings.heading.V,
        };

        void Optimise()
        {
            var model = ReadVessel();
            if (model == null) return;
            if (model.body.hasAtmosphere && settings.disposal_pe_km.V >= model.body.atmosphereDepth / 1000)
                info_hint.text += $"\nNote: the max periapsis is above the atmosphere ({model.body.atmosphereDepth / 1000:n0} km) - stage 1 may not decay.";
            try
            {
                last = Stage1Optimiser.Run(model, CurrentParams(), settings.optimise_throttle.V);
                has_result = last.found;
                ShowResult(last.best, last.result, model, $"{last.runs} simulations" + (last.note != null ? " - " + last.note : ""));
                var res = last.result;
                apply_button.SetEnabled(has_result && res.upperFeasible && res.maxHeat <= Stage1Optimiser.HeatLimit);
                if (!res.upperFeasible)
                    info_hint.text += "\nNo profile gets the upper stage to orbit - it's short of Δv.";
            }
            catch (Exception e)
            {
                L.Log("[Stage1] optimiser threw: " + e);
                info_hint.text = "Optimiser error - see log";
            }
        }

        void PredictCurrent()
        {
            var model = ReadVessel();
            if (model == null) return;
            try
            {
                var p = CurrentParams();
                var r = new Stage1Sim(model) { trace = true }.Run(p);
                L.Log($"[Stage1] predict: start {p.startKm} mid {p.MidKm:n0} end {p.EndKm:n0} km thr {p.throttle:n2} -> " +
                      $"upper {r.UpperDv:n0} (climb {r.upperClimbDv:n0} + circ {r.circDv:n0}) margin {r.upperMarginDv:n0}, " +
                      $"S1 cut-off T+{r.tBurnout:n0} alt {r.altKm:n1} Ap {r.apKm:n1} Pe {r.peKm:n1} cap {r.capHit} " +
                      $"maxQ {r.maxQkPa:n1} heat {r.maxHeat / Stage1Optimiser.HeatLimit * 100:n0}%");
                ShowResult(p, r, model, "current Lift settings");
            }
            catch (Exception e)
            {
                L.Log("[Stage1] predict threw: " + e);
                info_hint.text = "Prediction error - see log";
            }
        }

        void Apply()
        {
            if (!has_result) return;
            var p = last.best;
            settings.start_altitude_km.V = p.startKm;
            // end first so the mid<=end listeners in LiftUI don't clamp the new values
            if (p.endRatio >= settings.mid_rotate_ratio.V)
            {
                settings.end_rotate_ratio.V = p.endRatio;
                settings.mid_rotate_ratio.V = p.midRatio;
            }
            else
            {
                settings.mid_rotate_ratio.V = p.midRatio;
                settings.end_rotate_ratio.V = p.endRatio;
            }
            settings.max_throttle.V = p.throttle;
            settings.disposal_enabled.V = true;
            info_hint.text = "Applied to the Lift profile; disposal ascent is on.";
        }

        // ---------------------------------------------------------------- results

        void ShowResult(SimParams p, SimResult r, Stage1Model model, string source)
        {
            result_table.Clear();
            string verdict;
            if (r.crashed)
                verdict = "Crashes - profile too flat";
            else if (!r.upperFeasible)
                verdict = $"Upper stage short by {-r.upperMarginDv:n0} m/s";
            else if (r.maxHeat > Stage1Optimiser.HeatLimit)
                verdict = $"Upper stage needs {r.UpperDv:n0} m/s, but overheats - not usable";
            else
                verdict = $"Upper stage needs {r.UpperDv:n0} m/s to orbit ({r.upperMarginDv:n0} m/s spare)";

            Row("Result", verdict);
            Row("From", source);
            Row("Profile", $"turn {p.startKm} km, 45° at {p.MidKm:n0} km, 1° at {p.EndKm:n0} km");
            Row("Thrust limit", $"{p.throttle * 100:n0} %");
            string why = r.capHit ? $"Pe reached the {p.disposalPeKm:n0} km cap, {r.leftoverDv:n0} m/s left in stage 1"
                                  : "stage 1 burnt dry";
            Row("Stage 1 cut-off", $"T+{r.tBurnout:n0} s at {r.altKm:n1} km - {why}");
            Row("Orbit at cut-off", $"Ap {r.apKm:n1} km / Pe {r.peKm:n1} km, path {r.fpaDeg:n1}°");
            if (r.upperClimbDv > 0) Row("Upper stage climb", $"{r.upperClimbDv:n0} m/s to finish raising Ap");
            Row("Circularise at Ap", $"{r.circDv:n0} m/s");
            Row("Max Q", $"{r.maxQkPa:n1} kPa");
            Row("Heating (est.)", $"{r.maxHeat / Stage1Optimiser.HeatLimit * 100:n0} % of limit at {r.maxHeatAltKm:n1} km" +
                                  (r.maxHeat > Stage1Optimiser.HeatLimit ? " - TOO HOT" : ""));
        }

        void Row(string label, string value)
        {
            var row = new VisualElement();
            row.AddToClassList("advanced-info-row");
            var l = new Label(label);
            l.AddToClassList("advanced-info-row-label");
            row.Add(l);
            var v = new Label(value);
            v.AddToClassList("advanced-info-row-value");
            v.style.whiteSpace = WhiteSpace.Normal;
            row.Add(v);
            result_table.Add(row);
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
            b.style.flexGrow = 1;
            return b;
        }
    }
}
