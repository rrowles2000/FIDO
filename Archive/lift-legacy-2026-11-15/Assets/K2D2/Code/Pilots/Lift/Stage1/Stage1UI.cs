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

        VisualElement toggle_el, section_el;
        bool visible = true;

        // Lift tab: only the Optimiser lift system uses stage-1 disposal, so hide it for the others
        public void SetVisible(bool v)
        {
            visible = v;
            toggle_el?.Show(v);
            section_el?.Show(v && settings.disposal_enabled.V);
        }

        public Stage1UI(LiftSettings settings)
        {
            this.settings = settings;
        }

        // with_toggle false: just the optimiser (the Mission tab's Launch step owns the on/off itself)
        public void Build(VisualElement panel, bool with_toggle = true)
        {
            var section = new Foldout { text = "STAGE 1 OPTIMISER", value = true };
            if (!with_toggle)
            {
                panel.Add(section);
                BuildSection(section, false);
                return;
            }
            // On/off sits above the foldout, so a simple flight can switch the whole thing off and
            // get it out of the way.
            var enabled = new K2Toggle("Stage 1 optimiser (disposal ascent)");
            toggle_el = enabled;
            section_el = section;
            enabled.Bind(settings.disposal_enabled);
            var profile = panel.Q<Foldout>("profile");
            var parent = profile?.parent;
            if (parent != null)
            {
                parent.Insert(parent.IndexOf(profile), enabled);
                parent.Insert(parent.IndexOf(profile), section);
            }
            else
            {
                panel.Insert(0, section);
                panel.Insert(0, enabled);
            }
            section.Show(settings.disposal_enabled.V);
            settings.disposal_enabled.listeners += v => section.Show(visible && v);
            BuildSection(section, true);
        }

        void BuildSection(Foldout section, bool with_pe_cap)
        {
            if (with_pe_cap)
            {
                var pe_field = new FloatField("Stage 1 max periapsis (km)") { isDelayed = true };
                pe_field.Bind(settings.disposal_pe_km);
                section.Add(pe_field);
            }

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

            // Live check of what will actually fly: re-simulated whenever the profile or thrust limit
            // changes (2026-11-13: an applied plan was changed with the Lift sliders and launched
            // unchecked, at a thrust the predictor already said would overheat).
            check_label = Hint(section);
            check_label.style.unityFontStyleAndWeight = UnityEngine.FontStyle.Bold;
            void Changed() => ScheduleCheck(section);
            settings.start_altitude_km.listeners += _ => Changed();
            settings.mid_rotate_ratio.listeners += _ => Changed();
            settings.end_rotate_ratio.listeners += _ => Changed();
            settings.max_throttle.listeners += _ => Changed();
            settings.destination_Ap_km.listeners += _ => Changed();
            settings.disposal_pe_km.listeners += _ => Changed();

            result_table = new VisualElement();
            result_table.AddToClassList("advanced-info-table");
            section.Add(result_table);
        }

        // ---------------------------------------------------------------- live check

        Label check_label;
        IVisualElementScheduledItem pending_check;
        SimParams applied;
        bool has_applied;
        static readonly StyleColor BadColor = new(new UnityEngine.Color(1f, 0.35f, 0.3f));
        static readonly StyleColor GoodColor = new(new UnityEngine.Color(0.45f, 0.9f, 0.45f));

        // Slider drags fire on every step - wait until they stop before simulating.
        void ScheduleCheck(VisualElement owner)
        {
            pending_check?.Pause();
            pending_check = owner.schedule.Execute(RunCheck).StartingIn(400);
        }

        void RunCheck()
        {
            pending_check = null;
            if (check_label == null || !settings.disposal_enabled.V) return;
            var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
            if (vessel == null || !vessel.Landed) { check_label.text = ""; return; }
            try
            {
                var model = Stage1Model.Build(vessel);
                if (!model.valid) { check_label.text = ""; return; }
                var p = SimParams.FromSettings(settings);
                var c = Stage1Optimiser.Evaluate(model, p);
                string changed = has_applied && !p.SameProfile(applied) ? "Changed since Apply. " : "";
                if (c.Usable)
                {
                    check_label.text = $"{changed}Current settings OK: {c.nominal.upperMarginDv:n0} m/s spare " +
                                       $"(worst case {c.worst.upperMarginDv:n0}), max Q {c.nominal.maxQkPa:n1} kPa, " +
                                       $"heating {c.nominal.maxHeat / Stage1Optimiser.HeatLimit * 100:n0}%.";
                    check_label.style.color = GoodColor;
                }
                else
                {
                    check_label.text = $"{changed}Current settings NOT SAFE - {c.problem}. Lift will refuse to launch.";
                    check_label.style.color = BadColor;
                }
            }
            catch (Exception e)
            {
                L.Log("[Stage1] check threw: " + e);
                check_label.text = "";
            }
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

        SimParams CurrentParams() => SimParams.FromSettings(settings);

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
                ShowResult(last.best, last.result, last.worst, model, $"{last.runs} simulations" + (last.note != null ? " - " + last.note : ""));
                string problem = has_result ? Stage1Optimiser.Evaluate(model, last.best).problem : "no profile could be simulated";
                bool can_apply = has_result && problem == null;
                apply_button.SetEnabled(can_apply);
                if (can_apply) FlashApply();
                else
                    info_hint.text += $"\nNo safe profile found: {problem}. More thrust, less payload or a lower destination may help" +
                                      (settings.optimise_throttle.V ? "." : "; or let it optimise the thrust limit.");
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
                var c = Stage1Optimiser.Evaluate(model, p);
                L.Log($"[Stage1] predict: start {p.startKm} mid {p.MidKm:n0} end {p.EndKm:n0} km thr {p.throttle:n2} -> " +
                      $"upper {r.UpperDv:n0} (climb {r.upperClimbDv:n0} + circ {r.circDv:n0}) margin {r.upperMarginDv:n0}, " +
                      $"S1 cut-off T+{r.tBurnout:n0} alt {r.altKm:n1} Ap {r.apKm:n1} Pe {r.peKm:n1} cap {r.capHit} " +
                      $"maxQ {r.maxQkPa:n1} heat {r.maxHeat / Stage1Optimiser.HeatLimit * 100:n0}%; " +
                      (c.Usable ? "usable" : "NOT SAFE: " + c.problem));
                ShowResult(p, r, c.worst, model, "current Lift settings");
                RunCheck();
            }
            catch (Exception e)
            {
                L.Log("[Stage1] predict threw: " + e);
                info_hint.text = "Prediction error - see log";
            }
        }

        // Flash Apply after a usable optimiser result, until it's clicked (or ~20 s pass) - an
        // un-applied result is easy to miss, and the steep old profile then flies instead.
        IVisualElementScheduledItem flash;
        static readonly StyleColor FlashColor = new(new UnityEngine.Color(0.95f, 0.65f, 0.1f));

        void FlashApply()
        {
            StopFlash();
            int ticks = 0;
            flash = apply_button.schedule.Execute(() =>
            {
                ticks++;
                if (ticks > 50) { StopFlash(); return; }
                apply_button.style.backgroundColor = ticks % 2 == 1 ? FlashColor : new StyleColor(StyleKeyword.Null);
            }).Every(400);
        }

        void StopFlash()
        {
            flash?.Pause();
            flash = null;
            apply_button.style.backgroundColor = new StyleColor(StyleKeyword.Null);
        }

        void Apply()
        {
            StopFlash();
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
            applied = SimParams.FromSettings(settings);
            has_applied = true;
            info_hint.text = "Applied to the Lift profile; disposal ascent is on. Moving the profile or thrust sliders now " +
                             "changes the plan - the line below re-checks it.";
            RunCheck();
        }

        // ---------------------------------------------------------------- results

        void ShowResult(SimParams p, SimResult r, SimResult worst, Stage1Model model, string source)
        {
            result_table.Clear();
            string problem = Stage1Optimiser.Problem(r);
            string verdict = problem == null
                ? $"Upper stage needs {r.UpperDv:n0} m/s to orbit ({r.upperMarginDv:n0} m/s spare)"
                : (r.upperFeasible ? $"Upper stage needs {r.UpperDv:n0} m/s, but it {problem} - not usable" : problem);

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
            Row("Max Q", $"{r.maxQkPa:n1} kPa (limit {Stage1Optimiser.MaxQLimitKPa:n0})" +
                         (r.maxQkPa > Stage1Optimiser.MaxQLimitKPa ? " - TOO HIGH" : ""));
            Row("Max-Q throttle-down", r.minBucket < 0.999
                ? $"eases to {r.minBucket * p.throttle * 100:n0} % thrust through max Q, then back to {p.throttle * 100:n0} %"
                : "not needed");
            Row("Heating (est.)", $"{r.maxHeat / Stage1Optimiser.HeatLimit * 100:n0} % of limit at {r.maxHeatAltKm:n1} km" +
                                  (r.maxHeat > Stage1Optimiser.HeatLimit ? " - TOO HOT" : ""));
            string worstProblem = Stage1Optimiser.Problem(worst);
            Row($"Worst case (-{(1 - Stage1Optimiser.WorstThrottleFactor) * 100:n0}% thrust, +{(Stage1Optimiser.WorstDragFactor - 1) * 100:n0}% drag)",
                worstProblem == null
                    ? $"OK - {worst.upperMarginDv:n0} m/s spare, max Q {worst.maxQkPa:n1} kPa, heating {worst.maxHeat / Stage1Optimiser.HeatLimit * 100:n0} %"
                    : worstProblem);
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
