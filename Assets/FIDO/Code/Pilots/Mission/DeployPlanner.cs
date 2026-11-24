using System;
using System.Linq;
using Fido.Constellation;
using KSP.Game;
using KSP.Sim.impl;
using UnityEngine;
using UnityEngine.UIElements;

namespace Fido.Mission
{
    /// <summary>
    /// The Deploy step's live plan: which body, how many satellites (decouplers ticked "Releases a
    /// satellite"), and the altitude, spacing and phasing that follow - with an Auto button that
    /// writes them into the step. Leaving the fields at 0 still works them out in flight instead.
    /// </summary>
    public static class DeployPlanner
    {
        public class Plan
        {
            public CelestialBodyComponent body;
            public int sats;
            public double alt_m, sep_deg, phase_dv_each;
            public int phasing;
            public bool fits;
        }

        static UniverseModel Universe => GameManager.Instance?.Game?.UniverseModel;

        /// <summary>The step's planned body, else the vessel's current body (flight), else the home world.</summary>
        public static CelestialBodyComponent BodyFor(MissionStepData s, CelestialBodyComponent fallback)
        {
            var name = s.Get("body");
            if (!string.IsNullOrEmpty(name))
            {
                var b = Universe?.FindCelestialBodyByName(name);
                if (b != null) return b;
            }
            return fallback ?? Universe?.HomeWorld;
        }

        public static Plan Compute(MissionStepData s, int sats, CelestialBodyComponent body)
        {
            var plan = new Plan { body = body, sats = sats };
            if (body == null) return plan;
            ConstellationVessel.DefaultAltitudes(body, out _, out double ideal, out _);
            plan.alt_m = s.GetFloat("alt_km") > 0 ? s.GetFloat("alt_km") * 1000.0 : ideal;
            // the carrier stays on station as the last spacecraft of the ring
            plan.sep_deg = s.GetFloat("separation") > 0 ? s.GetFloat("separation") : 360.0 / Math.Max(1, sats + 1);

            double r = body.radius + plan.alt_m;
            double min_r = body.radius + (body.hasAtmosphere ? body.atmosphereDepth : 0) + 10000;
            double DvEach(int n) => 2.0 * Math.Abs(OrbitMath.SafePhasingBurn(body.gravParameter, r, plan.sep_deg, n, min_r, out _, out _));

            double budget = s.GetFloat("phase_budget");
            if (s.GetInt("phasing") > 0)
            {
                plan.phasing = s.GetInt("phasing");
                plan.fits = sats * DvEach(plan.phasing) <= budget;
            }
            else
            {
                plan.phasing = 30;
                for (int n = 1; n <= 30; n++)
                    if (sats * DvEach(n) <= budget)
                    {
                        plan.phasing = n;
                        plan.fits = true;
                        break;
                    }
            }
            plan.phase_dv_each = DvEach(plan.phasing);
            return plan;
        }

        /// <param name="sats">satellites on the craft (VAB: ticked decouplers; flight: found on the vessel)</param>
        /// <param name="current_body">the vessel's body in flight, null in the VAB</param>
        public static VisualElement Build(MissionStepData s, int sats, CelestialBodyComponent current_body,
            bool can_edit, Action changed)
        {
            var box = new VisualElement();
            box.style.marginTop = 6;

            var bodies = (Universe?.GetAllCelestialBodies() ?? new())
                .Where(b => b.referenceBody != null) // not the sun
                .Select(b => b.bodyName).OrderBy(n => n).ToList();
            var body = BodyFor(s, current_body);
            if (bodies.Count > 0 && body != null)
            {
                var pick = new DropdownField("Body", bodies, Math.Max(0, bodies.IndexOf(body.bodyName)));
                pick.SetEnabled(can_edit);
                pick.RegisterValueChangedCallback(e =>
                {
                    s.p["body"] = e.newValue;
                    changed();
                });
                box.Add(pick);
            }

            var plan = Compute(s, sats, body);
            string text;
            if (sats == 0)
                text = current_body == null
                    ? "No satellites marked - tick 'Releases a satellite' on the decoupler under each satellite"
                    : "No satellites found on this vessel";
            else
                text = $"Plan: {sats} sat(s) + carrier · {plan.body?.bodyName} · {plan.alt_m / 1000:n0} km · " +
                       $"{plan.sep_deg:0.#}° apart · {plan.phasing} phasing orbit(s) · " +
                       $"phasing Δv ~{plan.sats * plan.phase_dv_each:n0} m/s" +
                       (plan.fits ? "" : " (over budget)");
            var label = new Label(text);
            label.style.whiteSpace = WhiteSpace.Normal;
            label.style.color = sats == 0 || !plan.fits ? new Color(1f, 0.8f, 0.35f) : new Color(0.75f, 0.9f, 1f);
            box.Add(label);

            var auto = new Button(() =>
            {
                s.Set("alt_km", (float)Math.Round(plan.alt_m / 1000.0));
                s.Set("separation", (float)Math.Round(plan.sep_deg, 1));
                s.Set("phasing", plan.phasing);
                if (plan.body != null) s.p["body"] = plan.body.bodyName;
                changed();
            })
            {
                text = "Auto - fill in altitude, separation and phasing",
                tooltip = "Writes the plan above into the fields. Set them back to 0 to work them out in flight instead.",
            };
            auto.style.height = 24;
            auto.SetEnabled(can_edit && sats > 0 && plan.body != null);
            box.Add(auto);
            return box;
        }
    }
}
