using System;
using System.Collections.Generic;
using KSP.Modules;
using KSP.OAB;
using KSP.Sim.impl;
using UnityEngine;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Aerodynamic stability estimate, in the VAB and on the pad (NN ascent report, Section 9).
    ///
    /// A rocket is stable in the air when its centre of lift (where the fins push) is behind its
    /// centre of mass. On 2026-11-13 the 83 t test vessel, with fins at the top and the bottom,
    /// flipped three times just after max Q - the nose drifted off the airflow with zero AoA
    /// commanded, which no limit setting fixes.
    ///
    /// This takes the fins (every lifting surface, weighted by its lift coefficient) and the mass
    /// along the vessel's nose axis, at liftoff and at stage-1 burnout (stage 1's parts at their dry
    /// mass), and gives the margin = (CoM - CoL) / length, positive when the fins' lift is behind
    /// the CoM. Fins only: the body's own lift (nose cone, tanks) sits further forward, so the real
    /// margin is somewhat lower - hence the 10% line for "stable".
    /// </summary>
    public static class StabilityCheck
    {
        public const double StableMargin = 0.10, MarginalMargin = 0.0;

        public enum Verdict { Unknown, NoFins, Unstable, Marginal, Stable }

        public class Result
        {
            public Verdict verdict = Verdict.Unknown;
            public string problem = "";
            public int fins;
            public int finsAhead, finsBehind; // lifting surfaces ahead of / behind the liftoff CoM
            public double lengthM;
            public double colM, comLiftoffM, comBurnoutM; // along the nose axis, from the aft end
            public double marginLiftoff, marginBurnout;   // fraction of the length
            public double Worst => Math.Min(marginLiftoff, marginBurnout);

            public string Short()
            {
                switch (verdict)
                {
                    case Verdict.NoFins: return "no fins (steering only)";
                    case Verdict.Unknown: return "stability ?";
                    default:
                        string v = verdict == Verdict.Stable ? "stable" : verdict == Verdict.Marginal ? "MARGINAL" : "UNSTABLE";
                        return $"{v} {Worst * 100:n0}%" + (finsAhead > 0 ? $" ({finsAhead} surfaces ahead of CoM)" : "");
                }
            }

            public string Summary()
            {
                if (verdict == Verdict.Unknown) return "stability: " + problem;
                if (verdict == Verdict.NoFins) return $"stability: no fins, length {lengthM:n1} m - relies on engine steering";
                return $"stability: {Short()} - {fins} lifting surface(s), {finsAhead} ahead of the CoM / {finsBehind} behind, centre of lift {colM:n1} m, CoM {comLiftoffM:n1} m at liftoff / " +
                       $"{comBurnoutM:n1} m at stage-1 burnout (from the aft end, length {lengthM:n1} m), margin " +
                       $"{marginLiftoff * 100:n0}% / {marginBurnout * 100:n0}% (fins only - the body's lift makes it a little less)";
            }
        }

        struct MassPoint { public Vector3 pos; public double wet, dry; }
        struct FinPoint { public Vector3 pos; public double weight; }

        static Result Compute(List<MassPoint> masses, List<FinPoint> fins, Vector3 nose, Vector3? separator)
        {
            var r = new Result();
            if (masses.Count == 0) { r.problem = "no parts"; return r; }
            nose = nose.normalized;

            double min = double.MaxValue, max = double.MinValue;
            foreach (var m in masses)
            {
                double a = Vector3.Dot(m.pos, nose);
                min = Math.Min(min, a);
                max = Math.Max(max, a);
            }
            r.lengthM = Math.Max(0.1, max - min);
            double sep = separator.HasValue ? Vector3.Dot(separator.Value, nose) : double.NaN;

            double wSum = 0, wPos = 0, dSum = 0, dPos = 0;
            foreach (var m in masses)
            {
                double a = Vector3.Dot(m.pos, nose) - min;
                wSum += m.wet; wPos += m.wet * a;
                // at burnout stage 1 (aft of the separator) is empty; everything above stays full
                bool stage1 = !double.IsNaN(sep) && a + min < sep;
                double mb = stage1 ? m.dry : m.wet;
                dSum += mb; dPos += mb * a;
            }
            r.comLiftoffM = wSum > 0 ? wPos / wSum : 0;
            r.comBurnoutM = dSum > 0 ? dPos / dSum : r.comLiftoffM;

            double fSum = 0, fPos = 0;
            foreach (var fp in fins)
            {
                if (fp.weight <= 0) continue;
                r.fins++;
                fSum += fp.weight;
                double fa = Vector3.Dot(fp.pos, nose) - min;
                fPos += fp.weight * fa;
                if (fa > r.comLiftoffM) r.finsAhead++; else r.finsBehind++;
            }
            if (r.fins == 0 || fSum <= 0) { r.verdict = Verdict.NoFins; return r; }
            r.colM = fPos / fSum;
            r.marginLiftoff = (r.comLiftoffM - r.colM) / r.lengthM;
            r.marginBurnout = (r.comBurnoutM - r.colM) / r.lengthM;
            double worst = r.Worst;
            r.verdict = worst >= StableMargin ? Verdict.Stable : worst >= MarginalMargin ? Verdict.Marginal : Verdict.Unstable;
            return r;
        }

        // a lifting surface with no coefficient still counts (procedural parts can report 0 until built)
        static double FinWeight(Data_LiftingSurface d) =>
            d == null ? 0 : d.deflectionLiftCoeff > 0 ? d.deflectionLiftCoeff : 0.01;

        /// <summary>Diagnostics for the log when fins were expected but none found.</summary>
        public static string lastDiagnostic = "";

        // ---------------------------------------------------------------- VAB

        /// <summary>The VAB craft. Rockets are built standing up, so the nose axis is world up.</summary>
        public static Result ForAssembly(IObjectAssembly assembly, object separatorPart)
        {
            var r = new Result();
            try
            {
                if (assembly?.Parts == null) { r.problem = "no craft"; return r; }
                var masses = new List<MassPoint>();
                foreach (var p in assembly.Parts)
                {
                    if (p?.AvailablePart == null) continue;
                    masses.Add(new MassPoint { pos = p.WorldPosition, wet = p.AvailablePart.TotalMass, dry = p.AvailablePart.Mass });
                }
                // Every behaviour module of every part that is a lifting surface - plain fins and
                // control surfaces (body flaps, elevons: Module_ControlSurface derives from it).
                // Walking the modules rather than asking the assembly for one type: on 2026-11-13
                // the typed lookups missed the 83 t vessel's 10 procedural body flaps.
                var fins = new List<FinPoint>();
                foreach (var p in assembly.Parts)
                {
                    var mods = p?.Modules;
                    if (mods == null) continue;
                    foreach (var m in mods.Values)
                        if (m is Module_LiftingSurface ls)
                            fins.Add(new FinPoint { pos = ((Component)ls).transform.position, weight = FinWeight(ls.DataLiftingSurface) });
                }
                Vector3? sep = separatorPart is IObjectAssemblyPart sp ? sp.WorldPosition : (Vector3?)null;
                return Compute(masses, fins, Vector3.up, sep);
            }
            catch (Exception e)
            {
                r.problem = "failed: " + e.Message;
                return r;
            }
        }

        static bool LooksLikeFin(string name)
        {
            name = (name ?? "").ToLowerInvariant();
            return name.Contains("fin") || name.Contains("flap") || name.Contains("wing") || name.Contains("ctrl") || name.Contains("elevon");
        }

        static IEnumerable<string> ModuleNames(PartComponent p)
        {
            if (p.Modules == null) yield break;
            foreach (var mod in p.Modules.Values)
            {
                if (mod == null) continue;
                var names = new List<string>();
                if (mod.DataModules != null)
                    foreach (var d in mod.DataModules.Values) if (d != null) names.Add(d.GetType().Name);
                yield return $"{mod.GetType().Name}[{string.Join("/", names)}]";
            }
        }

        // ---------------------------------------------------------------- flight (pad)

        /// <summary>
        /// The vessel as it stands. Nose axis = the control transform's up: that is the axis the
        /// autopilot points along the flight path, so it's the one that matters.
        /// </summary>
        public static Result ForVessel(VesselComponent vessel)
        {
            var r = new Result();
            try
            {
                var vb = vessel?.SimulationObject?.objVesselBehavior;
                var control = vb?.ControlTransform;
                var parts = vessel?.SimulationObject?.PartOwner?.Parts;
                if (vb == null || control == null || parts == null) { r.problem = "no vessel view"; return r; }

                var behaviors = new Dictionary<string, PartBehavior>();
                foreach (var b in vb.parts)
                    if (b != null) behaviors[b.Guid.ToString()] = b;

                var separator = K2D2.Lift.Stage1.Stage1Model.FindMarkedSeparator(vessel);
                Vector3? sep = null;
                var masses = new List<MassPoint>();
                var fins = new List<FinPoint>();
                var diag = new List<string>();
                foreach (var p in parts)
                {
                    if (p == null || !behaviors.TryGetValue(p.Guid, out var pb) || pb == null) continue;
                    Vector3 pos = pb.transform.position;
                    masses.Add(new MassPoint { pos = pos, wet = p.DryMass + p.ResourceMass, dry = p.DryMass });
                    // any module carrying lifting-surface data (the typed TryGetModuleData lookups
                    // found none of the 83 t vessel's procedural body flaps, twice)
                    Data_LiftingSurface lift = null;
                    if (p.Modules != null)
                        foreach (var mod in p.Modules.Values)
                        {
                            if (mod?.DataModules == null) continue;
                            foreach (var data in mod.DataModules.Values)
                                if (data is Data_LiftingSurface ls) { lift = ls; break; }
                            if (lift != null) break;
                        }
                    if (lift != null)
                        fins.Add(new FinPoint { pos = pos, weight = FinWeight(lift) });
                    else if (LooksLikeFin(p.PartName))
                        diag.Add($"{p.PartName}: " + string.Join(", ", ModuleNames(p)));
                    if (ReferenceEquals(p, separator)) sep = pos;
                }
                lastDiagnostic = diag.Count == 0 ? "" : "fin-like parts without lifting-surface data: " + string.Join(" | ", diag.GetRange(0, Math.Min(3, diag.Count)));
                return Compute(masses, fins, control.up, sep);
            }
            catch (Exception e)
            {
                r.problem = "failed: " + e.Message;
                return r;
            }
        }
    }
}
