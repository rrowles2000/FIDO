using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using KSP.Modules;
using KSP.Sim.DeltaV;
using KSP.Sim.impl;
using UnityEngine;

namespace Fido.Lift.Stage1
{
    // Everything the stage-1 ascent simulator needs, read once from the vessel on the pad.
    //
    // Stage 1 is "everything from the separator marked in the VAB (StageGroup's
    // Data_StageOneSeparator.IsStageOneSeparator) downwards": every Δv stage that fires BEFORE that
    // separator burns as part of stage 1. Per-stage masses, thrust and Isp come straight from the
    // game's own VesselDeltaVComponent, so fuel flow, crossfeed and booster drops are whatever the
    // game itself computes for the staging stack.
    public class Stage1Model
    {
        public class BurnPhase
        {
            public int stage;
            public double startMass, endMass; // t
            public double thrustVac;          // kN at full throttle
            public double ispVac, ispASL;     // s
            public double deltaVVac;          // m/s
        }

        public bool valid;
        public string problem = "";
        public string separatorName = "";
        public bool separatorMarked;
        public PartComponent separatorPart;
        public int separatorStage = -1;
        // lifting surfaces ahead of / behind the CoM (StabilityCheck; -1 = not looked at). Part of a
        // design's identity in CraftMemory: removing the 83 t vessel's control surfaces changed its
        // mass by 0.17 t, so mass and thrust alone called it the same (fragile) design.
        public int finsAhead = -1, finsBehind = -1;

        public void SetSurfaces(Fido.Lift.Next.StabilityCheck.Result s)
        {
            if (s == null || s.verdict == Fido.Lift.Next.StabilityCheck.Verdict.Unknown) return;
            finsAhead = s.finsAhead;
            finsBehind = s.finsBehind;
        }
        public List<BurnPhase> phases = new();
        public List<BurnPhase> upperPhases = new(); // stages from the separator onwards, firing order

        // drag: summed cube AreaDrag at each Mach sample, nose-first and side-on
        public float[] machSamples;
        public double[] areaDrag;
        public double[] areaDragSide;
        public bool dragValid;

        public CelestialBodyComponent body;
        public double latitudeDeg;
        public double launchAltitude; // m above sea level

        public double TotalDeltaVVac => phases.Sum(p => p.deltaVVac);
        public double UpperDeltaVVac => upperPhases.Sum(p => p.deltaVVac);

        const string DataTypeName = "Data_StageOneSeparator";

        // ---------------------------------------------------------------- separator mark

        static bool IsMarkedSeparator(PartComponent part)
        {
            if (part?.Modules == null) return false;
            foreach (var module in part.Modules.Values)
            {
                if (module?.DataModules == null) continue;
                foreach (var data in module.DataModules.Values)
                {
                    if (data == null || data.GetType().Name != DataTypeName) continue;
                    var prop = data.GetType().GetField("IsStageOneSeparator")?.GetValue(data);
                    var value = prop?.GetType().GetMethod("GetValue", Type.EmptyTypes)?.Invoke(prop, null);
                    return value is bool b && b;
                }
            }
            return false;
        }

        public static PartComponent FindMarkedSeparator(VesselComponent vessel)
        {
            var parts = vessel?.SimulationObject?.PartOwner?.Parts;
            if (parts == null) return null;
            foreach (var p in parts)
                if (IsMarkedSeparator(p)) return p;
            return null;
        }

        // ---------------------------------------------------------------- build

        // allowNoUpper: a craft whose stage 1 is all there is (the New lift system flies stage one
        // to orbit) - the Optimiser needs an upper stage, so it keeps the default.
        public static Stage1Model Build(VesselComponent vessel, bool allowNoUpper = false)
        {
            var model = new Stage1Model();
            try { model.Fill(vessel, allowNoUpper); }
            catch (Exception e)
            {
                model.valid = false;
                model.problem = "error reading vessel - see log";
                L.Log("[Stage1] Build threw: " + e);
            }
            return model;
        }

        /// <summary>
        /// The VAB version (no flying vessel yet): stages from the assembly's Δv data, the marked
        /// separator passed in as the VAB part (StageGroup knows which one it is). No drag table -
        /// the VAB has no drag cubes to read - so a simulation of this model leaves drag out.
        /// </summary>
        public static Stage1Model FromDeltaV(VesselDeltaVComponent dv, CelestialBodyComponent body, double latitudeDeg,
                                             double launchAltitude, IDeltaVPart separator, bool allowNoUpper)
        {
            var model = new Stage1Model();
            try
            {
                model.body = body;
                model.latitudeDeg = latitudeDeg;
                model.launchAltitude = launchAltitude;
                if (body == null) model.problem = "no planet";
                else if (model.FillStages(dv, separator, separator?.PartName ?? "", allowNoUpper))
                    model.valid = true;
            }
            catch (Exception e)
            {
                model.valid = false;
                model.problem = "error reading Δv data - see log";
                L.Log("[Stage1] FromDeltaV threw: " + e);
            }
            return model;
        }

        void Fill(VesselComponent vessel, bool allowNoUpper)
        {
            if (vessel == null) { problem = "no vessel"; return; }
            body = vessel.mainBody;
            latitudeDeg = vessel.Latitude;
            launchAltitude = vessel.AltitudeFromSeaLevel;

            separatorPart = FindMarkedSeparator(vessel);
            if (!FillStages(vessel.VesselDeltaV, separatorPart,
                    separatorPart != null ? separatorPart.PartData?.partName ?? separatorPart.Name : "", allowNoUpper))
                return;

            BuildDragTable(vessel);
            valid = true;
        }

        // Stage-1 and upper burn phases from the game's Δv data; false (with problem set) if unusable.
        bool FillStages(VesselDeltaVComponent dv, IDeltaVPart separator, string separator_name, bool allowNoUpper)
        {
            if (dv?.StageInfo == null || dv.StageInfo.Count == 0) { problem = "no Δv data yet"; return false; }

            separatorMarked = separator != null;
            if (separatorMarked)
            {
                separatorName = separator_name;
                foreach (var stage in dv.StageInfo)
                foreach (var info in stage.Parts)
                    if (ReferenceEquals(info?.PartRef, separator))
                        separatorStage = info.ActivationStage;
                if (separatorStage < 0) separatorStage = separator.ActivationStage;
            }

            var burning = dv.StageInfo
                .Where(s => s.ThrustVac > 0 && s.StartMass > s.EndMass && s.IspVac > 0)
                .OrderByDescending(s => s.Stage)
                .ToList();
            if (burning.Count == 0) { problem = "no engine stages"; return false; }

            List<DeltaVStageInfo> upper;
            if (separatorMarked)
            {
                upper = burning.Where(s => s.Stage <= separatorStage).ToList();
                burning = burning.Where(s => s.Stage > separatorStage).ToList();
            }
            else
            {
                // No mark: assume stage 1 is just the first stage that burns.
                upper = burning.Skip(1).ToList();
                burning = burning.Take(1).ToList();
                separatorName = "(none marked - using first engine stage)";
            }
            if (burning.Count == 0) { problem = $"no engines fire before {separatorName}"; return false; }
            if (upper.Count == 0 && !allowNoUpper) { problem = "no upper-stage engines after the separator"; return false; }

            BurnPhase ToPhase(DeltaVStageInfo s) => new BurnPhase
            {
                stage = s.Stage,
                startMass = s.StartMass,
                endMass = s.EndMass,
                thrustVac = s.ThrustVac,
                ispVac = s.IspVac,
                ispASL = s.IspASL > 0 ? s.IspASL : s.IspVac * 0.85,
                deltaVVac = s.DeltaVinVac,
            };
            phases.AddRange(burning.Select(ToPhase));
            upperPhases.AddRange(upper.Select(ToPhase));

            foreach (var p in phases.Concat(upperPhases))
                L.Log($"[Stage1] {(phases.Contains(p) ? "stage-1" : "upper")} phase stage {p.stage}: {p.startMass:n2}t -> {p.endMass:n2}t, " +
                      $"thrust {p.thrustVac:n0} kN vac, Isp {p.ispASL:n0}/{p.ispVac:n0}, Δv {p.deltaVVac:n0}");
            L.Log($"[Stage1] separator '{separatorName}' activation stage {separatorStage}");
            return true;
        }

        // ---------------------------------------------------------------- drag

        static readonly float[] MachSamples =
            { 0f, 0.3f, 0.5f, 0.7f, 0.85f, 0.95f, 1.0f, 1.05f, 1.1f, 1.2f, 1.4f, 1.7f, 2f, 3f, 5f, 8f };

        // Nose-first AreaDrag per Mach, same per-part GetAeroDataForDirection call the game's
        // Module_Drag uses (see Landing/VesselAeroLookup.cs). The vessel's nose axis isn't known for
        // sure from ControlTransform, so both its up and forward axes are evaluated and the lower-drag
        // one is taken - a rocket's nose-first direction is its minimum-drag axis.
        void BuildDragTable(VesselComponent vessel)
        {
            var vesselBehavior = vessel.SimulationObject?.objVesselBehavior;
            var control = vesselBehavior?.ControlTransform;
            if (vesselBehavior == null || control == null) return;

            var behaviors = new Dictionary<string, PartBehavior>();
            foreach (var b in vesselBehavior.parts)
                if (b != null) behaviors[b.Guid.ToString()] = b;

            double[] SumFor(Vector3 flowWorld)
            {
                var sums = new double[MachSamples.Length];
                foreach (var part in vessel.SimulationObject.PartOwner.Parts)
                {
                    if (part == null || !behaviors.TryGetValue(part.Guid, out var pb) || pb == null) continue;
                    if (!part.TryGetModuleData<PartComponentModule_Drag, Data_Drag>(out var drag) || drag == null) continue;
                    Vector3 local = pb.ModelTransform.InverseTransformDirection(flowWorld);
                    for (int i = 0; i < MachSamples.Length; i++)
                    {
                        var cube = default(Data_Drag.CubeData);
                        drag.GetAeroDataForDirection(local, MachSamples[i], ref cube);
                        sums[i] += cube.AreaDrag;
                    }
                }
                return sums;
            }

            var up = SumFor(control.up);
            var fwd = SumFor(control.forward);
            bool upIsNose = up[0] <= fwd[0];
            areaDrag = upIsNose ? up : fwd;
            areaDragSide = upIsNose ? fwd : up;
            machSamples = MachSamples;
            dragValid = areaDrag.Any(v => v > 0);
            L.Log($"[Stage1] AreaDrag nose-first: M0 {areaDrag[0]:n3}, M1 {areaDrag[6]:n3}, M3 {areaDrag[13]:n3} " +
                  $"(up-axis {up[0]:n3} / fwd-axis {fwd[0]:n3})");
        }

        static double Interp(float[] xs, double[] ys, double x)
        {
            if (x <= xs[0]) return ys[0];
            for (int i = 1; i < xs.Length; i++)
                if (x <= xs[i])
                {
                    double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                    return ys[i - 1] + (ys[i] - ys[i - 1]) * t;
                }
            return ys[ys.Length - 1];
        }

        // AreaDrag with the nose aoaDeg off the airflow: nose-first and side-on areas blended by
        // |cos| and |sin|, scaled to the game's own per-part drag with the FLOWN attitude.
        // 2026-10-20 flight 4 alone fitted 1.05 / 1.2 (5% rms over Mach 0-7, AoA -48..+6°);
        // flight 5 then read ~8% under that, so these average the two.
        // (An earlier 0.4 side share came from commanded, not flown, AoA - 32% rms.)
        public const double NoseFactor = 1.0;
        public const double SideShare = 1.1;

        public double AreaDragAt(double mach, double aoaDeg = 0)
        {
            if (!dragValid) return 0;
            double a = aoaDeg * Math.PI / 180;
            return TransonicFactor(mach) *
                   (NoseFactor * Interp(machSamples, areaDrag, mach) * Math.Abs(Math.Cos(a)) +
                    SideShare * Interp(machSamples, areaDragSide, mach) * Math.Abs(Math.Sin(a)));
        }

        // Extra drag through Mach 1 that the cube tables miss. 2026-11-13 (83 t disposal vessel,
        // |AoA| < 2°): real/model 1.0-1.09 up to Mach 0.9, then 1.24 at Mach 1.0-1.1 and 1.83 at
        // Mach 1.2 (that last one probably already the start of the flip). Fitted on the low side
        // from one flight, so it only peaks at 1.25 and fades out by Mach 1.6; the optimiser's
        // worst-case run covers the rest. The in-flight "[Stage1] flight" log compares against
        // this corrected model, so real/model there should read ~1.0 once it's right.
        public static double TransonicFactor(double mach)
        {
            if (mach <= 0.85) return 1;
            if (mach <= 1.0) return 1 + 0.25 * (mach - 0.85) / 0.15;
            if (mach <= 1.25) return 1.25;
            if (mach <= 1.6) return 1.25 - 0.25 * (mach - 1.25) / 0.35;
            return 1;
        }
    }
}
