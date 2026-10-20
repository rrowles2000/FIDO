using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using KSP.Modules;
using KSP.Sim.DeltaV;
using KSP.Sim.impl;
using UnityEngine;

namespace K2D2.Lift.Stage1
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

        public static Stage1Model Build(VesselComponent vessel)
        {
            var model = new Stage1Model();
            try { model.Fill(vessel); }
            catch (Exception e)
            {
                model.valid = false;
                model.problem = "error reading vessel - see log";
                L.Log("[Stage1] Build threw: " + e);
            }
            return model;
        }

        void Fill(VesselComponent vessel)
        {
            if (vessel == null) { problem = "no vessel"; return; }
            body = vessel.mainBody;
            latitudeDeg = vessel.Latitude;
            launchAltitude = vessel.AltitudeFromSeaLevel;

            var dv = vessel.VesselDeltaV;
            if (dv?.StageInfo == null || dv.StageInfo.Count == 0) { problem = "no Δv data yet"; return; }

            separatorPart = FindMarkedSeparator(vessel);
            separatorMarked = separatorPart != null;
            if (separatorMarked)
            {
                separatorName = separatorPart.PartData?.partName ?? separatorPart.Name;
                foreach (var stage in dv.StageInfo)
                foreach (var info in stage.Parts)
                    if (ReferenceEquals(info?.PartRef, separatorPart))
                        separatorStage = info.ActivationStage;
                if (separatorStage < 0) separatorStage = separatorPart.ActivationStage;
            }

            var burning = dv.StageInfo
                .Where(s => s.ThrustVac > 0 && s.StartMass > s.EndMass && s.IspVac > 0)
                .OrderByDescending(s => s.Stage)
                .ToList();
            if (burning.Count == 0) { problem = "no engine stages"; return; }

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
            if (burning.Count == 0) { problem = $"no engines fire before {separatorName}"; return; }
            if (upper.Count == 0) { problem = "no upper-stage engines after the separator"; return; }

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

            BuildDragTable(vessel);
            valid = true;
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
            return NoseFactor * Interp(machSamples, areaDrag, mach) * Math.Abs(Math.Cos(a)) +
                   SideShare * Interp(machSamples, areaDragSide, mach) * Math.Abs(Math.Sin(a));
        }
    }
}
