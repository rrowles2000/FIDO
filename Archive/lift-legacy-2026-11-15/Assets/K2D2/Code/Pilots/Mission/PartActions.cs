using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using I2.Loc;
using K2D2.Constellation;
using K2D2.KSPService;
using KSP.Modules;
using KSP.OAB;
using KSP.Sim;
using KSP.Sim.Definitions;
using KSP.Sim.impl;
using KTools;
using UnityEngine;

namespace K2D2.Mission
{
    /// <summary>One thing to do to one tagged part, stored in a "Part actions" step.</summary>
    public class PartActionData
    {
        public string tag = "";     // matches Data_K2ActionTarget.Tag on the part (symmetry copies share it)
        public string part = "";    // display name when it was added, for the editor
        public string kinds = "";   // comma list of PartActionCatalog kinds the part offered
        public string action = "";  // PartActionCatalog action id
    }

    // Added by patches/k2d2_actions.lua to every part with a module a mission can drive. In the VAB
    // its part window gets "Add action to mission"; the tag it hands out survives into flight, so
    // the step finds the same part again.
    [Serializable]
    [DisallowMultipleComponent]
    public class Module_K2ActionTarget : PartBehaviourModule
    {
        public override Type PartComponentModuleType => typeof(PartComponentModule_K2ActionTarget);

        [SerializeField] protected Data_K2ActionTarget _data;

        public Data_K2ActionTarget Data => _data;

        internal static readonly List<Module_K2ActionTarget> Live = new();

        // The click is handled on the next OAB update, outside the UI callback (see Module_K2Mission).
        bool _pendingAdd;

        protected override void AddDataModules()
        {
            base.AddDataModules();
            _data ??= new Data_K2ActionTarget();
            DataModules.TryAddUnique(_data, out _data);
        }

        protected override void OnInitialize()
        {
            base.OnInitialize();
            _data.SetVisible(_data.Tag, false);
            bool oab = PartBackingMode == PartBackingModes.OAB;
            _data.SetVisible(_data.ReleasesSatellite, oab && GetComponent<Module_Decouple>() != null);
            if (!oab) return;
            Live.Add(this);
            _data.AddAction("PartModules/K2D2Actions/Add", new ModuleAction((Action)(() => _pendingAdd = true)), 1);
        }

        protected override void OnShutdown()
        {
            Live.Remove(this);
            base.OnShutdown();
        }

        protected override void OnModuleOABUpdate(float deltaTime)
        {
            base.OnModuleOABUpdate(deltaTime);
            if (!_pendingAdd) return;
            _pendingAdd = false;
            try { AddToMission(); }
            catch (Exception e) { Debug.LogError("[K2D2 Mission] " + e); }
        }

        void AddToMission()
        {
            if (string.IsNullOrEmpty(_data.Tag.GetValue()))
            {
                // next free number on this craft
                int next = 1 + Live.Where(m => m != null && m.OABPart?.Assembly == OABPart?.Assembly)
                    .Select(m => int.TryParse(m.Data.Tag.GetValue(), out int n) ? n : 0)
                    .DefaultIfEmpty(0).Max();
                _data.Tag.SetValue(next.ToString());
            }
            var kinds = PartActionCatalog.KindsOf(gameObject);
            var action = new PartActionData
            {
                tag = _data.Tag.GetValue(),
                part = PartActionCatalog.Title(OABPart?.Name),
                kinds = string.Join(",", kinds),
                action = PartActionCatalog.DefaultAction(kinds),
            };
            Debug.Log($"[K2D2 Mission] part action: {action.part} #{action.tag} ({action.kinds})");
            MissionVabWindow.Instance?.AddPartAction(action);
        }

        /// <summary>Decouplers on the VAB craft ticked "Releases a satellite".</summary>
        internal static int MarkedSatellitesInVab()
        {
            var assembly = MissionStore.MainAssembly;
            return Live.Count(m => m != null && m.Data != null && m.Data.ReleasesSatellite.GetValue() &&
                                   (assembly == null || m.OABPart?.Assembly == assembly));
        }

        public static bool IsMarkedSatelliteRelease(PartComponent p) =>
            p.TryGetModuleData<PartComponentModule_K2ActionTarget, Data_K2ActionTarget>(out var d) && d.ReleasesSatellite.GetValue();

        /// <summary>How many parts on the VAB craft carry this tag (symmetry copies share one).</summary>
        internal static int CountInVab(string tag)
        {
            var assembly = MissionStore.MainAssembly;
            return Live.Count(m => m != null && m.Data != null && m.Data.Tag.GetValue() == tag &&
                                   (assembly == null || m.OABPart?.Assembly == assembly));
        }
    }

    public class Data_K2ActionTarget : ModuleData
    {
        public override Type ModuleType => typeof(Module_K2ActionTarget);

        [KSPState]
        public ModuleProperty<string> Tag = new("");

        // decouplers only: the payload this decoupler frees is a constellation satellite
        [KSPState] [LocalizedField("PartModules/K2D2Actions/Satellite")] [PAMDisplayControl(SortIndex = 2)]
        public ModuleProperty<bool> ReleasesSatellite = new(false);
    }

    public class PartComponentModule_K2ActionTarget : PartComponentModule
    {
        public override Type PartBehaviourModuleType => typeof(Module_K2ActionTarget);
    }

    /// <summary>What each kind of part can do, and doing it in flight.</summary>
    public static class PartActionCatalog
    {
        public const string Fairing = "fairing", Decoupler = "decoupler", Docking = "docking", Light = "light";

        static readonly (string kind, string id, string label)[] Actions =
        {
            (Fairing, "jettison", "Jettison fairing"),
            (Decoupler, "decouple", "Decouple"),
            (Docking, "undock", "Undock / release"),
            (Light, "light_on", "Light on"),
            (Light, "light_off", "Light off"),
            (Light, "light_toggle", "Light toggle"),
        };

        public static List<string> KindsOf(GameObject part)
        {
            var kinds = new List<string>();
            if (part.GetComponent<Module_Fairing>() != null) kinds.Add(Fairing);
            if (part.GetComponent<Module_Decouple>() != null) kinds.Add(Decoupler);
            if (part.GetComponent<Module_DockingNode>() != null) kinds.Add(Docking);
            if (part.GetComponent<Module_Light>() != null) kinds.Add(Light);
            return kinds;
        }

        public static List<(string id, string label)> ActionsFor(string kinds)
        {
            var set = (kinds ?? "").Split(',');
            return Actions.Where(a => set.Contains(a.kind)).Select(a => (a.id, a.label)).ToList();
        }

        public static string DefaultAction(List<string> kinds) =>
            Actions.FirstOrDefault(a => kinds.Contains(a.kind)).id ?? "";

        public static string Label(string id) => Actions.FirstOrDefault(a => a.id == id).label ?? id;

        public static string Title(string part_name)
        {
            if (string.IsNullOrEmpty(part_name)) return "?";
            try
            {
                string title = new LocalizedString("Parts/Title/" + part_name);
                if (!string.IsNullOrEmpty(title) && !title.StartsWith("Parts/")) return title;
            }
            catch (Exception) { }
            return part_name;
        }

        public static string Describe(PartActionData a) => $"{Label(a.action)}: {a.part} #{a.tag}";

        public static List<PartComponent> FindInFlight(VesselComponent vessel, string tag)
        {
            var found = new List<PartComponent>();
            var parts = vessel?.SimulationObject?.PartOwner?.Parts;
            if (parts == null) return found;
            foreach (var p in parts)
                if (p.TryGetModuleData<PartComponentModule_K2ActionTarget, Data_K2ActionTarget>(out var data) &&
                    data.Tag.GetValue() == tag)
                    found.Add(p);
            return found;
        }

        static readonly MethodInfo UndockMethod =
            typeof(Module_DockingNode).GetMethod("Undock", BindingFlags.Instance | BindingFlags.NonPublic);

        static T Behaviour<T>(PartComponent p) where T : PartBehaviourModule
        {
            var behaviour = GeneralTools.Game.SpaceSimulation.ModelViewMap.FromModel(p.SimulationObject)?.Part;
            return behaviour?.Modules.Values.OfType<T>().FirstOrDefault();
        }

        /// <summary>Does the action on one part; returns why it couldn't, or null.</summary>
        public static string Perform(PartComponent p, string action)
        {
            switch (action)
            {
                case "jettison":
                {
                    if (p.TryGetModuleData<PartComponentModule_Fairing, Data_Fairing>(out var d) && d.IsDeployed.GetValue())
                        return "already jettisoned";
                    var m = Behaviour<Module_Fairing>(p);
                    if (m == null) return "no fairing module";
                    m.PerformJettison();
                    return null;
                }
                case "decouple":
                {
                    if (p.TryGetModuleData<PartComponentModule_Decouple, Data_Decouple>(out var d) && d.isDecoupled.GetValue())
                        return "already decoupled";
                    var m = Behaviour<Module_Decouple>(p);
                    if (m == null) return "no decoupler module";
                    m.OnDecouple();
                    return null;
                }
                case "undock":
                {
                    var m = Behaviour<Module_DockingNode>(p);
                    if (m == null || UndockMethod == null) return "no docking port module";
                    UndockMethod.Invoke(m, null);
                    return null;
                }
                case "light_on":
                case "light_off":
                case "light_toggle":
                {
                    if (!p.TryGetModuleData<PartComponentModule_Light, Data_Light>(out var d)) return "no light module";
                    bool on = action == "light_on" || action == "light_toggle" && !d.isLightEnabled.GetValue();
                    d.isLightEnabled.SetValue(on);
                    return null;
                }
            }
            return $"unknown action '{action}'";
        }
    }

    /// <summary>Optionally points radial-in, then does each action in order with a pause between.</summary>
    class PartActionsRunner : StepRunner
    {
        ConstellationPilot.AlignStep align;
        int next;
        float next_at, done_at;
        readonly List<string> problems = new();

        public override string Check()
        {
            if (step.actions.Count == 0) return "No part actions in this step";
            return null;
        }

        public override void Start()
        {
            next = 0;
            done_at = -1;
            problems.Clear();
            align = step.GetBool("radial_in")
                ? new ConstellationPilot.AlignStep(ConstellationPilot.Instance, AutopilotMode.RadialIn, "Pointing radial-in")
                : null;
            align?.Start();
            next_at = Time.time;
        }

        public override StepResult Update()
        {
            if (align != null)
            {
                bool lined_up;
                try { lined_up = align.Update(); }
                catch (Exception e)
                {
                    status = e.Message;
                    return StepResult.Failed;
                }
                status = "Pointing radial-in: " + align.status;
                if (!lined_up) return StepResult.Running;
                align = null;
                next_at = Time.time;
            }

            if (next < step.actions.Count)
            {
                if (Time.time < next_at) return StepResult.Running;
                var a = step.actions[next];
                var parts = PartActionCatalog.FindInFlight(Vessel?.VesselComponent, a.tag);
                if (parts.Count == 0) problems.Add($"{a.part} #{a.tag} not on the vessel");
                foreach (var p in parts)
                {
                    var why = PartActionCatalog.Perform(p, a.action);
                    if (why != null) problems.Add($"{a.part} #{a.tag}: {why}");
                }
                K2D2_Plugin.logger.LogInfo($"[Mission] {PartActionCatalog.Describe(a)} on {parts.Count} part(s)");
                status = $"{next + 1}/{step.actions.Count}: {PartActionCatalog.Describe(a)}";
                next++;
                next_at = Time.time + Mathf.Max(0, step.GetFloat("delay"));
                if (next == step.actions.Count) done_at = Time.time + 2;
                return StepResult.Running;
            }

            if (problems.Count > 0) step.note = string.Join("; ", problems);
            status = problems.Count == 0 ? $"Done {step.actions.Count} action(s)" : "Done with problems: " + step.note;
            return Time.time >= done_at ? StepResult.Done : StepResult.Running;
        }

        public override void Stop()
        {
            if (step.GetBool("radial_in")) SASTool.setAutoPilot(AutopilotMode.StabilityAssist);
        }
    }
}
