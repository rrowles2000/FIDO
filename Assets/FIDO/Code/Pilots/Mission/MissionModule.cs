using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using KSP.Game;
using KSP.OAB;
using KSP.Sim;
using KSP.Sim.Definitions;
using KSP.Sim.impl;
using UnityEngine;

namespace Fido.Mission
{
    // Added to every command part by patches/fido_mission.lua. Only one part per craft carries a
    // plan; the others stay empty. In the VAB the part window has a "Carries the mission" toggle
    // (picks the part the plan is stored on) and an "Edit mission" button; the plan itself is
    // edited in the FIDO Mission window (VAB) or the Mission tab (flight).
    [Serializable]
    public class Data_FidoMission : ModuleData
    {
        public override Type ModuleType => typeof(Module_FidoMission);

        [KSPState] [LocalizedField("PartModules/FidoMission/Holder")] [PAMDisplayControl(SortIndex = 1)]
        public ModuleProperty<bool> IsMissionHolder = new(false);

        [LocalizedField("PartModules/FidoMission/Steps")] [PAMDisplayControl(SortIndex = 2)]
        public ModuleProperty<string> Summary = new("", true);

        [KSPState]
        public ModuleProperty<string> PlanJson = new("");
    }

    [DisallowMultipleComponent]
    public class Module_FidoMission : PartBehaviourModule
    {
        public override Type PartComponentModuleType => typeof(PartComponentModule_FidoMission);

        [SerializeField] protected Data_FidoMission _data;

        public Data_FidoMission Data => _data;

        // live VAB instances, so the editor window can find the craft's command parts
        internal static readonly List<Module_FidoMission> Live = new();

        // Set by the toggle callback; the work runs on the next OAB update, outside the UI click
        // (doing it inside the callback crashed Mono natively in StageGroup).
        int _pendingToggle; // 0 none, 1 turned on, 2 turned off
        bool _suppress;

        protected override void AddDataModules()
        {
            base.AddDataModules();
            _data ??= new Data_FidoMission();
            DataModules.TryAddUnique(_data, out _data);
        }

        protected override void OnInitialize()
        {
            base.OnInitialize();
            _data.SetVisible(_data.PlanJson, false);
            bool oab = PartBackingMode == PartBackingModes.OAB;
            _data.SetVisible(_data.IsMissionHolder, oab);
            _data.SetVisible(_data.Summary, oab);
            if (oab)
            {
                Live.Add(this);
                _data.IsMissionHolder.OnChangedValue += OnToggled;
                _data.AddAction("PartModules/FidoMission/Edit", new ModuleAction((Action)OpenEditor), 3);
                UpdateSummary();
            }
        }

        protected override void OnShutdown()
        {
            _data.IsMissionHolder.OnChangedValue -= OnToggled;
            Live.Remove(this);
            base.OnShutdown();
        }

        void OnToggled(bool on)
        {
            if (!_suppress) _pendingToggle = on ? 1 : 2;
        }

        void OpenEditor() => MissionVabWindow.Instance?.Open();

        /// <summary>Sets the toggle without treating it as a click.</summary>
        internal void SetHolderFlag(bool on)
        {
            if (_data.IsMissionHolder.GetValue() == on) return;
            _suppress = true;
            try { _data.IsMissionHolder.SetValue(on); }
            finally { _suppress = false; }
        }

        internal void UpdateSummary()
        {
            var plan = MissionPlanData.FromJson(_data.PlanJson.GetValue());
            _data.Summary.SetValue(plan == null || plan.IsEmpty ? (_data.IsMissionHolder.GetValue() ? "empty plan" : "-")
                : $"{plan.steps.Count} step(s)");
        }

        protected override void OnModuleOABUpdate(float deltaTime)
        {
            base.OnModuleOABUpdate(deltaTime);
            if (_pendingToggle == 0) return;
            bool on = _pendingToggle == 1;
            _pendingToggle = 0;
            try { MissionStore.SelectHolder(this, on); }
            catch (Exception e) { Debug.LogError("[FIDO Mission] " + e); }
        }
    }

    public class PartComponentModule_FidoMission : PartComponentModule
    {
        public override Type PartBehaviourModuleType => typeof(Module_FidoMission);
    }

    /// <summary>Where a plan lives: one module data block on one part.</summary>
    public class PlanHolder
    {
        public Data_FidoMission data;
        public PartComponent part;      // flight
        public IObjectAssemblyPart oab; // VAB
        public Module_FidoMission module; // VAB

        public MissionPlanData Read() => MissionPlanData.FromJson(data?.PlanJson.GetValue());

        public void Write(MissionPlanData plan)
        {
            data?.PlanJson.SetValue(plan == null || plan.IsEmpty && !plan.Started ? "" : plan.ToJson());
            module?.UpdateSummary();
        }

        public string PartName => part?.PartName ?? oab?.AvailablePart?.PartData?.partName ?? "?";
    }

    public static class MissionStore
    {
        // ---------------------------------------------------------------- flight

        static IEnumerable<PlanHolder> FlightHolders(VesselComponent vessel)
        {
            var parts = vessel?.SimulationObject?.PartOwner?.Parts;
            if (parts == null) yield break;
            foreach (var p in parts)
                if (p.TryGetModuleData<PartComponentModule_FidoMission, Data_FidoMission>(out var data))
                    yield return new PlanHolder { data = data, part = p };
        }

        /// <summary>The part carrying this vessel's plan, or the part a new plan would go on (null if no command part has the module).</summary>
        public static PlanHolder Find(VesselComponent vessel, out bool has_plan)
        {
            var holders = FlightHolders(vessel).ToList();
            var with_plan = holders.FirstOrDefault(h => !string.IsNullOrEmpty(h.data.PlanJson.GetValue()));
            has_plan = with_plan != null;
            if (with_plan != null) return with_plan;
            // a new plan goes on the first command part; Rehome moves it if that part leaves
            return holders.FirstOrDefault();
        }

        /// <summary>
        /// Moves the plan onto <paramref name="vessel"/> if the part carrying it has left (stage 1
        /// dropping a probe core, or a satellite released with the plan on its core).
        /// </summary>
        public static PlanHolder Rehome(PlanHolder holder, MissionPlanData plan, VesselComponent vessel)
        {
            if (vessel == null) return holder;
            if (holder?.part != null && holder.part.PartOwner?.SimulationObject?.Vessel == vessel) return holder;
            var target = Find(vessel, out bool has_plan);
            if (target == null || has_plan && target.data != holder?.data)
            {
                FidoPlugin.logger.LogWarning("[Mission] can't move the plan to " + vessel.Name +
                    (target == null ? ": no command part with the mission module" : ": it already carries a plan"));
                return holder;
            }
            target.Write(plan);
            if (holder != null && holder.data != target.data) holder.data.PlanJson.SetValue("");
            FidoPlugin.logger.LogInfo($"[Mission] plan moved from {holder?.PartName} to {target.PartName} on {vessel.Name}");
            return target;
        }

        // ---------------------------------------------------------------- VAB

        static readonly FieldInfo MainAssemblyField =
            typeof(OABSessionInformation).GetField("mainAssembly", BindingFlags.Instance | BindingFlags.NonPublic);

        public static IObjectAssembly MainAssembly
        {
            get
            {
                try
                {
                    var stats = GameManager.Instance?.Game?.OAB?.Current?.Stats;
                    return stats == null ? null : MainAssemblyField?.GetValue(stats) as IObjectAssembly;
                }
                catch (Exception) { return null; }
            }
        }

        static int Depth(IObjectAssemblyPart p)
        {
            int d = 0;
            for (var q = p?.ParentPart; q != null && d < 1000; q = q.ParentPart) d++;
            return d;
        }

        /// <summary>The VAB part carrying the craft's plan, else the command part nearest the root.</summary>
        public static PlanHolder FindInVab(out bool has_plan)
        {
            var assembly = MainAssembly;
            var modules = Module_FidoMission.Live
                .Where(m => m != null && m.Data != null && m.OABPart != null)
                .Where(m => assembly == null || m.OABPart.Assembly == assembly)
                .ToList();
            var with_plan = modules.FirstOrDefault(m => !string.IsNullOrEmpty(m.Data.PlanJson.GetValue()));
            has_plan = with_plan != null;
            var flagged = modules.FirstOrDefault(m => m.Data.IsMissionHolder.GetValue());
            var pick = with_plan ?? flagged ?? modules.OrderBy(m => Depth(m.OABPart)).FirstOrDefault();
            if (pick == null) return null;
            foreach (var m in modules) m.SetHolderFlag(m == pick);
            return new PlanHolder { data = pick.Data, oab = pick.OABPart, module = pick };
        }

        static List<Module_FidoMission> VabModules(IObjectAssemblyPart part) =>
            Module_FidoMission.Live.Where(m => m != null && m.Data != null && m.OABPart != null && m.OABPart.Assembly == part?.Assembly).ToList();

        /// <summary>Part window toggle: on moves the plan to this part, off moves it to the command part nearest the root.</summary>
        internal static void SelectHolder(Module_FidoMission chosen, bool on)
        {
            var modules = VabModules(chosen.OABPart);
            var current = modules.FirstOrDefault(m => !string.IsNullOrEmpty(m.Data.PlanJson.GetValue()));
            Module_FidoMission target = chosen;
            if (!on)
                target = modules.Where(m => m != chosen).OrderBy(m => Depth(m.OABPart)).FirstOrDefault() ?? chosen;
            if (current != null && current != target)
            {
                target.Data.PlanJson.SetValue(current.Data.PlanJson.GetValue());
                current.Data.PlanJson.SetValue("");
            }
            foreach (var m in modules)
            {
                m.SetHolderFlag(m == target);
                m.UpdateSummary();
            }
            Debug.Log($"[FIDO Mission] plan now on {target.OABPart?.AvailablePart?.PartData?.partName}");
            MissionVabWindow.Instance?.Rescan();
        }
    }
}
