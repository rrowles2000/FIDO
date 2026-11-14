using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using KSP.Game;
using KSP.Modules;
using KSP.Sim.impl;
using KSP.Sim.ResourceSystem;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Constellation
{
    /// <summary>One releasable payload: the decoupler that frees it and the parts that go with it.</summary>
    public class SatelliteSlot
    {
        public PartComponent decoupler;
        public List<PartComponent> parts = new();
        public PartComponent command;
        public int stage;
        public bool enabled = true;
        public bool released = false;

        public string Label
        {
            get
            {
                string dish = HasDish ? ", dish" : "";
                return $"Stage {stage} - {parts.Count} parts{dish}";
            }
        }

        public bool HasDish => parts.Any(ConstellationVessel.IsSurveyPart);
    }

    /// <summary>
    /// Part-level operations the Constellation pilot needs that K2-D2 didn't have yet: working
    /// out what each decoupler releases, moving propellant, switching Orbital Survey dishes on,
    /// firing a specific decoupler and keeping the carrier as the active vessel.
    /// Orbital Survey is accessed by reflection so K2-D2 still loads without it.
    /// </summary>
    public static class ConstellationVessel
    {
        static ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Constellation");

        static GameInstance Game => GameManager.Instance?.Game;

        const string SurveyModuleName = "PartComponentModule_OrbitalSurvey";

        // ---------------------------------------------------------------- part graph

        static Dictionary<PartComponent, List<PartComponent>> Adjacency(VesselComponent vessel)
        {
            var adj = new Dictionary<PartComponent, List<PartComponent>>();
            foreach (var p in vessel.SimulationObject.PartOwner.Parts)
                adj[p] = new List<PartComponent>();
            foreach (var p in vessel.SimulationObject.PartOwner.Parts)
            {
                var parent = p.ParentPart;
                if (parent != null && adj.ContainsKey(parent))
                {
                    adj[p].Add(parent);
                    adj[parent].Add(p);
                }
            }
            return adj;
        }

        static HashSet<PartComponent> Reachable(Dictionary<PartComponent, List<PartComponent>> adj,
            PartComponent start, PartComponent blocked)
        {
            var seen = new HashSet<PartComponent> { start };
            var stack = new Stack<PartComponent>();
            stack.Push(start);
            while (stack.Count > 0)
            {
                var p = stack.Pop();
                foreach (var n in adj[p])
                    if (n != blocked && seen.Add(n))
                        stack.Push(n);
            }
            return seen;
        }

        public static bool IsEngine(PartComponent p) => p.HasModule<PartComponentModule_Engine>();
        public static bool IsCommand(PartComponent p) => p.HasModule<PartComponentModule_Command>();
        public static bool IsDecoupler(PartComponent p) => p.HasModule<PartComponentModule_Decouple>();

        public static bool IsSurveyPart(PartComponent p) => FindSurveyModule(p) != null;

        /// <summary>The command part that stays with the engine (the carrier).</summary>
        public static PartComponent FindCarrierCommand(VesselComponent vessel, out List<PartComponent> carrierParts)
        {
            carrierParts = new List<PartComponent>();
            var engine = vessel.SimulationObject.PartOwner.Parts.FirstOrDefault(IsEngine);
            if (engine == null)
                return null;

            var adj = Adjacency(vessel);
            // carrier = everything still connected to the engine once every decoupler is cut
            var seen = new HashSet<PartComponent> { engine };
            var stack = new Stack<PartComponent>();
            stack.Push(engine);
            while (stack.Count > 0)
            {
                var p = stack.Pop();
                foreach (var n in adj[p])
                    if (!IsDecoupler(n) && seen.Add(n))
                        stack.Push(n);
            }
            carrierParts = seen.ToList();
            return carrierParts.FirstOrDefault(IsCommand);
        }

        /// <summary>
        /// Every decoupler that, when fired, frees a group of parts containing a command module
        /// and no engine. Ordered furthest-from-engine first, which is the safe release order for
        /// a stacked dispenser (release the outermost payload first).
        /// </summary>
        public static List<SatelliteSlot> FindSatellites(VesselComponent vessel)
        {
            var result = new List<SatelliteSlot>();
            if (vessel == null) return result;

            var engine = vessel.SimulationObject.PartOwner.Parts.FirstOrDefault(IsEngine);
            if (engine == null) return result;

            var adj = Adjacency(vessel);
            var depth = Depths(adj, engine);

            // decouplers ticked "Releases a satellite" in the VAB, when there are any; else every decoupler
            var decouplers = vessel.SimulationObject.PartOwner.Parts.Where(IsDecoupler).ToList();
            var marked = decouplers.Where(K2D2.Mission.Module_K2ActionTarget.IsMarkedSatelliteRelease).ToList();
            if (marked.Count > 0) decouplers = marked;
            foreach (var dec in decouplers)
            {
                // each neighbour of the decoupler that is not on the engine side is a payload
                var engineSide = Reachable(adj, engine, dec);
                var free = new HashSet<PartComponent>();
                foreach (var n in adj[dec])
                {
                    if (engineSide.Contains(n)) continue;
                    foreach (var p in Reachable(adj, n, dec))
                        free.Add(p);
                }

                // only the parts this decoupler alone frees: drop anything behind another decoupler
                var own = new HashSet<PartComponent>();
                foreach (var n in adj[dec])
                {
                    if (engineSide.Contains(n)) continue;
                    var s = new Stack<PartComponent>();
                    if (own.Add(n)) s.Push(n);
                    while (s.Count > 0)
                    {
                        var p = s.Pop();
                        if (IsDecoupler(p)) continue;
                        foreach (var q in adj[p])
                            if (q != dec && own.Add(q))
                                s.Push(q);
                    }
                }

                var command = own.FirstOrDefault(IsCommand);
                if (command == null || own.Any(IsEngine))
                    continue;

                result.Add(new SatelliteSlot
                {
                    decoupler = dec,
                    parts = own.ToList(),
                    command = command,
                    stage = dec.ActivationStage,
                });
            }

            return result.OrderByDescending(s => depth.TryGetValue(s.decoupler, out int d) ? d : 0).ToList();
        }

        static Dictionary<PartComponent, int> Depths(Dictionary<PartComponent, List<PartComponent>> adj, PartComponent start)
        {
            var d = new Dictionary<PartComponent, int> { [start] = 0 };
            var q = new Queue<PartComponent>();
            q.Enqueue(start);
            while (q.Count > 0)
            {
                var p = q.Dequeue();
                foreach (var n in adj[p])
                    if (!d.ContainsKey(n))
                    {
                        d[n] = d[p] + 1;
                        q.Enqueue(n);
                    }
            }
            return d;
        }

        // ---------------------------------------------------------------- resources

        static ResourceDefinitionID ElectricCharge =>
            Game.ResourceDefinitionDatabase.GetResourceIDFromName("ElectricCharge");

        /// <summary>
        /// Moves every propellant (anything but ElectricCharge) out of the satellite's parts into
        /// parts that stay on the vessel, carrier first. Returns the units moved and left behind.
        /// </summary>
        public static void MoveFuel(SatelliteSlot sat, List<PartComponent> carrierParts,
            IEnumerable<PartComponent> otherParts, out double moved, out double leftBehind)
        {
            moved = 0;
            leftBehind = 0;
            var ec = ElectricCharge;
            var sinks = carrierParts.Concat(otherParts.Where(p => !carrierParts.Contains(p))).ToList();

            foreach (var src in sat.parts)
            {
                var container = src.PartResourceContainer;
                if (container == null) continue;

                foreach (var data in container.GetAllResourcesContainedData().ToList())
                {
                    if (data.ResourceID == ec || data.StoredUnits <= 0) continue;
                    double remaining = data.StoredUnits;

                    foreach (var sink in sinks)
                    {
                        if (remaining <= 1e-9) break;
                        var c = sink.PartResourceContainer;
                        if (c == null || c.GetResourceCapacityUnits(data.ResourceID) <= 0) continue;
                        double room = c.GetResourceEmptyUnits(data.ResourceID);
                        if (room <= 0) continue;
                        double take = Math.Min(room, remaining);
                        double removed = container.RemoveResourceUnits(data.ResourceID, take);
                        c.AddResourceUnits(data.ResourceID, removed);
                        remaining -= removed;
                        moved += removed;
                    }
                    leftBehind += remaining;
                }
            }
        }

        // ---------------------------------------------------------------- Orbital Survey

        static PartComponentModule FindSurveyModule(PartComponent p)
        {
            if (p?.Modules == null) return null;
            foreach (var m in p.Modules.Values)
                if (m != null && m.GetType().Name == SurveyModuleName)
                    return m;
            return null;
        }

        static object SurveyData(PartComponentModule module)
        {
            var field = module.GetType().GetField("_dataOrbitalSurvey", BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
            return field?.GetValue(module);
        }

        static object PropertyValue(object data, string name)
        {
            var prop = data?.GetType().GetField(name)?.GetValue(data);
            return prop?.GetType().GetMethod("GetValue", Type.EmptyTypes)?.Invoke(prop, null);
        }

        static void SetPropertyValue(object data, string name, object value)
        {
            var prop = data?.GetType().GetField(name)?.GetValue(data);
            prop?.GetType().GetMethod("SetValue", new[] { value.GetType() })?.Invoke(prop, new[] { value });
        }

        /// <summary>Switches every Orbital Survey dish in the parts on (extending it first if it deploys).</summary>
        public static int ActivateDishes(IEnumerable<PartComponent> parts)
        {
            int count = 0;
            foreach (var p in parts)
            {
                var module = FindSurveyModule(p);
                if (module == null) continue;

                if (p.TryGetModuleData<PartComponentModule_Deployable, Data_Deployable>(out var deploy) &&
                    !deploy.toggleExtend.GetValue())
                    deploy.toggleExtend.SetValue(true);

                var data = SurveyData(module);
                if (data == null) continue;
                SetPropertyValue(data, "EnabledToggle", true);
                count++;
            }
            if (count > 0)
                logger.LogInfo($"[Constellation] Enabled {count} Orbital Survey dish(es)");
            return count;
        }

        /// <summary>
        /// Orbital Survey's own min/ideal/max scanning altitudes (m) for the current body, if a dish
        /// aboard has been set up for it. A dish that was never switched on still carries the
        /// editor's preview values (the Small category), so its BodyCategory text - "Large (Kerbin)" -
        /// must name the current body before the numbers are trusted.
        /// </summary>
        public static bool SurveyAltitudes(VesselComponent vessel, out double min, out double ideal, out double max)
        {
            min = ideal = max = 0;
            if (vessel?.mainBody == null) return false;
            string body_tag = "(" + vessel.mainBody.Name + ")";
            foreach (var p in vessel.SimulationObject.PartOwner.Parts)
            {
                var module = FindSurveyModule(p);
                if (module == null) continue;
                var data = SurveyData(module);
                if (data == null) continue;
                if (!(PropertyValue(data, "BodyCategory") is string category) || !category.Contains(body_tag))
                    continue;
                try
                {
                    min = Convert.ToDouble(PropertyValue(data, "MinimumAltitude"));
                    ideal = Convert.ToDouble(PropertyValue(data, "IdealAltitude"));
                    max = Convert.ToDouble(PropertyValue(data, "MaximumAltitude"));
                }
                catch (Exception) { continue; }
                if (ideal > 0) return true;
            }
            return false;
        }

        /// <summary>
        /// Orbital Survey only re-sorts dishes between vessels (after a decouple) while its own window
        /// is open, so until then every released satellite's dish stays filed under the vessel it
        /// left, and the map draws a single scanning beam. Moves each dish to its current vessel's
        /// entry the same way OS's own refresh loop does. Reflection keeps OS optional.
        /// </summary>
        public static int RefreshSurveyRegistration(IEnumerable<PartComponent> parts)
        {
            try
            {
                var dishes = new Dictionary<object, VesselComponent>();
                foreach (var p in parts)
                {
                    var module = FindSurveyModule(p);
                    var data = module == null ? null : SurveyData(module);
                    var vessel = p.PartOwner?.SimulationObject?.Vessel;
                    if (data != null && vessel != null) dishes[data] = vessel;
                }
                if (dishes.Count == 0) return 0;

                var vm_type = AppDomain.CurrentDomain.GetAssemblies()
                    .Select(a => a.GetType("OrbitalSurvey.Managers.VesselManager", false))
                    .FirstOrDefault(t => t != null);
                var manager = vm_type?.GetProperty("Instance", BindingFlags.Public | BindingFlags.Static)?.GetValue(null);
                var vessels = vm_type?.GetField("OrbitalSurveyVessels")?.GetValue(manager) as System.Collections.IList;
                var register = vm_type?.GetMethods().FirstOrDefault(m =>
                    m.Name.Contains("Register") && !m.Name.Contains("Un") &&
                    m.GetParameters().Length == 2 && m.GetParameters()[0].ParameterType == typeof(VesselComponent));
                var unregister_vessel = vm_type?.GetMethod("UnRegisterVessel");
                if (manager == null || vessels == null || register == null)
                {
                    logger.LogWarning("[Constellation] Orbital Survey's VesselManager not found - open its window once to refresh the beams");
                    return 0;
                }

                int moved = 0;
                foreach (var vs in vessels.Cast<object>().ToList())
                {
                    var vessel = vs.GetType().GetField("Vessel")?.GetValue(vs) as VesselComponent;
                    var modules = vs.GetType().GetField("ModuleStats")?.GetValue(vs) as System.Collections.IList;
                    if (modules == null) continue;
                    foreach (var ms in modules.Cast<object>().ToList())
                    {
                        var data = ms.GetType().GetField("DataModule")?.GetValue(ms);
                        if (data == null || !dishes.TryGetValue(data, out var now_on) || now_on == vessel) continue;
                        modules.Remove(ms);
                        register.Invoke(manager, new[] { now_on, data });
                        moved++;
                    }
                    if (modules.Count == 0)
                        unregister_vessel?.Invoke(manager, new[] { vs });
                }

                // dishes OS hasn't filed anywhere yet
                foreach (var kv in dishes)
                {
                    bool known = vessels.Cast<object>().Any(vs =>
                        (vs.GetType().GetField("ModuleStats")?.GetValue(vs) as System.Collections.IList)?.Cast<object>()
                        .Any(ms => ms.GetType().GetField("DataModule")?.GetValue(ms) == kv.Key) == true);
                    if (!known)
                    {
                        register.Invoke(manager, new[] { kv.Value, kv.Key });
                        moved++;
                    }
                }

                if (moved > 0)
                    logger.LogInfo($"[Constellation] re-filed {moved} Orbital Survey dish(es) under their own vessels");
                return moved;
            }
            catch (Exception e)
            {
                logger.LogWarning("[Constellation] couldn't refresh Orbital Survey vessels: " + e.Message);
                return 0;
            }
        }

        /// <summary>Fallback when no dish reports altitudes: Orbital Survey's default visual table by body radius.</summary>
        public static void DefaultAltitudes(CelestialBodyComponent body, out double min, out double ideal, out double max)
        {
            double r = body.radius;
            if (r <= 150000) { min = 60000; ideal = 170000; max = 220000; }
            else if (r <= 350000) { min = 100000; ideal = 300000; max = 500000; }
            else if (r <= 10000000) { min = 500000; ideal = 800000; max = 1100000; }
            else { min = 5000000; ideal = 8000000; max = 11000000; }

            // never suggest anything inside the atmosphere
            if (body.hasAtmosphere)
            {
                double floor = body.atmosphereDepth + 10000;
                min = Math.Max(min, floor);
                ideal = Math.Max(ideal, floor);
                max = Math.Max(max, ideal);
            }
        }

        // ---------------------------------------------------------------- control / decouple

        public static void SetControlFrom(VesselComponent vessel, PartComponent command)
        {
            if (vessel == null || command == null) return;
            vessel.SetControlOwner(command);
        }

        public static bool Decouple(PartComponent decoupler)
        {
            var view = Game.SpaceSimulation.ModelViewMap.FromModel(decoupler.SimulationObject);
            var behaviour = view?.Part;
            if (behaviour == null)
            {
                logger.LogError("[Constellation] Decoupler isn't loaded - can't fire it");
                return false;
            }
            var module = behaviour.Modules.Values.OfType<Module_Decouple>().FirstOrDefault();
            if (module == null)
            {
                logger.LogError("[Constellation] No Module_Decouple on " + decoupler.PartName);
                return false;
            }
            module.OnDecouple();
            return true;
        }

        /// <summary>Makes the vessel holding this part the active one (after a decouple the game may pick the payload).</summary>
        public static bool EnsureActive(PartComponent carrierCommand)
        {
            var vessel = carrierCommand?.PartOwner?.SimulationObject?.Vessel;
            if (vessel == null) return false;
            if (Game.ViewController.TryGetActiveSimVessel(out var active) && active == vessel) return true;
            logger.LogInfo("[Constellation] Switching back to the carrier");
            return Game.ViewController.SetActiveVehicle(vessel, true, true);
        }
    }
}
