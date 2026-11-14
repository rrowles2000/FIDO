using System;
using System.Collections.Generic;
using KSP.Game;
using KSP.Sim.impl;
using KTools;
using UnityEngine;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Follows the stage-1 debris a New-system flight drops until the game destroys it, so the
    /// debris-periapsis rule can be checked against what really happens (ksplog dataset): how long
    /// it lasted, how many orbits, the lowest altitude seen. Before 2026-11-13 the log only had
    /// the debris orbit at the end of the run.
    ///
    /// Polled from K2D2_Plugin.Update every PollS of real time while in flight. Leaving the flight
    /// scene (revert, KSC) ends the tracking with a "still up" line, since the vessel list can't be
    /// trusted across a scene change.
    /// </summary>
    public static class DebrisTracker
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Debris");
        const float PollS = 2f;

        class Tracked
        {
            public string id, name;
            public double startUT, period, startPeKm, startApKm;
            public double lastUT, lastAltKm, minAltKm = double.MaxValue, lastPeKm;
        }

        static readonly List<Tracked> tracked = new();
        static readonly HashSet<string> known = new();

        /// <summary>Remember every vessel that exists now (at launch), so later debris can be told apart.</summary>
        public static void Snapshot()
        {
            known.Clear();
            try
            {
                foreach (var v in GameManager.Instance.Game.SpaceSimulation.UniverseModel.GetAllVessels())
                    if (v?.SimulationObject != null) known.Add(v.SimulationObject.GlobalId.ToString());
            }
            catch (Exception) { }
        }

        /// <summary>Start tracking all debris that appeared since Snapshot (called right after stage 1 separates).</summary>
        public static void RegisterNew()
        {
            try
            {
                foreach (var v in GameManager.Instance.Game.SpaceSimulation.UniverseModel.GetAllVessels())
                    if (v?.SimulationObject != null && v.IsDebris() && !known.Contains(v.SimulationObject.GlobalId.ToString()))
                        Register(v);
            }
            catch (Exception e) { logger.LogWarning("[Debris] register-new failed: " + e.Message); }
        }
        static float next_poll;

        public static void Register(VesselComponent v)
        {
            try
            {
                var o = v?.Orbit;
                if (o == null || v.SimulationObject == null) return;
                string id = v.SimulationObject.GlobalId.ToString();
                if (tracked.Exists(t => t.id == id)) return;
                double R = o.referenceBody.radius;
                double now = GeneralTools.Current_UT;
                var t = new Tracked
                {
                    id = id, name = v.Name, startUT = now, lastUT = now,
                    period = o.period, startPeKm = (o.Periapsis - R) / 1000, startApKm = (o.Apoapsis - R) / 1000,
                    lastAltKm = v.AltitudeFromSeaLevel / 1000, lastPeKm = (o.Periapsis - R) / 1000,
                };
                t.minAltKm = t.lastAltKm;
                tracked.Add(t);
                logger.LogInfo($"[Debris] tracking '{t.name}': Ap {t.startApKm:n1} Pe {t.startPeKm:n1} km, alt {t.lastAltKm:n1} km");
            }
            catch (Exception e) { logger.LogWarning("[Debris] register failed: " + e.Message); }
        }

        public static void Update()
        {
            if (tracked.Count == 0 || Time.unscaledTime < next_poll) return;
            next_poll = Time.unscaledTime + PollS;
            try
            {
                var state = GameManager.Instance?.Game?.GlobalGameState?.GetGameState().GameState;
                // keep going at the KSC / tracking station (players leave the flight scene within a
                // minute of separation; the debris is still on rails); stop at the menu or an editor
                if (state == GameState.MainMenu || state == GameState.VehicleAssemblyBuilder || state == GameState.Loading ||
                    state == GameState.WarmUpLoading || state == GameState.Invalid)
                {
                    foreach (var t in tracked)
                        logger.LogInfo($"[Debris] '{t.name}' STILL UP when tracking stopped ({state}) after " +
                                       $"{t.lastUT - t.startUT:n0} s{Orbits(t)}: last alt {t.lastAltKm:n1} km, Pe {t.lastPeKm:n1} km, lowest {t.minAltKm:n1} km");
                    tracked.Clear();
                    return;
                }

                var alive = new Dictionary<string, VesselComponent>();
                foreach (var v in GameManager.Instance.Game.SpaceSimulation.UniverseModel.GetAllVessels())
                    if (v?.SimulationObject != null) alive[v.SimulationObject.GlobalId.ToString()] = v;

                double now = GeneralTools.Current_UT;
                for (int i = tracked.Count - 1; i >= 0; i--)
                {
                    var t = tracked[i];
                    if (!alive.TryGetValue(t.id, out var v))
                    {
                        logger.LogInfo($"[Debris] '{t.name}' DESTROYED after {now - t.startUT:n0} s{Orbits(t, now)}: last seen alt {t.lastAltKm:n1} km, " +
                                       $"Pe {t.lastPeKm:n1} km, lowest {t.minAltKm:n1} km (dropped with Pe {t.startPeKm:n1} km)");
                        tracked.RemoveAt(i);
                        continue;
                    }
                    var o = v.Orbit;
                    t.lastUT = now;
                    t.lastAltKm = v.AltitudeFromSeaLevel / 1000;
                    t.minAltKm = Math.Min(t.minAltKm, t.lastAltKm);
                    if (o != null) t.lastPeKm = (o.Periapsis - o.referenceBody.radius) / 1000;
                    if (v.Landed || v.Splashed)
                    {
                        logger.LogInfo($"[Debris] '{t.name}' DOWN (landed/splashed) after {now - t.startUT:n0} s{Orbits(t, now)} (dropped with Pe {t.startPeKm:n1} km)");
                        tracked.RemoveAt(i);
                    }
                }
            }
            catch (Exception e)
            {
                logger.LogWarning("[Debris] update failed: " + e.Message);
                tracked.Clear();
            }
        }

        static string Orbits(Tracked t, double now = double.NaN)
        {
            double end = double.IsNaN(now) ? t.lastUT : now;
            return t.period > 0 && !double.IsInfinity(t.period) && t.startPeKm > 0
                ? $" (~{(end - t.startUT) / t.period:n1} orbits)" : "";
        }
    }
}
