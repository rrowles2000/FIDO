using System;
using Fido.Controller;
using Fido.Lift;
using KSP.Game;
using KSP.Messages;
using KSP.Sim.impl;
using KTools;
using UnityEngine;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido.Training
{
    /// <summary>
    /// Training tab: flies the vessel from the pad to a circular orbit with the New lift system,
    /// reverts to launch and repeats, X times. The New system learns the design after every pad
    /// launch (CraftMemory + Calibrator), so the loop itself is the training.
    /// </summary>
    public class TrainingPilot : Pilot
    {
        public static TrainingPilot Instance { get; private set; }

        public ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.Training");

        public Setting<int> flights = new("training.flights", DefaultFlights);

        // 2026-11-14: the game froze on the 7th flight in a row, then crashed in d3d11 (DXVK)
        // reloading the scene after the 4th revert - sessions are capped at the most that has
        // completed cleanly; restart the game between sessions
        // the 3-flight cap (above) was lifted at the player's request on 2026-11-15; if the game
        // freezes or crashes after many reverts again, restart it between sessions
        public const int DefaultFlights = 3;

        public const int MaxFailsInARow = 3;
        const float PadSettleSeconds = 5, ResultPauseSeconds = 3, RevertTimeoutSeconds = 120;

        public enum Phase { Idle, WaitPad, Flying, Pause, Reverting }
        public Phase phase { get; private set; } = Phase.Idle;

        public int done, ok_count, fails_in_a_row, target;
        public string status_line = "";
        public bool status_ok = true;
        public readonly System.Collections.Generic.List<string> results = new();

        IGGuid vessel_id;
        double flight_end_ut;
        float phase_time;

        public TrainingPilot()
        {
            Instance = this;
            debug_mode_only = false;
            _page = new TrainingUI(this);
            FidoPilotsMgr.Instance.RegisterPilot("Training", this);
        }

        static LiftPilot Lift => LiftPilot.Instance;
        static VesselComponent Vessel => FidoPlugin.Instance.current_vessel?.VesselComponent;

        // ================================================================ start / stop

        public string StartProblem()
        {
            var vessel = Vessel;
            if (vessel == null) return "No vessel";
            if (!vessel.Landed) return "Put the vessel on the launch pad first";
            if (!RevertAllowed()) return "Revert is turned off in this game's difficulty options - training needs it";
            return null;
        }

        public void Begin()
        {
            if (isRunning) return;
            string problem = StartProblem();
            if (problem != null) { SetStatus(problem, false); return; }
            target = Mathf.Max(1, flights.V);
            done = ok_count = fails_in_a_row = 0;
            results.Clear();
            logger.LogInfo($"[Training] start: {target} flights");
            base.isRunning = true;
            StartFlight();
        }

        public void Stop(string why = "Stopped")
        {
            if (!isRunning) return;
            if (phase == Phase.Flying && Lift.isRunning) Lift.isRunning = false;
            phase = Phase.Idle;
            base.isRunning = false;
            SetStatus($"{why} after {done} of {target} flights ({ok_count} reached orbit)", why != "Stopped");
            logger.LogInfo($"[Training] {why}: {done}/{target} flown, {ok_count} reached orbit");
        }

        public override bool isRunning
        {
            get => base.isRunning;
            set
            {
                if (value == isRunning) return;
                if (value) Begin(); else Stop();
            }
        }

        public override void onReset()
        {
            // every pilot start resets the others (we start Lift ourselves) and the revert reloads
            // the flight scene - only leaving flight for good ends training
            if (!isRunning || phase == Phase.Reverting || InFlight()) return;
            Stop("Left flight");
        }

        // ================================================================ loop

        void StartFlight()
        {
            var vessel = Vessel;
            vessel_id = vessel.SimulationObject.GlobalId;
            var s = Lift.settings;
            s.auto_circularize.V = true;      // a flight ends once the orbit is circular
            SetStatus($"Flight {done + 1} of {target}: launching", true);
            logger.LogInfo($"[Training] flight {done + 1}/{target}: launch");
            phase = Phase.Flying;
            phase_time = Time.unscaledTime;
            Lift.isRunning = true;
            if (!Lift.isRunning) FlightOver();  // refused on the pad (the end status says why)
        }

        void FlightOver()
        {
            bool ok = Lift.result_ok;
            string why = string.IsNullOrEmpty(Lift.end_status) ? (ok ? "orbit" : "stopped") : Lift.end_status;
            done++;
            if (ok) { ok_count++; fails_in_a_row = 0; } else fails_in_a_row++;
            string line = $"{done}: {(ok ? "orbit" : "failed")} - {why}";
            results.Add(line);
            logger.LogInfo($"[Training] flight {done}/{target} {(ok ? "OK" : "FAILED")}: {why}");

            if (done >= target) { Stop("Finished"); return; }
            if (fails_in_a_row >= MaxFailsInARow) { Stop($"{MaxFailsInARow} failed flights in a row - stopped"); return; }

            flight_end_ut = GeneralTools.Current_UT;
            phase = Phase.Pause;
            phase_time = Time.unscaledTime;
            SetStatus($"Flight {done} {(ok ? "reached orbit" : "failed")} - reverting to launch", ok);
        }

        public override void Update()
        {
            if (!isRunning) return;
            float t = Time.unscaledTime - phase_time;
            switch (phase)
            {
                case Phase.Flying:
                    if (!Lift.isRunning) { FlightOver(); return; }
                    SetStatus($"Flight {done + 1} of {target}: {Lift.status}", true);
                    break;

                case Phase.Pause:
                    if (t < ResultPauseSeconds) return;
                    if (!Revert()) { Stop("Revert failed - stopped"); return; }
                    phase = Phase.Reverting;
                    phase_time = Time.unscaledTime;
                    break;

                case Phase.Reverting:
                {
                    // back at launch: the clock goes back to launch time and the vessel sits on the pad
                    var vessel = Vessel;
                    bool back = InFlight() && vessel != null && vessel.Landed && GeneralTools.Current_UT < flight_end_ut - 1;
                    if (back) { phase = Phase.WaitPad; phase_time = Time.unscaledTime; return; }
                    if (t > RevertTimeoutSeconds) Stop("Revert didn't complete - stopped");
                    break;
                }

                case Phase.WaitPad:
                {
                    var vessel = Vessel;
                    if (vessel == null || !vessel.Landed) { phase_time = Time.unscaledTime; return; }
                    SetStatus($"Flight {done + 1} of {target}: settling on the pad", true);
                    if (t >= PadSettleSeconds) StartFlight();
                    break;
                }
            }
        }

        // ================================================================ game

        bool Revert()
        {
            try
            {
                var game = GeneralTools.Game;
                if (!RevertAllowed()) return false;

                // The pause menu's own "Revert to Launch" (GlobalEscapeMenu.RevertToLaunchpad, private,
                // no confirmation dialog): exactly what clicking it does - leave mission control / the
                // training centre, hide the launchpad dialog, quit any tutorial, then the revert message.
                var menus = Resources.FindObjectsOfTypeAll<GlobalEscapeMenu>();
                var method = typeof(GlobalEscapeMenu).GetMethod("RevertToLaunchpad",
                    System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public);
                if (menus != null && menus.Length > 0 && method != null)
                {
                    method.Invoke(menus[0], null);
                    logger.LogInfo("[Training] revert to launch (pause menu's Revert to Launch)");
                    return true;
                }

                // fallback: the message that menu item sends
                logger.LogWarning("[Training] pause menu not found - sending the revert message directly");
                if (!game.Messages.TryCreateMessage(out RevertToLaunchMessage msg)) return false;
                msg.vesselID = vessel_id;
                game.Messages.Publish(msg);
                logger.LogInfo("[Training] revert to launch");
                return true;
            }
            catch (Exception e)
            {
                logger.LogError("[Training] revert: " + e);
                return false;
            }
        }

        static bool RevertAllowed()
        {
            try { return GeneralTools.Game.SessionManager.IsDifficultyOptionEnabled("AllowRevert"); }
            catch (Exception) { return false; }
        }

        static bool InFlight()
        {
            try
            {
                var state = GeneralTools.Game.GlobalGameState.GetGameState().GameState;
                return state == GameState.FlightView || state == GameState.Map3DView;
            }
            catch (Exception) { return false; }
        }

        void SetStatus(string text, bool ok)
        {
            status_line = text;
            status_ok = ok;
        }
    }
}
