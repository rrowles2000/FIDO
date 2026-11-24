using System;
using System.Collections.Generic;
using System.Linq;
using K2D2.Controller;
using KSP.Game;
using KSP.Sim.impl;
using KTools;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Mission
{
    /// <summary>
    /// Runs the active craft's mission plan one step at a time, each step driving one of the other
    /// pilots. Progress is written back onto the craft after every change, so a save captures it; a
    /// mission found "running" after a load comes back paused.
    /// </summary>
    public class MissionPilot : Pilot
    {
        public static MissionPilot Instance { get; private set; }

        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Mission");

        public MissionPlanData plan;
        PlanHolder holder;
        VesselComponent bound_vessel;
        int bound_parts = -1;

        StepRunner runner;
        int runner_index = -1;

        public string status_line = "";
        public bool status_ok = true;

        public MissionPilot()
        {
            Instance = this;
            debug_mode_only = false;
            _page = new MissionUI(this);
            K2D2PilotsMgr.Instance.RegisterPilot("Mission", this);
        }

        public bool CanStoreOnVessel => holder != null;
        public StepRunner Runner => runner;
        public bool IsDone => plan != null && plan.status == "done";
        public bool Editable(int index) => plan != null && (index > plan.current || index == plan.current && !isRunning);

        // ================================================================ binding

        void Bind(VesselComponent vessel)
        {
            bool same_vessel = vessel == bound_vessel;
            bound_vessel = vessel;
            bound_parts = PartCount(vessel);
            var old_holder = holder;
            holder = MissionStore.Find(vessel, out bool has_plan);
            if (!has_plan && same_vessel && plan != null)
            {
                // the part carrying the plan was staged away while paused: move the plan to what's left
                holder = MissionStore.Rehome(old_holder, plan, vessel);
                return;
            }
            plan = has_plan ? holder.Read() : null;
            runner = null;
            runner_index = -1;
            status_ok = true;
            status_line = "";
            if (plan == null) return;
            if (plan.status == "running")
            {
                plan.status = "paused";
                status_line = "Game loaded - press Go to carry on";
                Save();
            }
            PrepareCurrent();
        }

        static int PartCount(VesselComponent vessel) => vessel?.SimulationObject?.PartOwner?.Parts?.Count() ?? -1;

        void Save()
        {
            if (holder == null || plan == null) return;
            holder.Write(plan);
        }

        /// <summary>Called by the editor after any change to the plan.</summary>
        public void PlanChanged(int edited_index = -1)
        {
            if (plan == null) return;
            if (holder == null)
            {
                status_line = "This vessel has no command part with the K2-D2 mission module - the plan can't be saved";
                status_ok = false;
            }
            // steps added after the end reopen a finished mission
            if (plan.status == "done" && plan.current < plan.steps.Count)
                plan.status = "paused";
            if (!isRunning && (edited_index < 0 || edited_index == plan.current))
            {
                runner_index = -1;
                PrepareCurrent();
            }
            Save();
        }

        public void CreatePlan()
        {
            if (plan != null) return;
            plan = new MissionPlanData();
        }

        // ================================================================ running

        StepRunner RunnerFor(int index)
        {
            if (runner_index == index && runner != null)
            {
                runner.mission = plan;
                return runner;
            }
            var step = plan.steps[index];
            runner = step.Type?.create?.Invoke();
            if (runner != null)
            {
                runner.step = step;
                runner.mission = plan;
            }
            runner_index = index;
            return runner;
        }

        void PrepareCurrent()
        {
            if (plan == null || plan.status == "done" || plan.Current == null) return;
            try { RunnerFor(plan.current)?.Prepare(); }
            catch (Exception e) { logger.LogError("[Mission] prepare: " + e); }
        }

        public string GoProblem()
        {
            if (plan == null || plan.IsEmpty) return "No mission on this vessel";
            if (plan.status == "done") return "Mission complete";
            var errors = MissionValidator.Check(plan, Budget(plan), Estimates(plan), MissionValidator.BudgetFrom(plan, out _), ParkingPeriod(plan))
                .Where(i => i.error).ToList();
            if (errors.Count > 0) return errors[0].text;
            // on the pad (or landed) the next step to run must be Launch: a plan without one would run
            // e.g. a decouple part action on the pad (2026-11-23)
            if (OnGround())
            {
                int i = Math.Max(0, plan.current);
                while (i < plan.steps.Count && plan.steps[i].type == StepCatalog.Quicksave) i++;
                if (i < plan.steps.Count && plan.steps[i].type != StepCatalog.Launch)
                    return $"Step {i + 1} ({plan.steps[i].Summary}) would run on the pad - add a Launch step before it";
            }
            return null;
        }

        // ================================================================ stage checks

        Lift.Stage1.Stage1Model pad_model;
        VesselComponent pad_model_vessel;
        int pad_model_parts = -1;
        float pad_model_at = -100;

        /// <summary>
        /// The stages the plan's remaining burns come out of: before launch the stages above stage 1
        /// (read from the vessel), after it the vessel's stages as they are.
        /// </summary>
        public StageBudget Budget(MissionPlanData p)
        {
            var vessel = K2D2_Plugin.Instance?.current_vessel?.VesselComponent;
            if (vessel == null || p == null) return null;
            MissionValidator.BudgetFrom(p, out bool before_launch);
            if (!before_launch) return StageBudget.Live(vessel);
            int parts = PartCount(vessel);
            // nothing burns on the pad: rebuild only for another vessel or a change of parts (building logs every stage)
            bool retry = pad_model != null && !pad_model.valid && UnityEngine.Time.unscaledTime - pad_model_at > 5;
            if (pad_model == null || retry || pad_model_vessel != vessel || pad_model_parts != parts)
            {
                pad_model_at = UnityEngine.Time.unscaledTime;
                pad_model = Lift.Stage1.Stage1Model.Build(vessel, allowNoUpper: true);
                pad_model_vessel = vessel;
                pad_model_parts = parts;
            }
            return StageBudget.AfterStage1(pad_model);
        }

        public static List<double> Estimates(MissionPlanData p)
        {
            var body = K2D2_Plugin.Instance?.current_vessel?.VesselComponent?.mainBody;
            return body == null || p == null ? null : MissionValidator.Estimates(p, body.gravParameter, body.radius);
        }

        public static double ParkingPeriod(MissionPlanData p)
        {
            var vessel = K2D2_Plugin.Instance?.current_vessel?.VesselComponent;
            var body = vessel?.mainBody;
            if (body == null || p == null) return 0;
            MissionValidator.BudgetFrom(p, out bool before_launch);
            if (before_launch) return MissionValidator.ParkingPeriod(p, body.gravParameter, body.radius);
            try { return vessel.Orbit != null && vessel.Orbit.eccentricity < 1 ? vessel.Orbit.period : 0; }
            catch (Exception) { return 0; }
        }

        /// <summary>The craft is on the pad, landed or splashed down: still to launch.</summary>
        public static bool OnGround()
        {
            var v = K2D2_Plugin.Instance?.current_vessel?.VesselComponent;
            if (v == null) return false;
            try
            {
                var sit = v.Situation;
                return v.Landed || sit == KSP.Sim.impl.VesselSituations.PreLaunch || sit == KSP.Sim.impl.VesselSituations.Landed ||
                       sit == KSP.Sim.impl.VesselSituations.Splashed;
            }
            catch { return false; }
        }

        public void Go()
        {
            if (isRunning) return;
            string problem = GoProblem();
            if (problem != null)
            {
                SetStatus(problem, false);
                return;
            }
            StartCurrent();
        }

        void StartCurrent()
        {
            var step = plan.Current;
            var r = RunnerFor(plan.current);
            if (r == null)
            {
                SetStatus($"Step {plan.current + 1}: unknown step type '{step.type}'", false);
                return;
            }
            string problem = r.Check();
            if (problem != null)
            {
                plan.status = "paused";
                Save();
                base.isRunning = false;
                SetStatus($"Step {plan.current + 1} can't start: {problem}", false);
                return;
            }

            logger.LogInfo($"[Mission] step {plan.current + 1}: {step.Summary}");
            step.state = "";
            step.note = "";
            if (step.type == StepCatalog.Quicksave)
            {
                Quicksave();
                return;
            }
            plan.status = "running";
            Save();
            SetStatus($"Step {plan.current + 1}: {step.Label}", true);
            base.isRunning = true;
            try { r.Start(); }
            catch (Exception e)
            {
                logger.LogError("[Mission] start: " + e);
                Complete(StepResult.Failed, e.Message);
            }
        }

        // The plan is written with this step done before the save, so loading the quicksave carries
        // on from the step after it.
        void Quicksave()
        {
            var step = plan.Current;
            step.state = "done";
            step.note = "Saved";
            plan.current++;
            plan.status = plan.current >= plan.steps.Count ? "done" : "paused";
            Save();
            try
            {
                GameManager.Instance.Game.SaveLoadManager.ForceQuickSave();
                logger.LogInfo("[Mission] quicksaved");
            }
            catch (Exception e)
            {
                logger.LogError("[Mission] quicksave failed: " + e);
                step.note = "Quicksave failed - see log";
            }
            Next(true);
        }

        void Complete(StepResult result, string note)
        {
            var step = plan.Current;
            if (step == null) return;
            step.note = note ?? "";
            switch (result)
            {
                case StepResult.Done:
                    step.state = "done";
                    logger.LogInfo($"[Mission] step {plan.current + 1} done: {note}");
                    plan.current++;
                    holder = MissionStore.Rehome(holder, plan, K2D2_Plugin.Instance.current_vessel?.VesselComponent);
                    bound_vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
                    bound_parts = PartCount(bound_vessel);
                    Next(true);
                    return;
                case StepResult.Paused:
                    logger.LogInfo($"[Mission] step {plan.current + 1} paused: {note}");
                    plan.status = "paused";
                    SetStatus($"Step {plan.current + 1} paused: {note} - press Go to carry on", false);
                    break;
                default:
                    step.state = "failed";
                    logger.LogWarning($"[Mission] step {plan.current + 1} failed: {note}");
                    plan.status = "paused";
                    SetStatus($"Step {plan.current + 1} failed: {note}", false);
                    break;
            }
            Save();
            base.isRunning = false;
        }

        /// <summary>Moves on to plan.current: runs it straight away if it is set to auto-continue, else waits for Go.</summary>
        void Next(bool allow_auto)
        {
            runner = null;
            runner_index = -1;
            if (plan.current >= plan.steps.Count)
            {
                plan.status = "done";
                Save();
                base.isRunning = false;
                SetStatus("Mission complete", true);
                logger.LogInfo("[Mission] complete");
                return;
            }
            var step = plan.Current;
            PrepareCurrent();
            var previous = plan.current > 0 ? plan.steps[plan.current - 1] : null;
            // the stage-1 burn leaves periapsis in the atmosphere: never wait between Launch and Circularise
            bool forced = step.type == StepCatalog.Circularise && previous?.type == StepCatalog.Launch && previous.state == "done";
            if (allow_auto && (step.auto || forced))
            {
                StartCurrent();
                return;
            }
            plan.status = "paused";
            Save();
            base.isRunning = false;
            SetStatus($"Next: {step.Summary} - press Go", true);
        }

        public void Stop()
        {
            if (!isRunning) return;
            runner?.Stop();
            plan.status = "paused";
            Save();
            base.isRunning = false;
            SetStatus($"Stopped at step {plan.current + 1} - press Go to run it again", false);
        }

        public void Skip()
        {
            if (plan?.Current == null) return;
            if (isRunning) runner?.Stop();
            base.isRunning = false;
            var step = plan.Current;
            step.state = "skipped";
            step.note = "";
            logger.LogInfo($"[Mission] step {plan.current + 1} skipped");
            plan.current++;
            Next(false);
        }

        public void ResetMission()
        {
            if (plan == null) return;
            if (isRunning) runner?.Stop();
            base.isRunning = false;
            plan.ResetProgress();
            runner = null;
            runner_index = -1;
            PrepareCurrent();
            Save();
            SetStatus("Mission reset to step 1", true);
        }

        void SetStatus(string text, bool ok)
        {
            status_line = text;
            status_ok = ok;
        }

        public override bool isRunning
        {
            get => base.isRunning;
            set
            {
                if (value == isRunning) return;
                if (value) Go();
                else Stop();
            }
        }

        public override void onReset()
        {
            // every pilot start resets the others, and so does each vessel switch; only leaving flight stops a mission
            if (!isRunning || InFlight()) return;
            runner?.Stop();
            plan.status = "paused";
            Save();
            base.isRunning = false;
            SetStatus("Left flight - paused", false);
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

        public override void Update()
        {
            var vessel = K2D2_Plugin.Instance.current_vessel?.VesselComponent;
            // a running mission stays with its plan through the vessel switches its steps make;
            // otherwise follow the active vessel (and re-find the plan if parts came or went)
            if (!isRunning && vessel != null && (vessel != bound_vessel || PartCount(vessel) != bound_parts))
                Bind(vessel);

            if (!isRunning || runner == null) return;
            StepResult result;
            try { result = runner.Update(); }
            catch (Exception e)
            {
                logger.LogError("[Mission] " + e);
                runner.Stop();
                Complete(StepResult.Failed, e.Message);
                return;
            }
            if (result != StepResult.Running)
                Complete(result, runner.status);
        }
    }
}
