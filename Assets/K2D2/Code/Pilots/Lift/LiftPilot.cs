
using K2D2.KSPService;
using KTools;
using K2D2.Controller;
using K2D2.Node;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift
{
    public class LiftPilot : Pilot
    {
        public enum LiftStatus
        {
            Off,
            Ascent,
            Coasting,
            Stage1Burn,
            Adjust,
            Circularize
        }

        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Lift");

        public static LiftPilot Instance { get; set; }

        internal  LiftSettings settings;

        internal  LiftAscentPath ascent_path = null;

        KSPVessel current_vessel;

        Vector3d direction = Vector3d.zero;

        // One pipeline (status -> sub pilot) per lift system, see LiftSettings.LiftMode. A status a
        // system has no entry for is skipped (Classic and New have no Stage1Burn).
        readonly System.Collections.Generic.Dictionary<LiftMode, System.Collections.Generic.Dictionary<LiftStatus, ExecuteController>> pipelines = new();
        readonly System.Collections.Generic.Dictionary<LiftMode, AscentBase> ascents = new();

        Stage1Burn stage1_burn = null;

        // the system flying the current run; fixed at start so switching the selector mid-flight
        // can't swap controllers under the vessel
        LiftMode active_mode;
        internal LiftMode mode => isRunning ? active_mode : settings.mode.V;

        internal AscentBase ascent => ascents[mode];

        public ExecuteController current_subpilot = null;

        public LiftPilot()
        {
            settings = new LiftSettings();
            ascent_path = new LiftAscentPath(settings);
            _page = new LiftUI(this);

            current_vessel = K2D2_Plugin.Instance.current_vessel;
       
            BuildPipelines();

            Instance = this;

            debug_mode_only = false;

            K2D2PilotsMgr.Instance.RegisterPilot("Lift", this);
        }

        void BuildPipelines()
        {
            // Classic - the original K2-D2 ascent (Classic/), unchanged
            var classic_ascent = new Classic.AscentClassic(settings, ascent_path);
            ascents[LiftMode.Classic] = classic_ascent;
            pipelines[LiftMode.Classic] = new()
            {
                [LiftStatus.Ascent] = classic_ascent,
                [LiftStatus.Coasting] = new Classic.CoastingClassic(this, settings),
                [LiftStatus.Adjust] = new Adjust(settings, classic_ascent),
                [LiftStatus.Circularize] = new Classic.FinalCircularizeClassic(this, settings),
            };

            // Optimiser - our system (Controlers/ + Stage1/)
            var ascent_opt = new Ascent(settings, ascent_path);
            stage1_burn = new Stage1Burn(this, settings);
            ascents[LiftMode.Optimiser] = ascent_opt;
            pipelines[LiftMode.Optimiser] = new()
            {
                [LiftStatus.Ascent] = ascent_opt,
                [LiftStatus.Coasting] = new Coasting(this, settings),
                [LiftStatus.Stage1Burn] = stage1_burn,
                [LiftStatus.Adjust] = new Adjust(settings, ascent_opt),
                [LiftStatus.Circularize] = new FinalCircularize(this, settings),
            };

            // New - the third system, being developed in Next/ (Phase 0: layered loop flown with
            // the Optimiser's guidance as its policy). It hands over to the Classic coast - which,
            // unlike ours, ignores the stage-1 disposal flag the New system doesn't use - and to
            // our FinalCircularize, which has the past-apoapsis fix.
            var ascent_next = new Next.AscentNext(this, settings);
            ascents[LiftMode.New] = ascent_next;
            pipelines[LiftMode.New] = new()
            {
                [LiftStatus.Ascent] = ascent_next,
                [LiftStatus.Coasting] = new Classic.CoastingClassic(this, settings),
                // stage 1's leftover burned at apoapsis, then separation (LiftSettings.next_s1_ap_burn)
                [LiftStatus.Stage1Burn] = new Stage1Burn(this, settings),
                [LiftStatus.Adjust] = new Adjust(settings, ascent_next),
                [LiftStatus.Circularize] = new FinalCircularize(this, settings),
            };
        }

        // first status at or after `from` that the active system has a sub pilot for (Off if none)
        LiftStatus FirstAvailable(LiftStatus from)
        {
            var pipeline = pipelines[active_mode];
            for (var s = from; s != LiftStatus.Off; s = s.Next())
                if (pipeline.ContainsKey(s))
                    return s;
            return LiftStatus.Off;
        }

        public override void onReset()
        {
            isRunning = false;
        }

        LiftStatus _status = LiftStatus.Off;
        internal LiftStatus status
        {
            get { return _status; }
            set
            {
                if (_status == value)
                    return;
                _status = value;
                switch (value)
                {
                    case LiftStatus.Off:
                    {
                        if (active_mode == LiftMode.New && ascents.TryGetValue(LiftMode.New, out var next_ascent))
                            (next_ascent as Next.AscentNext)?.LogFlightResult(result_ok, end_status, last_circ_dv, last_s1_burn_dv, last_s1_sep_ap, last_s1_sep_pe);
                        // stop
                        if (current_vessel != null)
                        {
                            current_vessel.SetThrottle(0);
                            // Roll program (see Ascent.cs) drives this as a raw control axis input,
                            // not through SAS - if the pilot gets stopped mid-correction, release it
                            // here too so a stray roll command doesn't keep firing after the
                            // autopilot itself has stopped.
                            current_vessel.Roll = 0;
                        }

                        current_subpilot = null;
                    }
                        break;
                    default:
                        pipelines[active_mode].TryGetValue(value, out current_subpilot);
                        break;
                }

                if (current_subpilot != null)
                    current_subpilot.Start();
            }
        }

        public override bool isRunning
        {
            get { return _status != LiftStatus.Off; }
            set
            {
                if (value == isRunning)
                    return;

                if (!value)
                {
                    status = LiftStatus.Off;
                }
                else
                {
                    // reset controller to desactivate other controllers.
                    K2D2_Plugin.ResetControllers();
                    OnStartController();
                }

                // send call backs
                base.isRunning = value;
                // the chosen system had nothing to fly from start_status (OnStartController left
                // it Off) - report the stop too, so the Start button doesn't stay latched
                if (value && !isRunning)
                    base.isRunning = false;
            }
        }

        LiftStatus start_status = LiftStatus.Ascent;

        void OnStartController()
        {
            result_ok = false;
            end_status = "";
            active_mode = settings.mode.V;
            last_circ_dv = 0;
            last_s1_burn_dv = -1;
            last_s1_sep_ap = last_s1_sep_pe = double.NaN;
            logger.LogInfo($"[Lift] starting with the {active_mode} lift system");
            var first = FirstAvailable(start_status);
            if (first == LiftStatus.Off)
                end_status = $"The {active_mode} lift system has no {start_status} step";
            status = first;
        }

        /// <summary>Starts the pilot part-way through (the Mission tab's Circularise step starts at Adjust).</summary>
        public void StartAt(LiftStatus first)
        {
            if (isRunning) return;
            start_status = first;
            isRunning = true;
            start_status = LiftStatus.Ascent;
        }

        internal Stage1Burn Stage1 => stage1_burn;

        // the circularisation burn FinalCircularize planned this run (for the New system's summary)
        internal double last_circ_dv;
        // the stage-1 apoapsis burn Stage1Burn planned this run (-1: it didn't run)
        internal double last_s1_burn_dv = -1;
        // the orbit when stage 1 separated (Stage1Burn), km; NaN = it didn't
        internal double last_s1_sep_ap = double.NaN, last_s1_sep_pe = double.NaN;

        internal bool result_ok = false;
        internal string end_status;

        public void EndLiftPilot(bool result_ok, string end_status)
        {
            this.result_ok = result_ok;
            this.end_status = end_status;
            // Was: status = LiftStatus.Off directly. That flips the getter's answer
            // (isRunning reads _status != Off) but skips the isRunning SETTER entirely, so
            // base.isRunning's own backing field never gets updated and is_running_event never
            // fires. Node/Landing end their run via isRunning = false (see NodeExPilot.Stop(),
            // LandingPilot's touchdown/abort paths) for exactly this reason - anything bound to
            // is_running_event (LiftUI's run_button toggle state, K2's avatar SetRunning) needs
            // that callback to update, or it stays stuck showing "running" after the pilot has
            // actually finished on its own (as opposed to being stopped via the Brake/Start
            // button, which already goes through this setter).
            isRunning = false;
        }

        public WarpTo warp_to = new WarpTo();


        public void NextMode()
        {
            // a system that has no sub pilot for the next status skips it; past the last one
            // the run is over (Next() wraps round to Off)
            var next = status.Next();
            if (next != LiftStatus.Off)
                next = FirstAvailable(next);
            if (next == LiftStatus.Off)
                EndLiftPilot(true, end_status);
            else
                status = next;
        }

        public override void Update()
        {
        
            if (current_vessel == null) return;

            if (isRunning || page.isVisible)
                ascent.computeValues(status == LiftStatus.Ascent);

            if (!isRunning)
                return;

            var subpilot = current_subpilot;
            if (subpilot != null)
            {
                subpilot.Update();

                // A subpilot's Update() can end the whole run right here - Coasting's Ap-under-
                // atmosphere-limit check and FinalCircularize's burn-complete case (see Final.cs)
                // both call lift.EndLiftPilot() directly rather than just setting finished, so
                // that is_running_event actually fires (see EndLiftPilot's own comment about why
                // it goes through the isRunning setter instead of setting status directly). That
                // synchronously nulls current_subpilot via the status setter's Off case, before
                // we get back here. Re-checking the field itself (not the stale local) means we
                // don't NullReferenceException on a null current_subpilot, and don't call
                // NextMode() a second time on top of a run that already ended itself.
                if (current_subpilot == subpilot && subpilot.finished)
                    NextMode();
            }
        }
    }
}
