using K2D2.KSPService;
using System;
using System.Collections.Generic;
using KSP.Game;
using KSP.Modules;
using KSP.Sim;
using KSP.Sim.impl;
using KTools;
using UnityEngine;
using UnityEngine.UIElements;
using K2D2.UI;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// NEW LIFT SYSTEM (LiftSettings.mode = New), Phase 0 - see the NN ascent report
    /// (reports/k2d2-behavioural-nn-ascent-report.rtf), Sections 3, 5, 7 and 11.
    ///
    /// The layered loop the neural network will plug into, flown by the behaviour model:
    ///
    ///   every frame   Update(): sample, run any due control ticks, re-apply the latched command
    ///   every 0.1 s   Tick():   arbiter (L3) -> policy (L2) -> shield (L1) -> latch
    ///   every frame   game SAS (L0) holds the latched attitude; throttle re-set from the latch
    ///
    /// Timing (report Section 7.1): ticks are counted in game time (UT), not frames, so the
    /// controller behaves the same at any frame rate. No time passing (paused) runs no tick; a jump
    /// over MaxGapS (staging freeze, warp, a long hitch) is skipped as one normal tick rather than
    /// integrated across, with the shield's rate limits still in force. At most MaxCatchUpTicks run in one frame. Only this controller writes
    /// throttle and attitude during a run (LiftPilot resets the other pilots before starting).
    ///
    /// Settings are snapshotted at Start, so editing the Lift tab mid-flight can't change a run.
    ///
    /// The policy is the behaviour model (Behaviours.cs, spec 2026-11-14): on the pad
    /// BehaviourTrainer trains its weights in the calibrated simulator, LaunchCheck simulates the
    /// result and refuses a stage 1 that can't get there, and after the flight the calibration,
    /// the real score and the stage-1 burn loss are learned (CraftMemory). Heading correction goes
    /// through the shield; loss of control for 3 s stops the run. The pre-behaviour-model path
    /// (pitch kick + gravity turn, plan grid, PlanLearner, throttle holds, drift protection, the
    /// shield's AoA and max-Q limits) was removed 2026-11-15 - Archive/lift-legacy-2026-11-15/.
    /// </summary>
    public class AscentNext : AscentBase
    {
        public const double TickS = 0.1;          // 10 Hz control rate
        public const double MaxGapS = 1.0;        // larger UT jumps reset the shield
        public const int MaxCatchUpTicks = 3;
        public const double LogPeriodS = 1.0;     // [Next] telemetry line

        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Ascent.Next");

        readonly LiftPilot lift;
        readonly LiftSettings settings;
        readonly KSPVessel current_vessel;

        // layers
        readonly BehaviourPolicy behaviour_policy = new BehaviourPolicy(); // the network goes here later (Phase 2)
        double prev_speed = double.NaN, prev_speed_ut, speed_rate;
        IAscentPolicy policy;
        readonly SafetyShield shield = new();
        readonly Arbiter arbiter = new();

        // per-run snapshot
        BodyDescriptor body;
        PolicyContext ctx;
        float heading_deg;
        bool heading_correction_on;
        bool started;

        // loop state
        double last_tick_ut;
        int ticks;
        double rate_window_ut;
        int rate_window_ticks;
        float measured_rate_hz;
        double next_log_ut;
        double launch_ut;

        // latched output - written only by Tick, applied every frame
        AscentCommand latched = new AscentCommand(90, 0);
        float heading_correction;

        // pre-launch check (pad only) and what it decided
        LaunchCheck.Result check;
        double predicted_score = double.NaN; // behaviour model: the trained weights' simulated score
        string pending_refusal;
        double max_q_so_far;

        // last tick, for the UI and log
        AscentObservation obs;
        AscentCommand policy_cmd;
        string status = "";

        // ---- nose tracking (drift) - the 2026-11-13 flips started as the nose slowly leaving its
        // command with zero AoA commanded, seconds before loss of control. Logged so each vessel's
        // onset q / Mach is on record (ksplog dataset), not only whether it flipped.
        public const double DriftDeg = 1.5, DriftConfirmS = 0.5, DriftMinQKPa = 2;
        // drift below this surface speed is the liftoff settling (it was the kick before 2026-11-15)
        // and isn't learned from
        public const double EarlyDriftSpeed = 70;
        double drift_since, drift_calm_since;
        bool drift_logged;
        bool in_drift;
        int drift_episodes;
        double episode_peak, episode_start;
        double drift_peak_deg, drift_peak_q, drift_peak_mach, drift_peak_alt;
        double drift_onset_q = double.NaN, drift_onset_mach, drift_onset_alt, drift_onset_t;
        double prev_cmd_elev = 90;
        StabilityCheck.Result stability;
        LaunchCheck.Calibration calibration = new LaunchCheck.Calibration();
        readonly List<Calibrator.Sample> flight_samples = new();
        double next_sample_t;
        // for CraftMemory: the last drift after liftoff, the first loss-of-control point, top Mach
        double learn_onset_peak;
        bool episode_counts;
        double learn_onset_mach = double.NaN, learn_onset_q, learn_onset_thrust, loc_mach = double.NaN, loc_q, loc_thrust, loc_alt, max_mach;

        // full-throttle thrust, estimated as thrust / throttle (as LaunchCheck does in the sim)
        double full_thrust_est;

        /// <summary>UT of liftoff (-1 before it), for LiftPilot's telemetry line.</summary>
        public double LaunchUT => launch_ut;

        // Adjust reads the Ap rate (see computeValues)
        float last_ap_km;

        // for the end-of-flight summary (LogFlightResult)
        K2D2.Lift.Stage1.Stage1Model model;
        AscentObservation cut_obs;
        bool have_cut;
        bool summary_logged;
        double launch_mass;
        readonly System.Collections.Generic.HashSet<string> known_vessels = new();

        public AscentNext(LiftPilot lift, LiftSettings settings)
        {
            this.lift = lift;
            this.settings = settings;
            current_vessel = K2D2_Plugin.Instance.current_vessel;
            arbiter.log = msg => logger.LogInfo(msg);
        }

        public override void Start()
        {
            base.Start();
            started = false;

            var vessel = current_vessel?.VesselComponent;
            body = BodyDescriptor.From(vessel?.mainBody, settings.destination_Ap_km.V);
            ctx = PolicyContext.From(settings);
            heading_deg = settings.heading.V;
            heading_correction_on = settings.heading_correction.V;

            policy = behaviour_policy;
            prev_speed = double.NaN;
            speed_rate = 0;
            shield.Reset();
            arbiter.Reset();
            latched = new AscentCommand(90, 0);
            heading_correction = 0;
            ticks = 0;
            rate_window_ticks = 0;
            measured_rate_hz = 0;
            next_log_ut = 0;
            launch_ut = -1;
            last_ap_km = 0;
            delta_ap_per_second = 0;
            status = "Starting";

            max_q_so_far = 0;
            pending_refusal = null;
            check = null;
            model = null;
            have_cut = false;
            summary_logged = false;
            launch_mass = vessel?.totalMass ?? 0;
            full_thrust_est = 0;
            flight_samples.Clear();
            next_sample_t = 0;
            drift_since = -1;
            drift_calm_since = -1;
            drift_logged = false;
            in_drift = false;
            drift_episodes = 0;
            learn_onset_mach = double.NaN;
            learn_onset_peak = 0;
            episode_counts = false;
            loc_mach = double.NaN;
            max_mach = 0;
            DebrisTracker.Snapshot();
            drift_peak_deg = drift_peak_q = drift_peak_mach = drift_peak_alt = 0;
            drift_onset_q = double.NaN;
            prev_cmd_elev = 90;
            stability = vessel != null && vessel.Landed ? StabilityCheck.ForVessel(vessel) : null;
            if (stability != null) logger.LogInfo("[Next] STABILITY " + stability.Summary() +
                                                  (StabilityCheck.lastDiagnostic.Length > 0 ? " | " + StabilityCheck.lastDiagnostic : ""));
            RememberVessels();

            SASTool.setAutoPilot(AutopilotMode.StabilityAssist);

            PlanetLog.Log(logger, vessel, settings.destination_Ap_km.V);
            RunLaunchCheck(vessel);
            logger.LogInfo($"[Next] start: policy {policy.Name}, {1 / TickS:n0} Hz, target Ap {settings.destination_Ap_km.V} km " +
                           $"(circular {body.orbitSpeedAtTarget:n0} m/s), heading {heading_deg:n1}°, max throttle {ctx.maxThrottle:n2}, " +
                           $"max Q {ctx.maxQKPa:n0} kPa (scored), heading Q·AoA {ctx.qAlphaKPaDeg:n1} kPa·°; weights: {ctx.weights}");
        }

        // ---------------------------------------------------------------- frame loop

        public override void Update()
        {
            if (finished) return;
            var vessel = current_vessel?.VesselComponent;
            if (vessel == null || !body.Valid)
            {
                lift.EndLiftPilot(false, "New lift system: no vessel or planet data");
                return;
            }

            // a refusal decided at Start is acted on here: ending the run from inside Start would
            // re-enter LiftPilot's status setter mid-switch
            if (pending_refusal != null)
            {
                logger.LogWarning("[Next] " + pending_refusal);
                current_vessel.SetThrottle(0);
                lift.EndLiftPilot(false, pending_refusal);
                return;
            }

            double now = GeneralTools.Current_UT;
            if (!started)
            {
                started = true;
                last_tick_ut = now - TickS; // tick straight away
                rate_window_ut = now;
            }

            double elapsed = now - last_tick_ut;
            if (elapsed > MaxGapS)
            {
                // Don't integrate across the gap and don't reset the smoothing either (a reset
                // would let the next command jump straight to the policy's value): run one normal
                // tick and carry on from now.
                logger.LogInfo($"[Next] time jumped {elapsed:n2} s - backlog dropped");
                last_tick_ut = now - TickS;
            }

            if (now - last_tick_ut >= TickS)
            {
                obs = Sample(vessel, now); // one sample per frame, shared by any catch-up ticks
                if (!double.IsNaN(prev_speed) && now > prev_speed_ut)
                    speed_rate += 0.2 * ((obs.airSpeed - prev_speed) / (now - prev_speed_ut) - speed_rate);
                prev_speed = obs.airSpeed;
                prev_speed_ut = now;
                obs.speedRate = speed_rate;
                int run = 0;
                while (now - last_tick_ut >= TickS && run < MaxCatchUpTicks)
                {
                    last_tick_ut += TickS;
                    run++;
                    if (!Tick(vessel)) return; // the run ended or handed over
                }
                if (now - last_tick_ut >= TickS) last_tick_ut = now; // too far behind: drop the backlog
            }

            ApplyLatched();
        }

        // ---------------------------------------------------------------- control tick

        /// <returns>false when this controller is done (handed over or the run ended)</returns>
        bool Tick(VesselComponent vessel)
        {
            ticks++;
            rate_window_ticks++;
            if (obs.ut - rate_window_ut >= 5)
            {
                measured_rate_hz = (float)(rate_window_ticks / (obs.ut - rate_window_ut));
                rate_window_ticks = 0;
                rate_window_ut = obs.ut;
            }
            if (launch_ut < 0 && !obs.landed) launch_ut = obs.ut;

            // L2: the policy proposes
            policy_cmd = obs.valid ? policy.Decide(obs, body, ctx) : latched;

            // L3: behaviours decide what actually gets flown
            var action = arbiter.Decide(obs, body, ctx, policy_cmd, shield.report.lossOfControl,
                                        K2D2.Controller.StagingSettings.auto_staging.V, out AscentCommand wanted);
            status = arbiter.status;

            if (arbiter.behaviour == Behaviour.Fly) max_mach = Math.Max(max_mach, obs.mach);
            if (shield.report.lossOfControl && double.IsNaN(loc_mach))
            {
                loc_mach = obs.mach;
                loc_q = obs.qKPa;
                loc_thrust = obs.thrustKN;
                loc_alt = obs.altitude;
            }

            // L1: everything passes the shield
            var safe = shield.Apply(wanted, obs, ctx, TickS);

            if (shield.report.rejected)
                logger.LogWarning($"[Next] policy {policy.Name} gave an invalid command - the shield kept the last good one");

            heading_correction = shield.LimitHeading(heading_correction_on ? HeadingCorrection() : 0,
                                                     obs, safe.elevationDeg, ctx, TickS);

            TrackDrift();
            prev_cmd_elev = safe.elevationDeg;
            if (launch_ut >= 0 && obs.hasPath && obs.ut - launch_ut >= next_sample_t &&
                (arbiter.behaviour == Behaviour.Fly || arbiter.behaviour == Behaviour.ClearTower))
            {
                next_sample_t = obs.ut - launch_ut + LaunchCheck.PlanStepS;
                flight_samples.Add(new Calibrator.Sample { t = obs.ut - launch_ut, v = obs.airSpeed, path = obs.pathDeg, altKm = obs.altitude / 1000 });
            }
            latched = safe;
            Log(false);

            if (action == ArbiterAction.HandOff || action == ArbiterAction.EndRun)
            {
                cut_obs = obs;
                have_cut = true;
            }

            switch (action)
            {
                case ArbiterAction.Stage:
                    vessel.ActivateNextStage();
                    break;
                case ArbiterAction.HandOff:
                    latched.throttle = 0;
                    current_vessel.SetThrottle(0);
                    Log(true);
                    finished = true;
                    return false;
                case ArbiterAction.EndRun:
                    current_vessel.SetThrottle(0);
                    Log(true);
                    lift.EndLiftPilot(false, arbiter.end_status);
                    return false;
            }
            return true;
        }

        // ---------------------------------------------------------------- launch check

        // On the pad: train the behaviour model for the vessel in the calibrated simulator, simulate
        // the result, and refuse a stage 1 that can't reach the target (setting). Off the pad (a
        // mid-air start) it's skipped.
        void RunLaunchCheck(VesselComponent vessel)
        {
            if (vessel == null || !vessel.Landed)
            {
                logger.LogInfo("[Next] launch check skipped: not on the ground");
                return;
            }
            model = K2D2.Lift.Stage1.Stage1Model.Build(vessel, allowNoUpper: true);
            model?.SetSurfaces(stability);
            CraftMemory.ApplyTo(ref ctx, model);
            logger.LogInfo($"[Next] MEMORY {CraftMemory.Key(model)}: {CraftMemory.Describe(model)}");
            calibration = CraftMemory.GetCalibration(model);
            logger.LogInfo($"[Next] CALIB using {calibration}");
            var trained = BehaviourTrainer.TrainForLaunch(model, ref ctx, heading_deg, settings.destination_Ap_km.V, calibration, CapKm(), vessel.Name, logger);
            predicted_score = trained?.score ?? double.NaN;
            check = LaunchCheck.Run(model, ctx, heading_deg, settings.destination_Ap_km.V, calibration);
            logger.LogInfo($"[Next] launch check: {check.Summary()}");
            if (check.verdict == LaunchCheck.Verdict.Unknown) return;

            if (check.plan != null)
                foreach (var p in check.plan)
                    logger.LogInfo($"[Next] PLAN T+{p.t:n1} alt {p.altKm:n2} km, v {p.v:n0} m/s, path {p.path:n1}, cmd {p.cmd:n1}, " +
                                   $"q {p.q:n1} kPa, thrust {p.thrustKN:n0} kN, thr {p.throttle:n2}, mass {p.mass:n2} t, Ap {p.apKm:n1} Pe {p.peKm:n1} km");
            ModelExport.Write(model, ctx, heading_deg, settings.destination_Ap_km.V, check, calibration, vessel.Name);
            if (check.verdict == LaunchCheck.Verdict.Short && settings.next_refuse_short.V)
                pending_refusal = $"Launch refused - {check.Short()}. Turn off 'Refuse if stage 1 is short' to fly anyway.";
            else if (check.verdict == LaunchCheck.Verdict.NoCircularise)
                logger.LogWarning("[Next] launch check: reaches the target but can't circularise - flying anyway");
        }

        double CapKm() => settings.Stage1CapKm(body.hasAtmosphere ? body.atmosphereDepth / 1000 : 0);

        // Nose vs what SAS was asked to hold last tick, while flying in real air. Every episode is
        // logged - onset (over DriftDeg for DriftConfirmS) and recovery (under half that for 1 s) -
        // because on 2026-11-13 the first one was only the kick overshooting at T+11; the one that
        // flipped the vessel started at T+41 (Mach 0.98) and went unlogged. drift_onset_* keeps
        // the latest onset: the one before a loss of control is the interesting one.
        void TrackDrift()
        {
            if (arbiter.behaviour != Behaviour.Fly || !obs.hasPath || obs.qKPa < DriftMinQKPa) { drift_since = -1; return; }
            double err = obs.noseDeg - prev_cmd_elev;
            // sideways counts too: the 83 t vessel's departures were partly sideways
            double abs_err = Math.Sqrt(err * err + obs.sideslipDeg * obs.sideslipDeg);
            if (abs_err > drift_peak_deg)
            {
                drift_peak_deg = abs_err;
                drift_peak_q = obs.qKPa;
                drift_peak_mach = obs.mach;
                drift_peak_alt = obs.altitude;
            }
            if (in_drift)
            {
                episode_peak = Math.Max(episode_peak, abs_err);
                if (episode_counts) learn_onset_peak = Math.Max(learn_onset_peak, episode_peak);
                if (abs_err < DriftDeg / 2)
                {
                    if (drift_calm_since < 0) drift_calm_since = obs.ut;
                    if (obs.ut - drift_calm_since >= 1)
                    {
                        in_drift = false;
                        logger.LogInfo($"[Next] DRIFT #{drift_episodes} recovered after {obs.ut - episode_start:n1} s, peak {episode_peak:n1}°");
                    }
                }
                else drift_calm_since = -1;
                return;
            }
            if (abs_err < DriftDeg) { drift_since = -1; return; }
            if (drift_since < 0) drift_since = obs.ut;
            if (obs.ut - drift_since < DriftConfirmS) return;

            in_drift = true;
            drift_logged = true;
            drift_episodes++;
            drift_calm_since = -1;
            episode_start = drift_since;
            episode_peak = abs_err;
            drift_onset_q = obs.qKPa;
            drift_onset_mach = obs.mach;
            drift_onset_alt = obs.altitude;
            drift_onset_t = obs.ut - launch_ut;
            bool early = obs.airSpeed < EarlyDriftSpeed;
            string phase = early ? " (just after liftoff)" : "";
            episode_counts = !early;
            if (!early)
            {
                // drift is scored (BehaviourTrainer.RealScore) and remembered, not overridden
                learn_onset_peak = abs_err;
                learn_onset_mach = obs.mach;
                learn_onset_q = obs.qKPa;
                learn_onset_thrust = obs.thrustKN;
            }
            logger.LogWarning($"[Next] DRIFT #{drift_episodes} onset T+{drift_onset_t:n1} at {obs.altitude / 1000:n2} km{phase}: nose {err:+0.0;-0.0}° in pitch / " +
                              $"{obs.sideslipDeg:+0.0;-0.0}° sideways off its command (roll {obs.rollRateDegS:+0.0;-0.0}°/s, " +
                              $"ctl p {obs.ctlPitch:+0.00;-0.00} y {obs.ctlYaw:+0.00;-0.00} r {obs.ctlRoll:+0.00;-0.00}) " +
                              $"for {DriftConfirmS:n1} s, commanded AoA {prev_cmd_elev - obs.pathDeg:+0.0;-0.0}°, q {obs.qKPa:n1} kPa, " +
                              $"Mach {obs.mach:n2}, throttle {obs.throttle:n2}");
        }

        // ---------------------------------------------------------------- end-of-flight summary

        static System.Collections.Generic.List<VesselComponent> AllVessels()
        {
            try { return GameManager.Instance.Game.SpaceSimulation.UniverseModel.GetAllVessels(); }
            catch (System.Exception) { return new System.Collections.Generic.List<VesselComponent>(); }
        }

        void RememberVessels()
        {
            known_vessels.Clear();
            foreach (var v in AllVessels())
                if (v?.SimulationObject != null) known_vessels.Add(v.SimulationObject.GlobalId.ToString());
        }

        // stage 1's own Δv left (vacuum) at mass m, from the model's stage-1 phases
        static double Stage1LeftDv(K2D2.Lift.Stage1.Stage1Model m, double mass)
        {
            if (m?.phases == null) return double.NaN;
            const double g0 = 9.80665;
            double left = 0;
            foreach (var ph in m.phases)
            {
                if (mass >= ph.startMass - 0.01) left += ph.deltaVVac;       // phase not started
                else if (mass > ph.endMass) left += ph.ispVac * g0 * System.Math.Log(mass / ph.endMass);
            }
            return left;
        }

        static string Delta(double predicted, double actual, string unit, string fmt = "n0")
        {
            if (double.IsNaN(predicted)) return $"{actual.ToString(fmt)}{unit}";
            return $"{predicted.ToString(fmt)} -> {actual.ToString(fmt)}{unit} ({(actual - predicted).ToString("+#,0.#;-#,0.#;0")})";
        }

        /// <summary>
        /// Called by LiftPilot when a New-system run stops for any reason (complete, ended by a
        /// behaviour, or stopped by hand). Logs where the vessel ended up, the launch check's
        /// prediction against what was flown, and any debris the flight left behind - none of
        /// which was in the log for the 2026-11-13 flights.
        /// </summary>
        public void LogFlightResult(bool ok, string why, double circDv, double s1BurnDv = -1,
                                    double sepAp = double.NaN, double sepPe = double.NaN)
        {
            if (summary_logged || launch_ut < 0) return; // never left the pad (refused / aborted)
            summary_logged = true;
            var rec = new CraftMemory.FlightRecord { when = DateTime.Now.ToString("yyyy-MM-dd HH:mm"), ok = ok, simScore = predicted_score,
                                                     sepPeKm = sepPe, driftDeg = drift_peak_deg, maxQKPa = max_q_so_far };
            try
            {
                var vessel = current_vessel?.VesselComponent;
                var orbit = vessel?.Orbit;
                if (orbit == null) return;
                var cb = orbit.referenceBody;
                double R = cb.radius;
                double now = GeneralTools.Current_UT;
                double ap = (orbit.Apoapsis - R) / 1000, pe = (orbit.Periapsis - R) / 1000;
                string state = string.IsNullOrEmpty(why) ? (ok ? "complete" : "stopped by hand") : why;
                double dv_now = vessel.VesselDeltaV?.TotalDeltaVVac ?? double.NaN;
                // right after staging the game's figure can be garbage (314,084 m/s on 2026-11-13)
                if (dv_now > 20000) dv_now = double.NaN;
                logger.LogInfo($"[Next] RESULT {(ok ? "OK" : "NOT OK")}: {state} | T+{now - launch_ut:n0} s, " +
                               $"Ap {ap:n1} Pe {pe:n1} km, inc {orbit.inclination:n2}°, ecc {orbit.eccentricity:n4}, " +
                               $"mass {launch_mass:n2} -> {vessel.totalMass:n2} t, Δv left (game) {(double.IsNaN(dv_now) ? "n/a" : dv_now.ToString("n0"))} m/s, " +
                               $"heading {heading_deg:n1}°, max throttle {ctx.maxThrottle:n2}, max Q {ctx.maxQKPa:n0}, weights {ctx.weights}");
                // calibration: refit the model factors so the flown plan re-simulates to this flight
                if (model != null && model.valid && have_cut && cut_obs.hasPath &&
                    (arbiter.behaviour == Behaviour.CutOff || arbiter.behaviour == Behaviour.BurntOut))
                {
                    var fit = Calibrator.Run(model, ctx, heading_deg, settings.destination_Ap_km.V, calibration, flight_samples,
                                             cut_obs.ut - launch_ut, cut_obs.ut - launch_ut, cut_obs.airSpeed, cut_obs.pathDeg);
                    logger.LogInfo("[Next] CALIB " + fit.Summary());
                    if (fit.ok) { rec.calRmsBefore = fit.rmsBefore; rec.calRmsAfter = fit.rmsAfter; }
                    if (fit.ok) CraftMemory.SaveCalibration(model, vessel.Name, fit.after);
                }
                if (check != null && model != null && model.valid)
                {
                    double cap = CapKm();
                    // stage 1's margin: Δv it still had when its burn reached the cap
                    double unused = have_cut && s1BurnDv >= 0 ? Math.Max(0, Stage1LeftDv(model, cut_obs.mass) - K2D2.Lift.Stage1Burn.ReserveDv - s1BurnDv) : 0;
                    double real = BehaviourTrainer.RealScore(ok, sepPe, cap, drift_peak_deg, max_q_so_far, ctx, unused);
                    rec.realScore = real;
                    rec.capKm = cap;
                    if (!double.IsNaN(check.debrisPeKm)) rec.plannedPeKm = check.debrisPeKm;
                    // the burn's real loss: what was burned against an instant burn from the cut-off orbit
                    // to the orbit stage 1 was dropped on
                    if (!double.IsNaN(sepPe) && have_cut && s1BurnDv > 0 && cut_obs.peR > 0)
                    {
                        double VAp(double apR, double peR) => Math.Sqrt(Math.Max(0, body.mu * (2 / apR - 2 / (apR + peR))));
                        double instant = VAp(body.radius + sepAp * 1000, body.radius + sepPe * 1000) - VAp(cut_obs.apR, cut_obs.peR);
                        double learned = CraftMemory.LearnS1BurnLoss(model, vessel.Name, s1BurnDv - instant);
                        logger.LogInfo($"[Train] S1 BURN planned {s1BurnDv:n0} m/s, instant-burn equivalent {instant:n0} m/s - loss {s1BurnDv - instant:n0} -> learned {learned:n0} m/s");
                    }
                    logger.LogInfo($"[Train] REAL score {real:n1} (sim predicted {predicted_score:n1}): Pe at separation " +
                                   (double.IsNaN(sepPe) ? "-" : $"{sepPe:n1} km") + $", peak drift {drift_peak_deg:n1}°, max Q {max_q_so_far:n1} kPa{(ok ? "" : ", FAILED")}");
                    CraftMemory.RecordRealScore(model, vessel.Name, real);
                }
                if (!double.IsNaN(sepPe))
                    logger.LogInfo($"[Next] SEPARATION stage 1 dropped at Ap {sepAp:n1} Pe {sepPe:n1} km - planned Pe " +
                                   (check != null && !double.IsNaN(check.debrisPeKm) ? $"{check.debrisPeKm:n0} km ({sepPe - check.debrisPeKm:+0;-0} km)" : "-") +
                                   $" (target Ap {settings.destination_Ap_km.V:n0} km, debris cap {CapKm():n0} km)");
                if (model != null && model.valid)
                    CraftMemory.Learn(model, vessel.Name, learn_onset_mach, learn_onset_q, learn_onset_thrust, learn_onset_peak,
                                      state.Contains("Lost control"), loc_mach, loc_q, loc_thrust, loc_alt / 1000, max_mach, ok);
                logger.LogInfo($"[Next] TRACKING max nose error {drift_peak_deg:n1}° at {drift_peak_alt / 1000:n1} km (q {drift_peak_q:n1} kPa, " +
                               $"Mach {drift_peak_mach:n2}); {drift_episodes} drift episode(s), last onset " +
                               (double.IsNaN(drift_onset_q) ? "none" : $"T+{drift_onset_t:n1} at {drift_onset_alt / 1000:n1} km, q {drift_onset_q:n1} kPa, Mach {drift_onset_mach:n2}") +
                               $"; {(stability == null ? "stability not checked" : stability.Summary())}");

                if (have_cut)
                {
                    var c = cut_obs;
                    double s1_left = Stage1LeftDv(model, c.mass);
                    double upper = model?.UpperDeltaVVac ?? double.NaN;
                    bool s1_burned = s1BurnDv >= 0;
                    // with the stage-1 apoapsis burn the upper stage pays only the final circularisation
                    double spare = s1_burned ? upper - circDv : s1_left + upper - circDv;
                    bool p = check != null && check.verdict != LaunchCheck.Verdict.Unknown;
                    double N = double.NaN;
                    rec.spareDv = spare;
                    if (p) rec.plannedSpareDv = check.spareDv;
                    logger.LogInfo("[Next] PREDICTED -> ACTUAL at cut-off: " +
                                   $"T+ {Delta(p ? check.cutT : N, c.ut - launch_ut, " s")}, " +
                                   $"alt {Delta(p ? check.altKm : N, c.altitude / 1000, " km", "n1")}, " +
                                   $"Ap {Delta(p ? check.apKm : N, c.ApKm(body.radius), " km", "n1")}, " +
                                   $"Pe {Delta(p ? check.cutPeKm : N, c.PeKm(body.radius), " km")}, " +
                                   $"v {Delta(p ? check.cutSpeed : N, c.airSpeed, " m/s")}, " +
                                   $"path {Delta(p ? check.cutPathDeg : N, c.pathDeg, "°", "n1")}, " +
                                   $"mass {Delta(p ? check.cutMass : N, c.mass, " t", "n2")}, " +
                                   $"max Q {Delta(p ? check.maxQKPa : N, max_q_so_far, " kPa", "n1")}, " +
                                   $"S1 left {Delta(p ? check.stage1LeftDv : N, s1_left, " m/s")}, " +
                                   (s1_burned
                                       ? $"stage-1 Ap burn {Delta(p ? check.s1ApBurnDv : N, s1BurnDv, " m/s")}, " +
                                         $"upper circ {Delta(p ? check.upperCircDv : N, circDv, " m/s")}, "
                                       : $"circ {Delta(p ? check.circDv : N, circDv, " m/s")}, ") +
                                   $"spare {Delta(p ? check.spareDv : N, spare, " m/s")} " +
                                   (s1_burned ? $"(actual = upper {upper:n0} - upper circ)" : $"(actual = S1 left + upper {upper:n0} - circ)"));
                }
                else
                    logger.LogInfo("[Next] PREDICTED -> ACTUAL: the run stopped before cut-off");
                // pad launches only (check is null for a mid-air start)
                if (check != null && model != null && model.valid)
                    CraftMemory.RecordFlight(model, vessel.Name, rec);

                foreach (var v in AllVessels())
                {
                    if (v?.SimulationObject == null || v == vessel || !v.IsDebris()) continue;
                    if (known_vessels.Contains(v.SimulationObject.GlobalId.ToString())) continue;
                    var o = v.Orbit;
                    if (o == null || v.mainBody != cb) continue;
                    double dpe = (o.Periapsis - R) / 1000, dap = (o.Apoapsis - R) / 1000;
                    bool down = v.Landed || (cb.hasAtmosphere ? dpe < cb.atmosphereDepth / 1000 : dpe < 0);
                    logger.LogInfo($"[Next] DEBRIS '{v.Name}': Ap {dap:n1} Pe {dpe:n1} km, mass {v.totalMass:n2} t - " +
                                   (down ? "comes down" : "STAYS IN ORBIT"));
                    DebrisTracker.Register(v);
                }
            }
            catch (System.Exception e)
            {
                logger.LogWarning("[Next] flight summary failed: " + e.Message);
            }
        }

        // ---------------------------------------------------------------- sensing

        AscentObservation Sample(VesselComponent vessel, double now)
        {
            var o = new AscentObservation { ut = now };
            var orbit = vessel.Orbit;
            var vehicle = current_vessel.VesselVehicle;
            if (orbit == null || vehicle == null) return o;

            o.landed = vessel.Landed;
            o.altitude = vessel.AltitudeFromSeaLevel;
            o.mass = vessel.totalMass;
            o.throttle = current_vessel.GetThrottle();

            Vector3d r = orbit.GetRelativePositionAtUTZup(now);
            Vector3d v = orbit.GetOrbitalVelocityAtUTZup(now);
            o.r = r.magnitude;
            o.vr = Vector3d.Dot(v, r) / o.r;
            o.vt = System.Math.Sqrt(System.Math.Max(0, v.sqrMagnitude - o.vr * o.vr));
            K2D2.Lift.Stage1.Stage1Guidance.ApPe(body.mu, o.r, o.vr, o.vt, out o.apR, out o.peR);

            // surface-relative: speed, path angles, dynamic pressure, Mach, heating
            var up_coords = vehicle.Up.coordinateSystem;
            Vector3d sv = Vector.Reframed(vehicle.SurfaceVelocity, up_coords).vector;
            Vector3d up = vehicle.Up.vector.normalized;
            o.airSpeed = sv.magnitude;
            o.airPathDeg = o.airSpeed > 0.1
                ? System.Math.Asin(System.Math.Max(-1, System.Math.Min(1, Vector3d.Dot(sv, up) / o.airSpeed))) * 180 / System.Math.PI
                : 90;
            o.pathDeg = o.airPathDeg;
            o.hasPath = o.airSpeed >= 10;
            o.horizontalSpeed = (sv - up * Vector3d.Dot(sv, up)).magnitude;
            var telemetry = SASTool.getTelemetry();
            if (o.hasPath && telemetry != null)
            {
                // signed in the launch plane, the same frame and Euler convention ApplyLatched steers with
                var frame = telemetry.HorizonUp.coordinateSystem;
                Vector3d svp = Vector.Reframed(vehicle.SurfaceVelocity, frame).vector;
                Vector3d along = QuaternionD.Euler(0, heading_deg, 0) * Vector3d.forward;
                Vector3d plane_up = QuaternionD.Euler(-90, heading_deg, 0) * Vector3d.forward;
                o.pathDeg = System.Math.Atan2(Vector3d.Dot(svp, plane_up), Vector3d.Dot(svp, along)) * 180 / System.Math.PI;

                // sideways: nose and velocity against the launch plane
                Vector3d side = QuaternionD.Euler(0, heading_deg + 90, 0) * Vector3d.forward;
                Rotation rot_h = Rotation.Reframed(current_vessel.GetRotation(), frame);
                Vector3d nose_h = (rot_h.localRotation * Vector3.up).normalized;
                double vs = Vector3d.Dot(svp, side), va = Vector3d.Dot(svp, along), vu = Vector3d.Dot(svp, plane_up);
                double ns = Vector3d.Dot(nose_h, side), na = Vector3d.Dot(nose_h, along), nu = Vector3d.Dot(nose_h, plane_up);
                const double r2d = 180 / System.Math.PI;
                o.sideslipDeg = (System.Math.Atan2(ns, System.Math.Sqrt(na * na + nu * nu)) -
                                 System.Math.Atan2(vs, System.Math.Sqrt(va * va + vu * vu))) * r2d;
                o.trackOffDeg = System.Math.Abs(va) + System.Math.Abs(vs) > 1 ? System.Math.Atan2(vs, va) * r2d : 0;
                o.noseHdgOffDeg = System.Math.Abs(na) + System.Math.Abs(ns) > 0.05 ? System.Math.Atan2(ns, na) * r2d : 0;
            }

            var cb = vessel.mainBody;
            if (cb.hasAtmosphere && o.altitude < cb.atmosphereDepth)
            {
                double p = cb.GetPressure(o.altitude);
                double d = cb.GetDensity(p, cb.GetTemperature(o.altitude));
                o.qKPa = 0.0005 * d * o.airSpeed * o.airSpeed;
                double a = d > 0 ? AeroUtilities.GetSoundSpeed(cb.atmosphereAdiabaticIndex, p, d) : 0;
                o.mach = a > 0 ? o.airSpeed / a : 0;
                o.heat = System.Math.Sqrt(d) * o.airSpeed * o.airSpeed * o.airSpeed;
            }
            if (o.qKPa > max_q_so_far) max_q_so_far = o.qKPa;
            o.maxQSoFar = max_q_so_far;

            // nose elevation above the horizon
            Rotation rot = Rotation.Reframed(current_vessel.GetRotation(), up_coords);
            Vector3d nose = (rot.localRotation * Vector3.up).normalized;
            o.noseDeg = System.Math.Asin(System.Math.Max(-1, System.Math.Min(1, Vector3d.Dot(nose, up)))) * 180 / System.Math.PI;
            try
            {
                Vector3d w = Vector.Reframed(current_vessel.GetAngularSpeed(), up_coords).vector;
                o.rollRateDegS = Vector3d.Dot(w, nose) * 180 / System.Math.PI;
            }
            catch (System.Exception) { }
            o.ctlPitch = current_vessel.Pitch;
            o.ctlYaw = current_vessel.Yaw;
            o.ctlRoll = current_vessel.Roll;

            o.enginesLit = EnginesLit(vessel, out o.thrustKN);
            if (o.throttle > 0.1 && o.thrustKN > 0)
            {
                double now_full = o.thrustKN / o.throttle;
                full_thrust_est = full_thrust_est <= 0 ? now_full : full_thrust_est + 0.2 * (now_full - full_thrust_est);
            }
            o.fullThrustKN = full_thrust_est;
            o.valid = !double.IsNaN(o.r) && o.r > 0;
            return o;
        }

        // Same sources as Ascent.EnginesBurning: the Δv engine list, then the parts themselves, so
        // "nothing lit" (which stages once on the pad) never comes from a stale or partial list.
        static bool EnginesLit(VesselComponent vessel, out double thrust_kN)
        {
            thrust_kN = 0;
            bool lit = false;
            var engines = vessel.VesselDeltaV?.EngineInfo;
            if (engines != null)
                foreach (var info in engines)
                {
                    var engine = info?.Engine;
                    if (engine == null || !engine.EngineIgnited) continue;
                    lit = true;
                    thrust_kN += engine.FinalThrustValue;
                }
            if (!lit)
            {
                var parts = vessel.SimulationObject?.PartOwner?.Parts;
                if (parts != null)
                    foreach (var p in parts)
                        if (p.TryGetModule<PartComponentModule_Engine>(out var module) && module.EngineIgnited)
                            return true;
            }
            return lit;
        }

        // Steer back onto the launch heading when the ground track drifts (same law as Ascent's
        // computeSpeedHeading, ±45° max). Raw value - SafetyShield.LimitHeading decides how much of
        // it is flown. Computed on the tick, applied every frame.
        float HeadingCorrection()
        {
            var vehicle = current_vessel.VesselVehicle;
            if (vehicle == null) return 0;
            var up_coords = vehicle.Up.coordinateSystem;
            var sv = Vector.Reframed(vehicle.SurfaceVelocity, up_coords).vector;
            var north = Vector.Reframed(vehicle.North, up_coords).vector;
            var up = vehicle.Up.vector;
            var h = sv - up.normalized * Vector3d.Dot(sv, up);
            if (h.magnitude < 1) return 0;
            float track = (float)-Vector3d.SignedAngle(h.normalized, north, up);
            return Mathf.Clamp(GeneralTools.diffAngle(heading_deg, track), -45, 45);
        }

        // ---------------------------------------------------------------- actuation

        void ApplyLatched()
        {
            var autopilot = current_vessel.Autopilot;
            var telemetry = SASTool.getTelemetry();
            if (autopilot != null && telemetry != null)
            {
                autopilot.Enabled = true;
                var up = telemetry.HorizonUp;
                Vector3d direction = QuaternionD.Euler(-latched.elevationDeg, heading_deg + heading_correction, 0) * Vector3d.forward;
                autopilot.SAS.lockedMode = false;
                autopilot.SAS.SetTargetOrientation(new Vector(up.coordinateSystem, direction), false);
            }
            current_vessel.SetThrottle(latched.throttle);
        }

        // ---------------------------------------------------------------- AscentBase

        public override void computeValues(bool compute_delta_ap_per_second)
        {
            var vessel = current_vessel?.VesselComponent;
            if (vessel == null) return;

            PatchedConicsOrbit orbit = vessel.Orbit;
            ap_km = (float)(orbit.Apoapsis - orbit.referenceBody.radius) / 1000;
            current_altitude_km = (float)(current_vessel.GetSeaAltitude() / 1000);

            // Ap gained per second at full throttle - Adjust uses it to size its final trim burn
            if (last_ap_km == 0) last_ap_km = ap_km;
            else
            {
                float delta_ap = ap_km - last_ap_km;
                last_ap_km = ap_km;
                if (compute_delta_ap_per_second)
                {
                    float throttle = (float)current_vessel.GetThrottle();
                    if (throttle > 0.1f && Time.deltaTime != 0)
                        delta_ap_per_second = Mathf.Lerp(delta_ap_per_second, delta_ap / (Time.deltaTime * throttle), 0.1f);
                    else
                        delta_ap_per_second = 0;
                }
            }
        }

        // ---------------------------------------------------------------- telemetry and UI

        void Log(bool force)
        {
            if (!force && obs.ut < next_log_ut) return;
            next_log_ut = obs.ut + LogPeriodS;
            var rep = shield.report;
            logger.LogInfo($"[Next] T+{(launch_ut < 0 ? 0 : obs.ut - launch_ut):n1} {arbiter.behaviour} ({policy.Name}) " +
                           $"alt {obs.altitude / 1000:n2} km, v {obs.airSpeed:n0} m/s, M {obs.mach:n2}, q {obs.qKPa:n1} kPa, " +
                           $"heat {obs.heat / K2D2.Lift.Stage1.Stage1Sim.HeatLimit * 100:n0}%, " +
                           $"path {obs.pathDeg:n1}, nose {obs.noseDeg:n1}, policy {policy_cmd.elevationDeg:n1}°/{policy_cmd.throttle:n2}, " +
                           $"out {latched.elevationDeg:n1}°/{latched.throttle:n2}, hdg+{heading_correction:n1}, " +
                           $"Ap {obs.ApKm(body.radius):n1} Pe {obs.PeKm(body.radius):n1} km, mass {obs.mass:n2} t, " +
                           $"thrust {obs.thrustKN:n0} kN, shield {rep.Flags()}, tick {ticks} | " +
                           $"{behaviour_policy.WeightsText()}, accel {obs.speedRate:n1} m/s² | " +
                           $"slip {obs.sideslipDeg:+0.0;-0.0}°, track {obs.trackOffDeg:+0.0;-0.0}°, nose hdg {obs.noseHdgOffDeg:+0.0;-0.0}°, " +
                           $"roll {obs.rollRateDegS:+0.0;-0.0}°/s, ctl p {obs.ctlPitch:+0.00;-0.00} y {obs.ctlYaw:+0.00;-0.00} r {obs.ctlRoll:+0.00;-0.00}");
        }

        public override void updateUI(VisualElement root_el, FullStatus st)
        {
            st.Console($"Lift: {status}");
        }

        public override void UpdateInfoRows(System.Action<string, string> addRow)
        {
            addRow("Behaviour", $"{arbiter.behaviour}");
            addRow("Launch check", check == null ? "-" : check.Short());
            addRow("Weights", behaviour_policy.WeightsText());
            addRow("Policy", policy?.Name ?? "-");
            addRow("Altitude", $"{obs.altitude / 1000:n2} km");
            addRow("Apoapsis Alt.", $"{obs.ApKm(body.radius):n2} km");
            addRow("Periapsis Alt.", $"{obs.PeKm(body.radius):n2} km");
            addRow("Path / Nose", $"{obs.pathDeg:n1}° / {obs.noseDeg:n1}°");
            addRow("Command (policy)", $"{policy_cmd.elevationDeg:n1}° / {policy_cmd.throttle:n2}");
            addRow("Command (out)", $"{latched.elevationDeg:n1}° / {latched.throttle:n2}");
            addRow("q / Mach", $"{obs.qKPa:n1} kPa / {obs.mach:n2}");
            addRow("Heating", $"{obs.heat / K2D2.Lift.Stage1.Stage1Sim.HeatLimit * 100:n0}%");
            addRow("Shield", shield.report.Flags());
            addRow("Loop", measured_rate_hz > 0 ? $"{measured_rate_hz:n1} Hz, {ticks} ticks" : $"{ticks} ticks");
        }
    }
}
