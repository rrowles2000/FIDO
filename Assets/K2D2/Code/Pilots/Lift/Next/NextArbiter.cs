namespace K2D2.Lift.Next
{
    /// <summary>The behaviours of NN ascent report Section 5.1 that Phase 0 implements.</summary>
    public enum Behaviour
    {
        Ignite,     // on the pad, engines not lit yet
        ClearTower, // straight up until clear of the pad
        Fly,        // the policy (the behaviour model, later the network)
        CutOff,     // apoapsis at the target - engines off, hand over to Coasting
        BurntOut,   // stage 1 gave out before the target apoapsis
        Abort,      // stop the run
    }

    /// <summary>What the arbiter wants the controller to do this tick besides steering.</summary>
    public enum ArbiterAction
    {
        None,
        Stage,    // activate the next stage once (pad ignition)
        HandOff,  // ascent done - move on to Coasting
        EndRun,   // stop the Lift pilot with end_status
    }

    /// <summary>
    /// Layer L3 of the New lift system (NN ascent report, Sections 3.1 and 5.2): picks the active
    /// behaviour each tick and produces the command the shield will receive. Highest priority
    /// first: Abort > BurntOut > CutOff > Fly > ClearTower > Ignite. Abort covers the pad-ignition
    /// timeout and LossOfControlAbortS of continuous loss of control.
    ///
    /// Behaviours only ever move forward in Phase 0 (Ignite -> ClearTower -> Fly -> CutOff), so
    /// each transition is latched instead of re-evaluated - nothing can chatter back and forth at
    /// a boundary. Hysteresis and dwell times arrive with the first behaviour that can be left and
    /// re-entered (Protect, Phase 2).
    /// </summary>
    public class Arbiter
    {
        public const double TowerClearanceM = 100;  // straight up this far above the launch altitude
        public const double IgnitionTimeoutS = 3;   // staged for ignition and still nothing lit
        public const double BurnInS = 5;            // engines must have given thrust this long first
        public const double DryConfirmS = 2;        // no thrust at a commanded throttle this long = dry
        public const double LossOfControlAbortS = 3; // nose off the airflow this long = stop the run

        public Behaviour behaviour = Behaviour.Ignite;
        public string end_status = "";
        public string status = "";

        double launch_altitude = double.NaN;
        double ignition_ut = -1;
        double first_thrust_ut = -1;
        double dry_since = -1;
        bool loss_of_control_logged;
        double loc_since = -1;

        public System.Action<string> log;

        public void Reset()
        {
            behaviour = Behaviour.Ignite;
            end_status = "";
            status = "";
            launch_altitude = double.NaN;
            ignition_ut = -1;
            first_thrust_ut = -1;
            dry_since = -1;
            loss_of_control_logged = false;
            loc_since = -1;
        }

        /// <param name="policyCmd">what the policy asked for this tick (only used while flying)</param>
        /// <param name="lossOfControl">the shield's loss-of-control flag from the previous tick</param>
        public ArbiterAction Decide(in AscentObservation obs, in BodyDescriptor body, in PolicyContext ctx,
                                    AscentCommand policyCmd, bool lossOfControl, bool autoStaging,
                                    out AscentCommand cmd)
        {
            cmd = policyCmd;
            if (double.IsNaN(launch_altitude)) launch_altitude = obs.altitude;
            if (obs.enginesLit && obs.thrustKN > 0.5 && first_thrust_ut < 0) first_thrust_ut = obs.ut;

            // Ignite: on the pad with nothing lit - stage once, give up if that didn't light anything
            if (behaviour == Behaviour.Ignite)
            {
                cmd = new AscentCommand(90, ctx.maxThrottle);
                if (obs.enginesLit || !obs.landed)
                {
                    Enter(Behaviour.ClearTower, obs, "engines lit");
                }
                else if (ignition_ut < 0)
                {
                    ignition_ut = obs.ut;
                    status = "Ignition";
                    log?.Invoke("[Next] no engine lit on the pad - staging once for ignition");
                    return ArbiterAction.Stage;
                }
                else if (obs.ut - ignition_ut > IgnitionTimeoutS)
                {
                    return End(Behaviour.Abort, "No engine lit - stage the launch engines by hand", obs);
                }
                else
                    return ArbiterAction.None;
            }

            // CutOff (latched): apoapsis reached - engines off, hand over
            if (behaviour == Behaviour.CutOff ||
                obs.ApKm(body.radius) >= (body.targetApR - body.radius) / 1000 - ctx.endAscentErrorKm)
            {
                cmd = new AscentCommand(obs.hasPath ? obs.pathDeg : policyCmd.elevationDeg, 0);
                if (behaviour != Behaviour.CutOff)
                    Enter(Behaviour.CutOff, obs, $"Ap {obs.ApKm(body.radius):n1} km reached");
                status = "Ap reached - hand-off";
                return ArbiterAction.HandOff;
            }

            // BurntOut: thrust gone at a commanded throttle after a proper burn-in
            bool burnt_in = first_thrust_ut >= 0 && obs.ut - first_thrust_ut > BurnInS && !obs.landed;
            bool no_thrust = obs.throttle > 0.05 && obs.thrustKN < 0.5;
            dry_since = burnt_in && no_thrust ? (dry_since < 0 ? obs.ut : dry_since) : -1;
            if (dry_since >= 0 && obs.ut - dry_since > DryConfirmS && !autoStaging)
            {
                // above the atmosphere the coast and circularise can still finish the job
                bool ap_clear = !body.hasAtmosphere || obs.apR > body.radius + body.atmosphereDepth;
                if (ap_clear)
                {
                    Enter(Behaviour.BurntOut, obs, "stage 1 dry with Ap above the atmosphere - handing over");
                    status = "Stage 1 dry - hand-off";
                    cmd = new AscentCommand(obs.hasPath ? obs.pathDeg : 90, 0);
                    return ArbiterAction.HandOff;
                }
                return End(Behaviour.BurntOut,
                    $"Stage 1 burnt out short: Ap {obs.ApKm(body.radius):n1} km of {(body.targetApR - body.radius) / 1000:n0} km", obs);
            }

            // Loss of control: give the gimbal LossOfControlAbortS to pull it back (cutting thrust
            // straight away would lose that authority), then stop the run and hand the vessel back.
            // 2026-11-13 run 1 kept burning through ~30 s of tumbling before this existed.
            if (lossOfControl)
            {
                if (loc_since < 0) loc_since = obs.ut;
                if (!loss_of_control_logged)
                {
                    loss_of_control_logged = true;
                    log?.Invoke($"[Next] loss of control at {obs.altitude / 1000:n1} km - nose {obs.noseDeg:n1}°, " +
                                $"path {obs.airPathDeg:n1}°, q {obs.qKPa:n1} kPa");
                }
                if (obs.ut - loc_since > LossOfControlAbortS)
                    return End(Behaviour.Abort,
                        $"Lost control at {obs.altitude / 1000:n1} km (nose {obs.noseDeg:n0}° vs path {obs.airPathDeg:n0}° for {LossOfControlAbortS:n0} s) - stopped, you have control", obs);
            }
            else
                loc_since = -1;

            // ClearTower: straight up at the policy's throttle until clear of the pad
            if (behaviour == Behaviour.ClearTower)
            {
                if (obs.altitude - launch_altitude < TowerClearanceM)
                {
                    cmd = new AscentCommand(90, ctx.maxThrottle);
                    status = "Clearing the tower";
                    return ArbiterAction.None;
                }
                Enter(Behaviour.Fly, obs, "tower clear");
            }

            // Fly: the policy
            status = lossOfControl ? "Loss of control" : "Flying";
            return ArbiterAction.None;
        }

        void Enter(Behaviour b, in AscentObservation obs, string why)
        {
            log?.Invoke($"[Next] {behaviour} -> {b} at {obs.altitude / 1000:n2} km: {why}");
            behaviour = b;
        }

        ArbiterAction End(Behaviour b, string why, in AscentObservation obs)
        {
            Enter(b, obs, why);
            end_status = why;
            status = why;
            return ArbiterAction.EndRun;
        }
    }
}
