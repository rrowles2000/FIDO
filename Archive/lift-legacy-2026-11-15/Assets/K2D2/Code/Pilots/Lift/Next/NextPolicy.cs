using System;
using K2D2.Lift.Stage1;

namespace K2D2.Lift.Next
{
    /// <summary>
    /// Layer L2 of the New lift system (NN ascent report, Section 3.1): turns one observation into
    /// two setpoints. A policy is pure - it doesn't touch the vessel, the game or Unity - and holds
    /// no state between ticks, so the neural network can replace it later without changing anything
    /// around it, and the same policy runs in the simulator (LaunchCheck) and in flight.
    /// </summary>
    public interface IAscentPolicy
    {
        string Name { get; }
        AscentCommand Decide(in AscentObservation obs, in BodyDescriptor body, in PolicyContext ctx);
    }

    /// <summary>
    /// Marks a learned policy (the network, Phase 2). Only these fall back when the shield's AoA
    /// limit holds them off for long - the hand-written policies lean on that limit by design.
    /// </summary>
    public interface ILearnedPolicy : IAscentPolicy { }

    /// <summary>Everything a policy or the shield may know besides the observation: the run's settings snapshot.</summary>
    public struct PolicyContext
    {
        public double startKm, midKm, endKm; // Lift profile: 90° / 45° / ~0° altitudes
        public float maxThrottle;
        public double endAscentErrorKm;
        public double kickDeg;      // pitch kick off vertical (LaunchCheck picks it when on auto)
        public double kickSpeed;    // surface speed (m/s) the kick starts at; it ramps in over KickRampSpeed
        public double pullDownDeg;  // after max Q: nose this far below the flight path (the shield's AoA limit caps it)
        public double qHoldFraction; // > 0: learned max-Q hold point (share of max Q); 0 = the shield default
        public double holdTwr;      // > 0: thrust capped at this TWR ...
        public double holdKm;       // ... below this altitude (LaunchCheck picks both when the kick is on auto)
        public double maxQKPa;      // shield: max-Q throttle-back limit
        public double qAlphaKPaDeg; // shield: AoA limit = this / q
        public double minThrustPerQ; // > 0: learned steering-authority floor, kN of thrust per kPa of q (CraftMemory)
        public bool fragile;        // the design has drifted before: no throttle holds, tight AoA limit
        public double qCapKPa;      // > 0: q cap across a Mach band (no longer learned - it cut thrust, see CraftMemory) ...
        public double qCapMachLo, qCapMachHi; // ... across this Mach band
        public bool s1ApBurn;       // stage 1's leftover is burned at apoapsis (Stage1Burn) before the upper stage
        public float s1CapPeKm;     // ... up to this periapsis (before the atmosphere clamp)
        public bool blend;          // fly the behaviour model (Behaviours.cs): no kick/grid/holds, no AoA limit or max-Q throttle-back
        public BehaviourWeights weights; // its weights

        public static PolicyContext From(LiftSettings s) => new PolicyContext
        {
            startKm = s.start_altitude_km.V,
            midKm = s.mid_rotate_altitude_km,
            endKm = s.end_rotate_altitude_km,
            maxThrottle = s.next_max_throttle.V, // not max_throttle - see LiftSettings
            endAscentErrorKm = s.end_ascent_error,
            kickDeg = s.next_kick_deg.V,
            kickSpeed = s.next_kick_speed.V,
            maxQKPa = s.next_max_q_kpa.V,
            qAlphaKPaDeg = s.next_q_alpha.V,
            s1ApBurn = s.next_s1_ap_burn.V,
            s1CapPeKm = s.next_s1_pe_km.V,
            blend = s.next_behaviour_model.V,
            weights = BehaviourWeights.Default,
        };
    }

    /// <summary>
    /// Phase 0.1 policy: a pitch kick and a gravity turn. On 2026-11-13 the Phase 0 policy (the
    /// Lift profile steered onto the path) could not turn the rocket: the shield holds the nose
    /// within Q·AoA/q of the airflow, so from a vertical liftoff the profile's 45° asked for things
    /// the shield refused all the way up (wanted 19°, flew 59° at 33 km) and the 13 t test vessel
    /// reached Ap at 37 km still climbing at 60°, needing a 1,755 m/s circularisation.
    ///
    /// So the turn is started where the air allows it and then left to gravity:
    ///   below KickStartSpeed   straight up
    ///   KickStart..KickEnd     tip over to 90 - kickDeg (low q here, so the AoA limit is wide)
    ///   after                  hold that until the path has come down to it, then fly prograde
    ///                          (zero AoA); once past max-Q and in thin air, pull down toward the
    ///                          Lift profile as far as the shield allows
    /// Throttle: full (of max) until apoapsis nears the target, then easing off.
    /// The kick angle is what decides how lofted the trajectory is; LaunchCheck simulates this same
    /// policy to choose it per vehicle.
    /// </summary>
    public class GravityTurnPolicy : IAscentPolicy
    {
        // The kick starts at ctx.kickSpeed and ramps in over KickRampSpeed. Fixed at 20-50 m/s until
        // 2026-11-13; now LaunchCheck searches the start speed with the angle, since a low-TWR
        // vessel may want to be faster (less gravity-turn sag) before it leans over.
        public const double DefaultKickSpeed = 20, KickRampSpeed = 30;  // m/s, surface
        public const double PullDownStartKPa = 5, PullDownFullKPa = 1;

        public string Name => "GravityTurn";

        public AscentCommand Decide(in AscentObservation obs, in BodyDescriptor body, in PolicyContext ctx)
        {
            double kick_elev = 90 - ctx.kickDeg;
            double kick_start = ctx.kickSpeed > 0 ? ctx.kickSpeed : DefaultKickSpeed;
            double kick_end = kick_start + KickRampSpeed;
            double elev;
            if (obs.airSpeed < kick_start)
                elev = 90;
            else if (obs.airSpeed < kick_end)
                elev = 90 - ctx.kickDeg * (obs.airSpeed - kick_start) / (kick_end - kick_start);
            else
            {
                elev = obs.hasPath ? Math.Min(kick_elev, obs.pathDeg) : kick_elev;

                // Flatter climb (2026-11-14): once past max Q, fly the nose pullDownDeg below the
                // flight path, eased in as q falls to 70% of its peak and below. The shield's AoA
                // limit (Q·AoA / q) decides how much of it the air allows, so it bites more as the
                // air thins. LaunchCheck searches pullDownDeg per vessel - this replaces the old
                // pull toward the Optimiser's fixed Lift profile, and 0 is a pure gravity turn.
                bool past_max_q = obs.maxQSoFar > 1 && obs.qKPa < 0.85 * obs.maxQSoFar;
                if (past_max_q && ctx.pullDownDeg > 0 && obs.hasPath)
                {
                    double w = Math.Max(0, Math.Min(1, (0.85 * obs.maxQSoFar - obs.qKPa) / (0.15 * obs.maxQSoFar)));
                    elev = Math.Min(elev, obs.pathDeg - w * ctx.pullDownDeg);
                }
            }

            float throttle = ctx.maxThrottle * Stage1Guidance.ApThrottle(obs.apR, body.targetApR);
            throttle = Math.Min(throttle, HoldThrottle(obs, body, ctx));
            return new AscentCommand(elev, throttle);
        }

        /// <summary>
        /// The throttle-hold option LaunchCheck can pick: thrust capped at ctx.holdTwr while below
        /// ctx.holdKm, so a powerful vessel doesn't spend its Δv pushing through thick air.
        /// </summary>
        public static float HoldThrottle(in AscentObservation obs, in BodyDescriptor body, in PolicyContext ctx)
        {
            if (ctx.holdTwr <= 0 || obs.altitude >= ctx.holdKm * 1000 || obs.fullThrustKN <= 0) return 1f;
            double g = body.mu / (obs.r * obs.r);
            return (float)Math.Max(0.05, Math.Min(1, ctx.holdTwr * obs.mass * g / obs.fullThrustKN));
        }
    }

    /// <summary>
    /// Phase 0's policy, kept as the fallback: the Optimiser's own guidance (Lift profile steered
    /// onto the flight path). Fine where the shield doesn't fight it; see GravityTurnPolicy for why
    /// it isn't the primary any more.
    /// </summary>
    public class GuidancePolicy : IAscentPolicy
    {
        public string Name => "Guidance";

        public AscentCommand Decide(in AscentObservation obs, in BodyDescriptor body, in PolicyContext ctx)
        {
            double elev = Stage1Guidance.ProfileElevationDeg(obs.altitude / 1000, ctx.startKm, ctx.midKm, ctx.endKm);
            if (obs.hasPath && obs.airSpeed > 10)
                elev = Stage1Guidance.FollowPath(elev, obs.pathDeg, obs.airSpeed);

            float throttle = ctx.maxThrottle * Stage1Guidance.ApThrottle(obs.apR, body.targetApR);
            return new AscentCommand(elev, throttle);
        }
    }
}
