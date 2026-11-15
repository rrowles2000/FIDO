using System;

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

    /// <summary>Everything a policy or the shield may know besides the observation: the run's settings snapshot.</summary>
    public struct PolicyContext
    {
        public float maxThrottle;
        public double endAscentErrorKm;
        public double maxQKPa;      // scoring: training and the real-flight score are penalised above it
        public double qAlphaKPaDeg; // shield: heading correction kept within this / q of the airflow
        public bool fragile;        // the design has drifted before: tighter heading-correction limit (CraftMemory)
        public bool s1ApBurn;       // stage 1's leftover is burned at apoapsis (Stage1Burn) before the upper stage
        public float s1CapPeKm;     // ... up to this periapsis (before the atmosphere clamp)
        public BehaviourWeights weights; // the behaviour model's weights (Behaviours.cs)

        // Removed 2026-11-15 with the pre-behaviour-model path (Archive/lift-legacy-2026-11-15/):
        // the Lift profile altitudes, the pitch kick, throttle holds, pull-down, the learned max-Q
        // hold, the thrust/q floor, the q cap, and the `blend` switch (the behaviour model is the
        // only policy now).

        public static PolicyContext From(LiftSettings s) => new PolicyContext
        {
            maxThrottle = s.next_max_throttle.V,
            endAscentErrorKm = s.end_ascent_error,
            maxQKPa = s.next_max_q_kpa.V,
            qAlphaKPaDeg = s.next_q_alpha.V,
            s1ApBurn = s.next_s1_ap_burn.V,
            s1CapPeKm = s.next_s1_pe_km.V,
            weights = BehaviourWeights.Default,
        };
    }
}
