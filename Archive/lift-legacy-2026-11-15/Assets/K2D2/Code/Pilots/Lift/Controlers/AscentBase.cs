using K2D2.Controller;

namespace K2D2.Lift
{
    /// <summary>
    /// What the rest of the Lift pilot needs from an ascent controller, whichever lift system is
    /// flying it (LiftSettings.mode): Adjust reads Ap and the Ap rate, LiftUI the altitude for the
    /// profile graph.
    /// </summary>
    public abstract class AscentBase : ExecuteController
    {
        public float current_altitude_km = 0;
        public float ap_km = 0;
        public float delta_ap_per_second;

        public abstract void computeValues(bool compute_delta_ap_per_second);
    }
}
