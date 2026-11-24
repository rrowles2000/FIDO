using System;
using KSP.Sim.impl;
using KSP2FlightAssistant.MathLibrary;

namespace Fido.Constellation
{
    /// <summary>
    /// Closed-form two-body helpers for the Constellation tab and Lift's polar launch option.
    /// Everything works from radii/periods (no engine orbit objects), so it can be evaluated every
    /// frame for the UI estimates as well as when the actual nodes are created.
    /// </summary>
    public static class OrbitMath
    {
        public static double CircularSpeed(double mu, double r) => Math.Sqrt(mu / r);

        public static double Period(double mu, double sma) => 2.0 * Math.PI * Math.Sqrt(sma * sma * sma / mu);

        public static double SmaFromPeriod(double mu, double period) =>
            Math.Pow(mu * period * period / (4.0 * Math.PI * Math.PI), 1.0 / 3.0);

        public static double SpeedAt(double mu, double r, double sma) => Math.Sqrt(mu * (2.0 / r - 1.0 / sma));

        /// <summary>Total dv of a two-burn Hohmann-style move from (Pe, Ap) to a circular orbit of radius target.</summary>
        public static double CircularizeToDv(double mu, double peR, double apR, double target)
        {
            // first burn at Pe (raise/lower the far side to target), or at Ap when target is below Pe
            double burnR = target <= peR ? apR : peR;
            double sma0 = (peR + apR) / 2.0;
            double smaT = (burnR + target) / 2.0;
            double dv1 = Math.Abs(SpeedAt(mu, burnR, smaT) - SpeedAt(mu, burnR, sma0));
            double dv2 = Math.Abs(CircularSpeed(mu, target) - SpeedAt(mu, target, smaT));
            return dv1 + dv2;
        }

        /// <summary>
        /// Phasing from a circular orbit of radius r: falling behind by sepDeg over n orbits needs a
        /// longer period (raise the far side); getting ahead by (360 - sepDeg) needs a shorter one.
        /// Returns the cheaper option's per-burn dv (signed, + = prograde), its period, and the
        /// phasing orbit's far apsis radius.
        /// </summary>
        public static double PhasingBurn(double mu, double r, double sepDeg, int n,
            out double phasingPeriod, out double farApsis)
        {
            double T = Period(mu, r);
            double vc = CircularSpeed(mu, r);

            double behindPeriod = T * (1.0 + sepDeg / (360.0 * n));
            double behindSma = SmaFromPeriod(mu, behindPeriod);
            double behindDv = SpeedAt(mu, r, behindSma) - vc;

            double aheadPeriod = T * (1.0 - (360.0 - sepDeg) / (360.0 * n));
            double aheadSma = aheadPeriod > 0 ? SmaFromPeriod(mu, aheadPeriod) : 0;
            double aheadDv = aheadPeriod > 0 ? SpeedAt(mu, r, aheadSma) - vc : double.MaxValue;

            // a too-short "ahead" period has no orbit through r at all (2a < r gives NaN speed)
            bool aheadValid = aheadSma > r / 2.0 && !double.IsNaN(aheadDv);
            if (!aheadValid || Math.Abs(behindDv) <= Math.Abs(aheadDv))
            {
                phasingPeriod = behindPeriod;
                farApsis = 2.0 * behindSma - r;
                return behindDv;
            }

            phasingPeriod = aheadPeriod;
            farApsis = 2.0 * aheadSma - r;
            return aheadDv;
        }

        /// <summary>
        /// Phasing burn constrained so the phasing orbit's low side stays above minR. Returns
        /// double.NaN when no option is valid for that many orbits.
        /// </summary>
        public static double SafePhasingBurn(double mu, double r, double sepDeg, int n, double minR,
            out double phasingPeriod, out double farApsis)
        {
            double dv = PhasingBurn(mu, r, sepDeg, n, out phasingPeriod, out farApsis);
            if (farApsis >= minR)
                return dv;

            // the cheaper "ahead" option dips too low - fall back to falling behind
            double T = Period(mu, r);
            phasingPeriod = T * (1.0 + sepDeg / (360.0 * n));
            double sma = SmaFromPeriod(mu, phasingPeriod);
            farApsis = 2.0 * sma - r;
            return SpeedAt(mu, r, sma) - CircularSpeed(mu, r);
        }

        /// <summary>dv of a pure plane change of deltaIncDeg on a circular orbit at speed v.</summary>
        public static double PlaneChangeDv(double v, double deltaIncDeg) =>
            2.0 * v * Math.Sin(Math.Abs(deltaIncDeg) * Math.PI / 360.0);

        /// <summary>
        /// Launch heading (compass degrees) for a target inclination, corrected for the body's
        /// surface rotation. Returns false when the inclination can't be reached directly from this
        /// latitude (|cos i| > cos latitude), in which case heading is the closest reachable one.
        /// </summary>
        public static bool LaunchHeading(double targetIncDeg, double latitudeDeg, double orbitSpeed,
            double surfaceSpeed, bool northward, out double heading)
        {
            double lat = latitudeDeg * Math.PI / 180.0;
            double inc = targetIncDeg * Math.PI / 180.0;

            double ratio = Math.Cos(inc) / Math.Max(Math.Cos(lat), 1e-6);
            bool reachable = Math.Abs(ratio) <= 1.0;
            ratio = Math.Max(-1.0, Math.Min(1.0, ratio));

            // inertial azimuth, measured from north toward east
            double beta = Math.Asin(ratio);
            if (!northward)
                beta = Math.PI - beta;

            // remove the velocity the surface already gives us
            double vx = orbitSpeed * Math.Sin(beta) - surfaceSpeed * Math.Cos(lat);
            double vy = orbitSpeed * Math.Cos(beta);
            heading = Math.Atan2(vx, vy) * 180.0 / Math.PI;
            if (heading < 0) heading += 360.0;
            return reachable;
        }

        /// <summary>
        /// The orbit inclination a launch heading (navball degrees) aims for, the inverse of
        /// LaunchHeading: the inclination whose launch heading from this latitude is closest to
        /// <paramref name="headingDeg"/>. North-going headings (cos > 0) search northward launches,
        /// the rest southward. 90° (east) gives the latitude, i.e. the lowest inclination reachable.
        /// </summary>
        public static double InclinationForHeading(double headingDeg, double latitudeDeg, double orbitSpeed, double surfaceSpeed)
        {
            bool northward = Math.Cos(headingDeg * Math.PI / 180.0) >= 0;
            double best = 0, best_err = double.MaxValue;
            for (double inc = 0; inc <= 180.0001; inc += 0.05)
            {
                LaunchHeading(inc, latitudeDeg, orbitSpeed, surfaceSpeed, northward, out double h);
                double err = Math.Abs(((h - headingDeg) % 360 + 540) % 360 - 180);
                if (err < best_err) { best_err = err; best = inc; }
            }
            return best;
        }

        /// <summary>Surface rotation speed at the equator (m/s), signed for inverse rotation.</summary>
        public static double EquatorialSurfaceSpeed(CelestialBodyComponent body)
        {
            if (body.rotationPeriod == 0) return 0;
            double v = 2.0 * Math.PI * body.radius / body.rotationPeriod;
            return body.hasInverseRotation ? -v : v;
        }

        /// <summary>
        /// Time (s from now) of the next equator crossing, and whether it is the ascending node
        /// (moving towards +Z). Bisects on the sign of r.z, same approach as
        /// LandingTargeting.TimeToNextApoapsis.
        /// </summary>
        public static double TimeToNextNode(Vector3d r0, Vector3d v0, double mu, double period, out bool ascending)
        {
            const int samples = 72;
            double step = period / samples;
            double prevDt = 0;
            double prevZ = r0.z;
            ascending = v0.z > 0;

            for (int i = 1; i <= samples + 1; i++)
            {
                double dt = i * step;
                KeplerPropagator.Propagate(r0, v0, mu, dt, out Vector3d r, out Vector3d v);
                if ((prevZ < 0 && r.z >= 0) || (prevZ > 0 && r.z <= 0))
                {
                    ascending = prevZ < 0;
                    double lo = prevDt, hi = dt;
                    for (int j = 0; j < 40; j++)
                    {
                        double mid = (lo + hi) / 2.0;
                        KeplerPropagator.Propagate(r0, v0, mu, mid, out Vector3d rm, out Vector3d vm);
                        if ((rm.z >= 0) == (prevZ >= 0)) lo = mid; else hi = mid;
                    }
                    return (lo + hi) / 2.0;
                }
                prevDt = dt;
                prevZ = r.z;
            }
            return -1;
        }

        public static double InclinationDeg(Vector3d r, Vector3d v)
        {
            Vector3d h = Vector3d.Cross(r, v);
            return Math.Acos(Math.Max(-1, Math.Min(1, h.z / h.magnitude))) * 180.0 / Math.PI;
        }
    }
}
