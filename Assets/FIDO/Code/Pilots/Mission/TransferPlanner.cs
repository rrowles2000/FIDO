using System;
using System.Collections.Generic;
using System.Linq;
using Fido.Constellation;
using KSP2FlightAssistant.MathLibrary;

namespace Fido.Mission
{
    /// <summary>
    /// Transfer and course-correction burns from a parking orbit to one of the parent body's moons
    /// (Mun, Minmus...). Plain two-body maths on state vectors, written for FIDO from the textbook
    /// method (Hohmann transfer, phase-angle window, impact parameter for the arrival periapsis) -
    /// not ported from Flight Plan/MechJeb, which are GPLv3 and can't go into this CC BY-SA project.
    ///
    /// Departure (Compute):
    /// 1. Window: for burn times across one synodic period, a prograde Hohmann burn's apoapsis
    ///    (opposite the burn point) is compared with where the moon will be after the half-orbit
    ///    coast; the time with the smallest angle between them is the window.
    /// 2. In-plane aim: burn time and prograde Δv are adjusted (Nelder-Mead) so the vessel's Kepler
    ///    path passes the moon at the impact parameter b that bends into the wanted periapsis,
    ///    b = rp * sqrt(1 + 2 mu_moon / (rp v_inf^2)), as far as that can be done in the orbit's plane.
    /// 3. A moon off the vessel's plane (Minmus, 6°) is only reached by this burn if it arrives near
    ///    its node. Otherwise a normal component is tried, but only kept if it's cheap; if not, the
    ///    plan says a course correction is needed - fixing the plane mid-way costs tens of m/s
    ///    where doing it at departure costs hundreds (an offline sweep put it at 300-1,700 m/s).
    /// Course correction (Correct): a small burn (prograde, normal, radial) on the way out that moves
    /// the closest approach onto b. Earlier is cheaper; the cost term picks the time.
    ///
    /// Arrival plane: the miss is aimed as a vector, not just a distance. The orbit plane around the
    /// moon is the plane of the miss and the arrival velocity (h = B x v_inf), so an equatorial
    /// orbit (prograde) wants B = b * (s x z) and a polar one B = +/-b * (z - (s.z) s), s the arrival
    /// direction and z the rotation axis (Zup z; KSP bodies have no axial tilt). Polar needs the
    /// miss moved out of the departure plane, which a departure burn can't do cheaply (a normal
    /// burn's plane change crosses back over at the far side) - the course correction does it.
    ///
    /// The moon's gravity isn't modelled before closest approach, so the game's own patched-conic
    /// prediction is checked after each node is placed (TransferRunner).
    /// </summary>
    public static class TransferPlanner
    {
        public enum ArrivalPlane { Any, Equatorial, Polar }

        public const string PlaneEquatorial = "Equatorial (0°)", PlanePolar = "Polar (90°)", PlaneAny = "Any (cheapest)";
        public static List<string> PlaneChoices() => new() { PlaneEquatorial, PlanePolar, PlaneAny };

        public static ArrivalPlane ParsePlane(string s) =>
            s != null && s.StartsWith("Polar") ? ArrivalPlane.Polar
            : s != null && s.StartsWith("Equatorial") ? ArrivalPlane.Equatorial
            : ArrivalPlane.Any;

        public static string PlaneName(ArrivalPlane p) =>
            p == ArrivalPlane.Polar ? "polar" : p == ArrivalPlane.Equatorial ? "equatorial" : "any-plane";

        // arrival inclination within this of the wanted plane counts as hit
        public const double PlaneToleranceDeg = 3;

        public class Plan
        {
            public bool ok;            // a burn worth placing (it may still need a correction)
            public string problem = "";
            public bool encounter;     // the planned path passes the moon at about the wanted periapsis
            public double UT;          // burn time
            public double prograde, normal, radial;
            public double tof;         // burn to closest approach, s
            public double miss_m;      // planned closest approach to the moon's centre
            public double aim_m;       // the impact parameter aimed for
            public double off_plane_m; // closest approach out of the vessel's orbit plane
            public double pe_alt_m;    // periapsis altitude the miss bends into (estimate)
            public double v_inf;       // speed relative to the moon on arrival
            public double capture_dv;  // circularise at the wanted periapsis
            public double correction_dv = double.NaN; // departure: the follow-up correction, if one is needed
            public ArrivalPlane plane;
            public double arrival_inc = double.NaN; // inclination of the orbit around the moon this path gives, °
            public bool pe_ok, plane_ok;           // the two halves of 'encounter'
            public double Dv => Math.Sqrt(prograde * prograde + normal * normal + radial * radial);
        }

        public struct Moon
        {
            public Vector3d r, v;      // relative to the parent, at UT 'now'
            public double mu, radius, soi;
        }

        public const double DefaultLead = 120;
        // periapsis within this of the wanted one counts as hit
        public static double PeTolerance(double pe_alt) => Math.Max(5000, 0.15 * pe_alt);

        /// <summary>
        /// Departure burn. r, v: vessel state relative to the parent at 'now'; moon: the target's state
        /// at 'now'; lead: earliest burn, s from now; pe_alt: wanted periapsis altitude above the moon.
        /// </summary>
        public static Plan Compute(Vector3d r, Vector3d v, double mu, double now, Moon moon, double pe_alt, double lead = DefaultLead,
            ArrivalPlane plane = ArrivalPlane.Any)
        {
            var plan = new Plan { plane = plane };
            double alpha = 2.0 / r.magnitude - v.sqrMagnitude / mu;
            if (alpha <= 0) { plan.problem = "the vessel isn't in a closed orbit"; return plan; }
            double sma = 1.0 / alpha;
            double T = OrbitMath.Period(mu, sma);
            double moon_sma = 1.0 / (2.0 / moon.r.magnitude - moon.v.sqrMagnitude / mu);
            if (moon_sma <= 0) { plan.problem = "the target has no closed orbit"; return plan; }
            if (sma * 1.5 > moon_sma) { plan.problem = "the vessel's orbit already reaches out towards the target - use Course correction instead"; return plan; }
            double Tm = OrbitMath.Period(mu, moon_sma);
            double synodic = 1.0 / Math.Abs(1.0 / T - 1.0 / Tm);
            double window = Math.Min(synodic + T, 30 * T);
            double rp = moon.radius + pe_alt;

            // ---- 1. window: coarse sweep of the phase error, then golden-section on it
            double PhaseError(double dt)
            {
                Prop(r, v, mu, dt, out var rb, out _);
                double ra = moon_sma;
                double err = Math.PI;
                for (int k = 0; k < 2; k++)
                {
                    double a = (rb.magnitude + ra) / 2;
                    double tof = Math.PI * Math.Sqrt(a * a * a / mu);
                    Prop(moon.r, moon.v, mu, dt + tof, out var rm, out _);
                    ra = rm.magnitude;
                    // in the vessel's plane: an inclined moon's height above it is the correction's job
                    var h = Vector3d.Cross(r, v).normalized;
                    err = Angle(-rb, rm - h * Vector3d.Dot(rm, h));
                }
                return err;
            }

            int samples = Math.Max(60, (int)(window / T * 120));
            double best_dt = lead, best_err = double.MaxValue;
            for (int i = 0; i <= samples; i++)
            {
                double dt = lead + window * i / samples;
                double e = PhaseError(dt);
                if (e < best_err) { best_err = e; best_dt = dt; }
            }
            double step = window / samples;
            best_dt = Golden(PhaseError, Math.Max(lead, best_dt - step), best_dt + step, 40);

            Prop(r, v, mu, best_dt, out var rb0, out var vb0);
            double a0 = (rb0.magnitude + moon_sma) / 2;
            double p0 = OrbitMath.SpeedAt(mu, rb0.magnitude, a0) - vb0.magnitude;
            double tof0 = Math.PI * Math.Sqrt(a0 * a0 * a0 / mu);
            // near the Hohmann arrival: a much earlier or later pass is a faster, dearer path
            double lo = 0.6 * tof0, hi = 1.4 * tof0;

            Approach Eval(double dt, double p, double n) => AfterBurn(r, v, mu, moon, dt, p, n, 0, lo, hi, rp);

            // ---- 2. in-plane: aim the in-plane part of the miss so the whole miss can be b
            double InPlaneCost(double[] x)
            {
                if (x[0] < lead) return 1e12 + (lead - x[0]);
                var ap = Eval(x[0], x[1], 0);
                if (!ap.ok) return 1e11;
                // polar: the in-plane part of a polar B is ~0, i.e. straight at the moon - aim for a
                // safe pass beside it instead and leave the move over the pole to the correction
                double e = InPlaneError(ap, plane == ArrivalPlane.Polar ? ArrivalPlane.Any : plane) / 5000.0;
                return e * e + 0.01 * Math.Abs(x[1]);
            }
            var x2 = NelderMead(InPlaneCost, new[] { best_dt, p0 }, new[] { T / 50, 10.0 }, 300);
            x2 = NelderMead(InPlaneCost, x2, new[] { T / 400, 2.0 }, 200);
            double dt_b = x2[0], p_b = x2[1], n_b = 0;
            var result = Eval(dt_b, p_b, 0);

            // ---- 3. off the plane: keep a normal part only if it's cheap
            if (result.ok && !Hits(result, moon, pe_alt, plane))
            {
                double FullCost(double[] x)
                {
                    if (x[0] < lead) return 1e12 + (lead - x[0]);
                    var ap = Eval(x[0], x[1], x[2]);
                    if (!ap.ok) return 1e11;
                    double e = MissError(ap, plane) / 5000.0;
                    return e * e + 0.01 * Math.Sqrt(x[1] * x[1] + x[2] * x[2]);
                }
                var x3 = NelderMead(FullCost, new[] { dt_b, p_b, 0.0 }, new[] { T / 50, 10.0, 10.0 }, 400);
                x3 = NelderMead(FullCost, x3, new[] { T / 400, 2.0, 2.0 }, 300);
                var r3 = Eval(x3[0], x3[1], x3[2]);
                double cheap = Math.Max(30, 0.03 * Math.Abs(x3[1]));
                if (r3.ok && Hits(r3, moon, pe_alt, plane) && Math.Abs(x3[2]) <= cheap)
                {
                    dt_b = x3[0]; p_b = x3[1]; n_b = x3[2];
                    result = r3;
                }
            }

            Fill(plan, result, moon, pe_alt, now + dt_b, p_b, n_b, 0, plane);
            if (!result.ok) { plan.problem = "no approach to the target found"; return plan; }
            plan.ok = true;
            if (!plan.encounter)
            {
                // what the course correction will cost: plan it from a third of the way out
                Prop(r, v, mu, dt_b, out var rb, out var vb);
                vb += vb.normalized * p_b + Vector3d.Cross(rb, vb).normalized * n_b;
                double t_c = 0.3 * result.t;
                Prop(rb, vb, mu, t_c, out var rc, out var vc);
                Prop(moon.r, moon.v, mu, dt_b + t_c, out var mr, out var mv);
                var c = Correct(rc, vc, mu, now + dt_b + t_c, new Moon { r = mr, v = mv, mu = moon.mu, radius = moon.radius, soi = moon.soi }, pe_alt, 0, plane);
                plan.correction_dv = c.ok ? c.Dv : double.NaN;
            }
            return plan;
        }

        /// <summary>
        /// Course correction on the way out (vessel already on a transfer path around the parent):
        /// the cheapest single burn from lead seconds on that moves the closest approach onto the aim.
        /// </summary>
        public static Plan Correct(Vector3d r, Vector3d v, double mu, double now, Moon moon, double pe_alt, double lead = DefaultLead,
            ArrivalPlane plane = ArrivalPlane.Any)
        {
            var plan = new Plan { plane = plane };
            double rp = moon.radius + pe_alt;
            double alpha = 2.0 / r.magnitude - v.sqrMagnitude / mu;
            // where the coast reaches the moon now: a full orbit if closed, else the time to go twice the moon's distance
            double span = alpha > 0 ? OrbitMath.Period(mu, 1.0 / alpha) : 2 * moon.r.magnitude / Math.Max(100, v.magnitude);
            var now_ap = AfterBurn(r, v, mu, moon, 0, 0, 0, 0, lead, span, rp);
            if (!now_ap.ok) { plan.problem = "no approach to the target found"; return plan; }
            if (Hits(now_ap, moon, pe_alt, plane))
            {
                Fill(plan, now_ap, moon, pe_alt, now, 0, 0, 0, plane);
                plan.ok = true;
                return plan;
            }
            double t_ca = now_ap.t;
            if (t_ca < lead * 2) { plan.problem = "too close to the target to correct"; return plan; }
            double lo = Math.Max(lead, 0.5 * t_ca), hi = Math.Min(span, 1.5 * t_ca);

            double Cost(double[] x)
            {
                if (x[0] < lead) return 1e12 + (lead - x[0]);
                if (x[0] > 0.8 * t_ca) return 1e12 + (x[0] - 0.8 * t_ca);
                var ap = AfterBurn(r, v, mu, moon, x[0], x[1], x[2], x[3], lo - x[0], hi - x[0], rp);
                if (!ap.ok) return 1e11;
                double e = MissError(ap, plane) / 5000.0;
                return e * e + 0.01 * Math.Sqrt(x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
            }
            double start = lead + 0.05 * (t_ca - lead);
            var x = NelderMead(Cost, new[] { start, 0.0, 0.0, 0.0 }, new[] { 0.1 * t_ca, 5.0, 5.0, 5.0 }, 500);
            x = NelderMead(Cost, x, new[] { 0.02 * t_ca, 1.0, 1.0, 1.0 }, 300);
            var result = AfterBurn(r, v, mu, moon, x[0], x[1], x[2], x[3], lo - x[0], hi - x[0], rp);
            Fill(plan, result, moon, pe_alt, now + x[0], x[1], x[2], x[3], plane);
            if (!result.ok) { plan.problem = "no approach to the target found"; return plan; }
            if (!plan.encounter)
            {
                plan.problem = $"best correction still misses: periapsis {plan.pe_alt_m / 1000:n0} km, inclination {plan.arrival_inc:n0}°";
                return plan;
            }
            plan.ok = true;
            return plan;
        }

        // ================================================================ capture

        public class CapturePlan
        {
            public bool ok;
            public string problem = "";
            public bool done;          // already in the wanted orbit
            public double UT;          // burn time
            public double prograde;    // m/s along the velocity (negative = retrograde)
            public bool at_apoapsis;   // burning at apoapsis (the second half of a two-burn capture)
            public bool circular;      // the burn leaves a circular orbit (else an ellipse, one more burn to go)
            public double burn_alt_m, other_alt_m; // the resulting orbit's apsides: burn point, opposite side
            public double pe_alt_m, ap_alt_m = double.PositiveInfinity; // the orbit now
            public double inc;         // inclination now, °
            public bool closed;        // the orbit now is closed inside the SOI
        }

        /// <summary>Apsis altitudes within this of the wanted one count as there.</summary>
        public static double AltTolerance(double alt) => Math.Max(2000, 0.05 * alt);

        /// <summary>
        /// Capture into a circular orbit at alt around the body the vessel is now in (r, v relative to
        /// it at 'now'). One burn at periapsis if periapsis is already at alt (the transfer aims for
        /// that); otherwise the periapsis burn puts the far side at alt and a second capture step
        /// circularises there at apoapsis.
        /// </summary>
        public static CapturePlan Capture(Vector3d r, Vector3d v, double mu, double radius, double soi, double now, double alt, double lead = DefaultLead)
        {
            var c = new CapturePlan();
            var hv = Vector3d.Cross(r, v);
            double h = hv.magnitude;
            if (h < 1) { c.problem = "falling straight in"; return c; }
            c.inc = Math.Acos(Math.Max(-1, Math.Min(1, hv.z / h))) * 180 / Math.PI;
            var ev = Vector3d.Cross(v, hv) / mu - r.normalized;
            double e = ev.magnitude;
            double p = h * h / mu;
            double rp = p / (1 + e);
            double ra = e < 1 ? p / (1 - e) : double.PositiveInfinity;
            c.pe_alt_m = rp - radius;
            c.ap_alt_m = ra - radius;
            c.closed = e < 1 && ra < soi;
            double rw = radius + alt;
            double tol = AltTolerance(alt);
            if (rp < radius) { c.problem = $"periapsis {c.pe_alt_m / 1000:n0} km is below the surface - on course to hit it"; return c; }
            if (rw >= 0.9 * soi) { c.problem = $"{alt / 1000:n0} km is too close to the edge of the sphere of influence ({(soi - radius) / 1000:n0} km)"; return c; }

            bool pe_on = Math.Abs(rp - rw) <= tol, ap_on = c.closed && Math.Abs(ra - rw) <= tol;
            if (pe_on && ap_on) { c.ok = c.done = true; return c; }
            c.at_apoapsis = ap_on;

            // time to the burn point
            double alpha = 2.0 / r.magnitude - v.sqrMagnitude / mu;
            double a = 1.0 / alpha;
            double sign = Vector3d.Dot(r, v) >= 0 ? 1 : -1;
            double t;
            if (e < 1)
            {
                double n = Math.Sqrt(mu / (a * a * a)), T = 2 * Math.PI / n;
                double E = Math.Acos(Math.Max(-1, Math.Min(1, (1 - r.magnitude / a) / Math.Max(e, 1e-9)))) * sign;
                double M = E - e * Math.Sin(E);
                double target = c.at_apoapsis ? Math.PI : 2 * Math.PI;
                t = ((target - M) / n % T + T) % T;
                if (!c.closed && sign > 0) { c.problem = "already past periapsis and leaving"; return c; }
                if (t < lead)
                {
                    if (!c.closed) { c.problem = $"periapsis is only {t:n0} s away - too soon to turn and burn"; return c; }
                    t += T;
                }
            }
            else
            {
                if (sign > 0) { c.problem = "already past periapsis and leaving"; return c; }
                double n = Math.Sqrt(mu / -(a * a * a));
                double F = -Acosh(Math.Max(1, (1 - r.magnitude / a) / e));
                double M = e * Math.Sinh(F) - F;
                t = -M / n;
                if (t < lead) { c.problem = $"periapsis is only {t:n0} s away - too soon to turn and burn"; return c; }
            }

            double rb = c.at_apoapsis ? ra : rp;
            double other = c.at_apoapsis || pe_on ? rb : rw;
            c.circular = Math.Abs(other - rb) <= tol;
            double vb = Math.Sqrt(mu * (2 / rb - alpha));
            double a_new = (rb + other) / 2;
            c.prograde = Math.Sqrt(mu * (2 / rb - 1 / a_new)) - vb;
            c.UT = now + t;
            c.burn_alt_m = rb - radius;
            c.other_alt_m = other - radius;
            c.ok = true;
            return c;
        }

        static double Acosh(double x) => Math.Log(x + Math.Sqrt(x * x - 1));

        // ================================================================ estimates (VAB, no state vectors)

        /// <summary>Hohmann departure burn from a circular orbit of radius r1 to a moon at radius r2.</summary>
        public static double DepartureDv(double mu, double r1, double r2)
        {
            double a = (r1 + r2) / 2;
            return OrbitMath.SpeedAt(mu, r1, a) - OrbitMath.CircularSpeed(mu, r1);
        }

        /// <summary>Arrival speed relative to a moon on a circular orbit at r2, after a Hohmann from r1.</summary>
        public static double ArrivalVInf(double mu, double r1, double r2)
        {
            double a = (r1 + r2) / 2;
            return Math.Abs(OrbitMath.CircularSpeed(mu, r2) - OrbitMath.SpeedAt(mu, r2, a));
        }

        /// <summary>Burn at periapsis rp of the arrival hyperbola to a circular orbit there.</summary>
        public static double CaptureDv(double mu_moon, double rp, double v_inf) =>
            Math.Sqrt(v_inf * v_inf + 2 * mu_moon / rp) - Math.Sqrt(mu_moon / rp);

        public static double ImpactParameter(double rp, double v_inf, double mu_moon) =>
            rp * Math.Sqrt(1 + 2 * mu_moon / (rp * v_inf * v_inf));

        /// <summary>Inverse of ImpactParameter: the periapsis a miss distance b bends into.</summary>
        public static double PeriapsisFromMiss(double b, double v_inf, double mu_moon)
        {
            double k = mu_moon / (v_inf * v_inf);
            return Math.Sqrt(k * k + b * b) - k;
        }

        // ================================================================ internals

        struct Approach
        {
            public bool ok;
            public double t, miss, aim, v_rel, off_plane;
            public Vector3d d, s, h;   // miss vector (vessel - moon), arrival direction, the path's plane normal
            public double inc;         // inclination of the orbit around the moon, °
            public bool in_soi;        // d, s, miss, v_rel from the arrival hyperbola (else the straight-pass estimate)
        }

        /// <summary>
        /// The arrival hyperbola from a moon-relative state: its B vector (from the moon to the incoming
        /// asymptote, |B| = impact parameter), the incoming direction and v_inf. False if not hyperbolic.
        /// </summary>
        static bool Hyperbola(Vector3d r, Vector3d v, double mu, out Vector3d B, out Vector3d s_in, out double v_inf)
        {
            B = s_in = Vector3d.zero;
            v_inf = 0;
            double energy2 = v.sqrMagnitude - 2 * mu / r.magnitude;
            if (energy2 <= 1) return false;
            v_inf = Math.Sqrt(energy2);
            var hv = Vector3d.Cross(r, v);
            double h = hv.magnitude;
            if (h < 1) return false;
            var hn = hv / h;
            var ev = Vector3d.Cross(v, hv) / mu - r.normalized;
            double e = ev.magnitude;
            if (e <= 1) return false;
            var p = ev / e;
            var q = Vector3d.Cross(hn, p);
            s_in = p * (1 / e) + q * Math.Sqrt(1 - 1 / (e * e));
            B = Vector3d.Cross(s_in, hn) * (h / v_inf);
            return true;
        }

        /// <summary>The least inclination an orbit can have whose plane holds the arrival direction s, °.</summary>
        static double MinInclination(Vector3d s) => Math.Asin(Math.Min(1, Math.Abs(s.z) / Math.Max(1e-9, s.magnitude))) * 180 / Math.PI;

        /// <summary>Where the miss should be (the B vector) for the wanted arrival plane.</summary>
        static Vector3d TargetB(Approach ap, ArrivalPlane plane)
        {
            var z = new Vector3d(0, 0, 1);
            var d_dir = ap.miss > 1 ? ap.d / ap.miss : Vector3d.Cross(ap.s, ap.h).normalized;
            if (plane == ArrivalPlane.Equatorial)
            {
                var e = Vector3d.Cross(ap.s, z);
                if (e.magnitude > 0.05) return e.normalized * ap.aim;
            }
            else if (plane == ArrivalPlane.Polar)
            {
                var u = z - ap.s * Vector3d.Dot(ap.s, z);
                if (u.magnitude > 0.05)
                {
                    u = u.normalized;
                    return (Vector3d.Dot(d_dir, u) >= 0 ? u : -u) * ap.aim;
                }
            }
            return d_dir * ap.aim;
        }

        static double MissError(Approach ap, ArrivalPlane plane) =>
            plane == ArrivalPlane.Any ? Math.Abs(ap.miss - ap.aim) : (ap.d - TargetB(ap, plane)).magnitude;

        /// <summary>The part of the miss error a burn in the path's own plane can fix.</summary>
        static double InPlaneError(Approach ap, ArrivalPlane plane)
        {
            if (plane == ArrivalPlane.Any)
            {
                double z = ap.off_plane;
                double want = Math.Abs(z) < ap.aim ? Math.Sqrt(ap.aim * ap.aim - z * z) : 0;
                double inplane = Math.Sqrt(Math.Max(0, ap.miss * ap.miss - z * z));
                return Math.Abs(inplane - want);
            }
            var e = ap.d - TargetB(ap, plane);
            return (e - ap.h * Vector3d.Dot(e, ap.h)).magnitude;
        }

        // equatorial: as near as the arrival direction allows (an inclined moon's arrival can't lie in the equator)
        static bool PlaneHits(Approach ap, ArrivalPlane plane) =>
            plane == ArrivalPlane.Any ||
            (plane == ArrivalPlane.Equatorial
                ? ap.inc <= Math.Max(PlaneToleranceDeg, MinInclination(ap.s) + 1)
                : Math.Abs(ap.inc - 90) <= PlaneToleranceDeg);

        static bool PeHits(Approach ap, Moon moon, double pe_alt)
        {
            if (!ap.ok || !ap.in_soi || ap.miss > moon.soi) return false;
            double pe = PeriapsisFromMiss(ap.miss, ap.v_rel, moon.mu) - moon.radius;
            return Math.Abs(pe - pe_alt) <= PeTolerance(pe_alt);
        }

        static bool Hits(Approach ap, Moon moon, double pe_alt, ArrivalPlane plane) => PeHits(ap, moon, pe_alt) && PlaneHits(ap, plane);

        static void Fill(Plan plan, Approach ap, Moon moon, double pe_alt, double ut, double p, double n, double rad, ArrivalPlane plane)
        {
            plan.plane = plane;
            plan.arrival_inc = ap.ok ? ap.inc : double.NaN;
            plan.UT = ut;
            plan.prograde = p;
            plan.normal = n;
            plan.radial = rad;
            plan.tof = ap.t;
            plan.miss_m = ap.miss;
            plan.aim_m = ap.aim;
            plan.off_plane_m = ap.off_plane;
            plan.v_inf = ap.v_rel;
            if (ap.v_rel > 1)
            {
                plan.pe_alt_m = PeriapsisFromMiss(ap.miss, ap.v_rel, moon.mu) - moon.radius;
                plan.capture_dv = CaptureDv(moon.mu, moon.radius + pe_alt, ap.v_rel);
            }
            plan.pe_ok = PeHits(ap, moon, pe_alt);
            plan.plane_ok = ap.ok && PlaneHits(ap, plane);
            plan.encounter = plan.pe_ok && plan.plane_ok;
        }

        /// <summary>
        /// Coast dt, burn (prograde p, normal n, radial-out rad), then find the closest approach to the
        /// moon between lo and hi seconds after the burn.
        /// </summary>
        static Approach AfterBurn(Vector3d r, Vector3d v, double mu, Moon moon, double dt, double p, double n, double rad,
            double lo, double hi, double rp)
        {
            var result = new Approach();
            Prop(r, v, mu, dt, out var rb, out var vb);
            var prograde = vb.normalized;
            var normal = Vector3d.Cross(rb, vb).normalized;
            var radial = Vector3d.Cross(prograde, normal);
            var v1 = vb + prograde * p + normal * n + radial * rad;
            Prop(moon.r, moon.v, mu, dt, out var mr, out var mv);
            lo = Math.Max(0, lo);
            if (hi <= lo) return result;

            double Dist(double t)
            {
                Prop(rb, v1, mu, t, out var a, out _);
                Prop(mr, mv, mu, t, out var b, out _);
                return (a - b).magnitude;
            }

            const int N = 80;
            double best_t = lo, best = double.MaxValue;
            double entry_lo = double.NaN, entry_hi = double.NaN, prev_t = lo;
            for (int i = 0; i <= N; i++)
            {
                double t = lo + (hi - lo) * i / N;
                double d = Dist(t);
                if (d < best) { best = d; best_t = t; }
                if (double.IsNaN(entry_hi) && d < moon.soi)
                {
                    entry_lo = i == 0 ? t : prev_t;
                    entry_hi = t;
                }
                prev_t = t;
            }
            double h = (hi - lo) / N;
            best_t = Golden(Dist, Math.Max(lo, best_t - h), Math.Min(hi, best_t + h), 40);

            Prop(rb, v1, mu, best_t, out var ra, out var va);
            Prop(mr, mv, mu, best_t, out var rm, out var vm);
            var d_vec = ra - rm;
            result.t = best_t;
            result.miss = d_vec.magnitude;
            result.off_plane = Vector3d.Dot(d_vec, Vector3d.Cross(rb, v1).normalized);
            result.v_rel = (va - vm).magnitude;
            if (result.v_rel < 1) return result;
            result.d = d_vec;
            result.s = (va - vm) / result.v_rel;
            result.h = Vector3d.Cross(rb, v1).normalized;

            // inside the sphere of influence: the exact arrival hyperbola from the state at entry
            // (patched conics, as the game predicts it) instead of the straight-pass estimate
            if (!double.IsNaN(entry_hi))
            {
                double a = entry_lo, b = entry_hi;
                if (Dist(a) < moon.soi) b = a;
                else
                    for (int k = 0; k < 30; k++)
                    {
                        double m = (a + b) / 2;
                        if (Dist(m) < moon.soi) b = m; else a = m;
                    }
                Prop(rb, v1, mu, b, out var re, out var ve);
                Prop(mr, mv, mu, b, out var rme, out var vme);
                if (Hyperbola(re - rme, ve - vme, moon.mu, out var B, out var s_in, out double v_inf))
                {
                    result.d = B;
                    result.s = s_in;
                    result.miss = B.magnitude;
                    result.v_rel = v_inf;
                    result.in_soi = true;
                }
            }
            var hm = Vector3d.Cross(result.d, result.s);
            result.inc = hm.magnitude > 1e-6
                ? Math.Acos(Math.Max(-1, Math.Min(1, hm.z / hm.magnitude))) * 180 / Math.PI
                : double.NaN;
            result.aim = ImpactParameter(rp, result.v_rel, moon.mu);
            result.ok = true;
            return result;
        }

        /// <summary>KeplerPropagator, with whole revolutions of a closed orbit taken off first.</summary>
        static void Prop(Vector3d r, Vector3d v, double mu, double dt, out Vector3d r1, out Vector3d v1)
        {
            double alpha = 2.0 / r.magnitude - v.sqrMagnitude / mu;
            if (alpha > 0)
            {
                double T = 2 * Math.PI * Math.Sqrt(1.0 / (alpha * alpha * alpha * mu));
                dt %= T;
            }
            KeplerPropagator.Propagate(r, v, mu, dt, out r1, out v1);
        }

        static double Angle(Vector3d a, Vector3d b)
        {
            double c = Vector3d.Dot(a, b) / (a.magnitude * b.magnitude);
            return Math.Acos(Math.Max(-1, Math.Min(1, c)));
        }

        static double Golden(Func<double, double> f, double a, double b, int iterations)
        {
            const double g = 0.6180339887498949;
            double c = b - g * (b - a), d = a + g * (b - a);
            double fc = f(c), fd = f(d);
            for (int i = 0; i < iterations; i++)
            {
                if (fc < fd) { b = d; d = c; fd = fc; c = b - g * (b - a); fc = f(c); }
                else { a = c; c = d; fc = fd; d = a + g * (b - a); fd = f(d); }
            }
            return (a + b) / 2;
        }

        static double[] NelderMead(Func<double[], double> f, double[] start, double[] scale, int iterations)
        {
            int n = start.Length;
            var pts = new List<double[]> { (double[])start.Clone() };
            for (int i = 0; i < n; i++)
            {
                var p = (double[])start.Clone();
                p[i] += scale[i];
                pts.Add(p);
            }
            var vals = pts.Select(f).ToList();
            for (int it = 0; it < iterations; it++)
            {
                var order = Enumerable.Range(0, n + 1).OrderBy(i => vals[i]).ToList();
                pts = order.Select(i => pts[i]).ToList();
                vals = order.Select(i => vals[i]).ToList();
                var centroid = new double[n];
                for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    centroid[j] += pts[i][j] / n;
                double[] Along(double t) => centroid.Select((c, j) => c + t * (pts[n][j] - c)).ToArray();

                var xr = Along(-1);
                double fr = f(xr);
                if (fr < vals[0])
                {
                    var xe = Along(-2);
                    double fe = f(xe);
                    if (fe < fr) { pts[n] = xe; vals[n] = fe; }
                    else { pts[n] = xr; vals[n] = fr; }
                }
                else if (fr < vals[n - 1]) { pts[n] = xr; vals[n] = fr; }
                else
                {
                    var xc = fr < vals[n] ? Along(-0.5) : Along(0.5);
                    double fc = f(xc);
                    if (fc < Math.Min(fr, vals[n])) { pts[n] = xc; vals[n] = fc; }
                    else
                    {
                        for (int i = 1; i <= n; i++)
                        {
                            pts[i] = pts[i].Select((x, j) => pts[0][j] + 0.5 * (x - pts[0][j])).ToArray();
                            vals[i] = f(pts[i]);
                        }
                    }
                }
            }
            int besti = Enumerable.Range(0, n + 1).OrderBy(i => vals[i]).First();
            return pts[besti];
        }
    }
}
