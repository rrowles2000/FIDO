using System;
using System.Collections.Generic;
using KSP.Sim.impl;
using UnityEngine;

namespace K2D2.Lift.Stage1
{
    // Orbit and steering helpers shared by the simulator and the lift pilot (the Optimiser's
    // apoapsis-hold, coast-to-apoapsis, profile and max-Q bucket guidance went with it, 2026-11-15).
    public static class Stage1Guidance
    {
        public static bool ApPe(double mu, double r, double vr, double vt, out double apR, out double peR)
        {
            double v2 = vr * vr + vt * vt;
            double energy = v2 / 2 - mu / r;
            double h = r * vt;
            double e = Math.Sqrt(Math.Max(0, 1 + 2 * energy * h * h / (mu * mu)));
            if (energy >= 0)
            {
                apR = double.PositiveInfinity;
                peR = h * h / (mu * (1 + e));
                return false;
            }
            double a = -mu / (2 * energy);
            apR = a * (1 + e);
            peR = a * (1 - e);
            return true;
        }

        // Smooths the commanded pitch: limited rate (lower at high dynamic pressure) and limited
        // change of rate, so kinks in the profile (e.g. its 45° point) don't jerk the vessel. At
        // max-Q through Mach 1 a sudden stop in the pitch-down left SAS overshooting nose-up, and on
        // the 2026-10-20 flight 6 that grew into a tumble.
        public class PitchSmoother
        {
            public const double MaxRateDegS = 3.0, MaxRateHighQDegS = 1.5, MaxAccelDegS2 = 1.0;
            double last = double.NaN, rate;

            public void Reset() { last = double.NaN; rate = 0; }

            public double Next(double wantDeg, double dt, double qKPa)
            {
                if (double.IsNaN(last) || dt <= 0) { last = wantDeg; rate = 0; return wantDeg; }
                double maxRate = qKPa > HighQKPa ? MaxRateHighQDegS : MaxRateDegS;
                double want = Math.Max(-maxRate, Math.Min(maxRate, (wantDeg - last) / dt));
                rate = Math.Max(rate - MaxAccelDegS2 * dt, Math.Min(rate + MaxAccelDegS2 * dt, want));
                last += rate * dt;
                return last;
            }
        }

        // How far (deg) the nose may point from the airflow. Side-on drag is ~10x nose-first, so
        // stay close to prograde while the air is thick - and almost exactly prograde whenever
        // dynamic pressure is high. On 2026-10-20 the sat vessel pitched up uncontrollably from
        // ~+1° AoA at q≈25 kPa (Mach 1.66, flight 7) after recovering from +6° twice: with the nose
        // above the path at high q it is aerodynamically unstable.
        public const double HighQKPa = 10;

        // 2026-11-13 (R2-1, six unfaired satellites on top): flipped at q 13.5 kPa, Mach 0.55, from
        // -1.8° AoA - the old 2° floor between 2 and 10 kPa let it get there. The limit now keeps
        // falling as 4/q down to 0.5° (1° at 4 kPa, 0.5° from 8 kPa), so in thick air the ascent
        // is a near zero-AoA gravity turn.
        public static double MaxAoADeg(double qKPa, double mach = 0)
        {
            if (qKPa > HighQKPa) return 0.5;
            return qKPa <= 0.04 ? 90 : Math.Max(0.5, Math.Min(90, 4 / qKPa));
        }

        public static double LimitAoA(double elevDeg, double airPathDeg, double qKPa, double mach = 0)
        {
            double max = MaxAoADeg(qKPa, mach);
            return Math.Max(airPathDeg - max, Math.Min(airPathDeg + max, elevDeg));
        }

        // Full below the target apoapsis, easing to zero 1 km above it.
        public static float ApThrottle(double apR, double targetApR) =>
            (float)Math.Max(0, Math.Min(1, (targetApR + 1000 - apR) / 3000));



    }

    public struct SimResult
    {
        // stage 1
        public bool reachedAp;   // apoapsis reached the target during stage 1 (hold phase started)
        public bool capHit;      // MECO because periapsis reached the stage-1 max periapsis
        public bool outOfFuel;   // stage 1 burnt dry
        public bool crashed;
        public double tBurnout, altKm, apKm, peKm, fpaDeg;
        public double leftoverDv, maxQkPa, holdStartAltKm;
        public double maxHeat;      // max of sqrt(density)*v^3 - convective heating proxy
        public double maxHeatAltKm; // where it peaked
        public double minBucket;    // lowest max-Q throttle factor used (1 = never throttled back)
        // upper stage, from separation to a circular orbit at the target apoapsis
        public bool upperFeasible;
        public double upperClimbDv; // spent finishing the climb if stage 1 left Ap short
        public double circDv;       // circularisation at apoapsis
        public double UpperDv => upperClimbDv + circDv;
        public double upperMarginDv;
        public double score;
    }

    // 2D (orbital plane) point-mass ascent simulation: stage 1 from the pad to main engine
    // cut-off, then the upper stage to a circular orbit at the target apoapsis. Body rotation is
    // included along-track only (the cross-track part, which matters for polar launches, is
    // ignored).
    public class Stage1Sim
    {
        // Convective heating proxy sqrt(rho)*v^3 that an ascent should stay under. A rough
        // estimate (~1.8 km/s at 30 km on Kerbin is about here); not calibrated against KSP2's
        // thermal model yet. (Was Stage1Optimiser.HeatLimit until the Optimiser was removed, 2026-11-15.)
        public const double HeatLimit = 5e8;

        const double G0 = 9.80665;
        const double Dt = 0.2;
        public const double StepS = Dt; // simulation step, for controllers that need their dt

        readonly Stage1Model model;
        readonly double mu, R, depth, omegaCosLat, launchR;
        readonly double atmoStep = 100;
        readonly double[] pKPa, rho, sound;

        // Multiplies the modelled drag - the optimiser's worst-case run uses more than 1.
        public double dragScale = 1;
        // thrust at the same propellant flow (= an Isp factor) - one of the factors the New lift
        // system's calibration (Next/Calibrator.cs) fits from flights
        public double thrustScale = 1;

        // ---- read-only views for the model export (Next/ModelExport.cs)
        public double AtmoStepM => atmoStep;
        public double AtmoDepthM => depth;
        public double[] AtmoPressureKPa => pKPa;
        public double[] AtmoDensity => rho;
        public double[] AtmoSoundSpeed => sound;

        // Step() holds the command within the Optimiser's AoA limit. A caller that applies its
        // own limit (the New system's shield, whose Q·AoA limit is a setting) turns this off.
        public bool limitAoAInStep = true;

        public Stage1Sim(Stage1Model model)
        {
            this.model = model;
            var body = model.body;
            mu = body.gravParameter;
            R = body.radius;
            depth = body.hasAtmosphere ? body.atmosphereDepth : 0;
            double omega = body.rotationPeriod != 0 ? 2 * Math.PI / body.rotationPeriod : 0;
            omegaCosLat = omega * Math.Cos(model.latitudeDeg * Math.PI / 180);
            launchR = R + Math.Max(0, model.launchAltitude);

            int n = depth > 0 ? (int)(depth / atmoStep) + 2 : 0;
            pKPa = new double[n]; rho = new double[n]; sound = new double[n];
            for (int i = 0; i < n; i++)
            {
                double alt = i * atmoStep;
                double p = body.GetPressure(alt);
                double d = body.GetDensity(p, body.GetTemperature(alt));
                pKPa[i] = p;
                rho[i] = d;
                sound[i] = d > 0 ? AeroUtilities.GetSoundSpeed(body.atmosphereAdiabaticIndex, p, d) : 0;
            }
        }

        void Atmo(double alt, out double p, out double d, out double a)
        {
            p = d = a = 0;
            if (alt >= depth || pKPa.Length == 0) return;
            double x = Math.Max(0, alt) / atmoStep;
            int i = Math.Min((int)x, pKPa.Length - 2);
            double t = x - i;
            p = pKPa[i] + (pKPa[i + 1] - pKPa[i]) * t;
            d = rho[i] + (rho[i + 1] - rho[i]) * t;
            a = sound[i] + (sound[i + 1] - sound[i]) * t;
        }

        // Vessel state carried between the stage-1 and upper-stage burns.
        struct State
        {
            public double r, vr, vt, m, t;
            public int phase;
        }

        // One powered step. Returns false when the current phase list has run out of propellant.
        void AirPath(State s, double sinH, out double pathDeg, out double qKPa) => AirPath(s, sinH, out pathDeg, out qKPa, out _);

        void AirPath(State s, double sinH, out double pathDeg, out double qKPa, out double mach)
        {
            Atmo(s.r - R, out _, out double d, out double a);
            double vAirT = s.vt - omegaCosLat * s.r * sinH, vAirR = s.vr;
            double v2 = vAirT * vAirT + vAirR * vAirR;
            pathDeg = Math.Atan2(vAirR, vAirT) * 180 / Math.PI;
            qKPa = 0.0005 * d * v2;
            mach = a > 0 ? Math.Sqrt(v2) / a : 0;
        }

        // elevDeg is the commanded pitch; it is held within MaxAoADeg of the airflow, as the flight
        // controller does.
        bool Step(ref State s, List<Stage1Model.BurnPhase> phases, float throttle, double elevDeg,
                  double sinH, ref SimResult res, ref double dvUsed)
        {
            var ph = phases[s.phase];
            double alt = s.r - R;
            Atmo(alt, out double p, out double d, out double a);

            double vAirT = s.vt - omegaCosLat * s.r * sinH, vAirR = s.vr;
            double vAir = Math.Sqrt(vAirT * vAirT + vAirR * vAirR);
            double qPaNow = 0.5 * d * vAir * vAir;
            if (vAir > 5 && limitAoAInStep)
                elevDeg = Stage1Guidance.LimitAoA(elevDeg, Math.Atan2(vAirR, vAirT) * 180 / Math.PI, qPaNow / 1000,
                                                  a > 0 ? vAir / a : 0);
            double dragAcc = 0;
            if (d > 0 && vAir > 0.1)
            {
                // q and heating depend only on the air and the speed, so they're tracked with or
                // without a drag table (the VAB has none; before 2026-11-13 it reported 0 kPa / 0%)
                res.maxQkPa = Math.Max(res.maxQkPa, qPaNow / 1000);
                double heat = Math.Sqrt(d) * vAir * vAir * vAir;
                if (heat > res.maxHeat) { res.maxHeat = heat; res.maxHeatAltKm = alt / 1000; }
            }
            if (d > 0 && vAir > 0.1 && model.dragValid)
            {
                double mach = a > 0 ? vAir / a : 0;
                double aoa = vAir > 5 ? elevDeg - Math.Atan2(vAirR, vAirT) * 180 / Math.PI : 0;
                double pre = PhysicsSettings.DragCurvePseudoReynolds.Evaluate((float)(d * vAir));
                double dragKN = model.AreaDragAt(mach, aoa) * PhysicsSettings.DragCubeMultiplier * pre *
                                PhysicsSettings.DragMultiplier * qPaNow * 0.001 * dragScale;
                dragAcc = dragKN / s.m;
            }

            double atm = Math.Min(1.0, p / 101.325);
            double isp = ph.ispVac + (ph.ispASL - ph.ispVac) * atm;
            double thrust = ph.thrustVac * throttle * isp / ph.ispVac * thrustScale;
            double mdot = ph.thrustVac * throttle / (ph.ispVac * G0);
            double er = elevDeg * Math.PI / 180;

            double accT = thrust / s.m * Math.Cos(er) - (vAir > 0.1 ? dragAcc * vAirT / vAir : 0);
            double accR = thrust / s.m * Math.Sin(er) - (vAir > 0.1 ? dragAcc * vAirR / vAir : 0);
            s.vr += (accR + s.vt * s.vt / s.r - mu / (s.r * s.r)) * Dt;
            s.vt += (accT - s.vr * s.vt / s.r) * Dt;
            s.r += s.vr * Dt;
            if (s.r < launchR && s.vr < 0 && s.t < 30) { s.r = launchR; s.vr = 0; } // still on the pad
            s.t += Dt;

            double mNew = s.m - mdot * Dt;
            dvUsed += isp * G0 * Math.Log(s.m / Math.Max(mNew, ph.endMass));
            s.m = mNew;
            if (s.m <= ph.endMass)
            {
                if (++s.phase >= phases.Count) return false;
                s.m = phases[s.phase].startMass;
            }
            return true;
        }

        // Set to log the simulated trajectory every 10 s (Predict current), to line up with the
        // in-flight "[Stage1] flight" log.
        public bool trace;

        double traceElev;

        void Trace(State s, string stage, float thr)
        {
            if (!trace || (int)(s.t / Dt) % (int)(10 / Dt) != 0) return;
            double vAirT = s.vt - omegaCosLat * s.r * sinHTrace, vAirR = s.vr;
            Stage1Guidance.ApPe(mu, s.r, s.vr, Vt3(s.r, s.vt), out double ap, out double pe);
            L.Log($"[Stage1] sim T+{s.t:n0} {stage} alt {(s.r - R) / 1000:n1} km, v {Math.Sqrt(vAirT * vAirT + vAirR * vAirR):n0} m/s, " +
                  $"path {Math.Atan2(vAirR, vAirT) * 180 / Math.PI:n1}, cmd {traceElev:n1}, aoa {traceElev - Math.Atan2(vAirR, vAirT) * 180 / Math.PI:n1}, thr {thr:n2}, Ap {(ap - R) / 1000:n1} Pe {(pe - R) / 1000:n1}, mass {s.m:n1} t");
        }
        double sinHTrace;

        // ---- the velocity across the launch plane (2026-11-14). The sim is 2-D, in the launch
        // plane, but the ground spins under it: launching on heading H leaves omega*R*cos(lat)*cos(H)
        // of orbital velocity ACROSS the plane (175 m/s on Kerbin for a northward launch). The air
        // moves with the ground, so drag doesn't see it, but the orbit does: ignoring it made every
        // northward flight reach its target apoapsis later than real - low-TWR vessels cut off
        // 6-10° steeper, 7 s earlier and 170 m/s slower than predicted. Its angular momentum is
        // conserved, so the cross speed at radius r is crossH / r; apoapsis/periapsis use the full
        // horizontal speed.
        double crossH;
        void SetCross(double headingDeg) => crossH = omegaCosLat * launchR * Math.Cos(headingDeg * Math.PI / 180) * launchR;
        double Vt3(double r, double vt) { double c = crossH / r; return Math.Sqrt(vt * vt + c * c); }

        // ---------------------------------------------------------------- external controller

        /// <summary>What a controller sees each simulated step (plain numbers, SI, kPa).</summary>
        public struct Sample
        {
            public double t, altitude, r, vr, vt, apR, peR;
            public double airSpeed, pathDeg, qKPa, mach, heat;
            public double mass, thrustKN; // thrust at the last commanded throttle
            public int phase;
        }

        /// <summary>Returns false to stop; otherwise the pitch (deg above the horizon) and throttle to fly.</summary>
        public delegate bool Controller(in Sample s, out double elevDeg, out float throttle);

        /// <summary>
        /// Flies stage 1 (the model's stage-1 phases) under an external controller, so the New lift
        /// system can simulate its own arbiter, policy and shield rather than a copy of them. Stops
        /// when the controller says so, stage 1 runs dry, the vessel crashes or maxT passes.
        /// res.leftoverDv is what stage 1 has left at the stop; last is the final state.
        /// </summary>
        public SimResult RunControlled(Controller controller, double headingDeg, double maxT, out Sample last)
        {
            var res = new SimResult();
            double sinH = Math.Sin(headingDeg * Math.PI / 180);
            SetCross(headingDeg);
            var s = new State { r = launchR, vt = omegaCosLat * launchR * sinH, m = model.phases[0].startMass };
            double dvUsed = 0;
            float thr = 0;
            last = default;
            while (s.t < maxT)
            {
                var ph = model.phases[s.phase];
                Atmo(s.r - R, out double p, out double d, out double a);
                double vAirT = s.vt - omegaCosLat * s.r * sinH, vAirR = s.vr;
                double vAir = Math.Sqrt(vAirT * vAirT + vAirR * vAirR);
                Stage1Guidance.ApPe(mu, s.r, s.vr, Vt3(s.r, s.vt), out double apR, out double peR);
                double isp = ph.ispVac + (ph.ispASL - ph.ispVac) * Math.Min(1.0, p / 101.325);
                last = new Sample
                {
                    t = s.t, altitude = s.r - R, r = s.r, vr = s.vr, vt = s.vt, apR = apR, peR = peR,
                    airSpeed = vAir, pathDeg = vAir > 0.1 ? Math.Atan2(vAirR, vAirT) * 180 / Math.PI : 90,
                    qKPa = 0.0005 * d * vAir * vAir, mach = a > 0 ? vAir / a : 0, heat = Math.Sqrt(d) * vAir * vAir * vAir,
                    mass = s.m, thrustKN = ph.thrustVac * thr * isp / ph.ispVac * thrustScale, phase = s.phase,
                };
                if (!controller(last, out double elev, out thr)) break;
                if (!Step(ref s, model.phases, thr, elev, sinH, ref res, ref dvUsed)) { res.outOfFuel = true; break; }
                if (s.r < R - 50) { res.crashed = true; break; }
            }
            Stage1Guidance.ApPe(mu, s.r, s.vr, Vt3(s.r, s.vt), out double apEnd, out double peEnd);
            res.tBurnout = s.t;
            res.altKm = (s.r - R) / 1000;
            res.apKm = double.IsInfinity(apEnd) ? 1e9 : (apEnd - R) / 1000;
            res.peKm = (peEnd - R) / 1000;
            res.fpaDeg = Math.Atan2(s.vr, s.vt) * 180 / Math.PI;
            if (!res.outOfFuel && !res.crashed)
            {
                var ph = model.phases[s.phase];
                double left = ph.ispVac * G0 * Math.Log(Math.Max(s.m, ph.endMass) / ph.endMass);
                for (int i = s.phase + 1; i < model.phases.Count; i++) left += model.phases[i].deltaVVac;
                res.leftoverDv = left;
            }
            return res;
        }

    }
}
