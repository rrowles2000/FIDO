using System;
using System.Collections.Generic;
using KSP.Sim.impl;
using UnityEngine;

namespace K2D2.Lift.Stage1
{
    // The steering used once apoapsis has reached the target: keep apoapsis where it is and put
    // every bit of thrust into raising periapsis. Shared by the simulator and the real Ascent
    // controller so the optimised profile flies the same way it was simulated.
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

        // Elevation above the local horizontal (deg). Thrust goes perpendicular to the gradient of
        // apoapsis w.r.t. velocity (so apoapsis doesn't move) on the side that adds horizontal
        // speed (so periapsis rises), plus a small push along the gradient to remove any apoapsis
        // error.
        public static double HoldElevationDeg(double mu, double r, double vr, double vt, double targetApR)
        {
            const double dv = 1.0;
            if (!ApPe(mu, r, vr, vt, out double ap, out _)) return 0;
            ApPe(mu, r, vr + dv, vt, out double apRp, out _);
            ApPe(mu, r, vr - dv, vt, out double apRm, out _);
            ApPe(mu, r, vr, vt + dv, out double apTp, out _);
            ApPe(mu, r, vr, vt - dv, out double apTm, out _);
            if (double.IsInfinity(apRp) || double.IsInfinity(apTp)) return 0;
            double gr = (apRp - apRm) / (2 * dv), gt = (apTp - apTm) / (2 * dv);
            double gLen = Math.Sqrt(gr * gr + gt * gt);
            if (gLen < 1e-6) return 0;
            gr /= gLen; gt /= gLen;

            // perpendicular with a positive horizontal component
            double pt = gr, pr = -gt;
            if (pt < 0) { pt = -pt; pr = -pr; }

            // gentle: a few km of apoapsis error shouldn't swing the nose tens of degrees
            double c = Math.Max(-0.3, Math.Min(0.3, (targetApR - ap) / 10000.0));
            double dt = pt + c * gt, dr = pr + c * gr;
            double elev = Math.Atan2(dr, dt) * 180 / Math.PI;
            return Math.Max(-45, Math.Min(60, elev));
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

        public static double MaxAoADeg(double qKPa, double mach = 0)
        {
            if (qKPa > HighQKPa) return 0.5;
            return qKPa <= 0.04 ? 90 : Math.Max(2, Math.Min(90, 4 / qKPa));
        }

        public static double LimitAoA(double elevDeg, double airPathDeg, double qKPa, double mach = 0)
        {
            double max = MaxAoADeg(qKPa, mach);
            return Math.Max(airPathDeg - max, Math.Min(airPathDeg + max, elevDeg));
        }

        // While holding apoapsis, the AoA limit can stop the nose pitching down far enough (thick
        // air, still climbing steeply). Then apoapsis is controlled with the throttle instead:
        // full below the target, easing to zero 1 km above it, so stage 1 coasts up into thinner
        // air and burns horizontally there.
        public static float HoldThrottle(double holdElevDeg, double airPathDeg, double qKPa, double apR, double targetApR)
        {
            if (holdElevDeg >= airPathDeg - MaxAoADeg(qKPa) - 0.01) return 1f;
            return ApThrottle(apR, targetApR);
        }

        // Full below the target apoapsis, easing to zero 1 km above it.
        public static float ApThrottle(double apR, double targetApR) =>
            (float)Math.Max(0, Math.Min(1, (targetApR + 1000 - apR) / 3000));

        // Time (s) until apoapsis on the current orbit; 0 if past it or not elliptical.
        public static double TimeToApoapsis(double mu, double r, double vr, double vt)
        {
            double energy = (vr * vr + vt * vt) / 2 - mu / r;
            if (energy >= 0) return 0;
            double a = -mu / (2 * energy), h = r * vt;
            double e = Math.Sqrt(Math.Max(0, 1 + 2 * energy * h * h / (mu * mu)));
            if (e < 1e-6) return 0;
            double cosNu = Math.Max(-1, Math.Min(1, (h * h / (mu * r) - 1) / e));
            double nu = Math.Acos(cosNu);
            if (vr < 0) nu = 2 * Math.PI - nu;
            double E = 2 * Math.Atan(Math.Sqrt((1 - e) / (1 + e)) * Math.Tan(nu / 2));
            double M = E - e * Math.Sin(E);
            if (M < 0) M += 2 * Math.PI;
            double n = Math.Sqrt(mu / (a * a * a));
            double dM = Math.PI - M;
            return dM > 0 ? dM / n : 0;
        }

        // Burn time (s) for stage 1's apoapsis burn: enough to lift periapsis to the cap, or all
        // that's left in the stage, whichever is less.
        public static double ApoapsisBurnTime(double mu, double apR, double peR, double capPeR,
                                              double mass, double endMass, double thrustKN, double ispVac)
        {
            if (thrustKN <= 0 || mass <= endMass) return 0;
            double ve = ispVac * 9.80665;
            double vNow = Math.Sqrt(mu * (2 / apR - 2 / (apR + peR)));
            double vCap = Math.Sqrt(mu * (2 / apR - 2 / (apR + Math.Min(capPeR, apR))));
            double dv = Math.Min(Math.Max(0, vCap - vNow), ve * Math.Log(mass / endMass));
            double mdot = thrustKN / ve;
            return mass * (1 - Math.Exp(-dv / ve)) / mdot;
        }

        // Coast up to apoapsis and burn there (horizontal thrust raises Pe without moving Ap)
        // instead of burning at a steep negative pitch while still climbing. Only when apoapsis
        // is above the atmosphere; otherwise there's no free coast to have.
        public static bool CoastToApoapsis(double mu, double r, double vr, double vt, double apR, double peR,
                                           double atmosphereTopR, double capPeR, double mass, double endMass,
                                           double thrustKN, double ispVac)
        {
            if (apR <= atmosphereTopR || vr <= 0) return false;
            double tBurn = ApoapsisBurnTime(mu, apR, peR, capPeR, mass, endMass, thrustKN, ispVac);
            return TimeToApoapsis(mu, r, vr, vt) > tBurn / 2 + 1;
        }

        // Follow the profile as a flight-path target rather than a pitch: command past the target
        // by however far the path lags it. A gravity turn amplifies small early errors (a 0.2°
        // lean at liftoff moved the predicted upper-stage burn by ~65 m/s); following the path
        // steers them out, so the flight matches the simulation. pathDeg is signed in the launch
        // plane (above 90 = leaning back).
        public static double FollowPath(double profileDeg, double pathDeg) =>
            Math.Min(92, profileDeg + (profileDeg - pathDeg));

        // Same shape as LiftAscentPath.compute_elevation, with explicit parameters.
        public static double ProfileElevationDeg(double altKm, double startKm, double midKm, double endKm)
        {
            if (altKm < startKm) return 90;
            if (altKm < midKm)
                return Mathf.Lerp(90, 45, Mathf.InverseLerp((float)startKm, (float)midKm, (float)altKm));
            return Mathf.Lerp(45, 1, Mathf.InverseLerp((float)midKm, (float)endKm, (float)altKm));
        }
    }

    public struct SimParams
    {
        public int startKm;
        public float midRatio, endRatio;
        public float throttle;
        public double targetApKm, disposalPeKm, endAscentErrorKm;
        public double headingDeg;

        public double MidKm => Mathf.Lerp(startKm, (float)targetApKm, midRatio);
        public double EndKm => Mathf.Lerp(startKm, (float)targetApKm, endRatio);
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
        const double G0 = 9.80665;
        const double Dt = 0.2;

        readonly Stage1Model model;
        readonly double mu, R, depth, omegaCosLat, launchR;
        readonly double atmoStep = 100;
        readonly double[] pKPa, rho, sound;

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
            if (vAir > 5)
                elevDeg = Stage1Guidance.LimitAoA(elevDeg, Math.Atan2(vAirR, vAirT) * 180 / Math.PI, qPaNow / 1000,
                                                  a > 0 ? vAir / a : 0);
            double dragAcc = 0;
            if (d > 0 && vAir > 0.1 && model.dragValid)
            {
                double mach = a > 0 ? vAir / a : 0;
                double qPa = qPaNow;
                double aoa = vAir > 5 ? elevDeg - Math.Atan2(vAirR, vAirT) * 180 / Math.PI : 0;
                double pre = PhysicsSettings.DragCurvePseudoReynolds.Evaluate((float)(d * vAir));
                double dragKN = model.AreaDragAt(mach, aoa) * PhysicsSettings.DragCubeMultiplier * pre *
                                PhysicsSettings.DragMultiplier * qPa * 0.001;
                dragAcc = dragKN / s.m;
                res.maxQkPa = Math.Max(res.maxQkPa, qPa / 1000);
                double heat = Math.Sqrt(d) * vAir * vAir * vAir;
                if (heat > res.maxHeat) { res.maxHeat = heat; res.maxHeatAltKm = alt / 1000; }
            }

            double atm = Math.Min(1.0, p / 101.325);
            double isp = ph.ispVac + (ph.ispASL - ph.ispVac) * atm;
            double thrust = ph.thrustVac * throttle * isp / ph.ispVac;
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
            Stage1Guidance.ApPe(mu, s.r, s.vr, s.vt, out double ap, out double pe);
            L.Log($"[Stage1] sim T+{s.t:n0} {stage} alt {(s.r - R) / 1000:n1} km, v {Math.Sqrt(vAirT * vAirT + vAirR * vAirR):n0} m/s, " +
                  $"path {Math.Atan2(vAirR, vAirT) * 180 / Math.PI:n1}, cmd {traceElev:n1}, aoa {traceElev - Math.Atan2(vAirR, vAirT) * 180 / Math.PI:n1}, thr {thr:n2}, Ap {(ap - R) / 1000:n1} Pe {(pe - R) / 1000:n1}, mass {s.m:n1} t");
        }
        double sinHTrace;

        public SimResult Run(SimParams sp)
        {
            var res = new SimResult();
            double sinH = Math.Sin(sp.headingDeg * Math.PI / 180);
            sinHTrace = sinH;
            double targetApR = R + sp.targetApKm * 1000;
            double holdApR = R + (sp.targetApKm - sp.endAscentErrorKm) * 1000;
            double capPeR = R + sp.disposalPeKm * 1000;
            double midKm = sp.MidKm, endKm = sp.EndKm;
            float throttle = Mathf.Clamp(sp.throttle, 0.05f, 1f);

            var s = new State { r = launchR, vt = omegaCosLat * launchR * sinH, m = model.phases[0].startMass };
            bool hold = false, apBurn = false;
            var smoother = new Stage1Guidance.PitchSmoother();
            double atmoTopR = R + depth;
            double apR = s.r, peR = 0, dvStage1 = 0;

            // ---- stage 1: profile climb, then hold apoapsis and raise periapsis until it reaches
            // the cap or stage 1 runs dry
            while (s.t < 3500)
            {
                double elev;
                float thr = throttle;
                if (hold)
                {
                    elev = Stage1Guidance.HoldElevationDeg(mu, s.r, s.vr, s.vt, targetApR);
                    AirPath(s, sinH, out double path, out double qKPa);
                    var ph = model.phases[s.phase];
                    if (!apBurn && Stage1Guidance.CoastToApoapsis(mu, s.r, s.vr, s.vt, apR, peR, atmoTopR, capPeR,
                            s.m, model.phases[model.phases.Count - 1].endMass, ph.thrustVac * throttle, ph.ispVac))
                    {
                        thr = 0; // coast nose-prograde (as Coasting flies it), least drag
                        elev = path;
                    }
                    else
                    {
                        if (apR > atmoTopR) apBurn = true;
                        thr *= Stage1Guidance.HoldThrottle(elev, path, qKPa, apR, targetApR);
                    }
                }
                else
                {
                    elev = Stage1Guidance.ProfileElevationDeg((s.r - R) / 1000, sp.startKm, midKm, endKm);
                    AirPath(s, sinH, out double pathF, out _);
                    double vAirT = s.vt - omegaCosLat * s.r * sinH;
                    if (vAirT * vAirT + s.vr * s.vr > 100) elev = Stage1Guidance.FollowPath(elev, pathF);
                }
                {
                    AirPath(s, sinH, out double path2, out double q2, out double mach2);
                    if (q2 > 0 || s.t > 1) elev = Stage1Guidance.LimitAoA(elev, path2, q2, mach2);
                    elev = smoother.Next(elev, Dt, q2);
                }
                traceElev = elev;
                Trace(s, "S1", thr);
                if (!Step(ref s, model.phases, thr, elev, sinH, ref res, ref dvStage1)) { res.outOfFuel = true; }
                Stage1Guidance.ApPe(mu, s.r, s.vr, s.vt, out apR, out peR);
                if (res.outOfFuel) break;
                if (!hold && apR >= holdApR)
                {
                    hold = true;
                    res.reachedAp = true;
                    res.holdStartAltKm = (s.r - R) / 1000;
                }
                if (hold && peR >= capPeR) { res.capHit = true; break; }
                if (s.r < R - 50) { res.crashed = true; break; }
            }

            res.tBurnout = s.t;
            res.altKm = (s.r - R) / 1000;
            res.apKm = double.IsInfinity(apR) ? 1e9 : (apR - R) / 1000;
            res.peKm = (peR - R) / 1000;
            res.fpaDeg = Math.Atan2(s.vr, s.vt) * 180 / Math.PI;
            if (!res.outOfFuel)
            {
                var ph = model.phases[s.phase];
                double left = ph.ispVac * G0 * Math.Log(Math.Max(s.m, ph.endMass) / ph.endMass);
                for (int i = s.phase + 1; i < model.phases.Count; i++) left += model.phases[i].deltaVVac;
                res.leftoverDv = left;
            }
            if (res.crashed || double.IsInfinity(apR)) return res;

            // ---- upper stage: finish the climb on the profile if apoapsis is still short, then
            // coast to apoapsis and circularise there
            var u = new State { r = s.r, vr = s.vr, vt = s.vt, t = s.t, m = model.upperPhases[0].startMass };
            double climbDv = 0;
            bool upperOk = true;
            while (apR < holdApR)
            {
                double elev = Stage1Guidance.ProfileElevationDeg((u.r - R) / 1000, sp.startKm, midKm, endKm);
                Trace(u, "upper", 1f);
                if (!Step(ref u, model.upperPhases, 1f, elev, sinH, ref res, ref climbDv)) { upperOk = false; break; }
                Stage1Guidance.ApPe(mu, u.r, u.vr, u.vt, out apR, out peR);
                if (u.r < R - 50) { res.crashed = true; upperOk = false; break; }
                if (u.t > 4000) { upperOk = false; break; }
            }
            res.upperClimbDv = climbDv;
            if (upperOk && !double.IsInfinity(apR))
            {
                if (apR > targetApR + sp.endAscentErrorKm * 1000)
                {
                    // overshot: raise Pe to the target at Ap, then bring Ap down to it at the new Pe
                    double v1 = Math.Sqrt(mu * (2 / apR - 2 / (apR + targetApR))) - Math.Sqrt(mu * (2 / apR - 2 / (apR + peR)));
                    double v2 = Math.Sqrt(mu * (2 / targetApR - 2 / (apR + targetApR))) - Math.Sqrt(mu / targetApR);
                    res.circDv = Math.Max(0, v1) + Math.Max(0, v2);
                }
                else
                {
                    double sma = (apR + peR) / 2;
                    double vAp = Math.Sqrt(mu * (2 / apR - 1 / sma));
                    res.circDv = Math.Max(0, Math.Sqrt(mu / apR) - vAp);
                }
                res.upperMarginDv = model.UpperDeltaVVac - res.UpperDv;
                res.upperFeasible = res.upperMarginDv >= 0;
            }
            return res;
        }
    }

    // Search over the Lift profile (turn start, mid and end ratios) and optionally the thrust
    // limit, simulating each candidate.
    public static class Stage1Optimiser
    {
        public struct Outcome
        {
            public bool found;
            public SimParams best;
            public SimResult result;
            public int runs;
            public string note;
        }

        // Convective heating proxy sqrt(rho)*v^3 that an ascent should stay under. A rough
        // estimate (~1.8 km/s at 30 km on Kerbin is about here); not calibrated against KSP2's
        // thermal model yet.
        public const double HeatLimit = 5e8;

        // Lower is better: the upper stage's Δv from separation to a circular orbit at the target.
        static double Score(SimResult r)
        {
            if (r.crashed) return -1e7;
            // over the heating limit: heavily penalised, scaled so a smaller overshoot still wins
            double heatPenalty = r.maxHeat > HeatLimit ? 1e5 + (r.maxHeat / HeatLimit) * 1e4 : 0;
            if (!r.upperFeasible) return -1e6 + Math.Min(r.upperMarginDv, 0) - heatPenalty;
            return -r.UpperDv - 0.01 * r.maxQkPa - heatPenalty;
        }

        public static Outcome Run(Stage1Model model, SimParams baseParams, bool optimiseThrottle)
        {
            var sim = new Stage1Sim(model);
            var outcome = new Outcome();
            SimParams best = baseParams;
            SimResult bestRes = default;
            double bestScore = double.NegativeInfinity;

            // Don't let the turn go flat deep in the atmosphere: 45° no lower than 12% of the
            // atmosphere's height, horizontal no lower than 60% of it.
            double atmoKm = model.body.hasAtmosphere ? model.body.atmosphereDepth / 1000 : 0;
            double minMidKm = 0.12 * atmoKm, minEndKm = 0.6 * atmoKm;

            double Try(SimParams p)
            {
                if (p.endRatio <= p.midRatio + 0.02f) return double.NegativeInfinity;
                if (p.MidKm < minMidKm || p.EndKm < minEndKm) return double.NegativeInfinity;
                var r = sim.Run(p);
                outcome.runs++;
                r.score = Score(r);
                if (r.score > bestScore) { bestScore = r.score; best = p; bestRes = r; }
                return r.score;
            }

            float minThrottle = 1f;
            if (optimiseThrottle)
            {
                // keep at least 1.2 TWR at liftoff
                var ph = model.phases[0];
                double g = model.body.gravParameter / (model.body.radius * model.body.radius);
                double fullAsl = ph.thrustVac * ph.ispASL / ph.ispVac;
                minThrottle = Mathf.Clamp((float)(1.2 * ph.startMass * g / fullAsl), 0.2f, 1f);
            }

            int[] starts = { 1, 2, 3, 5, 8, 12 };
            float[] mids = { 0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.4f, 0.5f };
            float[] ends = { 0.3f, 0.45f, 0.55f, 0.65f, 0.75f, 0.85f, 0.95f, 1.0f };

            // 1) profile grid at full (or the given) thrust
            float gridThrottle = optimiseThrottle ? 1f : baseParams.throttle;
            foreach (var s in starts)
            foreach (var mi in mids)
            foreach (var en in ends)
            {
                var p = baseParams;
                p.startKm = s; p.midRatio = mi; p.endRatio = en; p.throttle = gridThrottle;
                Try(p);
            }
            Refine(best, 4);

            // 2) thrust limit: less thrust means less drag and heating but more gravity loss -
            // try a few settings, each with the profile re-refined
            if (optimiseThrottle && minThrottle < 0.999f)
            {
                var fullThrust = best;
                foreach (var thr in new[] { 0.9f, 0.8f, 0.7f, 0.6f })
                {
                    if (thr < minThrottle) break;
                    var p = fullThrust;
                    p.throttle = thr;
                    Refine(p, 2);
                }
            }

            // Local search around `start` (its thrust limit stays fixed); the overall best is
            // tracked by Try.
            void Refine(SimParams start, int iterations)
            {
                var centre = start;
                double centreScore = Try(start);
                float stepMid = 0.04f, stepEnd = 0.08f;
                for (int iter = 0; iter < iterations; iter++)
                {
                    var from = centre;
                    for (int ds = -1; ds <= 1; ds++)
                    for (int dm = -1; dm <= 1; dm++)
                    for (int de = -1; de <= 1; de++)
                    {
                        var p = from;
                        p.startKm = Math.Max(1, from.startKm + ds);
                        p.midRatio = Mathf.Clamp(from.midRatio + dm * stepMid, 0.01f, 0.9f);
                        p.endRatio = Mathf.Clamp(from.endRatio + de * stepEnd, 0.05f, 1f);
                        double sc = Try(p);
                        if (sc > centreScore) { centreScore = sc; centre = p; }
                    }
                    stepMid *= 0.6f; stepEnd *= 0.6f;
                }
            }

            outcome.found = true;
            outcome.best = best;
            outcome.result = bestRes;
            if (optimiseThrottle && minThrottle >= 0.999f)
                outcome.note = "thrust can't be lowered (liftoff TWR would drop below 1.2)";
            L.Log($"[Stage1] optimiser: {outcome.runs} runs, start {best.startKm} km, mid {best.midRatio:n2}, " +
                  $"end {best.endRatio:n2}, thr {best.throttle:n2} -> upper Δv {bestRes.UpperDv:n0} (climb {bestRes.upperClimbDv:n0} + circ {bestRes.circDv:n0}), " +
                  $"margin {bestRes.upperMarginDv:n0}, S1 cut-off Ap {bestRes.apKm:n1} Pe {bestRes.peKm:n1} km cap {bestRes.capHit}, maxQ {bestRes.maxQkPa:n1} kPa, " +
                  $"heat {bestRes.maxHeat / HeatLimit * 100:n0}% at {bestRes.maxHeatAltKm:n1} km");
            return outcome;
        }
    }
}
