using KSP.Sim.impl;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Lift
{
    /// <summary>
    /// [Planet] one log line per run describing the planet of origin, as the game reports it, so
    /// the planet-relative limits planned for the New lift system (see the NN ascent report,
    /// Section 4.4) are derived from real KSP2 values rather than remembered Kerbin figures.
    /// Logging only - never throws into the caller.
    /// </summary>
    public static class PlanetLog
    {
        public static void Log(ILogger logger, VesselComponent vessel_in, float target_ap_km)
        {
            try
            {
                var vessel = vessel_in;
                var body = vessel?.mainBody;
                if (body == null) return;

                double mu = body.gravParameter, R = body.radius;
                double g0 = mu / (R * R);
                double target_r = R + target_ap_km * 1000.0;
                double v_orbit = System.Math.Sqrt(mu / target_r);
                double v_eq = K2D2.Constellation.OrbitMath.EquatorialSurfaceSpeed(body);
                double lat = vessel.Latitude;
                double v_site = v_eq * System.Math.Cos(lat * System.Math.PI / 180.0);

                var sb = new System.Text.StringBuilder();
                sb.Append($"[Planet] {body.Name}: mu {mu:e4} m³/s², R {R / 1000:n1} km, g0 {g0:n3} m/s², " +
                          $"rotation {body.rotationPeriod:n0} s{(body.hasInverseRotation ? " (inverse)" : "")}, " +
                          $"equator {v_eq:n1} m/s, site lat {lat:n3}° ({v_site:n1} m/s), " +
                          $"launch alt {vessel.AltitudeFromSeaLevel:n0} m, " +
                          $"target {target_ap_km} km circular {v_orbit:n0} m/s " +
                          $"(g there {mu / (target_r * target_r) / g0 * 100:n0}% of g0)");

                if (!body.hasAtmosphere)
                {
                    sb.Append(", no atmosphere");
                }
                else
                {
                    double depth = body.atmosphereDepth;
                    sb.Append($", atmosphere {depth / 1000:n1} km, p0 {body.atmospherePressureSeaLevel:n3} kPa, " +
                              $"gamma {body.atmosphereAdiabaticIndex:n3}; alt km: rho kg/m³ / T K / sound m/s / H km");
                    // density profile with the local scale height between samples
                    double prev_alt = -1, prev_rho = 0;
                    foreach (double km in new double[] { 0, 2, 5, 10, 15, 20, 30, 40, 50, 60 })
                    {
                        double alt = km * 1000;
                        if (alt >= depth) break;
                        double p = body.GetPressure(alt);
                        double t = body.GetTemperature(alt);
                        double rho = body.GetDensity(p, t);
                        double a = rho > 0 ? AeroUtilities.GetSoundSpeed(body.atmosphereAdiabaticIndex, p, rho) : 0;
                        string h = prev_alt >= 0 && rho > 0 && prev_rho > rho
                            ? $"{(alt - prev_alt) / System.Math.Log(prev_rho / rho) / 1000:n2}" : "-";
                        sb.Append($" | {km:n0}: {rho:e3} / {t:n0} / {a:n0} / {h}");
                        prev_alt = alt;
                        prev_rho = rho;
                    }
                }
                logger.LogInfo(sb.ToString());
            }
            catch (System.Exception e)
            {
                logger.LogWarning("[Planet] could not describe the planet of origin: " + e.Message);
            }
        }
    }
}
