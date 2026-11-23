using System;
using System.Collections.Generic;
using System.Linq;
using K2D2.Controller;
using K2D2.KSPService;
using K2D2.Landing;
using K2D2.Node;
using KSP.Sim;
using KSP.Sim.impl;
using KSP.Sim.Maneuver;
using KTools;
using KSP2FlightAssistant.MathLibrary;
using UnityEngine;
using ILogger = ReduxLib.Logging.ILogger;

namespace K2D2.Constellation
{
    public class ConstellationSettings
    {
        // 0 in any of these means "use the suggested value"
        public Setting<float> target_altitude_km = new("constellation.target_altitude_km", 0);
        public Setting<float> separation_deg = new("constellation.separation_deg", 0);
        public Setting<int> phasing_orbits = new("constellation.phasing_orbits", 0);

        public ClampSetting<float> reserve_pc = new("constellation.reserve_pc", 10, 0, 50);
        // new key so the old, switched-off "activate_dish" value doesn't carry over
        public Setting<bool> auto_dish = new("constellation.auto_dish", true);
        public Setting<bool> move_fuel = new("constellation.move_fuel", true);
        // burn acceleration cap (m/s²) - full thrust on the lightened carrier makes it wobble
        public ClampSetting<float> max_accel = new("constellation.max_accel", 5, 1, 30);

        // how close (km) both apsides must already be to skip the altitude change
        public ClampSetting<float> altitude_tolerance_km = new("constellation.altitude_tolerance_km", 1, 0.1f, 50);
    }

    /// <summary>
    /// Deploys the satellites of a dispenser vessel into one evenly-phased ring: move to the survey
    /// altitude, then for each satellite point radial-out, hand its leftover propellant to the
    /// carrier, switch its dish on, release it and phase the carrier back by the separation angle.
    /// The carrier itself becomes the last satellite.
    /// </summary>
    public class ConstellationPilot : Pilot
    {
        public static ConstellationPilot Instance { get; private set; }

        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Constellation");

        public ConstellationSettings settings = new();

        KSPVessel current_vessel;
        ManeuverCreator maneuver_creator = new();

        // release plan for the current vessel (UI edits order and enabled flags)
        public List<SatelliteSlot> satellites = new();
        public PartComponent carrier_command;
        public List<PartComponent> carrier_parts = new();
        VesselComponent scanned_vessel;
        int scanned_parts = -1;

        static int PartCount(VesselComponent vessel) => vessel?.SimulationObject?.PartOwner?.Parts?.Count() ?? -1;

        ConstellationUI ui;

        public ConstellationPilot()
        {
            Instance = this;
            debug_mode_only = false;
            current_vessel = K2D2_Plugin.Instance.current_vessel;
            _page = ui = new ConstellationUI(this);
            K2D2PilotsMgr.Instance.RegisterPilot("Constellation", this);
        }

        // ================================================================ planning values (UI)

        public CelestialBodyComponent Body => current_vessel?.VesselComponent?.mainBody;

        public int EnabledCount => satellites.Count(s => s.enabled && !s.released);

        public string suggestion_source = "";

        public double SuggestedAltitude(out double min, out double max)
        {
            min = max = 0;
            var body = Body;
            if (body == null) return 0;
            suggestion_source = "Orbital Survey dish";
            if (!ConstellationVessel.SurveyAltitudes(current_vessel.VesselComponent, out min, out double ideal, out max))
            {
                ConstellationVessel.DefaultAltitudes(body, out min, out ideal, out max);
                suggestion_source = "body size";
            }
            return ideal;
        }

        public double TargetAltitude
        {
            get
            {
                if (settings.target_altitude_km.V > 0) return settings.target_altitude_km.V * 1000.0;
                return SuggestedAltitude(out _, out _);
            }
        }

        // number of spacecraft in the ring = released satellites + the carrier
        public double SuggestedSeparation => 360.0 / Math.Max(1, EnabledCount + 1);

        // fixed when Start is pressed - EnabledCount drops as satellites leave, which must not widen the spacing
        double planned_separation = -1;

        public double Separation => planned_separation > 0 ? planned_separation
            : settings.separation_deg.V > 0 ? settings.separation_deg.V : SuggestedSeparation;

        public double AvailableDv => current_vessel?.VesselComponent?.VesselDeltaV?.TotalDeltaVVac ?? 0;

        /// <summary>Fraction (0-1) of the current propellant a burn of dv would use.</summary>
        public double FuelFraction(double dv)
        {
            var vessel = current_vessel?.VesselComponent;
            double total = AvailableDv;
            if (vessel == null || total <= 0) return double.NaN;
            // rocket equation with the exhaust velocity implied by the game's own dv figure
            double m0 = vessel.totalMass;
            double prop = PropellantMass(vessel);
            if (prop <= 0 || prop >= m0) return dv / total;
            double ve = total / Math.Log(m0 / (m0 - prop));
            return m0 * (1 - Math.Exp(-dv / ve)) / prop;
        }

        static double PropellantMass(VesselComponent vessel)
        {
            var db = GeneralTools.Game.ResourceDefinitionDatabase;
            var ec = db.GetResourceIDFromName("ElectricCharge");
            double mass = 0;
            foreach (var p in vessel.SimulationObject.PartOwner.Parts)
            {
                var c = p.PartResourceContainer;
                if (c == null) continue;
                foreach (var d in c.GetAllResourcesContainedData())
                    if (d.ResourceID != ec)
                        mass += c.GetResourceStoredMass(d.ResourceID);
            }
            return mass;
        }

        bool GetState(out Vector3d r, out Vector3d v, out double mu)
        {
            r = v = Vector3d.zero;
            mu = 0;
            var orbit = current_vessel?.VesselComponent?.Orbit;
            if (orbit == null) return false;
            double now = GeneralTools.Current_UT;
            r = orbit.GetRelativePositionAtUTZup(now);
            v = orbit.GetOrbitalVelocityAtUTZup(now);
            mu = orbit.referenceBody.gravParameter;
            return true;
        }

        public double AltitudeChangeDv()
        {
            if (!GetState(out var r, out var v, out double mu)) return 0;
            LandingTargeting.OrbitalElementsFromStateVectors(r, v, mu, out _, out double ap, out double pe);
            double target = Body.radius + TargetAltitude;
            double tol = settings.altitude_tolerance_km.V * 1000.0;
            if (Math.Abs(ap - target) < tol && Math.Abs(pe - target) < tol) return 0;
            return OrbitMath.CircularizeToDv(mu, pe, ap, target);
        }

        public double PhasingDv(int n)
        {
            var body = Body;
            if (body == null) return 0;
            double r = body.radius + TargetAltitude;
            double minR = body.radius + (body.hasAtmosphere ? body.atmosphereDepth : 0) + 10000;
            double dv = OrbitMath.SafePhasingBurn(body.gravParameter, r, Separation, n, minR, out _, out _);
            return 2.0 * Math.Abs(dv);
        }

        /// <summary>Orbits to coast per phasing: fixed override, or fewest that fit the dv budget with the reserve kept.</summary>
        public int PhasingOrbits(out bool fits)
        {
            int sats = EnabledCount;
            double budget = AvailableDv * (1 - settings.reserve_pc.V / 100.0) - AltitudeChangeDv();
            if (settings.phasing_orbits.V > 0)
            {
                fits = sats * PhasingDv(settings.phasing_orbits.V) <= budget;
                return settings.phasing_orbits.V;
            }
            for (int n = 1; n <= 30; n++)
                if (sats * PhasingDv(n) <= budget)
                {
                    fits = true;
                    return n;
                }
            fits = false;
            return 30;
        }

        public double TotalMissionDv()
        {
            int n = PhasingOrbits(out _);
            return AltitudeChangeDv() + EnabledCount * PhasingDv(n);
        }

        public double MissionDuration()
        {
            var body = Body;
            if (body == null) return 0;
            int n = PhasingOrbits(out _);
            double r = body.radius + TargetAltitude;
            OrbitMath.SafePhasingBurn(body.gravParameter, r, Separation, n,
                body.radius + body.atmosphereDepth + 10000, out double tph, out _);
            return EnabledCount * n * tph;
        }

        // ---------------------------------------------------------------- polar

        public bool PolarEstimate(out double dv, out double inclination, out string problem, double target_inc = 90)
        {
            dv = 0;
            inclination = 0;
            problem = null;
            var orbit = current_vessel?.VesselComponent?.Orbit;
            if (orbit == null) { problem = "No vessel"; return false; }
            if (!GetState(out var r, out var v, out double mu)) { problem = "No orbit"; return false; }

            inclination = orbit.inclination;
            double stateInc = OrbitMath.InclinationDeg(r, v);
            if (Math.Abs(stateInc - inclination) > 1.0)
            {
                problem = $"Orbit frame check failed ({stateInc:n1}° vs {inclination:n1}°)";
                return false;
            }
            LandingTargeting.OrbitalElementsFromStateVectors(r, v, mu, out double sma, out double ap, out double pe);
            if (double.IsNaN(sma) || sma <= 0) { problem = "Not in a closed orbit"; return false; }

            // speed at the node of a near-circular orbit
            dv = OrbitMath.PlaneChangeDv(OrbitMath.CircularSpeed(mu, sma), target_inc - inclination);
            return true;
        }

        // ================================================================ vessel scan

        public void Rescan()
        {
            var vessel = current_vessel?.VesselComponent;
            scanned_vessel = vessel;
            scanned_parts = PartCount(vessel);
            satellites = ConstellationVessel.FindSatellites(vessel);
            carrier_command = vessel == null ? null : ConstellationVessel.FindCarrierCommand(vessel, out carrier_parts);
            ui?.RebuildOrder();
        }

        public void Move(SatelliteSlot slot, int delta)
        {
            int i = satellites.IndexOf(slot);
            int j = i + delta;
            if (i < 0 || j < 0 || j >= satellites.Count) return;
            satellites[i] = satellites[j];
            satellites[j] = slot;
            ui?.RebuildOrder();
        }

        // ================================================================ run

        public enum Job { None, Deploy, Polar, Altitude }
        public Job job = Job.None;

        readonly Queue<Step> steps = new();
        Step current_step;
        public string step_status = "";
        public string end_status = "";
        public bool result_ok = true;
        public int released_count;

        double reference_sma;
        bool expecting_switch;
        int phasing_n;

        public override bool isRunning
        {
            get => job != Job.None;
            set
            {
                if (value == isRunning) return;
                if (!value)
                {
                    if (job == Job.Deploy) Pause("Paused");
                    else Abort("Stopped");
                }
                base.isRunning = value;
            }
        }

        public void StartDeploy()
        {
            if (isRunning) return;
            if (paused != null)
            {
                Resume();
                return;
            }
            Rescan();
            var plan = satellites.Where(s => s.enabled).ToList();
            if (plan.Count == 0 || carrier_command == null)
            {
                Finish(false, carrier_command == null ? "No engine/command carrier found" : "No satellites to release");
                return;
            }

            K2D2_Plugin.ResetControllers();
            planned_separation = -1;
            phasing_n = PhasingOrbits(out bool fits);
            planned_separation = Separation;
            if (!fits)
                logger.LogWarning("[Constellation] dv budget doesn't cover the plan - flying it anyway, watch the fuel");

            steps.Clear();
            steps.Enqueue(new ActionStep("Taking control from the carrier", () =>
                ConstellationVessel.SetControlFrom(current_vessel.VesselComponent, carrier_command)));
            steps.Enqueue(new NodeStep(this, "Raise/lower to survey altitude", PlanAltitudeBurn1));
            steps.Enqueue(new NodeStep(this, "Correct the transfer orbit", PlanAltitudeCorrection));
            steps.Enqueue(new NodeStep(this, "Circularize at survey altitude", PlanAltitudeBurn2));
            steps.Enqueue(new ActionStep("Recording reference orbit", RecordReference));

            foreach (var sat in plan)
            {
                var s = sat;
                // every satellite inherits the carrier's period at release (plus the decoupler's kick),
                // so set the carrier to the reference minus the kick measured on earlier releases
                steps.Enqueue(new PeriodTrimStep(this, "Trim period for release", () => ReferencePeriod - KickEstimate));
                // the dish goes on while the satellite is still attached
                if (settings.auto_dish.V)
                    steps.Enqueue(new ActionStep("Switching the dish on", () => ActivateDish(s)));
                steps.Enqueue(new AlignStep(this, AutopilotMode.RadialOut, "Pointing radial-out for release"));
                if (settings.move_fuel.V)
                    steps.Enqueue(new ActionStep("Moving leftover fuel to the carrier", () => MoveFuel(s)));
                steps.Enqueue(new ActionStep("Releasing satellite", () => Release(s)));
                steps.Enqueue(new WaitStep(3, "Separating"));
                steps.Enqueue(new ActionStep("Back to the carrier", () => ConstellationVessel.EnsureActive(carrier_command)));
                steps.Enqueue(new WaitStep(2, "Settling"));
                steps.Enqueue(new ActionStep("Carrier in control", () => expecting_switch = false));
                steps.Enqueue(new ActionStep("Measuring release kick", () => LearnKick(s)));
                steps.Enqueue(new ActionStep("Naming satellite", () => NameSatellite(s)));
                steps.Enqueue(new NodeStep(this, "Phasing burn", PlanPhasing1));
                steps.Enqueue(new NodeStep(this, "Return to reference orbit", PlanPhasing2));
            }
            steps.Enqueue(new PeriodTrimStep(this, "Final period trim", () => ReferencePeriod));
            if (settings.auto_dish.V)
                steps.Enqueue(new ActionStep("Switching the carrier's dish on", () =>
                    ConstellationVessel.ActivateDishes(carrier_parts)));
            steps.Enqueue(new ActionStep("Naming the carrier", () =>
            {
                Rename(carrier_command.PartOwner?.SimulationObject?.Vessel, $"{base_name} S{released_count + 1}");
                RefreshBeams();
            }));

            released_count = 0;
            kicks.Clear();
            base_name = current_vessel.VesselComponent.Name ?? "Sat";
            if (base_name.StartsWith("Undocked: ")) base_name = base_name.Substring(10);
            end_status = "";
            job = Job.Deploy;
            base.isRunning = true;
            NextStep();
        }

        // inclination the plane change aims for: 90 from the Sats tab's polar button, anything from a mission step
        public double plane_target = 90;

        public void StartPolar() => StartPlaneChange(90);

        public void StartPlaneChange(double target_inc)
        {
            if (isRunning) return;
            plane_target = target_inc;
            Rescan();
            if (!PolarEstimate(out _, out double inc, out string problem))
            {
                Finish(false, problem);
                return;
            }
            if (Math.Abs(plane_target - inc) < 0.1)
            {
                Finish(true, $"Already at {plane_target:0.#}°");
                return;
            }
            K2D2_Plugin.ResetControllers();
            steps.Clear();
            var polar = new NodeStep(this, $"Plane change to {plane_target:0.#}°", PlanPolar);
            polar.verify = () => VerifyPolar(polar.NodeUT);
            steps.Enqueue(polar);
            end_status = "";
            job = Job.Polar;
            base.isRunning = true;
            NextStep();
        }

        /// <summary>Hohmann transfer to a circular orbit at the target altitude (settings.target_altitude_km).</summary>
        public void StartAltitude()
        {
            if (isRunning) return;
            Rescan();
            if (current_vessel?.VesselComponent?.Orbit == null)
            {
                Finish(false, "No vessel");
                return;
            }
            K2D2_Plugin.ResetControllers();
            steps.Clear();
            steps.Enqueue(new NodeStep(this, "Raise/lower to target altitude", PlanAltitudeBurn1));
            steps.Enqueue(new NodeStep(this, "Correct the transfer orbit", PlanAltitudeCorrection));
            steps.Enqueue(new NodeStep(this, "Circularize at target altitude", PlanAltitudeBurn2));
            end_status = "";
            job = Job.Altitude;
            base.isRunning = true;
            NextStep();
        }

        void NextStep()
        {
            if (steps.Count == 0)
            {
                Finish(true, job == Job.Polar ? $"Inclination {plane_target:0.#}° reached"
                    : job == Job.Altitude ? $"Circular at {TargetAltitude / 1000:n0} km"
                    : $"Constellation deployed ({released_count} released)");
                return;
            }
            current_step = steps.Dequeue();
            step_status = current_step.name;
            logger.LogInfo("[Constellation] step: " + current_step.name);
            current_step.Start();
        }

        void Finish(bool ok, string status)
        {
            result_ok = ok;
            end_status = status;
            logger.LogInfo($"[Constellation] {(ok ? "done" : "stopped")}: {status}");
            Cleanup();
        }

        void Abort(string why)
        {
            if (job == Job.None) return;
            Finish(false, why + " at: " + step_status);
        }

        void Cleanup()
        {
            current_step?.Stop();
            current_step = null;
            steps.Clear();
            expecting_switch = false;
            planned_separation = -1;
            job = Job.None;
            current_vessel?.SetThrottle(0);
            base.isRunning = false;
        }

        public override void onReset()
        {
            if (job == Job.None) return;
            // K2D2 resets every pilot on any vessel switch (a release, or the game refocusing a
            // satellite on leaving map view). Keep going and take the carrier back in Update;
            // only leaving flight pauses the run.
            if (InFlight()) return;
            if (job == Job.Deploy) Pause("Left flight - paused");
            else Abort("Left flight");
        }

        static bool InFlight()
        {
            try
            {
                var state = GeneralTools.Game.GlobalGameState.GetGameState().GameState;
                return state == KSP.Game.GameState.FlightView || state == KSP.Game.GameState.Map3DView;
            }
            catch (Exception) { return false; }
        }

        // ---------------------------------------------------------------- pause / resume

        List<Step> paused;
        public bool IsPaused => paused != null;

        void Pause(string why)
        {
            if (job != Job.Deploy) return;
            current_step?.Stop();
            paused = new List<Step>();
            if (current_step != null) paused.Add(current_step);
            paused.AddRange(steps);
            string at = step_status;
            current_step = null;
            steps.Clear();
            job = Job.None;
            current_vessel?.SetThrottle(0);
            TimeWarpTools.SetRateIndex(0, false);
            result_ok = false;
            end_status = $"{why} at: {at} - press Resume to carry on";
            logger.LogInfo("[Constellation] " + end_status);
            base.isRunning = false;
        }

        void Resume()
        {
            steps.Clear();
            foreach (var step in paused) steps.Enqueue(step);
            paused = null;
            end_status = "";
            job = Job.Deploy;
            expecting_switch = false;
            base.isRunning = true;
            logger.LogInfo("[Constellation] resumed");
            ConstellationVessel.EnsureActive(carrier_command);
            NextStep();
        }

        public void DiscardPaused()
        {
            paused = null;
            planned_separation = -1;
            end_status = "Paused plan discarded";
            Rescan();
        }

        float next_reclaim;

        void ReclaimCarrier()
        {
            if (expecting_switch || carrier_command == null || Time.time < next_reclaim) return;
            next_reclaim = Time.time + 1;
            var carrier = carrier_command.PartOwner?.SimulationObject?.Vessel;
            if (carrier != null && current_vessel.VesselComponent != carrier)
                ConstellationVessel.EnsureActive(carrier_command);
        }

        public override void Update()
        {
            // rescan on a vessel switch, and when staging changes the parts (e.g. stage 1 dropped)
            var vc = current_vessel?.VesselComponent;
            if (vc != null && !isRunning && paused == null && (scanned_vessel != vc || scanned_parts != PartCount(vc)))
                Rescan();

            if (current_step == null) return;
            if (job == Job.Deploy) ReclaimCarrier();
            try
            {
                if (current_step.Update())
                    NextStep();
            }
            catch (Exception e)
            {
                logger.LogError("[Constellation] " + e);
                Finish(false, e.Message);
            }
        }

        public string CurrentStepStatus => current_step?.status ?? "";

        // ================================================================ step actions

        void ActivateDish(SatelliteSlot sat)
        {
            int on = ConstellationVessel.ActivateDishes(sat.parts);
            if (sat.HasDish && on == 0)
                throw new Exception("Couldn't switch this satellite's dish on - not releasing it");
        }

        void MoveFuel(SatelliteSlot sat)
        {
            var remaining = satellites.Where(s => s != sat && !s.released).SelectMany(s => s.parts);
            ConstellationVessel.MoveFuel(sat, carrier_parts, remaining, out double moved, out double left);
            logger.LogInfo($"[Constellation] moved {moved:n3} units to the carrier, {left:n3} left aboard (no room)");
        }

        void Release(SatelliteSlot sat)
        {
            ConstellationVessel.SetControlFrom(current_vessel.VesselComponent, carrier_command);
            pre_release_period = CurrentPeriod();
            expecting_switch = true;
            if (!ConstellationVessel.Decouple(sat.decoupler))
                throw new Exception("Decoupler didn't fire");
            sat.released = true;
            released_count++;
        }

        string base_name = "Sat";

        // identical names make the satellites indistinguishable in Orbital Survey's window and the map
        void NameSatellite(SatelliteSlot sat)
        {
            Rename(sat.command?.PartOwner?.SimulationObject?.Vessel, $"{base_name} S{released_count}");
            RefreshBeams();
        }

        void RefreshBeams() =>
            ConstellationVessel.RefreshSurveyRegistration(satellites.SelectMany(s => s.parts).Concat(carrier_parts));

        void Rename(VesselComponent vessel, string name)
        {
            if (vessel == null) return;
            vessel.SimulationObject.Name = name;
            logger.LogInfo("[Constellation] named " + name);
        }

        double pre_release_period;
        readonly List<double> kicks = new();

        double KickEstimate => kicks.Count == 0 ? 0 : kicks.Average();

        double ReferencePeriod
        {
            get
            {
                GetState(out _, out _, out double mu);
                return OrbitMath.Period(mu, reference_sma);
            }
        }

        double CurrentPeriod()
        {
            GetState(out var r, out var v, out double mu);
            return LandingTargeting.OrbitalPeriodFromStateVectors(r, v, mu);
        }

        /// <summary>How much the release changed the satellite's period - fed back into the next release's trim.</summary>
        void LearnKick(SatelliteSlot sat)
        {
            var orbit = sat.command?.PartOwner?.SimulationObject?.Vessel?.Orbit;
            if (orbit == null) return;
            double now = GeneralTools.Current_UT;
            double sat_period = LandingTargeting.OrbitalPeriodFromStateVectors(orbit.GetRelativePositionAtUTZup(now),
                orbit.GetOrbitalVelocityAtUTZup(now), orbit.referenceBody.gravParameter);
            double kick = sat_period - pre_release_period;
            // ignore anything wild (a collision or a missed reading) rather than skew the average
            if (Math.Abs(kick) < 30) kicks.Add(kick);
            logger.LogInfo($"[Constellation] satellite period {sat_period:n2} s (reference {ReferencePeriod:n2} s), release kick {kick:+0.00;-0.00} s, average {KickEstimate:+0.00;-0.00} s");
        }

        void RecordReference()
        {
            GetState(out var r, out var v, out double mu);
            LandingTargeting.OrbitalElementsFromStateVectors(r, v, mu, out reference_sma, out _, out _);
            logger.LogInfo($"[Constellation] reference sma {reference_sma:n0} m, period {OrbitMath.Period(mu, reference_sma):n1} s, phasing {phasing_n} orbit(s), separation {Separation:n1}°");
        }

        // ---------------------------------------------------------------- burn plans

        public class BurnPlan
        {
            public double UT, prograde, normal;
        }

        const double Lead = 120;

        BurnPlan PlanAltitudeBurn1()
        {
            if (AltitudeChangeDv() <= 0) return null;
            GetState(out var r, out var v, out double mu);
            double period = LandingTargeting.OrbitalPeriodFromStateVectors(r, v, mu);
            LandingTargeting.OrbitalElementsFromStateVectors(r, v, mu, out _, out double ap, out double pe);
            double target = Body.radius + TargetAltitude;
            double now = GeneralTools.Current_UT;

            double dt = target <= pe
                ? LandingTargeting.TimeToNextApoapsis(r, v, mu, period)
                : LandingTargeting.TimeToNextPeriapsis(r, v, mu, period);
            if (dt < Lead) dt += period;

            KeplerPropagator.Propagate(r, v, mu, dt, out var rb, out var vb);
            double newSma = (rb.magnitude + target) / 2.0;
            return new BurnPlan { UT = now + dt, prograde = OrbitMath.SpeedAt(mu, rb.magnitude, newSma) - vb.magnitude };
        }

        /// <summary>
        /// After the raise/lower burn (2026-11-23): if the far apsis missed the target altitude by more
        /// than max(tolerance, 3%), burn again at the near apsis (PlanAltitudeBurn1 from the new orbit)
        /// instead of circularising at the wrong altitude. A burn that stopped half done left Ap at
        /// 314 km for an 800 km target and the constellation was deployed there.
        /// </summary>
        BurnPlan PlanAltitudeCorrection()
        {
            GetState(out var r, out var v, out double mu);
            LandingTargeting.OrbitalElementsFromStateVectors(r, v, mu, out _, out double ap, out double pe);
            double target = Body.radius + TargetAltitude;
            double tol = Math.Max(settings.altitude_tolerance_km.V * 1000.0, 0.03 * TargetAltitude);
            // raising: the far apsis is Ap; lowering: it is Pe
            double far = Math.Abs(ap - target) < Math.Abs(pe - target) ? ap : pe;
            if (Math.Abs(far - target) <= tol) return null;
            logger.LogWarning($"[Constellation] transfer orbit misses the target: far apsis {(far - Body.radius) / 1000:n1} km for {TargetAltitude / 1000:n0} km - correcting");
            return PlanAltitudeBurn1();
        }

        BurnPlan PlanAltitudeBurn2()
        {
            GetState(out var r, out var v, out double mu);
            LandingTargeting.OrbitalElementsFromStateVectors(r, v, mu, out _, out double ap, out double pe);
            double target = Body.radius + TargetAltitude;
            double tol = settings.altitude_tolerance_km.V * 1000.0;
            if (Math.Abs(ap - target) < tol && Math.Abs(pe - target) < tol) return null;

            double period = LandingTargeting.OrbitalPeriodFromStateVectors(r, v, mu);
            // circularize at whichever apsis sits at the target
            bool atAp = Math.Abs(ap - target) < Math.Abs(pe - target);
            double dt = atAp ? LandingTargeting.TimeToNextApoapsis(r, v, mu, period)
                             : LandingTargeting.TimeToNextPeriapsis(r, v, mu, period);
            if (dt < Lead) dt += period;

            KeplerPropagator.Propagate(r, v, mu, dt, out var rb, out var vb);
            return new BurnPlan
            {
                UT = GeneralTools.Current_UT + dt,
                prograde = OrbitMath.CircularSpeed(mu, rb.magnitude) - vb.magnitude,
            };
        }

        double phasing_start_UT, phasing_period;

        BurnPlan PlanPhasing1()
        {
            GetState(out var r, out var v, out double mu);
            double dt = Lead;
            KeplerPropagator.Propagate(r, v, mu, dt, out var rb, out var vb);

            double T = OrbitMath.Period(mu, reference_sma);
            double sep = Separation;
            double minR = Body.radius + (Body.hasAtmosphere ? Body.atmosphereDepth : 0) + 10000;

            // same choice as the estimate, but sized from the recorded reference period
            double behind = T * (1.0 + sep / (360.0 * phasing_n));
            double ahead = T * (1.0 - (360.0 - sep) / (360.0 * phasing_n));
            double rbm = rb.magnitude;
            double dvBehind = OrbitMath.SpeedAt(mu, rbm, OrbitMath.SmaFromPeriod(mu, behind)) - vb.magnitude;
            double dvAhead = double.MaxValue;
            if (ahead > 0)
            {
                double smaA = OrbitMath.SmaFromPeriod(mu, ahead);
                if (2 * smaA - rbm >= minR)
                    dvAhead = OrbitMath.SpeedAt(mu, rbm, smaA) - vb.magnitude;
            }

            bool useAhead = Math.Abs(dvAhead) < Math.Abs(dvBehind);
            phasing_period = useAhead ? ahead : behind;
            phasing_start_UT = GeneralTools.Current_UT + dt;
            return new BurnPlan { UT = phasing_start_UT, prograde = useAhead ? dvAhead : dvBehind };
        }

        BurnPlan PlanPhasing2()
        {
            GetState(out var r, out var v, out double mu);
            // measure the phasing orbit we actually got, then come back after n of its periods
            double actual = LandingTargeting.OrbitalPeriodFromStateVectors(r, v, mu);
            double ut = phasing_start_UT + phasing_n * actual;
            double dt = ut - GeneralTools.Current_UT;
            KeplerPropagator.Propagate(r, v, mu, dt, out var rb, out var vb);
            logger.LogInfo($"[Constellation] phasing period planned {phasing_period:n1} s, actual {actual:n1} s");
            return new BurnPlan
            {
                UT = ut,
                prograde = OrbitMath.SpeedAt(mu, rb.magnitude, reference_sma) - vb.magnitude,
            };
        }

        public double polar_normal_sign = 1;

        BurnPlan PlanPolar()
        {
            GetState(out var r, out var v, out double mu);
            double period = LandingTargeting.OrbitalPeriodFromStateVectors(r, v, mu);
            double dt = OrbitMath.TimeToNextNode(r, v, mu, period, out bool ascending);
            if (dt < 0) return null;
            if (dt < Lead)
            {
                KeplerPropagator.Propagate(r, v, mu, Lead, out var r2, out var v2);
                dt = Lead + OrbitMath.TimeToNextNode(r2, v2, mu, period, out ascending);
            }
            KeplerPropagator.Propagate(r, v, mu, dt, out var rb, out var vb);

            double inc = OrbitMath.InclinationDeg(r, v);
            double delta = (plane_target - inc) * Math.PI / 180.0;
            double speed = vb.magnitude;
            // +normal at the ascending node raises inclination; flipped at the descending node
            double sign = (ascending ? 1 : -1) * polar_normal_sign;
            return new BurnPlan
            {
                UT = GeneralTools.Current_UT + dt,
                prograde = speed * (Math.Cos(delta) - 1.0),
                normal = sign * speed * Math.Sin(delta),
            };
        }

        /// <summary>Reads the planned post-burn orbit; false asks NodeStep to replan with the normal flipped.</summary>
        bool? VerifyPolar(double node_UT)
        {
            var solver = current_vessel?.VesselComponent?.Orbiter?.ManeuverPlanSolver;
            var trajectory = solver?.ManeuverTrajectory;
            if (trajectory == null || trajectory.Count == 0) return null; // not solved yet
            // the post-burn orbit is the patch that begins at the node
            var patch = trajectory.OfType<PatchedConicsOrbit>()
                .Where(p => p.StartUT >= node_UT - 5)
                .OrderBy(p => p.StartUT)
                .FirstOrDefault();
            if (patch == null)
            {
                logger.LogInfo("[Constellation] post-burn patch not found yet: " + string.Join(", ",
                    trajectory.Select(p => $"{p.GetType().Name} {p.StartUT - node_UT:n0}s {p.PatchStartTransition}")));
                return null;
            }
            double after = patch.inclination;
            logger.LogInfo($"[Constellation] plane change node gives inclination {after:n2}° (target {plane_target:0.#}°)");
            if (Math.Abs(after - plane_target) < 1.0) return true;
            polar_normal_sign = -polar_normal_sign;
            return false;
        }

        // ================================================================ steps

        public abstract class Step
        {
            public string name;
            public string status = "";
            public virtual void Start() { }
            public abstract bool Update();
            public virtual void Stop() { }
        }

        class ActionStep : Step
        {
            readonly Action action;
            public ActionStep(string name, Action action) { this.name = name; this.action = action; }
            public override bool Update() { action(); return true; }
        }

        class WaitStep : Step
        {
            readonly float seconds;
            float end;
            public WaitStep(float seconds, string name) { this.seconds = seconds; this.name = name; }
            public override void Start() { end = Time.time + seconds; }
            public override bool Update()
            {
                status = $"{Math.Max(0, end - Time.time):n0}s";
                return Time.time >= end;
            }
        }

        internal class AlignStep : Step
        {
            readonly ConstellationPilot pilot;
            readonly AutopilotMode mode;
            float start, steady_since;
            public AlignStep(ConstellationPilot pilot, AutopilotMode mode, string name)
            { this.pilot = pilot; this.mode = mode; this.name = name; }

            public override void Start()
            {
                TimeWarpTools.SetRateIndex(0, false);
                SASTool.setAutoPilot(mode);
                start = Time.time;
                steady_since = -1;
            }

            public override bool Update()
            {
                SASTool.setAutoPilot(mode);
                var telemetry = SASTool.getTelemetry();
                if (telemetry == null) return false;

                Vector target = mode == AutopilotMode.RadialOut ? telemetry.OrbitMovementRadialOut : telemetry.OrbitMovementRadialIn;
                Rotation rot = Rotation.Reframed(pilot.current_vessel.GetRotation(), target.coordinateSystem);
                Vector3d forward = (rot.localRotation * Vector3.up).normalized;
                double angle = Vector3d.Angle(target.vector, forward);
                // same steadiness test as the Node tab's TurnTo
                var spin = pilot.current_vessel.GetAngularSpeed().vector;
                double max_spin = TurnToSettings.max_angular_speed.V;
                bool steady = Math.Abs(spin.x) <= max_spin && Math.Abs(spin.y) <= max_spin && Math.Abs(spin.z) <= max_spin;
                status = $"{angle:n1}° off{(steady ? "" : ", steadying")}";

                bool ok = angle < 1.0 && steady;
                if (!ok) { steady_since = -1; }
                else if (steady_since < 0) steady_since = Time.time;

                if (steady_since > 0 && Time.time - steady_since > 2) return true;
                if (Time.time - start > 120)
                    throw new Exception($"Couldn't line up {mode} within 2 minutes");
                return false;
            }
        }

        /// <summary>
        /// Closed-loop period trim: points prograde/retrograde and burns at a throttle sized to take
        /// a few seconds, watching the period itself until it is within a few hundredths of a
        /// second. The node executor stops at 0.1 m/s, about 1 s of period here - too coarse.
        /// </summary>
        class PeriodTrimStep : Step
        {
            const double Tolerance = 0.05;
            const int MaxPasses = 4;

            readonly ConstellationPilot pilot;
            readonly Func<double> target;
            readonly TurnTo turn = new();
            readonly BurndV burn_dv = new();

            enum Phase { Settle, Check, Turn, Burn }
            Phase phase;
            readonly SettleWait settle = new();
            int passes;
            double start_sign;
            float turn_start;

            public PeriodTrimStep(ConstellationPilot pilot, string name, Func<double> target)
            { this.pilot = pilot; this.name = name; this.target = target; }

            public override void Start()
            {
                passes = 0;
                TimeWarpTools.SetRateIndex(0, false);
                pilot.current_vessel.SetThrottle(0);
                settle.Start();
                phase = Phase.Settle;
            }

            double Error() => target() - pilot.CurrentPeriod();

            public override bool Update()
            {
                double err = Error();
                switch (phase)
                {
                    case Phase.Settle:
                        turn.Update();
                        status = "waiting for the engine to wind down";
                        if (settle.Update()) phase = Phase.Check;
                        return false;

                    case Phase.Check:
                        pilot.logger.LogInfo($"[Constellation] {name}: period error {err:+0.000;-0.000} s (pass {passes + 1})");
                        if (Math.Abs(err) < Tolerance || passes >= MaxPasses)
                            return true;
                        start_sign = Math.Sign(err);
                        // longer period = more energy = prograde
                        if (err > 0) turn.StartProGrade(SpeedDisplayMode.Orbit);
                        else turn.StartRetroGrade(SpeedDisplayMode.Orbit);
                        turn_start = Time.time;
                        phase = Phase.Turn;
                        return false;

                    case Phase.Turn:
                        turn.Update();
                        status = $"{err:+0.00;-0.00} s - {turn.status_line}";
                        if (!turn.finished)
                        {
                            if (Time.time - turn_start > 120) throw new Exception("Couldn't line up for the trim");
                            return false;
                        }
                        phase = Phase.Burn;
                        return false;

                    case Phase.Burn:
                        turn.Update(); // keeps SAS on prograde/retrograde
                        status = $"trimming, {err:+0.000;-0.000} s to go";
                        if (Math.Abs(err) < Tolerance || Math.Sign(err) != start_sign)
                        {
                            pilot.current_vessel.SetThrottle(0);
                            passes++;
                            settle.Start();
                            phase = Phase.Settle;
                            return false;
                        }
                        pilot.GetState(out var r, out var v, out double mu);
                        double period = pilot.CurrentPeriod();
                        // dT/T = 3 dv/v on a near-circular orbit
                        double dv = Math.Abs(err) * v.magnitude / (3.0 * period);
                        burn_dv.LateUpdate();
                        double accel = burn_dv.full_dv;
                        // aim to take about 4 s, so a frame's worth of thrust can't overshoot
                        float throttle = accel > 0 ? (float)(dv / (accel * 4.0)) : 0.01f;
                        pilot.current_vessel.SetThrottle(Mathf.Clamp(throttle, 0.002f, 1f));
                        return false;
                }
                return true;
            }

            public override void Stop() => pilot.current_vessel?.SetThrottle(0);
        }

        /// <summary>
        /// The Terrier keeps pushing for a moment after the throttle goes to zero, so orbit
        /// readings taken straight after a burn are stale. Waits until thrust is really zero and at
        /// least MinSeconds have passed.
        /// </summary>
        class SettleWait
        {
            const float MinSeconds = 2f;
            const float MaxSeconds = 15f;
            readonly BurndV thrust = new();
            float start;

            public void Start() => start = Time.time;

            public bool Update()
            {
                thrust.LateUpdate();
                float t = Time.time - start;
                if (t > MaxSeconds) return true;
                return t >= MinSeconds && thrust.actual_dv < 1e-3f;
            }
        }

        public class NodeStep : Step
        {
            readonly ConstellationPilot pilot;
            readonly Func<BurnPlan> plan;
            public Func<bool?> verify;

            const double MinBurn = 0.05;

            enum Phase { Create, WaitNode, Verify, Warp, Turn, FinalWarp, Burn, Settle, Done }

            // after lining up, warp again to this long before ignition (2026-11-23: it used to sit at
            // 1x for most of the minute between the first warp and ignition)
            const double FinalWarpLeadS = 10;
            readonly SettleWait settle = new();
            Phase phase;
            ManeuverNodeData node;
            int frames;
            int replans;

            readonly TurnTo turn = new();
            readonly WarpTo warp = new();
            readonly BurnManeuver burn = new();

            public NodeStep(ConstellationPilot pilot, string name, Func<BurnPlan> plan)
            { this.pilot = pilot; this.name = name; this.plan = plan; }

            public override void Start()
            {
                phase = Phase.Create;
                replans = 0;
            }

            void Create()
            {
                var p = plan();
                // too small to be worth turning, warping and lighting the engine for
                if (p != null && Math.Abs(p.prograde) + Math.Abs(p.normal) < MinBurn)
                {
                    pilot.logger.LogInfo($"[Constellation] {name}: skipped, only {Math.Abs(p.prograde) + Math.Abs(p.normal):n3} m/s");
                    p = null;
                }
                if (p == null)
                {
                    status = "not needed";
                    phase = Phase.Done;
                    return;
                }
                pilot.logger.LogInfo($"[Constellation] {name}: T+{p.UT - GeneralTools.Current_UT:n0}s prograde {p.prograde:n2} normal {p.normal:n2} m/s");
                node = null;
                phase = Phase.WaitNode;
                pilot.maneuver_creator.Update();
                pilot.maneuver_creator.RemoveAllNodesThenCreate(p.UT, p.prograde, n => node = n, p.normal);
            }

            // half the burn before the node: the game's BurnDuration (at full thrust, stretched by the
            // acceleration cap) unless it disagrees with Δv / acceleration - see BurndV.SaneBurnDuration
            double BurnStartUT => node.Time - accel_probe.SaneBurnDuration(node, pilot.settings.max_accel.V, ref burn_time_logged, pilot.logger) / 2;
            string burn_time_logged;
            readonly BurndV accel_probe = new();
            public double NodeUT => node?.Time ?? 0;

            public override bool Update()
            {
                switch (phase)
                {
                    case Phase.Create:
                        Create();
                        return phase == Phase.Done;

                    case Phase.WaitNode:
                        status = "plotting node";
                        if (node == null) return false;
                        frames = 0;
                        phase = verify != null ? Phase.Verify : Phase.Warp;
                        if (phase == Phase.Warp) StartWarp();
                        return false;

                    case Phase.Verify:
                        status = "checking node";
                        if (++frames < 10) return false;
                        var ok = verify();
                        if (ok == null)
                        {
                            if (frames > 300) throw new Exception("Node never solved");
                            return false;
                        }
                        if (ok == false)
                        {
                            if (++replans > 1) throw new Exception("Node doesn't reach the target - check the log");
                            phase = Phase.Create;
                            return false;
                        }
                        StartWarp();
                        return false;

                    case Phase.Warp:
                        warp.Update();
                        status = warp.status_line;
                        if (!warp.finished) return false;
                        TimeWarpTools.SetRateIndex(0, false);
                        turn.StartManeuver(node);
                        phase = Phase.Turn;
                        return false;

                    case Phase.Turn:
                        turn.Update();
                        status = turn.status_line;
                        if (!turn.finished) return false;
                        if (BurnStartUT - GeneralTools.Current_UT > FinalWarpLeadS + 5)
                        {
                            warp.Start_Ut(BurnStartUT - FinalWarpLeadS);
                            phase = Phase.FinalWarp;
                            return false;
                        }
                        StartBurn();
                        return false;

                    case Phase.FinalWarp:
                        warp.Update();
                        status = "lined up - " + warp.status_line;
                        if (!warp.finished) return false;
                        TimeWarpTools.SetRateIndex(0, false);
                        StartBurn();
                        return false;

                    case Phase.Burn:
                        burn.UT = BurnStartUT;
                        burn.Update();
                        status = burn.status_line;
                        if (!burn.finished) return false;
                        pilot.current_vessel.SetThrottle(0);
                        pilot.maneuver_creator.Update();
                        pilot.maneuver_creator.RemoveNode(node);
                        settle.Start();
                        phase = Phase.Settle;
                        return false;

                    case Phase.Settle:
                        status = "engine winding down";
                        if (!settle.Update()) return false;
                        phase = Phase.Done;
                        return true;

                    default:
                        return true;
                }
            }

            void StartBurn()
            {
                burn.StartManeuver(node);
                burn.max_accel = pilot.settings.max_accel.V;
                phase = Phase.Burn;
            }

            void StartWarp()
            {
                // warp to a minute before ignition, then turn (a long coast would undo an early turn)
                warp.Start_Ut(BurnStartUT - 60);
                phase = Phase.Warp;
            }

            public override void Stop()
            {
                TimeWarpTools.SetRateIndex(0, false);
                pilot.current_vessel?.SetThrottle(0);
            }
        }
    }
}
