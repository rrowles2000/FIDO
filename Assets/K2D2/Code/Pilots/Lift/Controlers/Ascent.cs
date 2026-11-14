
using K2D2.KSPService;
using KSP.Modules;
using KSP.Sim;
using KSP.Sim.impl;
using KTools;
using UnityEngine;
using UnityEngine.UIElements;
using K2D2.UI;
using K2D2.Controller;
using ILogger = ReduxLib.Logging.ILogger;
namespace K2D2.Lift
{
    /// <summary>
    /// rotation used for docking
    /// </summary>
    public class Ascent : AscentBase
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("K2D2.Ascent");

        LiftSettings settings = null;
        LiftAscentPath ascent_path = null;

        KSPVessel current_vessel;

        public Ascent(LiftSettings lift_settings, LiftAscentPath ascent_path)
        {
            current_vessel = K2D2_Plugin.Instance.current_vessel;
            this.settings = lift_settings;
            this.ascent_path = ascent_path;
        }

        float last_ap_km = 0;
        float wanted_elevation;

        float wanted_throttle = 0;

        float heading_correction = 0;
        float h_speed_heading = 0;

        // ROLL PROGRAM (see LiftSettings.roll_program* for the player-facing settings). Corrects the
        // vessel's roll relative to its OWN launch orientation - not an absolute compass/navball
        // heading - since an asymmetric build can spawn on the pad rolled away from the orientation
        // it was designed for.
        //
        // Roll is driven through SAS.LockRotation + lockedMode = true rather than
        // SetPersistentTargetOrientation: the latter's roll formula
        // (ComputePersistentTargetRollResponse in VesselSAS.cs) is a fixed-gain P+D loop with no
        // auto-tuning, while lockedMode routes all three axes through the same auto-tuned PID
        // (PidLockedRoll) already used for pitch/yaw. ComputeRollTargetRotation below builds a full
        // orientation (nose and dorsal/roll-reference pinned at once, via RotationFromNoseAndDorsal)
        // since neither FromToRotation nor LookRotation can pin two axes in a single call.
        //
        // The target ramps toward the configured angle at a limited rate (ramped_roll_target_deg,
        // settings.roll_program_rate_deg_s) instead of being commanded instantly - SAS's roll
        // response saturates on any large instantaneous error, which overshoots and oscillates. The
        // lockedMode target is only held while the program is actively ramping
        // (roll_program_started && !roll_program_done); control reverts to plain
        // SetTargetOrientation once settled, matching stock free/damped roll behavior.
        //
        // Roll traveled since launch (accumulated_roll_deg) is integrated tick-to-tick from the
        // dorsal reference's rotation delta (IntegrateRoll), rather than compared against a rotation
        // captured once at launch: Rotation.Reframed's target coordinate system is a per-position,
        // local-horizon-style frame, so reframing a rotation captured many ticks (and positions)
        // earlier and comparing it to the current one does not produce a valid delta. Reframing and
        // using the result immediately, same tick, is the pattern used elsewhere in this codebase
        // (TouchDown, TurnTo, DockingTurnTo) and the only one confirmed to work here.
        //
        // GetAngularSpeed()'s reported units are not plain radians/second, so roll rate
        // (debug_roll_rate_deg_s) is derived from the same tick-to-tick angle delta instead of that
        // API.
        bool roll_program_started = false;
        bool roll_program_done = false;
        double accumulated_roll_deg = 0; // total roll (deg) since launch, integrated tick-to-tick - see IntegrateRoll
        Rotation prev_roll_rotation; // previous tick's (reframed) rotation, for IntegrateRoll's tick-to-tick delta
        bool prev_roll_rotation_valid = false;
        double debug_roll_delta_deg = 0;
        double debug_roll_error_deg = 0;
        double debug_roll_rate_deg_s = 0;
        float ramped_roll_target_deg = 0; // current runtime roll target SAS is being fed, ramps toward settings.roll_program_angle_deg.V

        const float RollProgramCompletionErrorDeg = 2f;   // "close enough" once error drops under this...
        const float RollProgramCompletionOmega = 3f;      // ...AND the vessel has stopped spinning (deg/s, roll axis only - debug_roll_rate_deg_s, not GetAngularSpeed())
        // Ramp rate is player-adjustable (settings.roll_program_rate_deg_s, see LiftSettings.cs)
        // since different vessels can tolerate different ramp speeds before SAS overshoots.

        public override void Start()
        {
            base.Start();
            ResetDisposal();

            SASTool.setAutoPilot(AutopilotMode.StabilityAssist);
            last_ap_km = 0;
            ap_km = 0;
            delta_ap_per_second = 0;
            wanted_elevation = -90;

            accumulated_roll_deg = 0;
            prev_roll_rotation_valid = false;
            roll_program_started = false;
            roll_program_done = false;
            debug_roll_delta_deg = 0;
            debug_roll_error_deg = 0;
            ramped_roll_target_deg = 0;

            LogPlanet();
        }

        // [Planet] one line per run describing the planet of origin (see PlanetLog.cs).
        void LogPlanet() => PlanetLog.Log(logger, current_vessel.VesselComponent, settings.destination_Ap_km.V);

        // Integrates roll around the nose axis since launch by measuring, tick-to-tick, how far the
        // dorsal reference actually rotated between the previous tick's orientation and this tick's
        // (see the class comment for why comparing against a launch-time snapshot doesn't work).
        // Uses the same Cross/Dot signed-angle technique as ComputeRollTargetRotation and
        // TouchDown.ComputeSteeredDirection, applied to two rotations one tick apart, so the sign
        // convention stays consistent by construction and any Reframed staleness is bounded to a
        // single tick. Runs continuously from Start() onward, not just while the roll program is
        // engaged, so accumulated_roll_deg reflects real drift even before the program's trigger
        // altitude.
        void IntegrateRoll()
        {
            if (current_vessel.VesselComponent == null)
                return;

            Vector up_dir = current_vessel.VesselComponent.gravityForPos;
            Rotation cur_rotation = Rotation.Reframed(current_vessel.GetRotation(), up_dir.coordinateSystem);

            if (prev_roll_rotation_valid)
            {
                Vector3d nose_dir = (cur_rotation.localRotation * Vector3.up).normalized;
                Vector3d prev_dorsal = (prev_roll_rotation.localRotation * Vector3.forward).normalized;
                Vector3d cur_dorsal = (cur_rotation.localRotation * Vector3.forward).normalized;

                Vector3d prev_dorsal_flat = prev_dorsal - nose_dir * Vector3d.Dot(prev_dorsal, nose_dir);
                Vector3d cur_dorsal_flat = cur_dorsal - nose_dir * Vector3d.Dot(cur_dorsal, nose_dir);

                if (prev_dorsal_flat.magnitude > 1e-6 && cur_dorsal_flat.magnitude > 1e-6)
                {
                    prev_dorsal_flat = prev_dorsal_flat.normalized;
                    cur_dorsal_flat = cur_dorsal_flat.normalized;

                    double sign = System.Math.Sign(Vector3d.Dot(Vector3d.Cross(prev_dorsal_flat, cur_dorsal_flat), nose_dir));
                    if (sign == 0) sign = 1;

                    double delta_deg = Vector3d.Angle(prev_dorsal_flat, cur_dorsal_flat) * sign;
                    accumulated_roll_deg += delta_deg;
                    debug_roll_rate_deg_s = (Time.deltaTime > 1e-6) ? delta_deg / Time.deltaTime : 0;
                }
                // else: degenerate this tick (dorsal briefly ~parallel to nose) - skip, keep last
                // accumulated value and rate rather than risk a garbage spike.
            }

            prev_roll_rotation = cur_rotation;
            prev_roll_rotation_valid = true;
        }

        public override void computeValues(bool compute_delta_ap_per_second)
        {
            if (current_vessel.VesselComponent == null)
                return;

            // Reads Apoapsis/referenceBody through the PatchedConicsOrbit/IOrbit interface rather than
            // casting to the concrete PatchedConicsOrbit type: Redux's ECS layer hands back
            // Redux.Ecs.Components.CurrentPatchedConicsOrbit for the actively-flown vessel, an
            // unrelated sibling class that also implements PatchedConicsOrbit, so a hard cast throws
            // InvalidCastException for exactly the vessel this autopilot is flying.
            PatchedConicsOrbit orbit = current_vessel.VesselComponent.Orbit;
            ap_km = (float)(orbit.Apoapsis - orbit.referenceBody.radius) / 1000;
            current_altitude_km = (float)(current_vessel.GetSeaAltitude() / 1000);

            if (last_ap_km == 0)
            {
                last_ap_km = ap_km;
            }
            else
            {
                float delta_ap = ap_km - last_ap_km;
                last_ap_km = ap_km;
                if (compute_delta_ap_per_second)
                {
                    // compute delta_ap_per_second only on ascent
                    float throttle = (float)current_vessel.GetThrottle();
                    if (throttle > 0.1f && Time.deltaTime != 0)
                    {
                        float new_delta_ap_per_second = delta_ap / (Time.deltaTime * throttle);
                        delta_ap_per_second = Mathf.Lerp(delta_ap_per_second, new_delta_ap_per_second, 0.1f);
                    }
                    else
                        delta_ap_per_second = 0;
                }
            }

            wanted_elevation = ascent_path.compute_elevation(current_altitude_km);
        }

        public void applyDirection()
        {
            var autopilot = current_vessel.Autopilot;

            if (autopilot == null)
                return;

            // force autopilot
            autopilot.Enabled = true;

            var telemetry = SASTool.getTelemetry();
            var up = telemetry.HorizonUp;

            if (settings.heading_correction.V)
            {
                computeSpeedHeading();
            }
            else
                heading_correction = 0;


            Vector3d direction = QuaternionD.Euler(-wanted_elevation, settings.heading.V + heading_correction, 0) * Vector3d.forward;
            Vector direction_vector = new Vector(up.coordinateSystem, direction);

            autopilot.SAS.lockedMode = false;

            if (settings.roll_program.V && roll_program_started && !roll_program_done)
            {
                // Roll program actively ramping toward its target - lock onto a full orientation (see
                // class comment) so SAS drives roll via the auto-tuned PidLockedRoll path. Only held
                // while ramping; once roll_program_done this falls through to the plain branch below.
                Rotation target_rotation = ComputeRollTargetRotation(direction, up);
                autopilot.SAS.LockRotation(target_rotation);
                autopilot.SAS.lockedMode = true;
            }
            else
            {
                // Roll program off, hasn't reached its altitude yet, or already done - unchanged
                // stock behavior (roll left free/damped, same as ascent has always behaved without
                // this feature at all).
                autopilot.SAS.SetTargetOrientation(direction_vector, false);
            }
        }

        // Builds the full orientation handed to SAS.LockRotation for the roll program: lockedMode
        // needs both axes pinned at once - local Y (SAS's "nose") at nose_target, local Z (SAS's
        // dorsal/roll-reference) at the desired roll direction. Neither FromToRotation nor
        // LookRotation can pin two axes in one call, so RotationFromNoseAndDorsal builds it directly
        // from an orthonormal basis.
        Rotation ComputeRollTargetRotation(Vector3d nose_target, Vector up)
        {
            // Same-tick reframe: GetRotation() is read and used immediately, never stored across
            // ticks (see class comment on why comparing against an earlier-tick rotation fails).
            Rotation cur_rotation = Rotation.Reframed(current_vessel.GetRotation(), up.coordinateSystem);

            // Current dorsal/roll-reference axis - Vector3.forward, matching SAS's own convention.
            // (TouchDown.cs uses Vector3.down for this same rotation source, for its tail-first
            // landing-burn framing; that doesn't apply here.)
            Vector3d current_dorsal = (cur_rotation.localRotation * Vector3.forward).normalized;

            // Flatten against the TARGET nose direction (not the vessel's own current nose) so the
            // result stays a valid roll reference once the vessel actually gets there.
            Vector3d current_dorsal_flat = current_dorsal - nose_target * Vector3d.Dot(current_dorsal, nose_target);
            if (current_dorsal_flat.magnitude < 1e-6)
            {
                // Degenerate (current dorsal ended up ~parallel to the target nose) - fall back to
                // an arbitrary axis perpendicular to the nose so this still produces something
                // well-defined.
                Vector3d arbitrary = new Vector3d(0, 1, 0);
                current_dorsal_flat = arbitrary - nose_target * Vector3d.Dot(arbitrary, nose_target);
                if (current_dorsal_flat.magnitude < 1e-6)
                {
                    arbitrary = new Vector3d(1, 0, 0);
                    current_dorsal_flat = arbitrary - nose_target * Vector3d.Dot(arbitrary, nose_target);
                }
            }
            current_dorsal_flat = current_dorsal_flat.normalized;

            // Rotate the flattened dorsal reference around the nose axis by the remaining roll
            // needed (ramped target minus accumulated_roll_deg). Rodrigues' rotation formula, same
            // Cos/Cross/Sin pattern as TouchDown.ComputeSteeredDirection. current_dorsal_flat is
            // already perpendicular to nose_target, so the formula's parallel-component term drops
            // out.
            double remaining_roll_deg = ramped_roll_target_deg - accumulated_roll_deg;
            double roll_rad = remaining_roll_deg * (System.Math.PI / 180.0);
            Vector3d desired_dorsal_dir = current_dorsal_flat * System.Math.Cos(roll_rad)
                + Vector3d.Cross(nose_target, current_dorsal_flat) * System.Math.Sin(roll_rad);

            QuaternionD target_local_rotation = RotationFromNoseAndDorsal(nose_target, desired_dorsal_dir);
            return new Rotation(up.coordinateSystem, target_local_rotation);
        }

        // Builds a rotation whose local Y axis ("nose") points at nose_ex and local Z axis
        // (dorsal/roll-reference) points at dorsal_ez (re-orthogonalized below in case the inputs
        // aren't exactly perpendicular). Standard rotation-matrix-to-quaternion conversion
        // (Shepperd's method) rather than QuaternionD.FromToRotation/LookRotation, since neither can
        // pin two axes in one call.
        static QuaternionD RotationFromNoseAndDorsal(Vector3d nose_ex, Vector3d dorsal_ez)
        {
            Vector3d ey = nose_ex.normalized;
            Vector3d ez = (dorsal_ez - ey * Vector3d.Dot(dorsal_ez, ey)).normalized;
            Vector3d ex = Vector3d.Cross(ey, ez).normalized;

            // Standard rotation-matrix-to-quaternion conversion, matrix columns = (ex, ey, ez).
            double m00 = ex.x, m10 = ex.y, m20 = ex.z;
            double m01 = ey.x, m11 = ey.y, m21 = ey.z;
            double m02 = ez.x, m12 = ez.y, m22 = ez.z;

            double trace = m00 + m11 + m22;
            double qx, qy, qz, qw;
            if (trace > 0)
            {
                double s = System.Math.Sqrt(trace + 1.0) * 2.0;
                qw = s / 4.0;
                qx = (m21 - m12) / s;
                qy = (m02 - m20) / s;
                qz = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                double s = System.Math.Sqrt(1.0 + m00 - m11 - m22) * 2.0;
                qw = (m21 - m12) / s;
                qx = s / 4.0;
                qy = (m01 + m10) / s;
                qz = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                double s = System.Math.Sqrt(1.0 + m11 - m00 - m22) * 2.0;
                qw = (m02 - m20) / s;
                qx = (m01 + m10) / s;
                qy = s / 4.0;
                qz = (m12 + m21) / s;
            }
            else
            {
                double s = System.Math.Sqrt(1.0 + m22 - m00 - m11) * 2.0;
                qw = (m10 - m01) / s;
                qx = (m02 + m20) / s;
                qy = (m12 + m21) / s;
                qz = s / 4.0;
            }

            return new QuaternionD(new Vector3d(qx, qy, qz), qw);
        }

        void computeSpeedHeading()
        {
            // use Up local coordinate as reference frame
            var Upcoords = current_vessel.VesselVehicle.Up.coordinateSystem;
            var SurfaceVelocity = Vector.Reframed(current_vessel.VesselVehicle.SurfaceVelocity, Upcoords).vector;
            var North = Vector.Reframed(current_vessel.VesselVehicle.North, Upcoords).vector;
            var Up = current_vessel.VesselVehicle.Up.vector;

            var UpSpeed = Up.normalized * Vector3d.Dot(SurfaceVelocity, Up);
            var LocalHSpeed = SurfaceVelocity - UpSpeed;

       
            h_speed_heading = (float)-Vector3d.SignedAngle(LocalHSpeed.normalized, North, Up);


            heading_correction = GeneralTools.diffAngle(settings.heading.V, h_speed_heading);

            if (heading_correction > 45)
                heading_correction = 45;
            else if (heading_correction < -45)
                heading_correction = -45;
        }

        // Tracks whether the roll program has reached its trigger altitude, and advances the ramped
        // target toward the configured angle. Only updates state; applyDirection() actually commands
        // SAS.
        void UpdateRollProgram()
        {
            if (!settings.roll_program.V)
            {
                // Toggle is off - stay ready to run again (in case it gets turned on mid-ascent)
                // rather than latching state from a stale earlier attempt.
                roll_program_started = false;
                roll_program_done = false;
                return;
            }

            bool just_engaged = false;

            if (!roll_program_started)
            {
                if (current_altitude_km < settings.roll_program_altitude_km.V)
                    return; // hasn't reached the roll program's altitude yet

                roll_program_started = true;
                just_engaged = true;
            }

            UpdateRollTelemetry();

            if (just_engaged)
            {
                // Start the ramp from wherever the vessel is ACTUALLY rolled to right now
                // (debug_roll_delta_deg, just computed above), not an assumed 0 - avoids a jump the
                // instant the roll program turns on if the vessel had already drifted a bit.
                ramped_roll_target_deg = (float)debug_roll_delta_deg;
                logger.LogInfo($"[Ascent] Roll program engaged at {current_altitude_km:n2} km, " +
                    $"currently {ramped_roll_target_deg:n1}° from launch, ramping to " +
                    $"{settings.roll_program_angle_deg.V:n1}°.");
            }

            // Advance the ramped target toward the configured angle at a limited rate, so SAS is
            // never handed a big instantaneous roll error (see class-level "SLOW ROLL RAMP" comment).
            float remaining = GeneralTools.diffAngle(settings.roll_program_angle_deg.V, ramped_roll_target_deg);
            float max_step = settings.roll_program_rate_deg_s.V * Time.deltaTime;
            if (Mathf.Abs(remaining) <= max_step)
                ramped_roll_target_deg = settings.roll_program_angle_deg.V;
            else
                ramped_roll_target_deg += Mathf.Sign(remaining) * max_step;
        }

        // Reads accumulated roll (accumulated_roll_deg, integrated in IntegrateRoll) for the LIFT
        // INFO table, and decides whether the roll program has settled.
        void UpdateRollTelemetry()
        {
            debug_roll_delta_deg = accumulated_roll_deg;

            float roll_error_deg = GeneralTools.diffAngle(settings.roll_program_angle_deg.V, (float)accumulated_roll_deg);
            debug_roll_error_deg = roll_error_deg;

            // Uses debug_roll_rate_deg_s (IntegrateRoll's tick-to-tick measurement) rather than
            // GetAngularSpeed(), whose reported units are not plain rad/s here.
            roll_program_done = Mathf.Abs(roll_error_deg) < RollProgramCompletionErrorDeg && Mathf.Abs((float)debug_roll_rate_deg_s) < RollProgramCompletionOmega;
        }

        // Ascent has no narrative feedback of its own beyond LiftUI's generic "Status : Ascent"
        // headline - all its telemetry moved to UpdateInfoRows below (LIFT INFO table) instead of
        // scrolling through the console text.
        public override void updateUI(VisualElement root_el, FullStatus st)
        {
        }

        public override void UpdateInfoRows(System.Action<string, string> addRow)
        {
            if (settings.disposal_enabled.V)
            {
                addRow("Stage 1 Disposal", disposal_phase);
                addRow("Periapsis Alt.", $"{pe_km:n2} km (target {settings.disposal_pe_km.V:n0})");
            }
            addRow("Apoapsis Alt.", $"{ap_km:n2} km");
            addRow("Altitude", $"{current_altitude_km:n2} km");
            addRow("Climb Rate", $"{delta_ap_per_second:n2} km/s");
            addRow("Pitch Target", $"{wanted_elevation:n2} °");
            addRow("Throttle", $"{wanted_throttle:n2}");

            if (settings.heading_correction.V)
            {
                addRow("Surface Heading", $"{h_speed_heading:n2} °");
                addRow("Heading Correction", $"{heading_correction:n2} °");
            }

            if (settings.roll_program.V)
            {
                string roll_status = roll_program_done ? "Done" : (roll_program_started ? "Active" : "Waiting for Altitude");
                addRow("Roll Program", roll_status);
                addRow("Roll (from launch)", $"{debug_roll_delta_deg:n1} °");
                addRow("Roll Error", $"{debug_roll_error_deg:n1} °");
                addRow("Roll Dbg: RollRate", $"{debug_roll_rate_deg_s:n2} °/s");
            }
        }

        // ---------------------------------------------------------------- stage 1 disposal
        // Profile climb at the max throttle until apoapsis reaches the target, then hold apoapsis
        // and burn on (Stage1Guidance, the same law the optimiser simulated) until periapsis is up
        // to disposal_pe_km. Then cut the engines and stage until the marked stage-1 separator has
        // fired. If stage 1 runs dry first (down to its burnout mass, or its lit engines give no
        // thrust), stage it and let the upper stage finish the job.

        bool disposal_hold;
        string disposal_phase = "";
        float pe_km;
        float dry_time;
        int separation_attempts;
        double next_separation_ut;
        bool disposal_cutoff;
        KSP.Sim.impl.PartComponent stage1_separator;

        bool launch_checked;        // pre-launch safety check done (first Update on the pad)
        bool ignition_tried;        // staged once on the pad because no engine was lit
        double ignition_ut = -1;
        double first_thrust_ut = -1; // first time the engines gave thrust
        double next_inner_stage_ut;  // rate limit for staging between stage-1 burn stages
        bool loss_of_control_logged;
        float q_throttle = 1f;       // max-Q throttle bucket factor
        bool bucket_suspended_logged;
        const double AttitudeErrorNoBucketDeg = 1.5;

        void ResetDisposal()
        {
            separated = false;
            ap_burn = false;
            hold_throttle = 1f;
            pitch_smoother.Reset();
            disposal_hold = false;
            disposal_cutoff = false;
            disposal_phase = "Profile climb";
            dry_time = 0;
            separation_attempts = 0;
            launch_checked = false;
            ignition_tried = false;
            ignition_ut = -1;
            first_thrust_ut = -1;
            next_inner_stage_ut = 0;
            loss_of_control_logged = false;
            q_throttle = 1f;
            stage1_separator = K2D2.Lift.Stage1.Stage1Model.FindMarkedSeparator(current_vessel.VesselComponent);
            stage1_model = K2D2.Lift.Stage1.Stage1Model.Build(current_vessel.VesselComponent);
            // No marked separator on board (already dropped, e.g. after a reload, or never marked):
            // nothing to dispose of - fly the normal ascent instead of holding Ap with the upper stage.
            if (stage1_separator == null)
            {
                separated = true;
                disposal_phase = "No stage 1 separator on board - normal ascent";
            }
            next_log_ut = 0;
            launch_ut = current_vessel.VesselComponent != null && current_vessel.VesselComponent.Landed ? -1 : GeneralTools.Current_UT;
        }

        K2D2.Lift.Stage1.Stage1Model stage1_model;
        double next_log_ut;
        double launch_ut = -1;

        // Every 2 s in the atmosphere: real drag (the game's own per-part formula) next to the
        // simulator's nose-first table, plus the heating proxy - to calibrate Stage1Sim.
        void LogFlightCalibration(VesselComponent vessel, double now)
        {
            if (now < next_log_ut || stage1_model == null) return;
            next_log_ut = now + 2;
            var body = vessel.mainBody;
            double alt = vessel.AltitudeFromSeaLevel;
            if (!body.hasAtmosphere || alt > body.atmosphereDepth) return;
            double p = body.GetPressure(alt);
            double d = body.GetDensity(p, body.GetTemperature(alt));
            double v = K2D2.Landing.VesselAeroLookup.GetReferencePartSpeed(vessel);
            double a = d > 0 ? AeroUtilities.GetSoundSpeed(body.atmosphereAdiabaticIndex, p, d) : 0;
            double mach = a > 0 ? v / a : 0;
            double q = 0.5 * d * v * v;
            AirState(vessel, out double path_deg, out _);
            double nose = NoseElevationDeg();
            double aoa = nose - path_deg; // flown, not commanded
            double model_kN = stage1_model.AreaDragAt(mach, aoa) * PhysicsSettings.DragCubeMultiplier *
                              PhysicsSettings.DragCurvePseudoReynolds.Evaluate((float)(d * v)) *
                              PhysicsSettings.DragMultiplier * q * 0.001;
            var real = K2D2.Landing.VesselAeroLookup.EvaluateVesselAeroReal(vessel);
            if (launch_ut < 0 && !vessel.Landed) launch_ut = now;
            logger.LogInfo($"[Stage1] flight T+{(launch_ut < 0 ? 0 : now - launch_ut):n0} alt {alt / 1000:n1} km, v {v:n0} m/s, M {mach:n2}, q {q / 1000:n1} kPa, " +
                           $"drag real {real.totalForceEstimate:n1} kN / model {model_kN:n1} kN, " +
                           $"heat {System.Math.Sqrt(d) * v * v * v / K2D2.Lift.Stage1.Stage1Optimiser.HeatLimit * 100:n0}%, " +
                           $"cmd {wanted_elevation:n1}, nose {nose:n1}, path {path_deg:n1}, plane {(LaunchPlanePath(out double pp) ? pp : double.NaN):n1}, aoa {aoa:n1}, thr {current_vessel.GetThrottle():n2}, " +
                           $"Ap {ap_km:n1} Pe {pe_km:n1}, mass {vessel.totalMass:n1} t");
        }

        bool SeparatorAttached()
        {
            var parts = current_vessel.VesselComponent?.SimulationObject?.PartOwner?.Parts;
            if (stage1_separator == null || parts == null) return false;
            foreach (var p in parts)
                if (ReferenceEquals(p, stage1_separator)) return true;
            return false;
        }

        void UpdateDisposal()
        {
            var vessel = current_vessel.VesselComponent;
            var orbit = vessel?.Orbit;
            if (orbit == null) return;
            double now = GeneralTools.Current_UT;
            double mu = orbit.referenceBody.gravParameter, R = orbit.referenceBody.radius;

            // On the pad: refuse settings the simulator says won't make it (2026-11-13: a plan
            // changed after Apply flew at 0.72 thrust, which predicts 183% heating), then light
            // the engines if nobody has.
            if (!launch_checked)
            {
                launch_checked = true;
                if (!separated && vessel.Landed && stage1_model != null && stage1_model.valid)
                {
                    var check = K2D2.Lift.Stage1.Stage1Optimiser.Evaluate(stage1_model,
                        K2D2.Lift.Stage1.SimParams.FromSettings(settings));
                    if (!check.Usable)
                    {
                        string why = "Launch refused: " + check.problem + ". Re-run the Stage 1 optimiser and Apply, or change the profile.";
                        logger.LogWarning("[Ascent] " + why);
                        current_vessel.SetThrottle(0);
                        LiftPilot.Instance.EndLiftPilot(false, why);
                        return;
                    }
                    logger.LogInfo($"[Ascent] launch check ok: margin {check.nominal.upperMarginDv:n0} m/s (worst case {check.worst.upperMarginDv:n0}), " +
                                   $"max Q {check.nominal.maxQkPa:n1} kPa ({check.worst.maxQkPa:n1}), heat {check.nominal.maxHeat / K2D2.Lift.Stage1.Stage1Optimiser.HeatLimit * 100:n0}% " +
                                   $"({check.worst.maxHeat / K2D2.Lift.Stage1.Stage1Optimiser.HeatLimit * 100:n0}%)");
                }
            }
            bool lit = EnginesBurning(out double thrust_kN);
            if (!separated && !disposal_cutoff && vessel.Landed && !lit && !ignition_tried)
            {
                // Engaged before the launch stage was fired: fire it once, at the planned thrust.
                // (Firing it is all the old code did too, but only by accident, by "separating"
                // a stage 1 it thought was dry and then staging everything above it.)
                ignition_tried = true;
                ignition_ut = now;
                current_vessel.SetThrottle(settings.max_throttle.V);
                vessel.ActivateNextStage();
                logger.LogInfo("[Ascent] disposal: no engine lit on the pad - staged once for ignition");
            }
            if (ignition_ut > 0 && !lit && vessel.Landed && now - ignition_ut > 3)
            {
                ignition_ut = -1; // warn once
                disposal_phase = "No engine lit - stage the launch engines by hand";
                logger.LogWarning("[Ascent] disposal: still no engine lit 3 s after staging - stage the launch engines by hand");
            }
            if (lit && thrust_kN > 0.5 && first_thrust_ut < 0) first_thrust_ut = now;

            if (disposal_cutoff)
            {
                // stage until the marked separator has fired (stage 1 may still have booster stages)
                current_vessel.SetThrottle(0);
                IntegrateRoll();
                applyDirection();
                if (SeparatorAttached() && separation_attempts < 6)
                {
                    if (now >= next_separation_ut)
                    {
                        vessel.ActivateNextStage();
                        separation_attempts++;
                        next_separation_ut = now + 0.5;
                    }
                    return;
                }
                disposal_cutoff = false;
                separated = true;
            }

            // Stage 1 gone: the upper stage finishes the climb if apoapsis is still short, then
            // hands over to Coasting/Adjust/Final, which coast to apoapsis and circularise there -
            // the cheapest way to finish (that's what the optimiser scored).
            if (separated || (stage1_separator != null && !SeparatorAttached()))
            {
                separated = true;
                if (ap_km >= settings.destination_Ap_km.V - settings.end_ascent_error)
                {
                    disposal_phase = "Separated - coast to Ap";
                    current_vessel.SetThrottle(0);
                    finished = true;
                    return;
                }
                disposal_phase = "Separated - upper stage climbing";
                UpdateNormal();
                return;
            }

            IntegrateRoll();
            UpdateRollProgram();
            finished = false;

            Vector3d r = orbit.GetRelativePositionAtUTZup(now);
            Vector3d v = orbit.GetOrbitalVelocityAtUTZup(now);
            double rMag = r.magnitude;
            double vr = Vector3d.Dot(v, r) / rMag;
            double vt = System.Math.Sqrt(System.Math.Max(0, v.sqrMagnitude - vr * vr));
            K2D2.Lift.Stage1.Stage1Guidance.ApPe(mu, rMag, vr, vt, out _, out double peR);
            pe_km = (float)((peR - R) / 1000);

            if (!disposal_hold && ap_km >= settings.destination_Ap_km.V - settings.end_ascent_error)
            {
                // Apoapsis above the atmosphere: cut the engines and hand over - Coasting warps up
                // out of the air nose-prograde, then Stage1Burn plans and flies the apoapsis burn
                // as a manoeuvre node.
                var atmo_body = orbit.referenceBody;
                if (!atmo_body.hasAtmosphere || orbit.Apoapsis > R + atmo_body.atmosphereDepth)
                {
                    logger.LogInfo($"[Ascent] disposal: Ap {ap_km:n1} km reached at {current_altitude_km:n1} km - coasting, stage-1 burn at apoapsis");
                    current_vessel.SetThrottle(0);
                    disposal_phase = "Ap reached - coasting";
                    finished = true;
                    return;
                }
                disposal_hold = true;
                disposal_phase = "Holding Ap, raising Pe";
                logger.LogInfo($"[Ascent] disposal: Ap reached at {current_altitude_km:n1} km, holding {settings.destination_Ap_km.V} km");
            }
            if (disposal_hold)
                wanted_elevation = (float)K2D2.Lift.Stage1.Stage1Guidance.HoldElevationDeg(mu, rMag, vr, vt,
                    R + settings.destination_Ap_km.V * 1000.0);
            // keep the nose near the airflow while the air is thick (same limit the optimiser simulated);
            // when that stops the hold steering, control apoapsis with the throttle instead
            hold_throttle = 1f;
            double target_ap_r = R + settings.destination_Ap_km.V * 1000.0;
            if (AirState(vessel, out double path_deg, out double q_kpa, out double mach_now))
            {
                if (LaunchPlanePath(out double plane_path))
                {
                    path_deg = plane_path;
                    if (!disposal_hold)
                    {
                        var vehicle_now = current_vessel.VesselVehicle;
                        double air_speed = vehicle_now != null ? vehicle_now.SurfaceVelocity.vector.magnitude : 0;
                        wanted_elevation = (float)K2D2.Lift.Stage1.Stage1Guidance.FollowPath(wanted_elevation, path_deg, air_speed);
                    }
                }
                if (disposal_hold)
                    hold_throttle = K2D2.Lift.Stage1.Stage1Guidance.HoldThrottle(wanted_elevation, path_deg, q_kpa,
                        orbit.Apoapsis, target_ap_r);
                wanted_elevation = (float)K2D2.Lift.Stage1.Stage1Guidance.LimitAoA(wanted_elevation, path_deg, q_kpa, mach_now);
            }

            // Coast up to apoapsis (nose pre-pointed) and burn there, straddling it, rather than
            // burning far below the horizon while still climbing - same rule the optimiser simulates.
            if (disposal_hold && !ap_burn && stage1_model != null && stage1_model.valid)
            {
                var last = stage1_model.phases[stage1_model.phases.Count - 1];
                var body = orbit.referenceBody;
                double atmo_top_r = R + (body.hasAtmosphere ? body.atmosphereDepth : 0);
                double cap_pe_r = R + settings.disposal_pe_km.V * 1000.0;
                bool coast = K2D2.Lift.Stage1.Stage1Guidance.CoastToApoapsis(mu, rMag, vr, vt, orbit.Apoapsis, peR,
                    atmo_top_r, cap_pe_r, vessel.totalMass, last.endMass, last.thrustVac * settings.max_throttle.V, last.ispVac);
                if (coast)
                {
                    hold_throttle = 0;
                    // point at the burn attitude (about horizontal at apoapsis) and stay there,
                    // instead of following the burning-hold steering while coasting
                    wanted_elevation = (float)K2D2.Lift.Stage1.Stage1Guidance.LimitAoA(0, path_deg, q_kpa, mach_now);
                    double t_ap = K2D2.Lift.Stage1.Stage1Guidance.TimeToApoapsis(mu, rMag, vr, vt);
                    double t_burn = K2D2.Lift.Stage1.Stage1Guidance.ApoapsisBurnTime(mu, orbit.Apoapsis, peR, cap_pe_r,
                        vessel.totalMass, last.endMass, last.thrustVac * settings.max_throttle.V, last.ispVac);
                    disposal_phase = $"Coasting to Ap - burn in {System.Math.Max(0, t_ap - t_burn / 2):n0} s ({t_burn:n0} s burn)";
                }
                else if (orbit.Apoapsis > atmo_top_r)
                {
                    ap_burn = true;
                    disposal_phase = "Apoapsis burn";
                    logger.LogInfo($"[Ascent] disposal: apoapsis burn starts at alt {current_altitude_km:n1} km, " +
                                   $"T-{K2D2.Lift.Stage1.Stage1Guidance.TimeToApoapsis(mu, rMag, vr, vt):n0} s to Ap, Pe {pe_km:n1} km");
                }
            }

            wanted_elevation = (float)pitch_smoother.Next(wanted_elevation, Time.deltaTime, q_kpa);
            // Still turning towards the hold attitude: thrusting along the old one pumps apoapsis up
            // (the 2026-10-20 flight overshot 85 -> 101 km like this), so throttle on apoapsis until
            // the nose is within 5° of where it should be.
            nose_elevation = NoseElevationDeg();
            if (disposal_hold && hold_throttle > 0 && System.Math.Abs(nose_elevation - wanted_elevation) > 5)
                hold_throttle = System.Math.Min(hold_throttle,
                    K2D2.Lift.Stage1.Stage1Guidance.ApThrottle(orbit.Apoapsis, target_ap_r));

            applyDirection();
            // max-Q throttle bucket - the same law the optimiser simulated. But never while the nose
            // is drifting off its command: the engine gimbal is what pulls it back, and on
            // 2026-11-13 the bucket cut thrust 0.90 -> 0.70 just as R2-1's nose started to fall
            // away (1.8° -> 6° -> flipped). Then thrust comes back quickly instead.
            float bucket_before = q_throttle;
            double attitude_error = System.Math.Abs(nose_elevation - wanted_elevation);
            if (attitude_error > AttitudeErrorNoBucketDeg && q_kpa > 1)
            {
                q_throttle = UnityEngine.Mathf.MoveTowards(q_throttle, 1f, 2f * Time.deltaTime);
                if (!bucket_suspended_logged && bucket_before < 1f)
                    logger.LogInfo($"[Ascent] disposal: attitude error {attitude_error:n1}° at q {q_kpa:n1} kPa - max-Q throttle-down suspended, full thrust for control");
                bucket_suspended_logged = true;
            }
            else
            {
                bucket_suspended_logged = false;
                q_throttle = K2D2.Lift.Stage1.Stage1Guidance.MaxQThrottleStep(q_throttle, q_kpa, Time.deltaTime);
            }
            if (bucket_before >= 1f && q_throttle < 1f)
                logger.LogInfo($"[Ascent] disposal: max-Q throttle-down starts at q {q_kpa:n1} kPa, alt {current_altitude_km:n1} km");
            else if (bucket_before < 1f && q_throttle >= 1f)
                logger.LogInfo($"[Ascent] disposal: max-Q throttle-down over at q {q_kpa:n1} kPa, alt {current_altitude_km:n1} km");
            wanted_throttle = settings.max_throttle.V * hold_throttle * q_throttle;
            LogFlightCalibration(vessel, now);
            current_vessel.SetThrottle(wanted_throttle);

            // periapsis cap: stop so stage 1 still comes back down
            if (disposal_hold && pe_km >= settings.disposal_pe_km.V)
            {
                logger.LogInfo($"[Ascent] disposal: MECO at Pe cap {pe_km:n1} km, Ap {ap_km:n1} km, alt {current_altitude_km:n1} km");
                StartSeparation(now, "MECO at Pe cap - separating");
                return;
            }

            // Lost control (nose far off the airflow in thick air): say so, once. It is not stage 1
            // running dry, and staging would only make it worse.
            if (AirState(vessel, out double lc_path, out double lc_q) && lc_q > 1 &&
                System.Math.Abs(nose_elevation - lc_path) > 30)
            {
                if (!loss_of_control_logged)
                    logger.LogWarning($"[Ascent] disposal: loss of control at alt {current_altitude_km:n1} km - nose {nose_elevation:n1}°, " +
                                      $"path {lc_path:n1}°, q {lc_q:n1} kPa, mass {vessel.totalMass:n1} t");
                loss_of_control_logged = true;
                disposal_phase = "Loss of control";
            }

            // Stage 1 dry? Only once its engines have given thrust for 5 s off the pad, and then
            // either the vessel is down to stage 1's burnout mass, or the lit engines give no thrust
            // at a commanded throttle. (Was: "Ap/Pe stopped rising for 3 s". On 2026-11-13 that
            // fired on the pad before ignition - then staged everything up to the separator - and
            // after each loss of control, with 55-63 t on board against a 26 t burnout mass.)
            var last_phase = stage1_model != null && stage1_model.valid ? stage1_model.phases[stage1_model.phases.Count - 1] : null;
            bool burnt_in = first_thrust_ut >= 0 && now - first_thrust_ut > 5 && !vessel.Landed;
            bool at_burnout_mass = last_phase != null && vessel.totalMass <= last_phase.endMass * 1.03;
            bool no_thrust = wanted_throttle > 0.05f && thrust_kN < 0.5;
            if (burnt_in && (at_burnout_mass || no_thrust)) dry_time += Time.deltaTime;
            else dry_time = 0;
            if (dry_time > 2)
            {
                dry_time = 0;
                if (!at_burnout_mass && stage1_model != null && stage1_model.phases.Count > 1)
                {
                    // flamed out between stage 1's own burn stages (e.g. boosters, then core): stage once
                    if (now >= next_inner_stage_ut)
                    {
                        next_inner_stage_ut = now + 3;
                        logger.LogInfo($"[Ascent] disposal: no thrust at {vessel.totalMass:n1} t (stage-1 burnout {last_phase?.endMass:n1} t) - staging to the next stage-1 burn");
                        vessel.ActivateNextStage();
                    }
                }
                else
                {
                    logger.LogInfo($"[Ascent] disposal: stage 1 dry at Ap {ap_km:n1} Pe {pe_km:n1} km, alt {current_altitude_km:n1} km, " +
                                   $"mass {vessel.totalMass:n1} t (burnout {last_phase?.endMass:n1} t), thrust {thrust_kN:n0} kN");
                    StartSeparation(now, "Stage 1 dry - separating");
                    return;
                }
            }
        }

        // Whether any engine on board is lit, and the thrust (kN) the lit ones give right now.
        bool EnginesBurning(out double thrust_kN)
        {
            thrust_kN = 0;
            bool lit = false;
            // same engine list and fields BurndV.Compute_Thrust reads
            var engines = current_vessel.VesselComponent?.VesselDeltaV?.EngineInfo;
            if (engines == null) return false;
            foreach (var info in engines)
            {
                var engine = info?.Engine;
                if (engine == null || !engine.EngineIgnited) continue;
                lit = true;
                thrust_kN += engine.FinalThrustValue;
            }
            // Also ask the parts themselves (as Stage1Burn does): "nothing lit" makes the pad
            // ignition stage once, so it must not come from a stale or partial engine list.
            if (!lit)
            {
                var parts = current_vessel.VesselComponent?.SimulationObject?.PartOwner?.Parts;
                if (parts != null)
                    foreach (var p in parts)
                        if (p.TryGetModule<PartComponentModule_Engine>(out var module) && module.EngineIgnited)
                            return true;
            }
            return lit;
        }

        bool separated;
        float hold_throttle = 1f;
        bool ap_burn;
        double nose_elevation;
        readonly K2D2.Lift.Stage1.Stage1Guidance.PitchSmoother pitch_smoother = new();

        // Where the nose actually points: degrees above the local horizon.
        double NoseElevationDeg()
        {
            var vehicle = current_vessel.VesselVehicle;
            if (vehicle == null) return wanted_elevation;
            Rotation rot = Rotation.Reframed(current_vessel.GetRotation(), vehicle.Up.coordinateSystem);
            Vector3d nose = (rot.localRotation * Vector3.up).normalized;
            Vector3d up = vehicle.Up.vector.normalized;
            return System.Math.Asin(System.Math.Max(-1, System.Math.Min(1, Vector3d.Dot(nose, up)))) * 180 / System.Math.PI;
        }

        // Flight path angle in the launch plane, signed: built in the same HorizonUp frame and from
        // the same Euler(-elevation, heading) convention applyDirection steers with, so above 90
        // means leaning back against the heading.
        bool LaunchPlanePath(out double pathDeg)
        {
            pathDeg = 90;
            var vehicle = current_vessel.VesselVehicle;
            var telemetry = SASTool.getTelemetry();
            if (vehicle == null || telemetry == null) return false;
            var frame = telemetry.HorizonUp.coordinateSystem;
            Vector3d sv = Vector.Reframed(vehicle.SurfaceVelocity, frame).vector;
            if (sv.magnitude < 10) return false;
            Vector3d along = QuaternionD.Euler(0, settings.heading.V, 0) * Vector3d.forward;
            Vector3d up = QuaternionD.Euler(-90, settings.heading.V, 0) * Vector3d.forward;
            pathDeg = System.Math.Atan2(Vector3d.Dot(sv, up), Vector3d.Dot(sv, along)) * 180 / System.Math.PI;
            return true;
        }

        // Surface-relative flight path angle (deg above the horizon) and dynamic pressure (kPa).
        bool AirState(VesselComponent vessel, out double pathDeg, out double qKPa) => AirState(vessel, out pathDeg, out qKPa, out _);

        bool AirState(VesselComponent vessel, out double pathDeg, out double qKPa, out double mach)
        {
            pathDeg = 90; qKPa = 0; mach = 0;
            var vehicle = current_vessel.VesselVehicle;
            if (vehicle == null) return false;
            var up_coords = vehicle.Up.coordinateSystem;
            Vector3d sv = Vector.Reframed(vehicle.SurfaceVelocity, up_coords).vector;
            Vector3d up = vehicle.Up.vector.normalized;
            double speed = sv.magnitude;
            if (speed < 5) return false;
            pathDeg = System.Math.Asin(System.Math.Max(-1, System.Math.Min(1, Vector3d.Dot(sv, up) / speed))) * 180 / System.Math.PI;
            var body = vessel.mainBody;
            double alt = vessel.AltitudeFromSeaLevel;
            if (body.hasAtmosphere && alt < body.atmosphereDepth)
            {
                double p = body.GetPressure(alt);
                double d = body.GetDensity(p, body.GetTemperature(alt));
                qKPa = 0.0005 * d * speed * speed;
                double a = d > 0 ? AeroUtilities.GetSoundSpeed(body.atmosphereAdiabaticIndex, p, d) : 0;
                mach = a > 0 ? speed / a : 0;
            }
            return true;
        }

        void StartSeparation(double now, string phase)
        {
            // never stage stage 1 away on the ground
            if (current_vessel.VesselComponent != null && current_vessel.VesselComponent.Landed)
            {
                logger.LogWarning($"[Ascent] disposal: refused to separate on the ground ({phase})");
                return;
            }
            disposal_phase = phase;
            disposal_cutoff = true;
            separation_attempts = 0;
            next_separation_ut = now + 0.3;
            current_vessel.SetThrottle(0);
            // a stage with no separator mark: just stage once
            if (stage1_separator == null)
            {
                current_vessel.VesselComponent.ActivateNextStage();
                separated = true;
                disposal_cutoff = false;
            }
        }

        public override void Update()
        {
            if (settings.disposal_enabled.V)
            {
                UpdateDisposal();
                return;
            }
            if (current_vessel.VesselComponent != null)
                LogFlightCalibration(current_vessel.VesselComponent, GeneralTools.Current_UT);
            UpdateNormal();
        }

        // The standard Lift ascent: profile pitch, throttle eased off as apoapsis nears the target.
        void UpdateNormal()
        {
            IntegrateRoll();
            UpdateRollProgram();
            applyDirection();
            finished = false;
            float remaining_Ap = settings.destination_Ap_km.V - ap_km;
            if (remaining_Ap <= settings.end_ascent_error)
            {
                finished = true;
                return;
            }
            else
            {
                if (delta_ap_per_second <= 0)
                {
                    wanted_throttle = settings.max_throttle.V;
                }
                else
                {
                    wanted_throttle = remaining_Ap / delta_ap_per_second;
                    if (wanted_throttle > settings.max_throttle.V)
                        wanted_throttle = settings.max_throttle.V;
                }

                current_vessel.SetThrottle(wanted_throttle);
            }
        }


    }
}
