using System;
using System.Linq;
using K2D2.Constellation;
using K2D2.KSPService;
using K2D2.Lift;
using K2D2.Lift.Stage1;
using K2D2.Node;
using KSP.Modules;
using KSP.Sim;
using KSP.Sim.impl;
using KTools;
using UnityEngine;

namespace K2D2.Mission
{
    public enum StepResult { Running, Done, Failed, Paused }

    /// <summary>
    /// Runs one mission step by driving an existing pilot. Prepare() copies the step's settings into
    /// the normal K2-D2 settings (so the Advanced tabs show exactly what will fly), Start() sets the
    /// pilot going and Update() reports how it is doing.
    /// </summary>
    public abstract class StepRunner
    {
        public MissionStepData step;
        public MissionPlanData mission; // the whole plan, for steps that look ahead (transfers aim for a later Capture)
        public string status = "";

        protected static KSPVessel Vessel => K2D2_Plugin.Instance.current_vessel;

        public virtual void Prepare() { }

        /// <summary>A reason the step can't start right now, or null.</summary>
        public virtual string Check() => null;

        /// <summary>Warnings to show before Go (never block), or null.</summary>
        public virtual string Advice() => null;

        public abstract void Start();
        public abstract StepResult Update();
        public virtual void Stop() { }

        /// <summary>The tab that flies this step, for "take manual control".</summary>
        public virtual string Tab => null;
    }

    class QuicksaveRunner : StepRunner
    {
        // the pilot writes the plan (with this step marked done) before calling Save
        public Action save;

        public override void Start() => save?.Invoke();

        public override StepResult Update()
        {
            status = "Saved";
            return StepResult.Done;
        }
    }

    // ---------------------------------------------------------------- Lift

    class LaunchRunner : StepRunner
    {
        static LiftPilot Lift => LiftPilot.Instance;
        bool started_disposal;

        public override string Tab => "lift";

        public override void Prepare()
        {
            var s = Lift.settings;
            s.destination_Ap_km.V = Mathf.RoundToInt(step.GetFloat("ap_km"));
            s.polar_inclination.V = step.GetFloat("inclination");
            s.polar_south.V = step.GetBool("south");
            // the step's "optimiser" / "pe_cap_km" (names kept so saved plans load) were the
            // Optimiser's stage-1 disposal until 2026-11-15; now the lift system's stage-1 apoapsis burn
            s.next_s1_ap_burn.V = step.GetBool("optimiser");
            s.next_s1_pe_km.V = step.GetFloat("pe_cap_km");
            s.auto_circularize.V = true;
            s.coasting_warp.V = true;

            var vessel = Vessel?.VesselComponent;
            var body = vessel?.mainBody;
            if (body != null && vessel.Landed)
            {
                double orbit_speed = OrbitMath.CircularSpeed(body.gravParameter, body.radius + s.destination_Ap_km.V * 1000.0);
                OrbitMath.LaunchHeading(s.polar_inclination.V, vessel.Latitude, orbit_speed,
                    OrbitMath.EquatorialSurfaceSpeed(body), !s.polar_south.V, out double heading);
                s.heading.V = (float)heading;
            }
        }

        public override string Check()
        {
            var vessel = Vessel?.VesselComponent;
            if (vessel == null) return "No vessel";
            if (step.GetBool("optimiser") && Stage1Model.FindMarkedSeparator(vessel) == null)
                return "Stage-1 disposal is on but no separator is marked 'Stage 1 separator' - mark one in the VAB or turn disposal off";
            return null;
        }

        public override void Start()
        {
            started_disposal = Lift.settings.next_s1_ap_burn.V;
            Lift.StartAt(LiftPilot.LiftStatus.Ascent);
        }

        public override StepResult Update()
        {
            var lift = Lift;
            if (!lift.isRunning)
            {
                status = string.IsNullOrEmpty(lift.end_status) ? "Lift stopped" : lift.end_status;
                return StepResult.Failed;
            }
            status = lift.status.ToString();
            // Launch ends once stage 1 is off (or, without disposal, once clear of the atmosphere):
            // Lift has moved on to Adjust / Circularize, which the Circularise step flies.
            if (lift.status == LiftPilot.LiftStatus.Adjust || lift.status == LiftPilot.LiftStatus.Circularize)
            {
                string problem = lift.Stage1?.problem;
                lift.isRunning = false;
                if (!string.IsNullOrEmpty(problem))
                {
                    status = problem;
                    return StepResult.Failed;
                }
                status = started_disposal ? "Stage 1 separated" : "Above the atmosphere";
                return StepResult.Done;
            }
            return StepResult.Running;
        }

        public override void Stop()
        {
            if (Lift.isRunning) Lift.isRunning = false;
        }
    }

    class CirculariseRunner : StepRunner
    {
        static LiftPilot Lift => LiftPilot.Instance;

        public override string Tab => "lift";

        public override void Prepare()
        {
            Lift.settings.auto_circularize.V = true;
        }

        public override void Start() => Lift.StartAt(LiftPilot.LiftStatus.Adjust);

        public override StepResult Update()
        {
            var lift = Lift;
            if (lift.isRunning)
            {
                status = lift.status.ToString();
                return StepResult.Running;
            }
            status = string.IsNullOrEmpty(lift.end_status) ? "Lift stopped" : lift.end_status;
            return lift.result_ok ? StepResult.Done : StepResult.Failed;
        }

        public override void Stop()
        {
            if (Lift.isRunning) Lift.isRunning = false;
        }
    }

    // ---------------------------------------------------------------- fairings

    class FairingsRunner : StepRunner
    {
        ConstellationPilot.AlignStep align;
        float release_at = -1, done_at = -1;
        int released;

        public override void Start()
        {
            released = 0;
            release_at = done_at = -1;
            align = step.GetBool("radial_in")
                ? new ConstellationPilot.AlignStep(ConstellationPilot.Instance, AutopilotMode.RadialIn, "Pointing radial-in")
                : null;
            align?.Start();
            if (align == null) release_at = Time.time;
        }

        public override StepResult Update()
        {
            if (align != null)
            {
                bool lined_up;
                try { lined_up = align.Update(); }
                catch (Exception e)
                {
                    status = e.Message;
                    return StepResult.Failed;
                }
                status = "Pointing radial-in: " + align.status;
                if (!lined_up) return StepResult.Running;
                align = null;
                release_at = Time.time;
            }
            if (release_at >= 0 && done_at < 0)
            {
                released = Jettison();
                if (released == 0)
                {
                    status = "No fairings left to release";
                    return StepResult.Done;
                }
                done_at = Time.time + 3;
            }
            status = $"Released {released} fairing(s)";
            return Time.time >= done_at ? StepResult.Done : StepResult.Running;
        }

        static int Jettison()
        {
            var vessel = Vessel?.VesselComponent;
            var parts = vessel?.SimulationObject?.PartOwner?.Parts;
            if (parts == null) return 0;
            int count = 0;
            foreach (var p in parts.ToList())
            {
                if (!p.TryGetModuleData<PartComponentModule_Fairing, Data_Fairing>(out var data)) continue;
                if (data.IsDeployed.GetValue()) continue;
                var behaviour = GeneralTools.Game.SpaceSimulation.ModelViewMap.FromModel(p.SimulationObject)?.Part;
                var fairing = behaviour?.Modules.Values.OfType<Module_Fairing>().FirstOrDefault();
                if (fairing == null) continue;
                fairing.PerformJettison();
                count++;
            }
            K2D2_Plugin.logger.LogInfo($"[Mission] jettisoned {count} fairing(s)");
            return count;
        }

        public override void Stop() => SASTool.setAutoPilot(AutopilotMode.StabilityAssist);
    }

    // ---------------------------------------------------------------- Sats-tab pilots

    abstract class ConstellationRunner : StepRunner
    {
        protected static ConstellationPilot Sats => ConstellationPilot.Instance;

        public override string Tab => ConstellationUI.Code;

        public override StepResult Update()
        {
            var sats = Sats;
            if (sats.isRunning)
            {
                status = sats.step_status + (string.IsNullOrEmpty(sats.CurrentStepStatus) ? "" : " - " + sats.CurrentStepStatus);
                return StepResult.Running;
            }
            status = sats.end_status;
            if (sats.IsPaused) return StepResult.Paused;
            return sats.result_ok ? StepResult.Done : StepResult.Failed;
        }

        public override void Stop()
        {
            if (Sats.isRunning) Sats.isRunning = false;
        }
    }

    class PlaneRunner : ConstellationRunner
    {
        public override void Start() => Sats.StartPlaneChange(step.GetFloat("inclination"));
    }

    class AltitudeRunner : ConstellationRunner
    {
        public override void Prepare() => Sats.settings.target_altitude_km.V = step.GetFloat("alt_km");
        public override void Start() => Sats.StartAltitude();
    }

    class DeployRunner : ConstellationRunner
    {
        public override void Prepare()
        {
            var s = Sats.settings;
            s.target_altitude_km.V = step.GetFloat("alt_km");
            s.separation_deg.V = step.GetFloat("separation");
            s.phasing_orbits.V = step.GetInt("phasing");
            s.auto_dish.V = step.GetBool("auto_dish");
            s.move_fuel.V = step.GetBool("move_fuel");
        }

        public override string Check()
        {
            var sats = Sats;
            if (sats.IsPaused) return null; // Go resumes it
            sats.Rescan();
            if (sats.EnabledCount == 0) return "No satellites to release on this vessel";
            var planned = step.Get("body");
            var here = Vessel?.VesselComponent?.mainBody?.bodyName;
            if (!string.IsNullOrEmpty(planned) && here != null && planned != here)
                return $"This step was planned for {planned} but the vessel is orbiting {here}";
            return null;
        }

        // StartDeploy resumes a paused deployment
        public override void Start() => Sats.StartDeploy();
    }

    // ---------------------------------------------------------------- node

    class NodeRunner : StepRunner
    {
        public override string Tab => "node";

        public override string Check()
        {
            var nodes = Vessel?.VesselComponent?.SimulationObject?.FindComponent<ManeuverPlanComponent>()?.GetNodes();
            return nodes == null || nodes.Count == 0 ? "No manoeuvre node to execute" : null;
        }

        public override void Start() => NodeExPilot.Instance.Start();

        public override StepResult Update()
        {
            if (NodeExPilot.Instance.isRunning)
            {
                status = NodeExPilot.Instance.ApiStatus();
                return StepResult.Running;
            }
            status = "Node executed";
            return StepResult.Done;
        }

        public override void Stop()
        {
            if (NodeExPilot.Instance.isRunning) NodeExPilot.Instance.Stop();
        }
    }
}
