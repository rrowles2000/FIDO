using System.Collections.Generic;
using Fido.Controller;
using KSP.Sim.DeltaV;
// using KTools.UI;
using UnityEngine;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido.KSPService
{
    /// Simple class used to compute Burned DV
    public class BurndV : BaseController
    {
        public ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.SettingsFile");

        KSPVessel current_vessel;

        public BurndV()
        {
            current_vessel = FidoPlugin.Instance.current_vessel;
        }

        public float burned_dV = 0;

        public void reset()
        {
            burned_dV = 0;
        }


        public Vector3 actual_thrust;
        public float actual_dv;

        public Vector3 full_thrust;
        public float full_dv;

        public override void FixedUpdate()
        {
            burned_dV += actual_dv * Time.fixedDeltaTime;
        }

        public override void LateUpdate()
        {
            Compute_Thrust();
        }

        // it is the way engine burning is computed in KSP
        public bool Engine_Running(DeltaVEngineInfo engine_info)
        {
            return (engine_info.Engine.EngineIgnited && engine_info.Engine.RequestedMassFlow > 0f);
        }

        float compute_full_thrust(DeltaVEngineInfo engineInfo)
        {
            var partref = engineInfo.PartInfo.PartRef;
            float staticPressureAtm = partref.StaticPressureAtm;
            if (staticPressureAtm > 0f)
            {
                return engineInfo.Engine.MaxThrustOutputAtm(runningActive: false, useThrustLimiter: true, staticPressureAtm, partref.AtmosphericTemperature, partref.AtmDensity);
            }
            else if (engineInfo.RequiresAir)
            {
                return 0f;
            }
            else
            {
                return engineInfo.Engine.MaxThrustOutputVac();
            }
        }

        /// <summary>
        /// Burn time for a node - FIDO's own (2026-11-23): node Δv / acceleration (full thrust of the
        /// active engines, capped at max_accel when > 0). The game's ManeuverNodeData.BurnDuration came
        /// out ~800 s for a 360 m/s burn on a constellation carrier (it works from the wrong stage), so
        /// the burn lit 716 s before the node and burned mostly backwards. The game's time is only
        /// used when there's no thrust to work from; a disagreement beyond x1.5 is logged once per
        /// node. Returns seconds.
        /// </summary>
        public double SaneBurnDuration(KSP.Sim.Maneuver.ManeuverNodeData node, double max_accel, ref string logged_for, ILogger log)
        {
            if (node == null) return 0;
            Compute_Thrust();
            double a = full_dv;
            double stretch = max_accel > 0 && a > max_accel ? a / max_accel : 1;
            double game = node.BurnDuration * stretch;
            if (!(a > 0)) return game;
            double own = node.BurnRequiredDV / (max_accel > 0 ? System.Math.Min(a, max_accel) : a);
            bool agree = game > 0 && !double.IsNaN(game) && game <= own * 1.5 && game >= own / 1.5;
            string key = node.NodeID.ToString();
            if (!agree && logged_for != key)
            {
                logged_for = key;
                log?.LogWarning($"[Burn] game burn time {node.BurnDuration:n1} s (x{stretch:n2} cap = {game:n1} s) disagrees with Δv {node.BurnRequiredDV:n1} m/s / " +
                                $"{(max_accel > 0 ? System.Math.Min(a, max_accel) : a):n2} m/s² = {own:n1} s - FIDO uses {own:n1} s");
            }
            return own;
        }

        public void Compute_Thrust()
        {
            if (current_vessel.VesselComponent == null) return;
            VesselDeltaVComponent delta_v = current_vessel.VesselComponent.VesselDeltaV;
            if (delta_v == null) return;

            var totalMass = current_vessel.VesselComponent.totalMass;

            actual_thrust = Vector3.zero;
            full_thrust = Vector3.zero;
            List<DeltaVEngineInfo> engineInfos = delta_v.EngineInfo;
            for (int i = 0; i < engineInfos.Count; i++)
            {
                DeltaVEngineInfo engineInfo = engineInfos[i];

                Vector3 vector = ((engineInfo.Engine != null) ? engineInfo.Engine.ThrustDirRelativePartWorldSpace : (1f * Vector3.back));

                actual_thrust += vector * engineInfo.Engine.FinalThrustValue;
                full_thrust += vector * compute_full_thrust(engineInfo);
            }

            actual_dv = (float)(actual_thrust.magnitude / totalMass);
            full_dv = (float)(full_thrust.magnitude / totalMass);
        }

        // public override void onGUI()
        // {
        //     if (current_vessel.VesselComponent == null) return;
        //     VesselDeltaVComponent delta_v = current_vessel.VesselComponent.VesselDeltaV;
        //     if (delta_v == null)
        //     {
        //         UI_Tools.Error("NO VesselDeltaVComponent");
        //         return;
        //     }
        //     //List<DeltaVEngineInfo> engineInfos = delta_v.EngineInfo;

        //     var vehicle = current_vessel.VesselVehicle;
        //     if (vehicle == null) return;
        //     var mainThrottle = vehicle.mainThrottle;

        //     //UI_Tools.Console($"nb_engines {engineInfos.Count}  ");


        //     UI_Tools.Console($"mainThrottle {mainThrottle}");
        //     // UI_Tools.Console($"actual_thrust  {Tools.printVector(actual_thrust)}  ");
        //     UI_Tools.Console($"actual_dv  {actual_dv:n5}  ");


        //     // UI_Tools.Console($"full_thrust  {Tools.printVector(full_thrust)}  ");
        //     UI_Tools.Console($"full_dv  {full_dv:n5}  ");
        //     UI_Tools.Console($"burned_dV <b> {burned_dV:n5} </b>");
        //     if (GUILayout.Button("Reset"))
        //         burned_dV = 0;
        // }
    }
}
