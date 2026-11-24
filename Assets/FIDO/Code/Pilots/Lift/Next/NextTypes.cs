using System;
using KSP.Sim.impl;

namespace Fido.Lift.Next
{
    // Plain-data types passed between the New lift system's layers (see the NN ascent report,
    // Section 3.1). Nothing here holds a game object, so a policy or the shield can be run and
    // tested from plain numbers - in the simulator, an offline trainer or a unit test.

    /// <summary>
    /// The planet of origin, read once at the start of a run (report Section 4.4). Every network
    /// feature is measured against these values, never against Kerbin constants.
    /// </summary>
    public struct BodyDescriptor
    {
        public string name;
        public double mu, radius, g0;
        public bool hasAtmosphere;
        public double atmosphereDepth;
        public double equatorialSpeed;   // m/s, signed (negative = inverse rotation)
        public double targetApR;         // m from the centre
        public double orbitSpeedAtTarget; // circular speed at the target apoapsis, m/s

        public static BodyDescriptor From(CelestialBodyComponent body, double targetApKm)
        {
            var d = new BodyDescriptor();
            if (body == null) return d;
            d.name = body.Name;
            d.mu = body.gravParameter;
            d.radius = body.radius;
            d.g0 = d.mu / (d.radius * d.radius);
            d.hasAtmosphere = body.hasAtmosphere;
            d.atmosphereDepth = body.hasAtmosphere ? body.atmosphereDepth : 0;
            d.equatorialSpeed = Fido.Constellation.OrbitMath.EquatorialSurfaceSpeed(body);
            d.targetApR = d.radius + targetApKm * 1000.0;
            d.orbitSpeedAtTarget = Math.Sqrt(d.mu / d.targetApR);
            return d;
        }

        public bool Valid => mu > 0 && radius > 0;
    }

    /// <summary>One snapshot of the vessel, sampled once per control tick.</summary>
    public struct AscentObservation
    {
        public bool valid;
        public double ut;
        public bool landed;

        // position and orbital velocity
        public double altitude;   // m above sea level
        public double r;          // m from the centre
        public double vr, vt;     // radial and horizontal orbital speed, m/s
        public double apR, peR;   // apoapsis / periapsis radius (apR = +inf when escaping)

        // relative to the air
        public double airSpeed;   // surface speed, m/s
        public double speedRate;  // d(airSpeed)/dt, m/s², filtered (behaviour 1: never reduce speed)
        public bool hasPath;      // airSpeed high enough for the path angle to mean anything
        public double pathDeg;    // flight path angle in the launch plane, deg (above 90 = leaning back)
        public double airPathDeg; // surface flight path angle above the horizon, deg (any heading)
        public double horizontalSpeed; // surface speed along the ground, m/s
        public double maxQSoFar;  // highest q this run - tells the policy max-Q is behind it
        public double qKPa;       // dynamic pressure
        public double mach;
        public double heat;       // sqrt(rho) * v^3, the optimiser's heating proxy

        // vehicle
        public double noseDeg;    // where the nose points, deg above the horizon
        // sideways (2026-11-14: the 83 t vessel veered off sideways twice and nothing logged it)
        public double sideslipDeg;   // nose off the flight path sideways (+ = right of it), deg
        public double trackOffDeg;   // ground track off the launch heading (+ = right), deg
        public double noseHdgOffDeg; // nose heading off the launch heading (+ = right), deg
        public double rollRateDegS;  // spin about the nose axis
        public float ctlPitch, ctlYaw, ctlRoll; // control inputs the vessel is getting (SAS output), -1..1
        public double mass;       // t
        public bool enginesLit;
        public double thrustKN;   // thrust the lit engines give right now
        public double fullThrustKN; // estimated thrust at full throttle here (0 = not known yet)
        public double throttle;   // throttle currently set on the vessel

        public double ApKm(double radius) => double.IsInfinity(apR) ? 1e9 : (apR - radius) / 1000;
        public double PeKm(double radius) => (peR - radius) / 1000;
    }

    /// <summary>What a policy or behaviour asks for: two setpoints, nothing else.</summary>
    public struct AscentCommand
    {
        public double elevationDeg; // wanted pitch above the horizon, along the launch heading
        public float throttle;      // 0..1 of full thrust

        public AscentCommand(double elevationDeg, float throttle)
        {
            this.elevationDeg = elevationDeg;
            this.throttle = throttle;
        }

        public bool Finite => !double.IsNaN(elevationDeg) && !double.IsInfinity(elevationDeg) &&
                              !float.IsNaN(throttle) && !float.IsInfinity(throttle);
    }

    /// <summary>What the shield did to the last command (LIFT INFO and the [Next] log).</summary>
    public struct ShieldReport
    {
        public bool rejected;        // the command was NaN/infinite - the last good one was kept
        public bool rateLimited;     // the pitch smoother slowed the change
        public bool lossOfControl;   // nose more than 30° off the airflow in thick air
        public bool headingLimited;  // heading correction held back (rate or sideways AoA)

        public string Flags()
        {
            var f = "";
            if (rejected) f += "REJECT ";
            if (rateLimited) f += "RATE ";
            if (headingLimited) f += "HDG ";
            if (lossOfControl) f += "LOC ";
            return f.Length == 0 ? "-" : f.TrimEnd();
        }
    }
}
