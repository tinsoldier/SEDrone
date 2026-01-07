using VRage.Game.ModAPI.Ingame.Utilities;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Defines the role this grid plays in the formation.
    /// </summary>
    public enum GridRole
    {
        Drone,      // Follower - receives commands, maintains formation
        Leader      // Commander - broadcasts position, controls formation
    }

    public enum OrientationMode
    {
        HorizonLock,  // Keep roll locked to horizon (on-foot style)
        Free,         // No roll locking (jetpack style)
        Limited       // Horizon lock with max tilt limit
    }

    public class DroneConfig
    {
        // === Role Configuration ===
        public GridRole Role { get; set; } = GridRole.Drone;
        public string IGCChannel { get; set; } = "FORMATION_ALPHA";  // Channel for formation comms
        public bool RefHackMode { get; set; } = false;

        // === Target Identification ===
        public string TargetGridName { get; set; } = "";
        public long TargetEntityId { get; set; } = 0;  // Alternative: direct ID

        // === Station Keeping ===
        // Offset in target-local coordinates
        // X = right (+) / left (-)
        // Y = up (+) / down (-)
        // Z = forward (+) / backward (-)
        public Vector3D StationOffset { get; set; } = new Vector3D(30, 30, -30);

        // === Dynamic Formation ===
        // When enabled, drones auto-arrange in formation rather than using fixed StationOffset
        public bool DynamicFormation { get; set; } = false;
        public double FormationRadius { get; set; } = 40.0;       // Arc radius (meters)
        public double FormationBackOffset { get; set; } = -20.0;  // Distance behind leader (negative Z)
        public double FormationVerticalOffset { get; set; } = 5.0; // Height above leader (Y)
        public double FormationRotation { get; set; } = 180.0;    // Rotation in degrees (0=front, 180=behind)

        // === Flight Parameters ===
        public double MaxSpeed { get; set; } = 100.0;           // Hard speed cap (m/s)
        public double ApproachSpeed { get; set; } = 60.0;       // Speed when distant (m/s)
        public double StationRadius { get; set; } = 2.0;        // "Close enough" tolerance (m)
        public double HoverAltitude { get; set; } = 30.0;       // Ground-relative height (m)
        public double MinTerrainClearance { get; set; } = 10.0; // Minimum ground clearance (m)
        public double MaxTiltAngle { get; set; } = 20.0;        // Max pitch from level (degrees)
        public double BrakingSafetyMargin { get; set; } = 0.8;   // Braking distance factor (0.0-1.0)
        public OrientationMode OrientationMode { get; set; } = OrientationMode.HorizonLock;

        // === Docking Parameters ===
        public double DockingApproachSpeed { get; set; } = 60.0;   // Speed during waypoint approach (m/s)
        public double DockingFinalSpeed { get; set; } = 5.0;       // Speed during final alignment (m/s)
        public double DockingLockSpeed { get; set; } = 2;        // Speed during lock attempt (m/s)

        // === PID Tuning ===
        public PIDGains PositionPID { get; set; } = new PIDGains(2.0, 0.1, 0.8, 50);
        public PIDGains AltitudePID { get; set; } = new PIDGains(3.0, 0.2, 1.0, 30);
        // Orientation: Tuned for powerful gyros
        public PIDGains OrientationPID { get; set; } = new PIDGains(5.0, 1.0, 0.5, 15.0);

        // === Thrust Control ===
        public ThrustConfig ThrustConfig { get; set; } = new ThrustConfig();

        // === Potential Field Avoidance ===
        public PotentialFieldConfig FieldConfig { get; set; } = new PotentialFieldConfig();

        // === Timing ===
        public int UpdateFrequency { get; set; } = 10;          // Updates per second (1, 10, 100)
        public double PredictionTime { get; set; } = 0.5;       // How far ahead to predict target (seconds)

        /// <summary>
        /// Parses configuration from Custom Data in INI format.
        /// </summary>
        /// <param name="customData">The Custom Data string from the Programmable Block</param>
        /// <returns>A populated DroneConfig instance</returns>
        public static DroneConfig Parse(string customData)
        {
            var config = new DroneConfig();
            var ini = new MyIni();

            if (!ini.TryParse(customData))
            {
                // Return defaults if parsing fails
                return config;
            }

            // === Role Section ===
            string roleStr = ini.Get("Role", "Mode").ToString("Drone");
            config.Role = roleStr.ToUpperInvariant() == "LEADER" ? GridRole.Leader : GridRole.Drone;
            config.IGCChannel = ini.Get("Role", "Channel").ToString(config.IGCChannel);
            config.RefHackMode = ini.Get("Role", "RefHack").ToBoolean(config.RefHackMode);

            // === Target Section ===
            config.TargetGridName = ini.Get("Target", "GridName").ToString(config.TargetGridName);
            config.TargetEntityId = ini.Get("Target", "EntityId").ToInt64(config.TargetEntityId);

            // === Station Section ===
            double offsetRight = ini.Get("Station", "OffsetRight").ToDouble(config.StationOffset.X);
            double offsetUp = ini.Get("Station", "OffsetUp").ToDouble(config.StationOffset.Y);
            double offsetForward = ini.Get("Station", "OffsetForward").ToDouble(config.StationOffset.Z);
            config.StationOffset = new Vector3D(offsetRight, offsetUp, offsetForward);

            // === Formation Section ===
            config.DynamicFormation = ini.Get("Formation", "Dynamic").ToBoolean(config.DynamicFormation);
            config.FormationRadius = ini.Get("Formation", "Radius").ToDouble(config.FormationRadius);
            config.FormationBackOffset = ini.Get("Formation", "BackOffset").ToDouble(config.FormationBackOffset);
            config.FormationVerticalOffset = ini.Get("Formation", "VerticalOffset").ToDouble(config.FormationVerticalOffset);
            config.FormationRotation = ini.Get("Formation", "Rotation").ToDouble(config.FormationRotation);

            // === Flight Section ===
            config.MaxSpeed = ini.Get("Flight", "MaxSpeed").ToDouble(config.MaxSpeed);
            config.ApproachSpeed = ini.Get("Flight", "ApproachSpeed").ToDouble(config.ApproachSpeed);
            config.StationRadius = ini.Get("Flight", "StationRadius").ToDouble(config.StationRadius);
            config.HoverAltitude = ini.Get("Flight", "HoverAltitude").ToDouble(config.HoverAltitude);
            config.MinTerrainClearance = ini.Get("Flight", "MinTerrainClearance").ToDouble(config.MinTerrainClearance);
            config.MaxTiltAngle = ini.Get("Flight", "MaxTiltAngle").ToDouble(config.MaxTiltAngle);
            config.BrakingSafetyMargin = ini.Get("Flight", "BrakingSafetyMargin").ToDouble(config.BrakingSafetyMargin);
            string orientationModeStr = ini.Get("Flight", "OrientationMode").ToString(config.OrientationMode.ToString());
            config.OrientationMode = ParseOrientationMode(orientationModeStr, config.OrientationMode);

            // === Docking Section ===
            config.DockingApproachSpeed = ini.Get("Docking", "ApproachSpeed").ToDouble(config.DockingApproachSpeed);
            config.DockingFinalSpeed = ini.Get("Docking", "FinalSpeed").ToDouble(config.DockingFinalSpeed);
            config.DockingLockSpeed = ini.Get("Docking", "LockSpeed").ToDouble(config.DockingLockSpeed);

            // === PID Sections ===
            config.PositionPID = ParsePIDGains(ini, "PositionPID", config.PositionPID);
            config.AltitudePID = ParsePIDGains(ini, "AltitudePID", config.AltitudePID);
            config.OrientationPID = ParsePIDGains(ini, "OrientationPID", config.OrientationPID);

            // === Thrust Section ===
            config.ThrustConfig = ParseThrustConfig(ini, config.ThrustConfig);

            // === Potential Field Section ===
            config.FieldConfig = ParsePotentialFieldConfig(ini, config.FieldConfig);

            // === Advanced Section ===
            config.UpdateFrequency = ini.Get("Advanced", "UpdateFrequency").ToInt32(config.UpdateFrequency);
            config.PredictionTime = ini.Get("Advanced", "PredictionTime").ToDouble(config.PredictionTime);

            return config;
        }

        /// <summary>
        /// Parses PID gains from a specific INI section.
        /// </summary>
        private static PIDGains ParsePIDGains(MyIni ini, string section, PIDGains defaults)
        {
            return new PIDGains(
                ini.Get(section, "Kp").ToDouble(defaults.Kp),
                ini.Get(section, "Ki").ToDouble(defaults.Ki),
                ini.Get(section, "Kd").ToDouble(defaults.Kd),
                ini.Get(section, "IntegralLimit").ToDouble(defaults.IntegralLimit)
            );
        }

        /// <summary>
        /// Parses thrust configuration from INI.
        /// </summary>
        private static ThrustConfig ParseThrustConfig(MyIni ini, ThrustConfig defaults)
        {
            var config = new ThrustConfig();
            
            // Axis authority
            config.LateralAuthority = ini.Get("Thrust", "LateralAuthority").ToDouble(defaults.LateralAuthority);
            config.VerticalAuthority = ini.Get("Thrust", "VerticalAuthority").ToDouble(defaults.VerticalAuthority);
            config.LongitudinalAuthority = ini.Get("Thrust", "LongitudinalAuthority").ToDouble(defaults.LongitudinalAuthority);
            
            // Hover pad handling
            config.ExcludeHoverPads = ini.Get("Thrust", "ExcludeHoverPads").ToBoolean(defaults.ExcludeHoverPads);
            
            string patternsStr = ini.Get("Thrust", "HoverPadPatterns").ToString("");
            if (!string.IsNullOrEmpty(patternsStr))
            {
                config.HoverPadPatterns.Clear();
                foreach (var pattern in patternsStr.Split(','))
                {
                    string trimmed = pattern.Trim();
                    if (!string.IsNullOrEmpty(trimmed))
                        config.HoverPadPatterns.Add(trimmed);
                }
            }
            
            // Braking
            config.AccelerationFactor = ini.Get("Thrust", "AccelerationFactor").ToDouble(defaults.AccelerationFactor);
            
            return config;
        }

        /// <summary>
        /// Parses potential field configuration from INI.
        /// </summary>
        private static PotentialFieldConfig ParsePotentialFieldConfig(MyIni ini, PotentialFieldConfig defaults)
        {
            var config = new PotentialFieldConfig();

            config.Enabled = ini.Get("PotentialField", "Enabled").ToBoolean(defaults.Enabled);
            config.InfluenceRadius = ini.Get("PotentialField", "InfluenceRadius").ToDouble(defaults.InfluenceRadius);
            config.RepulsionStrength = ini.Get("PotentialField", "RepulsionStrength").ToDouble(defaults.RepulsionStrength);
            config.MaxRepulsionSpeed = ini.Get("PotentialField", "MaxRepulsionSpeed").ToDouble(defaults.MaxRepulsionSpeed);
            config.DroneMultiplier = ini.Get("PotentialField", "DroneMultiplier").ToDouble(defaults.DroneMultiplier);
            config.TerrainMultiplier = ini.Get("PotentialField", "TerrainMultiplier").ToDouble(defaults.TerrainMultiplier);
            config.RandomnessFactor = ini.Get("PotentialField", "RandomnessFactor").ToDouble(defaults.RandomnessFactor);
            config.UpdateInterval = ini.Get("PotentialField", "UpdateInterval").ToInt32(defaults.UpdateInterval);
            config.MaxObstacles = ini.Get("PotentialField", "MaxObstacles").ToInt32(defaults.MaxObstacles);

            return config;
        }

        private static OrientationMode ParseOrientationMode(string value, OrientationMode fallback)
        {
            if (string.IsNullOrWhiteSpace(value))
                return fallback;

            switch (value.Trim().ToUpperInvariant())
            {
                case "HORIZONLOCK":
                case "HORIZON":
                case "LEVEL":
                    return OrientationMode.HorizonLock;
                case "FREE":
                case "JETPACK":
                    return OrientationMode.Free;
                case "LIMITED":
                case "MAXTILT":
                    return OrientationMode.Limited;
                default:
                    return fallback;
            }
        }

        /// <summary>
        /// Generates a default configuration template for Custom Data.
        /// </summary>
        public static string GenerateTemplate()
        {
            return @"[Role]
; Mode: Leader or Drone
Mode=Drone
; IGC channel for formation communication
Channel=FORMATION_ALPHA
; Refhack mode: leader PB controls docked drones directly
RefHack=false

[Target]
; Name of the grid to follow (partial match supported)
GridName=Player Rover
; Alternative: EntityId=123456789

[Station]
; Offset from target in LOCAL coordinates
; Right(+)/Left(-), Up(+)/Down(-), Forward(+)/Back(-)
; Only used when Formation.Dynamic=false
OffsetRight=30
OffsetUp=30
OffsetForward=-30

[Formation]
; Dynamic formation: drones auto-arrange in half-circle behind leader
Dynamic=true
; Arc radius in meters (distance from formation center)
Radius=40
; Distance behind leader center (negative = behind)
BackOffset=-20
; Height above leader
VerticalOffset=5
; Rotation of formation arc in degrees (0=front, 180=behind)
Rotation=180

[Flight]
; Speed limits (m/s)
MaxSpeed=100
ApproachSpeed=60
; Distance thresholds (m)
PrecisionRadius=20
StationRadius=2
; Hover height above terrain (m)
HoverAltitude=30
; Minimum ground clearance - formation point lifted if terrain rises (m)
MinTerrainClearance=10
; Maximum tilt angle from level (degrees)
MaxTiltAngle=20
; Orientation mode: HorizonLock, Free, or Limited
OrientationMode=HorizonLock
; Braking safety margin (0.0-1.0)
; How much of braking distance to use as formation tolerance at speed
; 0.0 = tight formation (current behavior, may overshoot)
; 1.0 = always trail by full braking distance (very safe, may feel laggy)
BrakingSafetyMargin=0.8

[Docking]
; Speed during waypoint approach (m/s)
ApproachSpeed=40
; Speed during final alignment (m/s)
FinalSpeed=5.0
; Speed during connector lock attempt (m/s)
LockSpeed=2.0

[PositionPID]
Kp=2.0
Ki=0.1
Kd=0.8
IntegralLimit=50

[AltitudePID]
Kp=3.0
Ki=0.2
Kd=1.0
IntegralLimit=30

[OrientationPID]
; Tuned for powerful gyros
Kp=5.0
Ki=1.0
Kd=0.5
IntegralLimit=15

[Thrust]
; Axis authority: 0.0 = defer to hover pads, 1.0 = full script control
LateralAuthority=1.0
VerticalAuthority=0.0
LongitudinalAuthority=1.0
; Hover pad detection
ExcludeHoverPads=true
HoverPadPatterns=Hover
; Braking safety factor (0.0-1.0, lower = more conservative)
AccelerationFactor=0.7

[PotentialField]
; Enable/disable obstacle avoidance
Enabled=true
; Maximum distance to detect obstacles (meters)
InfluenceRadius=100
; Base repulsion strength (higher = stronger avoidance)
RepulsionStrength=500
; Maximum avoidance velocity (m/s)
MaxRepulsionSpeed=15
; Multiplier for avoiding other drones (>1 = more aggressive)
DroneMultiplier=1.5
; Multiplier for ground avoidance (>1 = more aggressive)
TerrainMultiplier=2.0
; Per-drone randomness to break deadlocks (0-1)
RandomnessFactor=0.2
; Obstacle cache update interval (ticks, 6 = ~10Hz)
UpdateInterval=6
; Max obstacles to consider (limits computation)
MaxObstacles=10

[Advanced]
; Updates per second: 1, 10, or 100
UpdateFrequency=10
; How far ahead to predict target position (seconds)
PredictionTime=0.5
";
        }
    }

    /// <summary>
    /// Holds PID controller gain values.
    /// </summary>
    public struct PIDGains
    {
        public double Kp;             // Proportional gain
        public double Ki;             // Integral gain
        public double Kd;             // Derivative gain
        public double IntegralLimit;  // Anti-windup clamp (max integral accumulation)
        public double Deadband;       // Error below which integral stops accumulating
        public double IntegralDecay;  // Decay factor per tick (0.995 = 0.5% decay)

        public PIDGains(double kp, double ki, double kd,
                        double integralLimit = 10.0,
                        double deadband = 0.3,
                        double integralDecay = 0.995)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            IntegralLimit = integralLimit;
            Deadband = deadband;
            IntegralDecay = integralDecay;
        }
    }

    /// <summary>
    /// Configuration for potential field obstacle avoidance.
    /// </summary>
    public class PotentialFieldConfig
    {
        /// <summary>
        /// Whether potential field avoidance is enabled.
        /// </summary>
        public bool Enabled { get; set; } = true;

        /// <summary>
        /// Maximum distance to detect obstacles (meters).
        /// Obstacles beyond this range are ignored.
        /// </summary>
        public double InfluenceRadius { get; set; } = 100;

        /// <summary>
        /// Base repulsion strength constant.
        /// Higher values = stronger avoidance at any distance.
        /// </summary>
        public double RepulsionStrength { get; set; } = 500;

        /// <summary>
        /// Maximum velocity added by repulsion (m/s).
        /// Caps the avoidance response to prevent erratic movement.
        /// </summary>
        public double MaxRepulsionSpeed { get; set; } = 15;

        /// <summary>
        /// Multiplier for drone-type obstacles.
        /// Values > 1 make drones avoid each other more aggressively.
        /// </summary>
        public double DroneMultiplier { get; set; } = 4.0;

        /// <summary>
        /// Multiplier for terrain obstacles.
        /// Values > 1 provide stronger ground avoidance.
        /// </summary>
        public double TerrainMultiplier { get; set; } = 0;

        /// <summary>
        /// Per-drone randomness factor (0-1).
        /// Creates asymmetric responses to help break deadlocks.
        /// 0 = identical behavior, 1 = up to +/-50% variation.
        /// </summary>
        public double RandomnessFactor { get; set; } = 0.2;

        /// <summary>
        /// How often to update obstacle cache (ticks).
        /// Higher values reduce computation but may miss fast-moving obstacles.
        /// </summary>
        public int UpdateInterval { get; set; } = 6;

        /// <summary>
        /// Maximum number of obstacles to consider per update.
        /// Limits computation for dense environments.
        /// </summary>
        public int MaxObstacles { get; set; } = 10;
    }
}
