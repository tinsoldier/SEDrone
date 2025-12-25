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

    public class DroneConfig
    {
        // === Role Configuration ===
        public GridRole Role { get; set; } = GridRole.Drone;
        public string IGCChannel { get; set; } = "FORMATION_ALPHA";  // Channel for formation comms

        // === Target Identification ===
        public string TargetGridName { get; set; } = "";
        public long TargetEntityId { get; set; } = 0;  // Alternative: direct ID

        // === Station Keeping ===
        // Offset in target-local coordinates
        // X = right (+) / left (-)
        // Y = up (+) / down (-)  
        // Z = forward (+) / backward (-)
        public Vector3D StationOffset { get; set; } = new Vector3D(30, 30, -30);

        // === Flight Parameters ===
        public double MaxSpeed { get; set; } = 100.0;           // Hard speed cap (m/s)
        public double ApproachSpeed { get; set; } = 60.0;       // Speed when distant (m/s)
        public double PrecisionRadius { get; set; } = 20.0;     // Slow-down distance (m)
        public double StationRadius { get; set; } = 2.0;        // "Close enough" tolerance (m)
        public double HoverAltitude { get; set; } = 30.0;       // Ground-relative height (m)
        public double MinTerrainClearance { get; set; } = 10.0; // Minimum ground clearance (m)
        public double MaxTiltAngle { get; set; } = 20.0;        // Max pitch from level (degrees)

        // === PID Tuning ===
        public PIDGains PositionPID { get; set; } = new PIDGains(2.0, 0.1, 0.8, 50);
        public PIDGains AltitudePID { get; set; } = new PIDGains(3.0, 0.2, 1.0, 30);
        // Orientation: Tuned for powerful gyros
        public PIDGains OrientationPID { get; set; } = new PIDGains(5.0, 1.0, 0.5, 15.0);

        // === Thrust Control ===
        public ThrustConfig ThrustConfig { get; set; } = new ThrustConfig();

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

            // === Target Section ===
            config.TargetGridName = ini.Get("Target", "GridName").ToString(config.TargetGridName);
            config.TargetEntityId = ini.Get("Target", "EntityId").ToInt64(config.TargetEntityId);

            // === Station Section ===
            double offsetRight = ini.Get("Station", "OffsetRight").ToDouble(config.StationOffset.X);
            double offsetUp = ini.Get("Station", "OffsetUp").ToDouble(config.StationOffset.Y);
            double offsetForward = ini.Get("Station", "OffsetForward").ToDouble(config.StationOffset.Z);
            config.StationOffset = new Vector3D(offsetRight, offsetUp, offsetForward);

            // === Flight Section ===
            config.MaxSpeed = ini.Get("Flight", "MaxSpeed").ToDouble(config.MaxSpeed);
            config.ApproachSpeed = ini.Get("Flight", "ApproachSpeed").ToDouble(config.ApproachSpeed);
            config.PrecisionRadius = ini.Get("Flight", "PrecisionRadius").ToDouble(config.PrecisionRadius);
            config.StationRadius = ini.Get("Flight", "StationRadius").ToDouble(config.StationRadius);
            config.HoverAltitude = ini.Get("Flight", "HoverAltitude").ToDouble(config.HoverAltitude);
            config.MinTerrainClearance = ini.Get("Flight", "MinTerrainClearance").ToDouble(config.MinTerrainClearance);
            config.MaxTiltAngle = ini.Get("Flight", "MaxTiltAngle").ToDouble(config.MaxTiltAngle);

            // === PID Sections ===
            config.PositionPID = ParsePIDGains(ini, "PositionPID", config.PositionPID);
            config.AltitudePID = ParsePIDGains(ini, "AltitudePID", config.AltitudePID);
            config.OrientationPID = ParsePIDGains(ini, "OrientationPID", config.OrientationPID);

            // === Thrust Section ===
            config.ThrustConfig = ParseThrustConfig(ini, config.ThrustConfig);

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
        /// Generates a default configuration template for Custom Data.
        /// </summary>
        public static string GenerateTemplate()
        {
            return @"[Role]
; Mode: Leader or Drone
Mode=Drone
; IGC channel for formation communication
Channel=FORMATION_ALPHA

[Target]
; Name of the grid to follow (partial match supported)
GridName=Player Rover
; Alternative: EntityId=123456789

[Station]
; Offset from target in LOCAL coordinates
; Right(+)/Left(-), Up(+)/Down(-), Forward(+)/Back(-)
OffsetRight=30
OffsetUp=30
OffsetForward=-30

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
        public double Kp;           // Proportional gain
        public double Ki;           // Integral gain
        public double Kd;           // Derivative gain
        public double IntegralLimit; // Anti-windup clamp

        public PIDGains(double kp, double ki, double kd, double integralLimit = 100)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            IntegralLimit = integralLimit;
        }
    }
}