using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Configuration for thrust control behavior.
    /// </summary>
    public class ThrustConfig
    {
        // === Axis Authority ===
        // 0.0 = Script has no control (defer to hover pads / external forces)
        // 1.0 = Script has full control
        
        /// <summary>X-axis (left/right strafing). Usually 1.0 for full control.</summary>
        public double LateralAuthority { get; set; } = 1.0;
        
        /// <summary>Y-axis (up/down). Set to 0.0 to defer to hover pads.</summary>
        public double VerticalAuthority { get; set; } = 0.0;
        
        /// <summary>Z-axis (forward/backward). Usually 1.0 for full control.</summary>
        public double LongitudinalAuthority { get; set; } = 1.0;
        
        // === Hover Pad Detection ===
        
        /// <summary>If true, exclude thrusters identified as hover pads from control.</summary>
        public bool ExcludeHoverPads { get; set; } = true;
        
        /// <summary>SubtypeId patterns that identify hover pad blocks (case-insensitive).</summary>
        public List<string> HoverPadPatterns { get; set; } = new List<string> { "Hover" };
        
        // === Braking Behavior ===
        
        /// <summary>
        /// Safety factor for braking calculations (0.0-1.0).
        /// Lower = more conservative (starts braking earlier).
        /// </summary>
        public double AccelerationFactor { get; set; } = 0.7;
        
        /// <summary>
        /// Applies axis authority to a force vector.
        /// </summary>
        public Vector3D ApplyAuthority(Vector3D force)
        {
            return new Vector3D(
                force.X * LateralAuthority,
                force.Y * VerticalAuthority,
                force.Z * LongitudinalAuthority
            );
        }
    }

    /// <summary>
    /// Manages thrust control for formation flying.
    /// Handles thrust mapping, hover pad filtering, and velocity-based movement.
    /// </summary>
    public class ThrusterController
    {
        // === Dependencies ===
        private IMyShipController _reference;
        private ThrustConfig _config;
        private Action<string> _echo;
        
        // === Thruster Hardware ===
        private List<IMyThrust> _allThrusters = new List<IMyThrust>();
        private List<IMyThrust> _controllableThrusters = new List<IMyThrust>();
        
        // === Thrust Mapping ===
        // 3x2 array: [axis][direction]
        // Axis: 0=X (left/right), 1=Y (up/down), 2=Z (forward/back)
        // Direction: 0=positive, 1=negative
        private double[,] _thrustMap = new double[3, 2];
        
        // === State ===
        private double _deltaTime = 0.016;
        private double _shipMass = 1000;
        
        /// <summary>
        /// Gets the count of controllable thrusters (excluding hover pads if configured).
        /// </summary>
        public int ThrusterCount => _controllableThrusters.Count;
        
        /// <summary>
        /// Gets the total thruster count including hover pads.
        /// </summary>
        public int TotalThrusterCount => _allThrusters.Count;
        
        /// <summary>
        /// Gets the number of thrusters excluded as hover pads.
        /// </summary>
        public int HoverPadCount => _allThrusters.Count - _controllableThrusters.Count;

        /// <summary>
        /// Gets the current ship mass used for calculations.
        /// </summary>
        public double ShipMass => _shipMass;

        /// <summary>
        /// Creates a new ThrusterController.
        /// </summary>
        /// <param name="reference">Ship controller for orientation reference</param>
        /// <param name="thrusters">All thrusters on the grid</param>
        /// <param name="config">Thrust configuration</param>
        /// <param name="echo">Optional debug output</param>
        public ThrusterController(IMyShipController reference, List<IMyThrust> thrusters, 
                                   ThrustConfig config, Action<string> echo = null)
        {
            _reference = reference;
            _allThrusters = thrusters;
            _config = config ?? new ThrustConfig();
            _echo = echo;
            
            RefreshThrustMap();
            
            // Auto-detect vertical authority: if no hover pads found, take full vertical control
            if (HoverPadCount == 0 && _config.VerticalAuthority == 0.0)
            {
                _config.VerticalAuthority = 1.0;
                _echo?.Invoke("[THRUST] No hover pads detected, enabling vertical control");
            }
        }

        /// <summary>
        /// Updates the reference controller.
        /// </summary>
        public void SetReference(IMyShipController reference)
        {
            _reference = reference;
        }

        /// <summary>
        /// Sets the time delta for this update tick.
        /// </summary>
        public void SetDeltaTime(double deltaTime)
        {
            _deltaTime = deltaTime > 0.001 ? deltaTime : 0.016;
        }

        /// <summary>
        /// Updates the ship mass for force calculations.
        /// Call this each tick or when mass changes significantly.
        /// </summary>
        public void SetShipMass(double mass)
        {
            _shipMass = mass > 0 ? mass : 1000;
        }

        /// <summary>
        /// Calculates available braking acceleration in a given world-space direction.
        /// This tells you how fast the drone can decelerate if traveling in that direction.
        /// </summary>
        /// <param name="worldVelocityDirection">Direction of travel in world space (will be normalized)</param>
        /// <returns>Braking acceleration in m/s² (always positive)</returns>
        public double GetBrakingAcceleration(Vector3D worldVelocityDirection)
        {
            if (_reference == null || worldVelocityDirection.LengthSquared() < 0.001)
                return 0;

            // Transform velocity direction to ship space
            Vector3D velocityDirShip = Vector3D.Normalize(WorldToShipDirection(worldVelocityDirection));
            
            // Braking thrust is opposite to velocity direction
            Vector3D brakingDirection = -velocityDirShip;
            
            // Get available thrust in braking direction
            Vector3D availableThrust = GetAvailableThrust(brakingDirection);
            
            // Account for gravity (helps or hurts braking depending on direction)
            Vector3D gravity = _reference.GetNaturalGravity();
            Vector3D gravityShip = WorldToShipDirection(gravity);
            Vector3D effectiveThrust = availableThrust;
            
            // Only account for gravity on vertical axis if we have authority there
            if (_config.VerticalAuthority > 0)
            {
                effectiveThrust += gravityShip * _shipMass * _config.VerticalAuthority;
            }
            
            // Project effective thrust onto braking direction
            double thrustMagnitude = Math.Abs(
                effectiveThrust.X * brakingDirection.X +
                effectiveThrust.Y * brakingDirection.Y +
                effectiveThrust.Z * brakingDirection.Z
            );
            
            // F = ma, so a = F/m
            double acceleration = thrustMagnitude / _shipMass;
            
            // Apply safety factor
            return acceleration * _config.AccelerationFactor;
        }

        /// <summary>
        /// Calculates the stopping distance from a given speed in a given direction.
        /// Uses kinematic equation: d = v² / (2a)
        /// </summary>
        /// <param name="speed">Current speed in m/s</param>
        /// <param name="worldVelocityDirection">Direction of travel in world space</param>
        /// <returns>Distance in meters required to stop</returns>
        public double GetBrakingDistance(double speed, Vector3D worldVelocityDirection)
        {
            if (speed <= 0)
                return 0;

            double brakingAccel = GetBrakingAcceleration(worldVelocityDirection);
            
            if (brakingAccel <= 0.01)
                return double.MaxValue; // Can't brake in this direction
            
            // d = v² / (2a)
            return (speed * speed) / (2 * brakingAccel);
        }

        /// <summary>
        /// Rebuilds the thrust map. Call when thrusters change, config changes, 
        /// or periodically to account for atmosphere/damage.
        /// </summary>
        public void RefreshThrustMap()
        {
            // Filter out hover pads if configured
            _controllableThrusters.Clear();
            foreach (var thruster in _allThrusters)
            {
                if (thruster == null || !thruster.IsFunctional || thruster.Closed)
                    continue;
                    
                if (_config.ExcludeHoverPads && IsHoverPad(thruster))
                    continue;
                    
                _controllableThrusters.Add(thruster);
            }
            
            // Build thrust capability map
            _thrustMap = new double[3, 2];
            
            if (_reference == null)
                return;
            
            foreach (var thruster in _controllableThrusters)
            {
                // Get thruster direction in ship-local space
                // Thruster pushes in opposite direction of its "Backward" vector
                Vector3D thrustDirection = WorldToShipDirection(thruster.WorldMatrix.Backward);
                
                // Use MaxEffectiveThrust (accounts for atmosphere, damage, etc.)
                double thrust = thruster.MaxEffectiveThrust;
                
                // Round to handle floating point imprecision
                if (Math.Abs(Math.Round(thrustDirection.X, 2)) > 0.01)
                {
                    if (thrustDirection.X >= 0) 
                        _thrustMap[0, 0] += thrust;
                    else 
                        _thrustMap[0, 1] += thrust;
                }
                if (Math.Abs(Math.Round(thrustDirection.Y, 2)) > 0.01)
                {
                    if (thrustDirection.Y >= 0) 
                        _thrustMap[1, 0] += thrust;
                    else 
                        _thrustMap[1, 1] += thrust;
                }
                if (Math.Abs(Math.Round(thrustDirection.Z, 2)) > 0.01)
                {
                    if (thrustDirection.Z >= 0) 
                        _thrustMap[2, 0] += thrust;
                    else 
                        _thrustMap[2, 1] += thrust;
                }
            }
        }

        /// <summary>
        /// Checks if a thruster is a hover pad based on subtype ID patterns.
        /// </summary>
        private bool IsHoverPad(IMyThrust thruster)
        {
            string subtypeId = thruster.BlockDefinition.SubtypeId.ToUpperInvariant();
            foreach (var pattern in _config.HoverPadPatterns)
            {
                if (subtypeId.Contains(pattern.ToUpperInvariant()))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Sets the desired velocity in world space.
        /// Controller calculates the force needed to achieve this velocity and applies it.
        /// This is the preferred method for behaviors - simpler than ApplyForce.
        /// </summary>
        /// <param name="worldVelocity">Desired velocity in world-space coordinates</param>
        public void SetDesiredVelocity(Vector3D worldVelocity)
        {
            if (_reference == null)
                return;

            // Ensure dampeners are on
            if (!_reference.DampenersOverride)
            {
                _reference.DampenersOverride = true;
            }

            // Get current velocity in world space
            Vector3D currentVelocity = _reference.GetShipVelocities().LinearVelocity;

            // Calculate velocity error in world space
            Vector3D velocityError = worldVelocity - currentVelocity;

            // Convert to force: F = m * (dv/dt)
            Vector3D worldForce = velocityError * _shipMass / _deltaTime;

            // Transform to ship-local coordinates
            Vector3D localForce = WorldToShipDirection(worldForce);

            // Apply axis authority
            localForce = _config.ApplyAuthority(localForce);

            // Apply force (handles gravity compensation internally)
            ApplyForce(localForce);
        }

        /// <summary>
        /// Applies a force vector in ship-local space.
        /// </summary>
        /// <param name="localForce">Force in Newtons, ship-local coordinates</param>
        public void ApplyForce(Vector3D localForce)
        {
            if (_controllableThrusters.Count == 0)
                return;
            
            // Get gravity in ship space for "gravity as propulsion" calculations
            Vector3D gravityShip = Vector3D.Zero;
            if (_reference != null)
            {
                gravityShip = WorldToShipDirection(_reference.GetNaturalGravity());
            }
            Vector3D gravityForce = gravityShip * _shipMass;
            
            // Calculate the THRUST needed to achieve the desired NET force
            // Net force = Thrust + Gravity, so Thrust = NetForce - Gravity
            Vector3D requiredThrust = localForce - gravityForce;
            
            // Get available thrust in each direction (based on required thrust direction)
            Vector3D availableThrust = GetAvailableThrust(requiredThrust);
            
            // Calculate what percentage of max thrust we need in each axis
            // This now properly handles "reduce thrust to let gravity help"
            float xRatio = availableThrust.X > 0 ? 
                (float)MathHelper.Clamp(Math.Abs(requiredThrust.X / availableThrust.X), 0, 1) : 0;
            float yRatio = availableThrust.Y > 0 ? 
                (float)MathHelper.Clamp(Math.Abs(requiredThrust.Y / availableThrust.Y), 0, 1) : 0;
            float zRatio = availableThrust.Z > 0 ? 
                (float)MathHelper.Clamp(Math.Abs(requiredThrust.Z / availableThrust.Z), 0, 1) : 0;
            
            // Handle "gravity as propulsion" case:
            // If we need negative thrust on an axis (reduce thrust to let gravity work)
            // but only have thrusters in the positive direction, calculate how much to throttle back
            bool useGravityY = false;
            float gravityAssistedYRatio = 0f;
            
            // Debug: Log the force calculations
            //_echo?.Invoke($"[THRUST] net.Y:{localForce.Y:F0}N grav.Y:{gravityForce.Y:F0}N req.Y:{requiredThrust.Y:F0}N");
            
            // Check if we need to go "down" (negative Y thrust) but only have "up" thrusters
            double upThrust = _thrustMap[1, 0];  // Positive Y thrust
            double downThrust = _thrustMap[1, 1]; // Negative Y thrust
            
            if (requiredThrust.Y < 0 && downThrust < 0.01 && upThrust > 0.01)
            {
                // We need downward net force but have no downward thrusters
                // Calculate how much upward thrust is needed to achieve the desired descent rate
                // Required thrust is negative, gravity force is negative (pulling down)
                // We need: requiredThrust = actualThrust + gravityForce
                // So: actualThrust = requiredThrust - gravityForce
                // If actualThrust is negative, we can't achieve it - just provide 0 thrust
                // If actualThrust is positive but less than hover, we provide partial thrust
                
                // How much upward thrust would we need to hover? (counteract gravity)
                double hoverThrust = -gravityForce.Y; // Positive value if gravity is down
                
                // What thrust do we actually need?
                // requiredThrust.Y is negative (want net downward force)
                // The actual thrust needed is: requiredThrust.Y - gravityForce.Y
                // But wait, requiredThrust already accounts for this (see calculation above)
                // Actually requiredThrust.Y = localForce.Y - gravityForce.Y
                // If localForce.Y = -1000N (want to accelerate down) and gravityForce.Y = -10000N (gravity)
                // Then requiredThrust.Y = -1000 - (-10000) = 9000N (we need 9000N up to only accelerate down at desired rate)
                
                // So if requiredThrust.Y > 0 but we calculated it as needing "down" thrust direction,
                // that means we need SOME upward thrust, but less than hover
                // Actually let me reconsider...
                
                // Let's recalculate: the issue is localForce is the NET force we want
                // requiredThrust = localForce - gravityForce = NET - GRAVITY
                // If NET is -1000N (down) and GRAVITY is -10000N (down)
                // Then requiredThrust = -1000 - (-10000) = +9000N (we need 9000N of UPWARD thrust)
                
                // So if requiredThrust.Y > 0, we actually need upward thrust (just less than hover)
                // If requiredThrust.Y < 0, we need downward thrust which we don't have - provide 0 and let gravity work
                
                useGravityY = true;
                
                // EXPERIMENT: Disable dampeners during gravity-assisted descent
                // This prevents dampeners from fighting our reduced thrust
                if (_reference != null)
                {
                    _reference.DampenersOverride = false;
                }
                
                if (requiredThrust.Y >= 0)
                {
                    // We need some upward thrust, but less than full hover
                    gravityAssistedYRatio = (float)MathHelper.Clamp(requiredThrust.Y / upThrust, 0, 1);
                }
                else
                {
                    // We need more downward force than gravity provides - can't achieve it
                    // Just provide 0 thrust and let gravity do its best
                    gravityAssistedYRatio = 0f;
                }
                
                _echo?.Invoke($"[THRUST] GRAVITY ASSIST: ratio:{gravityAssistedYRatio:F2} dampeners:OFF");
            }
            
            // Re-enable dampeners if not using gravity assist
            if (!useGravityY && _reference != null)
            {
                _reference.DampenersOverride = true;
            }
            
            foreach (var thruster in _controllableThrusters)
            {
                if (!thruster.IsFunctional || !thruster.Enabled)
                    continue;
                
                Vector3D thrustDir = WorldToShipDirection(thruster.WorldMatrix.Backward);
                
                // Round the direction to determine primary axis
                Vector3D roundedDir = new Vector3D(
                    Math.Round(thrustDir.X, 1),
                    Math.Round(thrustDir.Y, 1),
                    Math.Round(thrustDir.Z, 1)
                );
                
                float overridePercent = 0f;
                
                // Handle Y-axis with gravity assistance
                if (useGravityY && Math.Abs(roundedDir.Y) > 0.1 && roundedDir.Y > 0)
                {
                    // This is an upward thruster and we're using gravity-assisted descent
                    overridePercent = gravityAssistedYRatio;
                }
                // Normal thrust application for other cases
                else if (Math.Abs(roundedDir.X) > 0.1 && Math.Sign(roundedDir.X) == Math.Sign(requiredThrust.X))
                {
                    overridePercent = xRatio;
                }
                else if (Math.Abs(roundedDir.Y) > 0.1 && Math.Sign(roundedDir.Y) == Math.Sign(requiredThrust.Y))
                {
                    overridePercent = yRatio;
                }
                else if (Math.Abs(roundedDir.Z) > 0.1 && Math.Sign(roundedDir.Z) == Math.Sign(requiredThrust.Z))
                {
                    overridePercent = zRatio;
                }
                
                thruster.ThrustOverridePercentage = overridePercent;
            }
        }

        /// <summary>
        /// Returns the available thrust for a given desired force vector (ship-space).
        /// </summary>
        private Vector3D GetAvailableThrust(Vector3D desiredForce)
        {
            return new Vector3D(
                desiredForce.X >= 0 ? _thrustMap[0, 0] : _thrustMap[0, 1],
                desiredForce.Y >= 0 ? _thrustMap[1, 0] : _thrustMap[1, 1],
                desiredForce.Z >= 0 ? _thrustMap[2, 0] : _thrustMap[2, 1]
            );
        }

        /// <summary>
        /// Transforms a direction vector from world space to ship-local space.
        /// </summary>
        private Vector3D WorldToShipDirection(Vector3D worldDirection)
        {
            if (_reference == null)
                return worldDirection;
            return Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(_reference.WorldMatrix));
        }

        /// <summary>
        /// Releases all thruster overrides.
        /// </summary>
        public void Release()
        {
            foreach (var thruster in _allThrusters)
            {
                if (thruster != null && !thruster.Closed)
                {
                    thruster.ThrustOverridePercentage = 0f;
                }
            }
        }

        /// <summary>
        /// Gets the distance to a target position.
        /// </summary>
        public double GetDistanceTo(Vector3D targetPosition)
        {
            if (_reference == null)
                return double.MaxValue;
            return Vector3D.Distance(_reference.GetPosition(), targetPosition);
        }
    }
}
