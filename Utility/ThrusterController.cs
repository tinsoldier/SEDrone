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
        /// Moves toward a target position with velocity matching.
        /// This is the main method brains should call.
        /// </summary>
        /// <param name="targetPosition">World-space target position</param>
        /// <param name="targetVelocity">Desired velocity to match (e.g., leader's velocity)</param>
        /// <param name="maxSpeed">Maximum allowed speed</param>
        /// <param name="precisionRadius">Distance at which to slow down</param>
        public void MoveToward(Vector3D targetPosition, Vector3D targetVelocity, 
                               double maxSpeed, double precisionRadius)
        {
            if (_reference == null)
                return;
            
            // Get current state
            Vector3D currentPosition = _reference.GetPosition();
            Vector3D currentVelocity = _reference.GetShipVelocities().LinearVelocity;
            Vector3D gravity = _reference.GetNaturalGravity();
            
            // Transform to ship space
            Vector3D toTarget = targetPosition - currentPosition;
            Vector3D toTargetShip = WorldToShipDirection(toTarget);
            Vector3D velocityShip = WorldToShipDirection(currentVelocity);
            Vector3D gravityShip = WorldToShipDirection(gravity);
            Vector3D targetVelocityShip = WorldToShipDirection(targetVelocity);
            
            double distance = toTargetShip.Length();
            
            // Calculate desired velocity
            Vector3D desiredVelocity;
            
            if (distance < 0.5)
            {
                // Very close - just match target velocity
                desiredVelocity = targetVelocityShip;
            }
            else
            {
                // Calculate safe approach speed
                Vector3D direction = toTargetShip / distance;
                double safeSpeed = CalculateMaxApproachSpeed(toTargetShip, gravityShip);
                double targetSpeed = Math.Min(safeSpeed, maxSpeed);
                
                // Smooth approach when close
                if (distance < precisionRadius)
                    targetSpeed *= Math.Max(0.1, distance / precisionRadius);
                
                // Desired = base target velocity + correction toward position
                desiredVelocity = targetVelocityShip + direction * targetSpeed;
                
                // Clamp total speed
                double totalSpeed = desiredVelocity.Length();
                if (totalSpeed > maxSpeed)
                    desiredVelocity = desiredVelocity / totalSpeed * maxSpeed;
            }
            
            // Calculate velocity error
            Vector3D velocityError = desiredVelocity - velocityShip;
            
            // Convert to force: F = m * a
            Vector3D desiredForce = velocityError * _shipMass / _deltaTime;
            
            // Counteract gravity (scaled by vertical authority)
            Vector3D gravityCompensation = gravityShip * _shipMass * _config.VerticalAuthority;
            desiredForce -= gravityCompensation;
            
            // Apply axis authority
            desiredForce = _config.ApplyAuthority(desiredForce);
            
            // Apply thrust
            ApplyForce(desiredForce);
        }

        /// <summary>
        /// Applies a force vector in ship-local space.
        /// </summary>
        /// <param name="localForce">Force in Newtons, ship-local coordinates</param>
        public void ApplyForce(Vector3D localForce)
        {
            if (_controllableThrusters.Count == 0)
                return;
            
            // Get available thrust in each direction
            Vector3D availableThrust = GetAvailableThrust(localForce);
            
            // Calculate what percentage of max thrust we need in each axis
            float xRatio = availableThrust.X > 0 ? 
                (float)Math.Min(1.0, Math.Abs(localForce.X / availableThrust.X)) : 0;
            float yRatio = availableThrust.Y > 0 ? 
                (float)Math.Min(1.0, Math.Abs(localForce.Y / availableThrust.Y)) : 0;
            float zRatio = availableThrust.Z > 0 ? 
                (float)Math.Min(1.0, Math.Abs(localForce.Z / availableThrust.Z)) : 0;
            
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
                
                // Check if this thruster helps in the desired direction
                if (Math.Abs(roundedDir.X) > 0.1 && Math.Sign(roundedDir.X) == Math.Sign(localForce.X))
                    overridePercent = xRatio;
                else if (Math.Abs(roundedDir.Y) > 0.1 && Math.Sign(roundedDir.Y) == Math.Sign(localForce.Y))
                    overridePercent = yRatio;
                else if (Math.Abs(roundedDir.Z) > 0.1 && Math.Sign(roundedDir.Z) == Math.Sign(localForce.Z))
                    overridePercent = zRatio;
                
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
        /// Calculates the maximum safe approach speed based on stopping distance.
        /// Uses kinematic equation: v = sqrt(2 * a * d)
        /// </summary>
        private double CalculateMaxApproachSpeed(Vector3D distanceVector, Vector3D gravityShipSpace)
        {
            if (distanceVector.LengthSquared() < 0.01) 
                return 0;
            
            // Calculate available deceleration in the direction we're traveling
            Vector3D direction = Vector3D.Normalize(distanceVector);
            Vector3D availableThrust = GetAvailableThrust(-direction); // Opposite direction for braking
            
            // Account for gravity assistance/resistance
            Vector3D thrustWithGravity = availableThrust + gravityShipSpace * _shipMass * _config.VerticalAuthority;
            
            // Get component of available thrust in our direction
            double thrustInDirection = Math.Abs(
                thrustWithGravity.X * direction.X +
                thrustWithGravity.Y * direction.Y +
                thrustWithGravity.Z * direction.Z
            );
            
            double deceleration = thrustInDirection / _shipMass;
            
            if (deceleration <= 0.1) 
                return 0.5; // Minimum crawl speed
            
            // v = sqrt(2 * a * d) * safety_factor
            double distance = distanceVector.Length();
            double maxSpeed = Math.Sqrt(2 * deceleration * distance) * _config.AccelerationFactor;
            
            return maxSpeed;
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
        /// Stops the ship by zeroing velocity (emergency brake).
        /// </summary>
        public void EmergencyStop()
        {
            if (_reference == null)
            {
                Release();
                return;
            }
            
            Vector3D currentVelocity = _reference.GetShipVelocities().LinearVelocity;
            Vector3D velocityShip = WorldToShipDirection(currentVelocity);
            Vector3D gravity = _reference.GetNaturalGravity();
            Vector3D gravityShip = WorldToShipDirection(gravity);
            
            // Apply maximum braking force
            Vector3D brakingForce = -velocityShip * _shipMass * 10; // Strong braking
            brakingForce -= gravityShip * _shipMass * _config.VerticalAuthority;
            brakingForce = _config.ApplyAuthority(brakingForce);
            
            ApplyForce(brakingForce);
        }

        /// <summary>
        /// Gets the current speed in m/s.
        /// </summary>
        public double GetCurrentSpeed()
        {
            if (_reference == null)
                return 0;
            return _reference.GetShipSpeed();
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

        /// <summary>
        /// Checks if the ship has reached a target position within tolerance.
        /// </summary>
        public bool HasReached(Vector3D targetPosition, double tolerance = 2.0)
        {
            return GetDistanceTo(targetPosition) < tolerance;
        }
    }
}
