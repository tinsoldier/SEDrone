using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Missile interception mode - activates when smart projectiles are detected.
    /// Orients the drone toward incoming missiles so gimbal turrets can engage.
    /// 
    /// Uses pluggable aiming strategies:
    /// - CentroidAimStrategy: Aims at weighted center of mass (good for swarms)
    /// - ClosestMissileAimStrategy: Tracks nearest missile (aggressive point defense)
    /// 
    /// Supports two tracking modes:
    /// - Position tracking (WcPbApiBridge): Uses strategy to calculate aim direction
    /// - Count only (WcPbApi fallback): Noses up by configurable angle for best coverage
    /// 
    /// Transitions:
    /// - → SearchingMode: When no more projectiles detected (fresh start)
    /// </summary>
    public class InterceptMode : IDroneMode
    {
        // === Configuration ===
        private const double BLEND_RAMP_SECONDS = 0.8;        // Time to fully transition from nose-up to strategy tracking
        
        // === Strategy ===
        private readonly IInterceptAimStrategy _aimStrategy;
        
        // === State ===
        private int _projectileCount;
        private bool _hasPositionTracking;  // True if we have missile positions
        private List<Vector3D> _projectilePositions = new List<Vector3D>();
        private Vector3D _lastThreatDirection = Vector3D.Zero;
        private Vector3D _initialNoseUpDirection = Vector3D.Zero;  // Captured at mode entry - does not update
        private Vector3D _worldUp = Vector3D.Up;  // Cached world up direction
        private double _blendFactor = 0.0;  // 0 = pure nose-up, 1 = pure strategy tracking
        private DateTime _modeEntryTime;    // When we entered intercept mode
        
        public string Name => $"Intercept[{_aimStrategy.Name}] ({_projectileCount} missiles)";

        /// <summary>
        /// Creates InterceptMode with specified aiming strategy.
        /// </summary>
        /// <param name="aimStrategy">Strategy for calculating aim direction. Defaults to ClosestMissileAimStrategy.</param>
        /// <param name="projectileCount">Initial projectile count.</param>
        public InterceptMode(IInterceptAimStrategy aimStrategy = null, int projectileCount = 0)
        {
            _aimStrategy = aimStrategy ?? new ClosestMissileAimStrategy();
            _projectileCount = projectileCount;
            _hasPositionTracking = false;
        }

        /// <summary>
        /// Sets whether we have position tracking available.
        /// </summary>
        public void SetHasPositionTracking(bool hasTracking)
        {
            _hasPositionTracking = hasTracking;
        }

        /// <summary>
        /// Updates the projectile positions (called by DroneBrain each tick).
        /// </summary>
        public void UpdateProjectilePositions(List<Vector3D> positions)
        {
            _projectilePositions.Clear();
            _projectilePositions.AddRange(positions);
            _projectileCount = positions.Count;
            _hasPositionTracking = true;
        }

        /// <summary>
        /// Updates just the count (no positions available).
        /// </summary>
        public void UpdateProjectileCount(int count)
        {
            _projectileCount = count;
            _projectilePositions.Clear();  // Clear any stale positions
        }

        public void Enter(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] THREAT DETECTED - Entering intercept mode");
            _lastThreatDirection = brain.Context.Reference.WorldMatrix.Forward;
            _modeEntryTime = DateTime.UtcNow;
            _blendFactor = 0.0;  // Start with pure nose-up
            
            // Capture initial nose-up direction - this is FIXED for the entire blend ramp
            // so the drone has a stable starting point to blend from
            CaptureInitialNoseUpDirection(brain);
        }

        /// <summary>
        /// Captures the initial nose-up direction at mode entry.
        /// This is a FIXED direction used as the starting point for blending.
        /// It does NOT update as the drone turns - that's the whole point.
        /// </summary>
        private void CaptureInitialNoseUpDirection(DroneBrain brain)
        {
            Vector3D gravity = brain.Context.Reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.1)
            {
                // No gravity - just use current forward
                _initialNoseUpDirection = brain.Context.Reference.WorldMatrix.Forward;
                _worldUp = Vector3D.Up;
                return;
            }
            
            _worldUp = -Vector3D.Normalize(gravity);
            Vector3D currentForward = brain.Context.Reference.WorldMatrix.Forward;
            
            // Project forward onto horizontal plane
            Vector3D horizontalForward = currentForward - _worldUp * Vector3D.Dot(currentForward, _worldUp);
            if (horizontalForward.LengthSquared() < 0.001)
            {
                horizontalForward = brain.Context.Reference.WorldMatrix.Backward;
            }
            horizontalForward = Vector3D.Normalize(horizontalForward);
            
            // Rotate forward upward by configured angle
            double noseUpAngleRad = brain.Config.InterceptNoseUpAngle * Math.PI / 180.0;
            
            // Nose-up direction = blend of horizontal forward and world up
            // At 0° = pure horizontal, at 90° = pure up
            _initialNoseUpDirection = horizontalForward * Math.Cos(noseUpAngleRad) + _worldUp * Math.Sin(noseUpAngleRad);
            _initialNoseUpDirection = Vector3D.Normalize(_initialNoseUpDirection);
        }

        public IDroneMode Update(DroneBrain brain)
        {
            // Update blend factor based on time in mode
            double elapsedSeconds = (DateTime.UtcNow - _modeEntryTime).TotalSeconds;
            _blendFactor = Math.Min(1.0, elapsedSeconds / BLEND_RAMP_SECONDS);
            
            // Calculate threat direction with blending
            Vector3D threatDirection;
            
            if (_hasPositionTracking && _projectilePositions.Count > 0)
            {
                // Full position tracking - use strategy to calculate aim direction
                Vector3D droneForward = brain.Context.Reference.WorldMatrix.Forward;
                Vector3D strategyDirection = _aimStrategy.CalculateAimDirection(
                    brain.Position, droneForward, _projectilePositions);
                
                if (strategyDirection.LengthSquared() > 0.1)
                {
                    strategyDirection = Vector3D.Normalize(strategyDirection);
                    
                    // Blend from FIXED initial nose-up to LIVE strategy direction
                    // _initialNoseUpDirection was captured at mode entry and never changes
                    // strategyDirection updates every tick as missiles move
                    Vector3D blended = Vector3D.Lerp(_initialNoseUpDirection, strategyDirection, _blendFactor);
                    threatDirection = Vector3D.Normalize(blended);
                }
                else
                {
                    // Strategy returned no target - use initial nose-up
                    threatDirection = _initialNoseUpDirection;
                }
            }
            else
            {
                // Count-only mode - use initial nose-up direction only
                threatDirection = _initialNoseUpDirection;
            }
            
            // Orient toward threat direction
            if (threatDirection.LengthSquared() > 0.1)
            {
                _lastThreatDirection = threatDirection;
                Vector3D lookTarget = brain.Position + threatDirection * 1000.0;
                brain.Gyros.LookAt(lookTarget);
            }
            else
            {
                // Fallback - maintain last direction or stay level
                if (_lastThreatDirection.LengthSquared() > 0.1)
                {
                    Vector3D lookTarget = brain.Position + _lastThreatDirection * 1000.0;
                    brain.Gyros.LookAt(lookTarget);
                }
                else
                {
                    brain.Gyros.OrientLevel();
                }
            }
            
            // Movement: maintain formation if we have leader contact
            if (brain.HasLeaderContact)
            {
                Vector3D formationPos = brain.Navigator.CalculateFormationPosition(
                    brain.LastLeaderState,
                    brain.Config.StationOffset
                );
                
                double distance = Vector3D.Distance(brain.Position, formationPos);
                brain.UpdateFormationData(formationPos, distance);
                
                Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                    brain.Position,
                    brain.Velocity,
                    formationPos,
                    brain.LastLeaderState.Velocity,
                    brain.Config.MaxSpeed * 0.5  // Half speed during intercept
                );
                
                brain.Thrusters.MoveToward(
                    formationPos,
                    desiredVelocity,
                    brain.Config.MaxSpeed * 0.5,
                    brain.Config.PrecisionRadius
                );
            }
            else
            {
                brain.Thrusters.Release();
            }
            
            return this;
        }

        public void Exit(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] Threat cleared - exiting intercept mode");
        }
    }
}
