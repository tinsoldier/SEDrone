using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Approach strategy determines how the drone maneuvers toward formation.
    /// </summary>
    public enum ApproachStrategy
    {
        /// <summary>
        /// Direct approach: face the target and pitch toward it.
        /// Fast but can be unstable when turning sharply.
        /// </summary>
        Direct,

        /// <summary>
        /// Conservative approach: stay level, yaw to face, move horizontally, then adjust altitude.
        /// Slower but more stable for recovery from overshoot or sharp turns.
        /// </summary>
        LevelFirst
    }

    /// <summary>
    /// Moving toward formation position.
    /// Faces the destination waypoint and uses approach velocity calculation.
    /// 
    /// Supports two strategies:
    /// - Direct: Face and fly directly to target (faster, can be unstable on sharp turns)
    /// - LevelFirst: Stay level, turn yaw-only, approach horizontally, then adjust altitude (conservative)
    /// 
    /// Transitions:
    /// - → SearchingMode: If leader contact is lost
    /// - → HoldingMode: When within StationRadius of formation position
    /// </summary>
    public class ApproachingMode : IDroneMode
    {
        public string Name 
        { 
            get 
            {
                if (_strategy == ApproachStrategy.LevelFirst)
                {
                    string phase = _levelPhase.ToString();
                    //return $"Approach-Level {phase}";
                    switch(_levelPhase)
                    {
                        case LevelPhase.Leveling:
                            return "Approaching-Leveling";
                        case LevelPhase.Turning:
                            return "Approaching-Turning";
                        case LevelPhase.Approaching:
                            return "Approaching-Horizontal";
                        case LevelPhase.AltitudeAdjust:
                            return "Approaching-Altitude";
                        default:
                            return "Approaching-Level";
                    }
                }
                return "Approaching";
            }
        }

        // === Strategy Configuration ===
        private ApproachStrategy _strategy = ApproachStrategy.Direct;
        private bool _strategyLocked = false;  // Once we commit to a strategy, stick with it
        
        // Thresholds for automatic strategy selection
        private const double OVERSHOOT_ANGLE_THRESHOLD = 90.0;  // degrees - if target is more than 90° off, use conservative
        private const double ELEVATION_DELTA_THRESHOLD = 20.0;   // meters - large altitude difference triggers conservative
        private const double LEVEL_TURN_COMPLETE_THRESHOLD = 30.0; // degrees - yaw error below this = turn complete (generous to avoid oscillation)
        private const double STRATEGY_UNLOCK_ANGLE = 45.0;  // degrees - unlock strategy when facing target within this

        // Track phase for LevelFirst strategy
        public enum LevelPhase { Leveling, Turning, Approaching, AltitudeAdjust }
        private LevelPhase _levelPhase = LevelPhase.Leveling;

        /// <summary>
        /// Creates ApproachingMode with automatic strategy selection.
        /// </summary>
        public ApproachingMode()
        {
            _strategy = ApproachStrategy.Direct;  // Will auto-detect if conservative needed
            //_strategy = ApproachStrategy.LevelFirst;
        }

        /// <summary>
        /// Creates ApproachingMode with a specific strategy.
        /// </summary>
        public ApproachingMode(ApproachStrategy strategy)
        {
            _strategy = strategy;
        }

        public void Enter(DroneBrain brain)
        {
            _levelPhase = LevelPhase.Leveling;
            _strategyLocked = false;  // Allow strategy selection on entry
            brain.Echo?.Invoke($"[{Name}] Moving to formation position");
        }

        public IDroneMode Update(DroneBrain brain)
        {
            // Check for leader loss
            if (!brain.HasLeaderContact)
            {
                return new SearchingMode();
            }

            // Calculate formation position
            Vector3D formationPos = brain.Navigator.CalculateFormationPosition(
                brain.LastLeaderState,
                brain.Config.StationOffset
            );

            // Apply terrain clearance adjustment
            formationPos = brain.Navigator.AdjustForTerrainClearance(
                formationPos,
                brain.Context.Reference,
                brain.Config.MinTerrainClearance
            );

            // Check distance and calculate braking metrics
            double distance = Vector3D.Distance(brain.Position, formationPos);
            Vector3D toFormation = formationPos - brain.Position;
            double currentSpeed = brain.Velocity.Length();
            double brakingDistance = brain.Thrusters.GetBrakingDistance(currentSpeed, brain.Velocity);
            double safeSpeed = brain.Thrusters.GetSafeApproachSpeed(distance, toFormation);

            // Check for arrival at formation (velocity-aware)
            if (brain.Navigator.IsInFormationWithBrakingMargin(distance, brakingDistance))
            {
                return new HoldingMode();
            }

            // Auto-detect if we should use conservative approach
            ApproachStrategy effectiveStrategy = DetermineStrategy(brain, formationPos, toFormation);

            // Update brain's cached formation data for status display
            brain.UpdateFormationData(formationPos, distance);

            // Execute based on strategy
            if (effectiveStrategy == ApproachStrategy.LevelFirst)
            {
                return ExecuteLevelFirstApproach(brain, formationPos, toFormation, distance, safeSpeed);
            }
            else
            {
                return ExecuteDirectApproach(brain, formationPos, distance, safeSpeed);
            }
        }

        /// <summary>
        /// Determines which strategy to use based on current situation.
        /// Once committed to LevelFirst, stays locked until facing the target.
        /// </summary>
        private ApproachStrategy DetermineStrategy(DroneBrain brain, Vector3D formationPos, Vector3D toFormation)
        {
            // Calculate current angle to target
            Vector3D forward = brain.Context.Reference.WorldMatrix.Forward;
            Vector3D toFormationNorm = Vector3D.Normalize(toFormation);
            double dotForward = Vector3D.Dot(forward, toFormationNorm);
            double angleOff = Math.Acos(MathHelper.Clamp(dotForward, -1, 1)) * 180.0 / Math.PI;

            // If strategy is locked to LevelFirst, check if we can unlock
            if (_strategyLocked && _strategy == ApproachStrategy.LevelFirst)
            {
                // Unlock only when we're facing the target AND past the turning phase
                if (angleOff < STRATEGY_UNLOCK_ANGLE && _levelPhase >= LevelPhase.Approaching)
                {
                    _strategyLocked = false;
                    // But stay in LevelFirst since we're already in the approach phase
                    return ApproachStrategy.LevelFirst;
                }
                // Stay locked
                return ApproachStrategy.LevelFirst;
            }

            // If explicitly set to LevelFirst, use it and lock
            if (_strategy == ApproachStrategy.LevelFirst)
            {
                _strategyLocked = true;
                return ApproachStrategy.LevelFirst;
            }

            // Auto-detect: Check if target is behind us (overshoot scenario)
            if (angleOff > OVERSHOOT_ANGLE_THRESHOLD)
            {
                // Target is behind us - use conservative approach and LOCK it
                _strategyLocked = true;
                return ApproachStrategy.LevelFirst;
            }

            // Auto-detect: Check for large elevation difference
            Vector3D gravity = brain.Context.Reference.GetNaturalGravity();
            if (gravity.LengthSquared() > 0.1)
            {
                Vector3D worldUp = -Vector3D.Normalize(gravity);
                double elevationDelta = Math.Abs(Vector3D.Dot(toFormation, worldUp));
                
                // If target is far off in elevation AND significantly behind/beside us
                if (elevationDelta > ELEVATION_DELTA_THRESHOLD && angleOff > 25.0)
                {
                    _strategyLocked = true;
                    return ApproachStrategy.LevelFirst;
                }
            }

            return ApproachStrategy.Direct;
        }

        /// <summary>
        /// Standard direct approach: face target and fly toward it.
        /// </summary>
        private IDroneMode ExecuteDirectApproach(DroneBrain brain, Vector3D formationPos, double distance, double safeSpeed)
        {
            // Calculate desired velocity with safe-speed cap
            Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                brain.Position,
                brain.Velocity,
                formationPos,
                brain.LastLeaderState.Velocity,
                safeSpeed
            );

            // Orientation: face destination waypoint
            brain.Gyros.LookAt(formationPos);

            // Movement: approach formation position
            brain.Thrusters.MoveToward(
                formationPos,
                desiredVelocity,
                brain.Config.MaxSpeed,
                brain.Config.PrecisionRadius
            );

            return this;
        }

        /// <summary>
        /// Conservative level-first approach:
        /// 1. Level the grid
        /// 2. Yaw to face target (horizontal only)
        /// 3. Approach horizontally
        /// 4. Adjust altitude
        /// </summary>
        private IDroneMode ExecuteLevelFirstApproach(DroneBrain brain, Vector3D formationPos, Vector3D toFormation, double distance, double safeSpeed)
        {
            Vector3D gravity = brain.Context.Reference.GetNaturalGravity();
            Vector3D worldUp = gravity.LengthSquared() > 0.1 
                ? -Vector3D.Normalize(gravity) 
                : brain.Context.Reference.WorldMatrix.Up;

            // Calculate horizontal distance and elevation delta
            double elevationDelta = Vector3D.Dot(toFormation, worldUp);
            Vector3D horizontalToFormation = toFormation - worldUp * elevationDelta;
            double horizontalDistance = horizontalToFormation.Length();

            // Phase state machine
            switch (_levelPhase)
            {
                case LevelPhase.Leveling:
                    // Get level first
                    brain.Gyros.OrientLevel();
                    brain.Thrusters.Release();  // Don't thrust while leveling
                    
                    if (brain.Gyros.IsLevel(5.0))
                    {
                        _levelPhase = LevelPhase.Turning;
                    }
                    break;

                case LevelPhase.Turning:
                    // Turn to face target (yaw only, stay level)
                    brain.Gyros.LevelTurnToward(formationPos);
                    brain.Thrusters.Release();  // Don't thrust while turning
                    
                    if (brain.Gyros.IsFacingTarget(formationPos, LEVEL_TURN_COMPLETE_THRESHOLD))
                    {
                        _levelPhase = LevelPhase.Approaching;
                    }
                    break;

                case LevelPhase.Approaching:
                    // Move horizontally toward target, staying level
                    brain.Gyros.LevelTurnToward(formationPos);
                    
                    if (horizontalDistance > brain.Config.StationRadius * 2)
                    {
                        // Create a horizontal-only target (same altitude as us)
                        Vector3D horizontalTarget = formationPos - worldUp * elevationDelta;
                        
                        Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                            brain.Position,
                            brain.Velocity,
                            horizontalTarget,
                            brain.LastLeaderState.Velocity,
                            safeSpeed
                        );

                        brain.Thrusters.MoveToward(
                            horizontalTarget,
                            desiredVelocity,
                            brain.Config.MaxSpeed,
                            brain.Config.PrecisionRadius
                        );
                    }
                    else
                    {
                        // Close enough horizontally, adjust altitude
                        _levelPhase = LevelPhase.AltitudeAdjust;
                    }
                    break;

                case LevelPhase.AltitudeAdjust:
                    // Now handle altitude while maintaining level flight
                    brain.Gyros.LevelTurnToward(formationPos);
                    
                    // Full approach now that we're close horizontally
                    Vector3D finalVelocity = brain.Navigator.CalculateDesiredVelocity(
                        brain.Position,
                        brain.Velocity,
                        formationPos,
                        brain.LastLeaderState.Velocity,
                        safeSpeed * 0.5  // Slower for final approach
                    );

                    brain.Thrusters.MoveToward(
                        formationPos,
                        finalVelocity,
                        brain.Config.MaxSpeed,
                        brain.Config.PrecisionRadius
                    );

                    // If we're close enough, we'll transition to HoldingMode via the normal check
                    break;
            }

            return this;
        }

        public void Exit(DroneBrain brain)
        {
            _levelPhase = LevelPhase.Leveling;  // Reset for next time
            _strategyLocked = false;  // Unlock strategy on exit
        }
    }
}
