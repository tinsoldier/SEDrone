using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Clean movement behavior without legacy abstractions.
    /// Moves to a target position (world or relative) with PID control.
    ///
    /// Design principles:
    /// - Direct thruster control via ApplyForce - no safe speed or precision slowdown
    /// - PID controller for smooth tracking of dynamic targets
    /// - Support for both world-space and relative positioning
    /// - No coupling to formation navigator or leader concepts
    /// </summary>
    public class Move : IPositionBehavior
    {
        // === Closing Speed Control ===
        // TAU: Time constant for closing - "close the gap in TAU seconds"
        // Lower TAU = more aggressive, Higher TAU = gentler approach
        private const double TAU = 3.5;  // Try to close gap in 1.5 seconds
        private const double MIN_CLOSING_SPEED = 1.0;  // Minimum 1 m/s closing
        private const double DEFAULT_MAX_CLOSING_SPEED = 150.0;  // Default max closing speed
        private const double MIN_CLOSE_SPEED_NEAR = 0.2;  // Allow very slow closure near target

        // === PID Tuning (for damping only) ===
        // With feed-forward closing velocity, PID is reduced to damping role
        private const double DEFAULT_KP = 2;  // Small proportional for fine correction
        private const double DEFAULT_KI = 0.2;  // No integral needed with feed-forward
        private const double DEFAULT_KD = 1.0;  // Derivative for damping oscillations
        private const double HOLD_ENTER_SPEED = 0.75;
        private const double HOLD_EXIT_SPEED = 1.5;
        private const double HOLD_EXIT_MULTIPLIER = 1.5;
        private const double NEAR_RADIUS_MULTIPLIER = 2.0;

        // === Target Configuration ===
        private readonly Func<Vector3D> _targetFunc;
        private readonly Func<Vector3D> _referenceFunc;  // For simple relative positioning
        private readonly Func<IOrientedReference> _orientedRefFunc;  // For formation flying
        private readonly bool _isRelative;
        private readonly bool _isFormation;

        // === Velocity Matching ===
        private readonly Func<Vector3D> _targetVelocityFunc;

        // === PID Control ===
        private readonly PIDController3D _pid;

        // === Speed Limiting ===
        private readonly double _maxSpeed;
        private readonly double _maxClosingSpeed;
        private bool _holdingStation;
        private double _stationRadiusOverride = -1;
        private double _holdEnterSpeedOverride = -1;
        private double _holdExitSpeedOverride = -1;

        // === Potential Field Exclusions ===
        private readonly HashSet<long> _exclusions = new HashSet<long>();
        private readonly List<Func<long>> _deferredExclusions = new List<Func<long>>();
        private readonly List<Func<IReadOnlyList<long>>> _deferredExclusionLists = new List<Func<IReadOnlyList<long>>>();
        private HashSet<long> _effectiveExclusions;
        private bool _disableTerrainRepulsion;

        /// <summary>
        /// Adds a static entity ID to the exclusion list.
        /// Excluded entities won't generate repulsion forces.
        /// </summary>
        public Move WithExclusion(long entityId)
        {
            if (entityId != 0)
                _exclusions.Add(entityId);
            return this;
        }

        /// <summary>
        /// Adds a dynamic entity ID provider to the exclusion list.
        /// Evaluated each tick - useful for leader ID that may change.
        /// </summary>
        public Move WithExclusion(Func<long> entityIdProvider)
        {
            if (entityIdProvider != null)
                _deferredExclusions.Add(entityIdProvider);
            return this;
        }

        /// <summary>
        /// Adds a dynamic list of entity IDs to the exclusion set.
        /// </summary>
        public Move WithExclusions(Func<IReadOnlyList<long>> entityIdsProvider)
        {
            if (entityIdsProvider != null)
                _deferredExclusionLists.Add(entityIdsProvider);
            return this;
        }

        /// <summary>
        /// Disables terrain/gravity repulsion for this move.
        /// </summary>
        public Move WithDisableTerrainRepulsion(bool disable = true)
        {
            _disableTerrainRepulsion = disable;
            return this;
        }

        public Move WithStopTuning(double stationRadius = -1, double holdEnterSpeed = -1, double holdExitSpeed = -1)
        {
            if (stationRadius > 0)
                _stationRadiusOverride = stationRadius;
            if (holdEnterSpeed > 0)
                _holdEnterSpeedOverride = holdEnterSpeed;
            if (holdExitSpeed > 0)
                _holdExitSpeedOverride = holdExitSpeed;
            return this;
        }

        /// <summary>
        /// Target position evaluated each tick.
        /// If relative, this is the offset; if world, this is the absolute position.
        /// </summary>
        public Vector3D Target => _targetFunc();

        /// <summary>
        /// Reference point for relative positioning (default: drone position).
        /// </summary>
        public Vector3D Reference => _referenceFunc?.Invoke() ?? Vector3D.Zero;

        /// <summary>
        /// Target velocity for matching (default: zero).
        /// </summary>
        public Vector3D TargetVelocity => _targetVelocityFunc?.Invoke() ?? Vector3D.Zero;

        // ============================================================
        // Static World Position
        // ============================================================

        /// <summary>
        /// Move to a static world position without PID.
        /// Simple direct movement - best for static waypoints.
        /// </summary>
        // public Move(Vector3D worldPosition, double maxSpeed = -1)
        // {
        //     _targetFunc = () => worldPosition;
        //     _isRelative = false;
        //     _usePID = false;
        //     _maxSpeed = maxSpeed;
        // }

        /// <summary>
        /// Move to a dynamic world position with PID tracking.
        /// Use for moving targets (e.g., another ship).
        /// </summary>
        public Move(Func<Vector3D> worldPositionFunc, double maxSpeed = -1,
                    double kp = DEFAULT_KP, double ki = DEFAULT_KI, double kd = DEFAULT_KD)
        {
            _targetFunc = worldPositionFunc;
            _isRelative = false;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
            _maxClosingSpeed = DEFAULT_MAX_CLOSING_SPEED;
        }

        // ============================================================
        // Dynamic World Position with Velocity Matching
        // ============================================================

        /// <summary>
        /// Move to a dynamic world position and match its velocity.
        /// Use for formation flying or intercept.
        /// </summary>
        /// <param name="worldPositionFunc">Function returning target world position</param>
        /// <param name="targetVelocityFunc">Function returning target velocity to match</param>
        /// <param name="maxSpeed">Absolute speed limit (-1 = no limit)</param>
        /// <param name="closingSpeed">Max speed to close gap with moving target (-1 = default 100 m/s)</param>
        public Move(Func<Vector3D> worldPositionFunc, Func<Vector3D> targetVelocityFunc,
                    double maxSpeed = -1, double closingSpeed = -1,
                    double kp = DEFAULT_KP, double ki = DEFAULT_KI, double kd = DEFAULT_KD)
        {
            _targetFunc = worldPositionFunc;
            _targetVelocityFunc = targetVelocityFunc;
            _isRelative = false;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
            _maxClosingSpeed = closingSpeed > 0 ? closingSpeed : DEFAULT_MAX_CLOSING_SPEED;
        }

        // ============================================================
        // Relative Positioning (local offset)
        // ============================================================

        /// <summary>
        /// Move to a position relative to a reference point.
        /// Example: offset=(0, -5, 15) relative to leader position
        /// </summary>
        public Move(Vector3D localOffset, Func<Vector3D> referenceFunc,
                    double maxSpeed = -1, double kp = DEFAULT_KP, double ki = DEFAULT_KI, double kd = DEFAULT_KD)
        {
            _targetFunc = () => localOffset;
            _referenceFunc = referenceFunc;
            _isRelative = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
            _maxClosingSpeed = DEFAULT_MAX_CLOSING_SPEED;
        }

        // ============================================================
        // Formation Flying (Oriented Reference)
        // ============================================================

        /// <summary>
        /// Formation flying - local offset relative to an oriented reference frame.
        /// Transforms local coordinates (X=right, Y=up, Z=forward) to world space.
        /// Automatically matches reference velocity.
        ///
        /// Example: Position = new Move(ctx.Config.StationOffset, () => ctx.LastLeaderState)
        /// </summary>
        /// <param name="localOffset">Offset in target ref's local space (X=right, Y=up, Z=forward)</param>
        /// <param name="orientedRefFunc">Function returning the oriented reference (leader state)</param>
        /// <param name="maxSpeed">Absolute speed limit (-1 = no limit)</param>
        /// <param name="closingSpeed">Max speed to close gap with moving target (-1 = default 100 m/s)</param>
        public Move(Vector3D localOffset, Func<IOrientedReference> orientedRefFunc,
                    double maxSpeed = -1, double closingSpeed = -1,
                    double kp = DEFAULT_KP, double ki = DEFAULT_KI, double kd = DEFAULT_KD)
        {
            _targetFunc = () => localOffset;
            _orientedRefFunc = orientedRefFunc;
            _isFormation = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
            _maxClosingSpeed = closingSpeed > 0 ? closingSpeed : DEFAULT_MAX_CLOSING_SPEED;
        }

        /// <summary>
        /// Formation flying with dynamic offset.
        /// Example: complex formations where offset changes over time.
        /// </summary>
        public Move(Func<Vector3D> localOffsetFunc, Func<IOrientedReference> orientedRefFunc,
                    double maxSpeed = -1, double closingSpeed = -1,
                    double kp = DEFAULT_KP, double ki = DEFAULT_KI, double kd = DEFAULT_KD)
        {
            _targetFunc = localOffsetFunc;
            _orientedRefFunc = orientedRefFunc;
            _isFormation = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
            _maxClosingSpeed = closingSpeed > 0 ? closingSpeed : DEFAULT_MAX_CLOSING_SPEED;
        }

        public void Execute(DroneContext ctx)
        {
            // Calculate world-space target position and velocity
            Vector3D worldTarget;
            Vector3D targetVelocity;

            if (_isFormation)
            {
                // Formation flying: transform local offset to world space using oriented reference
                var orientedRef = _orientedRefFunc();
                Vector3D localOffset = Target;

                // Create transformation matrix from oriented reference
                // MatrixD.CreateWorld uses (position, forward, up) and derives right via cross product
                MatrixD refMatrix = MatrixD.CreateWorld(
                    orientedRef.Position,
                    orientedRef.Forward,
                    orientedRef.Up);

                // Transform local offset to world space
                Vector3D worldOffset = Vector3D.TransformNormal(localOffset, refMatrix);
                worldTarget = orientedRef.Position + worldOffset;
                targetVelocity = orientedRef.Velocity;  // Automatic velocity matching
            }
            else if (_isRelative)
            {
                // Simple relative positioning (world-space offset)
                worldTarget = Reference + Target;
                targetVelocity = TargetVelocity;
            }
            else
            {
                // Absolute world position
                worldTarget = Target;
                targetVelocity = TargetVelocity;
            }

            // Calculate position error
            Vector3D positionError = worldTarget - ctx.Position;

            // Safety check for invalid values
            if (!IsValid(worldTarget) || !IsValid(ctx.Position) || !IsValid(targetVelocity))
            {
                ctx.Debug?.Log("Move: Invalid target or position detected, releasing thrusters");
                ctx.Thrusters.Release();
                return;
            }

            // Calculate current velocity
            Vector3D currentVelocity = ctx.Velocity;
            double currentSpeed = currentVelocity.Length();

            // Filter noise: treat very small target velocities as zero (physics jitter)
            double targetSpeed = targetVelocity.Length();
            if (targetSpeed < 0.5)
            {
                targetVelocity = Vector3D.Zero;
                targetSpeed = 0;
            }

            double positionErrorMagnitude = positionError.Length();
            double stationRadius = _stationRadiusOverride > 0
                ? _stationRadiusOverride
                : Math.Max(ctx.Config.StationRadius, 0.1);
            double holdEnterSpeed = _holdEnterSpeedOverride > 0 ? _holdEnterSpeedOverride : HOLD_ENTER_SPEED;
            double holdExitSpeed = _holdExitSpeedOverride > 0 ? _holdExitSpeedOverride : HOLD_EXIT_SPEED;
            if (targetSpeed < 0.1)
            {
                if (_holdingStation)
                {
                    if (positionErrorMagnitude <= stationRadius * HOLD_EXIT_MULTIPLIER && currentSpeed <= holdExitSpeed)
                    {
                        ctx.Thrusters.Release();
                        return;
                    }

                    _holdingStation = false;
                }
                else if (positionErrorMagnitude <= stationRadius && currentSpeed <= holdEnterSpeed)
                {
                    _holdingStation = true;
                    ctx.Thrusters.Release();
                    return;
                }
            }
            else
            {
                _holdingStation = false;
            }

            // Adjust for braking distance to prevent overshoot (only at higher speeds)
            // if (currentSpeed > 5.0 && IsValid(currentVelocity))
            // {
            //     double brakingDistance = ctx.Thrusters.GetBrakingDistance(currentSpeed, currentVelocity);
            //     if (!double.IsNaN(brakingDistance) && !double.IsInfinity(brakingDistance) && brakingDistance > 0)
            //     {
            //         Vector3D brakingVector = Vector3D.Normalize(currentVelocity) * brakingDistance * ctx.Config.BrakingSafetyMargin;
            //         if (IsValid(brakingVector))
            //         {
            //             positionError -= brakingVector;
            //         }
            //     }
            // }

            // Guard against invalid deltaTime (can happen on first tick after directive switch)
            // if (ctx.DeltaTime <= 0.0001)
            // {
            //     // Skip PID computation on first tick with invalid deltaTime
            //     ctx.Thrusters.Release();
            //     return;
            // }

            // Calculate desired velocity based on target motion state
            Vector3D desiredVelocity;

            if (targetSpeed < 0.1)
            {
                // Target is stationary - use direct velocity control with braking prediction
                // This prevents the "gentle drift" problem of PID when target velocity = 0

                double distance = positionErrorMagnitude;
                if (distance > 0.1)
                {
                    Vector3D direction = positionError / distance;

                    // Calculate braking distance at current speed
                    double currentBrakingDist = ctx.Thrusters.GetBrakingDistance(currentSpeed, currentVelocity);
                    if (double.IsNaN(currentBrakingDist) || double.IsInfinity(currentBrakingDist))
                        currentBrakingDist = 0;

                    // Determine target speed based on distance
                    // If we're far away, go fast. If we're close, slow down based on braking distance.
                    double targetApproachSpeed;

                    if (distance > currentBrakingDist * 3.0)
                    {
                        // Far away - use max speed or 100 m/s
                        targetApproachSpeed = _maxSpeed > 0 ? _maxSpeed : 100.0;
                    }
                    else
                    {
                        // Close - reduce speed proportional to remaining distance
                        // Speed where we can brake to stop within remaining distance
                        targetApproachSpeed = Math.Sqrt(2.0 * distance * 10.0); // Assume ~10 m/s² decel
                        targetApproachSpeed = Math.Min(targetApproachSpeed, _maxSpeed > 0 ? _maxSpeed : 100.0);
                        targetApproachSpeed = Math.Max(targetApproachSpeed, 1.0); // Minimum 1 m/s
                    }

                    if (distance < stationRadius * NEAR_RADIUS_MULTIPLIER)
                    {
                        double slowSpeed = Math.Max(distance / TAU, MIN_CLOSE_SPEED_NEAR);
                        targetApproachSpeed = Math.Min(targetApproachSpeed, slowSpeed);
                    }

                    desiredVelocity = direction * targetApproachSpeed;
                }
                else
                {
                    desiredVelocity = Vector3D.Zero;
                }
            }
            else
            {
                // Target is moving - use feed-forward closing velocity + intercept prediction
                // Formula: desiredVelocity = targetVelocity + direction * closingSpeed + damping

                double distance = positionError.Length();

                // Estimate time to intercept based on closing speed
                double timeToIntercept = Math.Min(distance / _maxClosingSpeed, 1);

                // Lead the target: predict where it will be when we arrive
                Vector3D leadOffset = targetVelocity * ctx.DeltaTime*4;
                Vector3D predictedError = positionError + leadOffset;
                double predictedDistance = predictedError.Length();

                //ctx.Debug?.Log($"Move: predDist={predictedDistance:F1}m");

                if (predictedDistance > 0.1)
                {
                    // Direction to predicted target position
                    Vector3D direction = predictedError / predictedDistance;
                    //Vector3D direction = positionError / predictedDistance;

                    // Feed-forward closing speed: distance / TAU, clamped
                    // This means "try to close the gap in TAU seconds"
                    double closingSpeed = predictedDistance / TAU;
                    closingSpeed = Math.Max(closingSpeed, MIN_CLOSING_SPEED);
                    closingSpeed = Math.Min(closingSpeed, _maxClosingSpeed);

                    // Small PID damping term for smooth approach (reduces oscillation)
                    Vector3D damping = _pid.Compute(positionError, ctx.DeltaTime);
                    if (!IsValid(damping))
                    {
                        damping = Vector3D.Zero;
                        _pid.Reset();
                    }

                    desiredVelocity = targetVelocity + direction * closingSpeed + damping;

                    var dir = positionError / positionError.Length();
                    var closingCmd = Vector3D.Dot(desiredVelocity - targetVelocity, dir);


                    // Debug logging
                    // if (distance > 10.0)
                    // {
                    //     ctx.Debug?.Log($"Move: close={closingCmd:F1}m/s");
                    // }
                }
                else
                {
                    // Very close - just match target velocity
                    desiredVelocity = targetVelocity;
                }
            }

            // Apply speed limit if specified
            // if (_maxSpeed > 0)
            // {
            //     double speed = desiredVelocity.Length();
            //     if (speed > _maxSpeed && speed > 0)
            //     {
            //         desiredVelocity = (desiredVelocity / speed) * _maxSpeed;
            //     }
            // }

            // Final safety check before commanding thrusters
            if (!IsValid(desiredVelocity))
            {
                ctx.Thrusters.Release();
                return;
            }

            // Apply potential field repulsion for obstacle avoidance
            if (ctx.FieldResolver != null)
            {
                // Build effective exclusion set (static + deferred)
                HashSet<long> effectiveExclusions = _exclusions;
                if (_deferredExclusions.Count > 0 || _deferredExclusionLists.Count > 0)
                {
                    if (_effectiveExclusions == null)
                    {
                        _effectiveExclusions = new HashSet<long>(_exclusions);
                    }
                    else
                    {
                        _effectiveExclusions.Clear();
                        foreach (var id in _exclusions)
                            _effectiveExclusions.Add(id);
                    }

                    effectiveExclusions = _effectiveExclusions;
                    foreach (var provider in _deferredExclusions)
                    {
                        long id = provider();
                        if (id != 0)
                            effectiveExclusions.Add(id);
                    }
                    for (int i = 0; i < _deferredExclusionLists.Count; i++)
                    {
                        var ids = _deferredExclusionLists[i]();
                        if (ids == null)
                            continue;
                        for (int j = 0; j < ids.Count; j++)
                        {
                            long id = ids[j];
                            if (id != 0)
                                effectiveExclusions.Add(id);
                        }
                    }
                }

                Vector3D repulsion = ctx.FieldResolver.ComputeRepulsion(
                    ctx.Position,
                    currentVelocity,
                    effectiveExclusions,
                    _disableTerrainRepulsion
                );

                if (IsValid(repulsion))
                {
                    desiredVelocity += repulsion;
                }
            }

            // Hard ground-avoidance guard (skip when terrain avoidance is disabled)
            if (!_disableTerrainRepulsion && ctx.Reference != null && ctx.Config.MinTerrainClearance > 0)
            {
                Vector3D gravity = ctx.Reference.GetNaturalGravity();
                if (gravity.LengthSquared() > 0.01)
                {
                    double altitude;
                    if (ctx.Reference.TryGetPlanetElevation(MyPlanetElevation.Surface, out altitude))
                    {
                        Vector3D down = Vector3D.Normalize(gravity);
                        double remaining = altitude - ctx.Config.MinTerrainClearance;
                        if (remaining <= 0.5)
                        {
                            double desiredDown = Vector3D.Dot(desiredVelocity, down);
                            if (desiredDown > 0)
                                desiredVelocity -= down * desiredDown;
                        }
                        else
                        {
                            double downSpeed = Vector3D.Dot(currentVelocity, down);
                            if (downSpeed > 0.1)
                            {
                                double brakingDist = ctx.Thrusters.GetBrakingDistance(downSpeed, down);
                                if (brakingDist >= remaining)
                                {
                                    double desiredDown = Vector3D.Dot(desiredVelocity, down);
                                    if (desiredDown > 0)
                                        desiredVelocity -= down * desiredDown;
                                }
                            }
                        }
                    }
                }
            }

            // Command thrusters - controller handles force calculation and transforms
            ctx.Thrusters.SetDesiredVelocity(desiredVelocity);
        }

        /// <summary>
        /// Checks if a vector contains valid (non-NaN, non-infinite) values.
        /// </summary>
        private static bool IsValid(Vector3D v)
        {
            return !double.IsNaN(v.X) && !double.IsInfinity(v.X) &&
                   !double.IsNaN(v.Y) && !double.IsInfinity(v.Y) &&
                   !double.IsNaN(v.Z) && !double.IsInfinity(v.Z);
        }
    }
}

/*
//I want to see how tidy the function looks with the PID logic switch
            if (_usePID)
            {
                // PID correction based on position error
                Vector3D correction = _pid.Compute(positionError, ctx.DeltaTime);

                // Desired velocity = target velocity + correction
                desiredVelocity = targetVelocity + correction;
            }
            else
            {
                // Simple proportional control for static targets
                double distance = positionError.Length();
                if (distance > 0.1)
                {
                    Vector3D direction = positionError / distance;
                    // Simple proportional speed: faster when far, slower when close
                    double speed = Math.Min(distance * 2.0, _maxSpeed > 0 ? _maxSpeed : 100.0);
                    desiredVelocity = direction * speed;
                }
                else
                {
                    desiredVelocity = Vector3D.Zero;
                }
            }
*/
