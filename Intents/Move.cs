using System;
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
                    double kp = 1.0, double ki = 0.1, double kd = 0.5)
        {
            _targetFunc = worldPositionFunc;
            _isRelative = false;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
        }

        // ============================================================
        // Dynamic World Position with Velocity Matching
        // ============================================================

        /// <summary>
        /// Move to a dynamic world position and match its velocity.
        /// Use for formation flying or intercept.
        /// </summary>
        public Move(Func<Vector3D> worldPositionFunc, Func<Vector3D> targetVelocityFunc,
                    double maxSpeed = -1, double kp = 1.0, double ki = 0.1, double kd = 0.5)
        {
            _targetFunc = worldPositionFunc;
            _targetVelocityFunc = targetVelocityFunc;
            _isRelative = false;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
        }

        // ============================================================
        // Relative Positioning (local offset)
        // ============================================================

        /// <summary>
        /// Move to a position relative to a reference point.
        /// Example: offset=(0, -5, 15) relative to leader position
        /// </summary>
        public Move(Vector3D localOffset, Func<Vector3D> referenceFunc,
                    double maxSpeed = -1, double kp = 1.0, double ki = 0.1, double kd = 0.5)
        {
            _targetFunc = () => localOffset;
            _referenceFunc = referenceFunc;
            _isRelative = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
        }

        /// <summary>
        /// Move to a dynamic offset relative to a reference point.
        /// Example: dynamic offset relative to leader for complex formations.
        /// </summary>
        public Move(Func<Vector3D> localOffsetFunc, Func<Vector3D> referenceFunc,
                    Func<Vector3D> targetVelocityFunc,
                    double maxSpeed = -1, double kp = 1.0, double ki = 0.1, double kd = 0.5)
        {
            _targetFunc = localOffsetFunc;
            _referenceFunc = referenceFunc;
            _targetVelocityFunc = targetVelocityFunc;
            _isRelative = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
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
        public Move(Vector3D localOffset, Func<IOrientedReference> orientedRefFunc,
                    double maxSpeed = -1, double kp = 1.0, double ki = 0.1, double kd = 0.5)
        {
            _targetFunc = () => localOffset;
            _orientedRefFunc = orientedRefFunc;
            _isFormation = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
        }

        /// <summary>
        /// Formation flying with dynamic offset.
        /// Example: complex formations where offset changes over time.
        /// </summary>
        public Move(Func<Vector3D> localOffsetFunc, Func<IOrientedReference> orientedRefFunc,
                    double maxSpeed = -1, double kp = 1.0, double ki = 0.1, double kd = 0.5)
        {
            _targetFunc = localOffsetFunc;
            _orientedRefFunc = orientedRefFunc;
            _isFormation = true;
            _pid = new PIDController3D(kp, ki, kd);
            _maxSpeed = maxSpeed;
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

                // Build orientation basis: Right = Up × Forward
                Vector3D right = Vector3D.Cross(orientedRef.Up, orientedRef.Forward);

                // Transform local offset to world space
                Vector3D worldOffset =
                    right * localOffset.X +
                    orientedRef.Up * localOffset.Y +
                    orientedRef.Forward * localOffset.Z;

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
                ctx.Thrusters.Release();
                return;
            }

            // Adjust for braking distance to prevent overshoot
            Vector3D currentVelocity = ctx.Velocity;
            double currentSpeed = currentVelocity.Length();

            if (currentSpeed > 0.1 && IsValid(currentVelocity))
            {
                double brakingDistance = ctx.Thrusters.GetBrakingDistance(currentSpeed, currentVelocity);
                if (!double.IsNaN(brakingDistance) && !double.IsInfinity(brakingDistance) && brakingDistance > 0)
                {
                    Vector3D brakingVector = Vector3D.Normalize(currentVelocity) * brakingDistance * ctx.Config.BrakingSafetyMargin;
                    if (IsValid(brakingVector))
                    {
                        positionError -= brakingVector;
                    }
                }
            }

            // PID correction based on position error
            Vector3D correction = _pid.Compute(positionError, ctx.DeltaTime);

            // Safety check on PID output
            if (!IsValid(correction))
            {
                ctx.Thrusters.Release();
                _pid.Reset();
                return;
            }

            // Calculate desired velocity
            // Desired velocity = target velocity + correction
            Vector3D desiredVelocity = targetVelocity + correction;

            // Apply speed limit if specified
            if (_maxSpeed > 0)
            {
                double speed = desiredVelocity.Length();
                if (speed > _maxSpeed && speed > 0)
                {
                    desiredVelocity = (desiredVelocity / speed) * _maxSpeed;
                }
            }

            // Final safety check before commanding thrusters
            if (!IsValid(desiredVelocity))
            {
                ctx.Thrusters.Release();
                return;
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