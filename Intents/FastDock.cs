using System;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Stateless docking behavior inspired by Spug's SEAD2 AutoLandToConnector.
    ///
    /// Unlike Move-based docking, this behavior:
    /// - Makes all decisions each tick based purely on current geometry
    /// - Directly controls thrusters without relying on PID buildup
    /// - Uses velocity-based approach with simple proportional control
    /// - Handles the entire docking sequence internally
    ///
    /// The behavior is truly stateless - it can be interrupted and resumed
    /// at any point without issues.
    /// </summary>
    public class FastDock : IPositionBehavior
    {
        // === Configuration Constants ===
        private const double SIDEWAYS_DIST_NEEDED = 3.0;        // Must be within 3m laterally
        private const double HEIGHT_CLEARANCE = 6.0;            // Start approach 6m out along connector axis
        private const double ROTATION_ACCURACY_RAD = 0.08;      // Alignment tolerance for connection

        // Speed control
        private const double APPROACH_SPEED = 150.0;            // Speed when approaching hold position
        private const double FINAL_SPEED = 15.0;                 // Speed for final approach
        private const double LANDING_SPEED = 2.0;               // Speed for landing

        // Proportional gain for velocity control (Spug uses simple P control)
        private const double VELOCITY_GAIN = 4.0;               // How aggressively to correct velocity errors
        private const double LEAD_TIME_MIN = 0.0;               // Minimum seconds of target lead
        private const double LEAD_TIME_MAX = 0.25;              // Maximum seconds of target lead
        private const double BRAKE_SAFETY_FACTOR = 0.8;         // Multiplier for brake-based safe speed

        // === State ===
        private readonly IMyShipConnector _droneConnector;
        private readonly Func<IOrientedReference> _connectorRefFunc;
        private readonly double _droneConnectorSize;
        private readonly double _targetConnectorSize;
        private readonly double _heightNeeded;

        /// <summary>
        /// True when connector is connected (docking complete).
        /// </summary>
        public bool IsConnected => _droneConnector.Status == MyShipConnectorStatus.Connected;

        /// <summary>
        /// True when connector is in range and can attempt lock.
        /// </summary>
        public bool IsConnectable => _droneConnector.Status == MyShipConnectorStatus.Connectable;

        /// <summary>
        /// Creates a FastDock behavior for docking to a connector.
        /// </summary>
        /// <param name="droneConnector">The drone's connector to dock with</param>
        /// <param name="connectorRefFunc">Function returning the target connector's oriented reference</param>
        /// <param name="droneConnectorSize">Radius of drone connector</param>
        /// <param name="targetConnectorSize">Radius of target connector</param>
        public FastDock(
            IMyShipConnector droneConnector,
            Func<IOrientedReference> connectorRefFunc,
            double droneConnectorSize,
            double targetConnectorSize)
        {
            _droneConnector = droneConnector;
            _connectorRefFunc = connectorRefFunc;
            _droneConnectorSize = droneConnectorSize;
            _targetConnectorSize = targetConnectorSize;
            _heightNeeded = HEIGHT_CLEARANCE + droneConnectorSize + targetConnectorSize;
        }

        public void Execute(DroneContext ctx)
        {
            // Get current reference frame
            IOrientedReference connectorRef = _connectorRefFunc();
            if (connectorRef == null) return;

            Vector3D targetConnectorPos = connectorRef.Position;
            Vector3D connectorForward = connectorRef.Forward;
            Vector3D targetVelocity = connectorRef.Velocity;

            // Adjust target position back to where the target connector actually is
            // (connectorRef.Position is adjusted for reference-to-connector offset)
            // We need raw connector position for geometry calculations
            Vector3D referenceToConnector = _droneConnector.GetPosition() - ctx.Reference.GetPosition();
            Vector3D rawTargetConnectorPos = targetConnectorPos + referenceToConnector;

            // Get drone connector position
            Vector3D droneConnectorPos = _droneConnector.GetPosition();

            // === Calculate Geometry (Spug's method) ===
            Vector3D pointOnAxis = NearestPointOnLine(rawTargetConnectorPos, connectorForward, droneConnectorPos);
            Vector3D heightDifference = pointOnAxis - rawTargetConnectorPos;
            double heightDiffLength = heightDifference.Length();
            double signedHeightDistance = heightDiffLength > 0.01
                ? Vector3D.Dot(connectorForward, heightDifference / heightDiffLength) * heightDiffLength
                : 0;
            double sidewaysDistance = (droneConnectorPos - pointOnAxis).Length();

            // Distance to connector
            Vector3D toConnector = rawTargetConnectorPos - droneConnectorPos;
            double distanceToConnector = toConnector.Length();

            // === Determine Target Position and Speed ===
            Vector3D targetPosition;
            double maxSpeed;
            string phase;

            // Check if connectable - attempt lock
            if (_droneConnector.Status == MyShipConnectorStatus.Connectable)
            {
                // Check alignment before connecting
                Vector3D droneForward = _droneConnector.WorldMatrix.Forward;
                double alignmentDot = Vector3D.Dot(droneForward, -connectorForward);
                double angleError = Math.Acos(MathHelper.Clamp(alignmentDot, -1, 1));

                if (angleError < ROTATION_ACCURACY_RAD)
                {
                    _droneConnector.Connect();
                }
            }

            // Already connected - hold position
            if (_droneConnector.Status == MyShipConnectorStatus.Connected)
            {
                ctx.Thrusters.Release();
                return;
            }

            // Decision tree based on geometry
            double finalDist = _droneConnectorSize + _targetConnectorSize - 0.5;

            if (distanceToConnector < _heightNeeded * 1.5 && sidewaysDistance <= SIDEWAYS_DIST_NEEDED * 1.5)
            {
                // Close and aligned - final approach
                phase = "Final";
                targetPosition = targetConnectorPos + connectorForward * finalDist;
                maxSpeed = FINAL_SPEED;
            }
            else if (sidewaysDistance > SIDEWAYS_DIST_NEEDED && signedHeightDistance < _heightNeeded * 0.5)
            {
                // Behind connector - go to hold position
                phase = "Reposition";
                targetPosition = targetConnectorPos + connectorForward * _heightNeeded;
                maxSpeed = APPROACH_SPEED;
            }
            else if (sidewaysDistance > SIDEWAYS_DIST_NEEDED)
            {
                // Sideways offset - approach hold position
                phase = "Approach";
                targetPosition = targetConnectorPos + connectorForward * _heightNeeded;
                maxSpeed = APPROACH_SPEED;
            }
            else
            {
                // Aligned - landing
                phase = "Landing";
                targetPosition = targetConnectorPos + connectorForward * finalDist;
                maxSpeed = LANDING_SPEED;
            }

            //ctx.Debug?.Log($"FastDock: {phase}");

            // === Calculate Desired Velocity (Spug's approach) ===
            // Simple proportional control: velocity proportional to position error
            // Plus target velocity for moving target tracking

            double leadTime = MathHelper.Clamp(ctx.DeltaTime, LEAD_TIME_MIN, LEAD_TIME_MAX);
            Vector3D predictedTargetPosition = targetPosition + targetVelocity * leadTime;
            Vector3D positionError = predictedTargetPosition - ctx.Position;
            double errorDistance = positionError.Length();
            Vector3D currentVelocity = ctx.Velocity;
            double currentSpeed = currentVelocity.Length();

            Vector3D desiredVelocity;
            if (errorDistance > 0.1)
            {
                // Direction to target
                Vector3D direction = positionError / errorDistance;

                // Speed proportional to distance, clamped to max
                double speed = Math.Min(errorDistance * VELOCITY_GAIN, maxSpeed);

                // Brake-aware clamp based on current braking distance
                double brakingDistance = ctx.Thrusters.GetBrakingDistance(currentSpeed, currentVelocity);
                if (!double.IsNaN(brakingDistance) && !double.IsInfinity(brakingDistance) && brakingDistance > 0.01)
                {
                    double brakingAccel = (currentSpeed * currentSpeed) / (2.0 * brakingDistance);
                    if (brakingAccel > 0.01)
                    {
                        double safeSpeed = Math.Sqrt(2.0 * brakingAccel * errorDistance);
                        speed = Math.Min(speed, safeSpeed * BRAKE_SAFETY_FACTOR);
                    }
                }

                // Desired velocity = direction * speed + target velocity
                desiredVelocity = direction * speed + targetVelocity;
            }
            else
            {
                // Very close - just match target velocity
                desiredVelocity = targetVelocity;
            }

            // === Apply Thrust ===
            // Use the thruster controller's velocity path so it handles world->local,
            // mass/deltaTime scaling, gravity compensation, and axis authority.
            ctx.Thrusters.SetDesiredVelocity(desiredVelocity);
        }

        /// <summary>
        /// Finds nearest point on a line to a given point.
        /// </summary>
        private Vector3D NearestPointOnLine(Vector3D linePoint, Vector3D lineDirection, Vector3D point)
        {
            Vector3D toPoint = point - linePoint;
            double projection = Vector3D.Dot(toPoint, lineDirection);
            return linePoint + lineDirection * projection;
        }
    }
}
