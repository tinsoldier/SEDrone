using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Fast docking directive using stateless, reentrant logic.
    /// Inspired by Spug's SEAD2 AutoLandToConnector method.
    ///
    /// Unlike DockDirective (which uses explicit phases with state tracking),
    /// this version makes all decisions each tick based purely on current position
    /// and geometry. State emerges naturally from the drone's position relative
    /// to the connector.
    ///
    /// Key characteristics:
    /// - Stateless: No phase tracking, all decisions from current geometry
    /// - Reentrant: Can be interrupted and resumed without issues
    /// - Predictive: Uses delta-time to predict connector movement
    /// - Simple: Clear if/else branching rather than state machine
    /// </summary>
    public class FastDockDirective : IDirective
    {
        public string Name { get { return "FastDock"; } }

        // === Configuration Constants (Spug-inspired) ===
        private const double SIDEWAYS_DIST_NEEDED = 3.0;        // Must be within 3m laterally
        private const double HEIGHT_CLEARANCE = 6.0;             // Start approach 6m out along connector axis
        private const double ROTATION_ACCURACY_RAD = 0.035;      // ~2 degrees alignment tolerance
        private const double DIRECTION_ACCURACY_DEG = 15.0;      // Must be within 15 degrees to proceed
        private const double VELOCITY_DAMPENER = 0.8;            // Reduce prediction for high-speed targets

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            // === PHASE 1: Request Docking Pad ===
            PendingDockingRequest padRequest = ctx.IGCRequests.RequestDockingPad(ctx.GameTime);

            // Wait for response
            while (padRequest.IsPending)
            {
                yield return new BehaviorIntent
                {
                    Position = new FormationFollow(() => ctx.GetFormationPosition()),
                    Orientation = new MatchLeader(),
                    ExitWhen = () => !padRequest.IsPending || !ctx.HasLeaderContact
                };

                if (!ctx.HasLeaderContact)
                {
                    yield return BehaviorIntent.Aborted(AbortReason.LostLeader,
                        "Lost leader contact while requesting docking pad");
                    yield break;
                }
            }

            // Get the response
            DockingPadResponse padResponse;
            if (!padRequest.TryGetResult(out padResponse))
            {
                yield return BehaviorIntent.Aborted(AbortReason.Timeout,
                    "Docking pad request timed out");
                yield break;
            }

            if (!padResponse.Available)
            {
                yield return BehaviorIntent.Aborted(AbortReason.TargetLost,
                    "No docking pad available on leader");
                yield break;
            }

            // === PHASE 2: Connector Selection ===
            var tempHelpers = new FastDockHelpers(ctx, padResponse, null);
            IMyShipConnector droneConnector = ctx.DockingNav.SelectDroneConnector(
                ctx.GridTerminalSystem,
                ctx.Me.CubeGrid.EntityId,
                tempHelpers.GetTargetConnectorForward()
            );

            if (droneConnector == null)
            {
                yield return BehaviorIntent.Aborted(AbortReason.Damaged,
                    "No suitable connector found on drone");
                yield break;
            }

            var helpers = new FastDockHelpers(ctx, padResponse, droneConnector);

            // === PHASE 3: Stateless Docking Loop ===
            // This is the key difference: no explicit phase tracking.
            // Each tick we evaluate position and choose the appropriate behavior.

            double droneConnectorSize = DockingNavigator.GetConnectorRadius(droneConnector);
            double targetConnectorSize = padResponse.ConnectorSize;
            double heightNeeded = HEIGHT_CLEARANCE + droneConnectorSize + targetConnectorSize;

            // Use only connector-based alignment (no overall drone orientation requirement)
            // This allows the drone to maintain any orientation as long as the connector aligns
            // Use current drone heading as the "up" constraint to avoid forcing rotation
            var connectorOrientation = new AlignConnector(
                droneConnector,
                () => -helpers.GetTargetConnectorForward(),
                () => ctx.Reference.WorldMatrix.Forward  // Maintain current heading
            );

            // === Main Stateless Loop ===
            // Each iteration: check status → evaluate geometry → yield ONE intent → repeat
            while (ctx.HasLeaderContact)
            {
                // Check if already connected - stay docked
                if (droneConnector.Status == MyShipConnectorStatus.Connected)
                {
                    ctx.ActiveConnector = droneConnector;
                    ctx.SetDampeners(false);

                    // Yield single intent to hold docked state
                    yield return new BehaviorIntent
                    {
                        Position = null,
                        Orientation = null,
                        ExitWhen = () => droneConnector.Status != MyShipConnectorStatus.Connected
                    };

                    // If we get here, connector was disconnected
                    ctx.SetDampeners(true);
                    ctx.ActiveConnector = null;
                    yield return BehaviorIntent.Complete();
                    yield break;
                }

                // Check if connectable AND well-aligned - attempt lock immediately
                // Spug's approach: only connect when both conditions met in same tick
                if (droneConnector.Status == MyShipConnectorStatus.Connectable)
                {
                    // Check rotation accuracy
                    Vector3D targetForward = helpers.GetTargetConnectorForward();
                    Vector3D droneForward = droneConnector.WorldMatrix.Forward;
                    double alignmentDot = Vector3D.Dot(droneForward, -targetForward);
                    double angleError = Math.Acos(MathHelper.Clamp(alignmentDot, -1, 1));

                    if (angleError < ROTATION_ACCURACY_RAD)
                    {
                        // Both conditions met - connect NOW
                        droneConnector.Connect();

                        // If it connected immediately, loop back to check docked state
                        if (droneConnector.Status == MyShipConnectorStatus.Connected)
                        {
                            continue;
                        }
                    }

                    // If we get here, we're connectable but either:
                    // 1. Not aligned well enough, OR
                    // 2. Connection attempt didn't complete immediately
                    // Fall through to position/align logic
                }

                // === Stateless Position-Based Decision Making ===
                // This is where Spug's approach shines: pure geometry, no state tracking

                // Get raw target connector position for geometry calculations
                Vector3D targetConnectorWorldPos = helpers.GetPredictedConnectorPosition();
                Vector3D connectorForward = helpers.GetTargetConnectorForward();
                Vector3D connectorUp = helpers.GetTargetConnectorUp();

                // Get drone connector position for geometry
                Vector3D currentPosition = droneConnector.GetPosition();

                // Calculate geometry (Spug's method)
                Vector3D pointOnAxis = NearestPointOnLine(targetConnectorWorldPos, connectorForward, currentPosition);
                Vector3D heightDifference = pointOnAxis - targetConnectorWorldPos;
                double signedHeightDistance = Vector3D.Dot(
                    connectorForward,
                    Vector3D.Normalize(heightDifference)) * heightDifference.Length();
                double sidewaysDistance = (currentPosition - pointOnAxis).Length();

                // Check alignment before proceeding with movement
                Vector3D toConnector = targetConnectorWorldPos - currentPosition;
                double distanceToConnector = toConnector.Length();
                Vector3D directionToConnector = distanceToConnector > 0.1
                    ? Vector3D.Normalize(toConnector)
                    : Vector3D.Zero;

                // Rough alignment check (using simple angle)
                double alignmentAngle = 180.0;
                if (directionToConnector != Vector3D.Zero)
                {
                    Vector3D droneForward = droneConnector.WorldMatrix.Forward;
                    double dot = Vector3D.Dot(droneForward, -connectorForward);
                    alignmentAngle = Math.Acos(MathHelper.Clamp(dot, -1, 1)) * (180.0 / Math.PI);
                }

                // Decision tree based on position (Spug's logic)
                // KEY: Yield ONE intent, exit immediately, let loop re-evaluate
                // Added: Prioritize final approach once close, avoid oscillation

                // Priority 1: If very misaligned, orient first
                if (alignmentAngle > DIRECTION_ACCURACY_DEG)
                {
                    // Not aligned enough - hold formation position while orienting
                    yield return new BehaviorIntent
                    {
                        Position = new Approach(() => ctx.GetFormationPosition()),
                        Orientation = connectorOrientation,
                        ExitWhen = () => true  // Exit immediately, re-evaluate next tick
                    };
                }
                // Priority 2: If close AND well-aligned laterally, commit to final approach
                // This prevents oscillation when nearly aligned
                else if (distanceToConnector < heightNeeded * 1.5 && sidewaysDistance <= SIDEWAYS_DIST_NEEDED * 1.5)
                {
                    // Close to connector AND reasonably aligned - commit to final approach
                    // The 1.5x multiplier on sideways check provides some tolerance to prevent oscillation
                    yield return new BehaviorIntent
                    {
                        Position = new Approach(() => helpers.GetWaypointAtDistance(droneConnectorSize + targetConnectorSize),
                            speedLimit: ctx.Config.DockingFinalSpeed),
                        Orientation = connectorOrientation,
                        ExitWhen = () => true  // Exit immediately, re-evaluate next tick
                    };
                }
                // Priority 3: Behind connector and far sideways - reposition
                else if (sidewaysDistance > SIDEWAYS_DIST_NEEDED &&
                         signedHeightDistance < heightNeeded * 0.5)  // Only if REALLY behind
                {
                    // Behind connector - move to correct side first
                    // Clamp repositioning distance to prevent launching away
                    yield return new BehaviorIntent
                    {
                        Position = new Approach(() =>
                        {
                            Vector3D targetPos = helpers.GetPredictedConnectorPosition();
                            Vector3D forward = helpers.GetTargetConnectorForward();
                            // Simply go to holding position, don't try to calculate fancy reposition
                            return helpers.GetWaypointAtDistance(heightNeeded);
                        },
                        speedLimit: ctx.Config.DockingApproachSpeed),
                        Orientation = connectorOrientation,
                        ExitWhen = () => true  // Exit immediately, re-evaluate next tick
                    };
                }
                // Priority 4: Sideways offset too large - approach holding position
                else if (sidewaysDistance > SIDEWAYS_DIST_NEEDED)
                {
                    yield return new BehaviorIntent
                    {
                        Position = new Approach(() => helpers.GetWaypointAtDistance(heightNeeded),
                            speedLimit: ctx.Config.DockingApproachSpeed),
                        Orientation = connectorOrientation,
                        ExitWhen = () => true  // Exit immediately, re-evaluate next tick
                    };
                }
                // Priority 5: Default - final landing approach
                else
                {
                    yield return new BehaviorIntent
                    {
                        Position = new Approach(() => helpers.GetWaypointAtDistance(droneConnectorSize + targetConnectorSize),
                            speedLimit: ctx.Config.DockingFinalSpeed),
                        Orientation = connectorOrientation,
                        ExitWhen = () => true  // Exit immediately, re-evaluate next tick
                    };
                }
            }

            // Lost leader contact
            yield return BehaviorIntent.Aborted(AbortReason.LostLeader);
        }

        /// <summary>
        /// Finds nearest point on a line to a given point.
        /// Spug's PID.NearestPointOnLine equivalent.
        /// </summary>
        private Vector3D NearestPointOnLine(Vector3D linePoint, Vector3D lineDirection, Vector3D point)
        {
            Vector3D toPoint = point - linePoint;
            double projection = Vector3D.Dot(toPoint, lineDirection);
            return linePoint + lineDirection * projection;
        }

        /// <summary>
        /// Helper class for coordinate transformations and predictions.
        /// </summary>
        private class FastDockHelpers
        {
            private DroneContext _ctx;
            private DockingPadResponse _response;
            private IMyShipConnector _droneConnector;

            public FastDockHelpers(DroneContext ctx, DockingPadResponse response, IMyShipConnector droneConnector)
            {
                _ctx = ctx;
                _response = response;
                _droneConnector = droneConnector;
            }

            private Vector3D GetReferenceToConnectorOffset()
            {
                if (_droneConnector == null) return Vector3D.Zero;
                return _droneConnector.GetPosition() - _ctx.Reference.GetPosition();
            }

            private MatrixD GetCoordinateMatrix()
            {
                Vector3D leaderPos = _ctx.LastLeaderState.Position;
                Vector3D leaderForward = _ctx.LastLeaderState.Forward;
                Vector3D leaderLeft = _ctx.LastLeaderState.Left;

                return MatrixD.CreateWorld(
                    leaderPos,
                    leaderForward,
                    Vector3D.Cross(-leaderLeft, leaderForward)
                );
            }

            public Vector3D GetTargetConnectorForward()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;
                MatrixD coordMatrix = GetCoordinateMatrix();
                return Vector3D.TransformNormal(_response.ConnectorForward, coordMatrix);
            }

            public Vector3D GetTargetConnectorUp()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;
                MatrixD coordMatrix = GetCoordinateMatrix();
                return Vector3D.TransformNormal(_response.ConnectorUp, coordMatrix);
            }

            /// <summary>
            /// Gets connector position with velocity prediction (Spug's approach).
            /// Uses a default prediction time of ~1/6th second (10 ticks at 60fps).
            /// </summary>
            public Vector3D GetPredictedConnectorPosition(double deltaTime = 0.166)
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                MatrixD coordMatrix = GetCoordinateMatrix();
                Vector3D targetConnectorWorldPos = Vector3D.Transform(_response.ConnectorOffset, coordMatrix);

                // Apply velocity prediction (Spug uses DeltaTimeReal)
                Vector3D stationVelocity = _ctx.LastLeaderState.Velocity;
                double speedDampener = 1.0 - ((stationVelocity.Length() / 100.0) * 0.2 * VELOCITY_DAMPENER);
                Vector3D prediction = deltaTime * stationVelocity * speedDampener;

                return targetConnectorWorldPos + prediction;
            }

            /// <summary>
            /// Gets connector position adjusted for reference block offset.
            /// Used for behavior targets (reference block positioning).
            /// </summary>
            public Vector3D GetConnectorPosition()
            {
                Vector3D targetPos = GetPredictedConnectorPosition();
                Vector3D referenceToConnectorOffset = GetReferenceToConnectorOffset();
                return targetPos - referenceToConnectorOffset;
            }

            /// <summary>
            /// Gets a waypoint at a specific distance along the connector's forward axis.
            /// Returns reference-adjusted position for Approach behaviors.
            /// </summary>
            public Vector3D GetWaypointAtDistance(double distance)
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                Vector3D connectorPos = GetConnectorPosition();
                Vector3D connectorForward = GetTargetConnectorForward();
                return connectorPos + connectorForward * distance;
            }

            /// <summary>
            /// Adjusts a world position to account for reference-to-connector offset.
            /// Converts connector-relative positions to reference-relative positions.
            /// </summary>
            public Vector3D AdjustForReferenceBlock(Vector3D worldPosition)
            {
                if (_droneConnector == null) return worldPosition;
                Vector3D referenceToConnectorOffset = GetReferenceToConnectorOffset();
                return worldPosition - referenceToConnectorOffset;
            }
        }
    }
}
