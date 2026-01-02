using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Directive for autonomous docking with leader grid.
    ///
    /// Flow:
    /// 1. Send IGC request to leader for docking pad assignment
    /// 2. Wait for response with connector info and optional waypoints
    /// 3. Select appropriate drone connector (axis-aligned with target)
    /// 4. Navigate through approach waypoints
    /// 5. Final precision approach to docking position
    /// 6. Attempt connector lock
    ///
    /// Inspired by SEAD2 (Spug's Easy Auto-Docking 2) but adapted to
    /// the declarative directive/intent architecture.
    /// </summary>
    public class DockDirective : IDirective
    {
        public string Name { get { return "Dock"; } }

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            ctx.Debug?.Log($"Dock: Starting Docking");
            // === PHASE 1: Request Docking Pad ===
            PendingDockingRequest padRequest = ctx.IGCRequests.RequestDockingPad(ctx.GameTime);

            // Wait for response (Azure Durable Functions-style await pattern)
            while (padRequest.IsPending)
            {
                yield return new BehaviorIntent
                {
                    Position = new Move(() => ctx.GetFormationPosition(), () => ctx.LastLeaderState.Velocity),
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

            ctx.Debug?.Log($"Dock: Waiting for response");
            // Get the response
            DockingPadResponse padResponse;
            if (!padRequest.TryGetResult(out padResponse))
            {
                yield return BehaviorIntent.Aborted(AbortReason.Timeout,
                    "Docking pad request timed out");
                yield break;
            }

            ctx.Debug?.Log($"Dock: Received response");

            if (!padResponse.Available)
            {
                ctx.Debug?.Log($"Dock: No docking pad available on leader");
                yield return BehaviorIntent.Aborted(AbortReason.TargetLost,
                    "No docking pad available on leader");
                yield break;
            }

            // === PHASE 2: Connector Selection ===

            // Temporary helper to get target connector forward (before we have drone connector)
            var tempHelpers = new DockingHelpers(ctx, padResponse, null);

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

            // Now create helpers with the drone connector for position calculations
            var helpers = new DockingHelpers(ctx, padResponse, droneConnector);

            // === PHASE 3: Waypoint Generation ===

            double droneConnectorSize = DockingNavigator.GetConnectorRadius(droneConnector);
            double targetConnectorSize = padResponse.ConnectorSize;

            // Generate approach waypoint distances (scalar distances along connector forward)
            // These will be transformed to world positions dynamically each tick
            List<double> waypointDistances = ctx.DockingNav.GenerateApproachDistances(
                droneConnectorSize,
                targetConnectorSize
            );

            // Create AlignConnector behavior for proper orientation during docking
            // The drone connector should face OPPOSITE to the target connector (anti-parallel)
            var dockingOrientation = new AlignConnector(
                droneConnector,
                () => -helpers.GetTargetConnectorForward(),  // Face opposite direction
                () => helpers.GetTargetConnectorUp()         // Match target's up
            );

            // === PHASE 4: Main Docking Loop ===

            while (ctx.HasLeaderContact)
            {
                int waypointIndex = 0;

                // Navigate approach waypoints (exclude final position)
                while (waypointIndex < waypointDistances.Count - 1)
                {
                    int currentIndex = waypointIndex; // Capture for closure
                    ctx.Debug?.Log($"Dock: Phase 4 - Waypoint {currentIndex + 1}/{waypointDistances.Count}");

                    if(currentIndex == 0)
                    {
                        // First waypoint: ensure velocity is arrested and matched to leader
                        // Use level turn to prevent rolling during the initial 180° rotation
                        yield return new BehaviorIntent
                        {
                            Position = new Move(
                                new Vector3D(0, 0, waypointDistances[currentIndex]),  // Local offset along connector forward
                                () => helpers.GetConnectorReference()),
                            Orientation = new LevelTurnToward(() => helpers.GetWaypointAtDistance(waypointDistances[currentIndex]) + helpers.GetTargetConnectorUp() * 20.0),
                            ExitWhen = () =>
                            {
                                // Check distance
                                if (ctx.DistanceTo(helpers.GetWaypointAtDistance(waypointDistances[currentIndex])) >= 3.0)
                                {
                                    //ctx.Debug?.Log("Dock: Not yet at first waypoint");
                                    return false;
                                }

                                // Check velocity matching (within 1 m/s of leader's velocity magnitude)
                                Vector3D relativeVelocity = ctx.Velocity - ctx.LastLeaderState.Velocity;
                                if (relativeVelocity.Length() > 1.0)
                                {
                                    //ctx.Debug?.Log($"Dock: Velocity not yet matched (relVel={relativeVelocity.Length():F2} m/s)");
                                    return false;
                                }

                                // Check orientation alignment (within 5 degrees)
                                // if(!ctx.Gyros.IsLevel(5.0))
                                //     return false;

                                if(!ctx.Gyros.IsAligned)
                                {   
                                    //ctx.Debug?.Log("Dock: Orientation not yet aligned");
                                    return false;
                                }

                                //ctx.Debug?.Log("Dock: Reached first waypoint with velocity matched");
                                return true;
                            }
                        };
                    }
                    else
                    {
                        yield return new BehaviorIntent
                        {
                            Position = new Move(
                                new Vector3D(0, 0, waypointDistances[currentIndex]),  // Local offset along connector forward
                                () => helpers.GetConnectorReference(),
                                closingSpeed: ctx.Config.DockingApproachSpeed),
                            Orientation = dockingOrientation,
                            ExitWhen = () => ctx.DistanceTo(helpers.GetWaypointAtDistance(waypointDistances[currentIndex])) < 3.0
                        };
                    }

                    waypointIndex++;
                }

                // === PHASE 5: Final Docking Approach ===
                ctx.Debug?.Log($"Dock: Phase 5 - Final approach");

                yield return new BehaviorIntent
                {
                    Position = new Move(
                        () => helpers.GetDockingApproachOffset(droneConnectorSize, targetConnectorSize),
                        () => helpers.GetConnectorReference(),
                        closingSpeed: ctx.Config.DockingFinalSpeed),
                    Orientation = dockingOrientation,
                    // Measure from drone connector to target - not from ship controller
                    ExitWhen = () => Vector3D.Distance(droneConnector.GetPosition(), helpers.GetConnectorPosition()) < (droneConnectorSize + targetConnectorSize + 1.0)
                };

                // === PHASE 6: Connector Lock Attempt ===
                ctx.Debug?.Log($"Dock: Phase 6 - Lock attempt (status={droneConnector.Status})");

                // Move to one connector radius away (let magnetic lock pull them together gently)
                // This prevents the "bump" by stopping short and letting connector magnets do final pull
                double finalApproachDistance = droneConnectorSize + targetConnectorSize;
                yield return new BehaviorIntent
                {
                    Position = new Move(
                        new Vector3D(0, 0, finalApproachDistance),
                        () => helpers.GetConnectorReference(),
                        closingSpeed: ctx.Config.DockingFinalSpeed),
                    Orientation = dockingOrientation,
                    ExitWhen = () =>
                    {
                        // Exit when connected. e.g auto-connect or manual
                        if (droneConnector.Status == MyShipConnectorStatus.Connected)
                            return true;

                        // Exit when connectable so we can attempt lock
                        if (droneConnector.Status == MyShipConnectorStatus.Connectable)
                            return true;

                        // Timeout if taking too long
                        if (ctx.DistanceTo(helpers.GetConnectorPosition()) > 10.0)
                            return true;

                        return false;
                    }
                };

                // Check if docking succeeded
                if (droneConnector.Status == MyShipConnectorStatus.Connected)
                {
                    ctx.Debug?.Log("Dock: Connected!");
                    yield return BehaviorIntent.Complete();
                    yield break;
                }

                // Attempt connector lock if connectable
                ctx.Debug?.Log($"Dock: Attempting lock (status={droneConnector.Status})");
                if (droneConnector.Status == MyShipConnectorStatus.Connectable)
                {
                    droneConnector.Connect();

                    // Hold position while waiting for connection to establish
                    // Keep holding at one connector radius away - connector magnets will pull us in
                    // Use a timeout to avoid infinite loop
                    double lockStartTime = ctx.GameTime;
                    yield return new BehaviorIntent
                    {
                        Position = new Move(
                            new Vector3D(0, 0, finalApproachDistance),
                            () => helpers.GetConnectorReference(),
                            closingSpeed: ctx.Config.DockingLockSpeed),
                        Orientation = dockingOrientation,
                        ExitWhen = () => {
                            droneConnector.Connect();
                            return droneConnector.Status == MyShipConnectorStatus.Connected ||
                            (ctx.GameTime - lockStartTime) > 6.0;  // 3 second timeout
                        }
                    };

                    if (droneConnector.Status == MyShipConnectorStatus.Connected)
                    {
                        // === PHASE 7: Docked - remain until undocked ===
                        
                        // Store connector reference for other code to check docked state
                        ctx.ActiveConnector = droneConnector;
                        
                        // Disable dampeners to avoid fighting parent grid movement
                        ctx.SetDampeners(false);
                        
                        // Wait while docked - exit when disconnected
                        while (droneConnector.Status == MyShipConnectorStatus.Connected)
                        {
                            yield return new BehaviorIntent
                            {
                                Position = null,      // No movement while docked
                                Orientation = null,   // No rotation while docked
                                ExitWhen = () => droneConnector.Status != MyShipConnectorStatus.Connected
                            };
                        }
                        
                        // Disconnected - re-enable dampeners (also done by ThrusterController as safety)
                        ctx.SetDampeners(true);
                        ctx.ActiveConnector = null;
                        
                        // Exit directive - will fall through to EscortDirective
                        yield return BehaviorIntent.Complete();
                        yield break;
                    }
                }

                // Docking failed - loop will retry from waypoints
            }

            // Lost leader contact
            yield return BehaviorIntent.Aborted(AbortReason.LostLeader);
        }

        /// <summary>
        /// Helper class to avoid local functions (C# 6 doesn't support them).
        /// Encapsulates coordinate transformations for docking.
        /// </summary>
        private class DockingHelpers
        {
            private DroneContext _ctx;
            private DockingPadResponse _response;
            private IMyShipConnector _droneConnector;
            private int _debugCounter = 0;

            public DockingHelpers(DroneContext ctx, DockingPadResponse response, IMyShipConnector droneConnector)
            {
                _ctx = ctx;
                _response = response;
                _droneConnector = droneConnector;
            }

            /// <summary>
            /// Gets the offset from the drone's reference block to its connector.
            /// Used to adjust target positions so the connector ends up in the right place.
            /// </summary>
            private Vector3D GetReferenceToConnectorOffset()
            {
                if (_droneConnector == null) return Vector3D.Zero;
                return _droneConnector.GetPosition() - _ctx.Reference.GetPosition();
            }

            /// <summary>
            /// Builds the coordinate matrix exactly like DockingPadManager does.
            /// CRITICAL: Must match the encoding side to ensure correct transformations.
            /// Uses SEAD2's formula: Matrix.CreateWorld(pos, forward, (-left)×forward)
            /// </summary>
            private MatrixD GetCoordinateMatrix()
            {
                Vector3D leaderPos = _ctx.LastLeaderState.Position;
                Vector3D leaderForward = _ctx.LastLeaderState.Forward;
                Vector3D leaderLeft = _ctx.LastLeaderState.Left;

                // SEAD2 formula: up = (-left) × forward
                // This MUST match DockingPadManager's encoding!
                return MatrixD.CreateWorld(
                    leaderPos,
                    leaderForward,
                    Vector3D.Cross(-leaderLeft, leaderForward)
                );
            }

            /// <summary>
            /// Transforms connector forward from leader-local to world space.
            /// Uses SEAD2's method: Vector3D.TransformNormal with the coordinate matrix.
            /// </summary>
            public Vector3D GetTargetConnectorForward()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                MatrixD coordMatrix = GetCoordinateMatrix();
                return Vector3D.TransformNormal(_response.ConnectorForward, coordMatrix);
            }

            /// <summary>
            /// Transforms connector up from leader-local to world space.
            /// Uses SEAD2's method: Vector3D.TransformNormal with the coordinate matrix.
            /// </summary>
            public Vector3D GetTargetConnectorUp()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                MatrixD coordMatrix = GetCoordinateMatrix();
                return Vector3D.TransformNormal(_response.ConnectorUp, coordMatrix);
            }

            /// <summary>
            /// Gets the connector world position (dynamically updated each tick).
            /// Uses SEAD2's method: Vector3D.Transform with the coordinate matrix.
            /// Adjusted for drone reference-to-connector offset so behaviors position the connector correctly.
            /// </summary>
            public Vector3D GetConnectorPosition()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                MatrixD coordMatrix = GetCoordinateMatrix();
                Vector3D targetConnectorWorldPos = Vector3D.Transform(_response.ConnectorOffset, coordMatrix);

                // Adjust for reference-to-connector offset
                // We want: droneConnector.GetPosition() == targetConnectorWorldPos
                // Since: droneConnector.GetPosition() = referenceBlock.GetPosition() + offset
                // Therefore: referenceBlock target = targetConnectorWorldPos - offset
                Vector3D referenceToConnectorOffset = GetReferenceToConnectorOffset();
                Vector3D referenceTargetPos = targetConnectorWorldPos - referenceToConnectorOffset;

                // DEBUG: Log decoded world position (every 10th call to reduce spam)
                _debugCounter++;
                if (_debugCounter >= 10)
                {
                    _debugCounter = 0;
                    var debugBlock = _ctx.GridTerminalSystem.GetBlockWithName("Debug") as IMyTextPanel;
                    if (debugBlock != null)
                    {
                        Vector3D droneConnectorPos = _droneConnector != null ? _droneConnector.GetPosition() : _ctx.Reference.GetPosition();
                        Vector3D offset = targetConnectorWorldPos - droneConnectorPos;
                        double distance = offset.Length();

                        // string debugMsg = string.Format(
                        //     "[DECODE] Local: X={0:F2} Y={1:F2} Z={2:F2}\n" +
                        //     "Target Connector: {3:F2},{4:F2},{5:F2}\n" +
                        //     "Drone Connector: {6:F2},{7:F2},{8:F2}\n" +
                        //     "Offset: {9:F2},{10:F2},{11:F2} (dist={12:F2}m)",
                        //     _response.ConnectorOffset.X, _response.ConnectorOffset.Y, _response.ConnectorOffset.Z,
                        //     targetConnectorWorldPos.X, targetConnectorWorldPos.Y, targetConnectorWorldPos.Z,
                        //     droneConnectorPos.X, droneConnectorPos.Y, droneConnectorPos.Z,
                        //     offset.X, offset.Y, offset.Z, distance);
                        // debugBlock.WriteText(debugMsg + "\n", true);
                    }
                }

                return referenceTargetPos;
            }

            /// <summary>
            /// Gets a waypoint at a specific distance along the connector's forward axis.
            /// Dynamically computed each tick so it follows the leader.
            /// </summary>
            public Vector3D GetWaypointAtDistance(double distance)
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                Vector3D connectorPos = GetConnectorPosition();
                Vector3D connectorForward = GetTargetConnectorForward();
                return connectorPos + connectorForward * distance;
            }

            /// <summary>
            /// Gets world position for a pre-computed waypoint offset.
            /// (Legacy - kept for compatibility)
            /// </summary>
            public Vector3D GetWaypointWorldPosition(int index, List<Vector3D> waypointOffsets)
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                Vector3D connectorWorld = GetConnectorPosition();
                return connectorWorld + waypointOffsets[index];
            }

            public DockingApproachResult GetDockingApproach(double droneConnectorSize, double targetConnectorSize)
            {
                return _ctx.DockingNav.CalculateDockingApproach(
                    GetConnectorPosition(),
                    _ctx.LastLeaderState.Velocity,
                    GetTargetConnectorForward(),
                    droneConnectorSize,
                    targetConnectorSize
                );
            }

            /// <summary>
            /// Creates an IOrientedReference representing the target connector's reference frame.
            /// Position is adjusted for drone reference-to-connector offset.
            /// Velocity matches the leader for smooth tracking.
            /// </summary>
            public IOrientedReference GetConnectorReference()
            {
                if (!_ctx.HasLeaderContact)
                    return new ConnectorReference(Vector3D.Zero, Vector3D.Forward, Vector3D.Up, Vector3D.Zero);

                return new ConnectorReference(
                    GetConnectorPosition(),
                    GetTargetConnectorForward(),
                    GetTargetConnectorUp(),
                    _ctx.LastLeaderState.Velocity
                );
            }

            /// <summary>
            /// Gets the local offset (in connector space) for the docking approach position.
            /// This is the position just before final connector lock.
            /// </summary>
            public Vector3D GetDockingApproachOffset(double droneConnectorSize, double targetConnectorSize)
            {
                // The docking approach is a small offset along the connector forward axis
                // We can calculate this as a local offset in connector space
                DockingApproachResult approach = GetDockingApproach(droneConnectorSize, targetConnectorSize);

                // Convert world position back to local offset
                // approach.Position is in world space, we need it in connector-local space
                Vector3D connectorPos = GetConnectorPosition();
                Vector3D worldOffset = approach.Position - connectorPos;

                // Transform to local coordinates
                // Since we only care about forward distance, project onto forward axis
                Vector3D connectorForward = GetTargetConnectorForward();
                double forwardDistance = Vector3D.Dot(worldOffset, connectorForward);

                return new Vector3D(0, 0, forwardDistance);
            }
        }
    }
}
