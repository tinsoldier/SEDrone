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
            // === PHASE 1: Request Docking Pad ===
            PendingDockingRequest padRequest = ctx.IGCRequests.RequestDockingPad(ctx.GameTime);

            // Wait for response (Azure Durable Functions-style await pattern)
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

            // Helper class to avoid local functions (C# 6 limitation)
            var helpers = new DockingHelpers(ctx, padResponse);

            IMyShipConnector droneConnector = ctx.DockingNav.SelectDroneConnector(
                ctx.GridTerminalSystem,
                ctx.Me.CubeGrid.EntityId,
                helpers.GetTargetConnectorForward()
            );

            if (droneConnector == null)
            {
                yield return BehaviorIntent.Aborted(AbortReason.Damaged,
                    "No suitable connector found on drone");
                yield break;
            }

            // === PHASE 3: Waypoint Generation ===

            double droneConnectorSize = DockingNavigator.GetConnectorRadius(droneConnector);
            double targetConnectorSize = padResponse.ConnectorSize;

            // Generate approach waypoints (offsets relative to connector)
            List<Vector3D> waypointOffsets = ctx.DockingNav.GenerateApproachWaypoints(
                helpers.GetTargetConnectorForward(),
                droneConnectorSize,
                targetConnectorSize
            );

            // === PHASE 4: Main Docking Loop ===

            while (ctx.HasLeaderContact)
            {
                int waypointIndex = 0;

                // Navigate approach waypoints (exclude final position)
                while (waypointIndex < waypointOffsets.Count - 1)
                {
                    int currentIndex = waypointIndex; // Capture for closure

                    yield return new BehaviorIntent
                    {
                        Position = new Approach(() => helpers.GetWaypointWorldPosition(currentIndex, waypointOffsets)),
                        Orientation = new LookAt(() => helpers.GetWaypointWorldPosition(currentIndex, waypointOffsets)),
                        ExitWhen = () => ctx.DistanceTo(helpers.GetWaypointWorldPosition(currentIndex, waypointOffsets)) < 3.0
                    };

                    waypointIndex++;
                }

                // === PHASE 5: Final Docking Approach ===

                yield return new BehaviorIntent
                {
                    Position = new Approach(
                        () => helpers.GetDockingApproach(droneConnectorSize, targetConnectorSize).Position), // Slow final approach ,speed: 2.0
                    Orientation = new LookDirection(() => helpers.GetDockingApproach(droneConnectorSize, targetConnectorSize).Heading),
                    ExitWhen = () => ctx.DistanceTo(helpers.GetDockingApproach(droneConnectorSize, targetConnectorSize).Position) < 0.5
                };

                // === PHASE 6: Connector Lock Attempt ===

                // Match leader velocity precisely while attempting lock
                yield return new BehaviorIntent
                {
                    Position = new FormationFollow(() => padResponse.ConnectorOffset),
                    Orientation = new LookDirection(() => helpers.GetDockingApproach(droneConnectorSize, targetConnectorSize).Heading),
                    ExitWhen = () =>
                    {
                        if (droneConnector.Status == MyShipConnectorStatus.Connected)
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
                    yield return BehaviorIntent.Complete();
                    yield break;
                }

                // Attempt connector lock if connectable
                if (droneConnector.Status == MyShipConnectorStatus.Connectable)
                {
                    droneConnector.Connect();

                    // Wait briefly for connection
                    yield return new BehaviorIntent
                    {
                        Position = new FormationFollow(() => padResponse.ConnectorOffset),
                        Orientation = new LookDirection(() => helpers.GetDockingApproach(droneConnectorSize, targetConnectorSize).Heading),
                        ExitWhen = () => droneConnector.Status == MyShipConnectorStatus.Connected
                    };

                    if (droneConnector.Status == MyShipConnectorStatus.Connected)
                    {
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

            public DockingHelpers(DroneContext ctx, DockingPadResponse response)
            {
                _ctx = ctx;
                _response = response;
            }

            public Vector3D GetTargetConnectorForward()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                Vector3D leaderRight = Vector3D.Cross(_ctx.LastLeaderState.Forward, _ctx.LastLeaderState.Up);
                return leaderRight * _response.ConnectorForward.X +
                       _ctx.LastLeaderState.Up * _response.ConnectorForward.Y +
                       _ctx.LastLeaderState.Forward * _response.ConnectorForward.Z;
            }

            public Vector3D GetConnectorPosition()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;

                Vector3D leaderRight = Vector3D.Cross(_ctx.LastLeaderState.Forward, _ctx.LastLeaderState.Up);
                return _ctx.LastLeaderState.Position +
                    leaderRight * _response.ConnectorOffset.X +
                    _ctx.LastLeaderState.Up * _response.ConnectorOffset.Y +
                    _ctx.LastLeaderState.Forward * _response.ConnectorOffset.Z;
            }

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
        }
    }
}
