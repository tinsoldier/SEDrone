using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Alternative fast docking directive using a single stateless FastDock intent.
    ///
    /// Unlike the original FastDock directive (which yields new Move intents every tick),
    /// this directive yields a single FastDock intent that runs continuously until
    /// docking is complete. This allows stable velocity control without PID reset issues.
    ///
    /// The FastDock intent handles all geometry calculations and thruster control
    /// internally, similar to Spug's SEAD2 AutoLandToConnector approach.
    /// </summary>
    public class FastDockAltDirective : IDirective
    {
        public string Name { get { return "FastDockAlt"; } }

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            ctx.Debug?.Log("FastDockAlt: Starting");

            // === PHASE 1: Request Docking Pad ===
            PendingDockingRequest padRequest = ctx.IGCRequests.RequestDockingPad(ctx.GameTime);

            // Wait for response
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

            ctx.Debug?.Log("FastDockAlt: Got docking pad response");

            // === PHASE 2: Connector Selection ===
            var helpers = new FastDockAltHelpers(ctx, padResponse, null);
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

            // Recreate helpers with drone connector
            helpers = new FastDockAltHelpers(ctx, padResponse, droneConnector);

            double droneConnectorSize = DockingNavigator.GetConnectorRadius(droneConnector);
            double targetConnectorSize = padResponse.ConnectorSize;

            ctx.Debug?.Log($"FastDockAlt: Connectors selected (drone={droneConnectorSize:F1}m, target={targetConnectorSize:F1}m)");

            // === PHASE 3: Create FastDock Intent ===
            // This single intent handles the entire docking sequence
            var fastDockIntent = new FastDock(
                droneConnector,
                () => helpers.GetConnectorReference(),
                droneConnectorSize,
                targetConnectorSize
            );

            // Orientation: align connector to target
            var dockingOrientation = new AlignConnector(
                droneConnector,
                () => -helpers.GetTargetConnectorForward(),
                () => ctx.Reference.WorldMatrix.Forward  // Maintain current heading
            );

            // === PHASE 4: Execute Docking ===
            // Yield single intent that runs until connected or lost leader
            ctx.Debug?.Log("FastDockAlt: Beginning docking sequence");

            yield return new BehaviorIntent
            {
                Position = fastDockIntent,
                Orientation = dockingOrientation,
                ExitWhen = () => fastDockIntent.IsConnected || !ctx.HasLeaderContact
            };

            // === PHASE 5: Handle Result ===
            if (fastDockIntent.IsConnected)
            {
                ctx.Debug?.Log("FastDockAlt: Connected!");
                ctx.ActiveConnector = droneConnector;
                ctx.SetDampeners(false);

                // Stay docked until disconnected
                yield return new BehaviorIntent
                {
                    Position = null,
                    Orientation = null,
                    ExitWhen = () => droneConnector.Status != MyShipConnectorStatus.Connected
                };

                // Disconnected
                ctx.SetDampeners(true);
                ctx.ActiveConnector = null;
                yield return BehaviorIntent.Complete();
                yield break;
            }

            // Lost leader contact
            yield return BehaviorIntent.Aborted(AbortReason.LostLeader);
        }

        /// <summary>
        /// Helper class for coordinate transformations.
        /// </summary>
        private class FastDockAltHelpers
        {
            private DroneContext _ctx;
            private DockingPadResponse _response;
            private IMyShipConnector _droneConnector;

            public FastDockAltHelpers(DroneContext ctx, DockingPadResponse response, IMyShipConnector droneConnector)
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

            public Vector3D GetTargetConnectorPosition()
            {
                if (!_ctx.HasLeaderContact) return Vector3D.Zero;
                MatrixD coordMatrix = GetCoordinateMatrix();
                return Vector3D.Transform(_response.ConnectorOffset, coordMatrix);
            }

            public Vector3D GetConnectorPosition()
            {
                Vector3D targetPos = GetTargetConnectorPosition();
                Vector3D referenceToConnectorOffset = GetReferenceToConnectorOffset();
                return targetPos - referenceToConnectorOffset;
            }

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
        }
    }
}
