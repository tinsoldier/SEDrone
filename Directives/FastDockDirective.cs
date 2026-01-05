using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Alternative docking directive using a single stateless FastDock intent.
    ///
    /// Unlike the original FastDock directive (which yields new Move intents every tick),
    /// this directive yields a single FastDock intent that runs continuously until
    /// docking is complete. This allows stable velocity control without PID reset issues.
    ///
    /// The FastDock intent handles all geometry calculations and thruster control
    /// internally, similar to Spug's SEAD2 AutoLandToConnector approach.
    /// </summary>
    public class FastDockDirective : IDirective
    {
        public string Name { get { return "FastDock"; } }

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            ctx.Debug?.Log("FastDock: Starting");

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

            ctx.Debug?.Log("FastDock: Got docking pad response");

            // === PHASE 2: Connector Selection ===
            var helpers = new FastDockHelpers(ctx, padResponse, null);
            IMyShipConnector droneConnector;
            if (ctx.Hardware != null && ctx.Hardware.Connectors.Count > 0)
            {
                droneConnector = ctx.DockingNav.SelectDroneConnector(
                    ctx.Hardware.Connectors,
                    helpers.GetTargetConnectorForward()
                );
            }
            else
            {
                droneConnector = ctx.DockingNav.SelectDroneConnector(
                    ctx.GridTerminalSystem,
                    ctx.GridId,
                    helpers.GetTargetConnectorForward()
                );
            }

            if (droneConnector == null)
            {
                yield return BehaviorIntent.Aborted(AbortReason.Damaged,
                    "No suitable connector found on drone");
                yield break;
            }

            // Recreate helpers with drone connector
            helpers = new FastDockHelpers(ctx, padResponse, droneConnector);

            double droneConnectorSize = DockingNavigator.GetConnectorRadius(droneConnector);
            double targetConnectorSize = padResponse.ConnectorSize;

            ctx.Debug?.Log($"FastDock: Connectors selected (drone={droneConnectorSize:F1}m, target={targetConnectorSize:F1}m)");

            // === PHASE 3: Create FastDock Intent ===
            // This single intent handles the entire docking sequence
            var fastDockIntent = new FastDock(
                droneConnector,
                () => helpers.GetConnectorReference(),
                droneConnectorSize,
                targetConnectorSize
            );

            // Orientation: align connector to target
            var dockingOrientation = new AlignBlock(
                droneConnector,
                () => -helpers.GetTargetConnectorForward(),
                () => helpers.GetTargetConnectorUp()
            );

            // === PHASE 4: Execute Docking ===
            // Yield single intent that runs until connected or lost leader
            ctx.Debug?.Log("FastDock: Beginning docking sequence");

            yield return new BehaviorIntent
            {
                Position = fastDockIntent,
                Orientation = dockingOrientation,
                ExitWhen = () => fastDockIntent.IsConnected || !ctx.HasLeaderContact
            };

            // === PHASE 5: Handle Result ===
            if (fastDockIntent.IsConnected)
            {
                ctx.Debug?.Log("FastDock: Connected!");
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
        private class FastDockHelpers
        {
            private DroneContext _ctx;
            private DockingPadResponse _response;
            private IMyShipConnector _droneConnector;
            private int _diagCounter;
            private Vector3D _lastForward;
            private Vector3D _lastUp;

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
                return _ctx.LastLeaderState.WorldMatrix;
            }

            public Vector3D GetTargetConnectorForward()
            {
                UpdateTargetBasis();
                return _lastForward;
            }

            public Vector3D GetTargetConnectorUp()
            {
                UpdateTargetBasis();
                return _lastUp;
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

            private void UpdateTargetBasis()
            {
                if (!_ctx.HasLeaderContact)
                {
                    _lastForward = Vector3D.Zero;
                    _lastUp = Vector3D.Zero;
                    return;
                }

                MatrixD coordMatrix = GetCoordinateMatrix();
                _lastForward = Vector3D.TransformNormal(_response.ConnectorForward, coordMatrix);
                _lastUp = Vector3D.TransformNormal(_response.ConnectorUp, coordMatrix);

                _diagCounter++;
                if (_diagCounter >= 30)
                {
                    _diagCounter = 0;
                    LogOrientationDiagnostics(_lastForward, _lastUp);
                }
            }

            private void LogOrientationDiagnostics(Vector3D forward, Vector3D up)
            {
                double forwardLen = forward.Length();
                double upLen = up.Length();

                double dot = 0;
                if (forwardLen > 0.001 && upLen > 0.001)
                {
                    dot = Vector3D.Dot(forward / forwardLen, up / upLen);
                }

                if (!IsValid(forward) || !IsValid(up) || forwardLen < 0.5 || upLen < 0.5 || Math.Abs(dot) > 0.25)
                {
                    _ctx.Debug?.Log(
                        $"FastDock: basis fLen={forwardLen:F2} uLen={upLen:F2} dot={dot:F2} leader={_ctx.HasLeaderContact}");
                }
            }

            private static bool IsValid(Vector3D v)
            {
                return !double.IsNaN(v.X) && !double.IsInfinity(v.X)
                    && !double.IsNaN(v.Y) && !double.IsInfinity(v.Y)
                    && !double.IsNaN(v.Z) && !double.IsInfinity(v.Z);
            }
        }
    }
}
