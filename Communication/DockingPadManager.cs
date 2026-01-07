using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Manages docking pad assignments for the leader grid.
    /// Tracks which connectors are available and assigns them to requesting drones.
    /// </summary>
    public class DockingPadManager
    {
        private readonly IMyGridTerminalSystem _gts;
        private readonly IMyProgrammableBlock _me;
        private readonly IMyShipController _reference;
        private readonly Action<string> _echo;

        // Track connector assignments
        private Dictionary<long, DockingPadAssignment> _assignments = new Dictionary<long, DockingPadAssignment>();

        // Cached connector list (refreshed periodically)
        private List<IMyShipConnector> _availableConnectors = new List<IMyShipConnector>();
        private double _lastConnectorRefresh = 0;
        private const double CONNECTOR_REFRESH_INTERVAL = 5.0; // Refresh every 5 seconds

        private class DockingPadAssignment
        {
            public long DroneEntityId;
            public IMyShipConnector Connector;
            public double AssignedTime;
            public double LastHeartbeat;
        }

        public DockingPadManager(
            IMyGridTerminalSystem gts,
            IMyProgrammableBlock me,
            IMyShipController reference,
            Action<string> echo = null)
        {
            _gts = gts;
            _me = me;
            _reference = reference;
            _echo = echo;
        }

        /// <summary>
        /// Processes a docking pad request from a drone.
        /// </summary>
        /// <param name="request">The incoming request</param>
        /// <param name="currentTime">Current game time</param>
        /// <returns>Response to send back to the drone</returns>
        public DockingPadResponse ProcessRequest(DockingPadRequest request, double currentTime)
        {
            // Refresh connector list if needed
            if (currentTime - _lastConnectorRefresh > CONNECTOR_REFRESH_INTERVAL)
            {
                RefreshConnectorList();
                _lastConnectorRefresh = currentTime;
            }

            // Check if drone already has an assignment
            DockingPadAssignment existingAssignment;
            if (_assignments.TryGetValue(request.DroneEntityId, out existingAssignment))
            {
                // Update heartbeat
                existingAssignment.LastHeartbeat = currentTime;

                // Return existing assignment
                return CreateResponse(request, existingAssignment.Connector, currentTime);
            }

            // Find the closest available connector to the drone
            IMyShipConnector selectedConnector = SelectClosestAvailableConnector(request.DronePosition);

            if (selectedConnector == null)
            {
                _echo?.Invoke($"[DOCKING] No available connectors for drone {request.DroneGridName}");
                return new DockingPadResponse
                {
                    DroneEntityId = request.DroneEntityId,
                    RequestId = request.RequestId,
                    Available = false,
                    Timestamp = currentTime
                };
            }

            // Create new assignment
            _assignments[request.DroneEntityId] = new DockingPadAssignment
            {
                DroneEntityId = request.DroneEntityId,
                Connector = selectedConnector,
                AssignedTime = currentTime,
                LastHeartbeat = currentTime
            };

            _echo?.Invoke($"[DOCKING] Assigned pad {selectedConnector.CustomName} to {request.DroneGridName}");

            return CreateResponse(request, selectedConnector, currentTime);
        }

        /// <summary>
        /// Creates a docking pad response with connector information.
        /// </summary>
        private DockingPadResponse CreateResponse(
            DockingPadRequest request,
            IMyShipConnector connector,
            double currentTime)
        {
            if (_reference == null)
            {
                return new DockingPadResponse
                {
                    DroneEntityId = request.DroneEntityId,
                    RequestId = request.RequestId,
                    Available = false,
                    Timestamp = currentTime
                };
            }

            // Calculate connector position relative to grid center (matches LeaderStateMessage.Position)
            Vector3D connectorWorldPos = connector.GetPosition();
            Vector3D gridCenter = _reference.CubeGrid.WorldVolume.Center;
            Vector3D offsetWorld = connectorWorldPos - gridCenter;

            // Build coordinate matrix from forward/up to match LeaderStateMessage.WorldMatrix.
            // CRITICAL: Use the exact same formula on both encoding and decoding sides!
            MatrixD refMatrix = _reference.WorldMatrix;
            Vector3D leaderForward = refMatrix.Forward;
            Vector3D leaderUp = refMatrix.Up;

            MatrixD coordinateMatrix = MatrixD.CreateWorld(
                gridCenter,
                leaderForward,
                leaderUp
            );

            // Transform using world->local with matrix transpose
            Vector3D localOffset = Vector3D.TransformNormal(offsetWorld, MatrixD.Transpose(coordinateMatrix));

            // DEBUG: Output connector offset in leader-local coordinates
            string debugMsg = string.Format("[DOCK] Connector: {0}\nLocal Offset: X={1:F2} Y={2:F2} Z={3:F2}\nWorld Offset: {4:F2},{5:F2},{6:F2}",
                connector.CustomName,
                localOffset.X, localOffset.Y, localOffset.Z,
                offsetWorld.X, offsetWorld.Y, offsetWorld.Z);
            _echo?.Invoke(debugMsg);

            // Also log to a text panel or LCD named "Debug" if available
            var debugBlock = _gts.GetBlockWithName("Debug") as IMyTextPanel;
            if (debugBlock != null)
            {
                debugBlock.WriteText(debugMsg + "\n", true);
            }

            // Transform connector directions to leader-local
            Vector3D connectorForwardWorld = connector.WorldMatrix.Forward;
            Vector3D connectorUpWorld = connector.WorldMatrix.Up;

            Vector3D connectorForwardLocal = Vector3D.TransformNormal(connectorForwardWorld, MatrixD.Transpose(coordinateMatrix));
            Vector3D connectorUpLocal = Vector3D.TransformNormal(connectorUpWorld, MatrixD.Transpose(coordinateMatrix));

            return new DockingPadResponse
            {
                DroneEntityId = request.DroneEntityId,
                RequestId = request.RequestId,
                Available = true,
                ConnectorOffset = localOffset,
                ConnectorForward = connectorForwardLocal,
                ConnectorUp = connectorUpLocal,
                ConnectorSize = GetConnectorRadius(connector),
                WaypointsData = "", // Optional: could generate waypoints here
                Timestamp = currentTime
            };
        }

        /// <summary>
        /// Selects the closest available connector to the drone's position.
        /// </summary>
        private IMyShipConnector SelectClosestAvailableConnector(Vector3D dronePosition)
        {
            IMyShipConnector closest = null;
            double closestDistSq = double.MaxValue;

            foreach (var connector in _availableConnectors)
            {
                // Skip if not functional
                if (!connector.IsFunctional || !connector.Enabled)
                    continue;

                // Skip if already connected
                if (connector.Status == MyShipConnectorStatus.Connected)
                    continue;

                // Skip if already assigned to another drone
                bool isAssigned = false;
                foreach (var assignment in _assignments.Values)
                {
                    if (assignment.Connector.EntityId == connector.EntityId)
                    {
                        isAssigned = true;
                        break;
                    }
                }

                if (isAssigned)
                    continue;

                // Check distance to drone
                double distSq = Vector3D.DistanceSquared(connector.GetPosition(), dronePosition);
                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closest = connector;
                }
            }

            return closest;
        }

        /// <summary>
        /// Refreshes the list of available connectors on the leader grid.
        /// </summary>
        private void RefreshConnectorList()
        {
            _availableConnectors.Clear();
            var allConnectors = new List<IMyShipConnector>();
            _gts.GetBlocksOfType(allConnectors, c => c.CubeGrid.EntityId == _me.CubeGrid.EntityId);

            // Filter connectors based on name tags or other criteria
            foreach (var connector in allConnectors)
            {
                // You could add filtering logic here, e.g.:
                // - Connectors with "[dock]" in the name
                // - Connectors facing outward
                // - Small vs large connectors

                // For now, accept all functional connectors on our grid
                if (connector.IsFunctional)
                {
                    _availableConnectors.Add(connector);
                }
            }

            _echo?.Invoke($"[DOCKING] Found {_availableConnectors.Count} available connectors");
        }

        public void RefreshConnectors()
        {
            RefreshConnectorList();
            _lastConnectorRefresh = 0;
        }

        /// <summary>
        /// Cleans up stale assignments (drones that stopped requesting or docked successfully).
        /// Call this periodically.
        /// </summary>
        /// <param name="currentTime">Current game time</param>
        /// <param name="timeoutSeconds">How long before considering an assignment stale</param>
        public void CleanupStaleAssignments(double currentTime, double timeoutSeconds = 30.0)
        {
            var toRemove = new List<long>();

            foreach (var kvp in _assignments)
            {
                var assignment = kvp.Value;

                // Check if connector is now connected (successful dock)
                if (assignment.Connector.Status == MyShipConnectorStatus.Connected)
                {
                    _echo?.Invoke($"[DOCKING] Drone {kvp.Key} successfully docked, clearing assignment");
                    toRemove.Add(kvp.Key);
                    continue;
                }

                // Check for timeout (no heartbeat)
                if (currentTime - assignment.LastHeartbeat > timeoutSeconds)
                {
                    _echo?.Invoke($"[DOCKING] Assignment for drone {kvp.Key} timed out, clearing");
                    toRemove.Add(kvp.Key);
                    continue;
                }
            }

            foreach (var droneId in toRemove)
            {
                _assignments.Remove(droneId);
            }
        }

        /// <summary>
        /// Gets the connector radius based on grid size.
        /// </summary>
        private static double GetConnectorRadius(IMyShipConnector connector)
        {
            if (connector.CubeGrid.GridSize == 0.5) // Small grid
                return connector.CubeGrid.GridSize; // 0.5m
            return connector.CubeGrid.GridSize * 0.5; // 1.25m for large
        }

        /// <summary>
        /// Gets the number of currently assigned pads.
        /// </summary>
        public int AssignedPadCount
        {
            get { return _assignments.Count; }
        }

        /// <summary>
        /// Gets the number of available (unassigned) pads.
        /// </summary>
        public int AvailablePadCount
        {
            get
            {
                int count = 0;
                foreach (var connector in _availableConnectors)
                {
                    if (!connector.IsFunctional || !connector.Enabled)
                        continue;

                    if (connector.Status == MyShipConnectorStatus.Connected)
                        continue;

                    bool isAssigned = false;
                    foreach (var assignment in _assignments.Values)
                    {
                        if (assignment.Connector.EntityId == connector.EntityId)
                        {
                            isAssigned = true;
                            break;
                        }
                    }

                    if (!isAssigned)
                        count++;
                }
                return count;
            }
        }
    }
}
