using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Request sent from drone to leader asking for a docking pad assignment.
    /// </summary>
    public struct DockingPadRequest
    {
        public long DroneEntityId;      // Requesting drone's entity ID
        public string DroneGridName;    // Drone's grid name
        public long RequestId;          // Unique request ID for tracking response
        public double Timestamp;        // When request was sent

        public string Serialize()
        {
            return string.Join("|",
                "DOCK_REQUEST",
                DroneEntityId,
                DroneGridName,
                RequestId,
                Timestamp
            );
        }

        public static bool TryParse(string data, out DockingPadRequest request)
        {
            request = new DockingPadRequest();

            if (string.IsNullOrEmpty(data))
                return false;

            string[] parts = data.Split('|');
            if (parts.Length < 5 || parts[0] != "DOCK_REQUEST")
                return false;

            try
            {
                request.DroneEntityId = long.Parse(parts[1]);
                request.DroneGridName = parts[2];
                request.RequestId = long.Parse(parts[3]);
                request.Timestamp = double.Parse(parts[4]);
                return true;
            }
            catch
            {
                return false;
            }
        }
    }

    /// <summary>
    /// Response from leader assigning a docking pad to the drone.
    /// Contains connector offset and optional waypoint sequence.
    /// </summary>
    public struct DockingPadResponse
    {
        public long DroneEntityId;              // Target drone's entity ID
        public long RequestId;                  // Matches the request ID
        public bool Available;                  // Whether a pad is available

        // Connector information (in leader-local space)
        public Vector3D ConnectorOffset;        // Offset from leader position to connector
        public Vector3D ConnectorForward;       // Forward direction of connector (local)
        public Vector3D ConnectorUp;            // Up direction of connector (local)
        public double ConnectorSize;            // Radius of connector for clearance

        // Optional waypoint sequence (offsets relative to connector in connector-local space)
        public string WaypointsData;            // Serialized waypoints (if any)

        public double Timestamp;

        public string Serialize()
        {
            return string.Join("|",
                "DOCK_RESPONSE",
                DroneEntityId,
                RequestId,
                Available ? "1" : "0",
                ConnectorOffset.X, ConnectorOffset.Y, ConnectorOffset.Z,
                ConnectorForward.X, ConnectorForward.Y, ConnectorForward.Z,
                ConnectorUp.X, ConnectorUp.Y, ConnectorUp.Z,
                ConnectorSize,
                WaypointsData ?? "",
                Timestamp
            );
        }

        public static bool TryParse(string data, out DockingPadResponse response)
        {
            response = new DockingPadResponse();

            if (string.IsNullOrEmpty(data))
                return false;

            string[] parts = data.Split('|');
            if (parts.Length < 16 || parts[0] != "DOCK_RESPONSE")
                return false;

            try
            {
                response.DroneEntityId = long.Parse(parts[1]);
                response.RequestId = long.Parse(parts[2]);
                response.Available = parts[3] == "1";
                response.ConnectorOffset = new Vector3D(
                    double.Parse(parts[4]),
                    double.Parse(parts[5]),
                    double.Parse(parts[6])
                );
                response.ConnectorForward = new Vector3D(
                    double.Parse(parts[7]),
                    double.Parse(parts[8]),
                    double.Parse(parts[9])
                );
                response.ConnectorUp = new Vector3D(
                    double.Parse(parts[10]),
                    double.Parse(parts[11]),
                    double.Parse(parts[12])
                );
                response.ConnectorSize = double.Parse(parts[13]);
                response.WaypointsData = parts[14];
                response.Timestamp = double.Parse(parts[15]);
                return true;
            }
            catch
            {
                return false;
            }
        }
    }
}
