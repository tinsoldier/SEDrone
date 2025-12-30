using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Message broadcast by the leader containing its current state.
    /// Drones use this to track and follow the leader.
    /// Implements IOrientedReference for use in formation behaviors.
    /// </summary>
    public struct LeaderStateMessage : IOrientedReference
    {
        // === Identity ===
        public long EntityId;           // Leader's grid entity ID
        public string GridName;         // Leader's grid name (for display/debug)

        // === Position & Motion (IOrientedReference properties) ===
        public Vector3D Position { get; set; }       // World position
        public Vector3D Velocity { get; set; }       // World velocity
        public Vector3D Forward { get; set; }        // Forward direction vector
        public Vector3D Up { get; set; }             // Up direction vector

        // === Additional orientation data ===
        public Vector3D Left;           // Left direction vector (needed for SEAD2-compatible transforms)

        // === Timestamp ===
        public double Timestamp;        // Game time when message was created

        /// <summary>
        /// Serializes the message to a string for IGC transmission.
        /// Format: EntityId|GridName|PosX|PosY|PosZ|VelX|VelY|VelZ|FwdX|FwdY|FwdZ|UpX|UpY|UpZ|LeftX|LeftY|LeftZ|Timestamp
        /// </summary>
        public string Serialize()
        {
            return string.Join("|",
                EntityId,
                GridName,
                Position.X, Position.Y, Position.Z,
                Velocity.X, Velocity.Y, Velocity.Z,
                Forward.X, Forward.Y, Forward.Z,
                Up.X, Up.Y, Up.Z,
                Left.X, Left.Y, Left.Z,
                Timestamp
            );
        }

        /// <summary>
        /// Deserializes a message from an IGC string.
        /// </summary>
        /// <param name="data">The serialized message string</param>
        /// <param name="message">The parsed message if successful</param>
        /// <returns>True if parsing succeeded</returns>
        public static bool TryParse(string data, out LeaderStateMessage message)
        {
            message = new LeaderStateMessage();

            if (string.IsNullOrEmpty(data))
                return false;

            string[] parts = data.Split('|');
            if (parts.Length < 18)
                return false;

            try
            {
                message.EntityId = long.Parse(parts[0]);
                message.GridName = parts[1];
                message.Position = new Vector3D(
                    double.Parse(parts[2]),
                    double.Parse(parts[3]),
                    double.Parse(parts[4])
                );
                message.Velocity = new Vector3D(
                    double.Parse(parts[5]),
                    double.Parse(parts[6]),
                    double.Parse(parts[7])
                );
                message.Forward = new Vector3D(
                    double.Parse(parts[8]),
                    double.Parse(parts[9]),
                    double.Parse(parts[10])
                );
                message.Up = new Vector3D(
                    double.Parse(parts[11]),
                    double.Parse(parts[12]),
                    double.Parse(parts[13])
                );
                message.Left = new Vector3D(
                    double.Parse(parts[14]),
                    double.Parse(parts[15]),
                    double.Parse(parts[16])
                );
                message.Timestamp = double.Parse(parts[17]);
                return true;
            }
            catch
            {
                return false;
            }
        }
    }

    /// <summary>
    /// Command messages that can be sent from leader to drones.
    /// </summary>
    public enum DroneCommand
    {
        None,
        Dock,           // Initiate docking sequence
        FastDock,       // Initiate fast docking sequence
        Escort,         // Return to escort/formation mode
        FormUp,         // Return to formation positions
        HoldPosition,   // Stop and hover
        ReturnToBase,   // RTB command
        FollowMe,       // Resume following
        Scatter,        // Break formation, evade
        Attack          // Engage targets (WeaponCore handles targeting)
    }

    /// <summary>
    /// Command message sent from leader to specific drone or all drones.
    /// </summary>
    public struct DroneCommandMessage
    {
        public long TargetDroneId;      // 0 = broadcast to all drones
        public DroneCommand Command;
        public double Timestamp;

        public string Serialize()
        {
            return string.Join("|",
                "DRONE_COMMAND",
                TargetDroneId,
                (int)Command,
                Timestamp
            );
        }

        public static bool TryParse(string data, out DroneCommandMessage message)
        {
            message = new DroneCommandMessage();

            if (string.IsNullOrEmpty(data))
                return false;

            string[] parts = data.Split('|');
            if (parts.Length < 4 || parts[0] != "DRONE_COMMAND")
                return false;

            try
            {
                message.TargetDroneId = long.Parse(parts[1]);
                message.Command = (DroneCommand)int.Parse(parts[2]);
                message.Timestamp = double.Parse(parts[3]);
                return true;
            }
            catch
            {
                return false;
            }
        }
    }
}
