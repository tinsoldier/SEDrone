using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Message broadcast by the leader containing its current state.
    /// Drones use this to track and follow the leader.
    /// </summary>
    public struct LeaderStateMessage
    {
        // === Identity ===
        public long EntityId;           // Leader's grid entity ID
        public string GridName;         // Leader's grid name (for display/debug)

        // === Position & Motion ===
        public Vector3D Position;       // World position
        public Vector3D Velocity;       // World velocity
        public Vector3D Forward;        // Forward direction vector
        public Vector3D Up;             // Up direction vector

        // === Timestamp ===
        public double Timestamp;        // Game time when message was created

        /// <summary>
        /// Serializes the message to a string for IGC transmission.
        /// Format: EntityId|GridName|PosX|PosY|PosZ|VelX|VelY|VelZ|FwdX|FwdY|FwdZ|UpX|UpY|UpZ|Timestamp
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
            if (parts.Length < 15)
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
                message.Timestamp = double.Parse(parts[14]);
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
    /// For future use - formation commands, RTB, attack, etc.
    /// </summary>
    public enum FormationCommand
    {
        None,
        FormUp,         // Return to formation positions
        HoldPosition,   // Stop and hover
        ReturnToBase,   // RTB command
        FollowMe,       // Resume following
        Scatter,        // Break formation, evade
        Attack          // Engage targets (WeaponCore handles targeting)
    }
}
