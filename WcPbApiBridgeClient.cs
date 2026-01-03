using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Client for WcPbApiBridge mod - exposes WeaponCore projectile tracking to PB scripts.
    /// This bridges ModAPI methods that aren't available in the standard WC PB API.
    /// </summary>
    public class WcPbApiBridgeClient
    {
        private const string API_PROPERTY_NAME = "WcPbApiBridge";
        
        private Func<long, ICollection<Vector3D>, int> _getProjectilesLockedOnPos;
        private Action<ICollection<MyTuple<ulong, Vector3D, int, long>>> _getAllSmartProjectiles;
        private Func<bool> _isWcApiReady;
        
        /// <summary>
        /// Whether the API bridge was successfully activated.
        /// </summary>
        public bool IsReady { get; private set; }
        
        /// <summary>
        /// Whether WeaponCore's API is connected and ready.
        /// </summary>
        public bool IsWcApiReady => _isWcApiReady?.Invoke() ?? false;

        /// <summary>
        /// Attempts to activate the API bridge by reading delegates from the PB's properties.
        /// </summary>
        /// <param name="pb">The programmable block to read properties from</param>
        /// <returns>True if activation succeeded</returns>
        public bool Activate(IMyTerminalBlock pb)
        {
            var baseProp = pb.GetProperty(API_PROPERTY_NAME);
            if (baseProp == null) return false;
            var prop = baseProp.As<IReadOnlyDictionary<string, Delegate>>();
            if (prop == null) return false;
            var dict = prop.GetValue(pb);
            if (dict == null) return false;
            ApiAssign(dict);
            return IsReady = true;
        }

        private void ApiAssign(IReadOnlyDictionary<string, Delegate> dict)
        {
            Delegate d;
            if (dict.TryGetValue("GetProjectilesLockedOnPos", out d))
                _getProjectilesLockedOnPos = d as Func<long, ICollection<Vector3D>, int>;
            if (dict.TryGetValue("GetAllSmartProjectiles", out d))
                _getAllSmartProjectiles = d as Action<ICollection<MyTuple<ulong, Vector3D, int, long>>>;
            if (dict.TryGetValue("IsWcApiReady", out d))
                _isWcApiReady = d as Func<bool>;
        }

        /// <summary>
        /// Get positions of projectiles locked onto a grid.
        /// </summary>
        /// <param name="gridEntityId">Entity ID of the grid to check</param>
        /// <param name="positions">Collection to populate with projectile positions</param>
        /// <returns>Number of projectiles, or negative on error</returns>
        public int GetProjectilesLockedOnPos(long gridEntityId, ICollection<Vector3D> positions)
        {
            return _getProjectilesLockedOnPos?.Invoke(gridEntityId, positions) ?? -1;
        }

        /// <summary>
        /// Get all smart (guided) projectiles in the world.
        /// WARNING: This returns ALL smart projectiles server-wide. Use for testing only.
        /// </summary>
        /// <param name="collection">Collection to populate with projectile data.
        /// Each tuple contains: (ProjectileId, Position, AgeTicks, FactionId)</param>
        public void GetAllSmartProjectiles(ICollection<MyTuple<ulong, Vector3D, int, long>> collection)
        {
            _getAllSmartProjectiles?.Invoke(collection);
        }
    }
}
