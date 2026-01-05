using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public class TacticalCoordinator
    {
        private readonly IMyTerminalBlock _pb;
        private Program.WcPbApi _wcApi;
        private readonly WcPbApiBridgeClient _wcBridge;
        private readonly List<Vector3D> _tempPositions = new List<Vector3D>();
        private readonly Dictionary<MyDetectedEntityInfo, float> _tempEnemyTargets = new Dictionary<MyDetectedEntityInfo, float>();
        private bool _projectileTrackingEnabled;

        public bool HasWeaponCore => _wcApi != null;

        public TacticalCoordinator(IMyTerminalBlock pb, Action<string> echo)
        {
            _pb = pb;

            _wcApi = new Program.WcPbApi();
            try
            {
                if (_wcApi.Activate(pb))
                {
                    echo?.Invoke("[Tactical] WcPbApi connected");
                }
                else
                {
                    _wcApi = null;
                    echo?.Invoke("[Tactical] WcPbApi unavailable");
                }
            }
            catch
            {
                _wcApi = null;
            }

            _wcBridge = new WcPbApiBridgeClient();
            try
            {
                if (_wcBridge.Activate(pb))
                {
                    _projectileTrackingEnabled = true;
                    echo?.Invoke("[Tactical] WcPbApiBridge connected");
                }
            }
            catch
            {
                _projectileTrackingEnabled = false;
            }
        }

        public void UpdateSnapshot(TacticalSnapshot snapshot, long droneEntityId, long leaderEntityId, double gameTime)
        {
            if (snapshot == null)
                return;

            if (snapshot.Timestamp == gameTime
                && snapshot.DroneEntityId == droneEntityId
                && snapshot.LeaderEntityId == leaderEntityId)
            {
                return;
            }

            snapshot.Clear();
            snapshot.Timestamp = gameTime;
            snapshot.DroneEntityId = droneEntityId;
            snapshot.LeaderEntityId = leaderEntityId;

            if (_projectileTrackingEnabled && _wcBridge != null && _wcBridge.IsReady && _wcBridge.IsWcApiReady)
            {
                UpdateThreatsWithPositions(snapshot, droneEntityId, leaderEntityId);
                snapshot.HasPositionData = true;
            }
            else if (_wcApi != null)
            {
                snapshot.ProjectileCount = GetThreatsCountOnly(droneEntityId, leaderEntityId);
                snapshot.HasPositionData = false;
            }

            if (_wcApi != null && _pb != null)
            {
                _tempEnemyTargets.Clear();
                // _wcApi.GetSortedThreats(_pb, _tempEnemyTargets);
                snapshot.SetEnemyTargetsFromThreatMap(_tempEnemyTargets);
            }
        }

        private void UpdateThreatsWithPositions(TacticalSnapshot snapshot, long droneEntityId, long leaderEntityId)
        {
            _tempPositions.Clear();

            snapshot.IsDroneBeingTargetedByProjectiles = false;
            int count = _wcBridge.GetProjectilesLockedOnPos(droneEntityId, _tempPositions);
            if (count > 0)
            {
                snapshot.ProjectilePositions.AddRange(_tempPositions);
                snapshot.IsDroneBeingTargetedByProjectiles = true;
            }

            snapshot.IsLeaderBeingTargetedByProjectiles = false;
            if (leaderEntityId != 0)
            {
                _tempPositions.Clear();
                int leaderCount = _wcBridge.GetProjectilesLockedOnPos(leaderEntityId, _tempPositions);
                if (leaderCount > 0)
                {
                    snapshot.IsLeaderBeingTargetedByProjectiles = true;

                    foreach (var pos in _tempPositions)
                    {
                        bool isDuplicate = false;
                        for (int i = 0; i < snapshot.ProjectilePositions.Count; i++)
                        {
                            if (Vector3D.DistanceSquared(pos, snapshot.ProjectilePositions[i]) < 1.0)
                            {
                                isDuplicate = true;
                                break;
                            }
                        }

                        if (!isDuplicate)
                        {
                            snapshot.ProjectilePositions.Add(pos);
                        }
                    }
                }
            }

            snapshot.ProjectileCount = snapshot.ProjectilePositions.Count;
        }

        private int GetThreatsCountOnly(long droneEntityId, long leaderEntityId)
        {
            int totalCount = 0;

            var droneResult = _wcApi.GetProjectilesLockedOn(droneEntityId);
            if (droneResult.Item1)
            {
                totalCount += droneResult.Item2;
            }

            if (leaderEntityId != 0)
            {
                var leaderResult = _wcApi.GetProjectilesLockedOn(leaderEntityId);
                if (leaderResult.Item1)
                {
                    totalCount += leaderResult.Item2;
                }
            }

            return totalCount;
        }
    }
}
