using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Provides tactical awareness to directives and executors.
    /// Updated each tick by the Brain before directive execution.
    /// 
    /// This is a simple wrapper around threat data. More sophisticated
    /// threat analysis can be added later without changing directive code.
    /// </summary>
    public class TacticalContext
    {
        private List<Vector3D> _projectilePositions = new List<Vector3D>();
        private List<MyDetectedEntityInfo> _enemyTargets = new List<MyDetectedEntityInfo>();
        private long _lastWeaponTargetId;
        private readonly Program.WcPbApi _wcApi;


        /// <summary>
        /// Number of detected missiles/projectile threats.
        /// </summary>
        public int ProjectileCount { get; private set; }
        public int EnemyCount => _enemyTargets.Count;

        /// <summary>
        /// Whether any threats are currently detected.
        /// </summary>
        public bool HasThreats => ProjectileCount > 0 || EnemyCount > 0;

        /// <summary>
        /// Whether position data is available for threats.
        /// False means we only have a count (will aim nose-up).
        /// </summary>
        public bool HasPositionData { get; private set; }

        /// <summary>
        /// Whether the drone is currently being targeted by projectiles.
        /// </summary>
        public bool IsDroneBeingTargetedByProjectiles { get; internal set; }
        /// <summary> 
        /// Whether the leader is currently being targeted by projectiles.
        /// </summary>
        public bool IsLeaderBeingTargetedByProjectiles { get; internal set; }
        public Action<string> Echo { get; }


        public TacticalContext(Program.WcPbApi wcApi, Action<string> echo)
        {
            _wcApi = wcApi;
            Echo = echo;
        }

        /// <summary>
        /// Updates threat data with positions.
        /// </summary>
        public void UpdateThreats(List<Vector3D> positions)
        {
            _projectilePositions.Clear();
            if (positions != null)
            {
                _projectilePositions.AddRange(positions);
            }
            ProjectileCount = _projectilePositions.Count;
            HasPositionData = true;
        }

        /// <summary>
        /// Updates threat data with count only (no positions).
        /// </summary>
        public void UpdateThreatCount(int count)
        {
            _projectilePositions.Clear();
            ProjectileCount = count;
            HasPositionData = false;
        }

        /// <summary>
        /// Clears all threat data.
        /// </summary>
        public void ClearProjectileThreats()
        {
            _projectilePositions.Clear();
            ProjectileCount = 0;
            HasPositionData = false;
        }

        /// <summary>
        /// Gets the position of the closest threat to the given position.
        /// Returns null if no threats with position data.
        /// </summary>
        public Vector3D? GetClosestThreatPosition(Vector3D fromPosition)
        {
            if (_projectilePositions.Count == 0 && _enemyTargets.Count == 0)
                return null;

            Vector3D closest = Vector3D.Zero;
            double closestDistSq = double.MaxValue;

            if (_projectilePositions.Count > 0)
            {
                closest = _projectilePositions[0];
                closestDistSq = Vector3D.DistanceSquared(fromPosition, closest);
            }

            //iterate over projectiles and find closest
            for (int i = 0; i < _projectilePositions.Count; i++)
            {
                double distSq = Vector3D.DistanceSquared(fromPosition, _projectilePositions[i]);
                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closest = _projectilePositions[i];
                }
            }

            //iterate over enemy targets and find closest
            for (int i = 0; i < _enemyTargets.Count; i++)   
            {
                Vector3D enemyPos = _enemyTargets[i].Position;
                double distSq = Vector3D.DistanceSquared(fromPosition, enemyPos);
                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closest = enemyPos;
                }
            }

            return closestDistSq < double.MaxValue ? (Vector3D?)closest : null;
        }

        /// <summary>
        /// Gets all threat positions (for advanced strategies).
        /// Returns empty list if no position data.
        /// </summary>
        public IReadOnlyList<Vector3D> GetAllThreatPositions()
        {
            return _projectilePositions;
        }

        public void UpdateEnemyTargets(Dictionary<MyDetectedEntityInfo, float> enemyTargets)
        {
            _enemyTargets.Clear();
            if (enemyTargets != null)
            {
                foreach (var target in enemyTargets.Keys)
                {
                    _enemyTargets.Add(target);
                }
            }
        }

        public ITargetTelemetry GetTargetTelemetry(long entityId)
        {
            if (entityId == 0)
                return null;

            for (int i = 0; i < _enemyTargets.Count; i++)
            {
                if (_enemyTargets[i].EntityId == entityId)
                {
                    return new TargetTelemetry(_enemyTargets[i]);
                }
            }

            return null;
        }

        public ITargetTelemetry GetTargetTelemetry(long entityId, IMyTerminalBlock weaponBlock)
        {
            Echo?.Invoke($"(Tactical) GetTargetTelemetry for entityId={entityId}");

            if (entityId == 0)
                return null;

            if (_wcApi != null && weaponBlock != null)
            {
                if (_lastWeaponTargetId != entityId)
                {
                    //Echo?.Invoke($"(Tactical) Setting weapon focus to entityId={entityId}");
                    weaponBlock.SetValue("WC_FocusFire", true);
                    _wcApi.SetAiFocus(weaponBlock, entityId, 0);
                    _lastWeaponTargetId = entityId;
                }

                var weaponTarget = _wcApi.GetAiFocus(weaponBlock.CubeGrid.EntityId, 0);
                if (weaponTarget.HasValue && weaponTarget.Value.EntityId != 0)
                {
                    //check if target has a valid position and velocity
                    if(weaponTarget.Value.Position == Vector3D.Zero)
                    {
                        ////Echo?.Invoke($"(Tactical) target position invalid");
                        return null;
                    }
                    //Echo?.Invoke($"(Tactical) Found valid target");
                    return new TargetTelemetry(weaponTarget.Value);
                }
                else
                {
                    //Echo?.Invoke($"(Tactical) target not found");
                }
            }
            else
            {
                //if(weaponBlock == null)
                    //Echo?.Invoke($"(Tactical) Weapon block is null");
                //if(_wcApi == null)
                    //Echo?.Invoke($"(Tactical) WC API is null");
            }

            return GetTargetTelemetry(entityId);
        }

        public ITargetTelemetry GetPredictedTargetTelemetry(long entityId, Program.WcPbApi wcApi, IMyTerminalBlock weaponBlock)
        {
            if (entityId == 0)
                return null;

            for (int i = 0; i < _enemyTargets.Count; i++)
            {
                if (_enemyTargets[i].EntityId == entityId)
                    return new PredictedTargetTelemetry(_enemyTargets[i], wcApi, weaponBlock);
            }

            return null;
        }

        public ITargetTelemetry GetClosestEnemyTelemetry(Vector3D fromPosition)
        {
            if (_enemyTargets.Count == 0)
                return null;

            MyDetectedEntityInfo closest = _enemyTargets[0];
            double closestDistSq = Vector3D.DistanceSquared(fromPosition, closest.Position);

            for (int i = 1; i < _enemyTargets.Count; i++)
            {
                double distSq = Vector3D.DistanceSquared(fromPosition, _enemyTargets[i].Position);
                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closest = _enemyTargets[i];
                }
            }

            return new TargetTelemetry(closest);
        }
    }
}
