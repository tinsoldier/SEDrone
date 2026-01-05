using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public class TacticalSnapshot
    {
        public readonly List<Vector3D> ProjectilePositions = new List<Vector3D>();
        public readonly List<MyDetectedEntityInfo> EnemyTargets = new List<MyDetectedEntityInfo>();

        public long DroneEntityId { get; set; }
        public long LeaderEntityId { get; set; }
        public double Timestamp { get; set; }
        public int ProjectileCount { get; set; }
        public bool HasPositionData { get; set; }
        public bool IsDroneBeingTargetedByProjectiles { get; set; }
        public bool IsLeaderBeingTargetedByProjectiles { get; set; }

        public void Clear()
        {
            ProjectilePositions.Clear();
            EnemyTargets.Clear();
            ProjectileCount = 0;
            HasPositionData = false;
            IsDroneBeingTargetedByProjectiles = false;
            IsLeaderBeingTargetedByProjectiles = false;
        }

        public void SetEnemyTargetsFromThreatMap(Dictionary<MyDetectedEntityInfo, float> threats)
        {
            EnemyTargets.Clear();
            if (threats == null)
                return;

            foreach (var pair in threats)
            {
                if (pair.Value > 0)
                {
                    EnemyTargets.Add(pair.Key);
                }
            }
        }
    }
}
