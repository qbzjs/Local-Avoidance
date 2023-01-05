using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace ProjectDawn.LocalAvoidance
{
    public class AgentDraw : MonoBehaviour
    {
        public float InnerRadius = 1;
        public float OuterRadius = 5;
        public float3 Velocity;
        public Transform Obstacle;
        public float3 ObstacleVelocity;
        public float ObstacleRadius = 1;

        void OnDrawGizmos()
        {
            using (var sonar = new SonarAvoidance(transform.position, quaternion.identity, InnerRadius, OuterRadius, math.length(Velocity), Allocator.Temp))
            {
                sonar.InsertObstacle(new float3(-1, 0, 0), math.radians(180));
                if (Obstacle)
                {
                    sonar.InsertObstacle(Obstacle.transform.position, ObstacleVelocity, ObstacleRadius);
                    sonar.DrawObstacle(Obstacle.transform.position, ObstacleVelocity, ObstacleRadius);
                }
                sonar.DrawSonar();
                sonar.DrawClosestDirection();
            }
        }
    }
}
