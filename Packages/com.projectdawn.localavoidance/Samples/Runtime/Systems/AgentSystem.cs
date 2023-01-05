using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// System that executs local avoidance logic for <see cref="Agent"/>.
    /// </summary>
    [DefaultExecutionOrder(50)]
    public class AgentSystem : System
    {
        [Range(1, 10)]
        public float SonarRadius = 6;
        [Range(0, 1)]
        public float SonarForwardVelocityScaler = 1;
        [Range(0, 1)]
        public float SonarBackVelocityScaler = 1;
        [Range(0, 180)]
        public float SonarCutBackVisionAngle = 135f;
        public bool ChangeTransform = true;
        public bool TightFormation = false;
        public bool Is3D = true;
        public float AgentAcceleration = 8;

        void FixedUpdate()
        {
            var agents = Query<Agent>();
            if (agents.Length == 0)
                return;

            foreach (var agent in agents)
            {
                float3 impulse = 0;
                if (!agent.IsStopped && agent.Speed > 0)
                {
                    float3 desiredDirection = math.normalizesafe(agent.Destination - agent.Position);
                    if (agent.Avoid)
                        impulse += GetAvoid(agent, agents, desiredDirection);
                    else
                        impulse += desiredDirection * agent.Speed;

                    var velocity = math.lerp(agent.Velocity, impulse, Time.deltaTime * AgentAcceleration);
                    agent.Velocity = velocity;
                }
                else
                {
                    agent.Velocity = 0;
                }

                if (ChangeTransform && math.lengthsq(agent.Velocity) != 0)
                {
                    var offset = agent.Velocity * Time.deltaTime;

                    // Avoid over-steping destination
                    var distance = math.distance(agent.Destination, agent.Position);
                    offset = ForceLength(offset, distance);

                    agent.transform.position += (Vector3)offset;

                    if (Is3D)
                    {
                        var direction = math.normalizesafe(agent.Velocity);
                        float angle = math.atan2(direction.z, -direction.x);
                        var rotation = quaternion.RotateY(angle);
                        rotation = quaternion.LookRotation(math.normalizesafe(new float3(agent.Velocity.x, 0, agent.Velocity.z)), new float3(0, 1, 0));
                        agent.transform.rotation = math.slerp(transform.rotation, rotation, math.saturate(Time.deltaTime * agent.TurnSpeed));
                    }
                }
            }
        }

        float3 GetAvoid(Agent agent, Agent[] nearbyAgents, float3 desiredDirection)
        {
            if (math.lengthsq(desiredDirection) == 0)
                return float3.zero;

            // Destination should not be farther than the vision
            var sonarRadius = math.min(SonarRadius, math.distance(agent.Destination, agent.Position));

            var up = Is3D ? new float3(0, 1, 0) : new float3(0, 0, -1);
            var sonar = new SonarAvoidance(agent.Position, desiredDirection, up, agent.Radius, sonarRadius, math.length(agent.Velocity));

            var agentCircle = new Circle(agent.Position.xz + math.length(agent.Velocity) * Time.deltaTime * desiredDirection.xz, agent.Radius);
            bool desiredDirectionOccluded = false;

            foreach (var nearbyAgent in nearbyAgents)
            {
                // Skip itself
                if (nearbyAgent == agent)
                    continue;

                sonar.InsertObstacle(nearbyAgent.Position, nearbyAgent.Velocity, nearbyAgent.Radius);

                // Check if taking a step to desired direction would collide with target agent
                if (TightFormation && !desiredDirectionOccluded && Circle.Collide(agentCircle, new Circle(nearbyAgent.Position.xz, nearbyAgent.Radius)))
                {
                    desiredDirectionOccluded = true;
                }
            }

            // Add blocker behind the velocity
            // This will prevent situations where agent has on right and left equally good paths
            sonar.InsertObstacle(math.normalizesafe(-agent.Velocity), math.radians(SonarCutBackVisionAngle));

            bool success = sonar.FindClosestDirection(out float3 newDirection);

            // If all directions obstructed, try to at least move agent as close as possible to destination without colliding
            // This is not necessary step, but it yields much better formation movement
            if (TightFormation && !success && !desiredDirectionOccluded)
            {
                newDirection = new float3(desiredDirection.x, 0, desiredDirection.y);
            }

            sonar.Dispose();

            return newDirection * agent.Speed;
        }

        float3 ForceLength(float3 value, float length)
        {
            var valueLength = math.length(value);
            return valueLength > length ? value / valueLength * length : value;
        }
    }
}
