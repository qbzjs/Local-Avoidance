using UnityEngine;
using UnityEngine.Jobs;
using UnityEngine.Profiling;
using System.Collections.Generic;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using Unity.Profiling;

namespace ProjectDawn.LocalAvoidance.Demo
{
    [BurstCompile]
    public struct BoidsJob : IJobParallelFor
    {
        public float SeperationWeight;
        public float AlignmentWeight;
        public float CohesionWeight;
        public float AvoidanceWeight;
        public bool PushStanding;
        public bool TightFormation;
        public float DeltaTime;
        [ReadOnly]
        public NativeArray<JobifiedBoidsAgent> Agents;
        [WriteOnly]
        public NativeArray<float2> Impulses;

        public void Execute(int i)
        {
            var agent = Agents[i];

            float2 impulse = float2.zero;

            // Notice: In scalable game this would been optimized. For example using spatial partitioning for querying only nearby agents
            // Check JobifiedAgentSystem it has spatial partitioning done with multi hash map
            var nearbyAgents = Agents;

            if (agent.CanMove())
            {
                impulse += GetSeek(agent, agent.Destination);
                impulse += GetAlignment(nearbyAgents, agent) * AlignmentWeight;
                impulse += GetCohesion(nearbyAgents, agent) * CohesionWeight;
                impulse += GetSeparation(nearbyAgents, agent) * SeperationWeight;
                impulse += GetAvoid(nearbyAgents, agent, math.normalizesafe(agent.Destination - agent.Position)) * AvoidanceWeight;
            }
            else
            {
                if (!agent.Pushable || !PushStanding)
                    return;

                impulse += GetSeparation(nearbyAgents, agent) * SeperationWeight;

                // TODO: This should account of time
                agent.Velocity *= 0.9f;
            }

            Impulses[i] = impulse;
        }

        float2 GetSeek(JobifiedBoidsAgent agent, float2 destination)
        {
            float2 position = agent.Position;

            if (math.all(position == destination))
                return float2.zero;

            float2 desired = destination - position;

            desired *= agent.MaxSpeed/math.length(desired);

            float2 velocityChange = desired - agent.Velocity;

            velocityChange *= agent.ForceDivSpeed;

            return velocityChange;
        }

        float2 GetSeparation(NativeArray<JobifiedBoidsAgent> agents, JobifiedBoidsAgent agent)
        {
            float2 totalForce = float2.zero;
            int neighboursCount = 0;
            int count = agents.Length;

            for (var i = 0; i < count; i++)
            {
                JobifiedBoidsAgent targetAgent = agents[i];

                if (agent.CanSeperate(targetAgent))
                {
                    float2 pushForce = agent.Position - targetAgent.Position;
                    float distance = math.length(pushForce);

                    if (distance < agent.Separation && distance > 0)
                    {
                        float r = (agent.Radius + targetAgent.Radius);

                        totalForce += pushForce*(1f - ((distance - r)/(agent.Separation - r)));

                        neighboursCount++;
                    }
                }
            }

            if (neighboursCount == 0)
                return float2.zero;

            totalForce *= agent.MaxForce/neighboursCount;

            return totalForce;
        }

        float2 GetCohesion(NativeArray<JobifiedBoidsAgent> agents, JobifiedBoidsAgent agent)
        {
            float2 centerOfMass = float2.zero;
            int neighboursCount = 0;
            int count = agents.Length;

            for (var i = 0; i < count; i++)
            {
                JobifiedBoidsAgent targetAgent = agents[i];

                if (agent.CanFlock(targetAgent))
                {
                    float distance = math.distance(agent.Position, targetAgent.Position);

                    if (distance < agent.Cohesion)
                    {
                        centerOfMass += targetAgent.Position;

                        neighboursCount++;
                    }
                }
            }

            if (neighboursCount == 0)
                return float2.zero;

            centerOfMass /= neighboursCount;

            return GetSeek(agent, centerOfMass);
        }

        float2 GetAlignment(NativeArray<JobifiedBoidsAgent> agents, JobifiedBoidsAgent agent)
        {
            float2 averageHeading = float2.zero;
            int neighboursCount = 0;
            int count = agents.Length;

            for (var i = 0; i < count; i++)
            {
                JobifiedBoidsAgent targetAgent = agents[i];
                float distance = math.distance(agent.Position, targetAgent.Position);
                float speed = math.length(targetAgent.Velocity);

                if (distance < agent.Cohesion && speed > 0 && agent.CanFlock(targetAgent))
                {
                    float2 head = targetAgent.Velocity/speed;

                    averageHeading += head;
                    neighboursCount++;
                }
            }

            if (neighboursCount == 0)
                return averageHeading;

            averageHeading /= neighboursCount;

            return GetSteerTowards(agent, averageHeading);
        }

        float2 GetAvoid(NativeArray<JobifiedBoidsAgent> agents, JobifiedBoidsAgent agent,
            float2 desiredDirection)
        {
            if (math.lengthsq(desiredDirection) == 0)
                return float2.zero;

            float maxDistance = math.min(6, math.distance(agent.Destination, agent.Position));
            int count = agents.Length;

            if (maxDistance == 0)
                return float2.zero;

            SonarAvoidance sonar = new SonarAvoidance(
                new float3(agent.Position.x, 0, agent.Position.y),
                new float3(desiredDirection.x, 0, desiredDirection.y),
                new float3(0, 1, 0),
                agent.Radius,
                maxDistance,
                math.length(new float3(agent.Velocity.x, 0, agent.Velocity.y))
            );

            var agentCircle = new Circle(agent.Position + math.length(agent.Velocity) * DeltaTime * desiredDirection, agent.Radius);
            bool desiredDirectionOccluded = false;

            for (int i = 0; i < count; i++)
            {
                JobifiedBoidsAgent targetAgent = agents[i];
                float2 direction = targetAgent.Position - agent.Position;
                float distance = math.length(direction);

                if (!agent.CanAvoid(targetAgent))
                    continue;

                sonar.InsertObstacle(
                    new float3(targetAgent.Position.x, 0, targetAgent.Position.y),
                    new float3(targetAgent.Velocity.x, 0, targetAgent.Velocity.y),
                    targetAgent.Radius);

                // Check if taking a step to desired direction would collide with target agent
                if (TightFormation && !desiredDirectionOccluded && Circle.Collide(agentCircle, new Circle(targetAgent.Position, targetAgent.Radius)))
                {
                    desiredDirectionOccluded = true;
                }
            }

            sonar.InsertObstacle(math.normalizesafe(-new float3(agent.Velocity.x, 0, agent.Velocity.y)), math.radians(135));

            bool success = sonar.FindClosestDirection(out float3 newDirection);

            // If all directions obstructed, try to at least move agent as close as possible to destination without colliding
            // This is not necessary step, but it yields much better formation movement
            if (TightFormation && !success && !desiredDirectionOccluded)
            {
                newDirection = new float3(desiredDirection.x, 0, desiredDirection.y);
            }

            sonar.Dispose();

            return GetSteerTowards(agent, new float2(newDirection.x, newDirection.z));
        }

        float2 GetSteerTowards(JobifiedBoidsAgent agent, float2 desiredDirection)
        {
            float2 desiredVelocity = desiredDirection*agent.MaxSpeed;
            float2 velocityChange = desiredVelocity - agent.Velocity;

            velocityChange *= agent.ForceDivSpeed;

            return velocityChange;
        } 
    }

    /// <summary>
    /// Demo system of BOIDS simulation.
    /// BOIDS is an artificial life program of simulating birds flocking behaviour.
    /// This implementation uses: Seek, Alignment, Seperation and Sonar Avoidance of this package.
    /// You can find more information in https://en.wikipedia.org/wiki/Boids.
    /// </summary>
    [DefaultExecutionOrder(50)]
    public class JobifiedBoidsSystem : System
    {
        static class Markers
        {
            public static readonly ProfilerMarker Convert = new ProfilerMarker("Convert");
            public static readonly ProfilerMarker Boids = new ProfilerMarker("Boids");
            public static readonly ProfilerMarker Transform = new ProfilerMarker("Transform");
            public static readonly ProfilerMarker Cleanup = new ProfilerMarker("Cleanup");
        }

        public float SeperationWeight = 2.2f;
        public float AlignmentWeight = 0.3f;
        public float CohesionWeight = 0.05f;
        public float AvoidanceWeight = 2.8f;
        public bool PushStanding = false;
        public bool ChangeTransform = true;
        public bool TightFormation = true;

        void FixedUpdate()
        {
            var agents = Query<BoidsAgent>();

            // Early out
            if (agents.Length == 0)
                return;

            JobHandle dependency = new JobHandle();

            var jobifiedAgents = new NativeArray<JobifiedBoidsAgent>(agents.Length, Allocator.TempJob);
            using (Markers.Convert.Auto())
            {
                for (int i = 0; i < agents.Length; ++i)
                {
                    var agent = agents[i];
                    jobifiedAgents[i] = new JobifiedBoidsAgent(i, agent);
                }
            }

            var impulses = new NativeArray<float2>(agents.Length, Allocator.TempJob);

            using (Markers.Boids.Auto())
            {
                var job = new BoidsJob
                {
                    SeperationWeight = SeperationWeight,
                    AlignmentWeight = AlignmentWeight,
                    CohesionWeight = CohesionWeight,
                    AvoidanceWeight = AvoidanceWeight,
                    PushStanding = PushStanding,
                    TightFormation = TightFormation,
                    DeltaTime = Time.deltaTime,
                    Agents = jobifiedAgents,
                    Impulses = impulses,
                };
                dependency = job.Schedule(agents.Length, 5);
            }

            dependency.Complete();

            using (Markers.Transform.Auto())
            {
                int index = 0;
                foreach (var agent in agents)
                {
                    float2 impulse = impulses[index++];

                    var nearbyAgents = agents;

                    if (ChangeTransform)
                    {
                        agent.ApplyImpulse(impulse);
                        Distinguish(nearbyAgents, agent);
                    }

                    Arrival(agent);
                }
            }

            using (Markers.Cleanup.Auto())
            {
                jobifiedAgents.Dispose();
                impulses.Dispose();
            }
        }

        void Arrival(BoidsAgent agent)
        {
            float distance = math.distance(agent.Position, agent.Destination);

            if (distance <= agent.StopingDistance)
                agent.Stop();
        }

        void Distinguish(IList<BoidsAgent> agents, BoidsAgent agent)
        {
            int count = agents.Count;

            for (int i = 0; i < count; i++)
            {
                BoidsAgent targetAgent = agents[i];

                if (agent.CanDistinguish(targetAgent))
                {
                    float2 direction = agent.Position - targetAgent.Position;
                    float directionMagnitude = math.length(direction);
                    float distance = agent.Radius + targetAgent.Radius - directionMagnitude;
                    float2 seperate = direction/directionMagnitude*distance;

                    if (distance > 0 && directionMagnitude > 0)
                    {
                        agent.transform.position += new Vector3(seperate.x, 0, seperate.y) *
                            agent.GetDistinguishWeigth(targetAgent);
                    }
                }
            }
        }
    }
}