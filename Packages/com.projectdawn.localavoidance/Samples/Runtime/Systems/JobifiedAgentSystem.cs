using UnityEngine;
using UnityEngine.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Profiling;

namespace ProjectDawn.LocalAvoidance.Demo.New
{
    [BurstCompile]
    public struct ReadTransformJob : IJobParallelForTransform
    {
        [WriteOnly]
        public NativeArray<float3> Positions;
        [WriteOnly]
        public NativeArray<quaternion> Rotations;

        public void Execute(int i, TransformAccess transformAccess)
        {
            Positions[i] = transformAccess.position;
            Rotations[i] = transformAccess.rotation;
        }
    }

    [BurstCompile]
    public struct WriteTransformJob : IJobParallelForTransform
    {
        [ReadOnly]
        public NativeArray<float3> Positions;
        [ReadOnly]
        public NativeArray<quaternion> Rotations;

        public void Execute(int i, TransformAccess transformAccess)
        {
            transformAccess.position = Positions[i];
            transformAccess.rotation = Rotations[i];
        }
    }

    [BurstCompile]
    public struct SpatialPartitioningJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<float3> Positions; 
        [WriteOnly]
        public NativeMultiHashMap<int, int>.ParallelWriter HashMap;
        public float CellRadius;

        public void Execute(int i)
        {
            var hash = (int)math.hash(new int2(
                (int)(Positions[i].x / CellRadius),
                (int)(Positions[i].z / CellRadius)));
            HashMap.Add(hash, i);
        }
    }
    
    [BurstCompile]
    public struct LocalAvoidanceJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<AgentData> Agents;
        [ReadOnly]
        public NativeArray<float3> Positions;
        [ReadOnly]
        public NativeArray<float3> Velocities;

        [WriteOnly]
        public NativeArray<float3> Impulses;

        public float SonarCutBackVisionAngle;
        public float SonarRadius;
        public float DeltaTime;
        public bool Is3D;
        public bool TightFormation;

        [ReadOnly]
        public NativeMultiHashMap<int, int> HashMap;
        public float CellRadius;

        public void Execute(int i)
        {
            var agent = Agents[i];
            var agentPosition = Positions[i];
            var agentVelocity = Velocities[i];

            if (!agent.IsStopped(agentPosition) && agent.Speed > 0)
            {   
                float3 desiredDirection = math.normalizesafe(agent.Destination - agentPosition);
                desiredDirection.y = 0;

                float3 impulse = 0;
                if (agent.Avoid)
                    impulse += GetAvoid(i, agent, agentPosition, agentVelocity, desiredDirection);
                else
                    impulse += desiredDirection * agent.Speed;
                Impulses[i] = impulse;
            }
            else
            {
                Impulses[i] = 0;
            }
        }

        float3 GetAvoid(int current, AgentData agent, float3 agentPosition, float3 agentVelocity, float3 desiredDirection)
        {
            if (math.lengthsq(desiredDirection) == 0)
                return float3.zero;

            var sonarRadius = math.min(SonarRadius, math.distance(agent.Destination, agentPosition));

            var up = Is3D ? new float3(0, 1, 0) : new float3(0, 0, -1);
            var sonar = new SonarAvoidance(agentPosition, desiredDirection, up, agent.Radius, sonarRadius, math.length(agentVelocity));

            var agentCircle = new Circle(agentPosition.xz + math.length(agentVelocity) * DeltaTime * desiredDirection.xz, agent.Radius);
            bool desiredDirectionOccluded = false;

            var nearbyAgents = Agents;

            float2 boxMin = (agentPosition.xz - SonarRadius);
            float2 boxMax = (agentPosition.xz + SonarRadius);

            // Find how many chunks does box overlap
            int2 min = (int2) (boxMin / CellRadius);
            int2 max = (int2) (boxMax / CellRadius) + 2;

            for (int i = min.x; i < max.x; ++i)
            {
                for (int j = min.y; j < max.y; ++j)
                {
                    int key = GetCellHash(i, j);

                    // Find all entities in the bucket
                    if (HashMap.TryGetFirstValue(key, out var index, out var iterator))
                    {
                        do
                        {
                            // Skip itself
                            if (index == current)
                                continue;

                            var nearbyAgent = nearbyAgents[index];
                            var nearbyAgentPosition = Positions[index];
                            var nearbyAgentVelocity = Velocities[index];
                            sonar.InsertObstacle(nearbyAgentPosition, nearbyAgentVelocity, nearbyAgent.Radius);

                            // Check if taking a step to desired direction would collide with target agent
                            if (TightFormation && !desiredDirectionOccluded && Circle.Collide(agentCircle, new Circle(nearbyAgentPosition.xz, nearbyAgent.Radius)))
                            {
                                desiredDirectionOccluded = true;
                            }
                        }
                        while (HashMap.TryGetNextValue(out index, ref iterator));
                    }
                }
            }

            // Add blocker behind the velocity
            // This will prevent situations where agent has on right and left equally good paths
            sonar.InsertObstacle(math.normalizesafe(-agentVelocity), math.radians(SonarCutBackVisionAngle));

            bool success = sonar.FindClosestDirection(out float3 newDirection);

            // If all directions obstructed, try to at least move agent as close as possible to destination without colliding
            // This is not necessary step, but it yields much better formation movement
            if (TightFormation && !success && !desiredDirectionOccluded)
            {
                newDirection = new float3(desiredDirection.x, 0, desiredDirection.y);
            }

            sonar.Dispose();

            var impulse = newDirection * agent.Speed;

            return impulse;
        }

        static int GetCellHash(int x, int y)
        {
            var hash = (int)math.hash(new int2(x, y));
            return hash;
        }
    }

    [BurstCompile]
    public struct TransformJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<AgentData> Agents;
        [ReadOnly]
        public NativeArray<float3> Impulses;

        public NativeArray<float3> Velocities;
        public NativeArray<float3> Positions;
        public NativeArray<quaternion> Rotations;
        public float DeltaTime;
        public bool Is3D;

        public void Execute(int i)
        {
            var agent = Agents[i];
            var agentPosition = Positions[i];
            var agentVelocity = Velocities[i];
            var agentRotation = Rotations[i];

            if (!agent.IsStopped(agentPosition) && agent.Speed > 0)
            {   
                agentVelocity = math.lerp(agentVelocity, Impulses[i], DeltaTime * agent.Acceleration); 
            }
            else
            {
                agentVelocity = 0;
            }
            Velocities[i] = agentVelocity;

            if (math.lengthsq(agentVelocity) != 0)
            {
                var offset = agentVelocity * DeltaTime;

                // Avoid over-steping destination
                var distance = math.distance(agent.Destination, agentPosition);
                offset = ForceLength(offset, distance);

                Positions[i] += offset;

                if (Is3D)
                {
                    var direction = math.normalizesafe(agentVelocity);
                    float angle = math.atan2(direction.z, -direction.x);
                    var rotation = quaternion.RotateY(angle);
                    rotation = quaternion.LookRotation(math.normalizesafe(new float3(agentVelocity.x, 0, agentVelocity.z)), new float3(0, 1, 0));
                    Rotations[i] = math.slerp(agentRotation, rotation, math.saturate(DeltaTime * agent.TurnSpeed));
                }
            }
        }

        float3 ForceLength(float3 value, float length)
        {
            var valueLength = math.length(value);
            return valueLength > length ? value / valueLength * length : value;
        }
    }

    /// <summary>
    /// System that executs local avoidance logic for <see cref="Agent"/>.
    /// Uses jobs system.
    /// </summary>
    [DefaultExecutionOrder(50)]
    public class JobifiedAgentSystem : System
    {
        static class Markers
        {
            public static readonly ProfilerMarker Convert = new ProfilerMarker("Convert");
            public static readonly ProfilerMarker ReadTransform = new ProfilerMarker("ReadTransform");
            public static readonly ProfilerMarker WriteTransform = new ProfilerMarker("WriteTransform");
            public static readonly ProfilerMarker SpatialPartitioning = new ProfilerMarker("SpatialPartitioning");
            public static readonly ProfilerMarker LocalAvoidance = new ProfilerMarker("LocalAvoidance");
            public static readonly ProfilerMarker Transform = new ProfilerMarker("Transform");
            public static readonly ProfilerMarker UpdateVelocities = new ProfilerMarker("UpdateVelocities");
            public static readonly ProfilerMarker Cleanup = new ProfilerMarker("Cleanup");
        }

        [Range(1, 10)]
        public float SonarRadius = 6;
        [Range(0, 180)]
        public float SonarCutBackVisionAngle = 135f;
        public bool TightFormation = false;
        [Range(1, 100)]
        public int AgentPerJob = 5;
        public float SpatialPartitioningCellRadius = 5;

        void FixedUpdate()
        {
            var agents = Query<Agent>();

            // Early out
            if (agents.Length == 0)
                return;

            JobHandle dependency = new JobHandle();

            var transforms = new TransformAccessArray(agents.Length);
            for (int agentIndex = 0; agentIndex < agents.Length; ++agentIndex)
            {
                var agent = agents[agentIndex];
                transforms.Add(agent.transform);
            }

            // Write transforms
            var positions = new NativeArray<float3>(agents.Length, Allocator.TempJob);
            var rotations = new NativeArray<quaternion>(agents.Length, Allocator.TempJob);
            using (Markers.ReadTransform.Auto())
            {
                var job = new ReadTransformJob
                {
                    Positions = positions,
                    Rotations = rotations,
                };
                dependency = job.Schedule(transforms, dependency);
            }

            // Spatial partition all agents
            var hashMap = new NativeMultiHashMap<int, int>(agents.Length, Allocator.TempJob);
            using (Markers.SpatialPartitioning.Auto())
            {
                var job = new SpatialPartitioningJob
                {
                    Positions = positions,
                    HashMap = hashMap.AsParallelWriter(),
                    CellRadius = SpatialPartitioningCellRadius,
                };
                dependency = job.Schedule(agents.Length, AgentPerJob, dependency);
            }

            // Convert Agent to AgentData
            var agentData = new NativeArray<AgentData>(agents.Length, Allocator.TempJob);
            var velocities = new NativeArray<float3>(agents.Length, Allocator.TempJob);
            using (Markers.Convert.Auto())
            {
                for (int agentIndex = 0; agentIndex < agents.Length; ++agentIndex)
                {
                    agentData[agentIndex] = agents[agentIndex].GetData();
                    velocities[agentIndex] = agents[agentIndex].Velocity;
                }
            }

            // Local Avoidance job
            var impulses = new NativeArray<float3>(agents.Length, Allocator.TempJob);
            using (Markers.LocalAvoidance.Auto())
            {
                var job = new LocalAvoidanceJob
                {
                    Agents = agentData,
                    Positions = positions,
                    Velocities = velocities,
                    Impulses = impulses,
                    CellRadius = SpatialPartitioningCellRadius,
                    HashMap = hashMap,
                    SonarCutBackVisionAngle = SonarCutBackVisionAngle,
                    SonarRadius = SonarRadius,
                    TightFormation = TightFormation,
                    Is3D = true,
                    DeltaTime = Time.deltaTime,
                };
                dependency = job.Schedule(agents.Length, AgentPerJob, dependency);
            }

            // Transform job
            using (Markers.Transform.Auto())
            {
                var job = new TransformJob
                {
                    Agents = agentData,
                    Positions = positions,
                    Rotations = rotations,
                    Velocities = velocities,
                    Impulses = impulses,
                    DeltaTime = Time.deltaTime,
                    Is3D = true,
                };
                dependency = job.Schedule(agents.Length, AgentPerJob, dependency);
            }

            // Transform job
            using (Markers.WriteTransform.Auto())
            {
                var job = new WriteTransformJob
                {
                    Positions = positions,
                    Rotations = rotations,
                };
                dependency = job.Schedule(transforms, dependency);
            }

            dependency.Complete();

            // Update velocities
            using (Markers.UpdateVelocities.Auto())
            {
                for (int agentIndex = 0; agentIndex < agents.Length; ++agentIndex)
                {
                    agents[agentIndex].Velocity = velocities[agentIndex];
                }
            }

            using (Markers.Cleanup.Auto())
            {
                impulses.Dispose();
                transforms.Dispose();
                agentData.Dispose();
                positions.Dispose();
                rotations.Dispose();
                velocities.Dispose();
                hashMap.Dispose();
            }
        }
    }
}
