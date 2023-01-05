using UnityEngine;
using UnityEngine.AI;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// Agent of <see cref="BoidsSystem"/>.
    /// </summary>
    public struct JobifiedBoidsAgent
    {
        public int Index;
        public float2 Position;
        public float MaxForce;
        public float MaxSpeed;
        public float Radius;
        public int SeperationGroup;

        public float2 Velocity;
        public float2 Destination;
        public bool Pushable;
        public bool Paused;

        public float Separation;
        public float Cohesion;
        public float Avoidance;
        public float ForceDivSpeed;
        public int FlockingGroup;

        public JobifiedBoidsAgent(int index, BoidsAgent agent)
        {
            Index = index;
            Position = agent.Position;
            MaxForce = agent.MaxForce;
            MaxSpeed = agent.MaxSpeed;
            Radius = agent.Radius;
            SeperationGroup = agent.SeperationGroup;
            Velocity = agent.Velocity;
            Destination = agent.Destination;
            Pushable = agent.Pushable;
            Paused = agent.Paused;
            Separation = agent.Separation;
            Avoidance = agent.Avoidance;
            Cohesion = agent.Cohesion;
            ForceDivSpeed = agent.ForceDivSpeed;
            FlockingGroup = agent.FlockingGroup;
        }

        public bool CanFlock(JobifiedBoidsAgent agent)
        {
            return Index != agent.Index && agent.FlockingGroup == FlockingGroup;
        }

        public bool CanSeperate(JobifiedBoidsAgent agent)
        {
            return Index != agent.Index && (agent.SeperationGroup == SeperationGroup);
        }

        public bool CanDistinguish(JobifiedBoidsAgent agent)
        {
            return Index != agent.Index;
        }

        public bool CanAvoid(JobifiedBoidsAgent agent)
        {
            return Index != agent.Index;
        }

        public bool CanMove()
        {
            return !Paused;
        }

        public void Stop()
        {
            Velocity = float2.zero;
            Paused = true;
        }

        public void Resume()
        {
            Paused = false;
        }

        public bool SetDestination(float3 destination)
        {
            if (NavMesh.SamplePosition(destination, out var hit, 2, -1))
            {
                Resume();
                Destination = new float2(hit.position.x, hit.position.z);
                FlockingGroup = (int) Destination.x << 16 + (int) Destination.y;

                return true;
            }

            return false;
        }
    }
}