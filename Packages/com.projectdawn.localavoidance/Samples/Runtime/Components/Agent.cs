using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance
{
    public class Agent : MonoBehaviour
    {
        public float3 Destination;
        public float Speed = 1;
        public float TurnSpeed = 10;
        public float Radius = 0.5f;
        public float StopDistance = 0.2f;
        public float3 Velocity;
        public bool Avoid = true;

        public float3 Position => transform.position;
        public bool IsStopped => math.distance(Destination, transform.position) < StopDistance + 0.01f;

        public AgentData GetData()
        {
            return new AgentData
            {
                Destination = Destination,
                Speed = Speed,
                TurnSpeed = TurnSpeed,
                Radius = Radius,
                StopDistance = StopDistance,
                Avoid = Avoid,
                Acceleration = 8,
            };
        }

        /// <summary>
        /// Callback to draw gizmos only if the object is selected.
        /// </summary>
        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(transform.position, Radius);
        }
    }

    public struct AgentData
    {
        public float3 Destination;
        public float Speed;
        public float TurnSpeed;
        public float Radius;
        public float StopDistance;
        public bool Avoid;
        public float Acceleration;

        public bool IsStopped(float3 position)
        {
            return math.distance(Destination, position) < StopDistance + 0.01f;
        }
    }
}
