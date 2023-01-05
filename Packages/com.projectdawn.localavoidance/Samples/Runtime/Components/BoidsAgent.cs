using UnityEngine;
using UnityEngine.AI;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// Agent of <see cref="BoidsSystem"/>.
    /// </summary>
    public class BoidsAgent : MonoBehaviour
    {
        [SerializeField]
        private float _StopingDistance = 0.5f;
        [SerializeField]
        private float _MaxForce = 1f;
        [SerializeField]
        private float _MaxSpeed = 0.1f;
        [SerializeField]
        private float _Radius = 0.5f;
        [SerializeField]
        private int _SeperationGroup;
        public float TurnSpeed = 10;

        public float2 _Velocity;
        private float2 _Destination;
        [SerializeField]
        private bool _Pushable = true;
        private bool _Paused;

        private float _Separation;
        private float _Cohesion;
        private float _Avoidance;
        private float _ForceDivSpeed;
        private int _FlockingGroup;

        public float2 Destination { get { return _Destination; } }
        public float2 Velocity { get { return _Velocity; } set { _Velocity = value; } }
        public float StopingDistance { set { _StopingDistance = value; } get { return _StopingDistance; } }
        public int FlockingGroup { get { return _FlockingGroup; } }
        public int SeperationGroup { get { return _SeperationGroup; } set { _SeperationGroup = value; } }
        public float MaxForce { get { return _MaxForce; } }
        public float MaxSpeed { get { return _MaxSpeed; } }
        public float Radius { get { return _Radius; } set { _Radius = value; } }
        public float Separation { get { return _Separation; } }
        public float Cohesion { get { return _Cohesion; } }
        public float Avoidance { get { return _Avoidance; } }
        public float ForceDivSpeed { get { return _ForceDivSpeed; } }
        public bool Pushable { get { return _Pushable; } set { _Pushable = value; } }
        public bool Paused => _Paused;

        public float2 Position
        {
            get
            {
                float3 pos = transform.position;

                return new float2(pos.x, pos.z);
            }
        }

        public void SetSpeedForce(float speed, float force)
        {
            _MaxSpeed = speed;
            _MaxForce = force;
            _ForceDivSpeed = _MaxForce/_MaxSpeed;
        }

        public void SetRadius(float radius)
        {
            _Radius = radius;
            _Separation = _Radius*2.3f;
            _Cohesion = _Radius*10f;
            _Avoidance = _Radius*10f;
        }

        public bool CanFlock(BoidsAgent agent)
        {
            return this != agent && agent.FlockingGroup == _FlockingGroup;
        }

        public bool CanSeperate(BoidsAgent agent)
        {
            return this != agent && (agent.SeperationGroup == _SeperationGroup);
        }

        public bool CanDistinguish(BoidsAgent agent)
        {
            return this != agent && _Pushable;
        }

        public bool CanAvoid(BoidsAgent agent)
        {
            // Avoid only agents that are in front
            return this != agent;// && IsInFront(agent);
        }

        /// <summary>
        /// True if agent is in front
        /// </summary>
        bool IsInFront(BoidsAgent agent)
        {
            var direction = math.normalizesafe(agent.Position - Position);
            bool agentIsInFront = math.dot(direction, math.normalizesafe(_Velocity)) >= math.cos(math.radians(135));
            return agentIsInFront;
        }

        public float GetDistinguishWeigth(BoidsAgent agent)
        {
            //TODO: should depend from seperationGroup
            return 0.5f;
        }

        public bool CanMove()
        {
            return !_Paused;
        }

        public void Stop()
        {
            _Velocity = float2.zero;
            _Paused = true;
        }

        public void Resume()
        {
            _Paused = false;
        }

        public bool SetDestination(float3 destination)
        {
            if (NavMesh.SamplePosition(destination, out var hit, 2, -1))
            {
                Resume();
                _Destination = new float2(hit.position.x, hit.position.z);
                _FlockingGroup = (int) _Destination.x << 16 + (int) _Destination.y;

                return true;
            }

            return false;
        }

        public void OffsetPosition(Vector3 offset)
        {
            if (NavMesh.SamplePosition(transform.position, out var hit, 2, -1) &&
                NavMesh.SamplePosition(transform.position + offset, out var hit2, 2, -1) &&
                math.distance(hit.position, hit2.position) < 0.2f
            )
            {
                transform.position = hit2.position;
            }
        }

        public void ApplyImpulse(float2 impulse)
        {
            float length = math.length(impulse);

            if (length > MaxForce)
                impulse *= MaxForce/length;

            if (Time.deltaTime > 0.15f)
                return;

            impulse *= Time.deltaTime;
            _Velocity += impulse;

            float magnitude = math.length(_Velocity);

            if (magnitude > _MaxSpeed)
            {
                _Velocity *= _MaxSpeed/magnitude;
            }

            if (magnitude < 0.1f)
                _Velocity = float2.zero;

            OffsetPosition(new Vector3(_Velocity.x, 0, _Velocity.y) * Time.deltaTime);

            var rotation = quaternion.LookRotation(math.normalizesafe(new float3(_Velocity.x, 0, _Velocity.y)), new float3(0, 1, 0));
            transform.rotation = math.slerp(transform.rotation, rotation, Time.deltaTime * TurnSpeed);
        }

        private void Awake()
        {
            Stop();
            SetRadius(_Radius);
            _ForceDivSpeed = _MaxForce/_MaxSpeed;
            //_Pushable = true;
        }
    }
}