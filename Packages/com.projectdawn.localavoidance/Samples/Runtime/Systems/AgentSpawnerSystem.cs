using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// Spawns agent with count and radius.
    /// </summary>
    public class AgentSpawnerSystem : System
    {
        public Agent Prefab;
        public Agent FollowTarget;
        public Transform Target;
        public int Count;
        public float InnerRadius;
        public float OuterRadius;

        void Start()
        {
            if (Prefab == null)
                return;

            var rnd = new Unity.Mathematics.Random(1);
            for (int i = 0; i < Count; ++i)
            {
                var direction = math.mul(quaternion.RotateY(i * math.radians(5)), new float3(1, 0, 0));
                var position = (float3)transform.position + new float3(direction.x, 0, direction.z) * math.lerp(InnerRadius, OuterRadius, ((float) i / Count));

                var instance = GameObject.Instantiate(Prefab.gameObject);
                instance.transform.position = position;

                //var followAgent = instance.GetComponent<FollowAgent>();
                //followAgent.TargetAgent = FollowTarget;

                if (Target)
                {
                    var followTranform = instance.GetComponent<FollowTransform>();
                    followTranform.Target = Target.transform;
                }
            }
        }
    }
}
