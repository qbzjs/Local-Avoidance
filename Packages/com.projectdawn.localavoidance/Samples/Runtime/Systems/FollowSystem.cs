using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance.Demo
{
    [DefaultExecutionOrder(49)]
    public class FollowSystem : System
    {
        void Update()
        {
            // Follow target
            foreach (var followAgent in Query<FollowAgent>())
            {
                if (!followAgent.TryGetComponent(out Agent agent))
                    continue;

                if (!followAgent.TargetAgent)
                    continue;

                var target = followAgent.TargetAgent.Position;

                // Offset by radius, so it would try to get as near as possible the target and not inside of it
                var directionOffset = math.normalizesafe(agent.Position - target);
                var offset = directionOffset * (agent.Radius + followAgent.TargetAgent.Radius);
                target += offset;

                agent.Destination = target;
            }

            // Follow transform
            foreach (var followTransform in Query<FollowTransform>())
            {
                if (!followTransform.TryGetComponent(out Agent agent))
                    continue;

                if (followTransform.Target)
                {
                    agent.Destination = followTransform.Target.position;
                }
            }

            // Move where mouse was clicked
            if (Input.GetMouseButtonDown(0))
            {
                var positionSS = Input.mousePosition;
                var ray = Camera.main.ScreenPointToRay(positionSS);

                // Lets use ground as singlaton for ground raycasting
                var ground = GameObject.FindObjectOfType<Ground>();

                if (ground.TryGetComponent(out SpriteRenderer boxCollider))
                {
                    var plane = new Plane(new float3(0, 0, -1), float3.zero);

                    if (!plane.Raycast(ray, out var hit))
                        return;

                    foreach (var followInput in Query<FollowInput>())
                    {
                        if (!followInput.TryGetComponent(out Agent agent))
                            continue;

                        agent.Destination = ray.GetPoint(hit);
                    }
                    return;
                }

                if (!ground.TryGetComponent(out MeshCollider meshCollider))
                    return;

                if (!meshCollider.Raycast(ray, out var hitInfo, float.MaxValue))
                    return;

                foreach (var followInput in Query<FollowInput>())
                {
                    if (!followInput.TryGetComponent(out Agent agent))
                        continue;

                    agent.Destination = ray.GetPoint(hitInfo.distance);
                }
            }
        }
    }
}
