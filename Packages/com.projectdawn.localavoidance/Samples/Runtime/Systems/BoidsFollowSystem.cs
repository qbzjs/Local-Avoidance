using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// System that sets destination either to ground or boids agent.
    /// </summary>
    [DefaultExecutionOrder(45)]
    public class BoidsFollowSystem : System
    {
        public float StopDistanceError = 0.1f;

        void Update()
        {
            // Move where mouse was clicked
            if (Input.GetMouseButtonDown(0))
            {
                var positionSS = Input.mousePosition;
                var ray = Camera.main.ScreenPointToRay(positionSS);

                if (!Physics.Raycast(ray, out var hitInfo, float.MaxValue))
                    return;

                if (hitInfo.collider.TryGetComponent<BoidsAgent>(out var targetAgent))
                {
                    foreach (var followInput in Query<FollowInput>())
                    {
                        if (followInput.TryGetComponent(out Path path))
                        {
                            float3 target = targetAgent.transform.position;
                            path.Destination = target;
                        }
                        else if (followInput.TryGetComponent(out BoidsAgent agent))
                        {

                            float3 target = targetAgent.transform.position;
                            agent.StopingDistance = agent.Radius + targetAgent.Radius + StopDistanceError;
                            agent.SetDestination(target);
                        }
                    }
                }
                else if (hitInfo.collider.TryGetComponent<MeshCollider>(out var meshCollider))
                {
                    foreach (var followInput in Query<FollowInput>())
                    {
                        if (followInput.TryGetComponent(out Path path))
                        {
                            float3 target = ray.GetPoint(hitInfo.distance);
                            path.Destination = target;
                        }
                        else if (followInput.TryGetComponent(out BoidsAgent agent))
                        {
                            float3 target = ray.GetPoint(hitInfo.distance);
                            agent.StopingDistance = agent.Radius + StopDistanceError;
                            agent.SetDestination(target);
                        }
                    }
                }
            }
        }
    }
}
