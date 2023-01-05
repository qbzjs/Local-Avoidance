using UnityEngine;
using UnityEngine.Assertions;
using Unity.Mathematics;
using Unity.Collections;
using UnityEngine.AI;

namespace ProjectDawn.LocalAvoidance.Demo
{
    [DefaultExecutionOrder(46)]
    public class PathSystem : System
    {
        public float StopDistanceError = 0.1f;
        NavMeshPath m_SharedNavmeshPath;

        void FixedUpdate()
        {
            var paths = Query<Path>();

            if (m_SharedNavmeshPath == null)
                m_SharedNavmeshPath = new NavMeshPath();

            foreach (var path in paths)
            {
                if (!path.Points.IsCreated)
                    path.Points = new NativeList<float3>(Allocator.Persistent);

                // Update path if needed
                SetDestination(path, path.Destination);

                if (IsDestinationChanged(path))
                {
                    if (path.TryGetComponent(out BoidsAgent boidsAgent))
                    {
                        var point = path.Points[path.CurrentPointIndex];
                        boidsAgent.StopingDistance = path.Finished ? 1 : 0;
                        boidsAgent.SetDestination(point);
                    }
                }
            }
        }

        bool SetDestination(Path path, float3 destination)
        {
            if (path.Points.Length > 0)
            {
                var towards = destination - path.Points[path.Points.Length - 1];

                // Ignore height
                towards.y = 0;

                if (math.lengthsq(towards) < 0.01f)
                    return false;
            }

            if (NavMesh.CalculatePath(path.transform.position, destination, -1, m_SharedNavmeshPath))
            {
                var navPoints = m_SharedNavmeshPath.corners;
                int count = navPoints.Length;

                path.Points.Clear();

                for (int i = 0; i < count; i++)
                {
                    Vector3 point = navPoints[i];

                    path.Points.Add(point);
                }

                path.CurrentPointIndex = 0;

                return true;
            }

            return false;
        }

        bool IsDestinationChanged(Path path)
        {
            if (path.Finished)
                return false;

            var point = path.Points[path.CurrentPointIndex + 1];
            if (!NavMesh.Raycast(path.transform.position, point, out var hit, -1))
            {
                path.CurrentPointIndex++;
                return true;
            }

            return false;
        }
    }
}
