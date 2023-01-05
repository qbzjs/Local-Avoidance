using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance
{
    public class Path : MonoBehaviour
    {
        public float3 Destination;
        public NativeList<float3> Points;
        public int CurrentPointIndex;

        public bool Finished => !Points.IsCreated || CurrentPointIndex >= Points.Length - 1;

        void OnDestroy()
        {
            if (Points.IsCreated)
                Points.Dispose();
        }
    }
}
