using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance
{
    public class AgentDebug : MonoBehaviour
    {
        public SonarAvoidance Vision;

        void OnDestroy()
        {
            if (Vision.IsCreated)
                Vision.Dispose();
        }
    }
}
