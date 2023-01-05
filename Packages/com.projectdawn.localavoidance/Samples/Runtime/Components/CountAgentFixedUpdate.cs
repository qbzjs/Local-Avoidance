using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance
{
    [RequireComponent(typeof(Agent))]
    public class CountAgentFixedUpdate : MonoBehaviour
    {
        public int Count;
        public int MaxExpectedCount = 10000;

        void FixedUpdate()
        {
            var agent = GetComponent<Agent>();

            if (!agent.IsStopped)
                Count++;
            Debug.Assert(MaxExpectedCount >= Count);
        }
    }
}
