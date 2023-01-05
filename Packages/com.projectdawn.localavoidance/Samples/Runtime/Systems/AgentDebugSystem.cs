using UnityEngine;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// System for debugging purpose draws sonar avoidance shape.
    /// </summary>
    [DefaultExecutionOrder(60)]
    public class AgentDebugSystem : System
    {
        public bool OnStartAddAgentDebug = true;

        void Start()
        {
            if (!OnStartAddAgentDebug)
                return;

            // Add AgentDebug to all Agents that do not have it
            foreach (var agent in Query<Agent>())
            {
                if (agent.gameObject.GetComponent<AgentDebug>() != null)
                    continue;

                agent.gameObject.AddComponent<AgentDebug>();
            }

            // Add AgentDebug to all Agents that do not have it
            foreach (var agent in Query<BoidsAgent>())
            {
                if (agent.gameObject.GetComponent<AgentDebug>() != null)
                    continue;

                agent.gameObject.AddComponent<AgentDebug>();
            }
        }

        void Update()
        {
            // Remove if agents are not moving
            foreach (var agentDebug in Query<AgentDebug>())
            {
                if (agentDebug.TryGetComponent(out Agent agent))
                {
                    if (agent.IsStopped || agent.Speed == 0)
                    {
                        if (agentDebug.Vision.IsCreated)
                            agentDebug.Vision.Dispose();
                    }
                }

                if (agentDebug.TryGetComponent(out BoidsAgent boidsAgent))
                {
                    if (!boidsAgent.CanMove())
                    {
                        if (agentDebug.Vision.IsCreated)
                            agentDebug.Vision.Dispose();
                    }
                }
            }
        }

        void OnDrawGizmos()
        {
            if (!isActiveAndEnabled)
                return;

            foreach (var agentDebug in Query<AgentDebug>())
            {
                if (agentDebug.Vision.IsCreated)
                {
                    agentDebug.Vision.DrawSonar();
                    agentDebug.Vision.DrawClosestDirection();
                }
            }
        }
    }
}
