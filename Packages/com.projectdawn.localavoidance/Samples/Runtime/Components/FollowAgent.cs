using UnityEngine;

namespace ProjectDawn.LocalAvoidance
{
    [RequireComponent(typeof(Agent))]
    public class FollowAgent : MonoBehaviour
    {
        public Agent TargetAgent;
    }
}
