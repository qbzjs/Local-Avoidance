using UnityEngine;

namespace ProjectDawn.LocalAvoidance
{
    [RequireComponent(typeof(Agent))]
    public class FollowTransform : MonoBehaviour
    {
        public Transform Target;
    }
}
