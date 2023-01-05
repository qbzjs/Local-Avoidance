using UnityEngine;
using Unity.Mathematics;

namespace ProjectDawn.LocalAvoidance.Demo
{
    [DefaultExecutionOrder(60)]
    public class PlayAnimationSystem : System
    {
        public float AnimationSpeed = 0.4f;
        void Update()
        {
            foreach (var playAnimation in Query<PlayAnimation>())
            {
                if (!playAnimation.Target)
                    continue;

                float speed;
                if (playAnimation.TryGetComponent(out BoidsAgent boidsAgent))
                {
                    speed = math.length(boidsAgent.Velocity);
                }
                else if (playAnimation.TryGetComponent(out Agent agent))
                {
                    speed = math.length(agent.Velocity);
                }
                else
                {
                    continue;
                }

                playAnimation.Target.SetFloat("Speed", speed);
                playAnimation.Target.speed = speed > 0.3f ? speed * AnimationSpeed : 1f;
            }
        }
    }
}
