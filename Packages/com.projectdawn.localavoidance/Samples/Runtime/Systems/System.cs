using UnityEngine;
using UnityEngine.Profiling;

namespace ProjectDawn.LocalAvoidance.Demo
{
    /// <summary>
    /// Simple abstract class for systems.
    /// </summary>
    public abstract class System : MonoBehaviour
    {
        /// <summary>
        /// Finds all game objects that has specified class.
        /// </summary>
        protected T[] Query<T>() where T : MonoBehaviour
        {
            Profiler.BeginSample("Query.FindObjectsOfType");
            // GameObject.FindObjectsOfType is quite heavy on performance as it allocates garbage and do not cache
            // In normal game it is recommended to replace this method with something more optimized
            // Few of possible solutions:
            // - with MonoBehaviour.OnEnable/OnDisable track all behaviours in NativeList and filter them here
            // - cache GameObject.FindObjectsOfType calls between systems
            var behaviours = GameObject.FindObjectsOfType<T>();
            Profiler.EndSample();
            return behaviours;
        }
    }
}
