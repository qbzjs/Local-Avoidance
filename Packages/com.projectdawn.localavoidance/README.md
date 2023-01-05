# Description

This package contains fast lightweight solution for local avoidance. It is developed with DOTS in mind, as result it takes advantage of Unity latest technology stack like SIMD mathematics, Jobs and Burst compiler.


Local avoidance allows agent to dynamicaly avoid obstacles, with high accuracy and speed, without the need of preprocessing techniques like navmesh or grid. At its core it uses custom unique solution, which is heavily inspired by Starcraft 2 game.


The whole logic is separated into single structure and no components are used. Thus allowing easy integration in any existing Global Pathing system, regardlessif it's OOD or DOD.

For this reason this package is recommended for people who have coding experience, but there are also component based demos ready for users who want to use it without coding.


# Key Behaviours

- Manages navigation out of concave obstacles
- Circles target
- Avoids head to head moving
- Accurately respects radius when avoiding static obstacles
- Accounts for obstacle's velocities

# Dependencies

- Tested with Unity 2019.4
- Package com.unity.mathematics
- Package com.unity.collections
- Package com.unity.burst

Note: This does not include Global Avoidance which is typical archieved using A* algorithm on data structures like NavMesh or Grid and targeted for static obstacles. Local avoidance typicaly used as complimentary system for it to target dynamic obstacle avoidance.