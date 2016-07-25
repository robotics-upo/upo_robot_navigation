# upo_navigation
This is the main navigation package.
It uses the A* global planner of ROS to plan a first path to the goal. This path is employed to calculate sub-goals in a local area of the robot where the desired RRT planner is used to plan a social path in a dynamic environment.

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
