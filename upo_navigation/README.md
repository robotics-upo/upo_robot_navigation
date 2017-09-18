# upo_navigation
This is the main navigation package.
It uses the A* global planner of ROS to plan a first path to the goal. This path is employed to calculate sub-goals in a local area of the robot where the desired RRT planner is used to plan a social path in a dynamic environment.

## Parameters

	- rrt_planner_type. Indicates the type of planning used.
		Value '1' indicates a regular navigation. If the planner used is RRT*, then the planner cost function will be considered as a weighted linear combination of features (package navigation_features will be used for features calculation). 
		Value '2' indicates a RRT* navigation using path planning predicition with Fully Convolutional Network. The library RRT_ros_wrapper3 and the package path_prediction will be employed. Only valid for RRT* planner.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
