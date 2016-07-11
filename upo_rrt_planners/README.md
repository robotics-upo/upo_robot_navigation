# upo_rrt_planners
C++ library of some RRT planners and API for its use with ROS

This is a catkin package of ROS that contains two libraries:

* *upo_rrt_planners*: C++ library that contains the following RRT planners:

	- *simple RRT*: RRT planner in x,y coordinates without reasoning about kinodynamic constraints.
	- *simple RRTstar*: RRT* planner in x,y coordinates without reasoning about kinodynamic constraints.
	- *RRT*: RRT planner with kinodynamic constrains.
	- *RRTstar*: RRT* planner with kinodynamic constrains.


* *upo_rrt_planners_ros*: C++ library that wraps the previous library in order to be used in ROS. 


## Parameters

* rrt_planner_type. RRT planner to use.
	1. RRT. x,y state space (no dynamics).
	2. RRT*. x,y state space (no dynamics).
	3. Kinodynamic RRT (x, y, yaw state space).
	4. Kinodynamic RRT* (x, y, yaw state space).
	5. Simplified Kinodynamic RRT*. RRT* that does not perform the tree rewiring.

* rrt_solve_time. Time in seconds that the RRT* planner is allowed to plan a path. Maximum time to find a path in the case of the RRT.

* rrt_goal_bias. probability bias to sample the goal.

* rrt_max_insertion_dist. Maximum distance (m) to insert a new node from the nearest node of the sample.

* rrt_goal_xy_tol. Tolerance (m) to consider that the goal has been reached in the x,y space.

* rrt_goal_th_tol. Tolerance (radians) to consider that the goal has been reached in the angular space.

* rrt_interpolate_path_dist. Distance (m) between nodes to perform an interpolation of the resulting path. Use value 0 for no interpolation.


#### TODO
- [ ] Replace regular pointers by boost smart pointers. 
- [ ] Add new RRT algorithms.
- [ ] Add a plugin to be used as a global planner in the move_base architecture of ROS.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
