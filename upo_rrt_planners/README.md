# upo_rrt_planners
C++ library of some RRT planners and API for its use with ROS

This is a catkin package of ROS that contains two libraries:

* *upo_rrt_planners*: C++ library that contains the following RRT planners:

	- *simple RRT*: RRT planner in x,y coordinates without reasoning about kinodynamic constraints.
	- *simple RRTstar*: RRT* planner in x,y coordinates without reasoning about kinodynamic constraints.
	- *RRT*: RRT planner with kinodynamic constrains.
	- *RRTstar*: RRT* planner with kinodynamic constrains.


* *upo_rrt_planners_ros*: C++ library that wraps the previous library in order to be used in ROS. 


#### TODO
- [ ] Replace regular pointers by boost smart pointers. 
- [ ] Add new RRT algorithms.
- [ ] Add a plugin to be used as a global planner in the move_base architecture of ROS.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
