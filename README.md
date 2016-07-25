# upo_robot_navigation
Metapackage of ROS that contains the packages involved in the navigation system of the UPO robotics lab.

The system employs the A* global planner of ROS to plan a first global path to the goal. Then, a RRT* planner with social cost functions for navigation is used to plan a path in a local area of the robot taking the intersection points between the local area and the A* plan as a sub-goals. 

This metapackage contains the following packages:
 
* **upo_navigation**. This is the main package which uses the A* global planner of ROS and the RRT planners to perform a social robot navigation.

* **upo_rrt_planners**. C++ library of some RRT planners and a wrapper to be used in ROS.

* **navigation_features**. Package that calculates the feature functions employed by the cost function of the RRT*.

* **simple_local_planner**. A local controller that sends the velocity commands to the robot in order to follow a path. It is based on a pure pursuit controller and follows the structure of the standard base local planner of ROS.

* **upo_msgs**. Custom messages.

* **upo_launchers**. Contains the maps, configuration files and launch files necessary to launch a simulation of the navigation system in two environments; a simple room of the UPO lab and a elderly center with different rooms.

* **upo_navigation_macro_actions**. A set of navigation macro-actions have been also implemented. This way, the navigation system is employed to perform different actions as reaching a simple goal, approaching a moving person, or walk side-by-side. A launch file to test the macro-actions is inside the launch directory.

* **upo_social_layer**. This package contains a layer that can be used in the costmap_2d of ROS as a plugin. The layer is built by using the social functions of the navigation features package related to people. This way, a social cost related to the distance and orientation to people can be included in a costmap for robot navigation.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.


