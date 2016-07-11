# upo_robot_navigation
Metapackage of ROS that contains the packages involved in the navigation system of the UPO robotics lab.

The system employs the A* global planner of ROS to plan a first global path to the goal. Then, a RRT* planner with social cost functions for navigation is used to plan a path in a local area of the robot taking the intersection points between the local area and the A* plan as a sub-goals. 

This metapackage contains the following packages:
 
* **upo_navigation**. This is the main package which uses the global planner of ROS and the RRT planners to perform a social robot navigation.

* **upo_rrt_planners**. C++ library of some RRT planners and a wrapper to be used in ROS.

* **navigation_features**. Package that calculates the feature functions employed by the cost function of the RRT*.

* **upo_msgs**. Custom messages.

* **upo_launchers**. Contains the maps, configuration files and launch files necessary to launch a simulation of the navigation system in two environments; a simple room of the UPO lab and a elderly center with different rooms.

* **upo_navigation_macro_actions**. A set of navigation macro-actions have been also implemented. This way, the navigation system is employed to perform different actions as reaching a simple goal or approaching a moving person. A launch file to test the macro-actions is inside the launch directory.


