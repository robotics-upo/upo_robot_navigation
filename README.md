# upo_robot_navigation
Metapackage of ROS that contains the packages involved in the navigation system of the UPO robotics lab.

The system employs the A* global planner of ROS to plan a first global path to the goal. Then, a RRT* planner with social cost functions for navigation is used to plan a path in a local area of the robot taking the intersection points between the local area and the A* plan as a sub-goals. 

This metapackage contains the following packages:
 
* **upo_navigation**. This is the main package which uses the A* global planner of ROS and the RRT planners to perform a social robot navigation.

* **upo_rrt_planners**. C++ library of some RRT planners and a wrapper to be used in ROS.

* **navigation_features**. Package that calculates the feature functions employed by the cost function of the RRT*.

* **upo_local_planner**. A local controller that sends the velocity commands to the robot in order to follow a path. It is based on a pure pursuit controller that has been extended to perform collision detection based on DWA. The collision detection is perfomed based on laser range subscription (it does not use the ROS local costmap). It follows the structure of the standard base local planner of ROS.

* **simple_local_planner**. A local controller that sends the velocity commands to the robot in order to follow a path. It is based on a pure pursuit controller and follows the structure of the standard base local planner of ROS.

* **upo_msgs**. Custom messages.

* **upo_launchers**. Contains the maps, configuration files and launch files necessary to launch a simulation of the navigation system in two environments; a simple room of the UPO lab and a elderly center with different rooms.

* **upo_navigation_macro_actions**. A set of navigation macro-actions have been implemented by using the *actionlib* of ROS. This way, the navigation system is employed to perform different actions as reaching a simple goal, approaching a moving person, or walk side-by-side. A launch file to test the macro-actions is inside the launch directory.

* **upo_decision_making**. A set of Python scripts that contains the finite state machine for the navigation macro-actions and the interaction with the corresponding actions defined through the ROS actionlib library.

* **gmm_sampling**. A library and its ROS wrapper that loads a set of GMMs defined through text files. Then these GMMs can be consulted trough ROS services to draw samples and costs from them.
Currently, the GMMs included are employed in the macro-action "approaching to a person target".

* **upo_social_layer**. This package contains a layer that can be used in the costmap_2d of ROS as a plugin. The layer is built by using the social functions of the navigation features package related to people. This way, a social cost related to the distance and orientation to people can be included in a costmap for robot navigation.

* **assisted_steering** ROS node used in the TERESA project. It checks the velocity commands that the user sends to the robot. If a possible collision is detected the node tries to find a similar valid command or stops the robot otherwise. 

## Dependences

* The instalation of the ROS packages **navigation** and **AMCL** are required.

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.


