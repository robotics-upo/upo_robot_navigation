# simple_local_planner 
A local controller that sends the velocity commands to the robot in order to follow a path. It is based on a pure pursuit controller and follows the structure of the standard base local planner of ROS.

## Parameters

* **Robot Configuration Parameters**
	- max_trans_acc. Maximum acceleration in translation (m/s^2).
  	- max_rot_acc. Maximum acceleration in rotation (rad/s^2).
  	- max_trans_vel. Maximum linear velocity (m/s).
  	- min_trans_vel. Minimum linear velocity (m/s).
  	- max_rot_vel. Maximum angular velocity (rad/s).
  	- min_rot_vel. Minimum angular velocity (rad/s).
  	- min_in_place_rot_vel. Angular velocity of rotations in place (rad/s).

* **Goal Tolerance Parameters**
	- yaw_goal_tolerance. Tolerance in angular distance (rad) to consider that the goal has been reached.
	- xy_goal_tolerance. Tolerance in euclidean distance (m) to consider that the goal has been reached.
	- wp_tolerance. Distance (m) from the robot to look for the point of the global plan to follow.
  
* **Forward Simulation Parameters**
	- sim_time. Time (seconds) to expand the robot movement and check for collisions. (default: 0.5).
	- sim_granularity. Resolution in meters to split the expanded trayectory and check for collisions (Default: 0.025).
	- angular_sim_granularity. Resolution in radians to split the expanded angular movement and check for collisions (Default: 0.025).
	- controller_freq. Frequency of execution in Hz (Default: 15.0).


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
