# simple_local_planner 
A controller based on pure pursuit path tracking has been extended to command velocities to the differential robot so as to follow the path smoothly.
This path tracker runs at a frequency of 15Hz and has been extended to perform a collision detection checking similar to the Dynamic Windows Approach algorithm. If the forward projection of the robot movement given by the control law is detected as a possible collision, a valid command is tried to be found by sampling small variations of the given angular velocity. If a possible collision is still detected, a rotation in place in the correct direction is performed in order to avoid very close obstacles. Moreover, a linear decrease of the velocities when approaching the goal has been also added.

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
