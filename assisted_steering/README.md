# assisted steering  
ROS node used in the TERESA project. It checks the velocity commands that the user sends to the robot. If a possible collision is detected the node tries to find a similar valid command or stops the robot otherwise.
This node is orientated to be used with circular robot footprints and differential kinematics.
The collision checking is based on a laser rangefinder sensor.


## Parameters

* **ROS topics**
	- laser_topic. Name of the topic where the readings of the laser sensor are being published (message type: sensor_msgs/LaserScan). The node will subscribe to this topic.
	- odom_topic. Name of the topic where the robot odometry is being published (message type: nav_msgs/Odometry). The node will subscribe to this topic.
	- cmdvel_topic. Name of the topic where the user's velocity commands (message type: geometry_msgs/Twist) are being published. The will subscribe to this topic.
	- new_cmdvel_topic. Name of the topic where the new valid velocity commands will be published. The robot should be subscribed to this topic.

* **Robot Configuration Parameters**
	- robot_radius. Radius of the robot footprint (m).
	- max_lin_vel. Maximum linear velocity (m/s).
  	- min_ang_vel. Minimum linear velocity (m/s).
	- max_lin_acc. Maximum acceleration in translation (m/s^2).
  	- max_ang_acc. Maximum acceleration in rotation (rad/s^2).
  
* **Forward Simulation Parameters**
	- time_step. Time (seconds) to expand the robot movement and check for collisions. (default: 1.0).
	- granularity. Resolution in meters to split the expanded trayectory and check for collisions (Default: 0.025).
	


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
