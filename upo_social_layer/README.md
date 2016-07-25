# upo_social_layer
This package contains a layer that can be used in the costmap_2d of ROS as a plugin. The layer is built by using the social functions of the navigation features package related to people. This way, a social cost related to the distance and orientation to people can be included in a costmap for robot navigation.

The layer can be included as a plugin in the local and/or global costmap of ROS. 

## Dependences

* Package **navigation_features**.

## Parameters

* **size_x**. Size of the layer area. Distance in meters from the center of the robot in the X axis. Size [-size_x, size_x].
* **size_y**. Size of the layer area. Distance in meters from the center of the robot in the Y axis. Size [-size_y, size_y].
* **all_features**. Boolean parameter to indicate the features functions to apply.
	- _True_. Obtain the cost of each cell of the layer as a combination of all the feature functions calculated in the navigation features library (people around the robot, distance to the goal and distance to the closest obstacle).
	- _False_. Obtain the cost of each cell of the layer by using the feature function related to the people around the robot. 

The directory _launch_ contains some examples of use of the layer.

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.


