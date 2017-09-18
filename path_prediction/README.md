# Path prediction 
Package, written in Python, used in navigation with path planning prediction. The predicition is done by a Fully Convolutional Network trained in Keras. The package provide a ROS service that receives a flattened input grid, and return the respective path planning predicition. It's called from the library RRT_ros_wrapper3 (see package upo_rrt_planners).


## Dependences

* **Keras** is required.
* **Open_CV** is also needed.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
