# gmm_sampling
A library and its ROS wrapper that loads a set of GMMs defined through text files. Then these GMMs can be consulted trough ROS services to draw samples and costs from them.
Currently, the GMMs included are employed in the macro-action "approaching to a person target".

## Parameters

	* **gmm_tasks_directory**. Route to the folder where the GMMs are defined for different tasks.

## Services

	* **GetApproachGMMSamples**. It accepts as an input the value of the current relative orientation of the robot according to the target person and the number of samples required. It returns the number of samples indicated obtained from the corresponding GMM according to the orientation provided. 
	* **GetApproachGMMProbs**. It accepts as an input a list of points in the coordinates frame of the target person (x and y coordinates) and returns a vector with the values of the density functions in those points. 

The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
