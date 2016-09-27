
#include <ros/ros.h>
#include <assisted_steering/AssistedSteering.h>

int main(int argc, char** argv)
{
 	ros::init(argc, argv, "Assisted Steering node");
 
	tf::TransformListener tf(ros::Duration(10));

	assisted_steering::AssistedSteering steering(&tf);
	
 	ros::spin();

 	return 0;
}
