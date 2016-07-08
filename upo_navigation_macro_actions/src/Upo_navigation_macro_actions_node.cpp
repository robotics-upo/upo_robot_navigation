#include <ros/ros.h>
#include <upo_navigation_macro_actions/Upo_navigation_macro_actions.h>

int main(int argc, char** argv)
{
 	ros::init(argc, argv, "navigation_macro_actions");
 
	ROS_INFO("Starting navigation_macro_actions...");
	tf::TransformListener tf(ros::Duration(10));

	upo_nav::UpoNavigation UpoNav(tf, true);

 	Upo_navigation_macro_actions *macro = new Upo_navigation_macro_actions(tf, &UpoNav);
	
 	ros::spin();

 	return 0;
}
