/*
 * 
 */

#include <upo_navigation/upo_navigation.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "upo_navigation_node");
  tf::TransformListener tf(ros::Duration(10));

  upo_nav::UpoNavigation UpoNav( tf );

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}
