
#include <upo_rrt_planners/ros/RRT_ros_wrapper.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ros_wrapper_node");
  tf::TransformListener tf(ros::Duration(10));

  upo_RRT_ros::RRT_ros_wrapper rrt_wrapper(&tf);

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}
