

#ifndef UPO_RRT_CHECKER_
#define UPO_RRT_CHECKER_


//upo RRT library
#include <upo_rrt_planners/StateChecker.h>

//C++
#include <vector>
#include <list>
#include <cmath>
#include <math.h> 
#include <iostream>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atof */
#include <exception>      // std::exception
#include <time.h>       /* time */

//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <upo_msgs/PersonPoseUPO.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/PoseStamped.h>

//Features for navigation cost functions
#include <navigation_features/nav_features.h>

//Mutex
#include <mutex> 


namespace upo_RRT_ros
{

	class ValidityChecker : public upo_RRT::StateChecker
	{
		public:
		
		ValidityChecker(bool use_fc_costmap, tf::TransformListener* tf, std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, float res, unsigned int dimensions, int distType); 

		virtual ~ValidityChecker();
		
		bool isValid(upo_RRT::State* s) const;
		
		
		//Distance function between two states
		float distance(upo_RRT::State* s1, upo_RRT::State* s2) const;
		
		float getCost(upo_RRT::State* s);
		
		std::vector<float> getFeatures(upo_RRT::State* s);
		
		void setGoal(upo_RRT::State* g) { 
			geometry_msgs::PoseStamped goal;
			goal.header.frame_id = "base_link";
			goal.header.stamp = ros::Time();
			goal.pose.position.x = g->getX();
			goal.pose.position.y = g->getY();
			goal.pose.position.z = 0.0;
			goal.pose.orientation = tf::createQuaternionMsgFromYaw(g->getYaw());
			navfeatures_->setGoal(goal);
		}
		
		//Implemented for learning purposes
		void setPeople(upo_msgs::PersonPoseArrayUPO p);
		
		
		geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime);
		
		bool isQuaternionValid(const geometry_msgs::Quaternion q);
		
		
		void setWeights(std::vector<float> we);
		
		void setUseLossFunc(bool l, std::vector<geometry_msgs::PoseStamped> path) {
			navfeatures_->set_use_loss_func(l);
			navfeatures_->set_demo_path(path);
		}
			
		//Pre-computations needed just before starting planning
		void preplanning_computations();
		
		void setInitialTime(ros::Time t) {
			time_ = t;
		}
		
		
		private:
		
		features::NavFeatures* 				navfeatures_;
		
		//const costmap_2d::Costmap2D* 		loc_costmap_;
		//const costmap_2d::Costmap2D* 		glo_costmap_;
		//bool 								use_global_costmap_;
		tf::TransformListener*				tf_;
		
		unsigned int 						dimensions_;
		int 								distanceType_;
		bool								get_cost_from_costmap_;
		
		ros::Time							time_;
		

	};

}
#endif 

