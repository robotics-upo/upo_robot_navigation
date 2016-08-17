#ifndef UVA_FEATURES_
#define UVA_FEATURES_


#include <iostream>
#include <string>
#include <math.h>
#include <vector>

// ROS 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <upo_msgs/PersonPoseUPO.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/package.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/GetMap.h>
//-- PCL
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Boost
#include <boost/thread.hpp>  /* Mutex */

using namespace std;

namespace uva_cost_functions {

	class UvaFeatures {

		public:
		
			UvaFeatures(tf::TransformListener* tf);


			enum dist_type{LINEAR_INC,LOG_INC,EXP_INC,INVERSE_DEC,LOG_DEC,EXP_DEC};


			void setGoal(geometry_msgs::PoseStamped g) {
				goal_ = g;
			}

			void setDistanceTransform(cv::Mat dt);

			void setPeople(upo_msgs::PersonPoseArrayUPO p);

			float getCost(geometry_msgs::PoseStamped* s);
			std::vector<float> getFeatures(geometry_msgs::PoseStamped* s);




		private:

			ros::NodeHandle 			nh_;			

			tf::TransformListener* 		listener_;
			
			geometry_msgs::PoseStamped 	goal_;			
			
			upo_msgs::PersonPoseArrayUPO people_;
			boost::mutex 				people_mutex_;

			cv::Mat						dt_;
			boost::mutex 				dt_mutex_;


	};

}
#endif
