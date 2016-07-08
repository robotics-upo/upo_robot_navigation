
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

			void timerCallback(const ros::TimerEvent& event);
			void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_in);

			boost::shared_ptr<cv::Mat> getDistanceTransform();
			cv::Mat getDistanceTransform2();
			cv::Mat getDistanceTransform3();
			nav_msgs::MapMetaData getMapMetadata();
			boost::shared_ptr<cv::Mat> getMapImage();


			enum dist_type{LINEAR_INC,LOG_INC,EXP_INC,INVERSE_DEC,LOG_DEC,EXP_DEC};


			void setGoal(geometry_msgs::PoseStamped g) {
				goal_ = g;
			}

			float getCost(geometry_msgs::PoseStamped* s);
			std::vector<float> getFeatures(geometry_msgs::PoseStamped* s);

			void setPeople(upo_msgs::PersonPoseArrayUPO p);

			boost::mutex people_mutex_;


		private:

			vector<int> worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata);
			void updateDT();
			ros::NodeHandle nh_;
			//ros::Subscriber sub_laser_;
			ros::Subscriber sub_pc_;
			cv::Mat map_image_;
			cv::Mat distance_transform_;
			nav_msgs::MapMetaData map_metadata_;
			tf::TransformListener* listener_;
			//tf::TransformListener listener2_;
			laser_geometry::LaserProjection projector_;
			sensor_msgs::PointCloud2 laser_cloud_;
			//boost::mutex timer_callbackMutex_;
			boost::mutex laser_callbackMutex_;
			boost::mutex distancetrans_Mutex_;

			geometry_msgs::PoseStamped goal_;			
			//ros::Timer timer_;
			upo_msgs::PersonPoseArrayUPO people_latest_;
			int people_paint_area_; // the amount of pixels to be painted over at the presence of people



	};

}
