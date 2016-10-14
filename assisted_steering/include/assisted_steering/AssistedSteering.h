
#ifndef ASSISTED_STEERING_H_
#define ASSISTED_STEERING_H_

#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <assisted_steering/odometry_helper_ros.h>

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <assisted_steering/AssistedSteeringConfig.h>

#include <boost/thread/mutex.hpp> //Mutex



namespace assisted_steering {

	class AssistedSteering {

		public:

			AssistedSteering(tf::TransformListener* tf);
			~AssistedSteering();

			void setup();

			void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
			void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);

			bool checkCommand(geometry_msgs::Twist* twist);
			bool findValidCmd(geometry_msgs::Twist* twist);

			bool inCollision(float x, float y, sensor_msgs::LaserScan* scan);

			void sendVelocityZero();

			void saturateVelocities(geometry_msgs::Twist* twist);

		private:

			inline float normalizeAngle(float val, float min, float max){
				float norm = 0.0;
				if (val >= min)
					norm = min + fmod((val - min), (max-min));
				else
					norm = max - fmod((min - val), (max-min));
						
				return norm;
			}


			tf::TransformListener* 			tf_;

			bool							isActive_;

			std::string 					laser_topic_;

			std::string 					odom_topic_;

			std::string						cmdvel_topic_;

			std::string						new_cmdvel_topic_;

			OdometryHelperRos*				odom_helper_;

			ros::Subscriber 				laser_sub_;
			ros::Subscriber 				cmdvel_sub_;

			ros::Publisher					out_cmdvel_pub_;

			sensor_msgs::LaserScan			laser_scan_;
			boost::mutex 					laser_mutex_;
			ros::Time						scan_time_;		
			geometry_msgs::Twist			twist_;
			boost::mutex 					twist_mutex_;


			double 							max_lin_vel_;
			double							max_ang_vel_;
			double							max_lin_acc_;
			double							max_ang_acc_;
			double							time_step_;
			double 							robot_radius_;
			double							granularity_;
			double							ang_vel_inc_;
			double							lin_vel_inc_;

			float 							max_lv_var_;
			float 							max_av_var_;

			tf::Stamped<tf::Pose> 			robot_vel_;

			//Dynamic reconfigure
			boost::recursive_mutex 		configuration_mutex_;
			dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig> *dsrv_;
			void reconfigureCB(assisted_steering::AssistedSteeringConfig &config, uint32_t level);

	};

}
#endif
