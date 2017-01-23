#ifndef COLLISION_DETECTION_H_
#define COLLISION_DETECTION_H_

#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <upo_local_planner/odometry_helper_ros.h>

//Dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <upo_local_planner/AssistedSteeringConfig.h>

#include <boost/thread/mutex.hpp> //Mutex

#include <sys/time.h>



namespace upo_local_planner {

	class CollisionDetection {

		public:

			CollisionDetection();
			CollisionDetection(tf::TransformListener* tf,
						upo_local_planner::OdometryHelperRos* oh,
						double max_lv, double max_av,
						double lin_acc, double ang_acc,
						double sim_t, double r_radius,
						double granularity);


			~CollisionDetection();

			void setup();

			void laser1Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
			void laser2Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
			//void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg);

			bool checkCommand(geometry_msgs::Twist* twist);
			//bool findValidCmd(geometry_msgs::Twist* twist);

			/**
			* Check if the indicated point (robot frame) is a valid position
			* according to the robot radius and the laserscan
			* parameters:
			* x, x position in robot frame
			* y, y position in robot frame
			* scan, laserscan
			* return true if a possible collision is detected, false otherwise
			*/
			bool inCollision(float x, float y, sensor_msgs::LaserScan* scan);
			//bool inCollision(float x, float y, std::vector<geometry_msgs::Point>* scanpoints);

			//bool inCollision2(float x, float y, sensor_msgs::LaserScan* scan); 
			bool inCollision2(float x, float y, std::vector<geometry_msgs::Point>* scanpoints);

			bool collision(float x, float y);

			//bool collision2(float x, float y);

			/**
		   * @brief  Generate and check a single trajectory
		   * @param cvx The current x velocity of the robot  
		   * @param cvy The current y velocity of the robot  
		   * @param cvth The current angular velocity of the robot
		   * @param tvx The x velocity used to seed the trajectory
		   * @param tvy The y velocity used to seed the trajectory
		   * @param tvth The theta velocity used to seed the trajectory
		   * @param px will be filled with the final x point of the trajectory (robot frame)
		   * @param py will be filled with the final y point of the trajectory (robot frame)
		   * @param pth will be filled with the final th point of the trajectory (robot frame)
		   * @return True if the trajectory is legal, false otherwise
		   */
			bool checkTraj(double cvx, double cvy, double cvth, double tvx, double tvy, double tvth, double& px, double& py, double &pth);

			std::vector<geometry_msgs::Point> laser_polar2euclidean(sensor_msgs::LaserScan* scan);

			void updateSensorReadings();

			//void sendVelocityZero();

			void saturateVelocities(geometry_msgs::Twist* twist);

		private:

			 
			  float inline normalizeAngle(float val, float min, float max) {
					float norm = 0.0;
					if (val >= min)
						norm = min + fmod((val - min), (max-min));
					else
						norm = max - fmod((min - val), (max-min));
					
					return norm;
				}
			  
			  

			  /**
			   * @brief  Compute x position based on velocity
			   * @param  xi The current x position
			   * @param  vx The current x velocity
			   * @param  vy The current y velocity
			   * @param  theta The current orientation
			   * @param  dt The timestep to take
			   * @return The new x position 
			   */
			  inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
				return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
			  }

			  /**
			   * @brief  Compute y position based on velocity
			   * @param  yi The current y position
			   * @param  vx The current x velocity
			   * @param  vy The current y velocity
			   * @param  theta The current orientation
			   * @param  dt The timestep to take
			   * @return The new y position 
			   */
			  inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
				return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
			  }

			  /**
			   * @brief  Compute orientation based on velocity
			   * @param  thetai The current orientation
			   * @param  vth The current theta velocity
			   * @param  dt The timestep to take
			   * @return The new orientation
			   */
			  inline double computeNewThetaPosition(double thetai, double vth, double dt){
				return thetai + vth * dt;
			  }

			  //compute velocity based on acceleration
			  /**
			   * @brief  Compute velocity based on acceleration
			   * @param vg The desired velocity, what we're accelerating up to 
			   * @param vi The current velocity
			   * @param a_max An acceleration limit
			   * @param  dt The timestep to take
			   * @return The new velocity
			   */
			  inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
				if((vg - vi) >= 0) {
				  return std::min(vg, vi + a_max * dt);
				}
				return std::max(vg, vi - a_max * dt);
			  }


			tf::TransformListener* 			tf_;

			//bool							isActive_;

			std::string 					laser1_topic_;
			std::string 					laser2_topic_;

			int 							n_lasers_;

			std::string 					odom_topic_;

			//std::string						cmdvel_topic_;

			//std::string						new_cmdvel_topic_;

			OdometryHelperRos*				odom_helper_;

			ros::Subscriber 				laser1_sub_;
			ros::Subscriber 				laser2_sub_;
			//ros::Subscriber 				cmdvel_sub_;

			//ros::Publisher					out_cmdvel_pub_;

			sensor_msgs::LaserScan			laser1_scan_;
			std::vector<geometry_msgs::Point> laser1_euclidean_;
			sensor_msgs::LaserScan			laser2_scan_;
			std::vector<geometry_msgs::Point> laser2_euclidean_;
			sensor_msgs::LaserScan			laser1_scan_copy_;
			sensor_msgs::LaserScan			laser2_scan_copy_;
			boost::mutex 					laser1_mutex_;
			boost::mutex 					laser2_mutex_;
			ros::Time						scan1_time_;	
			ros::Time						scan2_time_;	
			//geometry_msgs::Twist			twist_;
			//boost::mutex 					twist_mutex_;


			double 							max_lin_vel_;
			double							max_ang_vel_;
			double							max_lin_acc_;
			double							max_ang_acc_;
			double							sim_time_;
			double 							robot_radius_;
			double 							robot_radius_aug_;
			double							granularity_;
			double							laser_uncertainty_;

			float 							max_lv_var_;
			float 							max_av_var_;

			tf::Stamped<tf::Pose> 			robot_vel_;

			//Dynamic reconfigure
			//boost::recursive_mutex 		configuration_mutex_;
			//dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig> *dsrv_;
			//void reconfigureCB(assisted_steering::AssistedSteeringConfig &config, uint32_t level);

	};

}
#endif
