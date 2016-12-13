/*********************************************************************
*
*
* Author: Noé Pérez Higueras
		  Fernando Caballero
*********************************************************************/
#ifndef UPO_NAV_ACTION_H_
#define UPO_NAV_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
//#include "move_base/MoveBaseConfig.h"

// threads for RRT planning
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> 

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

//Service 
#include <upo_navigation/FeatureCounts.h>
#include <upo_navigation/SetWeights.h>

//****New RRT package
#include <upo_rrt_planners/ros/RRT_ros_wrapper.h>


namespace upo_nav {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum NavigationState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class UpoNavigation
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class UpoNavigation {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      UpoNavigation(tf::TransformListener& tf,  bool macro = false);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~UpoNavigation();

      /**
       * @brief  Performs a control cycle
       * @return 1 if processing of the goal is done, 0 if the goal still haven't been reached
	   *  -1 if a valid control can not be found.
       */
      int executeCycle();
      //bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);
      
      
      //----Noé-----------
      void setFeaturesWeights(std::vector<float> w);
    
	  
	  bool clearCostmaps();
	  void updateCostmaps();
	  bool areCostmapsCurrent();
	  bool clearLocalCostmap();
	  void updateLocalCostmap();
	  
	  bool makeRRTPlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	  

	  std::vector<float> get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path);
	  std::vector<float> get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people);
	  
	  float get_path_cost(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people);
	  float get_path_cost();


	  //methods for navigation. They are invoked from an external action server to perform different macro-actions
      bool executeNavigation(geometry_msgs::PoseStamped goal); 
      bool executeNavToPerson(geometry_msgs::PoseStamped goal, int id); 
      int pathFollow(geometry_msgs::PoseStamped& new_pose);
      int executeControllerCycle();
      geometry_msgs::PoseStamped getRobotGlobalPosition();
      void stopRRTPlanning();

	  //---------------
	  
	  /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();
	  

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
      
      
      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planRRTService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);


	  /**
	   * @brief A service call that return the feature counts of a path
       * @param req the path request
	   * @param res the feature counts response
 	   * @return True if feature counts calculation succeeded, false otherwise
	   */
	  bool getPathFeatures(upo_navigation::FeatureCounts::Request  &req, upo_navigation::FeatureCounts::Response &res);
      
      /**
	   * @brief A service call that return the set the values of the weights for path cost function calculation
       * @param req the weights array to set
	   * @param res empty
 	   * @return True if weight set succeeded, false otherwise
	   */
	  bool setWeightsRRT(upo_navigation::SetWeights::Request  &req, upo_navigation::SetWeights::Response &res);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      //bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      //void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      //void planThread();

	  void rrt_thread(void);

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
      

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

	  geometry_msgs::PoseStamped goalToLocalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
	  
	  float normalizeAngle(float val, float min, float max) {
			float norm = 0.0;
			if (val >= min)
				norm = min + fmod((val - min), (max-min));
			else
				norm = max - fmod((min - val), (max-min));
					
			return norm;
	  }
	  
	  
	  void setPathToBias(std::vector<geometry_msgs::PoseStamped> path_to_follow) {
				rrt_planner_->setBiasingPath(&path_to_follow);
			}
	

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
     // void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      //std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      //unsigned int recovery_index_;

      tf::Stamped<tf::Pose> global_pose_;
      geometry_msgs::PoseStamped global_goal_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      //MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      //pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      //std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* local_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::mutex planner_mutex_;
      boost::condition_variable planner_cond_;
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


	  //set up the RRT* planner thread
	  bool run_rrt_;
	  //bool rrt_sleep_;
	  boost::thread rrt_thread_;
      bool thread_active_;
	  int thread_sleep_msecs_;
      boost::mutex rrt_mutex_;
	  geometry_msgs::PoseStamped rrt_goal_;
	  //RRT plan service
	  ros::ServiceServer make_rrt_plan_srv_;
	  //Feature counts service
	  ros::ServiceServer features_srv_;
	  //Set weights service
	  ros::ServiceServer weights_srv_;


	  //upo_nav::OdometryHelperRos* odom_helper_;
	  std::vector<geometry_msgs::Point> footprint_spec_;
	  //MapGridVisualizer map_viz_;

	  //RRT planner
	  upo_RRT_ros::RRT_ros_wrapper* rrt_planner_;


      //boost::recursive_mutex configuration_mutex_;
      //dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      //void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      //move_base::MoveBaseConfig last_config_;
      //move_base::MoveBaseConfig default_config_;
      //bool setup_, p_freq_change_, c_freq_change_;

      bool new_global_plan_;
	  bool new_rrt_plan_;
  };
};
#endif

