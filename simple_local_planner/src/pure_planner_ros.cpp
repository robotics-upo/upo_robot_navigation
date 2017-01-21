/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <simple_local_planner/pure_planner_ros.h>

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <simple_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>



//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simple_local_planner::PurePlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner {

  void PurePlannerROS::reconfigureCB(SimpleLocalPlannerConfig &config, uint32_t level) {
      /*if (setup_ && config.restore_defaults) {
        config = default_config_;
        //Avoid looping
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }*/
      tc_->reconfigure(config);
      //reached_goal_ = false;
  }

  PurePlannerROS::PurePlannerROS() :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {}

  PurePlannerROS::PurePlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }



  void PurePlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);


      tf_ = tf;
      costmap_ros_ = costmap_ros;
      /*rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;*/


      //initialize the copy of the costmap the controller will use
      costmap_ = costmap_ros_->getCostmap();

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
     
	  // Robot Configuration Parameters
	  private_nh.param("max_trans_vel", max_vel_x_, 0.6);
      private_nh.param("min_trans_vel", min_vel_x_, 0.1);
	  private_nh.param("max_rot_vel", max_vel_th_, 0.5);
      private_nh.param("min_rot_vel", min_vel_th_, 0.1);
	  private_nh.param("max_trans_acc", max_trans_acc_, 1.0);
      private_nh.param("max_rot_acc", max_rot_acc_, 1.0);
	  private_nh.param("min_in_place_rot_vel", min_in_place_vel_th_, 0.3);

	  //Goal tolerance parameters
	  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
	  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
  	  private_nh.param("wp_tolerance", wp_tolerance_, 0.5);
  	  
	  private_nh.param("sim_time", sim_time_, 1.0);
      private_nh.param("sim_granularity", sim_granularity_, 0.025);
      private_nh.param("angular_sim_granularity", angular_sim_granularity_, sim_granularity_);

	 
      //Assuming this planner is being run within the navigation stack, we can
      //just do an upward search for the frequency at which its being run. This
      //also allows the frequency to be overwritten locally.
      private_nh.param("controller_freq", controller_freq_, 15.0);

	  bool dwa;
	  private_nh.param("dwa", dwa, false);

      
      world_model_ = new CostmapModel(*costmap_);
      
      footprint_spec_ = costmap_ros_->getRobotFootprint();

      tc_ = new PurePlanner(*world_model_, *costmap_, footprint_spec_, controller_freq_,
          max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_, max_trans_acc_, max_rot_acc_, 
		  yaw_goal_tolerance_, xy_goal_tolerance_, wp_tolerance_, sim_time_, sim_granularity_, angular_sim_granularity_, dwa);

      //map_viz_.initialize(name, global_frame_, boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
      initialized_ = true;

		//BE CAREFUL, this will load the values of cfg params overwritting the read ones from the yaml file.
      dsrv_ = new dynamic_reconfigure::Server<SimpleLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<SimpleLocalPlannerConfig>::CallbackType cb = boost::bind(&PurePlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

    } else {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
  }

  

  PurePlannerROS::~PurePlannerROS() {
    //make sure to clean things up
    delete dsrv_;

    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }


  


  bool PurePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    //reset the goal flag
    reached_goal_ = false;

    return true;
  }




  bool PurePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;


    tf::Stamped<tf::Pose> global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }
    
    
    //TODO: Check here if we have already reached the goal
    


    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {  //TransformGlobalPlan belongs to goal_functions.cpp
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    //if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);

    //tf::Stamped<tf::Pose> drive_cmds;
    //drive_cmds.frame_id_ = robot_base_frame_;

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double goal_th = yaw;

    //check to see if we've reached the goal position
    /*if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs(angle) <= yaw_goal_tolerance_) {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      } else {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        tc_->updatePlan(transformed_plan);
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);

        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        odom_helper_.getOdom(base_odom);

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
            return false;
          }
        }
      }

      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }*/


    tc_->updatePlan(transformed_plan);

	geometry_msgs::Twist drive_cmds;
    //compute what trajectory to drive along
    //Trajectory path = tc_->findBestAction(global_pose, robot_vel, drive_cmds);
	bool ok = tc_->findBestAction(global_pose, robot_vel, drive_cmds);


    //map_viz_.publishCostCloud(costmap_);
    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel = drive_cmds;
    if(!ok) {
		ROS_DEBUG_NAMED("trajectory_planner_ros",
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
		publishPlan(transformed_plan, g_plan_pub_);
		return false;
	}

    //if we cannot move... tell someone
    /*if (path.cost_ < 0) {
      ROS_DEBUG_NAMED("trajectory_planner_ros",
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);
      return false;
    }

    ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      tf::Stamped<tf::Pose> p =
          tf::Stamped<tf::Pose>(tf::Pose(
              tf::createQuaternionFromYaw(p_th),
              tf::Point(p_x, p_y, 0.0)),
              ros::Time::now(),
              global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }*/

    //publish information to the visualizer
    publishPlan(transformed_plan, g_plan_pub_);
    //publishPlan(local_plan, l_plan_pub_);
    return true;
  }






  bool PurePlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    //return reached_goal_; 
    return tc_->isGoalReached();
  }




};
