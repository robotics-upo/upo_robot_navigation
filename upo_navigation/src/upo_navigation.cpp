/*********************************************************************
*
* Software License Agreement (BSD License)
* 
* 
* Authors: Noé Pérez Higueras
*          Fernando Caballero
*
*********************************************************************/
#include <upo_navigation/upo_navigation.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

//#define _UPO_NAV_DEBUG_

namespace upo_nav {

  UpoNavigation::UpoNavigation(tf::TransformListener& tf, bool macro) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
	bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
	blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    /*recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),*/
    planner_plan_(NULL), local_plan_(NULL),
    runPlanner_(false), 
	new_global_plan_(false), new_rrt_plan_(false), run_rrt_(false) {

	if(!macro)
		as_ = new MoveBaseActionServer(ros::NodeHandle(), "upo_navigation", boost::bind(&UpoNavigation::executeCb, this, _1), false);

    //odom_helper_ = new upo_nav::OdometryHelperRos("odom");

    //ros::NodeHandle private_nh("~/upo_navigation");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the upo_navigation node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
	private_nh.param("thread_sleep_msecs", thread_sleep_msecs_, 200);
	
	//printf("local_planner: %s, controller_freq: %.2f, thread_msec: %i\n", local_planner.c_str(), controller_frequency_, thread_sleep_msecs_);

	//thread_sleep_msecs_ = (1/controller_frequency_)*1000.0;

    //private_nh.param("planner_patience", planner_patience_, 5.0);
    //private_nh.param("controller_patience", controller_patience_, 15.0);

    //private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    //private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    //latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    local_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    //planner_thread_ = new boost::thread(boost::bind(&UpoNavigation::planThread, this));

    //for comanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 5);

    ros::NodeHandle action_nh("upo_navigation");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("upo_navigation_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&UpoNavigation::goalCB, this, _1));
    
    
    //RRT path points
    path_points_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_path", 1);
    

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, false);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!bgp_loader_.isClassAvailable(global_planner)){
        std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(global_planner == bgp_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                global_planner.c_str(), classes[i].c_str());
            global_planner = classes[i];
            break;
          }
        }
      }

      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }
	
	
	//local costmap
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();
	
	//--------------------------------------------------------------------------
    //create a semi-local RRT planner
	//rrt_planner_ = new RRTPlannerROS(std::string("RRTPlannerROS"), &tf_, planner_costmap_ros_, controller_costmap_ros_);
	//******new
	rrt_planner_ = new upo_RRT_ros::RRT_ros_wrapper(&tf_, planner_costmap_ros_, controller_costmap_ros_);
	//set up the rrt planner's thread
	run_rrt_ = false;
	//rrt_sleep_ = true;
	thread_active_ = true;
    rrt_thread_ = boost::thread(&UpoNavigation::rrt_thread, this);
						

	//create a controller to follow the RRT path
	//WorldModel* world_model_ = new CostmapModel(*(controller_costmap_ros_->getCostmap()));
	//footprint_spec_ = controller_costmap_ros_->getRobotFootprint(); 
	//pure_pursuit_ = new TrajectoryPlannerPP(*world_model_, *(controller_costmap_ros_->getCostmap()), footprint_spec_, controller_frequency_);

	//map_viz_.initialize("TrajectoryPlannerPP", controller_costmap_ros_->getGlobalFrameID(), boost::bind(&TrajectoryPlannerPP::getCellCosts, pure_pursuit_, _1, _2, _3, _4, _5, _6));
	//--------------------------------------------------------------------------

	//create a local planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!blp_loader_.isClassAvailable(local_planner)){
        std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(local_planner == blp_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                local_planner.c_str(), classes[i].c_str());
            local_planner = classes[i];
            break;
          }
        }
      }

      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &UpoNavigation::planService, this);
    
    make_rrt_plan_srv_ = private_nh.advertiseService("make_rrt_plan", &UpoNavigation::planRRTService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &UpoNavigation::clearCostmapsService, this);

	//advertise a service for get the feature count of a path
	features_srv_ = private_nh.advertiseService("path_feature_count", &UpoNavigation::getPathFeatures, this);
	
	//advertise a service for setting the weights for cost function
	weights_srv_ = private_nh.advertiseService("set_cost_function_weights", &UpoNavigation::setWeightsRRT, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("upo_navigation","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    /*if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }*/

    //initially, we'll need to make a plan
    //state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    //recovery_index_ = 0;

	//If we are not using the macro-actions servers we activate this
	if(!macro) {
		//we're all set up now so we can start the action server
		as_->start();
	}
	
    /*dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);*/
  }



  /*void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        //check if a non fully qualified name has potentially been passed in
        if(!bgp_loader_.isClassAvailable(config.base_global_planner)){
          std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
          for(unsigned int i = 0; i < classes.size(); ++i){
            if(config.base_global_planner == bgp_loader_.getName(classes[i])){
              //if we've found a match... we'll get the fully qualified name and break out of the loop
              ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                  config.base_global_planner.c_str(), classes[i].c_str());
              config.base_global_planner = classes[i];
              break;
            }
          }
        }

        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        //check if a non fully qualified name has potentially been passed in
        ROS_INFO("Loading local planner: %s", config.base_local_planner.c_str());
        if(!blp_loader_.isClassAvailable(config.base_local_planner)){
          std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
          for(unsigned int i = 0; i < classes.size(); ++i){
            if(config.base_local_planner == blp_loader_.getName(classes[i])){
              //if we've found a match... we'll get the fully qualified name and break out of the loop
              ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                  config.base_local_planner.c_str(), classes[i].c_str());
              config.base_local_planner = classes[i];
              break;
            }
          }
        }
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

   last_config_ = config;
  }*/



  //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
  //they won't get any useful information back about its status, but this is useful for tools
  //like nav_view and rviz (upo_navigation_simple/goal)
  void UpoNavigation::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("upo_navigation","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }


  void UpoNavigation::clearCostmapWindows(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose); //map

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose); //odom

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool UpoNavigation::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    planner_costmap_ros_->resetLayers();
    controller_costmap_ros_->resetLayers();
    return true;
  }
  
  bool UpoNavigation::areCostmapsCurrent()
  {
	  if(planner_costmap_ros_->isCurrent() && controller_costmap_ros_->isCurrent())
		return true;

	  return false;
  }
  
  
  void UpoNavigation::updateCostmaps()
  {
	  planner_costmap_ros_->updateMap();
	  controller_costmap_ros_->updateMap();
  }
  
  void UpoNavigation::updateLocalCostmap()
  {
	  //planner_costmap_ros_->updateMap();
	  controller_costmap_ros_->updateMap();
  }
  
  bool UpoNavigation::clearCostmaps(){
    //clear the costmaps
    planner_costmap_ros_->resetLayers();
    controller_costmap_ros_->resetLayers();
    return true;
  }
  
  bool UpoNavigation::clearLocalCostmap(){
    //clear the costmaps
    //planner_costmap_ros_->resetLayers();
    controller_costmap_ros_->resetLayers();
    return true;
  }


  bool UpoNavigation::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){

	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)){
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }
    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id == "")
      tf::poseStampedTFToMsg(global_pose, start);
    else
      start = req.start;

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("upo_navigation","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("upo_navigation", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("upo_navigation","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }
  
  
  bool UpoNavigation::planRRTService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
  {
	  //printf("\nPLANRRT SERVICE CALLED!!!!\n\n");
	  geometry_msgs::PoseStamped start;
	  start = req.start;
	  
	  geometry_msgs::PoseStamped goal;
      goal = req.goal;
      
      double tolerance = req.tolerance;
      

		if(!isQuaternionValid(goal.pose.orientation)){
		  ROS_WARN("Aborting plan because of goal quaternion invalid");
		  return false;
		}

		//Local goal
		geometry_msgs::PoseStamped local_goal = goalToLocalFrame(goal);

		//Global Goal for A* (Already in map coordinates, no transformation required)
		//geometry_msgs::PoseStamped global_goal = goalToGlobalFrame(goal);
		global_goal_ = goalToGlobalFrame(goal);

		//Intermediate point in the A* path which we use as the RRT goal 
		//geometry_msgs::PoseStamped intermediate_goal = global_goal_;

		//double rrt_radius = rrt_planner_->get_rrt_planning_radius();
		
		std::vector<geometry_msgs::PoseStamped> rrt_path;

		bool ok = makeRRTPlan(goal, rrt_path);
		
		if(!ok) {
			ROS_FATAL("ERROR obtaining RRT path");
			return false;
		}
		
		//publish_rrt_path(&rrt_path);
		
      
		//copy the plan into a message to send out
		resp.plan.header.stamp = ros::Time::now();
		resp.plan.header.frame_id = rrt_path[0].header.frame_id;
		resp.plan.poses.resize(rrt_path.size());
		for(unsigned int i = 0; i < rrt_path.size(); ++i){
			resp.plan.poses[i] = rrt_path[i];
		}
      
		return true;
  }
  




  UpoNavigation::~UpoNavigation(){
    //recovery_behaviors_.clear();

    //delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    //planner_thread_->interrupt();
    //planner_thread_->join();

    thread_active_ = false;
    rrt_thread_.join();

    delete planner_plan_;
    //delete latest_plan_;
    delete local_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
	delete rrt_planner_;

  }
  
  
  
  void UpoNavigation::publish_rrt_path(std::vector<geometry_msgs::PoseStamped>* path)
  {

	if(path && path->size() > 0) {
	  visualization_msgs::Marker points;
	  
		points.header.frame_id = path->at(0).header.frame_id; //robot_base_frame_; 
		points.header.stamp = path->at(0).header.stamp; //ros::Time();
		points.ns = "rrt";
		points.id = 0;
		points.type = visualization_msgs::Marker::SPHERE_LIST;
		points.action = visualization_msgs::Marker::ADD;
		points.pose.position.x = 0.0;
		points.pose.position.y = 0.0;
		points.pose.position.z = 0.1; 
		points.scale.x = 0.12;
		points.scale.y = 0.12;
		points.color.r = 0.0f;
		points.color.g = 1.0f;
		points.color.b = 0.0f;
		points.color.a = 1.0;
		points.lifetime = ros::Duration();
			
		for(unsigned int i=0; i<path->size(); i++)
		{
			geometry_msgs::Point p = path->at(i).pose.position;
			points.points.push_back(p);
		}
		path_points_pub_.publish(points);	
	}
  }


  bool UpoNavigation::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

	
    //boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_ros_->getCostmap()->getLock()));
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
	

    //make sure to set the plan to be empty initially
    plan.clear();
	

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("A*MakePlan. Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      ROS_WARN("A*MakePlan. Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_WARN("A*MakePlan. Failed to find a  plan to point (%.2f, %.2f frame:%s)", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
      return false;
    }

    return true;
  }

//-----------------------------------------------------------------------------------------------------------------------------
  
  bool UpoNavigation::makeRRTPlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    
    
	//boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_ros_->getCostmap()->getLock()));
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
	//-----Added by Noé
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock2(*(controller_costmap_ros_->getCostmap()->getMutex()));
	//planner_costmap_ros_->pause();
	//controller_costmap_ros_->pause();
	//----------------

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

	if(controller_costmap_ros_ == NULL) {
      ROS_ERROR("Controller costmap ROS is NULL, unable to create global plan");
      return false;
    }

	//std::string base = planner_costmap_ros_->getBaseFrameID();
	geometry_msgs::Pose2D goal_rrt;
	goal_rrt.x = goal.pose.position.x;
    goal_rrt.y = goal.pose.position.y;
    goal_rrt.theta = tf::getYaw(goal.pose.orientation);
	//std::string goalframe = std::string(goal.header.frame_id);
	//if(goalframe.compare("base_link") != 0 && goalframe.compare("/base_link") != 0)
	//{
		//printf("makeRRTPlan. Transforming goal to base_link frame!!!!!!!!\n");
		//---Transform goal to base_link coordinates
    	tf::Stamped<tf::Pose> goal_point;
    	tf::poseStampedMsgToTF(goal, goal_point);
		goal_point.stamp_ = ros::Time(); //in order to get the last transformation
    	tf::Stamped<tf::Pose> goal_baselink;
		bool correct = true;
		try {
			tf_.transformPose(planner_costmap_ros_->getBaseFrameID(), goal_point, goal_baselink);
		}catch (tf::TransformException ex){
			ROS_WARN("makeRRTPlan. goal to base_link. TransformException: %s",ex.what());
			correct = false;
		}
    	if(correct) {
			goal_rrt.x = (double)goal_baselink.getOrigin().getX();
			goal_rrt.y = (double)goal_baselink.getOrigin().getY();
			goal_rrt.theta = (double)tf::getYaw(goal_baselink.getRotation());
    	}
	//}

	geometry_msgs::Pose2D start;
	start.x = 0.0;
	start.y = 0.0;
	start.theta = 0.0;
	
	/*//Get current robot velocity -->not possible from here
	tf::Stamped<tf::Pose> robot_vel;
    tc_->odom_helper_.getRobotVel(robot_vel);
	double lv = robot_vel.getOrigin().getX();
	//float rvy = robot_vel.getOrigin().getY();
	double av = tf::getYaw(robot_vel.getRotation());
	*/
	double lv = 0.0, av = 0.0;  //TODO: We should take the current robot velocities
	
	std::vector<geometry_msgs::PoseStamped> aux;
	//Plan is in base_link coordinates
	aux = rrt_planner_->RRT_plan(start, goal_rrt, lv, av); //(sx, sy, sh, lv, av, gx, gy, gh);
	if(aux.empty())
		return false;
    if(aux.size() == 1 && aux.at(0).pose.position.x == -100.0 && aux.at(0).pose.position.z == -100.0) {
		ROS_ERROR("makeRRTPlan. No RRT plan found!!!");
		return false;
	}
	
	//Transform plan to odom coordinates
	for(unsigned int i=0; i<aux.size(); i++)
	{
		geometry_msgs::PoseStamped in = aux[i];
		geometry_msgs::PoseStamped pose_out;
		
		geometry_msgs::Quaternion q = in.pose.orientation;
		if(!isQuaternionValid(q))
		{
			ROS_WARN("Transforming RRT plan to odom. Quaternion no valid. Creating new quaternion with yaw=0.0");
			in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		}
		try {
			tf_.transformPose("odom", in, pose_out);
		}catch (tf::TransformException ex){
			ROS_WARN("Transforming RRT plan to odom: %s",ex.what());
		}
		plan.push_back(pose_out);
	}
	
	//publish_rrt_path(&plan);

    return true;
  }
  
  
  
  void UpoNavigation::setFeaturesWeights(std::vector<float> w)
  {
		rrt_planner_->setWeights(w);
  }
  
  
  bool UpoNavigation::setWeightsRRT(upo_navigation::SetWeights::Request  &req, upo_navigation::SetWeights::Response &res)
  {
	  rrt_planner_->setWeights(req.weights);
	  return true;
  }


  bool UpoNavigation::getPathFeatures(upo_navigation::FeatureCounts::Request  &req, upo_navigation::FeatureCounts::Response &res)
  {
		geometry_msgs::PoseStamped* g = &req.goal;
		std::vector<geometry_msgs::PoseStamped>* p = &req.path;
		res.features = get_feature_counts(g, p);
		return true;
  }

  
  std::vector<float> UpoNavigation::get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path)
  {
		return rrt_planner_->get_feature_counts(goal, path);
  }
  
  
  std::vector<float> UpoNavigation::get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people)
  {
	  return rrt_planner_->get_feature_counts(goal, path, people);
  }
  
  float UpoNavigation::get_path_cost(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people)
  {
	  return rrt_planner_->get_path_cost(goal, path, people);
  }
  
  float UpoNavigation::get_path_cost()
  {
	  return rrt_planner_->get_path_cost();
  }





  void UpoNavigation::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool UpoNavigation::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped UpoNavigation::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

	if(global_frame.compare(goal_pose_msg.header.frame_id.c_str()) == 0) {
		//printf("goalToGlobalFrame. No transformation required\n");
		return goal_pose_msg;
	}

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("goalToGlobalFrame. Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }


  geometry_msgs::PoseStamped UpoNavigation::goalToLocalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string local_frame = planner_costmap_ros_->getBaseFrameID();
    tf::Stamped<tf::Pose> goal_pose, local_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform...
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(local_frame, goal_pose, local_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("goalToLocalFrame. Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), local_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped local_pose_msg;
    tf::poseStampedTFToMsg(local_pose, local_pose_msg);
    return local_pose_msg;
  }



  /*void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }*/

  /*void UpoNavigation::planThread(){
    ROS_DEBUG_NAMED("upo_navigation_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("upo_navigation_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        ROS_DEBUG_NAMED("upo_navigation_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit
        lock.lock();
        if(ros::Time::now() > attempt_end && runPlanner_){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }
        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }*/



  
/*
 * Thread for RRT planning
 */
 void UpoNavigation::rrt_thread(void) 
 {
	std::vector<geometry_msgs::PoseStamped> rrt_plan;
	bool run_rrt;
	//bool rrt_sleep;
	geometry_msgs::PoseStamped rrt_goal;
	//double rrt_plan_cost = -1;
	bool got_plan = false;

	 while(thread_active_)
	 {
		rrt_mutex_.lock();
		run_rrt = run_rrt_;
		//rrt_sleep = rrt_sleep_; //--
		//rrt_plan.clear();
		//rrt_plan = *local_plan_; //local_plan in odom coordinates
		rrt_goal = rrt_goal_;
		rrt_mutex_.unlock();

		bool use_new_path = true;
		float new_path_cost = 0.0;
		float prev_path_cost = rrt_planner_->get_path_cost();
		
		
		if(run_rrt /*!rrt_sleep && (run_rrt || !(rrt_planner_->check_rrt_path(&rrt_plan)))*/)
		{
			//---------
			//rrt_mutex_.lock();
			//rrt_goal = rrt_goal_; 
			//run_rrt_ = false; 
			//rrt_mutex_.unlock();
			//-----------
			ros::WallTime startRRT = ros::WallTime::now();
			rrt_plan.clear();
			
			//rrt_goal_ is in map coordinates but makeRRTPlan transform it to base_link
			if(makeRRTPlan(rrt_goal, rrt_plan)) {
				got_plan = true;
				//new_path_cost = rrt_planner_->get_path_cost();
				//if(new_path_cost > 0.0 && (fabs(prev_path_cost - new_path_cost)/new_path_cost) < 0.12)
				//	use_new_path = false;
				//printf("Prev_cost: %.3f, new_cost: %.2f, error: %.3f\n", prev_path_cost, new_path_cost, (fabs(prev_path_cost - new_path_cost)/new_path_cost));
			} else {
				got_plan = false;
				
			}
			ros::WallDuration rrt_time = ros::WallTime::now() - startRRT;
			
		} /*else if(!(rrt_planner_->check_rrt_path(rrt_plan)))
		{

		}*/

		rrt_mutex_.lock();
		run_rrt = run_rrt_;
		rrt_mutex_.unlock();
		if(got_plan && run_rrt && use_new_path)
		{
			rrt_mutex_.lock();
			new_rrt_plan_ = true;
			//run_rrt_ = false; //---
			local_plan_ = &rrt_plan; //odom coordinates
			//printf("RRT thread. Local plan of size:%u\n", (unsigned int)local_plan_->size());
			rrt_mutex_.unlock();
			got_plan = false;
		}

		// Sleep some milliseconsds
		 boost::this_thread::sleep(boost::posix_time::milliseconds(thread_sleep_msecs_));
	}
 }




  void UpoNavigation::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {

	printf("¡¡¡¡¡¡¡New goal received!!!!!!\n"); //Goal in map coordinates
	//printf("Goal in coordinates: %s\n\n", (move_base_goal->target_pose.header.frame_id).c_str());

	//geometry_msgs::PoseStamped rrt_goal_odom;

	/*
	//Transform goal into odom frame
	geometry_msgs::PoseStamped p_in;
	p_in = move_base_goal->target_pose;
	p_in.header.stamp = ros::Time(0); 
	geometry_msgs::PoseStamped p_out;
	try {
			tf_.transformPose("odom", p_in, p_out); //¿odom o map?
	}catch (tf::TransformException ex){
			ROS_ERROR("executeCb. TransformException: %s",ex.what());
	}
	
	rrt_planner_->setGoalOdom((double)p_out.pose.position.x, (double)p_out.pose.position.y);
	*/

    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
	  ROS_ERROR("Aborting navigation because of goal quaternion invalid");
      return;
    }

	//Local goal
    geometry_msgs::PoseStamped local_goal = goalToLocalFrame(move_base_goal->target_pose);

	//Global Goal for A* (Already in map coordinates, no transformation required)
	//geometry_msgs::PoseStamped global_goal = goalToGlobalFrame(move_base_goal->target_pose);
	global_goal_ = goalToGlobalFrame(move_base_goal->target_pose);

	//Intermediate point in the A* path which we use as the RRT goal 
	geometry_msgs::PoseStamped intermediate_goal = global_goal_;

	double rrt_radius = rrt_planner_->get_rrt_planning_radius();

	//if the goal is outside the local area, we use the A* and RRT*; if not, we use just the RRT* 
	//double max_x_from_robot = ((controller_costmap_ros_->getCostmap()->getSizeInMetersX())/2);
	//double max_y_from_robot = ((controller_costmap_ros_->getCostmap()->getSizeInMetersY())/2);
	//if(fabs(local_goal.pose.position.x) > rrt_radius ||
	//	fabs(local_goal.pose.position.y) > rrt_radius)
	//{
		planner_plan_->clear();
		//Plan with A*
      	if(makePlan(global_goal_, *planner_plan_)) //planner_plan is in map coordinates
			new_global_plan_ = true;
		else {
			new_global_plan_ = false;
			ROS_ERROR("ERROR. No global A* plan created!!!");
			return;
		}

		//get the pose of the robot
    	tf::Stamped<tf::Pose> global_pose;
    	if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      		ROS_WARN("Unable to get pose of robot!!!");
    	}
    	geometry_msgs::PoseStamped robot_pose;
    	tf::poseStampedTFToMsg(global_pose, robot_pose);

		//Now take the last point of the global path inside
		//the local area --> goal for the RRT*
		unsigned int path_i = 0;
		for(unsigned int i = 0; i < planner_plan_->size(); ++i) {
			double gx = planner_plan_->at(i).pose.position.x;
			double gy = planner_plan_->at(i).pose.position.y;
			unsigned int map_x, map_y;
			double dist_x = fabs(gx - robot_pose.pose.position.x);
			double dist_y = fabs(gy - robot_pose.pose.position.y);
			if (dist_x <= (rrt_radius) && dist_y <= (rrt_radius)) {
				path_i = i;
			} else
				break;
		}
   		//printf("!!!!map goal x:%.2f, y:%.2f\n", intermediate_goal.pose.position.x, intermediate_goal.pose.position.y);
   		//add orientation to the point
   		if(path_i < (planner_plan_->size()-1)) {
			intermediate_goal = planner_plan_->at(path_i);
			intermediate_goal.header.stamp = ros::Time::now();
			geometry_msgs::PoseStamped p1 = planner_plan_->at(path_i);
			geometry_msgs::PoseStamped p2 = planner_plan_->at(path_i+1);
			float x_diff = p2.pose.position.x - p1.pose.position.x;
			float y_diff = p2.pose.position.y - p1.pose.position.y;
			float yaw = atan2(y_diff, x_diff);
			intermediate_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		} 
		
		//----new-------
		std::vector<geometry_msgs::PoseStamped> path_to_follow;
		for(unsigned int i = 0; i <= path_i; ++i) {
			geometry_msgs::PoseStamped out = goalToLocalFrame(planner_plan_->at(i));
			path_to_follow.push_back(out);
		}
		setPathToBias(path_to_follow);
		
		
	//} else {
	//	intermediate_goal = global_goal_;
	//}

	current_goal_pub_.publish(intermediate_goal);
	//Tell the rrt thread to plan
	rrt_mutex_.lock();
	rrt_goal_ = intermediate_goal;
	//rrt_sleep_ = false;
	run_rrt_ = true;
	rrt_mutex_.unlock();


    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("upo_navigation","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

	ros::Rate r(controller_frequency_);
	ros::NodeHandle n;
	int pursue_status = -2;
	bool exit = false;
	ros::WallTime startt; // = ros::WallTime::now();
    while(n.ok()) //&& !exit)
    {
		startt = ros::WallTime::now();
		
		if(as_->isPreemptRequested()){
			
			
        	if(as_->isNewGoalAvailable()){
				
          		//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          		move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          		if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            		as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            		return;
          		}

          		global_goal_ = goalToGlobalFrame(new_goal.target_pose);

				intermediate_goal = global_goal_;

				local_goal = goalToLocalFrame(new_goal.target_pose);

          		//if the goal is outside the local area, we use the A* and RRT*; if not, we use just the RRT* 
				//if(fabs(local_goal.pose.position.x) > rrt_radius ||
				//		fabs(local_goal.pose.position.y) > rrt_radius)
				//{
					planner_plan_->clear();
					//Plan with A*
      				if(makePlan(global_goal_, *planner_plan_))
						new_global_plan_ = true;
					else {
						new_global_plan_ = false;
						ROS_ERROR("ERROR. No global A* plan created!!!");
						return;
					}

					//get the pose of the robot
    				tf::Stamped<tf::Pose> global_pose;
    				if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      					ROS_WARN("Unable to get pose of robot!!!");
    				}
    				geometry_msgs::PoseStamped robot_pose;
    				tf::poseStampedTFToMsg(global_pose, robot_pose);

					//Now take the last point of the global path inside
					//the local area --> goal for the RRT*
					unsigned int path_i = 0;
					for(unsigned int i = 0; i < planner_plan_->size(); ++i) {
						double gx = planner_plan_->at(i).pose.position.x;
						double gy = planner_plan_->at(i).pose.position.y;
						unsigned int map_x, map_y;
						double dist_x = fabs(gx - robot_pose.pose.position.x);
						double dist_y = fabs(gy - robot_pose.pose.position.y);
						if (dist_x <= (rrt_radius) && dist_y <= (rrt_radius)) {
							path_i = i;
						} else
							break;
					}
					//add orientation to the point
					if(path_i < (planner_plan_->size()-1)) {
						intermediate_goal = planner_plan_->at(path_i);
						intermediate_goal.header.stamp = ros::Time::now();
						geometry_msgs::PoseStamped p1 = planner_plan_->at(path_i);
						geometry_msgs::PoseStamped p2 = planner_plan_->at(path_i+1);
						float x_diff = p2.pose.position.x - p1.pose.position.x;
						float y_diff = p2.pose.position.y - p1.pose.position.y;
						float yaw = atan2(y_diff, x_diff);
						intermediate_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
					} 
					
					//----new-------
					std::vector<geometry_msgs::PoseStamped> path_to_follow;
					for(unsigned int i = 0; i <= path_i; ++i) {
						geometry_msgs::PoseStamped out = goalToLocalFrame(planner_plan_->at(i));
						path_to_follow.push_back(out);
					}
					setPathToBias(path_to_follow);
					
					
				//} else {
				//	intermediate_goal = global_goal_;
				//}

				//We update the local plan to an empty plan in order to stop the robot
				//until get a new plan
				//local_plan_->clear();
				//pure_pursuit_->updatePlan(*local_plan_, true);
				//tc_->setPlan(*local_plan_);
        		
				//as_->setAborted(move_base_msgs::MoveBaseResult(), "Current goal aborted.");
				//return;
				
				current_goal_pub_.publish(intermediate_goal);
				//Tell the rrt thread to plan
				rrt_mutex_.lock();
				rrt_goal_ = intermediate_goal;
				//rrt_sleep_ = false; //---
				run_rrt_ = true;
				rrt_mutex_.unlock();
				


          		//publish the goal point to the visualizer
          		ROS_DEBUG_NAMED("upo_navigation","upo_navigation has received a goal of x: %.2f, y: %.2f", global_goal_.pose.position.x, global_goal_.pose.position.y);

       	 	}
        	else {
          		//if we've been preempted explicitly we need to shut things down
          		resetState();

          		//notify the ActionServer that we've successfully preempted
          		ROS_DEBUG_NAMED("upo_navigation","upo_navigation preempting the current goal");
          		as_->setPreempted();

          		//we'll actually return from execute after preempting
          		return;
        	}
      	}


    	tf::Stamped<tf::Pose> global_pose;
    	if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      			ROS_WARN("Unable to get pose of robot!!!");
    	}
    	geometry_msgs::PoseStamped robot_pose;
    	tf::poseStampedTFToMsg(global_pose, robot_pose);
		
		rrt_mutex_.lock();
		geometry_msgs::PoseStamped goal_pose = rrt_goal_;
		rrt_mutex_.unlock();
		

		//Check the distance of the current goal---------------------------
		double robot_dist = sqrt(pow((goal_pose.pose.position.x - robot_pose.pose.position.x),2)+pow((goal_pose.pose.position.y - robot_pose.pose.position.y),2));
		//distance from the intermediate goal to the final goal
		//double final_dist = sqrt(pow((goal_pose.pose.position.x - global_goal_.pose.position.x),2)+pow((goal_pose.pose.position.y - global_goal_.pose.position.y),2));
		//if(final_dist >= 0.2 /*&& robot_dist < 1.8*/) //we plan again with an updated goal
		
		/*if(robot_dist < 0.2) {
			rrt_mutex_.lock();
			rrt_goal_ = global_goal_;
			rrt_sleep_ = true;
			run_rrt_ = false;
			rrt_mutex_.unlock();
			
		}*/
		if(robot_dist < 0.2) {
			rrt_mutex_.lock();
			new_rrt_plan_ = false;
			rrt_goal_ = global_goal_;
			//rrt_sleep_ = true; //--
			run_rrt_ = false;
			rrt_mutex_.unlock();
			geometry_msgs::PoseStamped p = goalToLocalFrame(global_goal_);
			//p.pose.position.x = 0.0;
			//p.pose.orientation.y = 0.0;
			std::vector<geometry_msgs::PoseStamped> parray;
			parray.push_back(p);
			tc_->setPlan(parray);
		} else {
		
			//double dist = sqrt(pow((global_goal_.pose.position.x - robot_pose.pose.position.x),2)+pow((global_goal_.pose.position.y - robot_pose.pose.position.y),2));
			//if(dist > rrt_radius)
			//{
				//Now take the last point of the global path inside
				//the local area --> goal for the RRT*
				unsigned int path_i = 0;
				unsigned int closest = 0;
				float max_dist = 100.0;
				for(unsigned int i = 0; i < planner_plan_->size(); ++i) {
					double gx = planner_plan_->at(i).pose.position.x;
					double gy = planner_plan_->at(i).pose.position.y;
					unsigned int map_x, map_y;
					float dist_x = fabs(gx - robot_pose.pose.position.x);
					float dist_y = fabs(gy - robot_pose.pose.position.y);
					float dist = sqrt((dist_x*dist_x)+(dist_y*dist_y));
					if(dist < max_dist) {
						closest = i;
						max_dist = dist;
					}
				}
				
				std::vector<geometry_msgs::PoseStamped> path_to_follow;
				for(unsigned int i = closest; i < planner_plan_->size(); ++i) {
					
					geometry_msgs::PoseStamped out = goalToLocalFrame(planner_plan_->at(i));
					path_to_follow.push_back(out);
					
					double gx = planner_plan_->at(i).pose.position.x;
					double gy = planner_plan_->at(i).pose.position.y;
					unsigned int map_x, map_y;
					float dist_x = fabs(gx - robot_pose.pose.position.x);
					float dist_y = fabs(gy - robot_pose.pose.position.y);
					float dist = sqrt((dist_x*dist_x)+(dist_y*dist_y));
					if (dist_x <= (rrt_radius) && dist_y <= (rrt_radius)) {
						path_i = i;
					} else
						break;
				}
				
				setPathToBias(path_to_follow);
				
				//add orientation to the point
				if(path_i < (planner_plan_->size()-1)) {
					intermediate_goal = planner_plan_->at(path_i);
					intermediate_goal.header.stamp = ros::Time::now();
					intermediate_goal.header.frame_id = move_base_goal->target_pose.header.frame_id;
					geometry_msgs::PoseStamped p1 = planner_plan_->at(path_i);
					geometry_msgs::PoseStamped p2 = planner_plan_->at(path_i+1);
					float x_diff = p2.pose.position.x - p1.pose.position.x;
					float y_diff = p2.pose.position.y - p1.pose.position.y;
					float yaw = atan2(y_diff, x_diff);
					intermediate_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
				} else {
					intermediate_goal = global_goal_;
				}
				
				
				
				
			//} else {
			//	intermediate_goal = global_goal_;
			//}
			
			//We update the local plan to an empty plan in order to stop the robot
			//until get a new plan
			//local_plan_->clear();
			//tc_->setPlan(*local_plan_);
				
				
			current_goal_pub_.publish(intermediate_goal);
			//printf("------RRT* is going to plan---------\n");
			//Tell the rrt thread to plan
			rrt_mutex_.lock();
			rrt_goal_ = intermediate_goal;
			//rrt_sleep_ = false;
			run_rrt_ = true;
			rrt_mutex_.unlock();
				
		}
		//-------------------------------------------------------------------
		
		
		//if RRT obtained a new local plan
		rrt_mutex_.lock();
		if(new_rrt_plan_) {
		
			//update the path
			//pure_pursuit_->updatePlan(*local_plan_, true);
			tc_->setPlan(*local_plan_);
			
			publish_rrt_path(local_plan_);
		
			//disable flag
			new_rrt_plan_ = false;
			pursue_status = 0;
		} 
		rrt_mutex_.unlock();

		//ros::WallDuration t9 = ros::WallTime::now() - startt;
		//printf("Test 9: %.4f secs\n", t9.toSec());
		
		//the real work on pursuing a goal is done here
		if(pursue_status != 1 && pursue_status!=-2) {
      		pursue_status = executeCycle();
      		//ros::WallDuration t10 = ros::WallTime::now() - startt;
			//printf("Test 10: %.4f secs\n", t10.toSec());
		}

		//ros::WallDuration t11 = ros::WallTime::now() - startt;
		//printf("Test 11: %.4f secs\n", t11.toSec());
			
      	//if we're done, then we'll return from execute
      	if(pursue_status == 1) {
			
			ROS_INFO("*******GOAL REACHED******");
			rrt_mutex_.lock();
			//rrt_sleep_ = true;
			run_rrt_ = false;
			rrt_mutex_.unlock();
			return;
			

		} else if(pursue_status == -1) { //if we don't find a valid control, plan again.

			//We update the local plan to a empty plan in order to stop the robot
			//until we get a new plan
			printf("Stopping the robot!!!!!!!\n");
			//local_plan_->clear(); //try to comment this
			//pure_pursuit_->updatePlan(*local_plan_, true);
			//tc_->setPlan(*local_plan_); //try to comment this

			//Tell the rrt thread to plan
			rrt_mutex_.lock();
			run_rrt_ = true;
			rrt_mutex_.unlock();
		}

		//map_viz_.publishCostCloud(controller_costmap_ros_->getCostmap());
		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());
		r.sleep();
	}
	//if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;

  }
  
  
  
  
  //method to navigate to be called from an external action server.
  //Similar to executeCb
  bool UpoNavigation::executeNavigation(geometry_msgs::PoseStamped goal)
  {
	  //printf("¡¡¡¡¡¡¡ExecuteNavigation: New goal received!!!!!!\n"); //Goal in map coordinates

	if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("upo_navigation","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    if(!isQuaternionValid(goal.pose.orientation)){
	  ROS_ERROR("ExecuteNavigation: Aborting navigation because of goal quaternion invalid");
      return false;
    }

	//printf("ExecuteNav. GOAL frame: %s, x:%.2f, y:%.2f\n", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);

	//Local goal
    geometry_msgs::PoseStamped local_goal = goalToLocalFrame(goal);
    
    //printf("ExecuteNav. LOCAL frame: %s, x:%.2f, y:%.2f\n", local_goal.header.frame_id.c_str(), local_goal.pose.position.x, local_goal.pose.position.y);

	//Global Goal for A* (Already in map coordinates, no transformation required)
	global_goal_ = goalToGlobalFrame(goal);
	
	//printf("ExecuteNav. GLOBAL frame: %s, x:%.2f, y:%.2f\n", global_goal_.header.frame_id.c_str(), global_goal_.pose.position.x, global_goal_.pose.position.y);

	//Intermediate point in the A* path which we use as the RRT goal 
	geometry_msgs::PoseStamped intermediate_goal = global_goal_;

	double rrt_radius = rrt_planner_->get_rrt_planning_radius();

	//if the goal is outside the local area, we use the A* and RRT*; if not, we use just the RRT* 
	//double max_x_from_robot = ((controller_costmap_ros_->getCostmap()->getSizeInMetersX())/2);
	//double max_y_from_robot = ((controller_costmap_ros_->getCostmap()->getSizeInMetersY())/2);
	//if(fabs(local_goal.pose.position.x) > rrt_radius ||
	//	fabs(local_goal.pose.position.y) > rrt_radius)
	//{
		planner_plan_->clear();
		//Plan with A*
      	if(makePlan(global_goal_, *planner_plan_)) //planner_plan is in map coordinates
			new_global_plan_ = true;
		else {
			new_global_plan_ = false;
			ROS_ERROR("ERROR. No global A* plan created!!! global goal x:%.2f, y:%.2f", global_goal_.pose.position.x, global_goal_.pose.position.y);
			return false;
		}

		//get the pose of the robot
    	tf::Stamped<tf::Pose> global_pose;
    	if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      		ROS_WARN("Unable to get pose of robot!!!");
    	}
    	geometry_msgs::PoseStamped robot_pose;
    	tf::poseStampedTFToMsg(global_pose, robot_pose);

		//Now take the last point of the global path inside
		//the local area --> goal for the RRT*
		unsigned int path_i = 0;
		//for(unsigned int i = planner_plan_->size()-1; i > 0; --i) {
		for(unsigned int i = 0; i < planner_plan_->size(); ++i) {
			double gx = planner_plan_->at(i).pose.position.x;
			double gy = planner_plan_->at(i).pose.position.y;
			unsigned int map_x, map_y;
			double dist_x = fabs(gx - robot_pose.pose.position.x);
			double dist_y = fabs(gy - robot_pose.pose.position.y);
			if (dist_x <= rrt_radius && dist_y <= rrt_radius) {
				path_i = i;
				//intermediate_goal.header.frame_id = move_base_goal->target_pose.header.frame_id;
			} else
				break;
		}
   		//printf("!!!!map goal x:%.2f, y:%.2f\n", intermediate_goal.pose.position.x, intermediate_goal.pose.position.y);
   		//add orientation to the point
		if(path_i < (planner_plan_->size()-1)) {
			intermediate_goal = planner_plan_->at(path_i);
			intermediate_goal.header.stamp = ros::Time::now();
			geometry_msgs::PoseStamped p1 = planner_plan_->at(path_i);
			geometry_msgs::PoseStamped p2 = planner_plan_->at(path_i+1);
			float x_diff = p2.pose.position.x - p1.pose.position.x;
			float y_diff = p2.pose.position.y - p1.pose.position.y;
			float yaw = atan2(y_diff, x_diff);
			intermediate_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		} else {
			intermediate_goal = global_goal_;
		}
		
		//----new-------
		std::vector<geometry_msgs::PoseStamped> path_to_follow;
		for(unsigned int i = 0; i <= path_i; ++i) {
			geometry_msgs::PoseStamped out = goalToLocalFrame(planner_plan_->at(i));
			path_to_follow.push_back(out);
		}
		setPathToBias(path_to_follow);
		//---------------
	//}

	current_goal_pub_.publish(intermediate_goal);
	//Tell the rrt thread to plan
	rrt_mutex_.lock();
	rrt_goal_ = intermediate_goal;
	//rrt_sleep_ = false;
	run_rrt_ = true;
	rrt_mutex_.unlock();

    
    return true;
  
 }
 
 
 
 
 
 int UpoNavigation::pathFollow(geometry_msgs::PoseStamped& new_pose)
 {
	
	tf::Stamped<tf::Pose> global_pose;
	if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      	ROS_WARN("Unable to get pose of robot!!!");
      	return -2;
    }
    geometry_msgs::PoseStamped robot_pose;
    tf::poseStampedTFToMsg(global_pose, robot_pose);
		
	rrt_mutex_.lock();
	geometry_msgs::PoseStamped goal_pose = rrt_goal_;
	rrt_mutex_.unlock();
		
	float rrt_radius = rrt_planner_->get_rrt_planning_radius();

	//Check the distance of the current goal--------------------------
	float dist = sqrt(pow((global_goal_.pose.position.x - robot_pose.pose.position.x),2)+pow((global_goal_.pose.position.y - robot_pose.pose.position.y),2));
	
	/*float ang_dist = tf::getYaw(global_goal_.pose.orientation) - tf::getYaw(robot_pose.pose.orientation);
	printf("Goal th:%.3f, robot th:%.3f, angle_dist:%.3f", tf::getYaw(global_goal_.pose.orientation), tf::getYaw(robot_pose.pose.orientation), ang_dist);
	ang_dist = normalizeAngle(ang_dist, -M_PI, M_PI);
	printf(" ang_dist_norm:%.3f\n", ang_dist);
	
	//TODO: this is provisional. If we receive a goal
	//in the same point with different orientation, the system will fail
	if(dist < 0.15 && fabs(ang_dist) < 0.15) {
		ROS_INFO("¡¡¡¡ GOAL REACHED !!!!");
		rrt_mutex_.lock();
		rrt_sleep_ = true;
		run_rrt_ = false;
		local_plan_->clear();
		tc_->setPlan(*local_plan_);
		rrt_mutex_.unlock();
		return 1;
	}*/
	
	if(dist < 0.20) {
		rrt_mutex_.lock();
		new_rrt_plan_ = false;
		rrt_goal_ = global_goal_;
		//rrt_sleep_ = true;
		run_rrt_ = false;
		rrt_mutex_.unlock();
		geometry_msgs::PoseStamped p = goalToLocalFrame(global_goal_);
		//p.pose.position.x = 0.0;
		//p.pose.orientation.y = 0.0;
		std::vector<geometry_msgs::PoseStamped> parray;
		parray.push_back(p);
		tc_->setPlan(parray);
			
	} else {
		
		geometry_msgs::PoseStamped intermediate_goal;
		
		//Now take the point of the global path inside
		//the local area --> goal for the RRT*
		unsigned int path_i = 0;
		unsigned int closest = 0;
		float max_dist = 100.0;
		for(unsigned int i = 0; i < planner_plan_->size(); ++i) {
			double gx = planner_plan_->at(i).pose.position.x;
			double gy = planner_plan_->at(i).pose.position.y;
			unsigned int map_x, map_y;
			float dist_x = fabs(gx - robot_pose.pose.position.x);
			float dist_y = fabs(gy - robot_pose.pose.position.y);
			float dist = sqrt((dist_x*dist_x)+(dist_y*dist_y));
			if(dist < max_dist) {
				closest = i;
				max_dist = dist;
			}
		}
			
		std::vector<geometry_msgs::PoseStamped> path_to_follow;
		for(unsigned int i = closest; i < planner_plan_->size(); ++i) {
				
			geometry_msgs::PoseStamped out = goalToLocalFrame(planner_plan_->at(i));
			path_to_follow.push_back(out);
				
			double gx = planner_plan_->at(i).pose.position.x;
			double gy = planner_plan_->at(i).pose.position.y;
			unsigned int map_x, map_y;
			float dist_x = fabs(gx - robot_pose.pose.position.x);
			float dist_y = fabs(gy - robot_pose.pose.position.y);
			float dist = sqrt((dist_x*dist_x)+(dist_y*dist_y));
			if (dist_x <= (rrt_radius) && dist_y <= (rrt_radius)) {
				path_i = i;
			} else
				break;
		}
			
		setPathToBias(path_to_follow);
			
			
		//add orientation to the point
		if(path_i < (planner_plan_->size()-1)) {
			intermediate_goal = planner_plan_->at(path_i);
			intermediate_goal.header.stamp = ros::Time::now();
			intermediate_goal.header.frame_id = planner_plan_->at(path_i).header.frame_id;
			geometry_msgs::PoseStamped p1 = planner_plan_->at(path_i);
			geometry_msgs::PoseStamped p2 = planner_plan_->at(path_i+1);
			float x_diff = p2.pose.position.x - p1.pose.position.x;
			float y_diff = p2.pose.position.y - p1.pose.position.y;
			float yaw = atan2(y_diff, x_diff);
			intermediate_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		} else {
			intermediate_goal = global_goal_;
		}
			
			
			
		current_goal_pub_.publish(intermediate_goal);
		//printf("------RRT* is going to plan---------\n");
		//Tell the rrt thread to plan
		rrt_mutex_.lock();
		rrt_goal_ = intermediate_goal;
		//rrt_sleep_ = false;
		run_rrt_ = true;
		rrt_mutex_.unlock();
			
	} 
		
	//if RRT obtained a new local plan
	rrt_mutex_.lock();
	if(new_rrt_plan_) {
		
		//disable flag
		new_rrt_plan_ = false;
		//update the path
		tc_->setPlan(*local_plan_);
		
		publish_rrt_path(local_plan_);
		
	} 
	rrt_mutex_.unlock();

	
	
	//the real work on pursuing a goal is done here
	int pursue_status = executeControllerCycle();
	
			
	//if we're done, then we'll return from execute
	if(pursue_status == 1) {
			
		ROS_INFO("*******GOAL REACHED******");
		rrt_mutex_.lock();
		//rrt_sleep_ = true;
		run_rrt_ = false;
		local_plan_->clear();
		tc_->setPlan(*local_plan_);
		rrt_mutex_.unlock();
			

	} else if(pursue_status == -1) { //if we don't find a valid control, plan again.

		//We update the local plan to a empty plan in order to stop the robot
		//until we get a new plan
		printf("PathFollow. Stopping the robot!!!!!!!\n");
		//local_plan_->clear();
		//tc_->setPlan(*local_plan_);

		//Tell the rrt thread to plan
		rrt_mutex_.lock();
		run_rrt_ = true;
		rrt_mutex_.unlock();
	}

	

		
	//update feedback to correspond to our curent position
	if(!planner_costmap_ros_->getRobotPose(global_pose)) {
		ROS_WARN("Unable to get pose of robot!!!");
		return -2;
	}
	geometry_msgs::PoseStamped current_position;
	tf::poseStampedTFToMsg(global_pose, current_position);

	new_pose = current_position;
		
	return pursue_status;
		
  }  
  
  
  
  
  void UpoNavigation::stopRRTPlanning()
  {
		geometry_msgs::PoseStamped g;
		g.header.stamp = ros::Time::now();
		g.header.frame_id = "base_link";
		g.pose.position.x = 0.0;
		g.pose.position.y = 0.0;
		g.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		rrt_mutex_.lock();
		rrt_goal_ = g;
		//rrt_sleep_ = true;
		run_rrt_ = false;
		rrt_mutex_.unlock();
		
		//printf("¡¡¡¡Stopping RRT planning!!!!\n");
		//publishZeroVelocity();
		local_plan_->clear();
		tc_->setPlan(*local_plan_);
  }
  
  
  
  geometry_msgs::PoseStamped UpoNavigation::getRobotGlobalPosition()
  {
		geometry_msgs::PoseStamped current_position;
		tf::Stamped<tf::Pose> global_pose;
		if(!planner_costmap_ros_->getRobotPose(global_pose)) {
			ROS_WARN("getRobotGlobalPosition. Unable to get pose of robot!!!");
			return current_position;
		}

		tf::poseStampedTFToMsg(global_pose, current_position);
		return current_position;
  }
  
  
  
  
  
  bool UpoNavigation::set_approaching_gmm_sampling(float orientation, int num_samp, geometry_msgs::PoseStamped person)
  {
	  return rrt_planner_->set_approaching_gmm_sampling(orientation, num_samp, person);
  }
  
  



  double UpoNavigation::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }


  int UpoNavigation::executeCycle(){

	//ros::WallTime t1 = ros::WallTime::now();

    //boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    /*if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index 
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }*/

	
    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return 0;
    }

	
	//check to see if we've reached our goal
 	if(tc_->isGoalReached()){
		
		//--Noé. Check if the end of the local_plan if still far
		//from the goal. In that case, return -1, to plan
		//again.
		double d = sqrt(pow((global_goal_.pose.position.x - current_position.pose.position.x), 2) + pow((global_goal_.pose.position.y - current_position.pose.position.y), 2));
		if(d > 0.8)
			return -1;
		

   		//disable the planner thread
		boost::unique_lock<boost::mutex> lock(planner_mutex_);
		runPlanner_ = false;
		lock.unlock();

		//disable rrt thread
		//rrt_mutex_.lock();
		//run_rrt_ = false;
		//rrt_mutex_.unlock();

		//ROS_INFO("*******GOAL REACHED******");
		as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
		//pure_pursuit_->resetGoal();
		return 1;
	}

	//ros::WallDuration t4 = ros::WallTime::now() - t1;
	//printf("Execute cycle. Time4: %.3f\n", t4.toSec());
	
	//¡¡¡¡¡be careful with this!!!!!!!!!
	//The rrtThread has locked the mutex, so the program stops here
	//boost::unique_lock< boost::shared_mutex > lock(*(controller_costmap_ros_->getCostmap()->getLock())); //old version costmap_2d
	//boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex())); //new version costmap_2d
        
    //ros::WallDuration t5 = ros::WallTime::now() - t1;
	//printf("Execute cycle. Time5: %.3f\n", t5.toSec());

	/*tf::Stamped<tf::Pose> global_pose_odom;
    if (!controller_costmap_ros_->getRobotPose(global_pose_odom)) {
	  ROS_WARN("Robot pose in odom coordinates can not be obtained!!!!");
      return 0;
    }*/

	//Take current robot velocity
    //tf::Stamped<tf::Pose> robot_vel;
    //odom_helper_->getRobotVel(robot_vel);
    

	//if(pure_pursuit_->findBestAction(global_pose_odom, robot_vel, cmd_vel))
	if(tc_->computeVelocityCommands(cmd_vel))
	{
          vel_pub_.publish(cmd_vel);
          //printf("publishing vel: %.2f\n", cmd_vel.linear.x);
		  return 0;
	} else {
		//Habilitar el clearing o planear de nuevo porque el robot se queda bloqueado!!!!!!!
		ROS_ERROR("Controller could not find a valid control!!!");
		return -1;
	}
  }
  
  
  
  int UpoNavigation::executeControllerCycle(){

	//ros::WallTime t1 = ros::WallTime::now();

    //boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;
	
    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return 0;
    }


	//if(local_plan_->empty())
	//	return 0;

	
	//check to see if we've reached our goal
 	if(tc_->isGoalReached()){
		
		//--Noé. Check if the end of the local_plan if still far
		//from the goal. In that case, return -1, to plan
		//again.
		//double d = sqrt(pow((global_goal_.pose.position.x - current_position.pose.position.x), 2) + pow((global_goal_.pose.position.y - current_position.pose.position.y), 2));
		//if(d > 0.8)
			//return -1;
		

   		//disable the planner thread
		boost::unique_lock<boost::mutex> lock(planner_mutex_);
		runPlanner_ = false;
		lock.unlock();

		//disable rrt thread
		rrt_mutex_.lock(); //-------
		run_rrt_ = false;
		rrt_mutex_.unlock();

		//ROS_INFO("*******GOAL REACHED******");
		//as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
		//pure_pursuit_->resetGoal();
		return 1;
	}

	//ros::WallDuration t4 = ros::WallTime::now() - t1;
	//printf("Execute cycle. Time4: %.3f\n", t4.toSec());
	
	//¡¡¡¡¡be careful with this!!!!!!!!!
	//The rrtThread has locked the mutex, so the program stops here
	//boost::unique_lock< boost::shared_mutex > lock(*(controller_costmap_ros_->getCostmap()->getLock())); //old version costmap_2d
	//boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex())); //new version costmap_2d
        
    //ros::WallDuration t5 = ros::WallTime::now() - t1;
	//printf("Execute cycle. Time5: %.3f\n", t5.toSec());

	/*tf::Stamped<tf::Pose> global_pose_odom;
    if (!controller_costmap_ros_->getRobotPose(global_pose_odom)) {
	  ROS_WARN("Robot pose in odom coordinates can not be obtained!!!!");
      return 0;
    }*/


	//Take current robot velocity
    //tf::Stamped<tf::Pose> robot_vel;
    //odom_helper_->getRobotVel(robot_vel);
    

	if(tc_->computeVelocityCommands(cmd_vel))
	{
          vel_pub_.publish(cmd_vel);
		  return 0;
	} else {
		ROS_ERROR("Controller could not find a valid control!!!");
		return -1;
	}
  }
	





  /*bool UpoNavigation::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }*/

  //we'll load our default recovery behaviors here
  /*void UpoNavigation::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }*/

  void UpoNavigation::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    //state_ = PLANNING;
    //recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("upo_navigation","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }
};
