#include <upo_rrt_planners/ros/RRT_ros_wrapper.h>


/*class ValidityChecker2 : public upo_RRT::StateChecker
{
	public:
    
	ValidityChecker2(tf::TransformListener* tf, WorldModel* loc_world_model, WorldModel* glob_world_model, double ins_rad, double cir_rad) : StateChecker()
    {
		inscribed_rad = ins_rad;
		circumscribed_rad = cir_rad;
		wm_local = loc_world_model;
		wm_global = glob_world_model;
		mytf = tf;
    }
	
	bool isValid(const State* s) const
	{
		//Take the values of the state
		float x_i = state->getX();
		float y_i = state->getY();
		//float h_i = state->getYaw();

		//Coordinates X and Y are in the robot local frame (base_link), we transform them to odom
		geometry_msgs::PointStamped p_in;
		p_in.header.frame_id = "base_link"; 
		p_in.header.stamp = ros::Time(0);
		p_in.point.x = x_i;
		p_in.point.y = y_i;


		std::vector<geometry_msgs::Point> my_footprint;
		double local_cost = 0.0;
		double global_cost = 0.0;


		//Global costmap ------------------------------------------
		geometry_msgs::PointStamped p_out_map;
		try {					
			mytf->transformPoint("map", p_in, p_out_map); 
		}catch (tf::TransformException ex){
			ROS_WARN("isValid. x:%.2f, y:%.2f. TransformException: %s",x_i, y_i, ex.what());
			return false;
		}
		geometry_msgs::Point robot_map;
		robot_map.x = (double)p_out_map.point.x;
		robot_map.y = (double)p_out_map.point.y;
		//we add just one point to the footprint since the footprintCost
		//method assumes a circular robot if the footprint size is less
		//than 3 
		my_footprint.push_back(robot_map);
		try{ 
			global_cost = global_wm->footprintCost(robot_map, my_footprint, ins_rad, cir_rad);
		} catch(...){
				ROS_ERROR("ERROR obtaining footprint cost!!!");
				return false;
		}
		if(global_cost < 0 || global_cost >= 253){
			//printf("No valid in Global Costmap. x:%.2f, y:%.2f. cost: %.2f\n", x_i, y_i, global_cost);
			return false;
		}


		//Local costmap---------------------------------------------
		geometry_msgs::PointStamped p_out_odom;
		try {					
			mytf->transformPoint("odom", p_in, p_out_odom); 
		}catch (tf::TransformException ex){
			ROS_WARN("isValid. x:%.2f, y:%.2f. TransformException: %s",x_i, y_i, ex.what());
			return false;
		}
		geometry_msgs::Point robot_odom;
		robot_odom.x = (double)p_out_odom.point.x;
		robot_odom.y = (double)p_out_odom.point.y;
		//we add just one point to the footprint since the footprintCost
		//method assumes a circular robot if the footprint size is less
		//than 3 
		my_footprint.clear();
		my_footprint.push_back(robot_odom);
		try{ 
			local_cost = local_wm->footprintCost(robot_odom, my_footprint, ins_rad, cir_rad);
		} catch(...){
				ROS_ERROR("IsValid. ERROR obtaining footprint cost!!!");
				return false;
		}
		if(local_cost < 0 || local_cost >= 253){ //>= 253
		  //printf("No Valid in Local Costmap. x:%.2f, y:%.2f. local_cost: %2f\n", x_i, y_i, local_cost);
		  return false;
		}

		

		return true;
	}


	private:
	double inscribed_rad;
	double circumscribed_rad;
	WorldModel* wm_local;
	WorldModel* wm_global;
	tf::TransformListener* mytf;

};*/








upo_RRT_ros::RRT_ros_wrapper::RRT_ros_wrapper() :
global_costmap_ros_(NULL), local_costmap_ros_(NULL), tf_(NULL) {}



upo_RRT_ros::RRT_ros_wrapper::RRT_ros_wrapper(tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap_ros, costmap_2d::Costmap2DROS* local_costmap_ros)
{
	tf_ = tf;
	global_costmap_ros_ = global_costmap_ros;
	local_costmap_ros_ = local_costmap_ros;

  	//initialize the copy of the costmaps the RRT will use
   	global_costmap_ = global_costmap_ros_->getCostmap();
	local_costmap_ = local_costmap_ros_->getCostmap();

	setup();

}

upo_RRT_ros::RRT_ros_wrapper::~RRT_ros_wrapper() {
		delete checker_;
		delete rrt_planner_;
}


void upo_RRT_ros::RRT_ros_wrapper::setup()
{
	//ros::NodeHandle private_nh("~/upo_navigation/RRT_ros_wrapper");
	ros::NodeHandle private_nh("~/RRT_ros_wrapper");
	//ros::NodeHandle private_nh("~");



	private_nh.param("rrt_planner_type", rrt_planner_type_, 1);
	printf("RRT_ros_wrapper. rrt_planner_type = %i\n",  rrt_planner_type_);
      	
  	//UvA parameters
	//bool use_uva_lib;
   	private_nh.param("use_feature_lib", use_uva_lib_, false);
	
	//RRT 
 	private_nh.param("rrt_solve_time", solve_time_, (float)0.8);
	//float goal_bias;
	private_nh.param("rrt_goal_bias", goal_bias_, (float)0.1);
	//float max_range;
	private_nh.param("rrt_max_insertion_dist", max_range_, (float)0.2);
	
	float robot_radius;
	private_nh.param("robot_radius", robot_radius, (float) 0.3);
	
	//RRT*
	//bool rrtstar_use_k_nearest = true;
	//bool rrtstar_path_biasing = false;
	//float rrtstar_path_bias = 0.0;
	//float rrtstar_stddev_bias = 0.0;
	//float rrtstar_rewire_factor = 0.0;
	if(rrt_planner_type_ == 2 || rrt_planner_type_ == 4) {
		private_nh.param("rrtstar_use_k_nearest", rrtstar_use_k_nearest_, true);
		private_nh.param("rrtstar_path_biasing", rrtstar_path_biasing_, false);
		private_nh.param("rrtstar_path_bias", rrtstar_path_bias_, (float)0.5);
		private_nh.param("rrtstar_stddev_bias", rrtstar_stddev_bias_, (float)0.8);
		private_nh.param("rrtstar_rewire_factor", rrtstar_rewire_factor_, (float)1.1);
	}
	
	
	//if RRT or RRT* are kinodynamics
	//float kino_linAcc, kino_angAcc;
	//int kino_minControlSteps, kino_maxControlSteps;
	//int kino_steeringType;
	if(rrt_planner_type_ > 2) {
		
		private_nh.param("kino_time_step", kino_timeStep_, (float)0.067);
		private_nh.param("kino_min_control_steps", kino_minControlSteps_, 5);
		private_nh.param("kino_max_control_steps", kino_maxControlSteps_, 30);
		//Robot accelerations
		private_nh.param("kino_linear_acc", kino_linAcc_, (float) 0.6);
		private_nh.param("kino_angular_acc", kino_angAcc_, (float) 1.57);
		
		private_nh.param("kino_steering_type", kino_steeringType_, 1); 
	}
	

	//RRT State Space
	private_nh.param("rrt_dimensions", dimensions_, 2);
  	private_nh.param("rrt_size_x", size_x_, (float)5.0);
	private_nh.param("rrt_size_y", size_y_, (float)5.0);
	//float xy_res;
	private_nh.param("rrt_xy_resolution", xy_res_, (float)0.1);
	//float yaw_res;
	private_nh.param("rrt_yaw_resolution", yaw_res_, (float)0.02);
	//float min_lin_vel;
	private_nh.param("rrt_min_linear_vel", min_lin_vel_, (float)0.0);
	//float max_lin_vel;
	private_nh.param("rrt_max_linear_vel", max_lin_vel_, (float)0.5);
	//float lin_vel_res;
	private_nh.param("rrt_lin_vel_resolution", lin_vel_res_, (float)0.05);
	//float max_ang_vel;
	private_nh.param("rrt_max_angular_vel", max_ang_vel_, (float)0.5);
	//float ang_vel_res;
	private_nh.param("rrt_ang_vel_resolution", ang_vel_res_, (float)0.1);
	//float goal_xy_tol;
	private_nh.param("rrt_goal_xy_tol", goal_xy_tol_, (float)0.15);
	//float goal_th_tol;
	private_nh.param("rrt_goal_th_tol", goal_th_tol_, (float)0.15);
	//int nn_params;
	private_nh.param("rrt_nn_type", nn_params_, 1);
	//int distanceType;
	private_nh.param("distance_type", distanceType_, 1);
	private_nh.param("motion_cost_type", motionCostType_, 1);
	
	//Visualization
  	private_nh.param("visualize_rrt_tree", visualize_tree_, false);
   	private_nh.param("visualize_nav_costmap", visualize_costmap_, false);
	private_nh.param("show_rrt_statistics", show_statistics_, false);
	private_nh.param("equal_path_percentage", equal_path_percentage_, (float)0.5);
	private_nh.param("rrt_interpolate_path_dist", interpolate_path_distance_, (float)0.05);
	private_nh.param("show_intermediate_states", show_intermediate_states_, false);
	
	
	//if the planner is an RRT, the nav costmap can not be visualized
	if(rrt_planner_type_ == 1 || rrt_planner_type_ == 3)
		visualize_costmap_ = false;
		
	ros::NodeHandle n;
	if(visualize_costmap_) {
		costmap_pub_ = n.advertise<nav_msgs::OccupancyGrid>("rrt_costmap", 1);
	}
	if(visualize_tree_) {
		tree_pub_ = n.advertise<visualization_msgs::Marker>("rrt_tree", 1);
	}

	local_goal_pub_ = n.advertise<visualization_msgs::Marker>("rrt_goal_marker", 1);
	path_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_points", 1);
	path_interpol_points_pub_ = n.advertise<visualization_msgs::Marker>("rrt_path_interpol_points", 1);

	if(size_x_ != size_y_) {
		ROS_ERROR("X size and Y size of the State Space has to be equal!!!");
		return;
	}

	//This is not working properly
	//double irad, crad;
	//costmap_2d::calculateMinAndMaxDistances(footprint_, irad, crad);
	//inscribed_radius_ = irad;
	//circumscribed_radius_ = crad;
	inscribed_radius_  = robot_radius;
	circumscribed_radius_ = robot_radius;

	checker_ = new ValidityChecker(tf_, local_costmap_, global_costmap_, &footprint_, inscribed_radius_, size_x_, size_y_, dimensions_, distanceType_);
	
	
	switch(rrt_planner_type_)
	{
		// ----- simple RRT --------------------
		case 1:
			printf("\n-------- Using simple RRT planner ----------\n");
			rrt_planner_ = new upo_RRT::SimpleRRT();
			rrt_planner_->as<upo_RRT::SimpleRRT>()->setMaxRange(max_range_);
			break;
			
		// ----- simple RRT* --------------------
		case 2:
			printf("\n-------- Using simple RRT* planner ----------\n");
			rrt_planner_ = new upo_RRT::SimpleRRTstar();
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->set_usePathBiasing(rrtstar_path_biasing_);
			rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			if(rrtstar_path_biasing_) {
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias(rrtstar_path_bias_);
				rrt_planner_->as<upo_RRT::SimpleRRTstar>()->setPathBias_stddev(rrtstar_stddev_bias_);
			}
			break;
		
		// ----- kinodynamic RRT --------------------
		case 3:
			printf("\n-------- Using Kinodynamic RRT planner ----------\n");
			rrt_planner_ = new upo_RRT::RRT();
			rrt_planner_->as<upo_RRT::RRT>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<upo_RRT::RRT>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<upo_RRT::RRT>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			break;
			
		// ----- kinodynamic RRT* --------------------
		case 4:
			printf("\n-------- Using Kinodynamic RRT* planner ----------\n");
			rrt_planner_ = new upo_RRT::RRTstar();
			rrt_planner_->as<upo_RRT::RRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			rrt_planner_->as<upo_RRT::RRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			rrt_planner_->as<upo_RRT::RRTstar>()->set_usePathBiasing(rrtstar_path_biasing_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setSteeringType(kino_steeringType_);
			rrt_planner_->as<upo_RRT::RRTstar>()->setMotionCostType(motionCostType_); 
			if(rrtstar_path_biasing_) {
				rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias(rrtstar_path_bias_);
				rrt_planner_->as<upo_RRT::RRTstar>()->setPathBias_stddev(rrtstar_stddev_bias_);
			}
			break;
			
		// ----- kinodynamic simplified RRT* --------------------
		case 5:
			printf("\n-------- Using Kinodynamic simplified RRT* planner ----------\n");
			rrt_planner_ = new upo_RRT::HalfRRTstar();
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setMaxRange(max_range_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setTimeStep(kino_timeStep_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setControlSteps(kino_minControlSteps_, kino_maxControlSteps_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setRobotAcc(kino_linAcc_, kino_angAcc_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->set_useKnearest(rrtstar_use_k_nearest_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->set_usePathBiasing(rrtstar_path_biasing_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setRewireFactor(rrtstar_rewire_factor_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setSteeringType(kino_steeringType_);
			rrt_planner_->as<upo_RRT::HalfRRTstar>()->setMotionCostType(motionCostType_); 
			if(rrtstar_path_biasing_) {
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias(rrtstar_path_bias_);
				rrt_planner_->as<upo_RRT::HalfRRTstar>()->setPathBias_stddev(rrtstar_stddev_bias_);
			}
			break;
			
		default:
			printf("\n-------- Using default simple RRT planner ----------\n");
			rrt_planner_ = new upo_RRT::SimpleRRT();
			rrt_planner_->as<upo_RRT::SimpleRRT>()->setMaxRange(max_range_);
	}
	
	rrt_planner_->setup(checker_, nn_params_, dimensions_, size_x_, size_y_, xy_res_, yaw_res_, min_lin_vel_, max_lin_vel_, lin_vel_res_, max_ang_vel_, ang_vel_res_);
				
	rrt_planner_->setGoalBias(goal_bias_);
	rrt_planner_->setGoalTolerance(goal_xy_tol_, goal_th_tol_);
	rrt_planner_->setStoreTree(visualize_tree_);
	
}



std::vector<geometry_msgs::PoseStamped> upo_RRT_ros::RRT_ros_wrapper::RRT_plan(geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal, float start_lin_vel, float start_ang_vel)
{
	
	if(!rrt_planner_->setStartAndGoal(start.x, start.y, start.theta, goal.x, goal.y, goal.theta)){
		ROS_ERROR("RRT_plan. Goal state is not valid!!!");
		rrt_plan_.clear();
		geometry_msgs::PoseStamped p;
		p.pose.position.x = -100.0;
		p.pose.position.y = -100.0;
		p.pose.position.z = -100.0;
		rrt_plan_.push_back(p);
		return rrt_plan_;
	}
	
	upo_RRT::State* g = NULL;
	
	//if we use costs
	if(rrt_planner_type_ == 2 || rrt_planner_type_ >= 4) {
		//Set the goal in the state checker
		g = new upo_RRT::State(goal.x, goal.y, goal.theta);
		checker_->setGoal(g);
	}
	
	//visualize rrt goal
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link"; //robot_base_frame_ = "/base_link"
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = goal.x;
	marker.pose.position.y = goal.y;
	marker.pose.position.z = 0.5;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta);
	// Set the scale of the marker 
	marker.scale.x = 0.7;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.5f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
	// Publish the marker
	local_goal_pub_.publish(marker);
	
	ros::Time time = ros::Time::now();

	//-------- GET THE RRT PATH ------------------------------
	
	//computations needed before starting planning
	//In this case, we calculate the gaussian functions over the current people
	checker_->preplanning_computations();
	
	std::vector<upo_RRT::Node> path;
	switch(rrt_planner_type_)
	{
		case 1:
			path = rrt_planner_->as<upo_RRT::SimpleRRT>()->solve(solve_time_);
			break;
			
		case 2:
			path = rrt_planner_->as<upo_RRT::SimpleRRTstar>()->solve(solve_time_);
			break;
			
		case 3:
			path = rrt_planner_->as<upo_RRT::RRT>()->solve(solve_time_);
			break;
			
		case 4:
			path = rrt_planner_->as<upo_RRT::RRTstar>()->solve(solve_time_);
			break;
			
		case 5:
			path = rrt_planner_->as<upo_RRT::HalfRRTstar>()->solve(solve_time_);
			break;
			
		default:
			path = rrt_planner_->as<upo_RRT::SimpleRRT>()->solve(solve_time_);
	}

	//--------------------------------------------------------
	//TODO optional: Program some tools for path smoothing (splines?)
	//--------------------------------------------------------

	if(show_statistics_) {
		upo_RRT::Planner::statistics stats = rrt_planner_->getStatistics();
		printf("Planning time:   %.4f secs\n", stats.planning_time);
		printf("First sol time:  %.4f secs\n", stats.first_sol_time);
		printf("Total samples:   %u \n", stats.total_samples);
		printf("Valid samples:   %u \n", stats.valid_samples);
		printf("Goal samples:    %u \n",  stats.goal_samples);
		printf("Tree nodes:      %u \n",  stats.tree_nodes);
		printf("Path nodes:      %u \n\n",  stats.path_nodes);
	}
	
	// Build the path in ROS format
	rrt_plan_.clear();
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";
	pose.header.stamp = time;
	int cont = 0;
	for(int i=path.size()-1; i>=0; --i)
	{
		pose.pose.position.z = 0.1;
		pose.pose.position.x = (double)path[i].getState()->getX();
		pose.pose.position.y = (double)path[i].getState()->getY();
		try {
			pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)path[i].getState()->getYaw());
		} catch (tf::TransformException ex) {
			printf("TransformException in getting sol path: %s",ex.what());
		}
		
		rrt_plan_.push_back(pose);
		//printf("Getting coordinates of node %i\n", cont);
		
		if(show_intermediate_states_ && path[i].hasIntermediateStates()) {	
			std::vector<upo_RRT::State>* inter = path[i].getIntermediateStates();
			for(unsigned int j=0; j<inter->size(); j++) 
			{
				//if(i != 0 || j != (inter.size()-1)) {
					pose.pose.position.x = (double)inter->at(j).getX();
					pose.pose.position.y = (double)inter->at(j).getY();
					try {
						pose.pose.orientation = tf::createQuaternionMsgFromYaw((double)inter->at(j).getYaw());
					} catch (tf::TransformException ex) {
						printf("TransformException in getting sol path: %s",ex.what());
					}
					pose.pose.position.z = 0.0;
					rrt_plan_.push_back(pose);
				//}
			}
				
		}
		cont++;
	}
	
	
	// Build the command list in ROS format
	std::vector<geometry_msgs::Twist> vels;
	if(rrt_planner_type_ > 2) {
		unsigned int cont = 0;
		unsigned int c = 0;
		for(unsigned int i=path.size()-1; i>0; i--)
		{
			std::vector<upo_RRT::Action>* acts = path[i].getAction();
			float vx, vy, vth; 
			unsigned int steps;
			for(unsigned int j=0; j<acts->size(); j++) {
				acts->at(j).getAction(vx, vy, vth, steps);
				geometry_msgs::Twist v;
				v.linear.x = vx;
				v.linear.y = vy;
				v.angular.z = vth;
				vels.push_back(v);
				cont+=steps;
				//printf("Action %u.  lv: %.2f, av: %.2f, Steps: %u\n", c, vx, vth, steps);
			}
			c++;
		}
		printf("Approximated Total Path Time: %.3f secs\n", kino_timeStep_*cont);
	}
	
	

	//Visualize the tree nodes of the resulting path
	visualization_msgs::Marker points;
	  
	points.header.frame_id = "base_link"; //robot_base_frame_; 
	points.header.stamp = time;
	points.ns = "basic_shapes";
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
		
	for(unsigned int i=0; i<rrt_plan_.size(); i++)
	{
		geometry_msgs::Point p = rrt_plan_[i].pose.position;
		points.points.push_back(p);
	}
	path_points_pub_.publish(points);
	

	if(visualize_tree_) 
		visualizeTree(time);
	
	
	if(visualize_costmap_) 
		publish_feature_costmap(time);
		
		
	if(interpolate_path_distance_ > 0.0)
	{
		rrt_plan_ = path_interpolation(rrt_plan_, interpolate_path_distance_);
		
		//Visualize the interpolated path nodes
		visualization_msgs::Marker mar;
		  
		mar.header.frame_id = "base_link"; //robot_base_frame_; 
		mar.header.stamp = time;
		mar.ns = "basic_shapes";
		mar.id = 2;
		mar.type = visualization_msgs::Marker::SPHERE_LIST;
		mar.action = visualization_msgs::Marker::ADD;
		mar.pose.position.x = 0.0;
		mar.pose.position.y = 0.0;
		mar.pose.position.z = 0.05; 
		mar.scale.x = 0.08;
		mar.scale.y = 0.08;
		mar.color.r = 0.0f;
		mar.color.g = 1.0f;
		mar.color.b = 1.0f;
		mar.color.a = 1.0;
		mar.lifetime = ros::Duration();
			
		for(unsigned int i=0; i<rrt_plan_.size(); i++)
		{
			geometry_msgs::Point p = rrt_plan_[i].pose.position;
			mar.points.push_back(p);
		}
		path_interpol_points_pub_.publish(mar);
	}

	if(g)
		delete g;
	//rrt_planner_->freeTreeMemory();
	
	//delete rrt_planner_;

	return rrt_plan_;
}





std::vector<geometry_msgs::PoseStamped> upo_RRT_ros::RRT_ros_wrapper::path_interpolation(std::vector<geometry_msgs::PoseStamped> path, float step_distance)
{
	std::vector<geometry_msgs::PoseStamped> pathnew;
	for(unsigned int i=0; i<path.size()-1; i++)
	{
		geometry_msgs::PoseStamped p1 = path[i];
		geometry_msgs::PoseStamped p2 = path[i+1];
		
		float dx = p2.pose.position.x - p1.pose.position.x;
		float dy = p2.pose.position.y - p1.pose.position.y;
		float dis = sqrt(dx*dx + dy*dy);
		
		geometry_msgs::PoseStamped intermediate = p1;
		
		pathnew.push_back(p1);
		
		float steps = dis/step_distance;
		//printf("Steps: %.2f\n", steps);
		if(steps > 1.0)
		{
			intermediate.header = path[0].header;
			intermediate.pose.position.z = p1.pose.position.z;
			for(unsigned int i=1; i<steps; i++)
			{
				float xr = (dx)*cos(0.0) + (dy)*sin(0.0);
				float yr =-(dx)*sin(0.0) + (dy)*cos(0.0);
				float tr = atan2(yr, xr);
				
				float newx = intermediate.pose.position.x + step_distance*cos(tr);
				float newy = intermediate.pose.position.y + step_distance*sin(tr);
				
				intermediate.pose.position.x = newx;
				intermediate.pose.position.y = newy;
				intermediate.pose.orientation = tf::createQuaternionMsgFromYaw(tr);
				pathnew.push_back(intermediate);
				
				dx = p2.pose.position.x - intermediate.pose.position.x;
				dy = p2.pose.position.y - intermediate.pose.position.y;
			}
		}
	}
	pathnew.push_back(path[path.size()-1]);
	printf("Path interpolation. Original size: %u, new size: %u\n", (unsigned int)path.size(), (unsigned int)pathnew.size());
	return pathnew;
}





float upo_RRT_ros::RRT_ros_wrapper::get_rrt_planning_radius() { return size_x_; }


void upo_RRT_ros::RRT_ros_wrapper::visualizeTree(ros::Time t) 
{
	//std::vector<upo_RRT::Node*> tree_nodes;
	//rrt_planner_->getTree(tree_nodes);
	std::vector<upo_RRT::State> tree_states = rrt_planner_->getTree();
		
	visualization_msgs::Marker edges;
	edges.header.frame_id = "base_link";
	edges.header.stamp = t;
	edges.ns = "rrt_tree";
	edges.id = 0;
	edges.type = visualization_msgs::Marker::LINE_LIST;
	edges.action = visualization_msgs::Marker::ADD;
	edges.pose.position.x = 0.0;
	edges.pose.position.y = 0.0;
	edges.pose.position.z = 0.05;
	edges.scale.x = 0.03;
	edges.scale.y = 0.03;
	edges.color.r = 1.0f;
	edges.color.g = 1.0f;
	edges.color.b = 1.0f;
	edges.color.a = 0.3;
	edges.lifetime = ros::Duration();
	
	//printf("VisualizeTree. tree nodes: %u\n", (unsigned int)tree_states.size());
	for(unsigned int i=0; i<tree_states.size()-1; i=i+2)
	{
		//printf("Node %u. x:%.2f, y:%.2f\n", i+1, tree_states[i].getX(), tree_states[i].getY());
		//Node* parent = tree_nodes[i].getParent();
		//if(parent != NULL) {
			geometry_msgs::Point p1;
			p1.x = tree_states[i].getX();
			p1.y = tree_states[i].getY();
			p1.z = 0.05;
			geometry_msgs::Point p2;
			p2.x = tree_states[i+1].getX();
			p2.y = tree_states[i+1].getY();
			p2.z = 0.05;
			edges.points.push_back(p1);
			edges.points.push_back(p2);
		//}							
	}
	tree_pub_.publish(edges);
}


void upo_RRT_ros::RRT_ros_wrapper::publish_feature_costmap(ros::Time t)
  {
		//Get the robot coordinates in odom frame
		tf::StampedTransform transform;
		try{
			tf_->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
			tf_->lookupTransform("/odom", "/base_link",  t, transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("Publish_feature_costmap. TF exception: %s",ex.what());
		}
	  
		nav_msgs::OccupancyGrid cmap;
		cmap.header.frame_id = "odom"; //"base_link";
		cmap.header.stamp = t;
		//time map_load_time. The time at which the map was loaded
		cmap.info.map_load_time = t;
		double cell_size = 0.25; // m/cell
		//float32 resolution. The map resolution [m/cell]
		cmap.info.resolution = cell_size;  //0.25 m/cell
		//uint32 width. Map width [cells]
		cmap.info.width = (size_x_*2.0)/cell_size;
		//uint32 height. Map height [cells]
		cmap.info.height = (size_y_*2.0)/cell_size;
		//geometry_msgs/Pose origin. The origin of the map [m, m, rad].  This is the real-world pose of the
		// cell (0,0) in the map.
		geometry_msgs::Pose p;
		p.position.x = transform.getOrigin().x()-size_x_;
		p.position.y = transform.getOrigin().y()-size_y_;
		p.position.z = 0.0;
		p.orientation = tf::createQuaternionMsgFromYaw(0.0); //robot_odom_h
		cmap.info.origin = p;
		//int8[] cmap.data. The map data, in row-major order, starting with (0,0).  Occupancy
		// probabilities are in the range [0,100].  Unknown is -1.
		std::vector<signed char> data; // size =(cmap.info.width*cmap.info.height)
		double cost = 0.0;
		for(int i=0; i<cmap.info.height; i++) 
		{
			for(unsigned int j=0; j<cmap.info.width; j++)
			{
					geometry_msgs::PoseStamped robotp;
					robotp.header.stamp = ros::Time();
					robotp.header.frame_id = "odom";
					robotp.pose.position.x = (transform.getOrigin().x()-size_x_ + cell_size*j) + (cell_size/2.0); //i
					robotp.pose.position.y = (transform.getOrigin().y()-size_y_ + cell_size*i) + (cell_size/2.0); //j
					robotp.pose.position.z = 0.0;
					tf::quaternionTFToMsg(transform.getRotation(), robotp.pose.orientation);
					
					geometry_msgs::PoseStamped robot_frame_pose = checker_->transformPoseTo(robotp, "base_link", true);
					upo_RRT::State* s = new upo_RRT::State(robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y, tf::getYaw(robot_frame_pose.pose.orientation)); 
					cost = checker_->getCost(s);
					//printf("publish_feature_map. x:%.2f, y:%.2f, cost:%.2f\n", robotp.pose.position.x, robotp.pose.position.y, cost);
						
					//Transform cost into the scale[0,100]
					data.push_back((int)round(cost*100.0));
			}
		}
		cmap.data = data;
		costmap_pub_.publish(cmap);
  }





std::vector<float> upo_RRT_ros::RRT_ros_wrapper::get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path)
{
	
	std::vector<float> feature_counts;
	geometry_msgs::PoseStamped mygoal = *goal;
	//Set the goal
	std::string goal_frame = goal->header.frame_id;
	if(goal_frame.compare("/base_link") != 0 && goal_frame.compare("base_link") != 0)
		mygoal = checker_->transformPoseTo(mygoal, "/base_link", false);
	upo_RRT::State* g;
	g = new upo_RRT::State(mygoal.pose.position.x, mygoal.pose.position.y, tf::getYaw(mygoal.pose.orientation));
	checker_->setGoal(g);
	
	  for(int i=0; i<path->size()-1; i++)
	  {
			//We have to transform the coordinates to robot frame (/base_link)
			geometry_msgs::PoseStamped robot_pose = path->at(i);
			if(robot_pose.header.frame_id.compare("/base_link") != 0 && robot_pose.header.frame_id.compare("base_link") != 0)
				robot_pose = checker_->transformPoseTo(robot_pose, "/base_link", false);

			geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
			if(robot_pose2.header.frame_id.compare("/base_link") != 0 && robot_pose2.header.frame_id.compare("base_link") != 0)
				robot_pose2 = checker_->transformPoseTo(robot_pose2, "/base_link", false);

			double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
			double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
			double d = hypot(dx,dy);

			if(!checker_->isQuaternionValid(robot_pose.pose.orientation)) {
				robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
			}
			
			upo_RRT::State* robot1;
			robot1 = new upo_RRT::State(robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation));
			std::vector<float> features1 = checker_->getFeatures(robot1);
			upo_RRT::State* robot2;
			robot2 = new upo_RRT::State(robot_pose2.pose.position.x, robot_pose2.pose.position.y, tf::getYaw(robot_pose2.pose.orientation));
			std::vector<float> features2 = checker_->getFeatures(robot2);

			if(i==0)
				feature_counts.assign(features1.size(), 0);
			
			switch(motionCostType_)
			{	
				case 1: 
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))/2.0;
					}
					break;
				
				case 2:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))*d/2.0;
					}
					break;
					
				case 3:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + ((features1.at(j) + features2.at(j))/2.0)*exp(d);
					}
					break;
					
				case 4:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j));
					}
					break;
			}
	  }
	  return feature_counts;
	
}


std::vector<float> upo_RRT_ros::RRT_ros_wrapper::get_feature_counts(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people)
{
	std::vector<float> feature_counts;
	
	//Set the goal
	geometry_msgs::PoseStamped mygoal = *goal;
	//Set the goal
	std::string goal_frame = goal->header.frame_id;
	if(goal_frame.compare("/base_link") != 0 && goal_frame.compare("base_link") != 0)
		mygoal = checker_->transformPoseTo(mygoal, "/base_link", false);
	upo_RRT::State* g;
	g = new upo_RRT::State(mygoal.pose.position.x, mygoal.pose.position.y, tf::getYaw(mygoal.pose.orientation));
	checker_->setGoal(g);
	
	  for(int i=0; i<path->size()-1; i++)
	  {
			//We have to transform the coordinates to robot frame (/base_link)
			geometry_msgs::PoseStamped robot_pose = path->at(i);
			if(robot_pose.header.frame_id.compare("/base_link") != 0 && robot_pose.header.frame_id.compare("base_link") != 0)
				robot_pose = checker_->transformPoseTo(robot_pose, "/base_link", false);

			geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
			if(robot_pose2.header.frame_id.compare("/base_link") != 0 && robot_pose2.header.frame_id.compare("base_link") != 0)
				robot_pose2 = checker_->transformPoseTo(robot_pose2, "/base_link", false);

			double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
			double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
			double d = hypot(dx,dy);

			if(!checker_->isQuaternionValid(robot_pose.pose.orientation)) {
				robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
			}
			
			upo_msgs::PersonPoseArrayUPO p = people->at(i);
			checker_->setPeople(p);
			
			upo_RRT::State* robot1;
			robot1 = new upo_RRT::State(robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation));
			std::vector<float> features1 = checker_->getFeatures(robot1);
			upo_RRT::State* robot2;
			robot2 = new upo_RRT::State(robot_pose2.pose.position.x, robot_pose2.pose.position.y, tf::getYaw(robot_pose2.pose.orientation));
			std::vector<float> features2 = checker_->getFeatures(robot2);

			if(i==0)
				feature_counts.assign(features1.size(), 0);
				
			switch(motionCostType_)
			{	
				case 1: 
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))/2.0;
					}
					break;
				
				case 2:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j))*d/2.0;
					}
					break;
					
				case 3:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + ((features1.at(j) + features2.at(j))/2.0)*exp(d);
					}
					break;
					
				case 4:
					for(unsigned int j=0; j<features1.size(); j++)
					{
						feature_counts.at(j) = feature_counts.at(j) + (features1.at(j) + features2.at(j));
					}
					break;
			}
	  }
	  return feature_counts;
	
}


float upo_RRT_ros::RRT_ros_wrapper::get_path_cost()
{
	return rrt_planner_->getCost();
}


float upo_RRT_ros::RRT_ros_wrapper::get_path_cost(geometry_msgs::PoseStamped* goal, std::vector<geometry_msgs::PoseStamped>* path, std::vector<upo_msgs::PersonPoseArrayUPO>* people)
{
	float cost = 0.0;
	
	//Set the goal
	geometry_msgs::PoseStamped mygoal = *goal;
	//Set the goal
	std::string goal_frame = goal->header.frame_id;
	if(goal_frame.compare("/base_link") != 0 && goal_frame.compare("base_link") != 0)
		mygoal = checker_->transformPoseTo(mygoal, "/base_link", false);
	upo_RRT::State* g;
	g = new upo_RRT::State(mygoal.pose.position.x, mygoal.pose.position.y, tf::getYaw(mygoal.pose.orientation));
	checker_->setGoal(g);
	
	  for(int i=0; i<path->size()-1; i++)
	  {
			//We have to transform the coordinates to robot frame (/base_link)
			geometry_msgs::PoseStamped robot_pose = path->at(i);
			if(robot_pose.header.frame_id.compare("/base_link") != 0 && robot_pose.header.frame_id.compare("base_link") != 0)
				robot_pose = checker_->transformPoseTo(robot_pose, "/base_link", false);

			geometry_msgs::PoseStamped robot_pose2 = path->at(i+1);
			if(robot_pose2.header.frame_id.compare("/base_link") != 0 && robot_pose2.header.frame_id.compare("base_link") != 0)
				robot_pose2 = checker_->transformPoseTo(robot_pose2, "/base_link", false);

			double dx = robot_pose.pose.position.x - robot_pose2.pose.position.x;
			double dy = robot_pose.pose.position.y - robot_pose2.pose.position.y;
			double d = hypot(dx,dy);

			if(!checker_->isQuaternionValid(robot_pose.pose.orientation)) {
				robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				printf("¡¡¡¡¡¡QUATERNION NO VALID!!!!!. Changing yaw to zero.\n");
			}
			
			upo_msgs::PersonPoseArrayUPO p = people->at(i);
			checker_->setPeople(p);
			
			upo_RRT::State* robot1;
			robot1 = new upo_RRT::State(robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation));
			float c1 = checker_->getCost(robot1);
			upo_RRT::State* robot2;
			robot2 = new upo_RRT::State(robot_pose2.pose.position.x, robot_pose2.pose.position.y, tf::getYaw(robot_pose2.pose.orientation));
			float c2 = checker_->getCost(robot2);

			
				
			switch(motionCostType_)
			{	
				case 1: 
					cost = cost + (c1 + c2)/2.0;
					break;
				
				case 2:
					cost = cost + (c1 + c2)*d/2.0;
					break;
					
				case 3:
					cost = cost + ((c1 + c2)/2.0)*exp(d);
					break;
					
				case 4:
					cost = cost + (c1 + c2);
					break;
					
				default:
					cost = cost + (c1 + c2)*d/2.0;
			}
	  }
	  
	  return cost;
	
}









