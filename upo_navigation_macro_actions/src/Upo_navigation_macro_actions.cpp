#include <upo_navigation_macro_actions/Upo_navigation_macro_actions.h>


/*
Status can take this values:
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
*/

//namespace macroactions {

Upo_navigation_macro_actions::Upo_navigation_macro_actions(tf::TransformListener& tf, upo_nav::UpoNavigation* nav)
{
	tf_listener_ = &tf;

	UpoNav_ = nav;

	ros::NodeHandle n("~");
	n.param<double>("secs_to_check_block", secs_to_check_block_, 5.0); //seconds
	n.param<double>("block_dist", block_dist_, 0.4); //meters
	n.param<double>("secs_to_wait", secs_to_wait_, 8.0);  //seconds
	n.param<double>("control_frequency", control_frequency_, 15.0);  
	n.param<int>("social_approaching_type", social_approaching_type_, 1);  
	n.param<bool>("check_battery_level", check_battery_level_, false);
	n.param<string>("battery_topic", battery_topic_, std::string("/teresa_diagnostics"));
	
	n.param<string>("yield_map", yieldmap_, std::string(""));
	std::string yieldpoint_file;
	n.param<string>("yield_points", yieldpoint_file, std::string(""));
	n.param<double>("secs_to_yield", secs_to_yield_, 8.0);
	
	if(yieldmap_.empty()) {
		ROS_ERROR("upo_navigation_macro_actions. ERROR: yieldmap is empty!!!");
		yield_ = new Yield();
	} else
		yield_ = new Yield(&yieldmap_, &yieldpoint_file);

	robot_inzone_ = false;
	robot_inzone2_ = false;
	person_inzone_ = false;

	//Assisted driving
	//as_ = new AssistedSteering(tf_listener_, odomtopic, lasertopic, in_cmd_vel_topic, out_cmd_vel_topic);
	//as_->pause();

	//Dynamic reconfigure
	dsrv_ = new dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>(n);
    dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig>::CallbackType cb = boost::bind(&Upo_navigation_macro_actions::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);


	ros::NodeHandle nh;
	people_sub_ = nh.subscribe("/people/navigation", 1, &Upo_navigation_macro_actions::peopleCallback, this); 
	rrtgoal_sub_ = nh.subscribe("/rrt_goal", 1, &Upo_navigation_macro_actions::rrtGoalCallback, this);
	amcl_sub_   = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &Upo_navigation_macro_actions::poseCallback, this);


	//Services for walking side by side
	start_client_ = nh.serviceClient<wsbs::start>("/wsbs/start");
	stop_client_ = nh.serviceClient<wsbs::stop>("/wsbs/stop");
	wsbs_status_sub_ = nh.subscribe<std_msgs::UInt8>("/wsbs/status", 1, &Upo_navigation_macro_actions::wsbsCallback, this);
	

	//Initialize action servers
	NWActionServer_ = new NWActionServer(nh1_, "NavigateWaypoint", boost::bind(&Upo_navigation_macro_actions::navigateWaypointCB, this, _1), false);
	NHActionServer_ = new NHActionServer(nh2_, "NavigateHome", boost::bind(&Upo_navigation_macro_actions::navigateHomeCB, this, _1), false);
	NITActionServer_ = new NITActionServer(nh3_, "NavigateInteractionTarget", boost::bind(&Upo_navigation_macro_actions::navigateInteractionTargetCB, this, _1), false);
	WActionServer_ = new WActionServer(nh4_, "Wait", boost::bind(&Upo_navigation_macro_actions::waitCB, this, _1), false);
	YActionServer_ = new YActionServer(nh5_, "Yield", boost::bind(&Upo_navigation_macro_actions::yieldCB, this, _1), false);
	WSActionServer_ = new WSActionServer(nh6_, "WalkSideBySide", boost::bind(&Upo_navigation_macro_actions::walkSideCB, this, _1), false);
	ASActionServer_ = new ASActionServer(nh7_, "AssistedSteering", boost::bind(&Upo_navigation_macro_actions::assistedSteeringCB, this, _1), false);
	
	NWActionServer_->start();
	NHActionServer_->start();
	NITActionServer_->start();
	WActionServer_->start();
	YActionServer_->start();
	WSActionServer_->start();
	ASActionServer_->start();

	ros::NodeHandle nodeh("~/RRT_ros_wrapper");
	nodeh.getParam("full_path_stddev", initial_stddev_);

}


Upo_navigation_macro_actions::~Upo_navigation_macro_actions()
{
	if(NWActionServer_)
      	delete NWActionServer_;
	if(NHActionServer_)
      	delete NHActionServer_;
	if(NITActionServer_)
      	delete NITActionServer_;
	if(WActionServer_)
      	delete WActionServer_;
	if(YActionServer_)
		delete YActionServer_;
	if(WSActionServer_)
		delete WSActionServer_;
	if(ASActionServer_)
		delete ASActionServer_;
      	
    if(yield_)
		delete yield_;
	//if(walk_)
	//	delete walk_;
	//if(as_)
	//	delete as_;
	if(UpoNav_)
		delete UpoNav_;
	if(dsrv_)
		delete dsrv_;
	
}




void Upo_navigation_macro_actions::reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig &config, uint32_t level){
    
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    
    control_frequency_ = config.control_frequency;
    secs_to_check_block_ = config.secs_to_check_block;
	block_dist_ = config.block_dist;
	secs_to_wait_ = config.secs_to_wait;
	social_approaching_type_ = config.social_approaching_type;
	secs_to_yield_ = config.secs_to_yield;
	//check_battery_level_ = config.check_battery_level;
	
}






/*
//Receive feedback messages from upo_navigation
void Upo_navigation_macro_actions::feedbackReceived(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
	
	pose_mutex_.lock();
	robot_pose_ = msg->feedback.base_position;
	pose_mutex_.unlock();
	if((unsigned int)(std::string(robot_pose_.header.frame_id).size()) < 3)
		robot_pose_.header.frame_id = "map";
}


//Receive status messages from upo_navigation
void Upo_navigation_macro_actions::statusReceived(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	unsigned int actions = msg->status_list.size();
	if(actions != 0)
	{
		status_mutex_.lock();
		nav_status_ = msg->status_list.at(0).status;
		nav_text_ = msg->status_list.at(0).text;
		goal_id_ = msg->status_list.at(0).goal_id.id;
		status_mutex_.unlock();
	} else {
		status_mutex_.lock();
		nav_status_ = -1;
		nav_text_ = " ";
		goal_id_ = " ";
		status_mutex_.unlock();
	}
}


//This topic publishes only when the action finishes (because of reaching the goal or cancelation)
void Upo_navigation_macro_actions::resultReceived(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
	action_end_ = true;
}*/



void Upo_navigation_macro_actions::navigateWaypointCB(const upo_navigation_macro_actions::NavigateWaypointGoal::ConstPtr& goal)
{

	printf("¡¡¡¡¡¡¡MacroAction navigatetoWaypoint  -->  started!!!!!!\n");
	

	bool ok = UpoNav_->executeNavigation(goal->target_pose); 
	if(!ok)
	{
		ROS_INFO("Setting ABORTED state 1");
		nwresult_.result = "Aborted. Navigation error";
		nwresult_.value = 2;
		NWActionServer_->setAborted(nwresult_, "Navigation aborted");
		return;
	}

	ros::Rate r(control_frequency_); 
	int pursue_status = 0;
	bool exit = false;
	ros::Time time_init = ros::Time::now();
	bool first = true;
	geometry_msgs::PoseStamped pose_init;
	//ros::WallTime startt;
    while(nh1_.ok())
    {
		//startt = ros::WallTime::now();
		
		if(NWActionServer_->isPreemptRequested()){
			
        	if(NWActionServer_->isNewGoalAvailable()){
				
				upo_navigation_macro_actions::NavigateWaypointGoal new_goal = *NWActionServer_->acceptNewGoal();

          		bool ok = UpoNav_->executeNavigation(new_goal.target_pose); 
          		if(!ok) {
					ROS_INFO("Setting ABORTED state 1");
					nwresult_.result = "Aborted. Navigation error";
					nwresult_.value = 2;
					NWActionServer_->setAborted(nwresult_, "Navigation aborted");
					return;
				}
				first = true;

       	 	}
        	else {
          		//if we've been preempted explicitly we need to shut things down
          		UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		nwresult_.result = "Preempted";
				nwresult_.value = 1;
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
				NWActionServer_->setPreempted(nwresult_, "Navigation preempted");
          		//we'll actually return from execute after preempting
          		return;
          		
        	}
      	}

		geometry_msgs::PoseStamped new_pose;
		pursue_status = UpoNav_->pathFollow(new_pose);
		if(first){
			pose_init = new_pose;
			time_init = ros::Time::now();
			first = false;
		}

		switch(pursue_status)
		{
			case -2:	//Error
				nwfeedback_.text = "Aborted";
				ROS_INFO("Setting ABORTED state");
				nwresult_.result = "Aborted. Navigation error 2";
				nwresult_.value = 3;
				NWActionServer_->setAborted(nwresult_, "Navigation aborted");
				exit = true;
				break;

			case -1:	//Error, try to replan
				nwfeedback_.text = "Replaning to continue navigating";
				break;

			case 0:		//Goal not reached, continue navigating
				nwfeedback_.text = "Navigating";
				break;

			case 1:		//Goal reached
				ROS_INFO("Setting SUCCEEDED state");
				nwresult_.result = "Navigation succeeded";
				nwresult_.value = 0;
				NWActionServer_->setSucceeded(nwresult_, "Goal Reached");
				nwfeedback_.text = "Succeeded";
				exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
				nwresult_.result = "Aborted. Navigation error 2";
				nwresult_.value = 3;
				nwfeedback_.text = "Unexpected pursue status value!!!";
				ROS_INFO("Setting ABORTED state");
				NWActionServer_->setAborted(nwresult_, "Navigation aborted because an unexpected pursue value was received");
				exit = true;
				
		}
		
		//push the feedback out
		nwfeedback_.base_position = new_pose;
		NWActionServer_->publishFeedback(nwfeedback_);

		if(exit)
			return;
			
		
		//changeParametersNarrowPlaces();
		changeParametersNarrowPlaces2();			

		boost::recursive_mutex::scoped_lock l(configuration_mutex_);

		//Check the yield situation
		pinzone_mutex_.lock();
		bool p_in = person_inzone_;
		//bool p_in2 = person_inzone2_;
		pinzone_mutex_.unlock();
		
		rinzone_mutex_.lock();
		bool r_in = robot_inzone_;
		rinzone_mutex_.unlock();
		//printf("robot_inzone: %i, person_inzone: %i\n", r_in, p_in);
		if(r_in && p_in && isYieldDirectionCorrect())
		{
			ROS_INFO("Setting ABORTED state because of No social path available (yielding)");
			nwresult_.result = "Aborted. No social Path Available";
			nwresult_.value = 4;
			NWActionServer_->setAborted(nwresult_, "Navigation aborted. No social path available. Yield");
			nwfeedback_.text = "No social path available";
			NWActionServer_->publishFeedback(nwfeedback_);
			UpoNav_->stopRRTPlanning();
			return;

		} else {
			//check the blocked situation.
			double time = (ros::Time::now() - time_init).toSec();
			//printf("now: %.2f, init: %.2f, time: %.2f secs\n", ros::Time::now().toSec(), time_init.toSec(), time);
			if(time > secs_to_check_block_) {
				double xinit = pose_init.pose.position.x;
				double yinit = pose_init.pose.position.y;
				double hinit = tf::getYaw(pose_init.pose.orientation);
				double xnow = new_pose.pose.position.x;
				double ynow = new_pose.pose.position.y;
				double hnow = tf::getYaw(new_pose.pose.orientation);
				double dist = sqrt(pow((xinit-xnow), 2) + pow((yinit-ynow), 2));
				double yaw_diff = fabs(angles::shortest_angular_distance(hnow, hinit));
				//printf("dist: %.2f, yaw_diff: %.2f\n", dist, yaw_diff);
				if(dist <= block_dist_ && yaw_diff < 0.79) { //0.79 = 45º
					ROS_INFO("Setting ABORTED state because of blocked situation");
					nwresult_.result = "Aborted. Blocked situation";
					nwresult_.value = 5;
					NWActionServer_->setAborted(nwresult_, "Navigation aborted. blocked");
					nwfeedback_.text = "Blocked";
					NWActionServer_->publishFeedback(nwfeedback_);
					UpoNav_->stopRRTPlanning();
					return;
				} else {
					pose_init = new_pose;
					time_init = ros::Time::now(); 
				}
			} 
		}
		


		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());

		r.sleep();
	}

	ROS_INFO("Setting ABORTED state");
	nwresult_.result = "Aborted. System is shuting down";
	nwresult_.value = 6;
	NWActionServer_->setAborted(nwresult_, "Navigation aborted because the node has been killed");

}




void Upo_navigation_macro_actions::navigateHomeCB(const upo_navigation_macro_actions::NavigateHomeGoal::ConstPtr& goal)
{

	printf("¡¡¡¡¡¡¡MacroAction NavigateHome  -->  started!!!!!!\n");
	
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	bool ok = UpoNav_->executeNavigation(goal->home_pose); 
	if(!ok)
	{
		ROS_INFO("Setting ABORTED state 1");
		nhresult_.result = "Aborted. Navigation error";
		nhresult_.value = 2;
		NHActionServer_->setAborted(nhresult_, "Navigation aborted");
		return;
	}

	ros::Rate r(control_frequency_); 
	int pursue_status = 0;
	bool exit = false;
	ros::Time time_init = ros::Time::now();
	bool first = true;
	geometry_msgs::PoseStamped pose_init;
	//ros::WallTime startt;
    while(nh2_.ok())
    {
		//startt = ros::WallTime::now();
		
		if(NHActionServer_->isPreemptRequested()){
			
        	if(NHActionServer_->isNewGoalAvailable()){
				
				upo_navigation_macro_actions::NavigateHomeGoal new_goal = *NHActionServer_->acceptNewGoal();

          		bool ok = UpoNav_->executeNavigation(new_goal.home_pose); 
          		if(!ok){
					ROS_INFO("Setting ABORTED state 1");
					nhresult_.result = "Aborted. Navigation error";
					nhresult_.value = 2;
					NHActionServer_->setAborted(nhresult_, "Navigation aborted");
					return;
				}
				first = true;
       	 	}
        	else {
          		//if we've been preempted explicitly we need to shut things down
          		UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
          		nhresult_.result = "Preempted";
				nhresult_.value = 1;
				NHActionServer_->setPreempted(nhresult_, "Navigation preempted");

          		//we'll actually return from execute after preempting
          		return;
          		
        	}
      	}

		geometry_msgs::PoseStamped new_pose;
		pursue_status = UpoNav_->pathFollow(new_pose);
		if(first){
			pose_init = new_pose;
			time_init = ros::Time::now();
			first = false;
		}

		switch(pursue_status)
		{
			case -2:	//Error
				nhfeedback_.text = "Aborted";
				ROS_INFO("Setting ABORTED state");
				nhresult_.result = "Aborted. Navigation error 2";
				nhresult_.value = 3;
				NHActionServer_->setAborted(nhresult_, "Navigation aborted");
				exit = true;
				break;

			case -1:	//Error, try to replan
				nhfeedback_.text = "Replaning to continue navigating";
				break;

			case 0:		//Goal not reached, continue navigating
				nhfeedback_.text = "Navigating";
				break;

			case 1:		//Goal reached
				ROS_INFO("Setting SUCCEEDED state");
				nhresult_.result = "Succeeded";
				nhresult_.value = 0;
				NHActionServer_->setSucceeded(nhresult_, "Goal Reached");
				nhfeedback_.text = "Succeeded";
				exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
				nhfeedback_.text = "Unexpected pursue status value!!!";
				ROS_INFO("Setting ABORTED state");
				nhresult_.result = "Aborted. Navigation error 2";
				nhresult_.value = 3;
				NHActionServer_->setAborted(nhresult_, "Navigation aborted because an unexpected pursue value was received");
				exit = true;
		}
		

		//push the feedback out
		nhfeedback_.base_position = new_pose;
		NHActionServer_->publishFeedback(nhfeedback_);

		if(exit)
			return;
		
		
		//changeParametersNarrowPlaces();
		changeParametersNarrowPlaces2();
		
		boost::recursive_mutex::scoped_lock l(configuration_mutex_);	
			
		//Check the yield situation
		pinzone_mutex_.lock();
		bool p_in = person_inzone_;
		//bool p_in2 = person_inzone2_;
		pinzone_mutex_.unlock();
		
		rinzone_mutex_.lock();
		bool r_in = robot_inzone_;
		rinzone_mutex_.unlock();
		
		if(r_in && p_in)
		{
			ROS_INFO("Setting ABORTED state because of No social path available (yielding)");
			nhresult_.result = "Aborted. No social path available";
			nhresult_.value = 4;
			NHActionServer_->setAborted(nhresult_, "Navigation aborted. No social path available. Yield");
			nhfeedback_.text = "No social path available";
			NHActionServer_->publishFeedback(nhfeedback_);
			UpoNav_->stopRRTPlanning();
			return;

		} else {
			//check the blocked situation.
			double time = (ros::Time::now() - time_init).toSec();
			//printf("now: %.2f, init: %.2f, time: %.2f secs\n", ros::Time::now().toSec(), time_init.toSec(), time);
			if(time > secs_to_check_block_) {
				double xinit = pose_init.pose.position.x;
				double yinit = pose_init.pose.position.y;
				double hinit = tf::getYaw(pose_init.pose.orientation);
				double xnow = new_pose.pose.position.x;
				double ynow = new_pose.pose.position.y;
				double hnow = tf::getYaw(new_pose.pose.orientation);
				double dist = sqrt(pow((xinit-xnow), 2) + pow((yinit-ynow), 2));
				double yaw_diff = fabs(angles::shortest_angular_distance(hnow, hinit));
				//printf("dist: %.2f, yaw_diff: %.2f\n", dist, yaw_diff);
				if(dist <= block_dist_ && yaw_diff < 0.79) { //0.79 = 45º
					ROS_INFO("Setting ABORTED state because of blocked situation");
					nhresult_.result = "Aborted. Blocked situation";
					nhresult_.value = 5;
					NHActionServer_->setAborted(nhresult_, "Navigation aborted. blocked");
					nhfeedback_.text = "Blocked";
					NHActionServer_->publishFeedback(nhfeedback_);
					UpoNav_->stopRRTPlanning();
					return;
				
				} else {
					pose_init = new_pose;
					time_init = ros::Time::now(); 
				}
			} 
		}

		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());

		r.sleep();
	}

	ROS_INFO("Setting ABORTED state");
	nhresult_.result = "Aborted. system is shuting down";
	nhresult_.value = 6;
	NHActionServer_->setAborted(nhresult_, "Navigation aborted because the node has been killed");

}







void Upo_navigation_macro_actions::navigateInteractionTargetCB(const upo_navigation_macro_actions::NavigateInteractionTargetGoal::ConstPtr& goal)
{
	printf("¡¡¡¡¡¡¡MacroAction NavigateToInteractionTarget  -->  started!!!!!!\n");
	
	//Disable uva_features if it is active
	ros::NodeHandle n("/upo_navigation_macro_actions/Navigation_features");
	bool uva_feat;
	n.param<bool>("use_uva_features", uva_feat, false);
	
	
	//Disable uva_features in order to use the upo gaussians for approaching
	if(uva_feat) {
		printf("Disabling uva_features_set\n");
		//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=False");
		reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("0"), BOOL_TYPE);
	} 
	

	upo_msgs::PersonPoseUPO p = goal->person;
	//printf("Received person with Id: %i and yaw: %.3f\n", p.id, tf::getYaw(p.orientation)); 
	/*p.header = goal->person_pose.header;
	p.id = -1;
	p.vel = -1;
	p.position = goal->person_pose.pose.position;
	p.orientation = goal->person_pose.pose.orientation;
	*/
	
	geometry_msgs::PoseStamped goal_pose;
	
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);
	
	goal_pose = approachIT(&p);
	if(goal_pose.header.frame_id == "bad") {
		ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state. Person not found");
		nitresult_.result = "Aborted. Navigation error";
		nitresult_.value = 2;
		NITActionServer_->setAborted(nitresult_, "Navigation aborted because of IT was lost");
		
		if(uva_feat) {
			printf("Enabling uva_features_set\n");
			//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
			reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
		} 
		//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
		reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
		return;
	}
	if(social_approaching_type_ != 1) {
		//char buffer [20]; 
		//char *intStr = itoa(p.id, buffer, 10);
		//string str = string(intStr);
		std::string str = std::to_string(p.id);
		reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), str, INT_TYPE);
	}

	//printf("1. Goalpose x:%.2f, y:%.2f, frame_id:%s\n", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.header.frame_id.c_str());

	//Enviar goal 
	bool ok = UpoNav_->executeNavigation(goal_pose); 
	if(!ok)
	{
		bool ok2 = false;
		if(social_approaching_type_ > 1) {
			social_approaching_type_ = 1;
			goal_pose = approachIT(&p);
			if(goal_pose.header.frame_id == "bad") 
				ok2 = UpoNav_->executeNavigation(goal_pose); 
			social_approaching_type_ = 2;
		}
		else if(!ok2)  {
			ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state 1");
			nitresult_.result = "Aborted. Navigation error";
			nitresult_.value = 2;
			NITActionServer_->setAborted(nitresult_, "Navigation aborted");
			if(uva_feat) {
				printf("Enabling uva_features_set\n");
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
			}
			//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
			reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
			return;
		}
	}


	ros::Rate r(control_frequency_); 
	int pursue_status = 0;
	bool exit = false;
	ros::Time time_init = ros::Time::now();
	bool first = true;
	geometry_msgs::PoseStamped pose_init;
	//ros::WallTime startt;
    while(nh3_.ok())
    {
		//startt = ros::WallTime::now();
		
		if(NITActionServer_->isPreemptRequested()){
			
        	if(NITActionServer_->isNewGoalAvailable()){
				
				upo_navigation_macro_actions::NavigateInteractionTargetGoal new_goal = *NITActionServer_->acceptNewGoal();

				/*p.header = new_goal.person_pose.header;
				p.id = -1;
				p.vel = -1;
				p.position = new_goal.person_pose.pose.position;
				p.orientation = new_goal.person_pose.pose.orientation;
				*/
				p = new_goal.person;
				goal_pose = approachIT(&p);
				if(goal_pose.header.frame_id == "bad") {
					ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state. Person not found");
					nitresult_.result = "Aborted. Navigation error";
					nitresult_.value = 2;
					NITActionServer_->setAborted(nitresult_, "Navigation aborted because of IT was lost");
					if(uva_feat) {
						printf("Enabling uva_features_set\n");
						//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
						reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
					}
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
					return;
				}
				if(social_approaching_type_ != 1){
					//char buffer [20]; 
					//char *intStr = itoa(p.id, buffer, 10);
					//string str = string(intStr);
					std::string str = std::to_string(p.id);
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), str, INT_TYPE);
				}
				//Enviar a new goal 
				bool ok = UpoNav_->executeNavigation(goal_pose); 
				if(!ok)
				{
					bool ok2 = false;
					if(social_approaching_type_ > 1) {
						social_approaching_type_ = 1;
						goal_pose = approachIT(&p);
						if(goal_pose.header.frame_id == "bad") 
							ok2 = UpoNav_->executeNavigation(goal_pose); 
						social_approaching_type_ = 2;
					}
					else if(!ok2)  {
						ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state 1");
						nitresult_.result = "Aborted. Navigation error";
						nitresult_.value = 2;
						NITActionServer_->setAborted(nitresult_, "Navigation aborted");
						if(uva_feat) {
							printf("Enabling uva_features_set\n");
							//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
							reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
						}
						//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
						reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
						return;
					}
				}
				first = true;
          		
       	 	} else {
          		//if we've been preempted explicitly we need to shut things down
          		UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
          		nitresult_.result = "Preempted";
				nitresult_.value = 1;
				NITActionServer_->setPreempted(nitresult_, "Navigation preempted");
				if(uva_feat) {
					printf("Enabling uva_features_set\n");
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
				}
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
          		//we'll actually return from execute after preempting
          		return;
        	}
      	}

		geometry_msgs::PoseStamped new_g;
		//printf("1. Person p id:%i, ", p.id);
		new_g = approachIT(&p);
		//printf(" found id:%i\n", p.id);
		if(new_g.header.frame_id == "bad") {
			ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state. Person not found");
			nitresult_.result = "Aborted. Navigation error";
			nitresult_.value = 2;
			NITActionServer_->setAborted(nitresult_, "Navigation aborted because of IT was lost");
			if(uva_feat) {
				printf("Enabling uva_features_set\n");
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
			}
			//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
			reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
			return;
		}
		
		float min_dist = 0.60;
		float dist = sqrt((goal_pose.pose.position.x-new_g.pose.position.x)*(goal_pose.pose.position.x-new_g.pose.position.x) 
				+ (goal_pose.pose.position.y-new_g.pose.position.y)*(goal_pose.pose.position.y-new_g.pose.position.y));
		if(dist >= min_dist){
			//Send new goal
			goal_pose = new_g; 
			if(social_approaching_type_ != 1){
				std::string str = std::to_string(p.id);
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), str, INT_TYPE);
			}
			
			bool ok = UpoNav_->executeNavigation(goal_pose); 
			if(!ok)
			{
				bool ok2 = false;
				if(social_approaching_type_ > 1) {
					social_approaching_type_ = 1;
					goal_pose = approachIT(&p);
					if(p.id != -100)
						ok2 = UpoNav_->executeNavigation(goal_pose); 
					social_approaching_type_ = 2;
				}
				else if(!ok2)  {
					ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state 1");
					nitresult_.result = "Aborted. Navigation error";
					nitresult_.value = 2;
					NITActionServer_->setAborted(nitresult_, "Navigation aborted");
					if(uva_feat) {
						printf("Enabling uva_features_set\n");
						//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
						reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
					}
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
					return;
				}
			}			
		}
		

		geometry_msgs::PoseStamped new_pose;
		pursue_status = UpoNav_->pathFollow(new_pose);
		if(first){
			pose_init = new_pose;
			time_init = ros::Time::now();
			first = false;
		}

		
		switch(pursue_status)
		{
			case -2:	//Error
				nitfeedback_.text = "Aborted";
				ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state. Status: -2");
				nitresult_.result = "Aborted. Navigation error 2";
				nitresult_.value = 3;
				NITActionServer_->setAborted(nitresult_, "Approaching aborted");
				if(uva_feat) {
					printf("Enabling uva_features_set\n");
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
				}
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
				exit = true;
				break;

			case -1:	//Error, try to replan
				nitfeedback_.text = "Replaning to continue the approaching";
				break;

			case 0:		//Goal not reached, continue navigating
				nitfeedback_.text = "Approaching";
				break;

			case 1:		//Goal reached
				ROS_INFO("NavigateToInteractionTarget. Setting SUCCEEDED state");
				nitresult_.result = "Succeeded";
				nitresult_.value = 0;
				NITActionServer_->setSucceeded(nitresult_, "IT Reached");
				nitfeedback_.text = "Succeeded";
				if(uva_feat) {
					printf("Enabling uva_features_set\n");
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
				}
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
				exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
				nitfeedback_.text = "Unexpected pursue status value!!!";
				ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state");
				nitresult_.result = "Aborted. Navigation error 2";
				nitresult_.value = 3;
				NITActionServer_->setAborted(nitresult_, "Navigation aborted because an unexpected pursue value was received");
				if(uva_feat) {
					printf("Enabling uva_features_set\n");
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
				}
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
				exit = true;
				break;
		}
		
		//push the feedback out
		nitfeedback_.base_position = new_pose;
		NITActionServer_->publishFeedback(nitfeedback_);

		if(exit) 
			return;
	
		//changeParametersNarrowPlaces();
		changeParametersNarrowPlaces2();
		
		//Check the yield situation
		pinzone_mutex_.lock();
		bool p_in = person_inzone_;
		//bool p_in2 = person_inzone2_;
		pinzone_mutex_.unlock();
		
		rinzone_mutex_.lock();
		bool r_in = robot_inzone_;
		rinzone_mutex_.unlock();
		
		if(r_in && p_in)
		{
			ROS_INFO("Setting ABORTED state because of No social path available (yielding)");
			nitresult_.result = "Aborted. No social path available";
			nitresult_.value = 4;
			NITActionServer_->setAborted(nitresult_, "Navigation aborted. No social path available. Yield");
			if(uva_feat) {
				printf("Enabling uva_features_set\n");
				//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
				reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
			}
			//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
			reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
			nitfeedback_.text = "No social path available";
			NITActionServer_->publishFeedback(nitfeedback_);
			UpoNav_->stopRRTPlanning();
			return;

		} else {
		
			//check the blocked situation.
			double time = (ros::Time::now() - time_init).toSec();
			//printf("now: %.2f, init: %.2f, time: %.2f secs\n", ros::Time::now().toSec(), time_init.toSec(), time);
			if(time > secs_to_check_block_) {
				double xinit = pose_init.pose.position.x;
				double yinit = pose_init.pose.position.y;
				double hinit = tf::getYaw(pose_init.pose.orientation);
				double xnow = new_pose.pose.position.x;
				double ynow = new_pose.pose.position.y;
				double hnow = tf::getYaw(new_pose.pose.orientation);
				double dist = sqrt(pow((xinit-xnow), 2) + pow((yinit-ynow), 2));
				double yaw_diff = fabs(angles::shortest_angular_distance(hnow, hinit));
				//printf("dist: %.2f, yaw_diff: %.2f\n", dist, yaw_diff);
				if(dist <= block_dist_ && yaw_diff < 0.79) { //0.79 = 45º
					ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state because of blocked situation");
					nitresult_.result = "Aborted. blocked situation";
					nitresult_.value = 5;
					NITActionServer_->setAborted(nitresult_, "Navigation aborted. blocked");
					if(uva_feat) {
						printf("Enabling uva_features_set\n");
						//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _use_uva_features:=True");
						reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("use_uva_features"), std::string("1"), BOOL_TYPE);
					}
					//system("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=-1");
					reconfigureParameters(std::string("/upo_navigation_macro_actions/Navigation_features"), std::string("interaction_target_id"), std::string("-1"), INT_TYPE);
					nitfeedback_.text = "Blocked";
					NITActionServer_->publishFeedback(nitfeedback_);
					UpoNav_->stopRRTPlanning();
					return;
				} else {
					pose_init = new_pose;
					time_init = ros::Time::now(); 
				}
			} 	
		}
		
		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());

		r.sleep();
	}

	ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state because the node has been killed");
	nitresult_.result = "Aborted. System is shuting down";
	nitresult_.value = 6;
	NITActionServer_->setAborted(nitresult_, "Navigation aborted because the node has been killed");


}


geometry_msgs::PoseStamped Upo_navigation_macro_actions::approachIT(upo_msgs::PersonPoseUPO* p)
{
	
	double safe_dist = 1.3;
	
	//Look for the person in the vector
	std::vector<upo_msgs::PersonPoseUPO> people;
	people_mutex_.lock();
	people = people_;
	people_mutex_.unlock();
	
	geometry_msgs::PoseStamped goal_pose;
	goal_pose.header.frame_id = "bad";
	upo_msgs::PersonPoseUPO newp;
	newp.id = -100;
	
	//printf("Received person with Id: %i and yaw: %.3f\n", p->id, tf::getYaw(p->orientation)); 

	
	if(people.size() == 0) {
		printf("approachIT. people array is empty!\n");
		//p->id = -100;
		//return goal_pose;
	} else {
	
		//printf("Person frame: %s, people vector frame: %s\n", p->header.frame_id.c_str(), people.at(0).header.frame_id.c_str());
		if(p->header.frame_id != people.at(0).header.frame_id) {
			//printf("2. Transforming frame. p frame: %s, p vector: %s\n", p->header.frame_id.c_str(), people.at(0).header.frame_id.c_str());
			//Transform coordinates if necessary
			geometry_msgs::PoseStamped aux;
			aux.header.frame_id = p->header.frame_id;
			aux.header.stamp = ros::Time();
			aux.pose.position = p->position;
			aux.pose.orientation = p->orientation; 
			geometry_msgs::PoseStamped out;
			out = transformPoseTo(aux, people.at(0).header.frame_id);
			
			newp.header = out.header;
			newp.position = out.pose.position;
			newp.orientation = out.pose.orientation;
		}
	}
	
	
	float min_dist = 1.5; //1.5 meter
	for(unsigned int i=0; i<people.size(); i++)
	{
		//Firstly look for the ID
		if(p->id != -1 && p->id == people.at(i).id) {
			//printf("Person with ID %i found!\n", p->id);
			newp = people.at(i);
			break;
			
		} else {  //Look for someone closer than 'min_dist' meters
			
			float dist = sqrt((p->position.x-people.at(i).position.x)*(p->position.x-people.at(i).position.x) 
				+ (p->position.y-people.at(i).position.y)*(p->position.y-people.at(i).position.y) );
			
			if(dist < min_dist) {
				min_dist = dist;
				newp = people.at(i);
			}
			
		}
	}
	
	
	if(social_approaching_type_ == 1)  //Person orientation is NOT taken into account
	{
		geometry_msgs::PoseStamped in;
		
		//No person found
		if(newp.id == -100) {
			printf("Person not found!\n");
			//p->id = -100;
			//return goal_pose;
			in.header = p->header;
			in.pose.position = p->position;
			in.pose.orientation = p->orientation; 
		} else {
			in.header = newp.header;
			in.pose.position = newp.position;
			in.pose.orientation = newp.orientation; 
			p->header = newp.header;
			p->id = newp.id;
			p->orientation = newp.orientation;
			p->vel = newp.vel;
			p->position = newp.position;
		}
		in.header.stamp = ros::Time();
		
		geometry_msgs::PoseStamped out;
		try {
			tf_listener_->transformPose("base_link", in, out);	
		}catch (tf::TransformException ex){
			ROS_WARN("ApproachWaypointSimple. TransformException: %s",ex.what());
			return goal_pose;
		}
		double x = out.pose.position.x;
		double y = out.pose.position.y;
		//double yaw = tf::getYaw(out.pose.orientation);
		double orientation = atan2(y,x); 
		orientation = normalizeAngle(orientation, -M_PI, M_PI);
		double distance = sqrt((x*x)+(y*y));
		if(distance <= safe_dist) { 		
			x = 0.0;
			y = 0.0;
		} else {
			x = x - safe_dist * cos(fabs(orientation));
			y = y - safe_dist * sin(fabs(orientation)); 
		}		
				
		goal_pose.header.stamp = ros::Time::now();
		goal_pose.header.frame_id = "base_link";
		goal_pose.pose.position.x = x;
		goal_pose.pose.position.y = y;
		goal_pose.pose.position.z = 0.0;
		goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
		//printf("goal_pose x:%.2f, y:%.2f, th:%.2f\n", x, y, orientation);
		
		out = transformPoseTo(goal_pose, "odom");
		return out;
			
		
	} else {  //Orientation is taken into account
		
		//1. Calculate goal in front of the person
		float yaw = 0.0;
		//No person found
		if(newp.id == -100) {
			printf("Person not found!\n");
			//p->id = -100;
			//return goal_pose;
			goal_pose.header.frame_id = p->header.frame_id;
			goal_pose.pose.position = p->position; 
			goal_pose.pose.orientation = p->orientation; 
			yaw = tf::getYaw(p->orientation);  
		} else {
			goal_pose.header.frame_id = newp.header.frame_id;
			goal_pose.pose.position = newp.position; 
			goal_pose.pose.orientation = newp.orientation; 
			yaw = tf::getYaw(newp.orientation); 
			p->header = newp.header;
			p->id = newp.id;
			p->orientation = newp.orientation;
			p->vel = newp.vel;
			p->position = newp.position;
		}
		goal_pose.header.stamp = ros::Time::now();
		
		goal_pose.pose.position.x = goal_pose.pose.position.x + safe_dist*cos(yaw);
		goal_pose.pose.position.y = goal_pose.pose.position.y + safe_dist*sin(yaw);
		yaw = normalizeAngle((yaw+M_PI), -M_PI, M_PI);
		goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		
		
		//2. Activate the id of the target to adapt the gaussian
		/*char buf[10];
		sprintf(buf, "%i", p->id);
		std::string st = std::string(buf);
		std::string cad = std::string("rosrun dynamic_reconfigure dynparam set_from_parameters /upo_navigation_macro_actions/Navigation_features _interaction_target_id:=");
		cad = cad + st;
		system(cad.c_str());
		*/
		
		//3. If social_approaching_type_ == 3
		// Indicar al planificador que realice sampleo de la gmm indicada (servicio?)
		// Modificar el tanto por ciento de sampleo de la GMM y normal
		// de acuerdo a la distancia entre el robot y la persona.
		
		geometry_msgs::PoseStamped out;
		out = transformPoseTo(goal_pose, "odom");
		
		return out;
	}
}



bool Upo_navigation_macro_actions::reconfigureParameters(std::string node, std::string param_name, std::string value, const datatype type)
{
	//printf("RECONFIGURE PARAMETERS METHOD\n");
	dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter param1;
    dynamic_reconfigure::BoolParameter param2;
    dynamic_reconfigure::DoubleParameter param3;
    dynamic_reconfigure::StrParameter param4;
    dynamic_reconfigure::Config conf;
    
    switch(type)
    {
		case INT_TYPE:
			param1.name = param_name.c_str();
			param1.value = stoi(value);
			conf.ints.push_back(param1);
			break;
		
		case DOUBLE_TYPE:
			param3.name = param_name.c_str();
			//printf("type double. Value: %s\n", param3.name.c_str());
			param3.value = stod(value);
			//printf("conversion to double: %.3f\n", param3.value);
			conf.doubles.push_back(param3);
			break;

		case BOOL_TYPE:
			param2.name = param_name.c_str();
			param2.value = stoi(value);
			conf.bools.push_back(param2);
			break;
		
		case STRING_TYPE:
			param4.name = param_name.c_str();
			param4.value = value;
			conf.strs.push_back(param4);
			break;
			
		default:
			ROS_ERROR("Upo_navigation_macro_actions. ReconfigureParameters. datatype not valid!");
	}
    srv_req.config = conf;

    std::string service = node + "/set_parameters";
    if (!ros::service::call(service, srv_req, srv_resp)) {
      ROS_ERROR("Could not call the service %s reconfigure the param %s to %s", service.c_str(), param_name.c_str(), value.c_str());
      return false;
    }
    return true;
}







void Upo_navigation_macro_actions::walkSideCB(const upo_navigation_macro_actions::WalkSideBySideGoal::ConstPtr& goal)
{

	printf("¡¡¡¡¡¡¡MacroAction  Walk side-by-side  -->  started!!!!!!\n");
	ros::Time time_init = ros::Time::now(); 
	upo_msgs::PersonPoseUPO p = goal->it;
	
	bool exit = false;
	
	char buf[40];
	sprintf(buf, "Walking with target id %i", p.id);
	std::string st = std::string(buf);

	/*
	uint32 target_id
	float64 goal_x
	float64 goal_y
	float64 goal_radius
	---
	uint8 error_code
	*/
	wsbs::start start_srv;
	start_srv.request.target_id = p.id;
	start_srv.request.goal_x = 0;
	start_srv.request.goal_y = 0;
	start_srv.request.goal_radius = -1;

	if (!start_client_.call(start_srv))
	{
		ROS_INFO("Setting ABORTED state. sevice call failed!");
		wsresult_.result = "Aborted. Walking sbs error 2";
		wsresult_.value = 2;
		WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
	}
	/*if(start_srv.response.error_code != 0) { //ERROR 
		ROS_INFO("Setting ABORTED state. Error_code received: %u", start_srv.response.error_code);
		wsresult_.result = "Aborted. Walking sbs error 2";
		wsresult_.value = 2;
		WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
	}*/

	ros::Rate r(control_frequency_);	
	
	while(nh6_.ok())
    {
		
		if(WSActionServer_->isPreemptRequested()){
			
        	if(WSActionServer_->isNewGoalAvailable()){
				
				upo_navigation_macro_actions::WalkSideBySideGoal new_goal = *WSActionServer_->acceptNewGoal();
				p = new_goal.it;

				//First stop the current wsbs
				wsbs::stop stop_srv;
				if (!stop_client_.call(stop_srv))
				{
					ROS_INFO("Setting ABORTED state. Stop service call failed");
					wsresult_.result = "Aborted. Walking sbs error while stopping";
					wsresult_.value = 2;
					WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
				} 
				/*if(stop_srv.response.error_code != 0) { //ERROR 
					ROS_INFO("Setting ABORTED state. Error code received: %u", stop_srv.response.error_code);
					wsresult_.result = "Aborted. Walking sbs error while stopping";
					wsresult_.value = 2;
					WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
				}*/

				//Now, start!
				start_srv.request.target_id = p.id;
				start_srv.request.goal_x = 0;
				start_srv.request.goal_y = 0;
				start_srv.request.goal_radius = -1;
				if (!start_client_.call(start_srv))
				{
					ROS_INFO("Setting ABORTED state. Start sevice call failed");
					wsresult_.result = "Aborted. Walking sbs error 2";
					wsresult_.value = 2;
					WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
				}
				/*if(start_srv.response.error_code != 0) { //ERROR 					
					ROS_INFO("Setting ABORTED state. Error code received: %u", start_srv.response.error_code);
					wsresult_.result = "Aborted. Walking sbs error 2";
					wsresult_.value = 2;
					WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
				}*/
	
			} else {
          		//if we've been preempted explicitly we need to shut things down
          		//UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
          		wsresult_.result = "Preempted";
				wsresult_.value = 1;
				WSActionServer_->setPreempted(wsresult_, "Walking sbs preempted");

          		//we'll actually return from execute after preempting
          		return;
        	}
		}
		
		//Check the status
		wsbs_mutex_.lock();
		int status = wsbs_status_;
		wsbs_mutex_.unlock();


		wsfeedback_.text = "WalkSideBySide running"; 

		/*
		WAITING_FOR_START  = 0,
		WAITING_FOR_ODOM   = 1,
		WAITING_FOR_LASER  = 2,
		WAITING_FOR_XTION  = 3,
		WAITING_FOR_PEOPLE = 4,
		WAITING_FOR_GOALS  = 5,
		RUNNING            = 6,
		TARGET_LOST        = 7,
		FINISHED           = 8
		*/
		if(status == 8) {
			ROS_INFO("WalkSideBySide. Setting SUCCEEDED state");
			wsresult_.result = "Succeeded";
			wsresult_.value = 0;
			WSActionServer_->setSucceeded(wsresult_, "Target goal reached");
			wsfeedback_.text = "WalkSideBySide finished successfully";
			exit = true;
		} else if(status <= 5) {
			ROS_INFO("Setting ABORTED state. Status received: %u", status);
			wsresult_.result = "Aborted. Walking sbs error 2";
			wsresult_.value = 2;
			WSActionServer_->setAborted(wsresult_, "Walking sbs aborted.");
			wsfeedback_.text = "WalkSideBySide aborted";
			exit = true;
		}
		
		WSActionServer_->publishFeedback(wsfeedback_);
		
		if(exit)
			return;
			
		r.sleep();
	}
	
	ROS_INFO("Setting ABORTED state");
	WSActionServer_->setAborted(wsresult_, "WalkSideBySide aborted because the node has been killed");

}





void Upo_navigation_macro_actions::waitCB(const upo_navigation_macro_actions::WaitGoal::ConstPtr& goal)
{
	
	printf("¡¡¡¡¡¡¡MacroAction Wait  -->  started!!!!!!\n");
	
	ros::Time time_init;
	time_init = ros::Time::now();
	bool exit = false;
	
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	//Monitorize the navigation action
	ros::Rate r(control_frequency_);	
	while(nh4_.ok())
	{

		if(WActionServer_->isPreemptRequested()){
        	if(WActionServer_->isNewGoalAvailable()){
          		//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
				ROS_INFO("Accepting new goal");
          		upo_navigation_macro_actions::WaitGoal new_goal = *WActionServer_->acceptNewGoal();
				
			} else {
				WActionServer_->setPreempted(wresult_, "Waiting preempted");
				return;
			}
		}

		wfeedback_.text = "Waiting";

		
		//check the blocked situation.
		double time = (ros::Time::now() - time_init).toSec();
		printf("Time: %.2f, secs_to_wait: %.2f\n", time, secs_to_wait_);
		if(time >= secs_to_wait_)
		{
			WActionServer_->setSucceeded(wresult_, "Clear");
			wfeedback_.text = "Waiting finished";
			exit = true;
		} 
		

		WActionServer_->publishFeedback(wfeedback_);
		
		if(exit)
			return;
		
		r.sleep();

	}
	
	ROS_INFO("Setting ABORTED state");
	WActionServer_->setAborted(wresult_, "Wait aborted because the node has been killed");
}





void Upo_navigation_macro_actions::yieldCB(const upo_navigation_macro_actions::YieldGoal::ConstPtr& goal)
{
	
	printf("¡¡¡¡¡¡¡MacroAction Yield  -->  started!!!!!!\n");
	
	
	//Get the current robot pose in global coordinates
	geometry_msgs::PoseStamped robot_pose = UpoNav_->getRobotGlobalPosition();
	
	//Get the closest yield goal to the current robot position
	double goal_x = 0.0, goal_y=0.0, goal_theta = 0.0;
	yield_->getClosestPoint(robot_pose.pose.position.x, robot_pose.pose.position.y, goal_x, goal_y, goal_theta);
	geometry_msgs::PoseStamped g;
	g.header.frame_id = "map";
	g.header.stamp = ros::Time::now();
	g.pose.position.x = goal_x;
	g.pose.position.y = goal_y;
	g.pose.position.z = 0.0;
	g.pose.orientation = tf::createQuaternionMsgFromYaw(goal_theta);
	
	//Enviar new goal 
	bool ok = UpoNav_->executeNavigation(g); 
	if(!ok)
	{
		ROS_INFO("Yield. Setting ABORTED state 1");
		YActionServer_->setAborted(yresult_, "Navigation aborted");
		return;
	}
	
	
	int pursue_status = 0;
	bool exit = false;
	ros::Time time_init = ros::Time::now();
	bool first = true;
	geometry_msgs::PoseStamped pose_init;

	//Monitorize the navigation action
	ros::Rate r(control_frequency_);	
	while(nh5_.ok())
	{

		if(YActionServer_->isPreemptRequested()){
        	if(YActionServer_->isNewGoalAvailable()){
          		//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
				ROS_INFO("Accepting new goal");
          		upo_navigation_macro_actions::YieldGoal new_goal = *YActionServer_->acceptNewGoal();
          		
				robot_pose = UpoNav_->getRobotGlobalPosition();
				yield_->getClosestPoint(robot_pose.pose.position.x, robot_pose.pose.position.y, goal_x, goal_y, goal_theta);
				g.header.stamp = ros::Time::now();
				g.pose.position.x = goal_x;
				g.pose.position.y = goal_y;
				g.pose.orientation = tf::createQuaternionMsgFromYaw(goal_theta);
				
				//Enviar new goal 
				bool ok = UpoNav_->executeNavigation(g); 
				if(!ok)
				{
					ROS_INFO("Yield. Setting ABORTED state 1");
					YActionServer_->setAborted(yresult_, "Navigation aborted");
					return;
				}
				
				
			} else {
				UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		yresult_.result = "Preempted";
				yresult_.value = 1;
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
				YActionServer_->setPreempted(yresult_, "Navigation preempted");
          		//we'll actually return from execute after preempting
          		return;
			}
		}
		
		
		geometry_msgs::PoseStamped new_pose;
		pursue_status = UpoNav_->pathFollow(new_pose);
		/*if(first){
			pose_init = new_pose;
			time_init = ros::Time::now();
			first = false;
		}*/

		switch(pursue_status)
		{
			case -2:	//Error
				yfeedback_.text = "Aborted";
				ROS_INFO("Setting ABORTED state");
				yresult_.result = "Aborted. Navigation error 2";
				yresult_.value = 3;
				YActionServer_->setAborted(yresult_, "Navigation aborted");
				exit = true;
				break;

			case -1:	//Error, try to replan
				nhfeedback_.text = "Replaning to continue navigating";
				break;

			case 0:		//Goal not reached, continue navigating
				nhfeedback_.text = "Navigating to yield position";
				break;

			case 1:		//Goal reached
				//ROS_INFO("Goal reached. Waiting for social path");
				//yresult_.result = "Succeeded";
				//yresult_.value = 0;
				//YActionServer_->setSucceeded(yresult_, "Goal Reached");
				yfeedback_.text = "No Social Path Available. Waiting in yield position";
				//exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
				yfeedback_.text = "Unexpected pursue status value!!!";
				ROS_INFO("Setting ABORTED state");
				yresult_.result = "Aborted. Navigation error 2";
				yresult_.value = 3;
				YActionServer_->setAborted(yresult_, "Navigation aborted because an unexpected pursue value was received");
				exit = true;
		}
		
		if(exit)
			return;

		boost::recursive_mutex::scoped_lock l(configuration_mutex_);

		//check the time
		double time = (ros::Time::now() - time_init).toSec();
		//printf("Time: %.2f, secs_to_yield: %.2f\n", time, secs_to_yield_);
		if(time >= secs_to_yield_)
		{
			//Check if there are people in the zone
			if(!person_inzone_) {
				yfeedback_.text = "Social Path Available";
				yresult_.result = "Succeeded";
				yresult_.value = 0;
				YActionServer_->setSucceeded(yresult_, "Yield succeeded. Social Path Available");
				exit = true;
			} else {
				//yfeedback_.text = "No Social Path Available. Waiting in yield position";
				time_init = ros::Time::now();
			}
		} 
		

		YActionServer_->publishFeedback(yfeedback_);
		
		if(exit)
			return;
		
		r.sleep();

	}
	
	ROS_INFO("Setting ABORTED state");
	yresult_.result = "Aborted. System is shuting down";
	yresult_.value = 6;
	YActionServer_->setAborted(yresult_, "Yield aborted because the node has been killed");
}






void Upo_navigation_macro_actions::assistedSteeringCB(const upo_navigation_macro_actions::AssistedSteeringGoal::ConstPtr& goal)
{
	
	//printf("¡¡¡¡¡¡¡MacroAction AssistedSteering  -->  started!!!!!!\n");
	
	UpoNav_->stopRRTPlanning();
	
	ros::Time time_init;
	time_init = ros::Time::now();
	bool exit = false;
	
	
	//as_->resume();
	
	
	//boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	//Monitorize the navigation action
	//ros::Rate r(control_frequency_);	
	ros::Rate r(30.0);
	while(nh7_.ok())
	{

		if(ASActionServer_->isPreemptRequested()){
        	if(ASActionServer_->isNewGoalAvailable()){
          		//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
				//ROS_INFO("Accepting new goal");
          		upo_navigation_macro_actions::AssistedSteeringGoal new_goal = *ASActionServer_->acceptNewGoal();
				time_init = ros::Time::now();
				//if(as_->isPaused())
					//as_->resume();
				
			} else {
				ASActionServer_->setPreempted(asresult_, "AssistedSteering preempted");
				return;
			}
		}
		
		asfeedback_.text = "Robot manually controlled";

		//if(as_->isReceivingCdms())
		//	time_init = ros::Time::now();
		//else{
			//Check the time without receiving commands from the interface
			double time = (ros::Time::now() - time_init).toSec();
			if(time > 7.0)
			{
				asfeedback_.text = "Manual control finished";
				asresult_.result = "Assisted Steering Succeeded";
				asresult_.value = 0;
				ASActionServer_->setSucceeded(asresult_, "Assisted Steering succeeded");
				exit = true;
			}
		//}
		
		ASActionServer_->publishFeedback(asfeedback_);
		
		if(exit)
			return;
		
		r.sleep();

	}
	
	ROS_INFO("Setting ABORTED state");
	ASActionServer_->setAborted(asresult_, "Assisted Steering aborted because the node has been killed");
}






void Upo_navigation_macro_actions::wsbsCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	/*
	WAITING_FOR_START  = 0,
    WAITING_FOR_ODOM   = 1,
    WAITING_FOR_LASER  = 2,
    WAITING_FOR_XTION  = 3,
    WAITING_FOR_PEOPLE = 4,
    WAITING_FOR_GOALS  = 5,
    RUNNING  = 6,
    TARGET_LOST  = 7,
    FINISHED = 8
	*/
	wsbs_mutex_.lock();
	wsbs_status_ = msg->data;
	wsbs_mutex_.unlock();
}




void Upo_navigation_macro_actions::peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg) 
{
	
	people_mutex_.lock();
	people_ = msg->personPoses;
	people_mutex_.unlock();
	
	rinzone_mutex_.lock();
	bool robotin = robot_inzone_;
	rinzone_mutex_.unlock();

	
	bool inside = false;
	//bool inside2 = false;
	if(robotin) 
	{
		std::vector<upo_msgs::PersonPoseUPO> ps = msg->personPoses;
		geometry_msgs::PoseStamped p;
		geometry_msgs::PoseStamped out;
		for(unsigned int i=0; i<ps.size(); i++)
		{
			p.header = ps[i].header;
			p.pose.position = ps[i].position;
			p.pose.orientation = ps[i].orientation;
			out = transformPoseTo(p, std::string("/map"));
			bool ok;
			if(yield_->getType(out.pose.position.x, out.pose.position.y, ok) == SUPERNARROW) {
				if(ok) {
					inside = true;
					//printf("PERSON inside supernarrow zone!!!\n");
				}
			}
		}
	}
	pinzone_mutex_.lock();
	person_inzone_ = inside;
	//person_inzone2_ = inside2;
	pinzone_mutex_.unlock();
	
}

void Upo_navigation_macro_actions::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	global_pose_mutex_.lock();
	robot_global_pose_.x = msg->pose.pose.position.x;
	robot_global_pose_.y = msg->pose.pose.position.y;
	robot_global_pose_.theta = tf::getYaw(msg->pose.pose.orientation);
	global_pose_mutex_.unlock();
	
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	bool ok;
	bool inside = false;
	bool inside2 = false;

	ros::NodeHandle n("~/RRT_ros_wrapper");
	double dev;
	n.getParam("full_path_stddev", dev); 	


	if(yield_->getType(x, y, ok) == NARROW)
	{
		if(ok) {
			inside = true;
		} 
	} else if(yield_->getType(x, y, ok) == SUPERNARROW) {
		inside2 = true;
	} 

	rinzone_mutex_.lock();
	robot_inzone_ = inside;
	robot_inzone2_ = inside2;
	rinzone_mutex_.unlock();
}


void Upo_navigation_macro_actions::rrtGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	geometry_msgs::PoseStamped out;
	out = transformPoseTo(*msg, "map");
	geometry_msgs::Pose2D p;
	p.x = out.pose.position.x;
	p.y = out.pose.position.y;
	p.theta = 0.0;

	goal_mutex_.lock();
	rrtgoal_ = p;
	goal_mutex_.unlock();
}


bool Upo_navigation_macro_actions::isYieldDirectionCorrect()
{
	//get rrt goal
	goal_mutex_.lock();
	geometry_msgs::Pose2D goal = rrtgoal_;
	goal_mutex_.unlock();

	//Get robot position
	global_pose_mutex_.lock();
	geometry_msgs::Pose2D robot = robot_global_pose_;
	global_pose_mutex_.unlock();

	//Get yield center point or the position of the person in yield zone
	geometry_msgs::Pose2D point;
	if(!yield_->getYieldAreaCenterPoint(robot.x, robot.y, point.x, point.y))
		return true;

	//Calculate distances
	float d1 = sqrt((goal.x-robot.x)*(goal.x-robot.x) + (goal.y-robot.y)*(goal.y-robot.y));
	float d2 = sqrt((goal.x-point.x)*(goal.x-point.x) + (goal.y-point.y)*(goal.y-point.y));

	//Do not yield if the robot has already crossed the door 
	if(d2 > d1){
		//printf("Yield not correct!!! gx:%.2f, gy:%.2f, rx:%.2f, ry:%.2f, point.x:%.2f, point.y:%.2f\n", goal.x, goal.y, robot.x, robot.y, point.x, point.y);
		return false;
	}
	return true;	
}



void Upo_navigation_macro_actions::changeParametersNarrowPlaces()
{
	rinzone_mutex_.lock();
	bool in = robot_inzone_;
	bool in2 = robot_inzone2_;
	rinzone_mutex_.unlock();
	if(/*in ||*/ in2) {
		//Inside the narrow area. Change the parameter  
		//printf("\nINSIDE the narrow area. Change the wp_tolerance parameter to 0.25\n\n");
		reconfigureParameters(std::string("/upo_navigation_macro_actions/PurePlannerROS"), std::string("wp_tolerance"), std::string("0.35"), DOUBLE_TYPE);
		reconfigureParameters(std::string("/upo_navigation_macro_actions/PurePlannerROS"), std::string("sim_time"), std::string("0.3"), DOUBLE_TYPE);
	} else {
		//Outside area. Establish the regular parameter
		//printf("\nOUTSIDE the narrow area. Change the wp_tolerance parameter to 0.5\n\n");
		reconfigureParameters(std::string("/upo_navigation_macro_actions/PurePlannerROS"), std::string("wp_tolerance"), std::string("0.5"), DOUBLE_TYPE);
		reconfigureParameters(std::string("/upo_navigation_macro_actions/PurePlannerROS"), std::string("sim_time"), std::string("0.5"), DOUBLE_TYPE);
	}
}


void Upo_navigation_macro_actions::changeParametersNarrowPlaces2()
{
	ros::NodeHandle n("~/RRT_ros_wrapper");
	double dev;
	n.getParam("full_path_stddev", dev); 

	rinzone_mutex_.lock();
	bool in = robot_inzone_;
	bool in2 = robot_inzone2_;
	rinzone_mutex_.unlock();

	if(in) {
		if(dev != 0.5) {
			printf("dev: %.3f != 0.5!!!\n", dev);
			reconfigureParameters(std::string("/upo_navigation_macro_actions/RRT_ros_wrapper"), std::string("full_path_stddev"), std::string("0.5"), DOUBLE_TYPE);
		}

	} else if(in2) {
		if(dev != 0.2) {
			printf("dev: %.3f != 0.2!!!\n", dev);
			reconfigureParameters(std::string("/upo_navigation_macro_actions/RRT_ros_wrapper"), std::string("full_path_stddev"), std::string("0.2"), DOUBLE_TYPE);
		}
	} else if(dev != initial_stddev_) {
			printf("dev: %.3f != initial_stddev: %.3f!!!\n", dev, initial_stddev_);
			reconfigureParameters(std::string("/upo_navigation_macro_actions/RRT_ros_wrapper"), std::string("full_path_stddev"), std::to_string(initial_stddev_), DOUBLE_TYPE);
	}
	

}






geometry_msgs::PoseStamped Upo_navigation_macro_actions::transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out)
{
	geometry_msgs::PoseStamped in = pose_in;
	geometry_msgs::PoseStamped pose_out;
	try {
		tf_listener_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("TransformException in method transformPoseTo: %s",ex.what());
		pose_out.header = in.header;
		pose_out.header.stamp = ros::Time::now();
		pose_out.pose.position.x = 0.0;
		pose_out.pose.position.y = 0.0;
		pose_out.pose.position.z = 0.0;
		pose_out.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	}
	
	return pose_out;
}


//This method removes the initial slash from the frame names 
//in order to compare the string names easily
void Upo_navigation_macro_actions::fixFrame(std::string& cad)
{
	if(cad[0] == '/')
	{
		cad.erase(0,1);
	}
}


float Upo_navigation_macro_actions::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}



