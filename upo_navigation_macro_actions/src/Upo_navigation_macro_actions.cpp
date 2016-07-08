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
	n.param<bool>("social_approaching", social_approach_, false);  


	ros::NodeHandle nh;
	people_sub_ = nh.subscribe("/people/navigation", 1, &Upo_navigation_macro_actions::peopleCallback, this); 
	

	//Initialize action servers
	NWActionServer_ = new NWActionServer(nh1_, "NavigateWaypoint", boost::bind(&Upo_navigation_macro_actions::navigateWaypointCB, this, _1), false);
	NHActionServer_ = new NHActionServer(nh2_, "NavigateHome", boost::bind(&Upo_navigation_macro_actions::navigateHomeCB, this, _1), false);
	NITActionServer_ = new NITActionServer(nh3_, "NavigateInteractionTarget", boost::bind(&Upo_navigation_macro_actions::navigateInteractionTargetCB, this, _1), false);
	WActionServer_ = new WActionServer(nh4_, "Wait", boost::bind(&Upo_navigation_macro_actions::waitCB, this, _1), false);
	//NITGActionServer_ = new NITGActionServer(nh5_, "NavigateInteractionTargetorGroup", boost::bind(&Upo_navigation_macro_actions::navigateInteractionTargetorGroupCB, this, _1), false);
	
	NWActionServer_->start();
	NHActionServer_->start();
	NITActionServer_->start();
	WActionServer_->start();
	//NITGActionServer_->start();
}


Upo_navigation_macro_actions::~Upo_navigation_macro_actions()
{
	if(NWActionServer_)
      	delete NWActionServer_;
	if(NHActionServer_)
      	delete NHActionServer_;
	if(NITActionServer_)
      	delete NITActionServer_;
	//if(NITGActionServer_)
    //  	delete NITGActionServer_;
	if(WActionServer_)
      	delete WActionServer_;
	if(UpoNav_)
		delete UpoNav_;
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
					NWActionServer_->setAborted(nwresult_, "Navigation aborted");
					return;
				
				}

       	 	}
        	else {
          		//if we've been preempted explicitly we need to shut things down
          		UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
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
				NWActionServer_->setSucceeded(nwresult_, "Goal Reached");
				nwfeedback_.text = "Succeeded";
				exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
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
				NWActionServer_->setAborted(nwresult_, "Navigation aborted because of blocked situation");
				nwfeedback_.text = "Blocked";
				NWActionServer_->publishFeedback(nwfeedback_);
				return;
			} else {
				pose_init = new_pose;
				time_init = ros::Time::now(); 
			}
		} 
		


		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());

		r.sleep();
	}

	ROS_INFO("Setting ABORTED state");
	NWActionServer_->setAborted(nwresult_, "Navigation aborted because the node has been killed");

}




void Upo_navigation_macro_actions::navigateHomeCB(const upo_navigation_macro_actions::NavigateHomeGoal::ConstPtr& goal)
{

	printf("¡¡¡¡¡¡¡MacroAction NavigateHome  -->  started!!!!!!\n");

	bool ok = UpoNav_->executeNavigation(goal->home_pose); 
	if(!ok)
	{
		ROS_INFO("Setting ABORTED state 1");
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
					NHActionServer_->setAborted(nhresult_, "Navigation aborted");
					return;
					
				}

       	 	}
        	else {
          		//if we've been preempted explicitly we need to shut things down
          		UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
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
				NHActionServer_->setSucceeded(nhresult_, "Goal Reached");
				nhfeedback_.text = "Succeeded";
				exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
				nhfeedback_.text = "Unexpected pursue status value!!!";
				ROS_INFO("Setting ABORTED state");
				NHActionServer_->setAborted(nhresult_, "Navigation aborted because an unexpected pursue value was received");
				exit = true;
		}
		

		//push the feedback out
		nhfeedback_.base_position = new_pose;
		NHActionServer_->publishFeedback(nhfeedback_);

		//if(exit)
		//	return;


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
				NHActionServer_->setAborted(nhresult_, "Navigation aborted because of blocked situation");
				nhfeedback_.text = "Blocked";
				NHActionServer_->publishFeedback(nhfeedback_);
				return;
			
			} else {
				pose_init = new_pose;
				time_init = ros::Time::now(); 
			}
		} 


		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());

		r.sleep();
	}

	ROS_INFO("Setting ABORTED state");
	NHActionServer_->setAborted(nhresult_, "Navigation aborted because the node has been killed");

}





// We can include a checking about the moving person position.
void Upo_navigation_macro_actions::navigateInteractionTargetCB(const upo_navigation_macro_actions::NavigateInteractionTargetGoal::ConstPtr& goal)
{
	printf("¡¡¡¡¡¡¡MacroAction NavigateToInteractionTarget  -->  started!!!!!!\n");

	upo_msgs::PersonPoseUPO p;
	p.header = goal->person_pose.header;
	p.id = -1;
	p.vel = -1;
	p.position = goal->person_pose.pose.position;
	p.orientation = goal->person_pose.pose.orientation;
	
	geometry_msgs::PoseStamped goal_pose;
	bool newWayPoint = false;
	
	if(!social_approach_) {
	
		goal_pose = approachWaypointSimple(&p, &newWayPoint);
		if(p.id == -100) {
			ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state");
			NITActionServer_->setAborted(nitresult_, "Navigation aborted because of IT was lost");
			return;
		}

	} else {

		//Still not implemented

	}

	//Enviar goal 
	bool ok = UpoNav_->executeNavigation(goal_pose); 
	if(!ok)
	{
		ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state 1");
		NITActionServer_->setAborted(nitresult_, "Navigation aborted");
		return;
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

				p.header = new_goal.person_pose.header;
				p.id = -1;
				p.vel = -1;
				p.position = new_goal.person_pose.pose.position;
				p.orientation = new_goal.person_pose.pose.orientation;

				if(!social_approach_) {
					goal_pose = approachWaypointSimple(&p, &newWayPoint);
					if(p.id == -100) {
						ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state");
						NITActionServer_->setAborted(nitresult_, "Navigation aborted because of IT was lost");
						return;
					}
				} else {
					//Still not implemented
				}

				//Enviar a new goal 
				bool ok = UpoNav_->executeNavigation(goal_pose); 
				if(!ok)
				{
					ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state 1");
					NITActionServer_->setAborted(nitresult_, "Navigation aborted");
					return;
				}
          		
       	 	} else {
          		//if we've been preempted explicitly we need to shut things down
          		UpoNav_->resetState();

          		//notify the ActionServer that we've successfully preempted
          		ROS_DEBUG_NAMED("upo_navigation_macro_actions","upo_navigation preempting the current goal");
				NITActionServer_->setPreempted(nitresult_, "Navigation preempted");
          		//we'll actually return from execute after preempting
          		return;
        	}
      	}


		if(!social_approach_) {
			
			goal_pose = approachWaypointSimple(&p, &newWayPoint);
			if(p.id == -100) {
				ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state");
				NITActionServer_->setAborted(nitresult_, "Navigation aborted because of IT was lost");
				return;
			}
			if(newWayPoint){
				//Send new goal 
				bool ok = UpoNav_->executeNavigation(goal_pose); 
				if(!ok)
				{
					ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state 1");
					NITActionServer_->setAborted(nitresult_, "Navigation aborted");
					return;
				}			
			}
			
		} else {
			//Still not implemented
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
				ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state");
				NITActionServer_->setAborted(nitresult_, "Approaching aborted");
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
				NITActionServer_->setSucceeded(nitresult_, "IT Reached");
				nitfeedback_.text = "Succeeded";
				exit = true;
				break;

			default:
				ROS_ERROR("Unexpected pursue status value!!!");
				nitfeedback_.text = "Unexpected pursue status value!!!";
				ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state");
				NITActionServer_->setAborted(nitresult_, "Navigation aborted because an unexpected pursue value was received");
				exit = true;
				break;
		}
		
		//push the feedback out
		nitfeedback_.base_position = new_pose;
		NITActionServer_->publishFeedback(nitfeedback_);

		if(exit) 
			return;
	
		
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
				NITActionServer_->setAborted(nitresult_, "Navigation aborted because of blocked situation");
				nitfeedback_.text = "Blocked";
				NITActionServer_->publishFeedback(nitfeedback_);
				return;
			} else {
				pose_init = new_pose;
				time_init = ros::Time::now(); 
			}
		} 

		//ros::WallDuration dur = ros::WallTime::now() - startt;
		//printf("Loop time: %.4f secs\n", dur.toSec());

		r.sleep();
	}

	ROS_INFO("NavigateToInteractionTarget. Setting ABORTED state because the node has been killed");
	NITActionServer_->setAborted(nitresult_, "Navigation aborted because the node has been killed");


}








void Upo_navigation_macro_actions::waitCB(const upo_navigation_macro_actions::WaitGoal::ConstPtr& goal)
{
	
	printf("¡¡¡¡¡¡¡MacroAction Wait  -->  started!!!!!!\n");
	
	ros::Time time_init;
	time_init = ros::Time::now();
	bool exit = false;

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






geometry_msgs::PoseStamped Upo_navigation_macro_actions::approachWaypointSimple(upo_msgs::PersonPoseUPO* p, bool* new_waypoint)
{
	double safe_dist = 1.5;
	
	//Look for the person in the vector
	std::vector<upo_msgs::PersonPoseUPO> people;
	people_mutex_.lock();
	people = people_;
	people_mutex_.unlock();
	
	geometry_msgs::PoseStamped goal_pose;
	upo_msgs::PersonPoseUPO newp;
	newp.id = -100;

	
	if(people.size() == 0) {
		//printf("people vector with size 0\n");
		p->id = -100;
		return goal_pose;
	}
	
	if(p->header.frame_id != people.at(0).header.frame_id) {
		//printf("Transforming frame. p frame: %s, p vector: %s\n", p->header.frame_id.c_str(), people.at(0).header.frame_id.c_str());
		//Transform coordinates if necessary
		geometry_msgs::PoseStamped aux;
		aux.header.frame_id = p->header.frame_id;
		aux.header.stamp = ros::Time();
		aux.pose.position = p->position;
		aux.pose.orientation = p->orientation; 
		geometry_msgs::PoseStamped out;
		out = transformPoseTo(aux, people.at(0).header.frame_id);
		
		p->header = out.header;
		p->position = out.pose.position;
		p->orientation = out.pose.orientation;
	}
	
	
	float min_dist = 1.5; //1.5 meter
	for(unsigned int i=0; i<people.size(); i++)
	{
		//First look for the ID
		if(p->id != -1 && p->id == people.at(i).id) {
			//printf("Person with ID %i found!\n", p->id);
			newp = people.at(i);
			break;
			
		} else {  //Look for distance (first time we don't have the id)
			
			float dist = sqrt((p->position.x-people.at(i).position.x)*(p->position.x-people.at(i).position.x) 
				- (p->position.y-people.at(i).position.y)*(p->position.y-people.at(i).position.y) );
			
			if(dist < min_dist) {
				min_dist = dist;
				newp = people.at(i);
			}
			
		}
	}
	
	//No person found
	if(newp.id == -100) {
		//printf("Person not found!\n");
		p->id = -100;
		return goal_pose;
	}
	
	//Check the distance between p and newp
	//If it is short enough, don't update the goal
	//We will check the angular diff also in near future
	min_dist = 0.40;
	float dist = sqrt((p->position.x-newp.position.x)*(p->position.x-newp.position.x) 
				- (p->position.y-newp.position.y)*(p->position.y-newp.position.y));
	
	if(dist <= min_dist && p->id != -1) {
		//printf("Person in a close position. We don't update the goal\n");
		*new_waypoint = false;
		return goal_pose;
	}
	
	//printf("Updating goal!!!\n");
	//Update the values of the pointer p
	p->header = newp.header;
	p->id = newp.id;
	p->vel = newp.vel;
	p->position = newp.position;
	p->orientation = newp.orientation;
	*new_waypoint = true;
	
	
	geometry_msgs::PoseStamped in;
	in.header = newp.header;
	in.header.stamp = ros::Time::now();
	in.pose.position = newp.position;
	in.pose.orientation = newp.orientation; 
	
	geometry_msgs::PoseStamped out;
	try {
		tf_listener_->transformPose("base_link", in, out);	
	}catch (tf::TransformException ex){
		ROS_WARN("ApproachWaypointSimple. TransformException: %s",ex.what());
		p->id = -100;
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
	return goal_pose;
}



void Upo_navigation_macro_actions::peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg) 
{
	
	people_mutex_.lock();
	people_ = msg->personPoses;
	people_mutex_.unlock();
	
}




geometry_msgs::PoseStamped Upo_navigation_macro_actions::transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out)
{
	geometry_msgs::PoseStamped in = pose_in;
	geometry_msgs::PoseStamped pose_out;
	try {
		tf_listener_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("TransformException in method transformPoseTo: %s",ex.what());
	}
	//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
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



