
#include <upo_local_planner/collision_detection.h>


namespace upo_local_planner {

CollisionDetection::CollisionDetection(tf::TransformListener* tf, upo_local_planner::OdometryHelperRos* oh,
		double max_lv, double max_av, double lin_acc, double ang_acc, double sim_t, double r_radius,
						double granularity) {
	
	tf_ = tf;
	odom_helper_ = NULL;
	if(oh)
		odom_helper_ = oh;

	max_lin_vel_ = max_lv;
	max_ang_vel_ = max_av;
	max_lin_acc_ = lin_acc;
	max_ang_acc_ = ang_acc;
	sim_time_ = sim_t;
	robot_radius_ = r_radius;
	granularity_ = granularity;
		
	setup();
}


CollisionDetection::~CollisionDetection() {
	//delete odom_helper_;
}


void CollisionDetection::setup() {

	//ros::NodeHandle n("~");
	ros::NodeHandle n("~/UpoPlanner");

	scan1_time_ = ros::Time::now();
	scan2_time_ = scan1_time_;

	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>(n); //ros::NodeHandle("~")
    //dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>::CallbackType cb = boost::bind(&AssistedSteering::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);

	double l = 1;
	n.param<double>("number_of_lasers", l, 2.0);
	n_lasers_ = (int) floor(l);
	n.param<std::string>("laser1_topic", laser1_topic_, std::string("scanfront"));
	n.param<std::string>("laser2_topic", laser2_topic_, std::string("scanback"));
	n.param<std::string>("odom_topic", odom_topic_, std::string("odom"));
	//n.param<std::string>("cmdvel_topic", cmdvel_topic_, std::string("teresa/cmd_vel"));
	//n.param<std::string>("new_cmdvel_topic", new_cmdvel_topic_, std::string("cmd_vel"));
	//n.param<double>("robot_radius", robot_radius_, 0.35);
	//n.param<double>("granularity", granularity_, 0.025);
	//n.param<double>("max_lin_vel", max_lin_vel_, 0.6);
	//n.param<double>("max_ang_vel", max_ang_vel_, 0.6);
	//n.param<double>("max_lin_acc", max_lin_acc_, 1.0);
	//n.param<double>("max_ang_acc", max_ang_acc_, 1.0);
	//n.param<double>("sim_time", sim_time_, 0.05);
	//n.param<bool>("is_active", isActive_, true);
	//n.param<double>("ang_vel_inc", ang_vel_inc_, 0.20);
	//n.param<double>("lin_vel_inc", lin_vel_inc_, 0.10);
	max_lv_var_ = max_lin_acc_ * sim_time_;
	max_av_var_ = max_ang_acc_ * sim_time_;

	printf("COLLISION DETECTOR:\n");
	printf("lasers employed: %i\n", n_lasers_);
	printf("laser 1 topic: %s\n", laser1_topic_.c_str());
	if(n_lasers_ >= 2)
		printf("laser 2 topic: %s\n", laser2_topic_.c_str());

	ros::NodeHandle nh;
	//out_cmdvel_pub_ = nh.advertise<geometry_msgs::Twist>(new_cmdvel_topic_, 1);

	laser1_sub_ = nh.subscribe<sensor_msgs::LaserScan>(laser1_topic_.c_str(), 1, &CollisionDetection::laser1Callback, this); 
	if(n_lasers_ >= 2)
		laser2_sub_ = nh.subscribe<sensor_msgs::LaserScan>(laser2_topic_.c_str(), 1, &CollisionDetection::laser2Callback, this);
	
	//cmdvel_sub_ = nh.subscribe<geometry_msgs::Twist>(cmdvel_topic_.c_str(), 1, &AssistedSteering::cmdvelCallback, this);

	if(odom_helper_ == NULL)
		odom_helper_ = new OdometryHelperRos(odom_topic_);

	//Dynamic reconfigure
	//dsrv_ = new dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>(n); //ros::NodeHandle("~")
    //dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>::CallbackType cb = boost::bind(&AssistedSteering::reconfigureCB, this, _1, _2);
    //dsrv_->setCallback(cb);

}


/*void CollisionDetection::reconfigureCB(assisted_steering::AssistedSteeringConfig &config, uint32_t level){
    
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    
	max_lin_vel_ = config.max_lin_vel;
	max_ang_vel_ = config.max_ang_vel;
	max_lin_acc_ = config.max_lin_acc;
	max_ang_acc_ = config.max_ang_acc;
	time_step_ = config.time_step;
	robot_radius_ = config.robot_radius;
	granularity_ = config.granularity;
	isActive_ = config.is_active;
	ang_vel_inc_ = config.ang_vel_inc;
	lin_vel_inc_ = config.lin_vel_inc;

	max_lv_var_ = max_lin_acc_ * time_step_;
	max_av_var_ = max_ang_acc_ * time_step_;

}*/


void CollisionDetection::laser1Callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	ROS_INFO_ONCE("Collision detector: Laser1 received!");
	//IMPORTANT: the frame of the laser should be the center of the robot (base_link)
	//Otherwise we should include a shift to the center in the calculations.
	laser1_mutex_.lock();
	laser1_scan_ = *msg;
	scan1_time_ = ros::Time::now();
	laser1_mutex_.unlock();
}

void CollisionDetection::laser2Callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	ROS_INFO_ONCE("Collision detector: Laser2 received!");
	//IMPORTANT: the frame of the laser should be the center of the robot (base_link)
	//Otherwise we should include a shift to the center in the calculations.
	laser2_mutex_.lock();
	laser2_scan_ = *msg;
	scan2_time_ = ros::Time::now();
	laser2_mutex_.unlock();
}



/*void AssistedSteering::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
	twist_mutex_.lock();
	twist_ = *msg;
	twist_mutex_.unlock();

	laser_mutex_.lock();
	ros::Time laser_time = scan_time_;
	laser_mutex_.unlock();

	boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	double secs = (ros::Time::now() - laser_time).toSec();
	if(isActive_ && secs < 1.5) {
		//Get current robot velocity
		odom_helper_->getRobotVel(robot_vel_);
		//Check if the command is valid
		if(!checkCommand(&twist_)){
			printf("\nPOSSIBLE COLLISION. Looking for a valid command...\n\n");
			//look for a valid command
			findValidCmd(&twist_);
		}
	} 
	out_cmdvel_pub_.publish(twist_);
	
}*/



/*
void AssistedSteering::sendVelocityZero()
{
	geometry_msgs::Twist v;
	v.linear.x = 0.0;
	v.linear.y = 0.0;
	v.linear.y = 0.0;
	v.angular.x = 0.0;
	v.angular.y = 0.0;
	v.angular.z = 0.0;
	printf("Stopping the robot!\n");
	out_cmdvel_pub_.publish(v);
}*/


void CollisionDetection::saturateVelocities(geometry_msgs::Twist* twist)
{
	float lv = twist->linear.x;
	float av = twist->angular.z;

	float rvx = robot_vel_.getOrigin().getX();
	float rvy = robot_vel_.getOrigin().getY();
	float rvt = tf::getYaw(robot_vel_.getRotation());

	// acc linear
	if(fabs(rvx - lv) > max_lv_var_) {
		lv = (lv < rvx) ? (rvx - max_lv_var_) : (rvx + max_lv_var_);
	} 
	// acc angular
	if(fabs(rvt - av) > max_av_var_) {
		av = (av < rvt) ? (rvt - max_av_var_) : (rvt + max_av_var_);
	} 
	
	//Check maximum velocities
	if(lv > max_lin_vel_)
		lv = max_lin_vel_;
	else if(lv < (-max_lin_vel_))
		lv = max_lin_vel_*(-1);
	
	if(av > max_ang_vel_)
		av = max_ang_vel_;
	else if(av < (-max_ang_vel_))
		av = max_ang_vel_*(-1);

	twist->linear.x = lv;
	twist->angular.z = av;

}



/*bool AssistedSteering::findValidCmd(geometry_msgs::Twist* twist)
{

	float lv = twist->linear.x;
	float av = twist->angular.z;

	float aux_lv = lv;
	float aux_av = av;

	//float ang_inc = 0.15;
	//float lin_inc = 0.1;

	float slot = 0.0;
	//if(lv > 0.0) {
		//Linear vels
		for(unsigned int i=0; i<3; i++)
		{
			aux_lv = fabs(lv) - (lin_vel_inc_*i);
			if(lv < 0.0){
				aux_lv *= (-1);
			}
			//Angular vels
			for(unsigned int j=1; j<=3; j++)
			{
				aux_av = av + (ang_vel_inc_*j);
				twist->linear.x = aux_lv;
				twist->angular.z = aux_av;
				if(checkCommand(twist)) {
					printf("Correct velocities found!!! lv:%.3f, av:%.3f\n", twist->linear.x, twist->angular.z);
					return true;
				}
				printf("Velocities lv:%.3f, av:%.3f not valid\n", twist->linear.x, twist->angular.z);

				
				aux_av = av - (ang_vel_inc_*j);
				twist->linear.x = aux_lv;
				twist->angular.z = aux_av;
				if(checkCommand(twist)) {
					printf("Correct velocities found!!! lv:%.3f, av:%.3f\n", twist->linear.x, twist->angular.z);
					return true;
				}
				printf("Velocities lv:%.3f, av:%.3f not valid\n", twist->linear.x, twist->angular.z);
				
			}
		}
	//}

	twist->linear.x = 0.0;
	twist->angular.z = 0.0;	
	printf("Collision! Stopping the robot!!!\n");
	return false;
}*/




void CollisionDetection::updateSensorReadings()
{
	//Take the laser scan 1
	laser1_mutex_.lock();
	laser1_scan_copy_ = laser1_scan_;
	laser1_mutex_.unlock();
	if(n_lasers_ >= 2)
	{
		//Take the laser scan 2
		laser2_mutex_.lock();
		laser2_scan_copy_ = laser2_scan_;
		laser2_mutex_.unlock();
	}
}


bool CollisionDetection::collision2(float x, float y)
{
	if(inCollision2(x, y, &laser1_scan_copy_))
		return true;

	if(n_lasers_ >= 2) {
		if(inCollision2(x, y, &laser2_scan_copy_))
			return true;
	}

	return false;
}

bool CollisionDetection::collision(float x, float y)
{
	//Take the laser scan 1
	laser1_mutex_.lock();
	sensor_msgs::LaserScan laser1 = laser1_scan_;
	laser1_mutex_.unlock();
	if(inCollision(x, y, &laser1))
		return true;

	if(n_lasers_ >= 2)
	{
		//Take the laser scan 2
		laser2_mutex_.lock();
		sensor_msgs::LaserScan laser1 = laser2_scan_;
		laser2_mutex_.unlock();
		if(inCollision(x, y, &laser1))
			return true;
	}

	return false;
}



bool CollisionDetection::checkCommand(geometry_msgs::Twist* twist)
{
	//boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	saturateVelocities(twist);	

	float lv = twist->linear.x;
	float av = twist->angular.z;

	//Movement from robot frame (base_link)
	//float lin_dist = lv * time_step_;
	//float th = av * time_step_;
	//float x = 0.0 + lin_dist*cos(th);
	//float y = 0.0 + lin_dist*sin(th); 
	//float dist = sqrt(x*x + y*y);
	//float steps = dist/granularity_;

	float vel_mag = sqrt(lv*lv);
	float steps = (vel_mag*sim_time_)/granularity_;
	float dt = sim_time_ / steps;
	float x=0.0, y=0.0, th=0.0;

	//Take the laser scan 1
	laser1_mutex_.lock();
	sensor_msgs::LaserScan laser1 = laser1_scan_;
	laser1_mutex_.unlock();

	int ini = floor(steps/2.0 + 0.5);
	for(unsigned int i=ini; i<steps; i++)
	{
		float lin_dist = lv * dt;
		th = th + (av * dt);
		//normalization just in case
		th = normalizeAngle(th, -M_PI, M_PI);
		x = x + lin_dist*cos(th); //cos(th+av*dt/2.0)
		y = y + lin_dist*sin(th); 
		if(inCollision(x, y, &laser1))
			return false;
	}

	if(n_lasers_ >= 2)
	{
		//Take the laser scan 2
		laser2_mutex_.lock();
		sensor_msgs::LaserScan laser2 = laser2_scan_;
		laser2_mutex_.unlock();

		int ini = floor(steps/2.0 + 0.5);
		for(unsigned int i=ini; i<steps; i++)
		{
			float lin_dist = lv * dt;
			th = th + (av * dt);
			//normalization just in case
			th = normalizeAngle(th, -M_PI, M_PI);
			x = x + lin_dist*cos(th); //cos(th+av*dt/2.0)
			y = y + lin_dist*sin(th); 
			if(inCollision(x, y, &laser2))
				return false;
		}
	}

	// Now we have to check that the robot is
	// able to stop from the final velocity, and
	// and check the possible collisions in the movement
	// until the robot stops. 
		

	return true;
}


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
bool CollisionDetection::checkTraj(double cvx, double cvy, double cvth, double tvx, double tvy, double tvth, double& px, double& py, double& pth)
{
	//boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	//Movement from robot frame (base_link)
	//float lin_dist = lv * time_step_;
	//float th = av * time_step_;
	//float x = 0.0 + lin_dist*cos(th);
	//float y = 0.0 + lin_dist*sin(th); 
	//float dist = sqrt(x*x + y*y);
	//float steps = dist/granularity_;

	px = 0.0;
	py = 0.0;
	pth = 0.0;

	double max_lv = computeNewVelocity(tvx, cvx, max_lin_acc_, sim_time_);
	float vel_mag = sqrt(max_lv*max_lv);
	float steps = (vel_mag*sim_time_)/granularity_;
	float dt = sim_time_ / steps;
	float x=0.0, y=0.0, th=0.0;

	//Take the laser scan 1
	laser1_mutex_.lock();
	sensor_msgs::LaserScan laser1 = laser1_scan_;
	laser1_mutex_.unlock();

	sensor_msgs::LaserScan laser2;
	if(n_lasers_ >= 2)
	{
		//Take the laser scan 2
		laser2_mutex_.lock();
		laser2 = laser2_scan_;
		laser1_mutex_.unlock();
	}

	double lv = cvx;
	double av = cvth;

	//int ini = floor(steps/2.0 + 0.5);
	for(unsigned int i=0; i<steps; i++)
	{
		lv = computeNewVelocity(tvx, lv, max_lin_acc_, dt);
		av = computeNewVelocity(tvth, av, max_ang_acc_, dt);

		float lin_dist = lv * dt;
		th = th + (av * dt);
		//normalization just in case
		th = normalizeAngle(th, -M_PI, M_PI);
		x = x + lin_dist*cos(th); //cos(th+av*dt/2.0)
		y = y + lin_dist*sin(th); 
		if(inCollision(x, y, &laser1))
			return false;
		if(n_lasers_ >= 2) {
			if(inCollision(x, y, &laser2))
				return false;
		}

	}

	px = x;
	py = y;
	pth = th;

	// Now we have to check that the robot is
	// able to stop from the final velocity, and
	// and check the possible collisions in the movement
	// until the robot stops. 
		
	return true;
}




/**
* Check if the indicated point (robot frame) is a valid position
* according to the robot radius and the laserscan
* parameters:
* x, x position in robot frame
* y, y position in robot frame
* scan, laserscan
* return true if a possible collision is detected, false otherwise
*/
bool CollisionDetection::inCollision(float x, float y, sensor_msgs::LaserScan* scan)
{
	
	//polar coordinates
	float d = sqrt(x*x + y*y);
	float th = atan2(y,x);

	//Obtain the index of the ranges array
	int i = 0;
	if(th < 0.0) {
		float ang = (fabs(scan->angle_min) - fabs(th));
		//float ind = round((ang/laser.angle_increment) - 1);
		float ind = floor((ang/scan->angle_increment) + 0.5);
		i = (int)ind;
		
	} else {
		float ind = th/scan->angle_increment;
		ind = scan->angle_max/scan->angle_increment + floor(ind+0.5);
		i = (int) ind;
	}

	float range_dist = scan->ranges[i];
	if(fabs(range_dist - d) <= robot_radius_) {
		return true;
	}

	//Check the ranges according to the robot radius
	//La longitud del arco (L) en una circunferencia, 
	//sabiendo el radio (r) y el ángulo (ɸ) que forman los dos radios, es: L = r * ɸ

	double arc_long = d * scan->angle_increment; //rad 
	double aux = (robot_radius_/arc_long); 
	int nranges = (int) ceil(aux);
	nranges = (int) floor(nranges/2 + 0.5);
	//printf("\narc_long:%.5f, aux:%.5f, nranges:%i totalrang:%u\n", arc_long, aux, nranges, (unsigned int)scan->ranges.size()); 
	if(nranges > scan->ranges.size()) {
		nranges = scan->ranges.size();
	}

	//Ranges to the left
	for(unsigned int j=1; j<=nranges; j++) {
		int newi = (i-j) % scan->ranges.size();
		range_dist = scan->ranges[newi];
		if(fabs(range_dist - d) <= robot_radius_) {
			//printf("collision in the left! ind:%i dist:%.2f\n", newi, range_dist);
			return true;
		}
	}

	//Ranges to the right
	for(unsigned int k=1; k<=nranges; k++) {
		int newi = (i+k) % scan->ranges.size();
		range_dist = scan->ranges[newi];
		if(fabs(range_dist - d) <= robot_radius_) {
			//printf("collision in the right! ind:%i dist:%.2f\n", newi, range_dist);
			return true;
		}
	}

	return false;
}


bool CollisionDetection::inCollision2(float x, float y, sensor_msgs::LaserScan* scan)
{
	float d = sqrt(x*x + y*y);

	for(unsigned int i=0; i<=scan->ranges.size(); i++)
	{
		float range_dist = scan->ranges[i];
		if(fabs(range_dist - d) <= robot_radius_)
			return false; 
	}
	return true;
}



} /* namespace collision detection */
