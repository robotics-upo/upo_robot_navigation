
#include <assisted_steering/AssistedSteering.h>


namespace assisted_steering {

AssistedSteering::AssistedSteering(tf::TransformListener* tf) {
	
	tf_ = tf;
	setup();
}


AssistedSteering::~AssistedSteering() {
	delete odom_helper_;
}


void AssistedSteering::setup() {
	
	ros::NodeHandle n("~");

	n.param<std::string>("laser_topic", laser_topic_, std::string("scan360"));
	n.param<std::string>("odom_topic", odom_topic_, std::string("odom"));
	n.param<std::string>("cmdvel_topic", cmdvel_topic_, std::string("teresa/cmd_vel"));
	n.param<std::string>("new_cmdvel_topic", new_cmdvel_topic_, std::string("cmd_vel"));
	n.param<double>("robot_radius", robot_radius_, 0.32);
	n.param<double>("granularity", granularity_, 0.025);
	n.param<double>("max_lin_vel", max_lin_vel_, 0.6);
	n.param<double>("max_ang_vel", max_ang_vel_, 0.6);
	n.param<double>("max_lin_acc", max_lin_acc_, 1.0);
	n.param<double>("max_ang_acc", max_ang_acc_, 1.0);
	n.param<double>("time_step", time_step_, 0.05);
	n.param<bool>("is_active", isActive_, true);
	max_lv_var_ = max_lin_acc_ * time_step_;
	max_av_var_ = max_ang_acc_ * time_step_;


	odom_helper_ = new OdometryHelperRos(odom_topic_);

	ros::NodeHandle nh;
	out_cmdvel_pub_ = nh.advertise<geometry_msgs::Twist>(new_cmdvel_topic_, 1);

	laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic_.c_str(), 1, &AssistedSteering::laserCallback, this); 
	cmdvel_sub_ = nh.subscribe<geometry_msgs::Twist>(cmdvel_topic_.c_str(), 1, &AssistedSteering::cmdvelCallback, this);

	//Dynamic reconfigure
	dsrv_ = new dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>(n); //ros::NodeHandle("~")
    dynamic_reconfigure::Server<assisted_steering::AssistedSteeringConfig>::CallbackType cb = boost::bind(&AssistedSteering::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

}


void AssistedSteering::reconfigureCB(assisted_steering::AssistedSteeringConfig &config, uint32_t level){
    
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    
	max_lin_vel_ = config.max_lin_vel;
	max_ang_vel_ = config.max_ang_vel;
	max_lin_acc_ = config.max_lin_acc;
	max_ang_acc_ = config.max_ang_acc;
	time_step_ = config.time_step;
	robot_radius_ = config.robot_radius;
	granularity_ = config.granularity;
	isActive_ = config.is_active;

	max_lv_var_ = max_lin_acc_ * time_step_;
	max_av_var_ = max_ang_acc_ * time_step_;

}


void AssistedSteering::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	ROS_INFO_ONCE("Laser received!");
	//IMPORTANT: the frame of the laser should be the center of the robot (base_link)
	//Otherwise we should include a shift to the center in the calculations.
	laser_mutex_.lock();
	laser_scan_ = *msg;
	laser_mutex_.unlock();
}



void AssistedSteering::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
	twist_mutex_.lock();
	twist_ = *msg;
	twist_mutex_.unlock();


	boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	if(isActive_) {
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
	
}




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
}


void AssistedSteering::saturateVelocities(geometry_msgs::Twist* twist)
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



bool AssistedSteering::findValidCmd(geometry_msgs::Twist* twist)
{

	float lv = twist->linear.x;
	float av = twist->angular.z;

	float aux_lv = lv;
	float aux_av = av;

	float ang_inc = 0.15;
	float lin_inc = 0.1;

	float slot = 0.0;
	//if(lv > 0.0) {
		//Linear vels
		for(unsigned int i=0; i<3; i++)
		{
			aux_lv = lv - (lin_inc*i);
			//Angular vels
			for(unsigned int j=1; j<=3; j++)
			{
				aux_av = av + (ang_inc*j);
				twist->linear.x = aux_lv;
				twist->angular.z = aux_av;
				if(checkCommand(twist)) {
					printf("Correct velocities found!!! lv:%.3f, av:%.3f\n", twist->linear.x, twist->angular.z);
					return true;
				}
				printf("Velocities lv:%.3f, av:%.3f not valid\n", twist->linear.x, twist->angular.z);

				
				aux_av = av - (ang_inc*j);
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
}



bool AssistedSteering::checkCommand(geometry_msgs::Twist* twist)
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
	float steps = (vel_mag*time_step_)/granularity_;
	float dt = time_step_ / steps;
	float x=0.0, y=0.0, th=0.0;

	//Take the laser scan
	laser_mutex_.lock();
	sensor_msgs::LaserScan laser = laser_scan_;
	laser_mutex_.unlock();

	int ini = floor(steps/2.0 + 0.5);
	for(unsigned int i=ini; i<steps; i++)
	{
		float lin_dist = lv * dt;
		th = th + (av * dt);
		//normalization just in case
		th = normalizeAngle(th, -M_PI, M_PI);
		x = x + lin_dist*cos(th); //cos(th+av*dt/2.0)
		y = y + lin_dist*sin(th); 
		if(inCollision(x, y, &laser))
			return false;
	}

	// Now we have to check that the robot is
	// able to stop from the final velocity, and
	// and check the possible collisions in the movement
	// until the robot stops. 
		

	return true;
}


bool AssistedSteering::inCollision(float x, float y, sensor_msgs::LaserScan* scan)
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
		//printf("-----------collision in the front-------------\n");
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



} /* namespace assisted_steering */
