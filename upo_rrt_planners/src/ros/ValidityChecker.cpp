
#include <upo_rrt_planners/ros/ValidityChecker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>


upo_RRT_ros::ValidityChecker::ValidityChecker(bool use_fc_costmap, tf::TransformListener* tf, const costmap_2d::Costmap2D* loc_costmap, const costmap_2d::Costmap2D* glob_costmap, 
		std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, unsigned int dimensions, int distType) : StateChecker()
{
	
	get_cost_from_costmap_ = use_fc_costmap;
	
	if(glob_costmap == NULL)
		use_global_costmap_ = false;
	else
		use_global_costmap_ = true;
	
	if(!get_cost_from_costmap_)
		navfeatures_ = new features::NavFeatures(tf, loc_costmap, glob_costmap, footprint, insc_radius, size_x, size_y);
	else {
		printf("----Using cost function to build a costmap-----\n");
		loc_costmap_ = loc_costmap;
		glo_costmap_ = glob_costmap;
		tf_ = tf;
	}
	dimensions_ = dimensions;
	distanceType_ = distType;
	time_ = ros::Time::now();
}


upo_RRT_ros::ValidityChecker::~ValidityChecker() {
	
	delete navfeatures_;
}


bool upo_RRT_ros::ValidityChecker::isValid(upo_RRT::State* s) const
{
	geometry_msgs::PoseStamped p_in;
	p_in.header.frame_id = "base_link"; 
	//p_in.header.stamp = ros::Time(0); //this is a problem when the planning time is long. the time stamp should be the time when the rrt started to plan.
	if((ros::Time::now()-time_).toSec() > 2.0) {
			//time_ = ros::Time::now();
			p_in.header.stamp = ros::Time(0);
	} else 
		p_in.header.stamp = time_;
	
	p_in.pose.position.x = s->getX();
	p_in.pose.position.y = s->getY();
	p_in.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	
	
	if(!get_cost_from_costmap_)  
	{
		//If we calculate the validity in a normal way 
		return navfeatures_->poseValid(&p_in);
		
		
	} else {  
		//we check the validity checking the value of the costmap built by using the RRT* cost function
		
		//Take the values of the state
		float x_i = s->getX();
		float y_i = s->getY();

		float local_cost = 0.0;
		float global_cost = 0.0;
		//used to put things into grid coordinates
		unsigned int cell_x, cell_y;
		unsigned char cost;

		if(use_global_costmap_) {
			//Global costmap ------------------------------------------
			geometry_msgs::PoseStamped p_out_map;
			try {					
				tf_->transformPose("map", p_in, p_out_map); 
			}catch (tf::TransformException ex){
				ROS_WARN("isValid. x:%.2f, y:%.2f. TransformException: %s",x_i, y_i, ex.what());
				return false;
			}
			//get the cell coord of the center point of the robot
			if(!glo_costmap_->worldToMap((double)p_out_map.pose.position.x, (double)p_out_map.pose.position.y, cell_x, cell_y))
				return false;

			//if number of points in the footprint is less than 3, we'll just assume a circular robot
			//if(myfootprint->size() < 3){
			cost = glo_costmap_->getCost(cell_x, cell_y);
			if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION || cost == -1)
				return false;
			//}
		}

		//Local costmap---------------------------------------------
		geometry_msgs::PoseStamped p_out_odom;
		try {					
			tf_->transformPose("odom", p_in, p_out_odom); 
		}catch (tf::TransformException ex){
			ROS_WARN("isValid. x:%.2f, y:%.2f. TransformException: %s",x_i, y_i, ex.what());
			return false;
		}

		//get the cell coord of the center point of the robot
		if(!loc_costmap_->worldToMap((double)p_out_odom.pose.position.x, (double)p_out_odom.pose.position.y, cell_x, cell_y))
		  return false;

		//if number of points in the footprint is less than 3, we'll just assume a circular robot
		//if(myfootprint->size() < 3){
		cost = loc_costmap_->getCost(cell_x, cell_y);
		if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION || cost == -1)
			return false;
		//}

		/*
		//now we really have to lay down the footprint in the local costmap grid
		unsigned int x0, x1, y0, y1;
		double line_cost = 0.0;
		double footprint_cost = 0.0;

		//we need to rasterize each line in the footprint
		for(unsigned int i = 0; i < footprint->size() - 1; ++i){
			//get the cell coord of the first point
			if(!costmap_local->worldToMap(footprint->at(i).x, footprint->at(i).y, x0, y0))
				return false;

			//get the cell coord of the second point
			if(!costmap_local->worldToMap(footprint->at(i+1).x, footprint->at(i+1).y, x1, y1))
				return false;

			line_cost = lineCost(x0, x1, y0, y1);
			footprint_cost = std::max(line_cost, footprint_cost);

			//if there is an obstacle that hits the line... we know that we can return false right away 
			if(line_cost < 0)
				return false;
		}

		//we also need to connect the first point in the footprint to the last point
		//get the cell coord of the last point
		if(!costmap_local->worldToMap(footprint->back().x, footprint->back().y, x0, y0))
			return false;

		//get the cell coord of the first point
		if(!costmap_local->worldToMap(footprint->front().x, footprint->front().y, x1, y1))
			return false;

		line_cost = lineCost(x0, x1, y0, y1);
		footprint_cost = std::max(line_cost, footprint_cost);

		if(line_cost < 0)
			return false;
		*/

		return true;
		
	}
}


void upo_RRT_ros::ValidityChecker::preplanning_computations()
{
	if(!get_cost_from_costmap_)
		navfeatures_->update();
}


float upo_RRT_ros::ValidityChecker::distance(upo_RRT::State* s1, upo_RRT::State* s2) const
{
	float dx = s1->getX() - s2->getX();
	float dy = s1->getY() - s2->getY();
	//float dist = sqrt(dx*dx + dy*dy);
	float dist = dx*dx + dy*dy;
	
	switch(distanceType_) {
		
		case 1:
			return dist;

		case 2:
			return sqrt(dist);

		case 3:
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				//SUM w1*|| Pi+1 - Pi|| + w2*(1-|Qi+1 * Qi|)²
				float euc_dist = sqrt(dist);
		
				tf::Quaternion q1 = tf::createQuaternionFromYaw(s1->getYaw());
				tf::Quaternion q2 = tf::createQuaternionFromYaw(s2->getYaw());
				float dot_prod = q1.dot(q2);
				float angle_dist =  (1 - fabs(dot_prod))*(1 - fabs(dot_prod));
				//printf("eu_dist: %.2f, angle_dist: %.3f, dist: %.3f\n", euc_dist, angle_dist, 0.8*euc_dist + 0.2*angle_dist);
				return 0.7*euc_dist + 0.3*angle_dist;
			}
			
		case 4:
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				// Another option
				/*
				First, transform the robot location into person location frame: 
											|cos(th)  sin(th)  0|
					Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
											|  0        0      1|
												 
					x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
					y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
				*/
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				return (0.8*sqrt(dist)+0.2*fabs(alpha));
			}
			
		case 5:  
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				//UPO. Dist + sum of the angles of both points regarding the intersection line
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				float beta = s2->getYaw() - alpha;
				beta = navfeatures_->normalizeAngle(beta, -M_PI, M_PI);
				return (0.6*sqrt(dist)+0.4*(fabs(alpha)+fabs(beta)));
			}
			
		case 6:  
			if(dimensions_ == 2)
				return sqrt(dist);
			else {
				//Paper IROS2015 "Feedback motion planning via non-holonomic RRT* for mobile robots"
				float x = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float y =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(y, x);
				float phi = s2->getYaw() - alpha;
				phi = navfeatures_->normalizeAngle(phi, -M_PI, M_PI);
				float ka = 0.5;
				float ko = ka/8.0;
				dist = sqrt(dist);
				// two options
				float alpha_prime = atan(-ko*phi);
				//float alpha_prime = atan(-ko*ko * phi/(dist*dist));
				float r = navfeatures_->normalizeAngle((alpha-alpha_prime), -M_PI, M_PI);
				return (sqrt(dist*dist + ko*ko + phi*phi) + ka*fabs(r));
			}
			
		default:
			return sqrt(dist);
	}
	
}



float upo_RRT_ros::ValidityChecker::getCost(upo_RRT::State* s)
{
	
	if(get_cost_from_costmap_) {
		
		//Coordinates X and Y are in the robot local frame (base_link), we transform them to odom
		geometry_msgs::PointStamped p_in;
		p_in.header.frame_id = "base_link"; 
		p_in.header.stamp = ros::Time(0);
		p_in.point.x = s->getX();
		p_in.point.y = s->getY();
				
		float local_cost = 0.0;
		//float global_cost = 0.0;
		//used to put things into grid coordinates
		unsigned int cell_x, cell_y;
		unsigned char cost;


		//Local costmap---------------------------------------------
		geometry_msgs::PointStamped p_out_odom;
		try {					
			tf_->transformPoint("odom", p_in, p_out_odom); 
		}catch (tf::TransformException ex){
			ROS_WARN("ValidityChecker. GetCost. x:%.2f, y:%.2f. TransformException: %s",p_in.point.x, p_in.point.y, ex.what());
			-100.0;
		}

		//get the cell coord of the center point of the robot
		if(!loc_costmap_->worldToMap((double)p_out_odom.point.x, (double)p_out_odom.point.y, cell_x, cell_y)) {
			ROS_WARN("ValidityChecker. GetCost. Error in worldToMap conversion in local costmap!!! Returning highest cost");
			return 255.0/255.0;
		}

		local_cost = (loc_costmap_->getCost(cell_x, cell_y))/255.0;
		//printf("LC: %.1f \t", local_cost);
				
		return local_cost;
		
		
	} else  {
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "base_link"; 
		if((ros::Time::now()-time_).toSec() > 2.0)
			time_ = ros::Time::now();
		pose.header.stamp = time_; 
		//pose.header.stamp = ros::Time(0);
		pose.pose.position.x = s->getX();
		pose.pose.position.y = s->getY();
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
		float cost = navfeatures_->getCost(&pose);
		return cost;
	}
	
}



std::vector<float> upo_RRT_ros::ValidityChecker::getFeatures(upo_RRT::State* s) 
{
	
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link"; 
	pose.header.stamp = ros::Time(0);
	pose.pose.position.x = s->getX();
	pose.pose.position.y = s->getY();
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	
	std::vector<float> features = navfeatures_->getFeatures(&pose);
	
	return features;
}




void upo_RRT_ros::ValidityChecker::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
	navfeatures_->setPeople(p);
}


void upo_RRT_ros::ValidityChecker::setWeights(std::vector<float> we) {
	navfeatures_->setWeights(we);
}


geometry_msgs::PoseStamped upo_RRT_ros::ValidityChecker::transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime) {
	
	if(!get_cost_from_costmap_) {
		return navfeatures_->transformPoseTo(pose_in, frame_out, usetime);
	} else {
		geometry_msgs::PoseStamped in = pose_in;
		if(!usetime)
			in.header.stamp = ros::Time();
			
		geometry_msgs::PoseStamped pose_out;
		
		/*geometry_msgs::Quaternion q = in.pose.orientation;
		if(!isQuaternionValid(q))
		{
			ROS_WARN("NavFeatures. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
			in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		}*/
		try {
			tf_->transformPose(frame_out.c_str(), in, pose_out);
		}catch (tf::TransformException ex){
			ROS_WARN("ValidityChecker. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
		}
		//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
		return pose_out;
	}
}

bool upo_RRT_ros::ValidityChecker::isQuaternionValid(const geometry_msgs::Quaternion q) {
	
	if(!get_cost_from_costmap_) {
		return navfeatures_->isQuaternionValid(q);
	} else {
		//first we need to check if the quaternion has nan's or infs
		if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
			ROS_ERROR("Quaternion has infs!!!!");
			return false;
		}
		if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
			ROS_ERROR("Quaternion has nans !!!");
			return false;
		}
		
		if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
			ROS_ERROR("Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
			return false;
		}

		tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

		//next, we need to check if the length of the quaternion is close to zero
		if(tf_q.length2() < 1e-6){
		  ROS_ERROR("Quaternion has length close to zero... discarding.");
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
}





