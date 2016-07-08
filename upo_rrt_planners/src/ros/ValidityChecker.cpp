
#include <upo_rrt_planners/ros/ValidityChecker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>


upo_RRT_ros::ValidityChecker::ValidityChecker(tf::TransformListener* tf, const costmap_2d::Costmap2D* loc_costmap, const costmap_2d::Costmap2D* glob_costmap, 
		std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, unsigned int dimensions, int distType) : StateChecker()
{
	
	navfeatures_ = new features::NavFeatures(tf, loc_costmap, glob_costmap, footprint, insc_radius, size_x, size_y);
	
	dimensions_ = dimensions;
	distanceType_ = distType;
}


upo_RRT_ros::ValidityChecker::~ValidityChecker() {
	
	delete navfeatures_;
}


bool upo_RRT_ros::ValidityChecker::isValid(upo_RRT::State* s) const
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link"; 
	pose.header.stamp = ros::Time(0);
	pose.pose.position.x = s->getX();
	pose.pose.position.y = s->getY();
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	
	return navfeatures_->poseValid(&pose);

}


void upo_RRT_ros::ValidityChecker::preplanning_computations()
{
	navfeatures_->calculateGaussians();
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
				return dist;
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
				return dist;
			else {
				// Another option
				float dx = (s2->getX()-s1->getX())*cos(s1->getYaw()) + (s2->getY()-s1->getY())*sin(s1->getYaw());
				float dy =-(s2->getX()-s1->getX())*sin(s1->getYaw()) + (s2->getY()-s1->getY())*cos(s1->getYaw()); 
				float alpha = atan2(dy, dx);
				return (0.8*sqrt(dist)+0.2*alpha);
			}
			
		default:
			return dist;
	}
	
}



float upo_RRT_ros::ValidityChecker::getCost(upo_RRT::State* s)
{
	
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link"; 
	pose.header.stamp = ros::Time(0);
	pose.pose.position.x = s->getX();
	pose.pose.position.y = s->getY();
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
	
	return navfeatures_->getCost(&pose);
	
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
	return navfeatures_->transformPoseTo(pose_in, frame_out, usetime);
}

bool upo_RRT_ros::ValidityChecker::isQuaternionValid(const geometry_msgs::Quaternion q) {
	return navfeatures_->isQuaternionValid(q);
}





