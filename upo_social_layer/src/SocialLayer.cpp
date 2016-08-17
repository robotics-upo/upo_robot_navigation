#include<upo_social_layer/SocialLayer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(social_layer::SocialLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

/*
static const unsigned char 	FREE_SPACE = 0
static const unsigned char 	INSCRIBED_INFLATED_OBSTACLE = 253
static const unsigned char 	LETHAL_OBSTACLE = 254
static const unsigned char 	NO_INFORMATION = 255
*/

namespace social_layer
{

/*SocialLayer::SocialLayer() {

	tf_listener_ = new tf::TransformListener();
	size_x_ = 5.0;
	size_y_ = 5.0;
	robot_frame_ = "base_link";
	navfeatures_ = new features::NavFeatures(tf_listener_, size_x_, size_y_); 
	rolling_window_ = layered_costmap_->isRolling();
}*/


/*SocialLayer::SocialLayer(tf::TransformListener* tf, float size_x, float size_y, std::string robot_frame) {

	tf_listener_ = tf;
	size_x_ = size_x;
	size_y_ = size_y;
	robot_frame_ = robot_frame;
	navfeatures_ = new features::NavFeatures(tf_listener_, size_x_, size_y_); 
	rolling_window_ = layered_costmap_->isRolling();
}*/

SocialLayer::~SocialLayer()
{
     if (dsrv_)
         delete dsrv_;
}


void SocialLayer::onInitialize()
{
  	ros::NodeHandle nh("~/" + name_);
  	current_ = true;
  	default_value_ = NO_INFORMATION;

  	tf_listener_ = new tf::TransformListener();

	nh.param("size_x", size_x_, 10.0);
	nh.param("size_y", size_y_, 10.0);
	nh.param("all_features", all_features_, false);
	//robot_frame_ = "base_link";
	global_frame_ = layered_costmap_->getGlobalFrameID();
	navfeatures_ = new features::NavFeatures(tf_listener_, size_x_, size_y_); 
	rolling_window_ = layered_costmap_->isRolling();

  	matchSize();

  	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SocialLayer::reconfigureCB, this, _1, _2);
  	dsrv_->setCallback(cb);
}


void SocialLayer::matchSize()
{
  	Costmap2D* master = layered_costmap_->getCostmap();
  	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void SocialLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  	enabled_ = config.enabled;
}

void SocialLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{

	if (rolling_window_) {
		updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
		//printf("updating bounds x: %.2f, y: %.2f\n", robot_x - getSizeInMetersX() / 2, (robot_y - getSizeInMetersY() / 2));
	}
 	if (!enabled_)
    	return;

	 useExtraBounds(min_x, min_y, max_x, max_y);
}


void SocialLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
 	if (!enabled_)
    	return;

	navfeatures_->update();

	ros::Time time = ros::Time::now();
	double wx, wy;
	for (unsigned int j = min_j; j < max_j; j++)
	{
		for (unsigned int i = min_i; i < max_i; i++)
		{
			mapToWorld(i, j, wx, wy);
			geometry_msgs::PoseStamped rpose;
			rpose.header.frame_id = global_frame_;
			rpose.header.stamp = time;
			rpose.pose.position.x = wx;
			rpose.pose.position.y = wy;
			rpose.pose.position.z = 0.0;
			rpose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			float cost = 0.0;
			if(all_features_)
				cost = navfeatures_->getCost(&rpose); //[0,1]
			else
				cost = navfeatures_->proxemicsFeature(&rpose);

			int c = (int)round(cost*254.0);
			//printf("C:%i  \t", c);
	  		int index = getIndex(i, j);
	  		costmap_[index] = c;		  
			//master_grid.setCost(i, j, costmap_[index]); 
		}
	}
	updateWithMax(master_grid, min_i, min_j, max_i, max_j); //update only if the cost is higher
	//updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j); //overwrite only if cell not equal to NO_INFORMATION
	//updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j); //overwrite everything
	//updateWithAddition(master_grid, min_i, min_j, max_i, max_j); //Addition of the costs
}



} // end namespace
