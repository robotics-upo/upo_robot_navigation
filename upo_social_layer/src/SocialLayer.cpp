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
	nh.param("only_proxemics", only_proxemics_, false);
	float res = 0.05;

	//robot_frame_ = "base_link";
	global_frame_ = layered_costmap_->getGlobalFrameID();
	navfeatures_ = new features::NavFeatures(tf_listener_, size_x_, size_y_, res); 
	rolling_window_ = layered_costmap_->isRolling();

  	matchSize();

  	dsrv_ = new dynamic_reconfigure::Server<upo_social_layer::SocialPluginConfig>(nh);
  	dynamic_reconfigure::Server<upo_social_layer::SocialPluginConfig>::CallbackType cb = boost::bind(
      &SocialLayer::reconfigureCB, this, _1, _2);
  	dsrv_->setCallback(cb);

	//Load weights "w1, w2, ..."
	w_.clear();
	bool ok = true;
	unsigned int i = 1;
	while(ok)
	{
		char buf[10];
		sprintf(buf, "w%u", i);
		std::string st = std::string(buf);
				
		if(nh.hasParam(st.c_str())){
			double wg = 0.0;
			nh.getParam(st.c_str(), wg);
			w_.push_back((float)wg);	
			printf("SocialLayer. weight %u= %.3f read\n", i, wg);
		} else {
			//printf("param '%s' not found\n", st.c_str());
			ok = false;
		}
		i++;
	}
	
	navfeatures_->setUpoFeatureSet(1); 
	navfeatures_->setWeights(w_);

	//Advertise services
	ros::NodeHandle n("upo_social_layer");
	feat_srv = n.advertiseService("point_features", &SocialLayer::getPointFeatures, this);
	cost_srv = n.advertiseService("point_cost", &SocialLayer::getPointCost, this);
	goal_srv = n.advertiseService("set_goal_feature", &SocialLayer::setGoalForFeatureCalculation, this);
	loss_srv = n.advertiseService("set_use_loss", &SocialLayer::setLossFunc, this);

}


void SocialLayer::matchSize()
{
  	Costmap2D* master = layered_costmap_->getCostmap();
  	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void SocialLayer::reconfigureCB(upo_social_layer::SocialPluginConfig &config, uint32_t level)
{
	printf("Social Layer. Reconfiguring weights:\n");
	wmutex_.lock();
  	enabled_ = config.enabled;
	only_proxemics_ = config.only_proxemics;
	w_.clear();
	float w1 = (float)config.w1;
	w_.push_back(w1);
	printf("w1: %.3f\n", w1);
	float w2 = (float)config.w2;
	w_.push_back(w2);
	printf("w2: %.3f\n", w2);
	float w3 = (float)config.w3;
	w_.push_back(w3);
	printf("w3: %.3f\n", w3);
	float w4 = (float)config.w4;
	w_.push_back(w4);
	printf("w4: %.3f\n", w4);
	float w5 = (float)config.w5;
	w_.push_back(w5);
	printf("w5: %.3f\n", w5);

	navfeatures_->setUpoFeatureSet(1);
	navfeatures_->setWeights(w_);
	wmutex_.unlock();
}





//service
bool SocialLayer::setGoalForFeatureCalculation(upo_social_layer::SetGoal::Request  &req, upo_social_layer::SetGoal::Response &res)
{
	geometry_msgs::PoseStamped pose = req.goal;
	pose.header.stamp = ros::Time::now();
	
	navfeatures_->setGoal(pose);

	return true;
}

//service
bool SocialLayer::setLossFunc(upo_social_layer::SetDemoPath::Request  &req, upo_social_layer::SetDemoPath::Response &res)
{
	std::vector<geometry_msgs::PoseStamped> p = req.demo;
	bool use_loss = req.use_loss_func;
	
	navfeatures_->set_use_loss_func(use_loss); 
	navfeatures_->set_demo_path(p);

	return true;
}



//service
bool SocialLayer::getPointFeatures(upo_social_layer::Features::Request  &req, upo_social_layer::Features::Response &res)
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = global_frame_;
	if(global_frame_.empty() || global_frame_.length() < 3)
		printf("\n¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡getPointFeatures. global_frame invalid!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = req.x;
	pose.pose.position.y = req.y;
	pose.pose.position.z = 0.0;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	std::vector<float> f =  navfeatures_->getFeatures(&pose);
	res.features = f;

	return true;
}


//service
bool SocialLayer::getPointCost(upo_social_layer::Cost::Request  &req, upo_social_layer::Cost::Response &res)
{
	//Two ways:

	//1
	unsigned int mx, my;
	worldToMap(req.x, req.y, mx, my);
	int index = getIndex(mx, my);
	res.cost = costmap_[index]/254.0;
	

	//2
	/*geometry_msgs::PoseStamped pose;
	pose.header.frame_id = global_frame_;
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = req.x;
	pose.pose.position.y = req.y;
	pose.pose.position.z = 0.0;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	res.cost = navfeatures_->getCost(&pose);
	*/
	return true;
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
			if(global_frame_.empty() || global_frame_.length() < 3)
				printf("\n¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡updateCosts. global_frame invalid!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
			rpose.header.stamp = time;
			rpose.pose.position.x = wx;
			rpose.pose.position.y = wy;
			rpose.pose.position.z = 0.0;
			rpose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			float cost = 0.0;
			wmutex_.lock();
			if(!only_proxemics_){
				cost = navfeatures_->getCost(&rpose); //[0,1]
				//printf("c: %.3f, i:%u, j:%u\t", cost, i, j);
			}
			else
				cost = navfeatures_->proxemicsFeature(&rpose);
			wmutex_.unlock();
			if(cost > 1.0)
				cost = 1.0;
			int c = (int)round(cost*254.0);
			//printf("C:%i  \t", c);
	  		int index = getIndex(i, j);
	  		costmap_[index] = c;		  
			//master_grid.setCost(i, j, costmap_[index]); 
		}
	}
	//if(!only_proxemics_)
		//updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j); //overwrite everything
		//updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j); //overwrite only if cell not equal to NO_INFORMATION
	//else
		updateWithMax(master_grid, min_i, min_j, max_i, max_j); //update only if the cost is higher

	
	//updateWithAddition(master_grid, min_i, min_j, max_i, max_j); //Addition of the costs
}



} // end namespace
