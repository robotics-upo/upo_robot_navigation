#ifndef SOCIAL_LAYER_H_
#define SOCIAL_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <tf/transform_listener.h>

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <upo_social_layer/SocialPluginConfig.h>

//Boost
#include <boost/thread.hpp>  // Mutex 

//Services
#include <upo_social_layer/Features.h>
#include <upo_social_layer/Cost.h>
#include <upo_social_layer/SetGoal.h>
#include <upo_social_layer/SetDemoPath.h>

//navigation features calculation
#include <navigation_features/nav_features.h>


namespace social_layer
{

	class SocialLayer : public costmap_2d::CostmapLayer    //public costmap_2d::Layer, public costmap_2d::Costmap2D
	{
		public:
		  //SocialLayer();
		  //SocialLayer(tf::TransformListener* tf, float size_x, float size_y, std::string robot_frame);
		  ~SocialLayer();

		  virtual void onInitialize();
		  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
				                     double* max_y);
		  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
		  bool isDiscretized()
		  {
			return true;
		  }

		  virtual void matchSize();



		  //Services
		  bool getPointFeatures(upo_social_layer::Features::Request  &req, upo_social_layer::Features::Response &res);
		  bool getPointCost(upo_social_layer::Cost::Request  &req, upo_social_layer::Cost::Response &res);
		  bool setGoalForFeatureCalculation(upo_social_layer::SetGoal::Request  &req, upo_social_layer::SetGoal::Response &res);
		  bool setLossFunc(upo_social_layer::SetDemoPath::Request  &req, upo_social_layer::SetDemoPath::Response &res);
		  

		private:
		  dynamic_reconfigure::Server<upo_social_layer::SocialPluginConfig> *dsrv_;
		  void reconfigureCB(upo_social_layer::SocialPluginConfig &config, uint32_t level);
		  

		  tf::TransformListener* 		tf_listener_;
		  features::NavFeatures* 		navfeatures_;
		  double 						size_x_;
		  double						size_y_;
		  //std::string					robot_frame_;
		  std::string 					global_frame_;
		  bool 							rolling_window_;
		  bool 							only_proxemics_;

		  //Weights to balance the costs
		  std::vector<float> 			w_;
		  boost::mutex 					wmutex_;

		  //Services
		  ros::ServiceServer feat_srv;
		  ros::ServiceServer cost_srv;
		  ros::ServiceServer goal_srv;
		  ros::ServiceServer loss_srv;
		
	};
}
#endif
