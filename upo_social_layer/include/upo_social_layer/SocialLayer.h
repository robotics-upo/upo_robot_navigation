#ifndef SOCIAL_LAYER_H_
#define SOCIAL_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>

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

		private:
		  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
		  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

		  tf::TransformListener* 		tf_listener_;
		  features::NavFeatures* 		navfeatures_;
		  double 						size_x_;
		  double						size_y_;
		  //std::string					robot_frame_;
		  std::string 					global_frame_;
		  bool 							rolling_window_;
		  bool 							all_features_;

		
	};
}
#endif
