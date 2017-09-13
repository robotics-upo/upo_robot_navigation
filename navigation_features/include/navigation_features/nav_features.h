
#ifndef NAV_FEATURES_
#define NAV_FEATURES_

#include <math.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <upo_msgs/PersonPoseUPO.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/GetMap.h>
//PCL
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/register_point_struct.h>

//Mutex
#include <mutex>  

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"



//Service msg
#include <navigation_features/SetWeights.h>
#include <navigation_features/SetLossCost.h>
#include <navigation_features/SetScenario.h>
#include <navigation_features/PoseValid.h>
#include <navigation_features/GetFeatureCount.h>
#include <navigation_features/InitWeights.h>

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <navigation_features/nav_featuresConfig.h>


using namespace std;


namespace features {

	class NavFeatures {

		public:
		
			enum gaussian_type{
				FRONT, 
				BACK, 
				LEFT, 
				RIGHT, 
				AROUND, 
				FRONT_APPROACH,
				AROUND_APPROACH
			};
			
			enum dist_type{LINEAR_INC,LOG_INC,EXP_INC,INVERSE_DEC,LOG_DEC,EXP_DEC};

			NavFeatures();
			
			NavFeatures(tf::TransformListener* tf, float size_x, float size_y, float res);
			
			//NavFeatures(tf::TransformListener* tf, const costmap_2d::Costmap2D* loc_costmap, const costmap_2d::Costmap2D* glob_costmap, std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y);

			NavFeatures(tf::TransformListener* tf, vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y, float res);

			~NavFeatures();

			void setParams();

			bool poseValid(geometry_msgs::PoseStamped* pose);

			//bool poseValid_projection(geometry_msgs::PoseStamped* pose);

			float getCost(geometry_msgs::PoseStamped* s);
		
			std::vector<float> getFeatures(geometry_msgs::PoseStamped* s);

			std::vector<float> getPathFeatureCount(vector<geometry_msgs::PoseStamped>* path);
			
			void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
			
			//void goalAstarCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

			void peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg);
			
			void pcCallback(const sensor_msgs::PointCloud::ConstPtr& pc_in);
			void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& pc_in);
			
			void setApproachingIT();
			
			
			void update();
			void calculateGaussians();

			//Implemented for learning purposes
			void setPeople(upo_msgs::PersonPoseArrayUPO p);
		
			//Feature: Distance to the goal
			float goalDistFeature(geometry_msgs::PoseStamped* s);
		
			//Feature: Distance to the closest obstacle (based on costmaps or map image)
			float obstacleDistFeature(geometry_msgs::PoseStamped* s);
		
			//Feature: Proxemics
			float proxemicsFeature(geometry_msgs::PoseStamped* s);
			float getProxemicsCost(float rx, float ry);		
			float gaussian_function(float x, float y, bool fr);
			float gaussian_function(float x, float y, float sx, float sy);
			
			std::vector<float> gaussianFeatures(geometry_msgs::PoseStamped* s);
			
			float calculate_loss_function(geometry_msgs::PoseStamped* p);

			geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime);
		
			bool isQuaternionValid(const geometry_msgs::Quaternion q);
			
			float normalizeAngle(float val, float min, float max);
			
			void setUpoFeatureSet(int s) {
				upo_featureset_ = s;
			}
		
			void setWeights(std::vector<float> we);

			void setGoal(geometry_msgs::PoseStamped g); 

			void setObstacles(sensor_msgs::PointCloud2 obs);

			void setScenario(sensor_msgs::PointCloud2 obs, upo_msgs::PersonPoseArrayUPO people, geometry_msgs::PoseStamped goal);


			//Services
			bool setWeightsService(navigation_features::SetWeights::Request  &req, navigation_features::SetWeights::Response &res);
			bool initializeWeightsService(navigation_features::InitWeights::Request  &req, navigation_features::InitWeights::Response &res);
			bool setLossService(navigation_features::SetLossCost::Request &req, navigation_features::SetLossCost::Response &res);
			bool setScenarioService(navigation_features::SetScenario::Request  &req, navigation_features::SetScenario::Response &res);
			bool isPoseValidService(navigation_features::PoseValid::Request &req, navigation_features::PoseValid::Response &res);
			bool getFeatureCountService(navigation_features::GetFeatureCount::Request &req, navigation_features::GetFeatureCount::Response &res);
			
			void set_use_loss_func(bool s) { 
				loss_mutex_.lock();
				use_loss_func_ = s; 
				loss_mutex_.unlock();
			}
			void set_demo_path(std::vector<geometry_msgs::PoseStamped>* p) {
				loss_mutex_.lock();
				demo_path_ = *p;
				loss_mutex_.unlock();
			}
			void set_demo_path(std::vector<geometry_msgs::PoseStamped> p) {
				loss_mutex_.lock();
				demo_path_ = p;
				loss_mutex_.unlock();
				printf("Setting use_loss: %i. Demo path with size: %u\n", use_loss_func_, (unsigned int)demo_path_.size());
			}
			
			
			//For laser projection
			void setupMapProjection();
			void setupNoMapProjection();
			float distance_functions(const float distance, const dist_type type);
			void updateDistTransform();
			vector<int> worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata);
			vector<int> BaseLinkWorldToImg(geometry_msgs::Point32* point);


		private:
		
		
			struct gaussian {	
				float x;
				float y;
				float th;
				float sx;
				float sy;
				gaussian_type type;
			};
			
			
			struct group_candidate {
				geometry_msgs::Pose2D position;
				geometry_msgs::Pose2D interaction_point;
				int id;
				int group_id;
			};
			
			ros::Publisher 						pub_gaussian_markers_;
			
			vector<gaussian> 					gaussians_;
			mutex 								gaussianMutex_;

			//For laser projection
			ros::Subscriber 					sub_pc_;
			cv::Mat 							map_image_;
			cv::Mat 							distance_transform_;
			nav_msgs::MapMetaData 				map_metadata_;
			double 								resolution_;
			vector<float>	 					origin_;
			laser_geometry::LaserProjection 	projector_;
			sensor_msgs::PointCloud2 			laser_cloud_;
			mutex 								laserMutex_;
			mutex 								dtMutex_;
			int 								people_paint_area_; // the amount of pixels to be painted over at the presence of people
			float 								max_cost_obs_;
			
			bool 								use_global_map_;

			
			tf::TransformListener* 				tf_listener_;
			vector<geometry_msgs::Point>* 		myfootprint_;
			float 								insc_radius_robot_;
			geometry_msgs::PoseStamped 			goal_;
			int 								goal_type_;
			float 								max_planning_dist_;
			float 								size_x_;
			float 								size_y_;

			ros::NodeHandle 					nh_;
		
			// list of person objects	
			vector<upo_msgs::PersonPoseUPO> people_;
			ros::Subscriber 					sub_people_;
			mutex 								peopleMutex_;
			string		 						people_frame_id_;
			
			//Id of the interaction target (if he/she exists)
			int									it_id_;
			bool								it_remove_gauss_;
			float 								approaching_angle_;
			
			ros::Subscriber 					goal_sub_;
			
			//Grouping people
			bool 								grouping_;
			float 								stddev_group_;
			float 								grouping_distance_;
			
		
			//parameters of the Gaussian functions
			float 								amp_;
			vector<float>	 					sigmas_;
		
			//Weights to balance the costs
			vector<float>	 					w_;

			
			
			//upo feature set to be used
			int 								upo_featureset_;
			
			//Only for learning algorithms
			ros::ServiceServer					loss_srv_;
			bool								use_loss_func_;
			vector<geometry_msgs::PoseStamped> 	demo_path_;
			mutex								loss_mutex_;
			
			//Service for point validity checking
			ros::ServiceServer					valid_srv_;

			//services
			ros::ServiceServer 					weights_srv_;
			ros::ServiceServer					init_weights_srv_;
			ros::ServiceServer					scenario_srv_;
			ros::ServiceServer					features_srv_;
			
			//Dynamic reconfigure
			boost::recursive_mutex configuration_mutex_;
			//mutex								configuration_mutex_;
			dynamic_reconfigure::Server<navigation_features::nav_featuresConfig> *dsrv_;
			void reconfigureCB(navigation_features::nav_featuresConfig &config, uint32_t level);
			

	};

}
#endif
