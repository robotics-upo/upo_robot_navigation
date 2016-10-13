//#ifndef UPO_NAV_MACRO_ACTION_H_
//#define UPO_NAV_MACRO_ACTION_H_

#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <upo_navigation_macro_actions/NavigateWaypointAction.h>
#include <upo_navigation_macro_actions/NavigateHomeAction.h>
#include <upo_navigation_macro_actions/NavigateInteractionTargetAction.h>
#include <upo_navigation_macro_actions/WaitAction.h>
#include <upo_navigation_macro_actions/YieldAction.h>
#include <upo_navigation_macro_actions/WalkSideBySideAction.h>
#include <upo_navigation_macro_actions/AssistedSteeringAction.h>

#include <upo_navigation/upo_navigation.h>
#include <upo_msgs/PersonPoseArrayUPO.h>

#include <upo_navigation_macro_actions/Yield.h>

//For walking side by side
//#include <upo_navigation_macro_actions/WalkSideBySide.h>
#include <teresa_wsbs/start.h>
#include <teresa_wsbs/stop.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

#include <boost/thread/mutex.hpp> //Mutex

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <upo_navigation_macro_actions/NavigationMacroActionsConfig.h>



//namespace macroactions {


	class Upo_navigation_macro_actions
	{
	
		public:
		
			enum datatype{INT_TYPE=1, DOUBLE_TYPE=2, BOOL_TYPE=3, STRING_TYPE=4, GROUP_TYPE=5};
		
			Upo_navigation_macro_actions(tf::TransformListener& tf, upo_nav::UpoNavigation* nav);
			~Upo_navigation_macro_actions();

			void navigateWaypointCB(const upo_navigation_macro_actions::NavigateWaypointGoal::ConstPtr& goal);
			void navigateHomeCB(const upo_navigation_macro_actions::NavigateHomeGoal::ConstPtr& goal);
			void navigateInteractionTargetCB(const upo_navigation_macro_actions::NavigateInteractionTargetGoal::ConstPtr& goal);
			void waitCB(const upo_navigation_macro_actions::WaitGoal::ConstPtr& goal);
			void yieldCB(const upo_navigation_macro_actions::YieldGoal::ConstPtr& goal);
			void walkSideCB(const upo_navigation_macro_actions::WalkSideBySideGoal::ConstPtr& goal);
			void assistedSteeringCB(const upo_navigation_macro_actions::AssistedSteeringGoal::ConstPtr& goal);
			
			void peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg);
			void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
			void wsbsCallback(const std_msgs::UInt8::ConstPtr& msg);
			void rrtGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

			void changeParametersNarrowPlaces();
			void changeParametersNarrowPlaces2();
			bool reconfigureParameters(std::string node, std::string param_name, std::string value, const datatype type);

		private:

			upo_nav::UpoNavigation* UpoNav_;
			
			
			//Dynamic reconfigure
			boost::recursive_mutex configuration_mutex_;
			dynamic_reconfigure::Server<upo_navigation_macro_actions::NavigationMacroActionsConfig> *dsrv_;
			void reconfigureCB(upo_navigation_macro_actions::NavigationMacroActionsConfig &config, uint32_t level);
			

			void fixFrame(std::string& cad);
			float normalizeAngle(float val, float min, float max);
			geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out);
			//geometry_msgs::PoseStamped approachIT(upo_msgs::PersonPoseUPO* person);
			geometry_msgs::PoseStamped approachIT(int id);
			

			tf::TransformListener* tf_listener_;

			ros::NodeHandle nh_;
			ros::NodeHandle nh1_;
			ros::NodeHandle nh2_;
			ros::NodeHandle nh3_;
			ros::NodeHandle nh4_;
			ros::NodeHandle nh5_;
			ros::NodeHandle nh6_;
			ros::NodeHandle nh7_;
		

			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateWaypointAction> NWActionServer;
			NWActionServer* NWActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateHomeAction> NHActionServer;
			NHActionServer* NHActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateInteractionTargetAction> NITActionServer;
			NITActionServer* NITActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::WaitAction> WActionServer;
			WActionServer* WActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::YieldAction> YActionServer;
			YActionServer* YActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::WalkSideBySideAction> WSActionServer;
			WSActionServer* WSActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::AssistedSteeringAction> ASActionServer;
			ASActionServer* ASActionServer_;


			upo_navigation_macro_actions::NavigateWaypointFeedback nwfeedback_;
			upo_navigation_macro_actions::NavigateWaypointResult nwresult_;

			upo_navigation_macro_actions::NavigateHomeFeedback nhfeedback_;
			upo_navigation_macro_actions::NavigateHomeResult nhresult_;

			upo_navigation_macro_actions::NavigateInteractionTargetFeedback nitfeedback_;
			upo_navigation_macro_actions::NavigateInteractionTargetResult nitresult_;

			upo_navigation_macro_actions::WaitFeedback wfeedback_;
			upo_navigation_macro_actions::WaitResult wresult_;
			
			upo_navigation_macro_actions::YieldFeedback yfeedback_;
			upo_navigation_macro_actions::YieldResult yresult_;
			
			upo_navigation_macro_actions::WalkSideBySideFeedback wsfeedback_;
			upo_navigation_macro_actions::WalkSideBySideResult wsresult_;
			
			upo_navigation_macro_actions::AssistedSteeringFeedback asfeedback_;
			upo_navigation_macro_actions::AssistedSteeringResult asresult_;


			//Parameters to be read
			double control_frequency_;
			
			// Wait
			double secs_to_check_block_;
			double block_dist_;
			double secs_to_wait_;
		
			// NavigateInteractionTarget
			int social_approaching_type_;
			
			
			// Yield
			Yield* yield_;
			double secs_to_yield_;
			std::string yieldmap_;
			bool robot_inzone_;
			bool robot_inzone2_;
			bool person_inzone_;
			//bool person_inzone2_;
			boost::mutex rinzone_mutex_;
			boost::mutex pinzone_mutex_;
			geometry_msgs::Pose2D rrtgoal_;
			boost::mutex goal_mutex_;
			geometry_msgs::Pose2D robot_global_pose_;
			boost::mutex global_pose_mutex_;
			bool isYieldDirectionCorrect();
			
			
			// Walk side-by-side
			ros::ServiceClient start_client_;
			ros::ServiceClient stop_client_;
			ros::Subscriber wsbs_status_sub_;
			boost::mutex wsbs_mutex_;
			int wsbs_status_;
			
			
			
			ros::Subscriber people_sub_;
			std::vector<upo_msgs::PersonPoseUPO> people_;
			boost::mutex people_mutex_;
			ros::Subscriber amcl_sub_;
			ros::Subscriber rrtgoal_sub_;

			double initial_stddev_;
			

	};
//};
//#endif
