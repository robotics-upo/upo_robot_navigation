//#ifndef UPO_NAV_MACRO_ACTION_H_
//#define UPO_NAV_MACRO_ACTION_H_

#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
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
//#include <upo_navigation_macro_actions/NavigateInteractionTargetorGroupAction.h>
#include <upo_navigation_macro_actions/WaitAction.h>

#include <upo_navigation/upo_navigation.h>
#include <upo_msgs/PersonPoseArrayUPO.h>

//#include <boost/thread.hpp>  /* Mutex */
#include <boost/thread/mutex.hpp>

//namespace macroactions {

	//typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateWaypointAction> NWActionServer;
	//typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateHomeAction> NHActionServer;
	//typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateInteractionTargetAction> NITActionServer;
	//typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::WaitAction> WActionServer;

	//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


	class Upo_navigation_macro_actions
	{
	
		public:
			Upo_navigation_macro_actions(tf::TransformListener& tf, upo_nav::UpoNavigation* nav);
			~Upo_navigation_macro_actions();

			void navigateWaypointCB(const upo_navigation_macro_actions::NavigateWaypointGoal::ConstPtr& goal);
			void navigateHomeCB(const upo_navigation_macro_actions::NavigateHomeGoal::ConstPtr& goal);
			void navigateInteractionTargetCB(const upo_navigation_macro_actions::NavigateInteractionTargetGoal::ConstPtr& goal);
			//void navigateInteractionTargetorGroupCB(const upo_navigation_macro_actions::NavigateInteractionTargetorGroupGoal::ConstPtr& goal);
			void waitCB(const upo_navigation_macro_actions::WaitGoal::ConstPtr& goal);
			
			void peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg);

			//void feedbackReceived(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
			//void statusReceived(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
			//void resultReceived(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

			

		private:

			upo_nav::UpoNavigation* UpoNav_;

			void fixFrame(std::string& cad);
			float normalizeAngle(float val, float min, float max);
			geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out);
			geometry_msgs::PoseStamped approachWaypointSimple(upo_msgs::PersonPoseUPO* person, bool* new_waypoint);

			tf::TransformListener* tf_listener_;

			ros::NodeHandle nh_;
			ros::NodeHandle nh1_;
			ros::NodeHandle nh2_;
			ros::NodeHandle nh3_;
			ros::NodeHandle nh4_;
			ros::NodeHandle nh5_;

			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateWaypointAction> NWActionServer;
			NWActionServer* NWActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateHomeAction> NHActionServer;
			NHActionServer* NHActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateInteractionTargetAction> NITActionServer;
			NITActionServer* NITActionServer_;
			//typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::NavigateInteractionTargetorGroupAction> NITGActionServer;
			//NITGActionServer* NITGActionServer_;
			typedef actionlib::SimpleActionServer<upo_navigation_macro_actions::WaitAction> WActionServer;
			WActionServer* WActionServer_;


			upo_navigation_macro_actions::NavigateWaypointFeedback nwfeedback_;
			upo_navigation_macro_actions::NavigateWaypointResult nwresult_;

			upo_navigation_macro_actions::NavigateHomeFeedback nhfeedback_;
			upo_navigation_macro_actions::NavigateHomeResult nhresult_;

			upo_navigation_macro_actions::NavigateInteractionTargetFeedback nitfeedback_;
			upo_navigation_macro_actions::NavigateInteractionTargetResult nitresult_;

			//upo_navigation_macro_actions::NavigateInteractionTargetorGroupFeedback nitfeedback_;
			//upo_navigation_macro_actions::NavigateInteractionTargetorGroupResult nitresult_;

			upo_navigation_macro_actions::WaitFeedback wfeedback_;
			upo_navigation_macro_actions::WaitResult wresult_;

	  		//ros::Subscriber feedback_sub_;
			//ros::Subscriber result_sub_;
			//ros::Subscriber status_sub_;


			//Client to connect with navigation server
			//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
			//MoveBaseClient* ac_;

			//ros::ServiceClient rrtplan_client_;

			//Parameters to be read
			double control_frequency_;
			double secs_to_check_block_;
			double block_dist_;
			double secs_to_wait_;
			bool social_approach_;

			ros::Subscriber people_sub_;
			std::vector<upo_msgs::PersonPoseUPO> people_;
			boost::mutex people_mutex_;

	};
//};
//#endif
