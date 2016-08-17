#ifndef UPO_RRT_STATE_SPACE_
#define UPO_RRT_STATE_SPACE_


#include <upo_rrt_planners/State.h>
#include <upo_rrt_planners/StateChecker.h>
#include <upo_rrt_planners/RandomNumbers.h>

// For pre C++ 11 gamma function
#include <boost/math/special_functions/gamma.hpp>

#include <vector>
#include <cmath>
#include <stdio.h>


namespace upo_RRT
{
	class StateSpace
	{
		public:
			StateSpace();

			StateSpace(StateChecker* stateChecker, unsigned int dim, float sx, float sy, float xyres = 0.1, float yawres = 0.02, 
				float min_lv = 0.1, float max_lv = 0.5, float lvres = 0.05, float max_av = 0.5, 
				float avres = 0.1);
		
		    ~StateSpace();

			State* sampleState();
			State* sampleStateFree();
			State* sampleStateNear(State* st);
			//State* sampleStateNearFree(State* st);
			
			State* samplePathBiasing(std::vector<State>* path, float stddev, float yawdev = 0.2);
			
			float sampleUniform(); //value between [0, 1]

			float distance(State* s1, State* s2);
			float euclideanDistance(State* s1, State* s2);

			bool isStateValid(State* s);
			
			float getCost(State* s);

			bool isSimpleGoalToleranceSatisfied(State* st, float &dist);
			
			bool isGoalToleranceSatisfied(State* st, float &dist);
			
			float getSpaceMeasure();
			float calculeUnitBallMeasure(unsigned int d, double r);
			float getUnitBallMeasure();
			
			
			float normalizeAngle(float val, float min, float max);
			
		
			unsigned int getDimensions();
			std::vector<unsigned int> getWeights();
			float getSizeX();
			float getSizeY();
			float getXYresolution();
			float getYawResolution();
			float getMinLinVel();
			float getMaxLinVel();
			float getLinVelResolution();
			float getMaxAngVel();
			float getAngVelResolution();
			float getGoalXYTolerance();
			float getGoalTHTolerance();
			State* getStart();
			State* getGoal();

			void setDimensions(unsigned int d);
			void setWeights(std::vector<unsigned int> w);
			void setSizeX(float sx);
			void setSizeY(float sy);
			void setXYresolution(float res);
			void setYawResolution(float res);
			void setMinLinVel(float v);
			void setMaxLinVel(float v);
			void setLinVelResolution(float res);
			void setMaxAngVel(float v);
			void setAngVelResolution(float res);
			void setGoalTolerance(float xy_tol, float th_tol);
			bool setStart(State* s);
			bool setGoal(State* g);
			bool setStartAndGoal(State* s, State* g);
			
			
			StateChecker*				stateChecker_;
			
		

		private:

			// ROS
			//geometry_msgs::Pose			robotOdomPose;
			//costmap_2d::Costmap2DROS* 	global_costmap_ros_;
		  	//costmap_2d::Costmap2D* 		global_costmap_;
			//costmap_2d::Costmap2DROS* 	local_costmap_ros_;
		  	//costmap_2d::Costmap2D* 		local_costmap_;

			//tf::TransformListener*		tf_;
			//WorldModel*					local_world_;
			//WorldModel*					global_world_;
			//float 						robot_inscribed_radius_;
			//float 						robot_circumscribed_radius_;

			
		
			RNG							random_;

			unsigned int 				dimensions_;

			std::vector<unsigned int> 	weights_;

			float 						size_x_;
			float 						size_y_;
			float 						xy_resolution_; 
			float						yaw_resolution_; 
			
			float 						space_volume_;
			float 						unit_ball_measure_;

			float						min_lin_vel_;
			float						max_lin_vel_;
			float						lin_vel_res_;
			std::vector<float>			lin_vels_;

			float						max_ang_vel_;
			float						ang_vel_res_;
			std::vector<float>			ang_vels_;

			float 						goal_xy_tolerance_;
			float                       goal_th_tolerance_;

			State*						start_;
			State*						goal_;
		
	};
}
#endif
