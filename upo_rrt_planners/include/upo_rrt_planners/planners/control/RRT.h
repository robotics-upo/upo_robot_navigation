#ifndef UPO_RRT_RRT_
#define UPO_RRT_RRT_

#include <upo_rrt_planners/StateSpace.h>
#include <upo_rrt_planners/State.h>
#include <upo_rrt_planners/Action.h>
#include <upo_rrt_planners/Node.h>
#include <upo_rrt_planners/StateChecker.h>
#include <upo_rrt_planners/NearestNeighborsFLANN.h>
#include <upo_rrt_planners/NearestNeighbors.h>
#include <upo_rrt_planners/planners/Planner.h>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>


namespace upo_RRT
{
	class RRT : public upo_RRT::Planner
	{
		public:
			RRT();
			~RRT();

			bool steer(Node* fromNode, Node* toNode, Node* newNode);

			std::vector<upo_RRT::Node> solve(float secs);
			

			/*void setMaxRange(float range) {
				maxRange_ = range;
			}*/
			
			void setTimeStep(float step) {
				steering_->setTimeStep(step);
			}
			
			void setControlSteps(int min_steps, int max_steps) {
				steering_->setMinMaxSteps(min_steps, max_steps);
			}
			
			void setRobotAcc(float linear_acc, float angular_acc) {
				steering_->setAccelerations(linear_acc, angular_acc);
			}


		//private:
		
			//float 				maxRange_; //max distance to insert a new node
			
			//float 				timeStep_; //should be 1/freq  with freq = freq of the controller(15~20Hz)
			//int 				minControlSteps_; //minTime = timeStep*minControlDuration
			//int 				maxControlSteps_;

	};
}
#endif
