#ifndef UPO_RRT_SIMPLE_RRT_
#define UPO_RRT_SIMPLE_RRT_

#include <upo_rrt_planners/StateSpace.h>
#include <upo_rrt_planners/State.h>
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
	class SimpleRRT : public upo_RRT::Planner
	{
		public:
			SimpleRRT();
			~SimpleRRT();

			State* steer(State* fromState, State* toState, std::vector<State>& istates);

			std::vector<upo_RRT::Node> solve(float secs);

			//void setRobotPosition();


			void setMaxRange(float range) {
				steering_->setMaxRange(range);
			}


		//private:
		
			//float 				maxRange_; //max distance to insert a new node

	};
}
#endif
