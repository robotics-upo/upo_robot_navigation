#ifndef UPO_RRT_STATE_CHECKER_
#define UPO_RRT_STATE_CHECKER_

#include <upo_rrt_planners/State.h>
#include <math.h>

namespace upo_RRT
{
	class StateChecker
	{

		public:
			StateChecker(){};
			virtual ~StateChecker(){}
		
			virtual bool isValid(State* s) const = 0;
			
			virtual float distance(State* s1, State* s2) const {
				float dx = s1->getX() - s2->getX();
				float dy = s1->getY() - s2->getY();
				return sqrt(dx*dx + dy*dy);
			}
			
			virtual float getCost(State* s) = 0;
			
			virtual void preplanning_computations() = 0;

	};
}
#endif
