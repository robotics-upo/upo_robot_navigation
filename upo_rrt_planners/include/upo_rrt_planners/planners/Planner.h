#ifndef UPO_RRT_PLANNER_
#define UPO_RRT_PLANNER_

#include <upo_rrt_planners/StateSpace.h>
#include <upo_rrt_planners/State.h>
#include <upo_rrt_planners/Node.h>
#include <upo_rrt_planners/steering/Steering.h>
#include <upo_rrt_planners/StateChecker.h>
#include <upo_rrt_planners/NearestNeighborsFLANN.h>
#include <upo_rrt_planners/NearestNeighbors.h>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>


namespace upo_RRT
{
	class Planner
	{

		public:
		
			Planner();
			
			
			virtual ~Planner(); 
			
			
			virtual std::vector<upo_RRT::Node> solve(float secs) { 
				std::vector<upo_RRT::Node> empty;
				return empty;
			} 
			
			virtual upo_RRT::State* steer(State* fromState, State* toState) { return NULL; }
			
			virtual upo_RRT::Node* steer(Node* fromState, Node* toState) { return NULL; }
			
			
			void setup(StateChecker* sch, unsigned int nn_params, unsigned int dim, float sx, float sy, float xyres = 0.1, float yawres = 0.02, 
				float min_lv = 0.0, float max_lv = 0.5, float lvres = 0.05, float max_av = 0.5, 
				float avres = 0.1);
			
			
			
			template<class T>
			T* as()
			{
				//BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
				return static_cast<T*>(this);
			}

			template<class T>
			const T* as() const
			{
				//BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>)); 
				return static_cast<const T*>(this);
			}
			
			
			
			float distanceFunction(Node* s1, Node* s2)
			{
				return space_->distance(s1->getState(), s2->getState());
			}
			

			void setGoalBias(float b) {
				goalBias_ = b;
			}

			void setGoalTolerance(float xy_tol, float th_tol) {
				space_->setGoalTolerance(xy_tol, th_tol);
			}
			
			bool setStartAndGoal(float start_x, float start_y, float start_h, float goal_x, float goal_y, float goal_h);
			
			
			bool setStoreTree(bool s){
				storeTree_ = s;
			}
			
			struct statistics
			{
				float planning_time;
				float first_sol_time;
				unsigned int total_samples;
				unsigned int valid_samples;
				unsigned int goal_samples;
				unsigned int tree_nodes;
				unsigned int path_nodes;
			};

			statistics getStatistics(){
				return stats_;
			}
			
			
			std::vector<upo_RRT::State> getTree() {
				return tree_;
			}
			
			float getCost() {
				return path_cost_;
			}


			

			void copyState(State *destination, const State *source) const
	  		{
		   		memcpy(destination, source, sizeof(*source));
			}
			void copyAction(Action *destination, const Action *source) const
	  		{
		   		memcpy(destination, source, sizeof(*source));
			}
			void copyNode(Node *destination, const Node *source) const
	  		{
		   		memcpy(destination, source, sizeof(*source));
			}
			
			void freeTreeMemory();
			
			void setNearestNeighbors(unsigned int nn_params)
			{
				nn_.reset(new NearestNeighborsFLANN<upo_RRT::Node*>(nn_params));
			}
			
			void storeTree(std::vector<Node*> list);
			
			void setBiasingPath(std::vector<upo_RRT::State>* path);
			
			void setFullBiasing(bool b) {
				fullBiasing_ = b;
			}
			
			void setPathBias(float f) {
				pathBias_ = f;
			}
			
			void setPathBias_stddev(float f) {
				pathBias_stddev_ = f;
			}
			
			void setInitialActionState(float vx, float vy, float vth, int steps) {
				init_action_state_ = new Action(vx, vy, vth, steps);
			}
			
			
		protected:

			float 				goalBias_;

			StateSpace* 		space_;

			boost::shared_ptr< NearestNeighborsFLANN<upo_RRT::Node*> > nn_;
			
			Steering*			steering_;

			State* 				start_;
			State*				goal_;
			
			Action*				init_action_state_;
			
			float 				path_cost_;
			
			statistics          stats_;
			
			bool				storeTree_;
			std::vector<upo_RRT::State> tree_;
			
			std::vector<upo_RRT::State> first_path_;
			bool 				fullBiasing_;
			float				pathBias_;
			float 				pathBias_stddev_;

	};
	
}
#endif


