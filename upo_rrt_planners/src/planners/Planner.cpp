#include <upo_rrt_planners/planners/Planner.h>


upo_RRT::Planner::Planner() {
	goalBias_ = 0.05;
	storeTree_ = false;
	space_ = NULL;
	steering_ = new Steering();
	start_ = NULL;
	goal_ = NULL;
	path_cost_ = 0.0;
	fullBiasing_ = false;
	pathBias_ = 0.0;
	pathBias_stddev_ = 0.0;
}

upo_RRT::Planner::~Planner() {
	freeTreeMemory();
	tree_.clear();
	nn_.reset();
	delete steering_;
	delete space_;
	delete start_;
	delete goal_;
}


void upo_RRT::Planner::freeTreeMemory() {
	if (nn_)
	{
		std::vector<Node*> nodes;
		nn_->list(nodes);
		for (unsigned int i = 0; i <nodes.size() ; ++i)
		{
			if(nodes[i]) {
				delete nodes[i];
			}
		}
		nodes.clear();
		nn_->clear();
		//if(goal_)
			//delete goal_;
		//if(start_)
			//delete start_;
	}
}


void upo_RRT::Planner::setup(StateChecker* sch, unsigned int nn_params, unsigned int dim, float sx, float sy, float xyres, float yawres, 
				float min_lv, float max_lv, float lvres, float max_av, float avres,
				float steer_kp, float steer_kv, float steer_ka, float steer_ko) 
{
	
	if(!nn_) {
		setNearestNeighbors(nn_params);
	}
    nn_->setDistanceFunction(boost::bind(&Planner::distanceFunction, this, _1, _2));
	
	//dimensions, size_x, size_y, xyres, yawres, min_lv, max_lv, lvres, max_av, avres
	space_ = new StateSpace(sch, dim, sx, sy, xyres, 
		yawres, min_lv, max_lv, lvres, max_av, avres);
		
	if(steering_ == NULL)
		steering_ = new Steering(space_);
	else
		steering_->setStateSpace(space_);
		
	steering_->setSteeringParams(steer_kp, steer_kv, steer_ka, steer_ko);			
}


bool upo_RRT::Planner::setStartAndGoal(float start_x, float start_y, float start_h, float goal_x, float goal_y, float goal_h)
{
	start_ = new State(start_x, start_y, start_h);
	goal_ = new State(goal_x, goal_y, goal_h);
	//printf("START x:%.1f, y:%.1f, h:%.1f  GOAL x:%.2f, y:%.2f, h:%.2f\n", start_x, start_y, start_h, goal_x, goal_y, goal_h);
	if(!space_->setStartAndGoal(start_, goal_)) {
		printf("UPO_RRT. Goal state is not valid!!!\n");
		return false;
	}
	return true;
}


void upo_RRT::Planner::setBiasingPath(std::vector<upo_RRT::State>* path)
{
	//printf("Planner. SetBiasingPath. path size: %u\n", (unsigned int)path->size());
	first_path_ = *path;
}



void upo_RRT::Planner::storeTree(std::vector<Node*> list) {
	
	if(tree_.size() > 0)
		tree_.clear();
		
	for(unsigned int j = 0; j<list.size(); j++) {
		Node* parent = list[j]->getParent();
		if(parent != NULL) {
			//Store parent
			State state_p = *(parent->getState());
			//copyState(&state_p, parent->getState());
			tree_.push_back(state_p);
			//Store child
			State state_c = *(list[j]->getState());
			//copyState(&state_c, list[j]->getState());
			tree_.push_back(state_c);
		}
	}
}


