#include <upo_rrt_planners/planners/simple/SimpleRRTstar.h>

upo_RRT::SimpleRRTstar::SimpleRRTstar() : Planner() {
	
	maxRange_ = 0.5;
	useKnearest_ = true;
	k_rrt_ = 0.0;
	r_rrt_ = 0.0;
	rewire_factor_ = 1.1;
			
	useFirstPathBiasing_ = false;
	pathBias_ = 0.0;
	pathBias_stddev_ = 0.0;
	

}

upo_RRT::SimpleRRTstar::~SimpleRRTstar() {

}


upo_RRT::State* upo_RRT::SimpleRRTstar::steer(State* fromState, State* toState, std::vector<State>& istates)
{
	return  steering_->simpleSteer(fromState, toState, istates);
}


bool upo_RRT::SimpleRRTstar::collisionFree(State* fromState, State* toState, std::vector<State>& istates)
{
	return  steering_->simpleCollisionFree(fromState, toState, istates);
}




std::vector<upo_RRT::Node> upo_RRT::SimpleRRTstar::solve(float secs)
{
	/******************************************************************************************
	V<-{Xinit}, E<-0;
	for int i=0,...,n do
		Xrand <- SampleFree;
		Xnearest <- Nearest(G=(V,E), Xrand);
		{(Xnew, Unew, Tnew)} <- Steer(Xnearest, Xrand);
		if(ObstacleFree(Xnearest, Xnew)) then
			Xs_near <- Near(G=(V,E), Xnew, min{gamma(log(Card(V)/Card(V))^(1/d), eta});
			V <- V U {Xnew}
			Xmin <- Xnearest; 
			Cmin <- Cost(Xnearest) + C(Line(Xnearest, Xnew));
			for each Xnear in Xs_near do   	//Connect along a minimum-cost path
				if CollisionFree(Xnear, Xnew) && Cost(Xnear)+C(Line(Xnear, Xnew)) < Cmin then
					Xmin <- Xnear;
					Cmin <- Cost(Xnear) + C(Line(Xnear, Xnear));
			E <- E U {(Xmin, Xnew)};
			for each Xnear in Xs_near do 	//Rewire the tree
				if CollisionFree(Xnew, Xnear) && Cost(Xnew) + C(Line(Xnew, Xnear)) < Cost(Xnear) then
					Xparent <- Parent(Xnear);
				E <- (E\{(Xparent, Xnear)}) U {(Xnew, Xnear)};
	return G=(V,E);
	***********************************************************************************************/


	//Clear datastructure and initilize it
	//nn_->clear();
	Node* ini = new Node(*start_);
	float singleCost = space_->getCost(start_);
	ini->setCost(singleCost);
	ini->setIncCost(singleCost);
	ini->setAccCost(singleCost);
	nn_->add(ini);
	
	calculateParamsNearest();

	tree_.clear();
	first_path_.clear();
	
	//Statistics
	unsigned int total_samples = 0;
	unsigned int valid_samples = 0;
	unsigned int goal_samples = 0;
	unsigned int tree_nodes = 1;
	unsigned int path_nodes = 0;
 	float time = 0.0;
 	float first_sol_time = 0.0;

	bool solved = false;
	bool end = false;
	bool first_sol = true;
	Node* solution = NULL;
	Node* approxSolution = NULL;
	float approxDist = std::numeric_limits<float>::max();
	
	//Goal node
	Node* goalNode = new Node(*goal_);
	goalNode->setCost(space_->getCost(goal_));
	
	
	//Node* randNode = NULL;
	
	unsigned int cont_null = 0;
	
	double t1, t2;
	struct timeval stop, start, first_stop;
	gettimeofday(&start, NULL);
	t1=start.tv_sec+(start.tv_usec/1000000.0);
	while(!end)
	{
		//State* randState = NULL;
		State randState;
		
		
		//sample goal according to the bias parameter
		if(space_->sampleUniform() < goalBias_)
		{
			//randNode = goalNode;
			randState = *goalNode->getState();
			goal_samples++;
			valid_samples++;
			total_samples++;
		} else {
			//Sample a random valid state
			unsigned int cont = 0;
			do {
				
				//if(randState)
				//	delete randState;
				
				if(fullBiasing_ && !first_path_.empty())
				{
					randState = *space_->samplePathBiasing(&first_path_, pathBias_stddev_);
					
				} else if(useFirstPathBiasing_ && !first_sol && space_->sampleUniform() < pathBias_) { //First RRT Path biasing
					
					randState = *space_->samplePathBiasing(&first_path_, pathBias_stddev_);
					
				
				} else { // Regular state sampling
					randState = *space_->sampleState();
					
				}
				cont++;
				if(cont>1)
					total_samples++;
				else {
					valid_samples++;
					total_samples++;
				}
				
			} while(!space_->isStateValid(&randState));
		}
		
		//if(randNode)
		//	delete randNode;
			
		//randNode = new Node(*randState);
		Node randNode(randState);
		//delete randState;
		
		//Find nearest node in the tree
		Node* nearNode = nn_->nearest(&randNode);

		//Steer from nearState to randState and reach the new state
		std::vector<State> inter_states;
		State* newState = steer(nearNode->getState(), randNode.getState(), inter_states);
		//delete randNode;
		
		if(newState != NULL)
		{
			Node* newNode = new Node(*newState);
			newNode->setIntermediateStates(inter_states);
			newNode->setCost(space_->getCost(newNode->getState())); //newState
			
			//Use the neighbors of the new node to find the best parent
			std::vector<upo_RRT::Node*> nbrs;
			getNearestNeighbors(newNode, nbrs);
			//printf("Neighbors obtained: %u\n", (unsigned int)nbrs.size());
			
			Node* node_min = nearNode;
			float inc_cost = steering_->motionCost(nearNode, newNode);
			float cost_min = nearNode->getAccCost() + inc_cost;
			//Check the nodes costs to chose the parent with 
			// a lower cost connection
			
			std::vector<upo_RRT::Node*> nbrs_aux;
			nbrs_aux.push_back(nearNode);
			
			std::vector<State> intermediates;
			intermediates = inter_states;
			for(unsigned int i=0; i<nbrs.size(); i++)
			{
				if(nbrs[i]!=nearNode && collisionFree(nbrs[i]->getState(), newNode->getState(), inter_states))  //newState
				{
					nbrs_aux.push_back(nbrs[i]);
					
					float total_cost = nbrs[i]->getAccCost() + steering_->motionCost(nbrs[i], newNode);
					if(total_cost < cost_min) {
						node_min = nbrs[i];
						cost_min = total_cost;
						intermediates = inter_states;
					}	
				}	
			}
			
			newNode->setParent(node_min);
			newNode->setIncCost(steering_->motionCost(node_min, newNode));
			newNode->setAccCost(cost_min);
			newNode->setIntermediateStates(intermediates);
			
			//Add the new node to the tree
			nn_->add(newNode);
			tree_nodes++;
			
			
			//Rewire the tree
			for(unsigned int i=0; i<nbrs_aux.size(); i++)
			{
				if(nbrs_aux[i] != node_min /*&& collisionFree(newState, nbrs_aux[i]->getState())*/)  //Only valid for non-kinematics RRT*
				{
					float total_cost = newNode->getAccCost() + steering_->motionCost(newNode, nbrs_aux[i]);
					if(total_cost < nbrs_aux[i]->getAccCost()) {
						//printf("¡¡¡¡performing a rewiring!!!\n");
						//newNode->setChildren(nbrs[i]);
						nbrs_aux[i]->setParent(newNode);
						nbrs_aux[i]->setIncCost(steering_->motionCost(newNode, nbrs_aux[i]));
						nbrs_aux[i]->setAccCost(total_cost);
					}	
				}	
			}
			
			float dist = 0.0;
			solved = space_->isSimpleGoalToleranceSatisfied(newNode->getState(), dist); //newState

			if (solved)
			{
				approxDist = dist;
				solution = newNode;
				if(first_sol) {
					gettimeofday(&first_stop, NULL);
					double t3 = first_stop.tv_sec+(first_stop.tv_usec/1000000.0);
					first_sol_time = (t3 - t1);
					
					//Store the first solution to draw samples from it.
					if(useFirstPathBiasing_) {
						Node* current = solution;
						while (current != NULL)
						{
							State state = *current->getState();
							//copyState(&state, current->getState());
							first_path_.push_back(state);
							current = current->getParent();
						}
							
					}
					first_sol = false;
				}
			}
			else if (dist < approxDist)
			{
				approxDist = dist;
				approxSolution = newNode;
			}
		} else {
			cont_null++;
		}
		
		//delete newState;
		
		gettimeofday(&stop, NULL);
		t2=stop.tv_sec+(stop.tv_usec/1000000.0);
		time = t2 - t1;
		//printf("Time: %.3f, fin: %.3f\n", time, secs);
		if(time >= secs) {
			end = true;
		}
	}


	if(solution == NULL)
	{
		printf("\nUPO_RRT. Approximate solution found. dist to goal: %.3f\n", approxDist);
		solution = approxSolution;
	
	} else {
		printf("\nUPO_RRT. Solution found in %.6f secs\n", time);
	}
	
	//printf("Number of null states: %u\n", cont_null);
	
	if(storeTree_) {
		std::vector<Node*> nodes;
		nn_->list(nodes);
		storeTree(nodes);
	}
	

	//Construct the solution path
	std::vector<upo_RRT::Node> path;
	solution->getState()->setYaw(goal_->getYaw());
	Node* current = solution;
	path_cost_ = current->getAccCost();
	if(path_cost_ == 0)
		printf("\n----------SimpleRRTStar. path_cost = 0!!!!-------------\n");
	
	while (current != NULL)
	{
		Node node = *current;
		//copyNode(&node, current);
		path.push_back(node);
 		current = current->getParent();
		path_nodes++;
	}	

	stats_.planning_time = time;
	stats_.first_sol_time = first_sol_time;
	stats_.total_samples = total_samples;
	stats_.valid_samples = valid_samples;
	stats_.goal_samples = goal_samples;
	stats_.tree_nodes = tree_nodes;
	stats_.path_nodes = path_nodes;
	

	delete current;
	//delete ini;
	delete goalNode;
	//delete solution;
	//delete approxSolution;
	
	freeTreeMemory();
	first_path_.clear();
	
	return path;

}


void upo_RRT::SimpleRRTstar::getNearestNeighbors(Node* node, std::vector<Node*> &nbrs) 
{
	
	double size = static_cast<double>(nn_->size() + 1u);
	if (useKnearest_)
	{
		// k-nearest RRT*
		unsigned int k = std::ceil(k_rrt_ * log(size));
		//printf("k: %u\n", k);
		nn_->nearestK(node, k, nbrs);
		
	} else {
		// Nearest in radius RRT*
		double r = std::min((double)maxRange_, r_rrt_ * std::pow(log(size) / size, 1 / static_cast<double>(space_->getDimensions())));
		//printf("Neighbors in Radius: %.3f\n", r);
		nn_->nearestR(node, r, nbrs);
	}
}

void upo_RRT::SimpleRRTstar::calculateParamsNearest() {

	double dim = (double)space_->getDimensions();

	//K-nearest
	if(useKnearest_)  {
		//k_rrt_ = 1.1 * (boost::math::constants::e<double>() + (boost::math::constants::e<double>() / dim));
		k_rrt_ = rewire_factor_ * (exp(1) + (exp(1) / dim));
	}else {
		//Radius
		float free_volume = space_->getSpaceMeasure() / 2.0;
		float unitBall = space_->getUnitBallMeasure();
		r_rrt_ = rewire_factor_ * 2 *  pow((1+1/dim)*(free_volume / unitBall), 1/dim);
	}
}


/*float upo_RRT::SimpleRRTstar::motionCost(Node* n1, Node* n2) {
	
	float dist = sqrt(distanceFunction(n1, n2));
	//Normalize
	//float max_d = sqrt((space_->getSizeX()*2*space_->getSizeX()*2)+(space_->getSizeY()*2*space_->getSizeY()*2));
	//dist = dist / max_d;
	
	//printf("Dist: %.3f, sqrt(dist): %.3f, avg cost: %.3f\n", dist, sqrt(dist), (n1->getCost() + n2->getCost())/2.0 );
	return ((n1->getCost() + n2->getCost()) / 2.0)  *  dist;
	//return ((n1->getCost() + n2->getCost()) / 2.0) + dist;
	
}*/




