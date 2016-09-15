#include <upo_rrt_planners/Node.h>


upo_RRT::Node::Node() : parent_(NULL)
{
	//state_ = new State();
	//Action* a = new Action();
	//control_.push_back(a);
	cost_ = 0.0;
	incCost_ = 0.0;	
	accCost_ = 0.0;
	exist_intermediate_states_ = false;
}


upo_RRT::Node::Node(State s) : parent_(NULL)
{
	state_ = s;
	//Action* a = new Action();
	//control_.push_back(a);
	cost_ = 0.0;
	incCost_ = 0.0;	
	accCost_ = 0.0;
	exist_intermediate_states_ = false;
}

upo_RRT::Node::Node(State s, Action a) : parent_(NULL)
{
	state_ = s;
	state_.setLv(a.getVx());
	state_.setAv(a.getVth());
	control_.clear();
	control_.push_back(a);
	cost_ = 0.0;
	incCost_ = 0.0;	
	accCost_ = 0.0;
	exist_intermediate_states_ = false;
}


upo_RRT::Node::~Node()
{
	//parent_ = NULL;
	
	//intermediate_states_.clear();
	//control_.clear();
}


/*bool upo_RRT::Node::deleteStateAndControl() {
	
	if(state_)
		delete state_;
		
	for(unsigned int i=0; i<control_.size(); i++)
		delete control_[i];
	
	//for(unsigned int j=0; j<intermediate_states_.size(); j++)
	//	delete intermediate_states_[j];
	
	intermediate_states_.clear();
	control_.clear();
	exist_intermediate_states_ = false;
}*/


upo_RRT::State* upo_RRT::Node::getState() {
	return &state_;
}

std::vector<upo_RRT::Action>* upo_RRT::Node::getAction() {
	return &control_;
}

upo_RRT::Node* upo_RRT::Node::getParent() {
	return parent_;
}

/*std::vector<upo_RRT::Node*> upo_RRT::Node::getChildren() {
	return children_;
}*/

std::vector<upo_RRT::State>* upo_RRT::Node::getIntermediateStates() {
	if(!exist_intermediate_states_)
		printf("Node. No intermediate states exist!!! Prone to error!\n");
	return &intermediate_states_;
}

float upo_RRT::Node::getCost() {
	return cost_;
}

float upo_RRT::Node::getIncCost() {
	return incCost_;
}

float upo_RRT::Node::getAccCost() {
	return accCost_;
}


void upo_RRT::Node::setParent(Node* p)
{
	parent_ = p;
}

/*void upo_RRT::Node::setChildren(Node* s)
{
	children_.push_back(s);
}*/

void upo_RRT::Node::setState(State s)
{
	state_ = s;
}

void upo_RRT::Node::addAction(Action a) {
	control_.push_back(a);
}

void upo_RRT::Node::setAction(std::vector<Action> a) {
	control_.clear();
	control_ = a;
}

void upo_RRT::Node::addIntermediateState(State s) {
	exist_intermediate_states_ = true;
	intermediate_states_.push_back(s);
}

void upo_RRT::Node::setIntermediateStates(std::vector<State> s) {
	intermediate_states_.clear();
	intermediate_states_ = s;
	exist_intermediate_states_ = true;
}

bool upo_RRT::Node::hasIntermediateStates() {
	return exist_intermediate_states_;
}

void upo_RRT::Node::setCost(float c) {
	cost_ = c;
}
void upo_RRT::Node::setIncCost(float c) {
	incCost_ = c;
}
void upo_RRT::Node::setAccCost(float c) {
	accCost_ = c;
}



