#include <upo_rrt_planners/Action.h>

//Constructor
upo_RRT::Action::Action()
{
	vx_ = 0.0;
	vy_ = 0.0;
	vth_ = 0.0;
	steps_ = 1;
}

//Constructor
upo_RRT::Action::Action(float vx, float vy, float vth, unsigned int steps)
{
	vx_ = vx;
	vy_ = vy;
	vth_ = vth;
	steps_ = steps;
}

// Destructor
upo_RRT::Action::~Action()
{
}


void upo_RRT::Action::getAction(float &vx, float &vy, float &vth, unsigned int &steps)
{
	vx = vx_;
	vy = vy_;
	vth = vth_;
	steps = steps_;
}


float upo_RRT::Action::getVx() {return vx_;}
float upo_RRT::Action::getVy() {return vy_;}
float upo_RRT::Action::getVth() {return vth_;}
unsigned int upo_RRT::Action::getSteps() {return steps_;}
