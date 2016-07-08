#include <upo_rrt_planners/State.h>

//Constructor
upo_RRT::State::State()
{
	x_ = 0.0;
	y_ = 0.0;
	yaw_ = 0.0;
	lin_vel_ = 0.0;
	ang_vel_ = 0.0;
}

//Constructor
upo_RRT::State::State(float x, float y, float yaw, float lv, float av)
{
	x_ = x;
	y_ = y;
	yaw_ = yaw;
	lin_vel_ = lv;
	ang_vel_ = av;
}

// Destructor
upo_RRT::State::~State()
{
}


void upo_RRT::State::getState(float &x, float &y, float &yaw, float &lv, float &av)
{
	x = x_;
	y = y_;
	yaw = yaw_;
	lv = lin_vel_;
	av = ang_vel_;
}

float upo_RRT::State::getX() {return x_;}
float upo_RRT::State::getY() {return y_;}
float upo_RRT::State::getYaw() {return yaw_;}
float upo_RRT::State::getLinVel() {return lin_vel_;}
float upo_RRT::State::getAngVel() {return ang_vel_;}

void upo_RRT::State::setX(float x){x_ = x;}
void upo_RRT::State::setY(float y){y_ = y;}
void upo_RRT::State::setYaw(float yaw){yaw_ = yaw;}
void upo_RRT::State::setLv(float lv){lin_vel_ = lv;}
void upo_RRT::State::setAv(float av){ang_vel_ = av;}
