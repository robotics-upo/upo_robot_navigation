#include <upo_rrt_planners/StateSpace.h>
//#include <ros/ros.h>

#define PI 3.14159

upo_RRT::StateSpace::StateSpace() {

	dimensions_ = 3;
	weights_.push_back(1.0); //x
	weights_.push_back(1.0); //y
	weights_.push_back(0.3); //th

	size_x_ = 4.0; //[-4,4]
	size_y_ = 4.0;
	xy_resolution_ = 0.1; 
	yaw_resolution_ = 0.02; 

	min_lin_vel_ = 0.1;
	max_lin_vel_ = 0.5;
	lin_vel_res_ = 0.1;

	max_ang_vel_ = 0.5;
	ang_vel_res_ = 0.25;
	
	goal_xy_tolerance_ = 0.1;
	goal_th_tolerance_ = 0.15;
	
	space_volume_ = (size_x_ - (-size_x_)) * (size_y_ - (-size_y_));
	unit_ball_measure_ = calculeUnitBallMeasure(dimensions_, 1.0);
}


upo_RRT::StateSpace::StateSpace(StateChecker* stateChecker, unsigned int dim, float sx, float sy, float xyres, 
	float yawres, float min_lv, float max_lv, float lvres, 
	float max_av, float avres)
	: dimensions_(dim), size_x_(sx), size_y_(sy), xy_resolution_(xyres), 
	yaw_resolution_(yawres), min_lin_vel_(min_lv), max_lin_vel_(max_lv),
	lin_vel_res_(lvres), max_ang_vel_(max_av), ang_vel_res_(avres) 
{

	stateChecker_ = stateChecker;

 	switch (dimensions_)
	{
		case 2: 
			weights_.push_back(1.0); //x
			weights_.push_back(1.0); //y
			break;
		case 3: 
			weights_.push_back(1.0); //x
			weights_.push_back(1.0); //y
			weights_.push_back(0.3); //th
			break;
		case 5:
			weights_.push_back(1.0); //x
			weights_.push_back(1.0); //y
			weights_.push_back(0.4); //th
			weights_.push_back(0.1); //lv
			weights_.push_back(0.1); //av
			break;
		default:
			printf("Number of dimensions not valid");
	}
	
	space_volume_ = (size_x_ - (-size_x_)) * (size_y_ - (-size_y_));
	
	goal_xy_tolerance_ = 0.1;
	goal_th_tolerance_ = 0.15;
	
	unit_ball_measure_ = calculeUnitBallMeasure(dimensions_, 1.0);
}
		
upo_RRT::StateSpace::~StateSpace(){
	//if(stateChecker_)
	//	delete stateChecker_;
	if(start_)
		delete start_;
	if(goal_)
		delete goal_;
}

float upo_RRT::StateSpace::calculeUnitBallMeasure(unsigned int d, double r) {
 return std::pow(std::sqrt(boost::math::constants::pi<double>()) * r, static_cast<double>(d)) / boost::math::tgamma(static_cast<double>(d)/2.0 + 1.0);
}

float upo_RRT::StateSpace::getUnitBallMeasure() {
	return unit_ball_measure_;
}

upo_RRT::State* upo_RRT::StateSpace::sampleState()
{
	float x=0.0, y=0.0, yaw=0.0, lv =0.0, av=0.0;
	x = random_.uniformReal(-size_x_, size_x_);
	y = random_.uniformReal(-size_y_, size_y_);
	if(dimensions_ == 3) {
		yaw = random_.uniformReal(-PI, PI);
	} 
	if(dimensions_ > 3) {
		lv = random_.uniformReal(min_lin_vel_, max_lin_vel_);
		av = random_.uniformReal(-max_ang_vel_, max_ang_vel_);
	}
	State* sample = new upo_RRT::State(x, y, yaw, lv, av);
	return sample;
}


upo_RRT::State* upo_RRT::StateSpace::sampleStateFree()
{
	float x, y;
	State* sample;
	do
	{
		x = random_.uniformReal(-size_x_, size_x_);
		y = random_.uniformReal(-size_y_, size_y_);
		sample = new upo_RRT::State(x, y);
	} while(!isStateValid(sample));
	if(dimensions_ == 2)
		return sample;

	if(dimensions_ == 3) {
		float yaw = random_.uniformReal(-PI, PI);
		sample->setYaw(yaw);
		return sample;
	}
	float yaw = random_.uniformReal(-PI, PI);
	sample->setYaw(yaw);
	float lv = random_.uniformReal(min_lin_vel_, max_lin_vel_);
	float av = random_.uniformReal(-max_ang_vel_, max_ang_vel_);
	sample->setLv(lv);
	sample->setAv(av);
	return sample;
}

float upo_RRT::StateSpace::sampleUniform() {
	return (float)random_.uniform01();
}

upo_RRT::State* upo_RRT::StateSpace::sampleStateNear(State* st)
{
	float low_x = st->getX()-1.0; 
	if(low_x < -size_x_)
		low_x = -size_x_;
	float high_x = st->getX()+1.0;
	if(high_x > size_x_)
		high_x = size_x_;
	float x = random_.uniformReal(low_x, high_x);

	float low_y = st->getY()-1.0; 
	if(low_y < -size_y_)
		low_y = -size_y_;
	float high_y = st->getY()+1.0;
	if(high_y > size_y_)
		high_y = size_y_;
	float y = random_.uniformReal(low_y, high_y);

	float low_h = st->getYaw()-0.35;  //0.35rad = 20º
	if(low_h < -PI)
		low_h = -PI;
	float high_h = st->getYaw()+0.35;
	if(high_h > PI)
		high_h = PI;
	float yaw = random_.uniformReal(low_h, high_h);

	float lv = 0.0;
	float av = 0.0;
	if(dimensions_ > 3)
	{
		float low_lv = st->getLinVel()-0.1; 
		if(low_lv < 0.0)
			low_lv = 0.0;
		float high_lv = st->getLinVel()+0.1;
		if(high_lv > max_lin_vel_)
			high_lv = max_lin_vel_;
		lv = random_.uniformReal(low_lv, high_lv);

		float low_av = st->getAngVel()-0.1; 
		if(low_av < -max_ang_vel_)
			low_av = -max_ang_vel_;
		float high_av = st->getAngVel()+0.1;
		if(high_av > max_ang_vel_)
			high_av = max_ang_vel_;
		av = random_.uniformReal(low_av, high_av);
	}

	State* sample = new State(x, y, yaw, lv, av);
	return sample;
}


upo_RRT::State* upo_RRT::StateSpace::samplePathBiasing(std::vector<State>* path, float stddev) {

	int ind = random_.uniformInt(1, int(path->size()-1));
	float x = path->at(ind).getX();
	float y = path->at(ind).getY();
	float x_sample = random_.gaussian(x, stddev);
	float y_sample = random_.gaussian(y, stddev);
	return new State(x_sample, y_sample);
}


float upo_RRT::StateSpace::distance(State* s1, State* s2) {
	return stateChecker_->distance(s1, s2);
	//return euclideanDistance(s1, s2);

}

float upo_RRT::StateSpace::euclideanDistance(State* s1, State* s2) {
	float dx = s1->getX() - s2->getX();
	float dy = s1->getY() - s2->getY();
	return sqrt(dx*dx + dy*dy);
}

bool upo_RRT::StateSpace::isSimpleGoalToleranceSatisfied(State* st, float &dist)
{
	//float dx = goal->getX() - st->getX();
	//float dy = goal->getY() - st->getY();
	//dist = sqrt(pow(dx,2) + pow(dy,2));
	dist = euclideanDistance(goal_, st);
	if(dist <= goal_xy_tolerance_)
		return true;

	return false;
}

bool upo_RRT::StateSpace::isGoalToleranceSatisfied(State* st, float &dist)
{
	dist = euclideanDistance(goal_, st);
	if(dimensions_ > 2) {
		float phi = st->getYaw() - goal_->getYaw();
		//Normalize phi
		phi = normalizeAngle(phi, -M_PI, M_PI);
		if(dist <= goal_xy_tolerance_ && fabs(phi) <= goal_th_tolerance_)
			return true;	
	}else {
		if(dist <= goal_xy_tolerance_)
			return true;
	}
	return false;
}


float upo_RRT::StateSpace::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}

float upo_RRT::StateSpace::getSpaceMeasure() {
	return space_volume_;
}


bool upo_RRT::StateSpace::setStart(State* s) {

	if(!isStateValid(s))
		return false;
	start_ = s;
	return true;
}

bool upo_RRT::StateSpace::setGoal(State* g) {

	if(!isStateValid(g))
		return false;
	goal_ = g;
	return true;
}

bool upo_RRT::StateSpace::setStartAndGoal(State* s, State* g) {

	if(s == NULL || g == NULL)
		return false;

	//if(!isStateValid(g))
	//	return false;

	start_ = s;
	goal_ = g;
	return true;
}


bool upo_RRT::StateSpace::isStateValid(upo_RRT::State* s) {
	return stateChecker_->isValid(s);
}


float upo_RRT::StateSpace::getCost(upo_RRT::State* s) {
	return stateChecker_->getCost(s);
}




unsigned int upo_RRT::StateSpace::getDimensions() {return dimensions_;}
std::vector<unsigned int> upo_RRT::StateSpace::getWeights() {return weights_;}
float upo_RRT::StateSpace::getSizeX() {return size_x_;}
float upo_RRT::StateSpace::getSizeY() {return size_y_;}
float upo_RRT::StateSpace::getXYresolution() {return xy_resolution_;}
float upo_RRT::StateSpace::getYawResolution() {return yaw_resolution_;}
float upo_RRT::StateSpace::getMinLinVel() {return min_lin_vel_;}
float upo_RRT::StateSpace::getMaxLinVel() {return max_lin_vel_;}
float upo_RRT::StateSpace::getLinVelResolution() {return lin_vel_res_;}
float upo_RRT::StateSpace::getMaxAngVel() {return max_ang_vel_;}
float upo_RRT::StateSpace::getAngVelResolution() {return ang_vel_res_;}
float upo_RRT::StateSpace::getGoalXYTolerance() {return goal_xy_tolerance_;}
float upo_RRT::StateSpace::getGoalTHTolerance() {return goal_th_tolerance_;}
upo_RRT::State* upo_RRT::StateSpace::getStart() {return start_;}
upo_RRT::State* upo_RRT::StateSpace::getGoal() {return goal_;}


void upo_RRT::StateSpace::setDimensions(unsigned int d) {dimensions_=d;}
void upo_RRT::StateSpace::setWeights(std::vector<unsigned int> w) {
	weights_.clear();
	for(unsigned int i=0; i<w.size(); i++)
		weights_.push_back(w[i]);
}
void upo_RRT::StateSpace::setSizeX(float sx) {size_x_=sx;}
void upo_RRT::StateSpace::setSizeY(float sy) {size_y_=sy;}
void upo_RRT::StateSpace::setXYresolution(float res) {xy_resolution_=res;}
void upo_RRT::StateSpace::setYawResolution(float res) {yaw_resolution_=res;}
void upo_RRT::StateSpace::setMinLinVel(float v) {min_lin_vel_=v;}
void upo_RRT::StateSpace::setMaxLinVel(float v) {max_lin_vel_=v;}
void upo_RRT::StateSpace::setLinVelResolution(float res) {lin_vel_res_=res;}
void upo_RRT::StateSpace::setMaxAngVel(float v) {max_ang_vel_=v;}
void upo_RRT::StateSpace::setAngVelResolution(float res) {ang_vel_res_=res;}
void upo_RRT::StateSpace::setGoalTolerance(float xy_tol, float th_tol) {
	goal_xy_tolerance_ = xy_tol;
	goal_th_tolerance_ = th_tol;
}




