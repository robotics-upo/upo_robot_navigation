
#include <navigation_features/uva_features.h>

using namespace std;
using namespace uva_cost_functions;


UvaFeatures::UvaFeatures(tf::TransformListener* tf) {

	listener_ = tf;

}


void UvaFeatures::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
	people_mutex_.lock();
	people_ = p;
	people_mutex_.unlock();
}


void UvaFeatures::setDistanceTransform(cv::Mat dt)
{
	dt_mutex_.lock();
	dt_ = dt.clone();
	dt_mutex_.unlock();
}


float UvaFeatures::getCost(geometry_msgs::PoseStamped* s) {

}


std::vector<float> UvaFeatures::getFeatures(geometry_msgs::PoseStamped* s) {

}
