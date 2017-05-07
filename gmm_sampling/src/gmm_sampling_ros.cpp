#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>
#include <time.h>
#include <numeric>      // std::accumulate


#include <gmm_sampling/gmm_sampling.h>

//ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
//Markers
#include <visualization_msgs/Marker.h>
//Service messages to be called
#include <gmm_sampling/GetApproachGMMSamples.h>
#include <gmm_sampling/GetApproachGMMProb.h>
#include <gmm_sampling/GetApproachGMMProbs.h>


//Global variables
gmm_samp::GMMSampling* gmm_;

//bool isActived_;

//tf::TransformListener* tf_listener_;

//Services
ros::ServiceServer samples_srv_;
ros::ServiceServer prob_srv_;
ros::ServiceServer probs_srv_;





//Visualize a costmap of the GMM
/*void publish_gmm_markers()
{

}*/


//Services
bool getSamples(gmm_sampling::GetApproachGMMSamples::Request  &req, gmm_sampling::GetApproachGMMSamples::Response &res)
{
	std::vector< std::pair<double,double> > samples = gmm_->getApproachSamples(req.person_orientation, req.num_samples);
	for(unsigned int i=0; i<samples.size(); i++)
	{
		res.distances.push_back(samples[i].first);
		res.orientations.push_back(samples[i].second);
	}

	//publish_gmm_markers();

	return true;
}


bool getProb(gmm_sampling::GetApproachGMMProb::Request  &req, gmm_sampling::GetApproachGMMProb::Response &res)
{
	//x and y coordinates must be in the target person frame
	float d = sqrt(req.x*req.x + req.y*req.y);
	float o = atan2(req.y,req.x);
	std::pair<double,double> p;
	p.first = d;
	p.second = o;
	unsigned int t = gmm_->classifyApproachTask(req.person_orientation);
	res.prob = gmm_->getPDFValue(t, p);
	return true;
}


bool getProbs(gmm_sampling::GetApproachGMMProbs::Request  &req, gmm_sampling::GetApproachGMMProbs::Response &res)
{
	float d, o;
	unsigned int t = gmm_->classifyApproachTask(req.person_orientation);
	std::vector< std::pair<double,double> > pos;
	//x and y coordinates must be in the target person frame
	for(unsigned int i=0; i<req.x.size(); i++)
	{
		d = sqrt(req.x[i]*req.x[i] + req.y[i]*req.y[i]);
		o = atan2(req.y[i],req.x[i]);
		std::pair<double,double> p;
		p.first = d;
		p.second = o;
		pos.push_back(p);
	}
	res.probs = gmm_->getPDFValues(t, pos);
	return true;
}






int main(int argc, char** argv){

	ros::init(argc, argv, "-GMMSampling-");
  	
	//tf_listener_ = new tf::TransformListener(ros::Duration(10));

	ros::NodeHandle n("~");

	std::string tasks_dir = "";
	n.param("gmm_tasks_directory", tasks_dir, std::string(" "));

	bool publish_markers = true;
	n.param("publish_gmm_markers", publish_markers, true);

	//Advertise services
	ros::NodeHandle nh("gmm_sampling");
	samples_srv_ = nh.advertiseService("GetApproachGMMSamples", &getSamples);
	prob_srv_ = nh.advertiseService("GetApproachGMMProb", &getProb);
	probs_srv_ = nh.advertiseService("GetApproachGMMProbs", &getProbs);
	
	gmm_ = new gmm_samp::GMMSampling(tasks_dir);

	ros::spin();

}
