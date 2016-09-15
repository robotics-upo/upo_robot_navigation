
#ifndef YIELD_H
#define YIELD_H

//#include <ros/ros.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <std_msgs/Bool.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


typedef enum {NORMAL, NARROW, SUPERNARROW} zone;


struct point {
	double x_;
	double y_;
	double theta_;
};

class Yield {

	public:
	
		Yield();
		Yield(std::string* yamlFile, std::string* pointsFile);
		
		void getClosestPoint(double curr_x, double curr_y, double& goal_x, double& goal_y, double& goal_theta);

		zone getType(double x, double y, bool &correct);
		
		bool loadYieldPoints(std::string* pointsFile);


	private:

		cv::Mat map;

		double origin[3];
		double res;
		int mapWidth;
		int mapHeight;
	
		bool has_map;
		
		std::vector<point> yield_points;

		//bool inzone;
		//ros::Subscriber amcl_sub;
		//geometry_msgs::PoseWithCovariance pose_robot;

};


#endif
