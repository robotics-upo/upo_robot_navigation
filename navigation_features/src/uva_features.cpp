
#include <navigation_features/uva_features.h>

using namespace std;
using namespace uva_cost_functions;


UvaFeatures::UvaFeatures(tf::TransformListener* tf) {

	listener_ = tf;

	people_paint_area_ = 25;
	
	ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap> ("/static_map");
	while (! ros::service::waitForService("/static_map",1)){
		ROS_INFO("Waiting for map service");
	}
	ROS_INFO("Connected");

	nav_msgs::GetMap srv;
	map_client.call(srv);
	ROS_INFO_STREAM(srv.response.map.info);
	map_image_ = cv::Mat(srv.response.map.info.height,srv.response.map.info.width,CV_8UC1,cv::Scalar(0));
	map_metadata_ =srv.response.map.info;
	uint8_t *myData = map_image_.data;
	for (int i=0;i<srv.response.map.data.size();i++){
		if (srv.response.map.data.at(i)==100  || srv.response.map.data.at(i)==-1 ){
		}
		else {
			map_image_.data[i] = 255;
		}
	}

	distancetrans_Mutex_.lock();
	distance_transform_ = cv::Mat(map_image_.rows,map_image_.cols,CV_32FC1);
	cv::distanceTransform(map_image_,distance_transform_,CV_DIST_L1,3);
	distancetrans_Mutex_.unlock();
 
	sub_pc_ = nh_.subscribe("/scan360/point_cloud", 1, &UvaFeatures::pcCallback, this);

	
	//timer_ = nh_.createTimer(ros::Duration(2),&UvaFeatures::timerCallback, this, true);

}



vector<int> NavigationHelper::worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata){
	vector<int> pixels;
	float x_map = world_point->x - map_metadata->origin.position.x;
	float y_map = world_point->y - map_metadata->origin.position.y;
	pixels.push_back((int)floor(x_map/map_metadata->resolution ));
	pixels.push_back((int)floor(y_map/map_metadata->resolution));
	return pixels;
};

void NavigationHelper::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_in){
	
	try{ 
		if(!listener_.waitForTransform(pc_in->header.frame_id,"/map",
			pc_in->header.stamp,ros::Duration(0.5))){
		}
		else{
			laser_callbackMutex_.lock();
			pcl_ros::transformPointCloud("/map",*pc_in,laser_cloud_,listener_);
			laser_callbackMutex_.unlock();
		//updateDT();
		}
	} catch (tf::TransformException ex){
		ROS_ERROR("FEATURE LIBRARY. pcCallback. TransformException: %s",ex.what());
	}
}
