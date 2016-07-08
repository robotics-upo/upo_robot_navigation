
#include <navigation_features/nav_features.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <ros/console.h>



features::NavFeatures::NavFeatures() {}

features::NavFeatures::NavFeatures(tf::TransformListener* tf, float size_x, float size_y) 
{
	tf_listener_ = tf;
	size_x_ = size_x;
	size_y_ = size_y;
	max_planning_dist_ = sqrt((size_x_*2*size_x_*2)+(size_y_*2*size_y_*2));
	
	use_uva_features_ = true;

}




features::NavFeatures::NavFeatures(tf::TransformListener* tf, const costmap_2d::Costmap2D* loc_costmap, const costmap_2d::Costmap2D* glob_costmap, std::vector<geometry_msgs::Point>* footprint, float insc_radius, float size_x, float size_y)
{
	
	tf_listener_ = tf;
	insc_radius_robot_ = insc_radius;
	size_x_ = size_x;
	size_y_ = size_y;
	max_planning_dist_ = sqrt((size_x_*2*size_x_*2)+(size_y_*2*size_y_*2));
	//max_dist_2_ = (size_x_*2*size_x_*2)+(size_y_*2*size_y_*2);
	
	costmap_local_ = loc_costmap;
	costmap_global_ = glob_costmap;
	myfootprint_ = footprint;
	
	
	//Read the ROS params from the server
	//ros::NodeHandle n("~/upo_navigation/Navigation_features");
	ros::NodeHandle n("~/Navigation_features");
	//ros::NodeHandle n("~");
	
	
	//Load weights "w1, w2, ..."
	w_.clear();
	bool ok = true;
	unsigned int i = 1;
	while(ok)
	{
		char buf[10];
		sprintf(buf, "w%u", i);
		std::string st = std::string(buf);
				
		if(n.hasParam(st.c_str())){
			double wg = 0.0;
			n.getParam(st.c_str(), wg);
			w_.push_back((float)wg);	
			printf("NavFeatures. weight %u= %.3f loaded\n", i, wg);
		} else {
			//printf("param '%s' not found\n", st.c_str());
			ok = false;
		}
		i++;
	}
	
	
	n.param("use_laser_projection", use_laser_projection_, false);
	
	n.param("use_uva_features", use_uva_features_, false);
	
	if(use_uva_features_)
	{
		use_laser_projection_ = true;
		//uva_features_ = new uva_cost_functions::UvaFeatures(tf);
		if(use_laser_projection_) {
			max_cost_obs_ = distance_functions(0.0, INVERSE_DEC); //EXP_DEC //INVERSE_DEC
			setupProjection();
		}
		
	} else {
		
		//If we use the laser projection onto the static map
		if(use_laser_projection_) {
			max_cost_obs_ = distance_functions(0.0, INVERSE_DEC); //EXP_DEC //INVERSE_DEC
			setupProjection();
		
		}
	
		float front;
		n.param("stddev_person_front", front, (float)1.2);
		float aside;
		n.param("stddev_person_aside", aside, (float)(1.2/1.5)); //=0.8
		
		n.param("enable_grouping", grouping_, true); 
		n.param("stddev_group", stddev_group_, (float)0.8);
		n.param("grouping_distance", grouping_distance_, (float)1.5);
		
		amp_ = 1.80;			//Amplitude, max value of the gaussian
		sigmas_.push_back(front);  //front of person. front
		sigmas_.push_back(aside); //aside of person. front
		sigmas_.push_back(sigmas_.at(1)); //front of person. back
		sigmas_.push_back(sigmas_.at(1)); //aside of person. back
	}
	
	sub_people_ = nh_.subscribe("/people/navigation", 1, &NavFeatures::peopleCallback, this); 
	
	pub_gaussian_markers_ = n.advertise<visualization_msgs::MarkerArray>("gaussian_markers", 1);
 	
}




features::NavFeatures::~NavFeatures() {
	//if(use_uva_features_)
		//delete uva_features_;
}



void features::NavFeatures::setupProjection()
{
	
	people_paint_area_ = 25;
	
	ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap> ("/static_map");
	while (! ros::service::waitForService("/static_map",1)){
		ROS_INFO("Waiting for map service");
	}

	nav_msgs::GetMap srv;
	map_client.call(srv);
	ROS_INFO_STREAM(srv.response.map.info);
	map_image_ = cv::Mat(srv.response.map.info.height, srv.response.map.info.width,CV_8UC1, cv::Scalar(0));
	map_metadata_ =srv.response.map.info;
	resolution_ = (double) map_metadata_.resolution;
	origin_.push_back(map_metadata_.origin.position.x);
	origin_.push_back(map_metadata_.origin.position.y);
	origin_.push_back(tf::getYaw(map_metadata_.origin.orientation));
	uint8_t *myData = map_image_.data;
	for (int i=0;i<srv.response.map.data.size();i++){
		if (srv.response.map.data.at(i)==100  || srv.response.map.data.at(i)==-1 ){
		}
		else {
			map_image_.data[i] = 255;
		}
	}

	dtMutex_.lock();
	distance_transform_ = cv::Mat(map_image_.rows,map_image_.cols,CV_32FC1);
	cv::distanceTransform(map_image_,distance_transform_,CV_DIST_L1,3);
	dtMutex_.unlock();
 
	sub_pc_ = nh_.subscribe("/scan360/point_cloud", 1, &NavFeatures::pcCallback, this);
}




void features::NavFeatures::peopleCallback(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg) 
{
	/*if(use_uva_features_) {
		uva_features_->people_mutex_.lock();
		uva_features_->setPeople(*msg);
		uva_features_->people_mutex_.unlock();
	} else {*/
	
		peopleMutex_.lock();
		people_ = msg->personPoses;
		if(people_frame_id_.empty())
			people_frame_id_ = msg->header.frame_id;
		peopleMutex_.unlock();
	
	//}
	
}


void features::NavFeatures::setPeople(upo_msgs::PersonPoseArrayUPO p)
{
	peopleMutex_.lock();
    
    people_.clear();
    people_ = p.personPoses;
    people_frame_id_ = p.header.frame_id;
	
	peopleMutex_.unlock();
}


//Point cloud callback
// Only used if laser projection is employed
void features::NavFeatures::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_in){
	
	sensor_msgs::PointCloud2 lcloud;
	sensor_msgs::PointCloud2 in = *pc_in;
	in.header.stamp = ros::Time();
	try{  
		//if(tf_listener_->waitForTransform(pc_in->header.frame_id, "/map", pc_in->header.stamp, ros::Duration(0.5))){
			//laserMutex_.lock();
			if(!pcl_ros::transformPointCloud("/map", in, lcloud, *tf_listener_))
				ROS_ERROR("TransformPointCloud failed!!!!!");
			//laserMutex_.unlock();
			//updateDT();
		//} else {
			//printf("waitForTransform  failed!\n");
		//}
	} catch (tf::TransformException ex){
		ROS_ERROR("NAV FEATURES. pcCallback. TransformException: %s", ex.what());
	}
	
	laserMutex_.lock();
	laser_cloud_ = lcloud;
	laserMutex_.unlock();
}



void features::NavFeatures::updateDistTransform(){
	
	sensor_msgs::PointCloud temp_pt_cloud;
	laserMutex_.lock();  
	sensor_msgs::PointCloud2 lcloud = laser_cloud_;
	laserMutex_.unlock();
	
	if(lcloud.data.size() <= 0) {
		ROS_ERROR("No cloud updated");
		return;
	}
	
	bool done = sensor_msgs::convertPointCloud2ToPointCloud(lcloud, temp_pt_cloud);
	if(!done) 
		ROS_ERROR("\n\nNAV FEATURES. UPDATEDT. convertPointCloud2toPoingCloud!!!!!\n");
	
	//Add the laser readings (pointcloud) to the map
	cv::Mat map_copy = map_image_.clone();
	for (int i =0;i<temp_pt_cloud.points.size();i++){
		std::vector<int> pix = worldToMap(&(temp_pt_cloud.points[i]),&map_metadata_);
	
		if(pix[0] < 0.0 || pix[0] > map_metadata_.width || pix[1] < 0.0 || pix[1] > map_metadata_.height) 
			ROS_ERROR("\n\nNAV FEATURES. UPDATEDT. pix out of range!!!!!\n");

		map_copy.at<unsigned char>(pix[1],pix[0]) = 0;
	}
	
	//Add the people detected to the map
	peopleMutex_.lock();
	std::vector<upo_msgs::PersonPoseUPO> per = people_;
	peopleMutex_.unlock();
	for (int i=0; i<per.size(); i++){
		geometry_msgs::Point32 temp_point;
		temp_point.x = per[i].position.x;
		temp_point.y = per[i].position.y;
		std::vector<int> pix = worldToMap(&temp_point,&map_metadata_);
		for (int j = -floor(people_paint_area_/2);j<ceil(people_paint_area_/2);j++){
			for (int k = -floor(people_paint_area_/2);k<ceil(people_paint_area_/2);k++){
				if (pix[1]+k>=0 && pix[0]+j>=0){
					map_copy.at<unsigned char>(pix[1]+k,pix[0]+j) =255;
				}
			}

		}
	}
	
	cv::Mat dt;
	try{
		//cv::distanceTransform(map_copy, distance_transform_, CV_DIST_L1,3);
		cv::distanceTransform(map_copy, dt, CV_DIST_L1, 3);
	} catch(...){
		ROS_ERROR("\n\nNAV FEATURES. UPDATEDT. cv::distanceTransform!!!!!\n");
	} 
	
	dtMutex_.lock();
	distance_transform_ = dt.clone();
	dtMutex_.unlock(); 

	//gettimeofday(&stop, NULL);
	//printf("took %lu\n", stop.tv_usec - start.tv_usec);
	//imwrite(ros::package::getPath("navigation_features").append("/test_write.jpg"), map_copy);
}


std::vector<int> features::NavFeatures::worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata){
	std::vector<int> pixels;
	float x_map = world_point->x - map_metadata->origin.position.x;
	float y_map = world_point->y - map_metadata->origin.position.y;
	pixels.push_back((int)floor(x_map/map_metadata->resolution ));
	pixels.push_back((int)floor(y_map/map_metadata->resolution));
	return pixels;
};





void features::NavFeatures::setGoal(geometry_msgs::PoseStamped g) {

	/*if(use_uva_features_)
		uva_features_->setGoal(g);
	else*/
		goal_ = g;
}




bool features::NavFeatures::poseValid(geometry_msgs::PoseStamped* pose)
{
	//Take the values of the state
	float x_i = pose->pose.position.x;
	float y_i = pose->pose.position.y;

	geometry_msgs::PoseStamped p_in = *pose;

	float local_cost = 0.0;
	float global_cost = 0.0;
	//used to put things into grid coordinates
	unsigned int cell_x, cell_y;
	unsigned char cost;

	//Global costmap ------------------------------------------
	geometry_msgs::PoseStamped p_out_map;
	try {					
		tf_listener_->transformPose("map", p_in, p_out_map); 
	}catch (tf::TransformException ex){
		ROS_WARN("isValid. x:%.2f, y:%.2f. TransformException: %s",x_i, y_i, ex.what());
		return false;
	}
	//get the cell coord of the center point of the robot
	if(!costmap_global_->worldToMap((double)p_out_map.pose.position.x, (double)p_out_map.pose.position.y, cell_x, cell_y))
		return false;

	//if number of points in the footprint is less than 3, we'll just assume a circular robot
	//if(myfootprint->size() < 3){
	cost = costmap_global_->getCost(cell_x, cell_y);
	if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION)
		return false;
	//}


	//Local costmap---------------------------------------------
	geometry_msgs::PoseStamped p_out_odom;
	try {					
		tf_listener_->transformPose("odom", p_in, p_out_odom); 
	}catch (tf::TransformException ex){
		ROS_WARN("isValid. x:%.2f, y:%.2f. TransformException: %s",x_i, y_i, ex.what());
		return false;
	}

	//get the cell coord of the center point of the robot
	if(!costmap_local_->worldToMap((double)p_out_odom.pose.position.x, (double)p_out_odom.pose.position.y, cell_x, cell_y))
	  return false;

	//if number of points in the footprint is less than 3, we'll just assume a circular robot
	//if(myfootprint->size() < 3){
	cost = costmap_local_->getCost(cell_x, cell_y);
	if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION)
		return false;
	//}

	/*
	//now we really have to lay down the footprint in the local costmap grid
	unsigned int x0, x1, y0, y1;
	double line_cost = 0.0;
	double footprint_cost = 0.0;

	//we need to rasterize each line in the footprint
	for(unsigned int i = 0; i < footprint->size() - 1; ++i){
		//get the cell coord of the first point
		if(!costmap_local->worldToMap(footprint->at(i).x, footprint->at(i).y, x0, y0))
			return false;

		//get the cell coord of the second point
		if(!costmap_local->worldToMap(footprint->at(i+1).x, footprint->at(i+1).y, x1, y1))
			return false;

		line_cost = lineCost(x0, x1, y0, y1);
		footprint_cost = std::max(line_cost, footprint_cost);

		//if there is an obstacle that hits the line... we know that we can return false right away 
		if(line_cost < 0)
			return false;
	}

	//we also need to connect the first point in the footprint to the last point
	//get the cell coord of the last point
	if(!costmap_local->worldToMap(footprint->back().x, footprint->back().y, x0, y0))
		return false;

	//get the cell coord of the first point
	if(!costmap_local->worldToMap(footprint->front().x, footprint->front().y, x1, y1))
		return false;

	line_cost = lineCost(x0, x1, y0, y1);
	footprint_cost = std::max(line_cost, footprint_cost);

	if(line_cost < 0)
		return false;
	*/

	return true;
}


//calculate the cost of a ray-traced line
/*float features::NavFeatures::lineCost(int x0, int x1, int y0, int y1){
		
	float line_cost = 0.0;
	float point_cost = -1.0;

	for( LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
	{
		point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

		if(point_cost < 0)
			return -1;

		if(line_cost < point_cost)
			line_cost = point_cost;
	}
	return line_cost;
}*/

/*float features::NavFeatures::pointCost(int x, int y){
	unsigned char cost = costmap_local_->getCost(x, y);
	//if the cell is in an obstacle the path is invalid
	if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION){
		return -1.0;
	}
		return cost;
}*/



float features::NavFeatures::costmapPointCost(float x, float y) const {
			
	//Coordinates X and Y are in the robot local frame (base_link), we transform them to odom
	geometry_msgs::PointStamped p_in;
	p_in.header.frame_id = "base_link"; 
	p_in.header.stamp = ros::Time(0);
	p_in.point.x = x;
	p_in.point.y = y;
			
	float local_cost = 0.0;
	float global_cost = 0.0;
	//used to put things into grid coordinates
	unsigned int cell_x, cell_y;
	unsigned char cost;

	//Global costmap ------------------------------------------
	geometry_msgs::PointStamped p_out_map;
	try {					
		tf_listener_->transformPoint("map", p_in, p_out_map); 
	}catch (tf::TransformException ex){
		ROS_WARN("CostmapPointCost. x:%.2f, y:%.2f. TransformException: %s",x, y, ex.what());
		return -100.0;
	}
	//get the cell coord of the center point of the robot
	if(!costmap_global_->worldToMap((double)p_out_map.point.x, (double)p_out_map.point.y, cell_x, cell_y)) {
		return 254.0;
	}
	
	global_cost = costmap_global_->getCost(cell_x, cell_y);
	//printf("GC: %.1f \t", global_cost);
		

	//Local costmap---------------------------------------------
	geometry_msgs::PointStamped p_out_odom;
	try {					
		tf_listener_->transformPoint("odom", p_in, p_out_odom); 
	}catch (tf::TransformException ex){
		ROS_WARN("CostmapPointCost. x:%.2f, y:%.2f. TransformException: %s",x, y, ex.what());
		-100.0;
	}

	//get the cell coord of the center point of the robot
	if(!costmap_local_->worldToMap((double)p_out_odom.point.x, (double)p_out_odom.point.y, cell_x, cell_y))
		return 254.0;

	local_cost = costmap_local_->getCost(cell_x, cell_y);
	//printf("LC: %.1f \t", local_cost);
			
	if(local_cost > global_cost)
		return local_cost;
	else
		return global_cost; 
}





float features::NavFeatures::getCost(geometry_msgs::PoseStamped* s)
{
	
	//if(use_uva_features_)
		//uva_features_->getCost(s);	
	
	
	std::vector<float> features;

	//Goal distance cost
	float dist_cost = goalDistFeature(s);
	features.push_back(dist_cost);
	if(dist_cost > 1.0 || dist_cost < 0.0)
		printf("GetCost. Dist_cost out of range!!!\n");
			
	//Obstacle distance cost
	float obs_cost = obstacleDistFeature(s);
	features.push_back(obs_cost);
	if(obs_cost > 1.0 || obs_cost < 0.0) {
		printf("GetCost. Obs_cost out of range!!! = %.2f\n", obs_cost);
	}
	//Proxemics cost
	float prox_cost = proxemicsFeature(s);
	features.push_back(prox_cost);
	if(prox_cost > 1.0 || prox_cost < 0.0) {
		printf("GetCost. Prox_cost out of range!!!\n");
		prox_cost = 0.0;
	}

	if(prox_cost == 0.0)
		return (0.5*dist_cost + 0.5*obs_cost);	
	
	
	float cost = 0.0;
	for(unsigned int i=0; i<w_.size(); i++)
		cost += w_[i]*features[i];
	
	//if(cost > 1.0)
	//	printf("RRT* COST OUT OF RANGE!!! %.3f\n", cost);
	return cost;	
}



std::vector<float> features::NavFeatures::getFeatures(geometry_msgs::PoseStamped* s) 
{
	std::vector<float> features;
	//Goal distance cost
	float dist_cost = goalDistFeature(s);
	features.push_back(dist_cost);
			
	//Obstacle distance cost
	float obs_cost = obstacleDistFeature(s);
	features.push_back(obs_cost);
	
	//Proxemics cost
	float prox_cost = proxemicsFeature(s);
	features.push_back(prox_cost);
	
	return features;
}




float features::NavFeatures::goalDistFeature(geometry_msgs::PoseStamped* s)  {
	float dx = goal_.pose.position.x - s->pose.position.x;
	float dy = goal_.pose.position.y - s->pose.position.y;
	return (float)(sqrt(dx*dx + dy*dy)/max_planning_dist_);
}



float features::NavFeatures::obstacleDistFeature(geometry_msgs::PoseStamped* s)  {
	
	if(use_laser_projection_) {
		
		//Transform to map coordinates
		geometry_msgs::PoseStamped sm = transformPoseTo(*s, std::string("/map"), true);
		
		// This functions transforms position of people to pixels on map, gets distance and then converts to meters
		float distance_x = sm.pose.position.x - (origin_)[0];
		float distance_y = sm.pose.position.y - (origin_)[1];

		float px = floor(distance_x/resolution_);
		//float py =distanceTransform->rows-floor(distance_y /resolution);
		float py =floor(distance_y/resolution_);
		//cout<<px<<"  "<<py<<endl;
		float distance = 0.0;
		if (py<0 || px<0) {
			distance = 0.0;
		} else{
			try{
				dtMutex_.lock();
				distance = distance_transform_.at<float>(py,px)*resolution_;
				dtMutex_.unlock();
			} catch(...)
			{
				ROS_ERROR("ERROR. OBSTACLE FEATURE. px:%.2f, py:%.2f, distance:%.2f", px, py, distance);
				distance = 0.0;
			}
		}
		// Take into account the robot radius 
		if(distance <= insc_radius_robot_)
			distance = 0.0;
		else
			distance = distance - insc_radius_robot_;
			
			
		float d = distance_functions(distance, INVERSE_DEC); //EXP_DEC //INVERSE_DEC
		//Normalize
		//printf("Distance: %.4f, Obs cost: %.4f, norm: %.4f\n", distance, d, (d/max_cost_obs_));
		return d/max_cost_obs_;
		
	} else {
		
		float obs_cost = costmapPointCost((float)s->pose.position.x, (float)s->pose.position.y);
		return obs_cost/254.0;
	}
}


float features::NavFeatures::distance_functions(const float distance, const dist_type type){
	// different distance transformations between two points are defined here
	// _INC means the function is monotonically increasing
	// _DEC means that the function is monotonically decreasing
	float out=0;
	switch (type){
		case LINEAR_INC:	
			out = distance;
			break;
		case LOG_INC:   	
			out = 2*log(1+distance);
			break;
		case EXP_INC:   	
			out = exp((distance-2)/4-1);
			break;
		case INVERSE_DEC: 	
			out = 2/(distance+0.2);
			break;
		case LOG_DEC:     	
			out = 1/log(0.5*distance+1.2);
			break;
		case EXP_DEC:    	
			out = exp(3-3*distance);
			break;
		default:
			out=distance;
		break;
	}
	return out;
}



void features::NavFeatures::calculateGaussians()
{
	std::vector<upo_msgs::PersonPoseUPO> people_aux;
	std::string frame;
	// read the person list (thread safe)
	peopleMutex_.lock();
	people_aux = people_;
	frame = people_frame_id_;
	peopleMutex_.unlock();
	
	std::vector<gaussian> gauss_aux;
	
	if(grouping_) 
	{
		
		std::vector<group_candidate> group_candidates;
		for(unsigned int i=0; i<people_aux.size(); i++)
		{
			//If the person is moving, do not group
			if(people_aux.at(i).vel > 0.18) {
				
				gaussian f;
				f.type = FRONT;
				f.x = people_aux.at(i).position.x;
				f.y = people_aux.at(i).position.y;
				f.th = tf::getYaw(people_aux.at(i).orientation);
				f.sx = sigmas_[0]; // 1.2
				f.sy = sigmas_[1]; // 1.2/1.5 = 0.8
				
				gauss_aux.push_back(f);
				
				gaussian b;
				b.type = BACK;
				b.x = people_aux.at(i).position.x;
				b.y = people_aux.at(i).position.y;
				float angle = (tf::getYaw(people_aux.at(i).orientation)) + M_PI;
				b.th = normalizeAngle(angle, -M_PI, M_PI);
				b.sx = sigmas_[2]; // 0.8
				b.sy = sigmas_[3]; // 0.8
				
				gauss_aux.push_back(b);
					
			} else {  //Obtain the group candidates
			
				group_candidate cand;
				geometry_msgs::Pose2D p1;
				p1.x = people_aux.at(i).position.x;
				p1.y = people_aux.at(i).position.y;
				p1.theta = tf::getYaw(people_aux.at(i).orientation);
				cand.position = p1;
				geometry_msgs::Pose2D p2;
				//Calculate the iteraction points
				p2.x = p1.x + (grouping_distance_/2.0)*cos(p1.theta);
				p2.y = p1.y + (grouping_distance_/2.0)*sin(p1.theta);
				p2.theta = p1.theta;
				cand.interaction_point = p2;
				cand.group = -1;
				group_candidates.push_back(cand);
			}
		}
		
		//check distance between the interaction points
		std::vector<group_candidate> group;
		int group_number = 1;
		for(unsigned int i=0; i<group_candidates.size(); i++)
		{
			if(group_candidates.at(i).group == -1) {
				group_candidates.at(i).group = group_number;
				group.push_back(group_candidates.at(i));
				
				geometry_msgs::Pose2D c1 = group_candidates.at(i).interaction_point;
				for(unsigned int j=0; j<group_candidates.size(); j++)
				{
					if(j!=i && group_candidates.at(j).group == -1) {
						geometry_msgs::Pose2D c2 = group_candidates.at(j).interaction_point;
						float dist = sqrt((c1.x-c2.x)*(c1.x-c2.x) + (c1.y-c2.y)*(c1.y-c2.y));
						if(dist < ((grouping_distance_/2.0)*0.9))
						{
							group_candidates.at(j).group = group_number;
							group.push_back(group_candidates.at(j));
						}
					}
				}
				
				//Check the members of the group and create the 
				//corresponding gaussians
				if(group.size() > 0)
				{
					//Only one person --> create the regular gaussian function
					if(group.size() == 1)
					{
						gaussian f;
						f.type = FRONT;
						f.x = group.at(0).position.x;
						f.y = group.at(0).position.y;
						f.th = group.at(0).position.theta;
						f.sx = sigmas_[0]; // 1.2
						f.sy = sigmas_[1]; // 1.2/1.5 = 0.8
						
						gauss_aux.push_back(f);
						
						gaussian b;
						b.type = BACK;
						b.x = group.at(0).position.x;
						b.y = group.at(0).position.y;
						float angle = (group.at(0).position.theta) + M_PI;
						b.th = normalizeAngle(angle, -M_PI, M_PI);
						b.sx = sigmas_[2]; // 0.8
						b.sy = sigmas_[3]; // 0.8
						
						gauss_aux.push_back(b);
						
					} else {
						// Calculate the center of the group
						float x = 0.0;
						float y = 0.0;
						float th = 0.0;
						float max_dist = 0.0;
						for(unsigned int k=0; k<group.size(); k++)
						{
							x = x + group.at(k).interaction_point.x;
							y = y + group.at(k).interaction_point.y;
							th = th + group.at(k).interaction_point.theta;
							
							float xk = group.at(k).position.x;
							float yk = group.at(k).position.y;
							for(unsigned int h=0; h<group.size() && h!=k; h++)
							{
								float xh = group.at(h).position.x;
								float yh = group.at(h).position.y;
								float d = sqrt((xk-xh)*(xk-xh) + (yk-yh)*(yk-yh));
								if(d > max_dist)
									max_dist = d;
							}
						}
						th = normalizeAngle(th, -M_PI, M_PI);
						th = th/group.size();
						th = th + M_PI;
						th = normalizeAngle(th, -M_PI, M_PI);
						
						x = x/group.size();
						y = y/group.size();
						
						gaussian g;
						g.type = AROUND;
						g.x = x;
						g.y = y;
						g.th = th;
						g.sx = sigmas_[2] + max_dist; // 0.8
						g.sy = sigmas_[2] + max_dist;  // 0.8
						gauss_aux.push_back(g);
					}
				}
				group.clear();
				group_number++;
			}
		}
		
		
	} else {
		
		for(unsigned int i=0; i<people_aux.size(); i++)
		{
			gaussian f;
			f.type = FRONT;
			f.x = people_aux.at(i).position.x;
			f.y = people_aux.at(i).position.y;
			f.th = tf::getYaw(people_aux.at(i).orientation);
			f.sx = sigmas_[0]; // 1.2
			f.sy = sigmas_[1]; // 1.2/1.5 = 0.8
			
			gauss_aux.push_back(f);
			
			gaussian b;
			b.type = BACK;
			b.x = people_aux.at(i).position.x;
			b.y = people_aux.at(i).position.y;
			float angle = (tf::getYaw(people_aux.at(i).orientation)) + M_PI;
			b.th = normalizeAngle(angle, -M_PI, M_PI);
			b.sx = sigmas_[2]; // 0.8
			b.sy = sigmas_[3]; // 0.8
			
			gauss_aux.push_back(b);
			
		}
	}
	
	gaussianMutex_.lock();
	gaussians_ = gauss_aux;
	gaussianMutex_.unlock();
	
	visualization_msgs::MarkerArray m;
	std::vector<visualization_msgs::Marker> markers;
	visualization_msgs::Marker marker;
	for(unsigned int i=0; i<gauss_aux.size(); i++)
	{
		marker.header.frame_id = frame; 
		marker.header.stamp = ros::Time::now();
		marker.ns = "gaussians";
		marker.id = (i+1);
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = gauss_aux.at(i).x;
		marker.pose.position.y = gauss_aux.at(i).y;
		marker.pose.position.z = 1.1;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(gauss_aux.at(i).th);
		marker.scale.x = 0.7;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration(1.0);
		m.markers.push_back(marker);
	}
	pub_gaussian_markers_.publish(m);
	
	//printf("\n\n %u Gaussians calculated!!!!\n", (unsigned int)gauss_aux.size());
	
	if(use_laser_projection_)
		updateDistTransform();
	
}



float features::NavFeatures::proxemicsFeature(geometry_msgs::PoseStamped* s)  {
	
	std::vector<gaussian> gauss;
	gaussianMutex_.lock();
	gauss = gaussians_;
	gaussianMutex_.unlock();
	
	if(gauss.size() == 0)
		return 0.0;
	
	//peopleMutex_.lock();
	//std::string people_frame = people_frame_id_;
	//peopleMutex_.unlock();
	
	//We need to transform the sample to the same frame of the people
	geometry_msgs::PoseStamped robot = *s;
	robot = transformPoseTo(robot, people_frame_id_, true);
	
	if(robot.header.frame_id.empty()){
		printf("frame id empty. returning 0\n");
		return 0.0;
	}
	float rx = robot.pose.position.x; 
	float ry = robot.pose.position.y;
	
	float prox_cost = 1.0;
	for (std::vector<gaussian>::iterator it = gauss.begin(); it != gauss.end(); ++it)
	{
		//odom coordinates
		float ph = it->th;
		float px = it->x;
		float py = it->y;
		
		//printf("robot x:%.2f, y:%.2f, th:%.2f, sx:%.2f, sy:%.2f\n", it->x, it->y, it->th, it->sx, it->sy);
		
		/*
		First, transform the robot location into person location frame: 
									|cos(th)  sin(th)  0|
			Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
									|  0        0      1|
					                     
			x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
			y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
		*/
		float x_r = (rx - px)*cos(ph) + (ry - py)*sin(ph);
		float y_r = (rx - px)*(-sin(ph)) + (ry - py)*cos(ph);
		
		float cost = 0.0;
		if(it->type == AROUND) {
			cost = gaussian_function(x_r, y_r, it->sx, it->sy);
		} else if (it->type == FRONT) {
			if(x_r >= 0.0) {
				cost = gaussian_function(x_r, y_r, it->sx, it->sy);
			}
		} else {  //type BACK
			if(x_r > 0.0) {
				cost = gaussian_function(x_r, y_r, it->sx, it->sy);
			}
		}
	
		prox_cost = prox_cost * (cost + 1.0);
		
	}
	
	prox_cost = prox_cost - 1.0;	
	//printf("prox_cost: %.3f\n", prox_cost);
	
	//return (prox_cost<=1.0)?prox_cost:1.0;
	if(prox_cost > 1.0) {
		//printf("prox_cost = %.3f\n", prox_cost);
		prox_cost = 1.0;
	}
	return prox_cost;
}


/*
float features::NavFeatures::proxemicsFeature(geometry_msgs::PoseStamped* s)  {
	
	callbackMutex_.lock();
	unsigned int size = (unsigned int)people_.size();
	callbackMutex_.unlock();
	
	if(size == 0)
		return 0.0;
	
	//We need to transform the state sample to the same people frame
	geometry_msgs::PoseStamped robot = *s;
	
	callbackMutex_.lock();
	robot = transformPoseTo(robot, people_frame_id_, true);
	callbackMutex_.unlock();
	
	float prox_cost = getProxemicsCost((float)robot.pose.position.x, (float)robot.pose.position.y);
	//return prox_cost/amp_;
	return prox_cost;
}*/


float features::NavFeatures::getProxemicsCost(float rx, float ry)  {

	// read the person list (thread safe)
	std::vector<upo_msgs::PersonPoseUPO> people_aux;
	peopleMutex_.lock();
	people_aux = people_;
	peopleMutex_.unlock();
	
	// Cost 0.0 if no persons detected
	if(people_aux.size() == 0) 
	{
		return 0.0;
  	}
  	

	float prox_cost = 1;
	
  	for (std::vector<upo_msgs::PersonPoseUPO>::iterator it = people_aux.begin(); it != people_aux.end(); ++it)
  	{
		//odom coordinates
		float ph = tf::getYaw(it->orientation);
		float px = it->position.x;
		float py = it->position.y;
		bool front = true;
		/*
		First, transform the robot location into person location frame: 
									|cos(th)  sin(th)  0|
			Rotation matrix R(th)= 	|-sin(th) cos(th)  0|
									|  0        0      1|
					                     
			x' = (xr-xp)*cos(th_p)+(yr-yp)*sin(th_p)
			y' = (xr-xp)*(-sin(th_p))+(yr-yp)*cos(th_p)
		*/
		float x_r = (rx - px)*cos(ph) + (ry - py)*sin(ph);
		float y_r = (rx - px)*(-sin(ph)) + (ry - py)*cos(ph);
		if(x_r < 0.0)
			front = false;
		
		float cost = gaussian_function(x_r, y_r, front);
		
		prox_cost = prox_cost * (cost + 1.0);
		
	}
	
	prox_cost = prox_cost - 1.0;	
	
	return (prox_cost<=1.0)?prox_cost:1.0;
    
}


float features::NavFeatures::gaussian_function(float x, float y, float sx, float sy) 
{
	return (amp_*exp(-( (x*x)/(2*(sx*sx)) + (y*y)/(2*(sy*sy))))) / amp_;
}



float features::NavFeatures::gaussian_function(float x, float y, bool fr) 
{
	float sx, sy;
	//front
	if(fr) {
		sx = sigmas_[0]; // 1.2
		sy = sigmas_[1]; // 1.2/1.5 = 0.8
	} else { //back
		sx = sigmas_[2]; // 0.8
		sy = sigmas_[3]; // 0.8
	}
	return (amp_*exp(-( (x*x)/(2*(sx*sx)) + (y*y)/(2*(sy*sy))))) / amp_;

	//phi in radians
	//positions are in odom coordinates
	//Proxemics:
	// intimate: 15-45 cm
	// personal: 46-120 cm
	// social: 120-360 cm
	// public: >360 cm 
	/*double A = amp_;
	double x = x_rob;
	double y = y_rob;
	double x0 = x_per;		
	double y0 = y_per; 
	double z = 0.0;
	
	// 3sigma = 3.60 --> sigma = 1.20
	// 2sigma = 3.60 --> sigma = 1.28
	double sigmaX = sigmas.at(0); //front 1.20
	//Según el artículo "Companion..." de Kirby --> sigmaX = 1.5*sigmaY;
	double sigmaY = sigmas.at(1);  //aside 1.20/1.50 = 0.8
	
	// front
	if(fr) {
	  double a = pow(cos(phi),2)/2/pow(sigmaX,2) + pow(sin(phi),2)/2/pow(sigmaY,2);
	  double b = - sin(2*phi)/4/pow(sigmaX,2) + sin(2*phi)/4/pow(sigmaY,2); 
	  double c = pow(sin(phi),2)/2/pow(sigmaX,2) + pow(cos(phi),2)/2/pow(sigmaY,2);
	  z = A*exp(-(a*pow((x-x0),2) + 2*b*(x-x0)*(y-y0) + c*pow((y-y0),2)));
	  
	} else { //back
	  //take the opposite orientation
	  double o_phi = 0.0;
	  if(phi > 0)
	    o_phi = phi - M_PI;
	  else
	    o_phi = phi + M_PI;
	    
	  sigmaX = sigmas.at(2);
	  sigmaY = sigmas.at(3); 
	  double a = pow(cos(o_phi),2)/2/pow(sigmaX,2) + pow(sin(o_phi),2)/2/pow(sigmaY,2);
	  double b = - sin(2*o_phi)/4/pow(sigmaX,2) + sin(2*o_phi)/4/pow(sigmaY,2); 
	  double c = pow(sin(o_phi),2)/2/pow(sigmaX,2) + pow(cos(o_phi),2)/2/pow(sigmaY,2);
	  z = A*exp(-(a*pow((x-x0),2) + 2*b*(x-x0)*(y-y0) + c*pow((y-y0),2)));
	}
	//ROS_INFO("GC=%.3f, xp=%.1f, yp=%.1f, h=%.1f, xr=%.1f, yr=%.1f, sx=%.1f sy=%.1f",z,x0,y0,phi,x,y,sigmaX,sigmaY);
	return z;*/
}






geometry_msgs::PoseStamped features::NavFeatures::transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime)
{
	geometry_msgs::PoseStamped in = pose_in;
	if(!usetime)
		in.header.stamp = ros::Time();
		
	geometry_msgs::PoseStamped pose_out;
	
	geometry_msgs::Quaternion q = in.pose.orientation;
	if(!isQuaternionValid(q))
	{
		ROS_WARN("NavFeatures. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
		in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	}
	try {
		tf_listener_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("NavFeatures. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
	return pose_out;
}




bool features::NavFeatures::isQuaternionValid(const geometry_msgs::Quaternion q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
		ROS_ERROR("Quaternion has infs!!!!");
		return false;
    }
    if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
		ROS_ERROR("Quaternion has nans !!!");
		return false;
	}
	
	if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
		ROS_ERROR("Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
		return false;
	}

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding.");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
}


float features::NavFeatures::normalizeAngle(float val, float min, float max) {
	
	float norm = 0.0;
	if (val >= min)
		norm = min + fmod((val - min), (max-min));
	else
		norm = max - fmod((min - val), (max-min));
            
    return norm;
}








