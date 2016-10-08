
#include <upo_navigation_macro_actions/Yield.h>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <libgen.h>

Yield::Yield()
{
	Yield(NULL, NULL);
}


Yield::Yield(std::string* yamlFile, std::string* pointsFile)
{
	if(!yamlFile) {
		has_map=false;
		return;
	} else
		has_map=true;


	// --- LOAD MAP ---
	printf("Opening obstacles Map File %s\n", yamlFile->c_str());
	
	std::string mapfname = ""; 


	YAML::Node node = YAML::LoadFile(yamlFile->c_str());
	

	try { 
		res = node["resolution"].as<double>(); 

	} catch (YAML::InvalidScalar) { 
		printf("The map does not contain a resolution tag or it is invalid.\n");
		return;
	}
	try { 
		
		std::vector<double> ori = node["origin"].as<std::vector<double> >();
		for(unsigned int i=0; i<ori.size(); i++) 
			origin[i] = ori[i];


	} catch (YAML::InvalidScalar) { 
		printf("The map does not contain an origin tag or it is invalid.\n");
		return;
	}
	try { 
		mapfname = node["image"].as<std::string>();

		// TODO: make this path-handling more robust
		if(mapfname.size() == 0)
		{
			printf("The heightmap tag cannot be an empty string.\n");
			return;
		}
		if(mapfname[0] != '/')
		{
			// dirname can modify what you pass it
			char* fname_copy = strdup(yamlFile->c_str());
			mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
			free(fname_copy);
		}
       
	} catch (YAML::InvalidScalar) { 
		printf("The map does not contain an image tag or it is invalid.\n");
		return;
	}

	map = cv::imread(mapfname);

	mapWidth = map.cols;
	mapHeight = map.rows;

	has_map=true;

	printf("Obstacles Map correctly opened\n");
	printf("Res: %f\n",res);



	// --- LOAD GLOBAL POINTS OF YIELDING ---
	loadYieldPoints(pointsFile);
	
	
}




zone Yield::getType(double x, double y, bool &correct)
{
	
	int value;
	zone type = NORMAL;
	correct=true;
	if(has_map)
	{
		float dx = x-origin[0];
		float dy = y-origin[1];
		
		int i = mapHeight - floor(dy/res);
		int j = floor(dx/res);

		if(i>=mapHeight){ i=mapHeight-1; correct=false;}
		if(i<0){ i=0; correct=false;}
		if(j>=mapWidth){ j=mapWidth-1; correct=false;}
		if(j<0){ j=0; correct=false;}

		cv::Vec3b intensity = map.at<cv::Vec3b>(i, j);
		uchar blue = intensity.val[0];
		uchar green = intensity.val[1];
		uchar red = intensity.val[2];

		//std::cout << "red: " << (unsigned int)red << ", green: " << (unsigned int)green << ", blue: " << (unsigned int)blue << std::endl;

		if((red==255 || red==254) && blue==0 && green==0 && correct) {
			type=NARROW;

		} else if((blue==255 || blue==254) && red==0 && green==0 && correct) {
			type=SUPERNARROW;
		}
	}
	return(type);
}


bool Yield::loadYieldPoints(std::string* pointsFile)
{
	if(!pointsFile){
		printf("Yield. No points file found!!!\n");
		return false;
	}
	YAML::Node node = YAML::LoadFile(pointsFile->c_str());
	
	printf("Loading point file...\n");
	
	//Load yield goal points
	bool ok = true;
	unsigned int i = 1;
	while(ok)
	{
		char buf[10];
		sprintf(buf, "point%u", i);
		std::string st = std::string(buf);
		if(node[st.c_str()]) {
			point p;
			p.x_ = node[st.c_str()]["x"].as<double>();
			p.y_ = node[st.c_str()]["y"].as<double>();
			p.theta_ = node[st.c_str()]["theta"].as<double>();
			yield_points.push_back(p);
			printf("p%u-> x:%.2f, y:%.2f\n", i, yield_points[i-1].x_, yield_points[i-1].y_); 
		}else {
			ok = false;
		}
		i++;
	}
	if(i==1)
		return false;


	//Load center points of the yield areas
	ok = true;
	i = 1;
	while(ok)
	{
		char buf[10];
		sprintf(buf, "center%u", i);
		std::string st = std::string(buf);
		if(node[st.c_str()]) {
			point p;
			p.x_ = node[st.c_str()]["x"].as<double>();
			p.y_ = node[st.c_str()]["y"].as<double>();
			p.theta_ = node[st.c_str()]["theta"].as<double>();
			center_points.push_back(p);
			printf("c%u-> x:%.2f, y:%.2f\n", i, center_points[i-1].x_, center_points[i-1].y_); 
		}else {
			ok = false;
		}
		i++;
	}
		
	return true;
}


void Yield::getClosestPoint(double curr_x, double curr_y, double& goal_x, double& goal_y, double& goal_theta)
{
	point closest;
	float min_dist = 9999.0;
	for(unsigned int i=0; i<yield_points.size(); i++)
	{
		point p = yield_points[i];
		float dist =  sqrt((curr_x - p.x_)*(curr_x - p.x_) + (curr_y - p.y_)*(curr_y - p.y_));
		if(dist < min_dist) {
			min_dist = dist;
			closest = p;
		}
	}
	goal_x = closest.x_;
	goal_y = closest.y_;
	goal_theta = closest.theta_;
	//printf("getClosestPoint. goal_x:%.2f, goal_y:%.2f\n", goal_x, goal_y);
}



bool Yield::getYieldAreaCenterPoint(double rx, double ry, double& cx, double& cy)
{
	point closest;
	float min_dist = 9999.0;
	for(unsigned int i=0; i<center_points.size(); i++)
	{
		point p = center_points[i];
		float dist =  sqrt((rx - p.x_)*(rx - p.x_) + (ry - p.y_)*(ry - p.y_));
		if(dist < min_dist) {
			min_dist = dist;
			closest = p;
		}
	}
	if(min_dist == 9999.0)
		return false;

	cx = closest.x_;
	cy = closest.y_;
	//printf("getClosestAreaPoint. goal_x:%.2f, goal_y:%.2f\n", cx, cy);
	return true;
}




