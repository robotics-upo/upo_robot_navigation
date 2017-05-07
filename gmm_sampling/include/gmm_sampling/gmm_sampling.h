#ifndef GMM_SAMPLING_H_
#define GMM_SAMPLING_H_

#include <vector>
#include <math.h>
#include <utility> //pair
#include <iostream>
#include <random>
#include <map>
//#include <initializer_list>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <dirent.h> //For directory read

//ROS
//#include <ros/ros.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>
//#include <upo_msgs/PersonPoseUPO.h>
//#include <upo_msgs/PersonPoseArrayUPO.h>
//#include <geometry_msgs/Pose.h>
//#include <visualization_msgs/Marker.h>
//#include <geometry_msgs/Pose.h>

//for GMM sampling
#include <gmm_sampling/eigenmvn.h>

//boost
//#include <boost/math/constants/constants.hpp>
//For pre C++ 11 gamma function
//#include <boost/math/special_functions/gamma.hpp>

//#include <boost/thread.hpp>  /* Mutex */

//Read a file
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>      /* printf, scanf, puts, NULL */

//for random number generation
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <cstdlib>




#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif


namespace gmm_samp {


	class GMMSampling 
	{
		public:
		
			struct gmodel_ {
				float weight;
				float mean1;
				float mean2;
				float covar1;
				float covar2;
				float covar12;
				float covar21;
				float maxPDF;
			};

			struct task_ {
				std::string name;
				unsigned int num_modes;
				bool two_homotopies;
				std::vector<gmodel_> gmm;
				std::vector< Eigen::EigenMultivariateNormal<double> > samplers;
			};
		
		

			GMMSampling(std::string task_dir);
			//GMM_sampling(tf::TransformListener* tf, std::string file1, std::string file2, unsigned int mode, int planner);

			~GMMSampling();
		

			// Load the tasks (directories) inside the folder "tasks"
			bool loadTasks(std::string task_dir);
			
			// Calculate the maximum PDF value (mean point) of each model
			void calculateMaxPDFValues();

			// Generate a sampler for each model
			void generateSamplers();

			//Get samples from the GMM according to person orientation
			std::vector< std::pair<double,double> > getApproachSamples(float orientation, unsigned int num_samples);
			//Get samples from the GMM according to the task indicated
			std::vector< std::pair<double,double> > getSamples(unsigned int task, unsigned int num_samples);
				

			unsigned int classifyApproachTask(float ori); 		

			Eigen::Matrix2d genCovar(double v0,double v1,double theta);
			
			float getPDFvalue(gmodel_* g, std::pair<double,double> sample);

			float getPDFValue(unsigned int task, std::pair<double,double> sample);

			std::vector<float> getPDFValues(unsigned int task, std::vector< std::pair<double,double> > samples);

			//void peopleReceived(const upo_msgs::PersonPoseArrayUPO::ConstPtr& msg);
	  
	  		//bool transformPoseTo(double x_in, double y_in, double h_in, std::string frame_in, std::string frame_out, geometry_msgs::Pose &pose_out);

			//void setRobotPose(geometry_msgs::Pose p);
		
			//void setGoalPose(geometry_msgs::Pose t);
			//void printSamples();

			//unsigned int num_people();
			//double getGMMCost(double rx, double ry, bool flag);
			//double gmm_cost(double x_per, double y_per, double phi, double x_rob, double y_rob, bool rol);
			//double getGMMCostXY(double rx, double ry, bool flag);
			//double gmm_costXY(double x_rob, double y_rob, bool rol);

			//double getStdDevR();
			//double getStdDevD();
			//double setStdDev(double stdD, double stdR);

			//std::vector< std::pair<double,double> >	 getvarsX()
			//{
			//	return varsX;
			//}

			//std::vector< std::pair<double,double> >	 getvarsY()
			//{
			// 	return varsY;	
			//}

			//unsigned int getNumModels(){
			//	if(gmm_mode_==2)
			//		return num_models_approach_;
			//	else
			//		return num_models_avoid_;
			//}

			/*std::vector< std::pair<double,double> > getGMMSamples()
			{
				return gmm_samples_;
			}*/

			/*std::vector< Eigen::EigenMultivariateNormal<double> > getModelSamplers()
			{
				if(gmm_mode_==2)
					return model_samplers_approach_;
				else
					return model_samplers_avoid_;	

			}

			std::vector<gmodel> getModel()
			{
				if(gmm_mode_==2){
					for(int i=0;i<gmodels_approach_.size();++i){
						gmodels_approach_.at(i).weight=gmodels_approach_.at(i).weight_approach;
					}
					return gmodels_approach_;
				}
				else{
					for(int i=0;i<gmodels_avoid_.size();++i){
						gmodels_avoid_.at(i).weight=gmodels_avoid_.at(i).weight_approach;
					}
					return gmodels_avoid_;	
				}
				
			}

			std::vector<upo_msgs::PersonPoseUPO> getPeople()
			{
			
				return people_;
			}
			// double setStdDevXY(double stdX, double stdY);		

			void getPersonPoseAndOrientation();

			double	 getPersonOrientation()
			{
				return person_orientation;
			}

			double	 getPersonPoseX()
			{
			 	return person_pose_x;	
			}

			double	 getPersonPoseY()
			{
			 	return person_pose_y;	
			}

			void pdf_norm_term(bool flag);

			bool calculatedTPDF(){
				return total;
			}
			bool updateTPDF(){
				total=!total;
			}

		  	std::vector<double> TPDF_k;
			*/

		private:
		

			std::vector<task_> 			tasks_;

			//unsigned int				num_models_, num_models_approach_, num_models_avoid_;  	//number of models of the GMM
		
			//std::vector<gmodel>			gmodels_, gmodels_approach_, gmodels_avoid_;	  	//store the values of each model
			//std::vector< std::pair<double,double> >			varsX, varsY;
		
			//std::vector< Eigen::EigenMultivariateNormal<double> > model_samplers_; //one sampler per model

			unsigned int 				gmm_dim_;		//number of dimensions (=2,  dist and ori)
		
			//unsigned int 				gmm_mode_; 		//1: dist and ori regarding the closest person
														//2: dist and ori regarding the target
														//3: take into account closest person and target
		
			//std::vector< std::pair<double,double> > gmm_samples_;	//samples of the GMM
		
			//geometry_msgs::Pose 		robot_pose_;
		
			//geometry_msgs::Pose 		goal_pose_;

			// list of person objects
	  		//std::vector<upo_msgs::PersonPoseUPO> people_;
	  		//std::string 				people_frame_id;
	  		//boost::mutex 				callbackMutex;
			//std::mutex 				callbackMutex;
	  		
	  		//tf::TransformListener* 		tf_;

	  		//double stdDist, stdRho;

	  		//double person_orientation, person_pose_x, person_pose_y;
	  		//bool total = false;

	  		//ros::Publisher samples_pub;
	};

} //namespace gmm_samp
#endif
