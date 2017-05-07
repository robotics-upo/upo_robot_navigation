#include <gmm_sampling/gmm_sampling.h>


//using gmm_samp 

gmm_samp::GMMSampling::GMMSampling(std::string task_dir)
{
	
	//gmm_samples_.clear();
	gmm_dim_ = 2;

	//Load the GMM files
	if(!loadTasks(task_dir)) {
		printf("\nGMM Sampling - ERROR LOADING GMMs\n\n");
		return;
	}

	//Calculate the max PDF value for each model
	calculateMaxPDFValues();

	//Generate a sampler for each model
	generateSamplers();
	

}


gmm_samp::GMMSampling::~GMMSampling() {}




bool gmm_samp::GMMSampling::loadTasks(std::string task_dir)
{
	std::vector<task_> aux;

	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (task_dir.c_str())) != NULL) {
		// print all the files and directories within directory 
		while ((ent = readdir (dir)) != NULL) 
		{
			std::size_t found = std::string(ent->d_name).find(".");
  			if (found!=std::string::npos)
				continue;

			task_ task;
			task.name = std::string(ent->d_name); 
			printf("\nReading task: %s\n", ent->d_name);

			//Open the weights file
			std::string file = task_dir + task.name + "/GMM_priors.txt";
			float w_sum = 0.0;
			std::ifstream wfile(file.c_str());
			if(wfile.is_open())
			{
				printf("\tLoading weights values file...\n");
				std::string line;
				getline(wfile,line);
				char* li = new char[line.length()+1];
				std::strcpy(li, line.c_str());
				char* pch;
				pch = strtok(li, "\t");
				int c = 1;
				while(pch!=NULL) {
					gmodel_ gm;
					gm.weight = atof(pch);
					w_sum += gm.weight; 
					printf("\tgmm%i w: %.3f\n", c, gm.weight);
					task.gmm.push_back(gm);
					c++;
					pch = strtok(NULL, "\t");
				}
				task.num_modes = (unsigned int)task.gmm.size();
				if(w_sum > 1.1)
					task.two_homotopies = true;
				else
					task.two_homotopies = false;

			} else {
				return false;
			}
			wfile.close();

			//Open the mean file
			file = task_dir + task.name + "/GMM_mu.txt";
			std::ifstream mufile(file.c_str());
			if(mufile.is_open())
			{
				printf("\tLoading mu values file...\n");
				std::string line;
				int lcount = 1;
				while (getline(mufile,line))
				{
					char* li = new char[line.length()+1];
					std::strcpy(li, line.c_str());
					char* ch;
					ch = strtok(li, "\t");
					if(lcount == 1) {	//line 1
						task.gmm[0].mean1 = atof(ch);
						printf("\tgmm1 - mu1: %.3f\n", task.gmm[0].mean1);
						for(unsigned int i=1; i<task.num_modes; i++) {
							ch = strtok(NULL, "\t");
							task.gmm[i].mean1 = atof(ch);
							printf("\tgmm%i - mu1: %.3f\n", (i+1), task.gmm[i].mean1); 
						}
						lcount++;
					} else {  		//line 2
						task.gmm[0].mean2 = atof(ch);
						printf("\tgmm1 - mu2: %.3f\n", task.gmm[0].mean2);
						for(unsigned int i=1; i<task.num_modes; i++) {
							ch = strtok(NULL, "\t");
							task.gmm[i].mean2 = atof(ch);
							printf("\tgmm%i - mu2: %.3f\n", (i+1), task.gmm[i].mean2);
						}
					}
				}

			} else {
				return false;
			}
			mufile.close();

			//Open the sigma file
			file = task_dir + task.name + "/GMM_sigma.txt";
			std::ifstream sigmafile(file.c_str());
			if(sigmafile.is_open())
			{
				printf("\tLoading sigma values file...\n");
				std::string line;
				int lcount = 1;
				while (getline(sigmafile,line))
				{
					char* li = new char[line.length()+1];
					std::strcpy(li, line.c_str());
					char* ch;
					ch = strtok(li, "\t");
					if(lcount == 1) {	//line 1
						task.gmm[0].covar1 = atof(ch);
						printf("\tgmm1 - covar1: %.3f\n", task.gmm[0].covar1);
						ch = strtok(NULL, "\t");
						task.gmm[0].covar12 = atof(ch);
						printf("\tgmm1 - covar12: %.3f\n", task.gmm[0].covar12);
						for(unsigned int i=1; i<task.num_modes; i++) {
							ch = strtok(NULL, "\t");
							task.gmm[i].covar1 = atof(ch);
							printf("\tgmm%i - covar1: %.3f\n", (i+1), task.gmm[i].covar1); 
							ch = strtok(NULL, "\t");
							task.gmm[i].covar12 = atof(ch);
							printf("\tgmm%i - covar12: %.3f\n", (i+1), task.gmm[i].covar12);
						}
						lcount++;
					} else {  		//line 2
						task.gmm[0].covar21 = atof(ch);
						printf("\tgmm1 - covar21: %.3f\n", task.gmm[0].covar21);
						ch = strtok(NULL, "\t");
						task.gmm[0].covar2 = atof(ch);
						printf("\tgmm1 - covar2: %.3f\n", task.gmm[0].covar2);
						for(unsigned int i=1; i<task.num_modes; i++) {
							ch = strtok(NULL, "\t");
							task.gmm[i].covar21 = atof(ch);
							printf("\tgmm%i - covar21: %.3f\n", (i+1), task.gmm[i].covar21); 
							ch = strtok(NULL, "\t");
							task.gmm[i].covar2 = atof(ch);
							printf("\tgmm%i - covar2: %.3f\n", (i+1), task.gmm[i].covar2);
						}
					}
				}
			} else {
				return false;
			}
			sigmafile.close();

			//Store the task
			aux.push_back(task);
		}
		closedir(dir);

		//Order the aux vector and store them in vector tasks_
		for(unsigned int i=1; i<=aux.size(); i++)
		{
			char buf[10];
			sprintf(buf, "%u-", i);
			std::string id = std::string(buf);
			for(unsigned int j=0; j<aux.size(); j++) 
			{
				std::size_t found = aux[j].name.find(id);
  				if (found!=std::string::npos){
					printf("Storing ordered task %s\n", aux[j].name.c_str());
					tasks_.push_back(aux[j]);
					break;
				}
			}
		}

		return true;

	} else {
		return false;
	}
}




void gmm_samp::GMMSampling::calculateMaxPDFValues()
{
	printf("\nObtaining maximum PDF values....\n");
	for(unsigned int i=0; i<tasks_.size(); i++)
	{
		printf("\tTask %s\n", tasks_[i].name.c_str());
		for(unsigned int j=0; j<tasks_[i].gmm.size(); j++)
		{
			
			gmodel_ gm = tasks_[i].gmm[j];
			
			std::pair<double,double> s = std::make_pair(gm.mean1, gm.mean2);
			tasks_[i].gmm[j].maxPDF = getPDFvalue(&gm, s);
			printf("\t\tModel %u, maxPDF: %.3f\n", (j+1), tasks_[i].gmm[j].maxPDF);
			
		}
	}
	printf("\n\n");
}






void gmm_samp::GMMSampling::generateSamplers()
{
	printf("\nGenerating samplers....\n");
	for(unsigned int i=0; i<tasks_.size(); i++)
	{
		printf("\tTask %s\n", tasks_[i].name.c_str());
		for(unsigned int j=0; j<tasks_[i].gmm.size(); j++)
		{
			printf("\t\tModel %u\n", (j+1));
			gmodel_ gm = tasks_[i].gmm[j];
		
			Eigen::Vector2d mean;
			Eigen::Matrix2d covar;
			// Set the mean
			mean << gm.mean1,gm.mean2; 
		
			// Create a covariance matrix
			// Much wider than it is tall
			// and rotated clockwise by a bit
			covar = genCovar(gm.covar1, gm.covar2, gm.covar12);  

			// Create a bivariate gaussian distribution of doubles.
			// with our chosen mean and covariance
			Eigen::EigenMultivariateNormal<double> normX_cholesk(mean,covar,true);
		
			tasks_[i].samplers.push_back(normX_cholesk);
		}
	}
	printf("\n\n");
}






/**
  Take a pair of un-correlated variances.
  Create a covariance matrix by correlating 
  them, sandwiching them in a rotation matrix.
*/
Eigen::Matrix2d gmm_samp::GMMSampling::genCovar(double v0,double v1,double theta)
{
  Eigen::Matrix2d rot = Eigen::Rotation2Dd(theta).matrix();
  return rot*Eigen::DiagonalMatrix<double,2,2>(v0,v1)*rot.transpose();
}




unsigned int gmm_samp::GMMSampling::classifyApproachTask(float ori)
{

	/* ORDER:
		0- Front
		1- 45_right
		2- 90_right
		3- 135_right
		4- back
		5- 135_left
		6- 90_left
		7- 45_left
	*/
	unsigned int ind = 0;
	//if(ori >= -0.393 && ori <= 0.393)		//Front 	[-pi/8, pi/8]
	//	ind = 0;
	/*else*/if(ori >=-1.178 && ori < -0.393)	//45_right	[-pi/8, -3pi/8]
		ind = 1;
	else if(ori >=-1.963 && ori < -1.178)	//90_right	[-3pi/8, -5pi/8]
		ind = 2;
	else if(ori >=-2.749 && ori < -1.963)	//135_right	[-5pi/8, -7pi/8]
		ind = 3;
	else if(ori >=2.749 || ori < -2.749)	//back		[-7pi/8, 7pi/8]
		ind = 4;
	else if(ori >=1.963 && ori < 2.749)		//135_left	[5pi/8, 7pi/8]
		ind = 5;
	else if(ori >=1.178 && ori < 1.963)		//90_left	[3pi/8, 5pi/8]
		ind = 6;
	else if(ori >0.393 && ori < 1.178)		//45_left	[pi/8, 3pi/8]
		ind = 7;

	return ind;
}


std::vector< std::pair<double,double> > gmm_samp::GMMSampling::getApproachSamples(float orientation, unsigned int num_samples)
{
	printf("GetApproachSamples Service called\n");
	unsigned int i = classifyApproachTask(orientation);
	return getSamples(i, num_samples);
}


std::vector< std::pair<double,double> > gmm_samp::GMMSampling::getSamples(unsigned int task, unsigned int num_samples)
{
	
	//printf("\nGMM biased sampling actived. Generating samples...\n\n");

	task_ t = tasks_[task];

	std::vector< std::pair<double,double> > gmm_samples;

		
	
	//Determine the number of samples of each model according to its weight
	int samples_count = 0;
	int c = 1;
	for(unsigned int i=0; i < t.gmm.size(); i++)
	{

		float w = t.gmm[i].weight;

		unsigned int samples_aux = (unsigned int)trunc(round(num_samples * w)); 
		
		printf("number of gaussians: %u, gmm: %u, weight: %.3f, samples: %u\n", (unsigned int)t.gmm.size(), i, w, samples_aux); 
		if(i == (t.gmm.size() - 1)) {
			if(t.two_homotopies)
				samples_aux = (num_samples*2) - samples_count;
			else
				samples_aux = num_samples - samples_count;
		}
		//Get the samples
		Eigen::Matrix2Xd samples = t.samplers[i].samples(samples_aux);
		
		for(unsigned int j=0; j < samples_aux; j++)
		{
			double dist = samples(0,j);
			double ori = samples(1,j);
			//Euclidean coordinates regarding the person position
			//double x = dist*cos(ori); 
			//double y = dist*sin(ori);
			//printf("%i - sample x: %.2f, y: %.2f\n", c, x, y);
			c++;
			gmm_samples.push_back(std::make_pair(dist,ori));	
		}
		
		samples_count += samples_aux;
	}

	//printSamples();	

	//printf("GMM samples generated!!!\n\n");
	return gmm_samples;	
}




// For the approach task:
//Sample must be the distance and orientation of the point
//regarding the target person pose. 
float gmm_samp::GMMSampling::getPDFValue(unsigned int task, std::pair<double,double> sample)
{
	task_ t = tasks_[task];
	float total_prob = 0.0;
	//for(std::vector<gmodel_>::const_iterator it = t.gmm.begin(); it != t.gmm.end(); ++it )
	for(unsigned int i=0; i<t.gmm.size(); i++)
	{
		float prob = getPDFvalue(&t.gmm[i], sample);
		
		//Normalize
		prob = prob/t.gmm[i].maxPDF;
		
		//if(prob > 1.0)
		//	printf("\nPDF value higher than 1.0!!! It is not normalized!!!!!\n");
			
		total_prob += prob; 
	}

	//return (total_prob/(unsigned int)t.gmm.size());
	return total_prob;

}


std::vector<float> gmm_samp::GMMSampling::getPDFValues(unsigned int task, std::vector< std::pair<double,double> > samples)
{
	std::vector<float> probs;
	for(unsigned int i=0; i<samples.size(); i++)
	{
		probs.push_back(getPDFValue(task, samples[i]));	
	}
	return probs;
}



float gmm_samp::GMMSampling::getPDFvalue(gmodel_* g, std::pair<double,double> sample)
{
	double wi = g->weight;
	Eigen::Matrix2d sigma;
	Eigen::Vector2d mu, samp;
	sigma << g->covar1, g->covar12, g->covar21, g->covar2;
	mu << g->mean1, g->mean2;
	samp << sample.first, sample.second;

	double inExp = -0.5*((samp-mu).transpose())*sigma.inverse()*(samp-mu);
	float prob = (wi*1/(2*M_PI*sqrt(sigma.determinant()))*exp(inExp));
	//double inExp = ((samp-mu).transpose())*sigma.inverse()*(samp-mu);
	//float prob = exp(-0.5*fabs(inExp)) / sqrt(pow((2*M_PI),2) 
	//	* (fabs(sigma.determinant()) + 0.001));
		
	return prob;
}







/*
double GMM_sampling::getGMMCost(double rx, double ry, bool flag) 
{  

//coordinates in base_link frame, transform to odom
	double x_out=0.0, y_out=0.0, h_out=0.0;
	double rob_x = 0.0;
	double rob_y = 0.0;

	geometry_msgs::Pose pose_out;

	if(flag) {
		if(!transformPoseTo(rx, ry, 0.0, std::string("odom"), std::string("base_link"), pose_out))
		{ //)x_out, y_out, h_out
			printf("Error transforming robot position from odom to base_link\n");
			return 0.0;
		}
		rob_x = pose_out.position.x;
		rob_y = pose_out.position.y;
	} else {

		rob_x = rx;
		rob_y = ry;

	}	
	
// read the person list (thread safe)
	std::vector<upo_msgs::PersonPoseUPO> people_aux;
	callbackMutex.lock();
	people_aux = people_;
	callbackMutex.unlock();		

	// Cost 0.0 if no persons detected
	if(people_aux.size() == 0) 
	{
		return 0.0;
  	}
  	
	double cost = 0.0;

  	for (std::vector<upo_msgs::PersonPoseUPO>::iterator it = people_aux.begin(); it != people_aux.end(); ++it)
  	{
		//People in odom coordinates
		double ph = tf::getYaw(it->orientation);
		double px = it->position.x;
		double py = it->position.y;
		bool rol = (it->id == 3); // GMM_IT if id == 3; GMM_SO other
		//printf("px: %f, py: %f\n", px,py);
	
		cost += gmm_cost(px, py, ph, rob_x, rob_y, rol);

  	}
  	cost = cost/people_aux.size();//2.0;
    
		return cost; 
}

double GMM_sampling::gmm_cost(double x_per, double y_per, double phi, double x_rob, double y_rob, bool rol)
{

	double z = 0.0;	
	double z2 = 0.0;
	double z3 = 0.0;
	double n = 0.0;
	int k = 0;

	// Calculate the features:
	double dist, theta, ctheta;

	dist = sqrt((x_rob-x_per)*(x_rob-x_per)+(y_rob-y_per)*(y_rob-y_per));

	double x =  (x_rob-x_per)*cos(phi) + (y_rob-y_per)*sin(phi);
	double y = -(x_rob-x_per)*sin(phi) + (y_rob-y_per)*cos(phi);
	
	theta = atan2(y,x);

	//approaching 
	if(gmm_mode_==2 || gmm_mode_==3)
	{
		//Calculate Total Density to Normalize
		pdf_norm_term(calculatedTPDF());
		//-------------------------------------
		for(std::vector<gmodel>::const_iterator it = gmodels_approach_.begin(); it != gmodels_approach_.end(); ++it )
		{
			double wi = it->weight_approach;
			Eigen::Matrix2d Sigma;
			Eigen::Vector2d mu,X_;
			Sigma << it->covar1_approach, it->covar12_approach, it->covar12_approach, it->covar2_approach;
			mu << it->mean1_approach, it->mean2_approach;
			X_ << dist, theta;
			// double det = it->covar1_approach*it->covar2_approach - it->covar12_approach*it->covar12_approach;
			// double inExp = -0.5*(1/det)*(pow(dist-it->mean1_approach,2)*it->covar2_approach+pow(theta-it->mean2_approach,2)*it->covar1_approach-2*it->covar12_approach*(theta-it->mean2_approach)*(dist-it->mean1_approach));
			// z += wi*1/(2*M_PI*sqrt(det))*exp(inExp);
			double inExp = -0.5*((X_-mu).transpose())*Sigma.inverse()*(X_-mu);
			z += (wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp));///TPDF_k.at(k);
			++k;
		}
	}
	//avoiding 
	else if(gmm_mode_==1 || gmm_mode_==3)
	{
		//Calculate Total Density to Normalize
		pdf_norm_term(calculatedTPDF());
		//-------------------------------------
		for(std::vector<gmodel>::const_iterator it = gmodels_avoid_.begin(); it != gmodels_avoid_.end(); ++it )
		{
			double wi = it->weight_avoid;
			Eigen::Matrix2d Sigma;
			Eigen::Vector2d mu,X_;
			Sigma << it->covar1_avoid, it->covar12_avoid, it->covar12_avoid, it->covar2_avoid;
			mu << it->mean1_avoid, it->mean2_avoid;
			X_ << dist, theta;
			// std::cout<<"Sigma: "<<Sigma<<std::endl;
			// std::cout<<"Mu: "<<mu<<std::endl;
			// std::cout<<"X_: "<<X_<<std::endl;
			// double det = it->covar1_avoid*it->covar2_avoid - it->covar12_avoid*it->covar12_avoid;
			// double inExp = -0.5*(1/det)*(pow(dist-it->mean1_avoid,2)*it->covar2_avoid+pow(theta-it->mean2_avoid,2)*it->covar1_avoid-2*it->covar12_avoid*(theta-it->mean2_avoid)*(dist-it->mean1_avoid));
			// z += wi*1/(2*M_PI*sqrt(det))*exp(inExp);
			double inExp = -0.5*((X_-mu).transpose())*Sigma.inverse()*(X_-mu);
			z += wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp);///TPDF_k.at(k);
			z2 += wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp)/TPDF_k.at(k);//exp(inExp);
			z3 += exp(inExp);
			//std::cout<<"k = "<< k;
			++k;
			//printf("PDF_%i: %f\n",k,(wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp))/TPDF_k.at(k));
			//printf("TPDF_K at k = %i : ",k);
			//std::cout<<TPDF_k.at(k-1)<<std::endl;
		}
	}
	unsigned int nM = getNumModels();

	//printf("Total PDF: %f\n", n);
	//printf("PDF: %f\n",z);
	//printf("PDF normalized: %f\n",z/n);
	
	//std::ofstream file_1("/home/rafa/Escritorio/cost1.txt", std::ofstream::app);
	//std::ofstream file_2("/home/rafa/Escritorio/cost2.txt", std::ofstream::app);
	//std::ofstream file_3("/home/rafa/Escritorio/cost3.txt", std::ofstream::app);

    //file_1 <<  1-z2/nM  << std::endl;     
    double zz = z < 0.001? 0.001 : z; 
    //file_2 << (1/zz)/1000 << std::endl; 
    //file_3 << 1-z3/nM  << std::endl; 
	
	//file_1.close();
	//file_2.close();
	//file_3.close();

	return (1/zz)/1000; //1-z2/nM;// 1/z;//1-z;
	//return 1/z;
}

void GMM_sampling::pdf_norm_term(bool flag){

	double n = 0.0;
	double t = 0.0;
	if(!flag)
	{
		if(gmm_mode_==2){
			for(std::vector<gmodel>::const_iterator it = gmodels_approach_.begin(); it != gmodels_approach_.end(); ++it )
			{
				double wi = it->weight_approach;
				Eigen::Matrix2d Sigma;
				Eigen::Vector2d mu,X_;
				Sigma << it->covar1_approach, it->covar12_approach, it->covar12_approach, it->covar2_approach;
				mu << it->mean1_approach, it->mean2_approach;
				// for(double dt=-20.0;dt<20.0;dt=dt+0.2)
				// {
				// 	for(double th=-M_PI;th<M_PI;th=th+0.1)
				// 	{
				// 		X_ << dt, th;
				// 		double inExp = -0.5*((X_-mu).transpose())*Sigma.inverse()*(X_-mu);
				// 		n += wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp);
				// 	}
				// }

				n += wi*1/(2*M_PI*sqrt(Sigma.determinant()));
				t += n;
				TPDF_k.push_back(n);
				n = 0.0;
			}	
			TPDF_k.push_back(t);
			updateTPDF();

		}
		if(gmm_mode_==1){
			for(std::vector<gmodel>::const_iterator it = gmodels_avoid_.begin(); it != gmodels_avoid_.end(); ++it )
			{
				double wi = it->weight_avoid;
				Eigen::Matrix2d Sigma;
				Eigen::Vector2d mu,X_;
				Sigma << it->covar1_avoid, it->covar12_avoid, it->covar12_avoid, it->covar2_avoid;
				mu << it->mean1_avoid, it->mean2_avoid;
				// for(double dt=-20.0;dt<20.0;dt=dt+0.2)
				// {
				// 	for(double th=-M_PI;th<M_PI;th=th+0.1)
				// 	{
				// 		X_ << dt, th;
				// 		double inExp = -0.5*((X_-mu).transpose())*Sigma.inverse()*(X_-mu);
				// 		n += wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp);
				// 	}
				// }

				n += wi*1/(2*M_PI*sqrt(Sigma.determinant()));
				t += n;
				TPDF_k.push_back(n);	
				n = 0.0;			
			}
			TPDF_k.push_back(t);
			updateTPDF();
		}
	}
}


double GMM_sampling::getGMMCostXY(double rx, double ry, bool flag) 
{  

//coordinates in base_link frame, transform to odom
	double x_out=0.0, y_out=0.0, h_out=0.0;
	double rob_x = 0.0;
	double rob_y = 0.0;

	geometry_msgs::Pose pose_out;

	if(flag) {
		if(!transformPoseTo(rx, ry, 0.0, std::string("base_link"), std::string("map"), pose_out))
		{ 
			printf("Error transforming robot position from odom to base_link\n");
			return 0.0;
		}
		rob_x = pose_out.position.x;
		rob_y = pose_out.position.y;
	} else {

		rob_x = rx;
		rob_y = ry;

	}	
	 	
	double cost = 0.0;

	cost += gmm_costXY(rob_x, rob_y, true);

  	    
		return cost; 
}

double GMM_sampling::gmm_costXY(double x_rob, double y_rob, bool rol)
{

	double z = 0.0;	
	double n = 0.0;
	
	//approaching 
	if(gmm_mode_==2 || gmm_mode_==3)
	{

		//-------------------------------------
		for(std::vector<gmodel>::const_iterator it = gmodels_approach_.begin(); it != gmodels_approach_.end(); ++it )
		{
			double wi = it->weight_approach;
			Eigen::Matrix2d Sigma;
			Eigen::Vector2d mu,X_;
			Sigma << it->covar1_approach, it->covar12_approach, it->covar12_approach, it->covar2_approach;
			mu << it->mean1_approach, it->mean2_approach;
			X_ << x_rob, y_rob;

			double inExp = -0.5*((X_-mu).transpose())*Sigma.inverse()*(X_-mu);
			z += (wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp));
		}
	}
	//avoiding 
	else if(gmm_mode_==1 || gmm_mode_==3)
	{

		//-------------------------------------
		for(std::vector<gmodel>::const_iterator it = gmodels_avoid_.begin(); it != gmodels_avoid_.end(); ++it )
		{
			double wi = it->weight_avoid;
			Eigen::Matrix2d Sigma;
			Eigen::Vector2d mu,X_;
			Sigma << it->covar1_avoid, it->covar12_avoid, it->covar12_avoid, it->covar2_avoid;
			mu << it->mean1_avoid, it->mean2_avoid;
			X_ << x_rob, y_rob;
			
			double inExp = -0.5*((X_-mu).transpose())*Sigma.inverse()*(X_-mu);
			z += wi*1/(2*M_PI*sqrt(Sigma.determinant()))*exp(inExp);
		}
	}
	unsigned int nM = getNumModels();
   
    double zz = z < 0.001? 0.001 : z; 

	return (1/zz)/1000; 
	
}*/



//}; //namespace gmm_samp
