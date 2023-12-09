#include "Posterior.h"
#include <eigen3/Eigen/Dense>

struct normal_random_variable{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](double x) { return dist(gen); });
    }
};

//constructor : uniform distribution
Posterior::Posterior(){
    sum = 1;
    max = 1/((float)x_nbins*(float)y_nbins*(float)Q_nbins*(float)a_y_nbins*(float)b_y_nbins);
    dirac_flag = 0;
    //pdf
    int i=0,j=0,l=0,m=0,n=0;
    for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
					for(n=0; n<b_y_nbins; n++){
							pdf[i][j][l][m][n] = 1/((float)x_nbins*(float)y_nbins*(float)Q_nbins*(float)a_y_nbins*(float)b_y_nbins);
							entropy += pdf[i][j][l][m][n] * log(pdf[i][j][l][m][n]);
						}
					}
				}
			//}
		}
	}
    entropy = (-1)*entropy;
    max_a_posteriori = {(float)(Q_max-Q_min)/(float)(2*Q_nbins) + Q_min,
    					{(float)(source_x_max-source_x_min)/(float)(2*x_nbins) + source_x_min, 
    					(float)(source_y_max-source_y_min)/(float)(2*y_nbins) + source_y_min},
    					(float)(a_y_max-a_y_min)/(float)(2*a_y_nbins) + a_y_min,
    					(float)(b_y_max-b_y_min)/(float)(2*b_y_nbins) + b_y_min};
    expected_value = {(Q_max-Q_min)/2 +Q_min, {(source_x_max-source_x_min)/2 +source_x_min,(source_y_max-source_y_min)/2 +source_y_min},
					  (a_y_max-a_y_min)/2 +a_y_min, (b_y_max-b_y_min)/2 +b_y_min};//, (a_y_max-a_y_min)/2, (b_y_max-b_y_min)/2};//(source_z_max-source_z_min)/2},
}

//constructor : empty distribution (all elements 0)
Posterior::Posterior(int zero){
    sum = 0;
    max = 0;
    dirac_flag = 0;
    //pdf
    int i=0,j=0,l=0,m=0,n=0;
    for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							pdf[i][j][l][m][n] = 0;
						}
					}
				}
		//}
		}
	}
    entropy = 1;
    max_a_posteriori = {(float)(Q_max-Q_min)/(float)(2*Q_nbins) + Q_min,
    					{(float)(source_x_max-source_x_min)/(float)(2*x_nbins) + source_x_min, 
    					(float)(source_y_max-source_y_min)/(float)(2*y_nbins) + source_y_min},
    					(float)(a_y_max-a_y_min)/(float)(2*a_y_nbins) + a_y_min,
    					(float)(b_y_max-b_y_min)/(float)(2*b_y_nbins) + b_y_min};
    expected_value = {(Q_max-Q_min)/2 +Q_min, {(source_x_max-source_x_min)/2 +source_x_min,(source_y_max-source_y_min)/2 +source_y_min},
						  (a_y_max-a_y_min)/2 +a_y_min, (b_y_max-b_y_min)/2 +b_y_min};//, (a_y_max-a_y_min)/2, (b_y_max-b_y_min)/2};//(source_z_max-source_z_min)/2},
}

//update the posterior pdf with the value given by sample paramter set
void Posterior::update(double value, struct parameters_set sample){ 
	pdf [(int)((sample.source_position[0]-source_x_min) / ((float)(source_x_max-source_x_min)/(float)(x_nbins)))]
		[(int)((sample.source_position[1]-source_y_min) / ((float)(source_y_max-source_y_min)/(float)(y_nbins)))]
		//[(int)(sample.source_position[2] / ((float)(source_z_max-source_z_min)/(float)(z_nbins)))]
		[(int)((sample.Q-Q_min) / ((float)(Q_max-Q_min)/(float)(Q_nbins)))]
		[(int)((sample.a_y-a_y_min) / ((float)(a_y_max-a_y_min)/(float)(a_y_nbins)))]
		[(int)((sample.b_y-b_y_min) / ((float)(b_y_max-b_y_min)/(float)(b_y_nbins)))] += value;
	sum += value;
}

//normalization + entropy + expected value + max a posterior (value and parameter set) using only the pdf
void Posterior::process(){ 
	int i=0,j=0,l=0,m=0,n=0;
	max = 0;
	entropy = 0; 
	expected_value = (struct parameters_set) {0,{0,0},0,0};
	
	for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							//normalize
							pdf[i][j][l][m][n] /= sum;
							//entropy
							if(pdf[i][j][l][m][n] > 0) 
								entropy += pdf[i][j][l][m][n] * log(pdf[i][j][l][m][n]);
							//expected value
							expected_value.source_position[0] += ((float)((source_x_max-source_x_min)*(2*i+1))/(float)(2*x_nbins) + source_x_min) * pdf[i][j][l][m][n];
							expected_value.source_position[1] += ((float)((source_y_max-source_y_min)*(2*j+1))/(float)(2*y_nbins) + source_y_min) * pdf[i][j][l][m][n];
							//expected_value.source_position[2] += ((float)((source_z_max-source_z_min)*(2*k+1))/(float)(2*z_nbins) + source_z_min) * pdf[i][j][k][l][m][n];
							expected_value.Q += ((float)((Q_max-Q_min)*(2*l+1))/(float)(2*Q_nbins) + Q_min) * pdf[i][j][l][m][n];
							expected_value.a_y += ((float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min) * pdf[i][j][l][m][n];
							expected_value.b_y += ((float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min) * pdf[i][j][l][m][n];
							//expected_value.a_z += ((float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min) * pdf[i][j][k][l][m][n];
							//expected_value.b_z += ((float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min) * pdf[i][j][k][l][m][n];
							//max a posteriori
							if(pdf[i][j][l][m][n] > max){
								max = pdf[i][j][l][m][n];
								max_a_posteriori.source_position[0] = (float)((source_x_max-source_x_min)*(2*i+1))/(float)(2*x_nbins) + source_x_min;
								max_a_posteriori.source_position[1] = (float)((source_y_max-source_y_min)*(2*j+1))/(float)(2*y_nbins) + source_y_min;
								//max_a_posteriori.source_position[2] = (float)((source_z_max-source_z_min)*(2*k+1))/(float)(2*z_nbins) + source_z_min;
								max_a_posteriori.Q = (float)((Q_max-Q_min)*(2*l+1))/(float)(2*Q_nbins) + Q_min;
								max_a_posteriori.a_y = (float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min;
								max_a_posteriori.b_y = (float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min;
								//max_a_posteriori.a_z = (float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min;
								//max_a_posteriori.b_z = (float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min;
							}
						}
					}
				}
			//}
		}
	}
	entropy = -1 * entropy;
	
	//save in log file
	//log_expected_a_posteriori();
	if(VERBOSE) printf("expected value (new method): Q=%f @(%f %f) sigma y : %f %f  \n", expected_value.Q, 
		expected_value.source_position[0], expected_value.source_position[1],
		expected_value.a_y, expected_value.b_y);
	//log_max_a_posteriori();
	if(VERBOSE) printf("max_a_posteriori (new method): Q=%f @(%f %f) sigma y : %f %f \n", max_a_posteriori.Q, 
		max_a_posteriori.source_position[0], max_a_posteriori.source_position[1],
		max_a_posteriori.a_y, max_a_posteriori.b_y);
	//entropy_logfile = fopen(entropy_logfile_name, "a");
	//fprintf(entropy_logfile, "%f ", entropy_posterior);
	//fclose(entropy_logfile);
}

//efficient sampling through MCMC
void Posterior::MCMC_evaluation(struct history history){
	int i=0, count = simulation_parameters.MCMC_iteration_num; //number of iterations
	double p_acceptance = 0, posterior_value = 0;
	struct parameters_set sample;
	struct parameters_set candidate;
	int acceptance_rate = 0;

	//initial values (maybe randomize later)
	struct parameters_set last_sample = expected_value; 
	sample = last_sample;
	
	//proposal distribution : 5D Gaussian
	Eigen::VectorXd mean(5);
	Eigen::MatrixXd covar(5,5);
	covar << 4  , 0  , 0   , 0      , 0   ,
			 0  , 4  , 0   , 0      , 0   ,
			 0  , 0  , 0.1 , 0      , 0   ,
			 0  , 0  , 0   , 0.00001, 0   ,
			 0  , 0  , 0   , 0      , 0.0005;
	
	//error detection
	if(prior(sample) == 0 || likelihood(history, sample) == 0){
		if(VERBOSE) printf("done ---- \n");
		dirac_flag = 1;
		return;
	}

	//sampling
	for(i=0; i<count; i++){
		//draw a point to jump to from a normal dist centered on the previous value
		mean << sample.Q, sample.source_position[0], sample.source_position[1], sample.a_y, sample.b_y; // Set the mean
		normal_random_variable random_sample{mean, covar};
		candidate = parameters_set{random_sample()(0), {random_sample()(1), random_sample()(2)},random_sample()(3), random_sample()(4)};
		//calculate the acceptance chance
		p_acceptance = (prior(candidate) * likelihood(history, candidate)) /
					   (prior(sample) * likelihood(history, sample));
		if(p_acceptance > 1) p_acceptance = 1;
		//accept if a random value higher than the acceptance chance
		if(randu() < p_acceptance){
			//set the next value
			sample = candidate;
			acceptance_rate++;
		}else{
			continue;
		}
		//use the new sample to evaluate the posterior pdf
		posterior_value = prior(sample) * likelihood(history, sample);
		update(posterior_value, sample);
	}
	if(acceptance_rate == 0){
		if(VERBOSE) printf("prior : %f, likelihood : %f, p_acceptance: %f \n", prior(sample), likelihood(history, sample),p_acceptance);
	}
	last_sample = sample;
	if(FSM.state == STATE_ESTIMATING){ 
		if(VERBOSE) printf("acceptance rate : %f  \n", (float)acceptance_rate/count);
		//fclose(tmp_file);
	}
}

//exhaustive evaluation of all the values of the posterior pdf
void Posterior::exhaustive_evaluation(struct history history){
	int i=0,j=0,l=0,m=0,n=0;
	struct parameters_set sample;

	for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							sample.source_position[0] = ((float)((source_x_max-source_x_min)*(2*i+1))/(float)(2*x_nbins) + source_x_min);
							sample.source_position[1] = ((float)((source_y_max-source_y_min)*(2*j+1))/(float)(2*y_nbins) + source_y_min);
							//sample.source_position[2] += ((float)((source_z_max-source_z_min)*(2*k+1))/(float)(2*z_nbins) + source_z_min);
							sample.Q = ((float)((Q_max-Q_min)*(2*l+1))/(float)(2*Q_nbins) + Q_min);
							sample.a_y = ((float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min);
							sample.b_y = ((float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min);
							//sample.a_z += ((float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min);
							//sample.b_z += ((float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min);
							pdf[i][j][l][m][n] = prior(sample) * likelihood(history, sample);
							//if(likelihood(measurement_history, position_history, history_size, sample) != 0)
							//	printf("exhaustive evaluation works\n");
							sum += pdf[i][j][l][m][n];
						}
					}
				}
			//}
		}
	}
	if(VERBOSE) printf("sum : %f\n", sum);
}

//relative entropy calculation between one posterior pdf and another P
double Posterior::Kullback_Leibler_divergence(Posterior P){
	double DKL = 0;
	int i=0, j=0, l=0, m=0, n=0;

	for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							if(pdf[i][j][l][m][n] > 0 && P.pdf[i][j][l][m][n] > 0){
								//DKL += ((P.pdf[i][j][l][m][n]+0.00000000000001)/P.sum) * log(((P.pdf[i][j][l][m][n]+0.00000000000001)/P.sum) / (pdf[i][j][l][m][n]+0.00000000000001));
								DKL += (P.pdf[i][j][l][m][n]/P.sum) * log((P.pdf[i][j][l][m][n]/P.sum) / (pdf[i][j][l][m][n]));
								// P is not normalized, but its sum was calculated and updated
							}
						}
					}
				}
			//}
		}
	}

	return DKL;
}

void Posterior::XYmarginal_distance_combined(double current_position[], double ratio, double * best_point){
	int i=0,j=0,l=0,m=0,n=0;
	double distance = 0, marginal = 0, value = 0;
	double tmp_position[2] = {0};
	double inv_dist [x_nbins][y_nbins] = {{0}};
	//FILE* ditance_cost_logfile = fopen("../../../Results/current/101/ditance_cost.csv", "w");
	//static double dist_threshold = sqrt( pow((float)(source_x_max-source_x_min)/(float)(x_nbins),2) + pow((float)(source_y_max-source_y_min)/(float)(y_nbins),2) ) / 2.0;
	int current_cell[2] = {	(int)( (current_position[0]-source_x_min) / ((float)(source_x_max-source_x_min)/(float)x_nbins) ), 
							(int)( (current_position[1]-source_y_min) / ((float)(source_y_max-source_y_min)/(float)y_nbins) )};
	double sum_inv_distance = 0;

	for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			// check if we are in the current cell
			if(i == current_cell[0] && j == current_cell[1]){
				inv_dist[i][j] = 0;
				continue;
			}
			tmp_position[0] = (float)((source_x_max-source_x_min)*(2*i+1))/(float)(2*x_nbins) + source_x_min;
			tmp_position[1] = (float)((source_y_max-source_y_min)*(2*j+1))/(float)(2*y_nbins) + source_y_min;
			inv_dist[i][j] = 1 / (sqrt(pow(current_position[0]-tmp_position[0], 2) + pow(current_position[1]-tmp_position[1], 2)));
			sum_inv_distance += inv_dist[i][j];
		}
	}


	for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				tmp_position[0] = (float)((source_x_max-source_x_min)*(2*i+1))/(float)(2*x_nbins) + source_x_min;
				tmp_position[1] = (float)((source_y_max-source_y_min)*(2*j+1))/(float)(2*y_nbins) + source_y_min;
				//distance = sqrt(pow(current_position[0]-tmp_position[0], 2) + pow(current_position[1]-tmp_position[1], 2));
				//the value for the cell we are currently on is 0!
				/*if(distance < dist_threshold){
					//tmp_value = 0;
					fprintf(ditance_cost_logfile, "%f %f %f\n", (distance), 0.0, 0.0);
					continue;
				}*/
				//printf("current_position (%f, %f), tmp_position (%f, %f), dist %f\n", current_position[0], current_position[1], tmp_position[0], tmp_position[1], distance);
				marginal = 0;
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							marginal += pdf[i][j][l][m][n];
						}
					}
				}
				double tmp_value = (ratio * marginal) + ((1-ratio) * (inv_dist[i][j]/sum_inv_distance));//marginal - ratio*(distance/MAX_DISTANCE);
				//printf("distance %f Marginal %f tmp value %f\n", distance/MAX_DISTANCE, marginal, tmp_value);
				//fprintf(ditance_cost_logfile, "%f %f %f\n", inv_dist[i][j]/sum_inv_distance, marginal, tmp_value);
				if(tmp_value > value ){
					value = tmp_value;
					//printf("best point %f %f\n", tmp_position[0], tmp_position[1]);
					memcpy(best_point, tmp_position, sizeof(double)*2);
				}
			//}
		}
	}
	//printf("best point %f %f\n", best_point[0], best_point[1]);
	//fclose(ditance_cost_logfile);
}

/*
void Posterior::XYmarginal_distance_combined(double current_position[], double ratio, double * best_point){
	int i=0,j=0,l=0,m=0,n=0;
	double distance = 0, marginal = 0, value = 0;
	double tmp_position[2] = {0};
	//FILE* ditance_cost_logfile = fopen("../../../Results/current/101/ditance_cost.csv", "w");
	static double dist_threshold = sqrt( pow((float)(source_x_max-source_x_min)/(float)(x_nbins),2) + pow((float)(source_y_max-source_y_min)/(float)(y_nbins),2) ) / 2.0;

	for(i=0; i<x_nbins; i++){
		for(j=0; j<y_nbins; j++){
			//for(k=0; k<z_nbins; k++){
				tmp_position[0] = (float)((source_x_max-source_x_min)*(2*i+1))/(float)((2*x_nbins) + source_x_min);
				tmp_position[1] = (float)((source_y_max-source_y_min)*(2*j+1))/(float)((2*y_nbins) + source_y_min);
				distance = sqrt(pow(current_position[0]-tmp_position[0], 2) + pow(current_position[1]-tmp_position[1], 2));
				//the value for the cell we are currently on is 0!
				if(distance < dist_threshold){
					//tmp_value = 0;
					continue;
				}
				//printf("current_position (%f, %f), tmp_position (%f, %f), dist %f\n", current_position[0], current_position[1], tmp_position[0], tmp_position[1], distance);
				marginal = 0;
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							marginal += pdf[i][j][l][m][n];
						}
					}
				}
				double tmp_value = (ratio * marginal) + ((1-ratio)/(distance/MAX_DISTANCE));//marginal - ratio*(distance/MAX_DISTANCE);
				//printf("distance %f Marginal %f tmp value %f\n", distance/MAX_DISTANCE, marginal, tmp_value);
				//fprintf(ditance_cost_logfile, "%f %f %f\n", (distance/MAX_DISTANCE), marginal, tmp_value);
				if(tmp_value > value){
					value = tmp_value;
					//printf("best point %f %f\n", tmp_position[0], tmp_position[1]);
					memcpy(best_point, tmp_position, sizeof(double)*2);
				}
			//}
		}
	}
	//printf("best point %f %f\n", best_point[0], best_point[1]);
	//fclose(ditance_cost_logfile);
}
*/

//log the posterior pdf
void Posterior::log_pdf(char * file_name, char writing_mode){
	
	FILE * posterior_pdf_3D = fopen(file_name, &writing_mode);
	
	for (int i=0; i<x_nbins; i++){
		for (int j=0; j<y_nbins; j++){
			//for (int k=0; k<z_nbins; k++){
				for (int l=0; l<Q_nbins; l++){
					for(int m=0; m<a_y_nbins; m++){
						for(int n=0; n<b_y_nbins; n++){
							fprintf(posterior_pdf_3D, "%f ", pdf[i][j][l][m][n]);
						}
					}
				}
			//}
		}
	}
	fprintf(posterior_pdf_3D, "\n");
	fclose(posterior_pdf_3D);
}
