/*
* File:          controller_STE_multi-robot.cpp
* Date:          Jan 2019
* Description:   The goal is to estimate the source terms including the position
* Author:        Faezeh Rahbar
* Modifications: 26/01/2019
*/

//custom libraries
#include <fstream>
#include <iostream>
#include "controller_STE_multi-robot.h"
//#include "nvwa/debug_new.h"
#include "Posterior.h"
#include "Message.h"
#include "webots_functions.h"
//#include "aStar.h"
#include "visilibity.hpp"
#include <webots/supervisor.h>
#include "CellMeasurements.h"
#include "GasDispersionMap.h"

using namespace std;

//Global Variables

// log files
char folder_name[256];
FILE* goalposition_logfile;
FILE* measurements_logfile;
FILE* posterior_logfile;
FILE* posteriorpdf_logfile;
FILE* expected_a_posteriori_logfile;
FILE* likelihood_logfile;
FILE* measurement_history_logfile;
FILE* position_history_logfile;
FILE* max_a_posteriori_logfile;
FILE* posterior_pdf_3D;
FILE* entropy_logfile;
FILE* tmp_file;
FILE* config_logfile;
FILE* trajectory_logfile;
FILE* gas_concentration_map_logfile;
FILE* gas_concentration_variance_map_logfile;
char goalposition_logfile_name[512];
char measurements_logfile_name[512];
char posterior_logfile_name[512];
char posteriorpdf_logfile_name[512];
char expected_a_posteriori_logfile_name[512];
char max_a_posteriori_logfile_name[512];
char likelihood_logfile_name[512];
char measurement_history_logfile_name[512];
char position_history_logfile_name[512];
char posterior_pdf_3D_name[512];
char entropy_logfile_name[512];
char config_logfile_name[512];
char trajectory_logfile_name[512];
char gas_concentration_map_logfile_name[512];
char gas_concentration_variance_map_logfile_name[512];

// finite state machine
struct FSM FSM;

// robot's measurements
SampleBuffer sampleBuffer(ODOR_FILTER_LENGTH);

//position in m
Position goal_position;
Position current_position;
double speed[2] = {0};
// robot's position and measurement history
struct history history;
struct history sample_in_motion;
vector<double> ground_truth_in_motion;

int sample_counter = 0;
//plume model
DeepCFDPlumeModel2D plume_model;

// vector of the cell_measurements
vector<CellMeasurements> cell_measurements_list; 
// gas concentration variance map
double gas_concentration_variance_map[x_nbins*y_nbins] = {0};

// current posterior distribution 
Posterior posterior(&plume_model);

//robot's ID
int ID;

//message structure
struct message message;

//list of other robots
struct friends friends[MAX_NB_ROBOTS];

int n_time_step = 0;

struct simulation_parameters simulation_parameters;

double previous_sensor_reading = 0.0;

// gas map
// param: sigma, gamma, scaling_param, wind_speed, rco
GasMap2D gas_map(0.2, 2, 1, 0.75, 0.3);

// obstacles definitions
int nbr_obs = 4;
float margin = 0.16;
float ** obstacles = NULL;
// float obstacles[][4] = {{5-margin, static_cast<float>(5.5+margin), static_cast<float>(2.5-margin), 4+margin}, 
							// {5-margin, 8+margin, static_cast<float>(2.5-margin), 3+margin}, 
							// {9-margin, 12+margin, 1-margin, static_cast<float>(1.5+margin)}, 
							// {static_cast<float>(11.5-margin), 12+margin, 0-margin, static_cast<float>(1.5+margin)}};
std::list<Position> path;
std::list<Position>::iterator i;

std::string environment_file("../../data/test_map.environment");
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

// =========================== FUNCTIONS ================================ //
//random value sampled from a nromal distribution
double randn(double mu, double sigma){
	//taken from https://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;
	
	if(call == 1){
    	call = !call;
      	return (mu + sigma * (double) X2);
    }

	do{
      	U1 = -1 + ((double) rand () / RAND_MAX) * 2;
    	U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      	W = pow (U1, 2) + pow (U2, 2);
    } while (W >= 1 || W == 0);
 
  	mult = sqrt ((-2 * log (W)) / W);
  	X1 = U1 * mult;
  	X2 = U2 * mult;
 
  	call = !call;
 
  	return (mu + sigma * (double) X1);
}

//random value sampled from a uniform distribution
double randu(){
	//random variable between 0 and 1
    return (double)rand() / (double)RAND_MAX ;
}

//get the current time in timestamp (ms)
long long current_timestamp(){
	struct timeval te; 
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

//Euclidean distance between two points
double pt_distance(Position p1, Position p2){
	return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));
}

//min distance between two robots position (p1 and p2) and their goals (g1 and g2)
double min_distance(Position p1, Position g1, Position p2, Position g2){
	double d_original = pt_distance(p1, g1) + pt_distance(p2, g2);
	double d_swap = pt_distance(p1, g2) + pt_distance(p2, g1);
	if(d_swap < d_original)
		return d_swap;
	else
		return d_original;
}

//Euclidean distance between current position to the goal position
double dist_to_goal(Position goal){
    double dx, dy;
    dx = goal.x - current_position.x;
    dy = goal.y - current_position.y;
    return sqrt(dx*dx + dy*dy);
}

//calculate wheel speeds to go toward goal position
void go_to_goal(Position goal){
	// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, goal.x,goal.y);
	//vector to the goal
    double movement[2];
    movement[0] = (goal.x - current_position.x);
    movement[1] = (goal.y - current_position.y);
    //printf("movement : %f %f \n", movement[0], movement[1]);

    //vector to the goal in robot coordinates
    float x =  movement[0]*cosf(current_position.heading) + movement[1]*sinf(current_position.heading); // x in robot coordinates
   
    float z = -movement[0]*sinf(current_position.heading) + movement[1]*cosf(current_position.heading); // z in robot coordinates
 
    float bearing = atan2(z, x) - PI/2;    //[-PI, PI]
    //printf("bearing : %f \n", bearing/PI*180);

    //angular and linear velocity
    float w = 1 * sinf(bearing);
    float u = 0.2 * cosf(bearing);
    // printf("linear v : %f; angular w : %f \n", u, w);
    // wheels speed
	speed[0] = (u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;   //10 is a speed-up factor
	speed[1] = (u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	//printf("speed : %f %f \n", speed[0], speed[1]);
}

//show the measurement history on screen
void history_print(struct history history){
	printf("_____ Sample History _____ \n");
	for(int i=0; i<history.size; i++){
		printf("%d : %f @ (%f, %f)\n", i, history.measurement[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list(){
	// open file
	std::ifstream infile("../../data/obstacle_list.txt");
	// first line is the number of obstacles
	infile >> nbr_obs;
	printf("There is %d obsatcle in total:\n",nbr_obs);
	// allocate memory for obstacle list array
	obstacles = new float*[nbr_obs];
	//read all obstacles now
	for(int i = 0; i < nbr_obs; i++){
		obstacles[i] = new float[4];
		infile >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3];
		obstacles[i][0] -= margin;
		obstacles[i][1] -= margin;
		obstacles[i][2] += margin;
		obstacles[i][3] += margin;
	}

	for(int i = 0; i < nbr_obs; i++){
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}
}

// ===================== LOG FILES FUNCTIONS ========================== //

void log_position(){
	double time = wb_robot_get_time();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(){
	//int i = 0;
	double time = wb_robot_get_time();
	measurements_logfile = fopen(measurements_logfile_name, "a");
	fprintf(measurements_logfile,"%d, %f, %f, %f\n", FSM.n_cycles, sampleBuffer.average_wind_intensity, sampleBuffer.average_concentration, time);
	//for (i = 0; i < ODOR_FILTER_LENGTH; i++)
	//	fprintf(measurements_logfile,"%f, ", measurements.odor_sample_memory.buffer[i]);
	//fprintf(measurements_logfile,"\n");
	fclose(measurements_logfile);
}

void log_expected_a_posteriori(){
	expected_a_posteriori_logfile = fopen(expected_a_posteriori_logfile_name, "a");
	fprintf(expected_a_posteriori_logfile,"%d, %f, %f, %f, %f, %f\n", FSM.n_cycles, 
		plume_model.expected_value[0], plume_model.expected_value[1], plume_model.expected_value[2], 
		plume_model.expected_value[3], plume_model.expected_value[4]);
	fclose(expected_a_posteriori_logfile);
}

void log_max_a_posteriori(){
	max_a_posteriori_logfile = fopen(max_a_posteriori_logfile_name, "a");
	fprintf(max_a_posteriori_logfile,"%d, %f, %f, %f, %f, %f\n", FSM.n_cycles,
		plume_model.max_a_post_value[0], plume_model.max_a_post_value[1], plume_model.max_a_post_value[2], 
		plume_model.max_a_post_value[3], plume_model.max_a_post_value[4]);
	fclose(max_a_posteriori_logfile);
}

void log_entropy(){
	double time = get_exp_time();
	entropy_logfile = fopen(entropy_logfile_name, "a");
	fprintf(entropy_logfile, "%f, %f\n", posterior.entropy, time);
	fclose(entropy_logfile);
}

void log_gas_concentration_variance_map(){
	gas_concentration_variance_map_logfile = fopen(gas_concentration_variance_map_logfile_name, "a");

	for(int i = 0; i < x_nbins * y_nbins; i++)
		fprintf(gas_concentration_variance_map_logfile, "%.9f, ", gas_concentration_variance_map[i]);

	fprintf(gas_concentration_variance_map_logfile, "\n");

	fclose(gas_concentration_variance_map_logfile);
}
void log_history(){
	// position
	//FILE* position_history_logfile;
	position_history_logfile = fopen(position_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++){
		fprintf(position_history_logfile,"%d, %f, %f, %f\n", i, history.position[i].x,history.position[i].y,history.position[i].heading);
	}
    fclose(position_history_logfile);
    // measurement
    //FILE * measurement_history_logfile;
    measurement_history_logfile = fopen(measurement_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++){
		fprintf(measurement_history_logfile,"%d, %f\n", i, history.measurement[i]);
	}
    fclose(measurement_history_logfile);
   
	//posterior 3D
	//posterior.log_pdf(posterior_pdf_3D_name, 'w');
}

void log_trajectory(Position p){
	double time = wb_robot_get_time();
	trajectory_logfile = fopen(trajectory_logfile_name, "a");
	fprintf(trajectory_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
	fclose(trajectory_logfile);
}


// ====================== ESTIMATIOM functions ========================= //

//ASSUMPTION : Using the history of measurements, but only one robot is operating
double likelihood(struct history history, double *parameters, PlumeModel *model){ 
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; //125;
	double sum_tmp = 0;
	if(sampleBuffer.average_wind_intensity == 0) printf("NO WIND, BYE BYE!\n");
	
	// for each sample, getting the model concentration and calculate the likelihood
	// double model_concentration = 0;
	// for (int i = 0; i < history.size; i++){
	// 	model_concentration = model->concentration(history.position[i], sampleBuffer.average_wind_intensity, parameters);
	// 	sum_tmp += pow(history.measurement[i] - model_concentration, 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
	// }

	// getting all model concentrations for all sample positions and then calculate the likelihood
	// printf("likelihood... history size %d\n", history.size);
	vector<float> model_c_avg(history.size); 
	vector<float> model_c_std(history.size); 
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);
	for (int i = 0; i < history.size; i++){
		sum_tmp += pow(history.measurement[i] - model_c_avg[i], 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
		//std::cout<<"size: "<< history.size <<"model_c_avg[i] "<<model_c_avg[i]<<std::endl;
	}
	return (exp(-0.5 * sum_tmp));
	
}

double likelihood_gas_hit(struct history history, double *parameters, PlumeModel *model){
	float sigmaE = 0.1;
	float sigmaD = 0; //125;
	double sum_tmp = 0;
	if(sampleBuffer.average_wind_intensity == 0) printf("NO WIND, BYE BYE!\n");
	
	// for each sample, getting the model concentration and calculate the likelihood
	// double model_concentration = 0;
	// for (int i = 0; i < history.size; i++){
	// 	model_concentration = model->concentration(history.position[i], sampleBuffer.average_wind_intensity, parameters);
	// 	sum_tmp += pow(history.measurement[i] - model_concentration, 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
	// }

	// getting all model concentrations for all sample positions and then calculate the likelihood
	// printf("likelihood... history size %d\n", history.size);
	vector<float> model_c_avg(history.size); 
	vector<float> model_c_std(history.size); 
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);
	// for (int i = 0; i < history.size; i++){
	// 	sum_tmp += pow(history.measurement[i] - model_c_avg[i], 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
	// 	//std::cout<<"size: "<< history.size <<"model_c_avg[i] "<<model_c_avg[i]<<std::endl;
	// }
	//return (exp(-0.5 * sum_tmp));

	// FIXME - temporary solution
	for (int i = 0; i < history.size; i++){
		if (history.measurement[i] > 200 && model_c_avg[i] > 200){
			 sum_tmp += sigmaE;
		}
		//std::cout<<"size: "<< history.size <<"model_c_avg[i] "<<model_c_avg[i]<<std::endl;
	}
	for (int i = 0; i < history.size; i++){
		if (history.measurement[i] < 200 && model_c_avg[i] < 200){
             sum_tmp += sigmaE;
		}
		//std::cout<<"size: "<< history.size <<"model_c_avg[i] "<<model_c_avg[i]<<std::endl;
	}
	sum_tmp = 1/sum_tmp;
    printf("likelihood: %f\n", exp(-0.5 * sum_tmp));
	
	return (exp(-0.5 * sum_tmp));
}

bool prior(double *parameters, int nbr_dimension, PlumeModel *model){
	int i = 0;
	bool output = 1;
	
	// verify if all the parameters are between the right intervals
	for(i = 0; i < nbr_dimension; i++)
		output = output && (parameters[i] < model->param_list[i].upper_limit && parameters[i] >= model->param_list[i].lower_limit);
	
	// verify the x and y elements of the parameters are not inside obstacles
	Position tmp_p(parameters[0], parameters[1], 0);
	if(tmp_p.is_in_obstacle_list(obstacles, nbr_obs) != -1){
		// printf("point %f %f is in obstacle\n", parameters[0], parameters[1]);
		output = 0;
	}

	//if(output == 0)
	//	printf("parameters: %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4]);
	
	return(output);
}


// ======================= NAVIGATION functions ========================= //
void random_navigation(){
	goal_position.x = (randu() * (XMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;  //position in m
    goal_position.y = (randu() * (YMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
    goal_position.heading = 0;//(randu() * (ZMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
}

bool position_outside_arena(Position p){
	bool outside = 0;
	if(p.x < float(XMIN) || p.x > float(XMAX)) outside = 1;
	if(p.y < float(YMIN) || p.y > float(YMAX)) outside = 1;
	//if(position[2] < ZMIN || position[2] > ZMAX) outside = 1;
	return outside;
}
/*
double posterior_predictive(double concentration, double position[][3]){ 
	int i=0, j=0, l=0,m=0,n=0;
	struct parameters_set source_param;
	struct history fake_history = {1, {{0}}, {0}};
	double predictive = 0;

	for (i=0; i<x_nbins; i++){
		for (j=0; j<y_nbins; j++){
			//for (k=0; k<z_nbins; k++){
				for (l=0; l<Q_nbins; l++){
					for(m=0; m<a_y_nbins; m++){
						for(n=0; n<b_y_nbins; n++){
							source_param = (struct parameters_set) {(float)((Q_max-Q_min)*(2*l+1))/(float)(2*Q_nbins) + Q_min, {
								(float)((source_x_max-source_x_min)*(2*i+1))/(float)(2*x_nbins) + source_x_min, 
								(float)((source_y_max-source_y_min)*(2*j+1))/(float)(2*y_nbins) + source_y_min}, 
								//(float)((source_z_max-source_z_min)*(2*k+1))/(float)(2*z_nbins) + source_z_min}, 
								//(float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min,
								//(float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min,
								(float)((a_y_max-a_y_min)*(2*m+1))/(float)(2*a_y_nbins) + a_y_min,
								(float)((b_y_max-b_y_min)*(2*n+1))/(float)(2*b_y_nbins) + b_y_min};
								memcpy(fake_history.position, position, sizeof *position); fake_history.measurement[0] = concentration;
							predictive += likelihood(fake_history, source_param) * posterior.pdf[i][j][l][m][n];

						}
					}
				}
			//}
		}
	}
	return predictive;
}
*/
void information_gain_navigation_approx(Position current_position){
	float p0 = 0.5; //50 cm
	double DKL=0, max_DKL=-100, max_meas=-100;
	//double position[1][3] = {{0}};
	Position tmp_position;
	struct history fake_history;
	fake_history.position = history.position;
	fake_history.measurement = history.measurement;
	fake_history.size = history.size;

	// add fake prediction for friends goal position, so we don't choose the same ones
	for(int i = 0; i < simulation_parameters.number_of_robots; i++){
		if(friends[i].ID == 0 || friends[i].ID  == ID) continue;
		fake_history.position[fake_history.size].copy_position(friends[i].goal);
		fake_history.measurement[fake_history.size] = plume_model.approx_concentration(friends[i].goal, sampleBuffer.average_wind_intensity);
		(fake_history.size)++;
	}
	// 4 candidate positions
    // Position candidates[4] = {	Position(current_position.x + p0, current_position.y, 0), 
	// 							Position(current_position.x - p0, current_position.y, 0),
	// 							Position(current_position.x, current_position.y + p0, 0),
	// 							Position(current_position.x, current_position.y - p0, 0)};
	// 8 candidate positions
	Position candidates[8] = {	Position(current_position.x + p0, current_position.y, 0),
								Position(current_position.x - p0, current_position.y, 0),
								Position(current_position.x, current_position.y + p0, 0),
								Position(current_position.x, current_position.y - p0, 0),
								Position(current_position.x + p0, current_position.y + p0, 0),
								Position(current_position.x - p0, current_position.y + p0, 0),
								Position(current_position.x + p0, current_position.y - p0, 0),
								Position(current_position.x - p0, current_position.y - p0, 0)};							
	for(auto &candidate_pos : candidates){
			// for each possible action
			tmp_position.x = candidate_pos.x;
			tmp_position.y = candidate_pos.y;
			tmp_position.heading = 0;
			// printf("position : %f %f\n", tmp_position.x, tmp_position.y);
			//printf("position : %f %f %f\n", j*p0 * cos(l*theta0) * sin(k*phi0),j*p0 * sin(l*theta0) * sin(k*phi0),j*p0 * cos(k*phi0));
			if(position_outside_arena(tmp_position)) continue;
			// //calculate the expected concentration on position
			// // FIXME: possible bug here, we are caluculating the futhyre posterior with accumulated sampling fake history
			// fake_history.measurement.push_back(plume_model.approx_concentration(tmp_position, sampleBuffer.average_wind_intensity));
			// std::cout<<"fake_history.measurement = "<<fake_history.measurement[fake_history.size]<<std::endl;
			// fake_history.position.push_back(tmp_position);
			// (fake_history.size)++;
			// // printf("fake_history.measurement[%d] = %f \n", fake_history.size-1, fake_history.measurement[fake_history.size-1]);
			// // memcpy(future_posterior.pdf, zero.pdf, sizeof zero.pdf); //memcpy(future_posterior_3D, zero_posterior, sizeof zero_posterior);
			// // future_posterior.sum = 0;
			// // TODO: Improve the way regenrate fake plume; at least use the same plume model for different positions
			// // TODO: Test the performance of the fake plume model
			// DeepCFDPlumeModel2D plume_model_fake;
			// Posterior future_posterior(&plume_model_fake, 0);
			// //future_posterior.copy_posterior(zero);
			// future_posterior.MCMC_evaluation(fake_history);
			// //calculate the expected information gain
			// DKL = posterior.Kullback_Leibler_divergence(future_posterior);
			// // printf("DKL: %f _______________________\n", DKL);
			// if(DKL > max_DKL ){
			// 	max_DKL = DKL;
			// 	goal_position.x = tmp_position.x; // + (randu()-0.5)/5.0
			// 	goal_position.y = tmp_position.y;
			// 	goal_position.heading = 0;
			// }
            // fake_history.measurement.pop_back();
			// fake_history.position.pop_back();
			// fake_history.size--;
			double fake_concentration = plume_model.approx_concentration(tmp_position, sampleBuffer.average_wind_intensity);
			if(fake_concentration > max_meas){
				max_meas = fake_concentration;
				goal_position.x = tmp_position.x; 
				goal_position.y = tmp_position.y;
				goal_position.heading = 0;
			}		
	}
	// for(int i =- 1; i <= 1; i++){
	// 	for (int j = -1; j <= 1; j++){
	// 		tmp_position.x = current_position.x + i*p0;
	//  		tmp_position.y = current_position.y + j*p0;
	// 		if(!position_outside_arena(tmp_position)){
	// 			goal_position.x = tmp_position.x ; // + (randu()-0.5)/5.0
	// 			goal_position.y = tmp_position.y ;
	// 			goal_position.heading = 0;
	// 			break;
	// 		}
	// 	}
	// }
}
/*
void information_gain_navigation_predictive(){
	int s=0, c=0;
	float p0 = 0.5; //50 cm
	double reward=0, max_reward = -100;
	double position[1][3] = {{0}};
	
	for(c=0; c<3; c++){
		for(s=-1; s<=1; s=s+2){
			position[0][0] = current_position[0];
			position[0][1] = current_position[1];
			position[0][2] = current_position[2];
			position[0][c] += s*p0;
			if(VERBOSE) printf("[%d] position : %f %f %f\n", ID, position[0][0],position[0][1],position[0][2]);
			if(position_outside_arena(position[0])) continue;
			reward = target_evaluation(position);
			if(VERBOSE) printf("[%d] reward: %f _______________________\n", ID, reward);
			if(reward > max_reward ){
				max_reward = reward;
				goal_position[0] = position[0][0];
				goal_position[1] = position[0][1];
				goal_position[2] = position[0][2];
			}
		}
	}
	if(VERBOSE) printf("[%d] goal_position : %f %f %f\n", ID, goal_position[0],goal_position[1],goal_position[2]);
}

double target_evaluation(double position[][3]){
	int i = 0;
	double predictive[N_CONCENTRATION_LEVEL] = {0};
	double sum_predictive = 0;
	double DKLs[N_CONCENTRATION_LEVEL] = {0};
	double reward = 0;
	//double entropy_values[N_CONCENTRATION_LEVEL] = {0};
	//double reward2 = 0;

	//calculate max concentration value based on current estimation
	float max_concentration = pseudo_gaussian_plume_model(posterior.expected_value.source_position, measurements.wind_intensity, posterior.expected_value);
	if(VERBOSE) printf("[%d] max_concentration : %f\n", ID, max_concentration);
	//for each level of concentration
	for(i=0; i<N_CONCENTRATION_LEVEL; i++){
		history.measurement[FSM.n_cycles] = i*max_concentration/(N_CONCENTRATION_LEVEL-1);
		//printf("measurement level %d: %f\n", i, measurement_history[FSM.n_cycles]);
		//calculate the predictive posterior and it's expected value P(i*max_concentration/(N_CONCENTRATION_LEVEL-1))
		predictive[i] = posterior_predictive(history.measurement[FSM.n_cycles], position);
		//printf("P(x~) = %f\n", predictive[i]);
		sum_predictive += predictive[i];
		//calculate posterior
		//memcpy(future_posterior.pdf, zero.pdf, sizeof zero.pdf);//memcpy(future_posterior_3D, zero_posterior, sizeof zero_posterior);
		//future_posterior.sum = 0;
		future_posterior.copy_posterior(zero);
		//posterior_fct(future_posterior, measurement_history, FSM.n_cycles); //future_posterior_3D, &sum_future_posterior,
		future_posterior.MCMC_evaluation(history);
		//log_posterior_pdf_3D(future_posterior_3D, "../../../Results/tmp_future_posterior.csv", "a");
		//DKL of each posterior
		//DKLs[i] = Kullback_Leibler_divergence(posterior.pdf, posterior.sum, future_posterior.pdf, future_posterior.sum);
		DKLs[i] = posterior.Kullback_Leibler_divergence(future_posterior);
		//printf("DKL: %f \n",DKLs[i]);
		//entropy_values[i] = entropy(future_posterior_3D, sum_future_posterior);
	}
	for(i=0; i<N_CONCENTRATION_LEVEL; i++){
		//weighted DKL with predictive expected value
		reward += (predictive[i]/sum_predictive) * DKLs[i];
		//reward2 += (predictive[i]/sum_predictive) * entropy_values[i];
		//printf("predictive: %f, DKL: %f, reward: %f   ---   predictive: %f, entropy: %f, reward: %f\n", 
			//predictive[i]/sum_predictive,DKLs[i],reward, predictive[i]/sum_predictive,entropy_values[i],reward2);
	}
	return reward;
}
*/
void balance_exploration_exploitation(float gloabl_local_search_ratio){
	float ratio = gloabl_local_search_ratio;//simulation_parameters.gloabl_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
	float vec1[2] = {0}, vec2[2] = {0};
	printf("current_position : %f %f , goal position : %f %f , estimated source : %f %f \n", current_position.x,current_position.y,
		goal_position.x,goal_position.y, plume_model.max_a_post_value[0],plume_model.max_a_post_value[1]);
	vec1[0] = goal_position.x - current_position.x; //exploration
	vec1[1] = goal_position.y - current_position.y;
 	//vec1[2] = goal_position[2] - current_position[2]; 
	//vec2[0] = posterior.max_a_posteriori.source_position[0] - current_position[0]; //exploitation (source) //estimated_source_parameters //max_a_post_source_parameters
	//vec2[1] = posterior.max_a_posteriori.source_position[1] - current_position[1];
	vec2[0] = plume_model.max_a_post_value[0] - current_position.x; //TODO: to improve
	vec2[1] = plume_model.max_a_post_value[1] - current_position.y;
	//vec2[2] = estimated_source_parameters.source_position[2] - current_position[2];
	goal_position.x = current_position.x + ratio*vec1[0] + (1-ratio)*vec2[0];
	goal_position.y = current_position.y + ratio*vec1[1] + (1-ratio)*vec2[1];
	goal_position.heading = 0;//current_position[2] + ratio*vec1[2] + (1-ratio)*vec2[2];
	//printf("new goal position : %f %f %f\n", goal_position[0],goal_position[1],goal_position[2]);
}
/*
void heterogeneous_navigation(){
	//ID of the robot decides what it does
	switch (ID){
		case 101 : 
			// just move to more informative point
			information_gain_navigation_approx(posterior.expected_value.source_position);
			balance_exploration_exploitation(0.8);
			break;
		case 102 :
			information_gain_navigation_approx(posterior.expected_value.source_position);
			balance_exploration_exploitation(0.2);
			// just move to maximum a posterior point
			//goal_position[0] = posterior.max_a_posteriori.source_position[0];
			//goal_position[1] = posterior.max_a_posteriori.source_position[1];
			//goal_position[2] = 0;
			break;
		case 103 :
			// Random
			goal_position[0] = 1;//(randu() * (XMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;  //position in m
    		goal_position[1] = 1;//(randu() * (YMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
    		goal_position[2] = 0;
			break;
	}
}*/

void greedy_planning(){
	double best_point[2]={0};
	// FIXME - temperoy solution; the code is kind of confusing
	// global_local_ratio = 1; only use the factor of the marginal posterior, not considersing the distance
	if (posterior.entropy > 4){
		posterior.XYmarginal_distance_combined(current_position, 1.0, best_point);
	} else {
		posterior.XYmarginal_distance_combined(current_position, 0.75, best_point);
	}
	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;

}
void informative_path_planning_based_on_variance(){
	double best_point[2]={0};
	// find the maximal variance in gas variance map
	double max_variance = 0;
	int max_variance_index = 0;
	for (int i = 0; i< x_nbins*y_nbins; i++){
		if (gas_concentration_variance_map[i] > max_variance){
			max_variance = gas_concentration_variance_map[i];
			max_variance_index = i;
		}
	}
	posterior.index_to_param_values(max_variance_index, best_point);

	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;
	printf("goal position : %f %f \n", goal_position.x, goal_position.y);

}

void balance_expected_maximal_pdf(){
	double maximal_pdf_point[2]={0};
	double expected_pdf_point[2]={0};
	maximal_pdf_point[0] = plume_model.max_a_post_value[0];
	maximal_pdf_point[1] = plume_model.max_a_post_value[1];
	expected_pdf_point[0] = plume_model.expected_value[0];
	expected_pdf_point[1] = plume_model.expected_value[1];
	goal_position.x = (maximal_pdf_point[0] + expected_pdf_point[0])/2;
	goal_position.y = (maximal_pdf_point[1] + expected_pdf_point[1])/2;
	goal_position.heading = 0;

}

void joint_planning(){
	//std::cout<<"joint_planning"<<std::endl;
	// 2 robots, 2 possibilities
	if(simulation_parameters.number_of_robots == 2){
		if(pt_distance(current_position, friends[0].goal)+pt_distance(friends[0].position, goal_position) < pt_distance(current_position, goal_position)+pt_distance(friends[0].position, friends[0].goal) ){
			//printf("let's swap ************************************************\n");
			//swap the goal positions
			//inform the other guy
			//wb_emitter_set_channel(emitter, 10);
			Message msg(ID, friends[0].ID, current_position, -1, goal_position, 1);
    		//wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
    		send_msg(msg);
    		//change mine as well
			goal_position.copy_position(friends[0].goal);
			//printf("my new goal %f %f %f , my friend's new goal \n", goal_position[0], goal_position[1], goal_position[2]);
		}
	}
	// 3 robots, 6 possibilites
	if(simulation_parameters.number_of_robots == 3){
		if(std::isnan(friends[0].goal.x) || std::isnan(friends[0].goal.y) || std::isnan(friends[1].goal.x) || std::isnan(friends[1].goal.y)) return;
		int new_dist = 0, min_dist = 0;
		Position my_new_goal;
		Position friend_1_new_goal;
		Position friend_2_new_goal;
		my_new_goal.copy_position(goal_position);
		friend_1_new_goal.copy_position(friends[0].goal);
		friend_2_new_goal.copy_position(friends[1].goal);
		//printf("our old goals : mine %f %f %f, friend 1: %f %f %f, friend 2: %f %f %f \n", goal_position[0], goal_position[1], goal_position[2], friends[0].goal[0], friends[0].goal[1], friends[0].goal[2], friends[1].goal[0],friends[1].goal[1],friends[1].goal[2]);
		
		//0 : no swap
		min_dist = pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[0].goal) + pt_distance(friends[1].position, friends[1].goal);
		//1 : friends swap
		new_dist = pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[1].goal) + pt_distance(friends[1].position, friends[0].goal);
		if(new_dist < min_dist){
			//printf("we are in 1\n");
			min_dist = new_dist;
			my_new_goal.copy_position(goal_position);
			friend_1_new_goal.copy_position(friends[1].goal);
			friend_2_new_goal.copy_position(friends[0].goal);
		}
		//2 : I swap with friend1, friend1 swaps with friend2
		new_dist = pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, friends[1].goal) + pt_distance(friends[1].position, goal_position);
		if(new_dist < min_dist){
			//printf("we are in 2\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[0].goal);
			friend_1_new_goal.copy_position(friends[1].goal);
			friend_2_new_goal.copy_position(goal_position);
		}
		//3 : I swap with friend1
		new_dist = pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, goal_position) + pt_distance(friends[1].position, friends[1].goal);
		if(new_dist < min_dist){
			//printf("we are in 3\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[0].goal);
			friend_1_new_goal.copy_position(goal_position);
			friend_2_new_goal.copy_position(friends[1].goal);
		}
		//4 : I swap with friend2, friend2 swaps with friend1
		new_dist = pt_distance(current_position, friends[1].goal) + pt_distance(friends[0].position, goal_position) + pt_distance(friends[1].position, friends[0].goal);
		if(new_dist < min_dist){
			//printf("we are in 4\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[1].goal);
			friend_1_new_goal.copy_position(goal_position);
			friend_2_new_goal.copy_position(friends[0].goal);
		}
		//5 : I swap with friend2
		new_dist = pt_distance(current_position, friends[1].goal) + pt_distance(friends[0].position, friends[0].goal) + pt_distance(friends[1].position, goal_position);
		if(new_dist < min_dist){
			//printf("we are in 5\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[1].goal);
			friend_1_new_goal.copy_position(friends[0].goal);
			friend_2_new_goal.copy_position(goal_position);
		}
		
		//printf("Our new goals : mine %f %f %f, friend 1: %f %f %f, friend 2: %f %f %f \n", goal_position[0], goal_position[1], goal_position[2], friends[0].goal[0], friends[0].goal[1], friends[0].goal[2], friends[1].goal[0],friends[1].goal[1],friends[1].goal[2]);
		
		//inform the other guys if any changes
		if(friend_1_new_goal.x!=friends[0].goal.x || friend_1_new_goal.y!=friends[0].goal.y){
			Message msg1(ID, friends[0].ID, current_position, -1, friend_1_new_goal, 1);
    		send_msg(msg1);
    	}//else printf("no change for friend %d\n", friends[0].ID);
    	if(friend_2_new_goal.x!=friends[1].goal.x || friend_2_new_goal.y!=friends[1].goal.y){
    		Message msg2(ID, friends[1].ID, current_position, -1, friend_2_new_goal, 1);
    		send_msg(msg2);
    	}//else printf("no change for friend %d\n", friends[1].ID);
    	
    	//change mine as well
		goal_position.copy_position(my_new_goal);
	}
}

void DKL_MAP_distance(float gloabl_local_search_ratio){
	double best_point[2]={0};
	posterior.XYmarginal_distance_combined(current_position, gloabl_local_search_ratio, best_point);
	printf("greedy goal %f %f\n", goal_position.x, goal_position.y);
	printf("best point %f %f\n", best_point[0], best_point[1]);

	float ratio = 0;//gloabl_local_search_ratio;//simulation_parameters.gloabl_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
	float vec1[2] = {0}, vec2[2] = {0};
	//printf("current_position : %f %f %f, goal position : %f %f %f, estimated source : %f %f %f\n", current_position[0],current_position[1],current_position[2],
	//	goal_position[0],goal_position[1],goal_position[2], estimated_source_parameters.source_position[0],estimated_source_parameters.source_position[1],estimated_source_parameters.source_position[2]);
	vec1[0] = goal_position.x - current_position.x; //exploration
	vec1[1] = goal_position.y - current_position.y;
 	//vec1[2] = goal_position[2] - current_position[2]; 
	vec2[0] = best_point[0] - current_position.x; //exploitation (source) //estimated_source_parameters //max_a_post_source_parameters
	vec2[1] = best_point[1] - current_position.y;
	//vec2[2] = estimated_source_parameters.source_position[2] - current_position[2];
	goal_position.x = current_position.x + ratio*vec1[0] + (1-ratio)*vec2[0];
	goal_position.y = current_position.y + ratio*vec1[1] + (1-ratio)*vec2[1];
	goal_position.heading = 0;//current_position[2] + ratio*vec1[2] + (1-ratio)*vec2[2];

}

void goal_outside_obstacles(Position * goal){
	// is the goal position in an obstacle?
	int idx = goal->is_in_obstacle_list(obstacles, nbr_obs);
	// if not, then back to business
	if(idx == -1 && !position_outside_arena(*goal)){
		printf(">>> the goal is not in any obstacles or outside of the region\n");
		return;
	}

	//set concentration to 0 in the original goal position
	// history.position[history.size].x = goal->x;
	// history.position[history.size].y = goal->y;
	// history.position[history.size].heading = 0;
	// history.measurement[history.size] = 0;
	// history.size++;
    
	if (idx != -1){
		printf(">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);
		// otherwise, lets try 4 different candidates
		float min_dist = 10000.0;
		Position nearest_position(*goal);
		Position candidates[4] = {	Position(obstacles[idx][0], goal->y, 0), 
									Position(obstacles[idx][2], goal->y, 0),
									Position(goal->x, obstacles[idx][1], 0),
									Position(goal->x, obstacles[idx][3], 0)};
		// for each candidate position
		for(int i=0; i<4; i++){
			//check if the distance to candidate is smaller than others
			// printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
			if(goal->distance_from(candidates[i]) < min_dist){
				// printf(">>> best sor far\n");
				//if so, then check if that position is not in another obstacle and it's inside the arena
				// printf(">>> inside another obstacle ? %d\n", candidates[i].is_in_obstacle_list(obstacles, nbr_obs));
				// printf(">>> outside the arena ? %d \n", position_outside_arena(candidates[i]));
				if(candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i])){
					//if so, then this is the nearest position
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
					// printf(">>> chose it as new position !!! \n");
				}
			}
		}
		//change the goal position using the pointer
	    printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}

    if (position_outside_arena(*goal)){
		printf(">>> ! position %f %f is outside the region\n", goal->x, goal->y);
		// otherwise, lets try 4 different candidates
		float min_dist = 10000.0;
		Position nearest_position(*goal);
		Position candidates[4] = {	Position(XMIN, goal->y, 0), 
									Position(XMAX, goal->y, 0),
									Position(goal->x, YMIN, 0),
									Position(goal->x, YMAX, 0)};
		// for each candidate position
		for(int i=0; i<4; i++){
			//check if the distance to candidate is smaller than others
			// printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
			if(goal->distance_from(candidates[i]) < min_dist){
				// printf(">>> best sor far\n");
				//if so, then check if that position is not in another obstacle and it's inside the arena
				// printf(">>> inside another obstacle ? %d\n", candidates[i].is_in_obstacle_list(obstacles, nbr_obs));
				// printf(">>> outside the arena ? %d \n", position_outside_arena(candidates[i]));
				if(candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i])){
					//if so, then this is the nearest position
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
					// printf(">>> chose it as new position !!! \n");
				}
			}
		}
		//change the goal position using the pointer
		printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}
	
}

// ============================== STATES ================================= //
void state_waiting(){
	// initial waiting for the plume to get established
	static int counter = 0;
	counter++;

	if(counter > 300){
		FSM.state = STATE_MEASURING;
		FSM.hook  = state_measuring;
	}
}

void state_measuring(){
	//printf("measuring sample number %d\n", measurements.n_samples);
	vector<double> sample_from_map;
    // if robot is sensing while moving, saving the measurement in motion 
    if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING){
		// if the kernel mapping is enabled, then retrieve the measurement from the gas map
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_REVERSE_SENSOR_MODEL){
			gas_map.retrieve_readings_from_gasmap(sample_in_motion.position,sample_from_map);
			for(int i=0; i<sample_in_motion.size; i++){
				// printf("instant measurement at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, sample_in_motion.measurement[i]);
				// printf("measurement from gas map at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, sample_from_map[i]);
				// printf("ground truth at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, ground_truth_in_motion[i]);
				history.position.push_back(sample_in_motion.position[i]);
				history.measurement.push_back(sample_from_map[i]);
				history.size++;
			}
		} else {
			// if else, save the instant measurement directly
			for(int i=0; i<sample_in_motion.size; i++){
				if (simulation_parameters.gasSensor == MOX){
					// printf("instant measurement at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, sample_in_motion.measurement[i]);
				    // printf("ground truth at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, ground_truth_in_motion[i]);
				}
				
				history.position.push_back(sample_in_motion.position[i]);
				history.measurement.push_back(sample_in_motion.measurement[i]);
				history.size++;
			}
		}
		sample_in_motion.position.clear();
		sample_in_motion.measurement.clear();
		sample_in_motion.size = 0;
		ground_truth_in_motion.clear();
		sample_from_map.clear();

		// save the instant reading at the goal position as well; we only care about the ground truth here
		double odor_concentration = 0.0;
        odor_read(&sampleBuffer.instant_concentration, odor_concentration);
		wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
		// FIXME - add the case for reverse sensor model
		history.position.push_back({current_position.x, current_position.y, 0});
		//history.measurement[history.size] = sampleBuffer.average_concentration;
		history.measurement.push_back(sampleBuffer.average_concentration);
		history.size++;
        if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
		log_position();
		log_measurements();
		// go to estimation
		FSM.state = STATE_ESTIMATING;
		FSM.hook  = state_estimating;
	}
	if (simulation_parameters.samplingMode == STOP_AND_GO_SAMPLING){
		// Sample for several seconds and take the average value
		if(sampleBuffer.n_samples < ODOR_FILTER_LENGTH){
			//continue measuring odor concentration
			double odor_concentration = 0.0;
			odor_read(&sampleBuffer.instant_concentration, odor_concentration);
			//measuring wind
			wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
			//printf("instant concentration: %f, wind intensity: %f\n", sampleBuffer.instant_concentration, sampleBuffer.instant_wind_intensity);
			//add samples to buffers
			sampleBuffer.add_all_to_buffer();
			//printf("n %d wind intensity: %f, angle %f \n", sampleBuffer.n_samples, sampleBuffer.instant_wind_intensity, sampleBuffer.instant_wind_angle);
		}else{
			//average all samples
			sampleBuffer.average_all();
			//log and save
			if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
			log_position();
			log_measurements();
			if (!position_outside_arena(current_position)){
				history.position.push_back({current_position.x, current_position.y, 0});
				//history.measurement[history.size] = sampleBuffer.average_concentration;
				history.measurement.push_back(sampleBuffer.average_concentration);
				history.size++;
				history_print(history);
			}
			// send it to friends
			if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
				Message msg(ID, 1, history.position[(history.size)-1], sampleBuffer.average_concentration, goal_position, 0);
				send_msg(msg);
			}

			// go to estimation
			FSM.state = STATE_ESTIMATING;
			FSM.hook  = state_estimating;
		}
	}
	if (simulation_parameters.samplingMode == SENSE_WITHOUT_STOP){
		//continue measuring odor concentration
		double odor_concentration = 0.0;
		odor_read(&sampleBuffer.instant_concentration, odor_concentration);
		//measuring wind
		wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
		//printf("instant concentration: %f, wind intensity: %f\n", sampleBuffer.instant_concentration, sampleBuffer.instant_wind_intensity);
		//add samples to buffers
		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
		//log and save
		if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
		log_position();
		log_measurements();
		if (!position_outside_arena(current_position)){
			history.position.push_back({current_position.x, current_position.y, 0});
			//history.measurement[history.size] = sampleBuffer.average_concentration;
			history.measurement.push_back(sampleBuffer.average_concentration);
			history.size++;
			history_print(history);
		}
		// send it to friends
		if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
			Message msg(ID, 1, history.position[(history.size)-1], sampleBuffer.average_concentration, goal_position, 0);
			send_msg(msg);
		}

		// go to estimation
		FSM.state = STATE_ESTIMATING;
		FSM.hook  = state_estimating;
	}
}

void state_estimating(){
	if(VERBOSE) printf("[%d] estimating...\n", ID);
	//reinitialize
	posterior.pdf_reset();
	//estimation
	if(simulation_parameters.MCMC_or_exhaustive_eval){
		printf("going for MCMC\n");
		posterior.MCMC_evaluation(history);
	}
	else{
		printf("going for exhaustive\n");
		posterior.exhaustive_evaluation(history);
	}
	// only when using MCMC
	if(posterior.dirac_flag){
		if(VERBOSE) printf("[%d] exhaustive search!\n", ID);
		posterior.exhaustive_evaluation(history);
	}
	
	printf("sum posterior : %f \n", posterior.sum);
	//printf("Before process. \n");
	posterior.process();
	plume_model.calculate_weighted_gas_con_variance_map(gas_concentration_variance_map, posterior.pdf);
	log_gas_concentration_variance_map();
	// calculate the variance of reading for each cell
	//posterior.calculate_cell_measure_variance_for_all_possible_source(cell_measurements_list);
	//printf("After Process. Start logging.\n");
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	//printf("Finish logging.\n");
	posterior.log_pdf(posterior_pdf_3D_name, 'a');
	//posterior.log_pdf("/home/wjin/Workspace/continuous_STE/Results/current/101/posterior.csv",'a');

	FSM.n_cycles++;
	if(VERBOSE) printf("[%d] cycle number %d\n", ID, FSM.n_cycles);
	if(VERBOSE) printf("[%d] entropy : %f\n", ID, posterior.entropy);

	char label[255];
	sprintf(label,"Iteration: %d\n",FSM.n_cycles);
	wb_supervisor_set_label(2,label,0.1,0.8,0.1,0x61A8E8,0,"Impact");
	char label2[255];
	sprintf(label2,"Reading: %f\n",sampleBuffer.average_concentration);
	wb_supervisor_set_label(1,label2,0.1,0.85,0.1,0xFF0000,0,"Impact");
	char label3[255];
	sprintf(label3,"Entropy: %.3f\n",posterior.entropy);
	wb_supervisor_set_label(3,label3,0.1,0.9,0.1,0xFFD966,0,"Impact");
	
	
	//check if time is up
	//if(FSM.n_cycles == MAX_SAMPLE_NUMBER || posterior.entropy < 0.1 || posterior.dirac_flag){
	if(FSM.n_cycles == MAX_SAMPLE_NUMBER || posterior.entropy < 0.1){
		//the end
		FSM.state = STATE_FINISHED;
		printf("Current state is STATE_FINISHED");
	}else{
		//update goal
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook  = state_updategoal;
	}
}

void state_updategoal(){
	if(VERBOSE) printf("[%d] updating the goal\n", ID);

	//find the best point to move to 

	// switch(simulation_parameters.scenario){
	// 	case NO_COM_NO_COOR :
	// 		information_gain_navigation_approx(current_position);
	// 		// printf("goal position here 1 %f %f\n", goal_position.x, goal_position.y);
    // 		//information_gain_navigation_predictive();
    // 		//balance_exploration_exploitation(simulation_parameters.gloabl_local_search_ratio);
    // 		DKL_MAP_distance(1.0);
    // 		// printf("goal position here 2 %f %f\n", goal_position.x, goal_position.y);
    // 		break;
    // 	case COM_NO_COOR :
    // 		information_gain_navigation_approx(current_position);
    // 		//balance_exploration_exploitation(simulation_parameters.gloabl_local_search_ratio);
    // 		DKL_MAP_distance(0.75);
    // 		break;
	//     case COM_COOR :
	//     	information_gain_navigation_approx(current_position);
	//     	//printf("[%d] goal position %f %f \n", ID, goal_position[0], goal_position[1]);
	//     	//balance_exploration_exploitation(simulation_parameters.gloabl_local_search_ratio);
	//     	DKL_MAP_distance(0.75);
	//     	//printf("[%d] goal position %f %f \n", ID, goal_position[0], goal_position[1]);
    // 		joint_planning();
    // 		//printf("[%d] goal position %f %f \n", ID, goal_position[0], goal_position[1]);
    // 		break;
    // }
    switch (simulation_parameters.planningStrategy)
	{
	case GREEDY:
		greedy_planning();
		break;
	case BALANCE_MAXIMAL_EXPECTED:
	    balance_expected_maximal_pdf();
		break;
    case INFORMATIVE:
	    informative_path_planning_based_on_variance();
		break;
	default:
		break;
	}
    goal_outside_obstacles(&goal_position);
    // printf("goal position here %f %f 3\n", goal_position.x, goal_position.y);
    //heterogeneous_navigation();

    printf("from %f %f to goal %f %f \n", current_position.x, current_position.y, goal_position.x, goal_position.y);
  //   aStar as;
  //   if(as.search(current_position, goal_position, astar_map, 0)){
		// as.path( path );
		// i = path.begin();
		// std::ofstream myfile;
		// myfile.open("aStar_paths.txt", ios::app);
		// for( std::list<Position>::iterator i = path.begin(); i != path.end(); i++ )
	 //    	myfile<< " " << i->x << ", " << i->y << ";";
  //   	myfile<< std::endl;
  //   	myfile.close();
  //   }else{
  //   	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> aStar could not find a way!\n");
  //   	as.search(current_position, goal_position, astar_map, 1);
  //   }

    //find the path using visibility graph
	// current_position.x = 4.095615;
	// current_position.y = 3.351716;
	// goal_position.x = 8.228684;
	// goal_position.y = 3.885515;
    VisiLibity::Point start(current_position.x, current_position.y);
    VisiLibity::Point finish(goal_position.x, goal_position.y);
    VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);
    // copy the path in another variable
    for(unsigned k=0; k<Polyline_path.size(); k++){
    	Position p(Polyline_path[k].x(), Polyline_path[k].y(),0);
        path.push_back(p);
        log_trajectory(p);
    }
    i = path.begin();
    printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
    //send the goal to the supervisor
    Message msg(ID, 0, goal_position, -1, goal_position, 0);
    send_msg(msg);
	//move to the goal position
	FSM.state = STATE_MOVING;
	FSM.hook = state_moving;
}

void state_moving(){
	//printf("moving toward the goal\n");
	if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING){
		sample_counter++;
		//sample and run kernel 
		double instant_concentration = 0.0;
		double odor_concentration = 0.0;
		double reverse_sensor_reading = 0.0;
		odor_read(&instant_concentration, odor_concentration);
		if (simulation_parameters.processingMode == REVERSE_SENSOR_MODEL || simulation_parameters.processingMode == MAPPING_AND_REVERSE_SENSOR_MODEL){
			// FIXME: temporary attemp for reverse sensor model
			double Ts = TIME_STEP / 1000.0; // sampling frequency
			double tau_r = 1.3; // time constant for the rise
			double tau_d = 2.0;
			double alpha = 0.0;
			if (instant_concentration > previous_sensor_reading)   //rising phase
				alpha = tau_r / Ts;
			else                                       //decaying phase
				alpha = tau_d / Ts;
			reverse_sensor_reading = (1 + alpha) * instant_concentration - alpha * previous_sensor_reading;
			// printf("reverse_sensor_reading %f\n", reverse_sensor_reading);
			// printf("instant_concentration %f\n", instant_concentration);
			// printf("odor_concentration %f\n", odor_concentration);
			previous_sensor_reading = instant_concentration;
			instant_concentration = reverse_sensor_reading;
		}
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_REVERSE_SENSOR_MODEL){
			gas_map.runKernelAlgorithm(current_position, instant_concentration);
		}
		if (sample_counter > 10){
			if (!position_outside_arena(current_position)){
				sample_in_motion.position.push_back({current_position.x, current_position.y, 0});
				sample_in_motion.measurement.push_back(instant_concentration);
				sample_in_motion.size++;
				ground_truth_in_motion.push_back(odor_concentration);
				//history.position.push_back({current_position.x, current_position.y, 0});
				//history.measurement.push_back(instant_concentration);
				//history.size++;
			}
			sample_counter = 0;
		}
	}
	//verify if arrived to the goal if applicable
	if(dist_to_goal(goal_position) > 0.05){
		//move
		if(dist_to_goal(*i) <= 0.05 && i != path.end()){
			i++;
			printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		}
		go_to_goal(*i);
		//obstacle avoidance if necessary
		// if(obstacle_or_not() == 1){
		// 	FSM.state = STATE_OBSTACLE_AVOIDANCE;
		// 	FSM.hook  = state_obstacle_avoidance;
		// 	return;
		// }
		//wb_differential_wheels_set_speed(speed[0],speed[1]);
		set_wheels_speed(speed[0],speed[1]);
		//log the position
		//log_position();
	}else{
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		sampleBuffer.reset_all();
		path.clear();
		//back to measuring
		FSM.state = STATE_MEASURING;
		FSM.hook  = state_measuring;
	}
}

void state_obstacle_avoidance(){
	//printf("avoiding obstacle\n");
	//obstacle_avoidance_k3();
	obstacle_avoidance_k4();
}


// =========================== MAIN ============================== //


int main(int argc, char **argv){
	// python instance (should be called ONLY ONCE and only in main)
	//CPyInstance hInstance;
	init();
	read_obstacle_list();
    //load_model_c();
	//visibility graph (visilibity library)
	std::cout << "Validating environment model . . . ";
  	if(  my_environment.is_valid( epsilon )  )
    	std::cout << "OK" << std::endl;
  	else{
    	std::cout << std::endl << "Warning:  Environment model "
	    << "is invalid." << std::endl
	    << "A valid environment model must have" << std::endl
	    << "   1) outer boundary and holes pairwise "
	    << "epsilon -disjoint simple polygons" << std::endl
	    << "   (no two features should come "
	    << "within epsilon of each other)," << std::endl 
	    << "   2) outer boundary is oriented ccw, and" 
	    << std::endl
	    << "   3) holes are oriented cw."
	    << std::endl;
    	exit(0);
  	}
  	VisiLibity::Visibility_Graph vg(my_environment, epsilon);
	char label[255];
    sprintf(label,"Iteration: %d\n",0);
    wb_supervisor_set_label(2,label,0.1,0.8,0.1,0x61A8E8,0,"Impact");
	char label2[255];
    sprintf(label2,"Reading: %d\n",0);
    wb_supervisor_set_label(1,label2,0.1,0.85,0.1,0xFF0000,0,"Impact");
    

	// //main loop
	while (FSM.state != STATE_FINISHED){
		update_position();
		//if we are not stopped to sample, let's log the trajectory
		/*if(speed[0]!=0 && speed[1]!=0){
			log_trajectory();
		}*/
		//send info (current position and goal position) to friends for joint planning
		if(simulation_parameters.scenario == COM_COOR){
			// //if a robot is already sampling (i.e. STATE_MEASURING), its goal should not be swapped with anyone. Therefore we send 0 as goal and current position
			if(FSM.state == STATE_MEASURING){
				Position tmp;
				Message msg(ID, 1, tmp, sampleBuffer.average_concentration, tmp, 1);
				send_msg(msg);
			}else{
				Message msg(ID, 1, current_position, sampleBuffer.average_concentration, goal_position, 1);
				//printf("[%d] I am sendig my position %f %f %f and my goal %f %f %f\n", msg.sender_ID, msg.position[0],msg.position[1], msg.position[2], msg.goal[0],msg.goal[1],msg.goal[2]);
    			send_msg(msg);
    		}
    	}
		//receive info from colleagues for joint planning and history
		if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
			//printf("Robot %d has %d unread messages! \n", ID, wb_receiver_get_queue_length(receiver));
			receive_from_friends();
			//printf("[%d] my history size is %d\n", ID, history.size);
		}
	   	//change state
    	FSM.hook();
    	//next
    	next_iteration_time();
    	n_time_step++;
	}
	// //final logs
	log_history();
	cleaning_up();
    //free the allocated memory to obstacle array
    for(int i = 0; i < nbr_obs; i++)
    	delete[] obstacles[i];
	delete[] obstacles;
	exit(0);
}

