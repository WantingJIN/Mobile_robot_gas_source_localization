/*
* File:          controller_STE_multi-robot.cpp
* Date:          Jan 2019
* Description:   The goal is to estimate the source terms including the position
* Author:        Faezeh Rahbar
* Modifications: 26/01/2019
*/

//custom libraries
#include <fstream>
#include "controller_STE_multi-robot.h"
#include "Posterior.h"
#include "Message.h"
#include "visilibity.hpp"
#include "udp_msg.h"
#include "tcp_msg.h"
#include "khepera_functions.h"
#include "CellMeasurements.h"
#include "GasDispersionMap.h"


using namespace std;

//Global Variables
#define PORT 3011
#define SA struct sockaddr 

// log files
char folder_name[25];
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
extern FILE *gas_concentration_map_logfile;
FILE* gas_information_map_logfile;
FILE* gas_hit_variance_map_logfile;
FILE* experiment_time_logfile;
FILE *visited_list_logfile;
FILE *scores_map_logfile;
FILE *distance_map_logfile;
FILE *acc_info_map_logfile;

// text file for storing some prints for debugging
FILE *console_logfile;

char goalposition_logfile_name[256];
char measurements_logfile_name[256];
char posterior_logfile_name[256];
char posteriorpdf_logfile_name[256];
char expected_a_posteriori_logfile_name[256];
char max_a_posteriori_logfile_name[256];
char likelihood_logfile_name[256];
char measurement_history_logfile_name[256];
char position_history_logfile_name[256];
char posterior_pdf_3D_name[256];
char entropy_logfile_name[256];
char config_logfile_name[256];
char trajectory_logfile_name[256];
char gas_information_map_logfile_name[512];
char gas_hit_variance_map_logfile_name[512];
char experiment_time_logfile_name[512];
char visited_list_logfile_name[512];
char relative_measurements_logfile_name[512];
char scores_map_logfile_name[512];
char distance_map_logfile_name[512];
char acc_info_map_logfile_name[512];

// text file for storing some prints for debugging
char console_logfile_name[512];

// read data from file
char logfile_folder_name[256];
FILE* experiment_measurements_logfile;
char experiment_measurements_logfile_name[300];
FILE* experiment_position_logfile;
char experiment_position_logfile_name[300];
FILE* experiment_goalposition_logfile;
char experiment_goalposition_logfile_name[300];

char scan_folder_name[256];
FILE* scan_measurements_logfile;
char scan_measurements_logfile_name[300];
FILE* scan_position_logfile;
char scan_position_logfile_name[300];

// define the global parameter, the path to the workspace
const char* workspacePath = getenv("STE_DDPM_ROOT");
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

//plume model
DeepCFDPlumeModel2D plume_model;

// gas information map
double gas_information_map[x_nbins*y_nbins] = {0};
double gas_hit_variance_map[x_nbins*y_nbins] = {0};
int cell_visited_list[x_nbins*y_nbins] = {0};
double scores_map[x_nbins * y_nbins] = {0};
double distance_map[x_nbins * y_nbins] = {0};
double acc_info_map[x_nbins * y_nbins] = {0}; 

// current posterior distribution 
Posterior posterior(&plume_model);

//robot's ID
int ID;

//message structure
struct message message;

//list of other robots
struct friends friends[MAX_NB_ROBOTS];

int n_time_step = 0;

SimulationParameters simulation_parameters;

// obstacles definitions
int nbr_obs = 4;
float margin = 0.2;
float ** obstacles = NULL;
// float obstacles[][4] = {{5-margin, static_cast<float>(5.5+margin), static_cast<float>(2.5-margin), 4+margin}, 
// 							{5-margin, 8+margin, static_cast<float>(2.5-margin), 3+margin}, 
// 							{9-margin, 12+margin, 1-margin, static_cast<float>(1.5+margin)}, 
// 							{static_cast<float>(11.5-margin), 12+margin, 0-margin, static_cast<float>(1.5+margin)}};
// map astar_map(obstacles, 4);
std::list<Position> path;
std::list<Position>::iterator i;

std::string environment_file = std::string(workspacePath) + "/Khepera/Data/test_map.environment";
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

int sock = 0; 

// log time
double start_timestamp;
double end_timestamp;
double time_before_moving_and_measuring;
double time_after_moving_and_measuring;
double navigation_sampling_time_sum = 0;
double time_before_estimation;
double time_after_estimation;
double time_before_planning;
double time_after_planning;
double planning_computation_time_sum = 0;
double estimation_computation_time_sum = 0;
double sent_first_goal = false;

// precomputed gas maps
double c_avg[x_nbins * y_nbins][x_nbins * y_nbins]; // left: source position, right: estimated cell position

// store last position for minimum distance threshold in measurement update
Position last_measurement_pos;

// =========================== FUNCTIONS ================================ //
int init_TCP_socket(){
	printf("enter init_TCP_socket\n");
    int server_fd;
    struct sockaddr_in address; 
    int addrlen = sizeof(address); 
    int opt = 1; 
       
    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){ 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))){ 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0){ 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
	printf("After bind\n");
    // listen
    if (listen(server_fd, 3) < 0){ 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
	printf("After listen\n");
    // accept
    if ((sock = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0){ 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    } 
	printf("After accept\n");
	return 1;
}

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

double index_distance(int x_idx_diff, int y_idx_diff){
	return sqrt(pow(x_idx_diff * x_resol, 2) + pow(y_idx_diff * y_resol, 2));
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
    float bearing = atan2(z, x);    //[-PI, PI]
    //printf("bearing : %f \n", bearing/PI*180);

    //angular and linear velocity
    float w = 1 * sinf(bearing);
    float u = 0.2 * cosf(bearing);
    
    // wheels speed
	speed[0] = (u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;   //10 is a speed-up factor
	speed[1] = (u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	//printf("speed : %f %f \n", speed[0], speed[1]);
}

//show the measurement history on screen
void history_print(struct history history){
	printf("_____ Sample History _____ \n");
	for(int i=0; i<history.size; i++){
		printf("%d : %f (+/- %f) @ (%f, %f)\n", i, history.measurement_avg[i], history.measurement_std[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list(){
	// open file
	string obs_file_path = string(workspacePath) + "/Khepera/Data/obstacle_list.txt";
	std::ifstream infile(obs_file_path.c_str());
	// first line is the number of obstacles
	infile >> nbr_obs;
	// allocate memory for obstacle list array
	obstacles = new float*[nbr_obs];
	// read all obstacles now
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

// takes to values and send them to the robot as goal position
bool send_goal_to_robot(double x, double y){
	double g_buffer[2] = {0};
	int timeout = 10;
	int success = -1;

	// make the buffer
	g_buffer[0] = x;
	g_buffer[1] = y;

	// send
	do{
		success = send(sock, g_buffer, sizeof(g_buffer), 0);
		--timeout;
	} while(success == -1 && timeout > 0);

	// print
	if(success > 0){
		printf("successfully sent goal position (%f, %f)\n", g_buffer[0], g_buffer[1]);
		return 1;
	}else{
		printf("sending failed with timeout!\n");
		return 0;
	}
}

// ===================== LOG FILES FUNCTIONS ========================== //

void log_position(){
	double time = get_exp_time();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(){
	//int i = 0;
	double time = get_exp_time();
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

void log_gas_information_map(){
	gas_information_map_logfile = fopen(gas_information_map_logfile_name, "a");

	for(int i = 0; i < x_nbins * y_nbins; i++)
		fprintf(gas_information_map_logfile, "%.9f, ", gas_information_map[i]);

	fprintf(gas_information_map_logfile, "\n");

	fclose(gas_information_map_logfile);
}

void log_gas_hit_variance_map(){
	gas_hit_variance_map_logfile = fopen(gas_hit_variance_map_logfile_name, "a");

	for(int i = 0; i < x_nbins * y_nbins; i++)
		fprintf(gas_hit_variance_map_logfile, "%.9f, ", gas_hit_variance_map[i]);

	fprintf(gas_hit_variance_map_logfile, "\n");

	fclose(gas_hit_variance_map_logfile);
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
		fprintf(measurement_history_logfile,"%d, %f, %f\n", i, history.measurement_avg[i], history.measurement_std[i]);
	}
   	fclose(measurement_history_logfile);
   
	//posterior 3D
	//posterior.log_pdf(posterior_pdf_3D_name, 'w');
}

void log_visited_list()
{
	visited_list_logfile = fopen(visited_list_logfile_name, "a");

	for (int i = 0; i < x_nbins * y_nbins; i++)
		fprintf(visited_list_logfile, "%d, ", cell_visited_list[i]);

	fprintf(visited_list_logfile, "\n");

	fclose(visited_list_logfile);
}

void log_trajectory(Position p){
	double time = get_exp_time();
	trajectory_logfile = fopen(trajectory_logfile_name, "a");
	fprintf(trajectory_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
	fclose(trajectory_logfile);
}

void log_experiment_time(){
	double experiment_time = end_timestamp - start_timestamp;
	printf("total experiment time: %f\n", experiment_time);
	experiment_time_logfile = fopen(experiment_time_logfile_name, "w");
	fprintf(experiment_time_logfile, "total experiment time: %f\n", experiment_time);
	fprintf(experiment_time_logfile, "navigation and sampling time: %f\n", navigation_sampling_time_sum);
	fprintf(experiment_time_logfile, "planning computation time: %f\n", planning_computation_time_sum);
	fprintf(experiment_time_logfile, "estimation computation time: %f\n", estimation_computation_time_sum);
    fclose(experiment_time_logfile);
}

void log_scores_map()
{
	scores_map_logfile = fopen(scores_map_logfile_name, "a");

	for (int i = 0; i < (x_nbins * y_nbins) - 1; i++) // Jusqu'à l'avant-dernière valeur
		fprintf(scores_map_logfile, "%.9f, ", scores_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(scores_map_logfile, "%.9f\n", scores_map[(x_nbins * y_nbins) - 1]);

	fclose(scores_map_logfile);
}

void log_distance_map()
{
	distance_map_logfile = fopen(distance_map_logfile_name, "a");

	for (int i = 0; i < (x_nbins * y_nbins) - 1; i++) // Jusqu'à l'avant-dernière valeur
		fprintf(distance_map_logfile, "%.9f, ", distance_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(distance_map_logfile, "%.9f\n", distance_map[(x_nbins * y_nbins) - 1]);

	fclose(distance_map_logfile);
}

void log_acc_info_map()
{
	acc_info_map_logfile = fopen(acc_info_map_logfile_name, "a");

	for (int i = 0; i < (x_nbins * y_nbins) - 1; i++) // Jusqu'à l'avant-dernière valeur
		fprintf(acc_info_map_logfile, "%.9f, ", acc_info_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(acc_info_map_logfile, "%.9f\n", acc_info_map[(x_nbins * y_nbins) - 1]);

	fclose(acc_info_map_logfile);
}

// ====================== ESTIMATIOM functions ========================= //

//ASSUMPTION : Using the history of measurements, but only one robot is operating
double likelihood(struct history history, double *parameters, PlumeModel *model, bool verbose){ 
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = simulation_parameters.measurement_error; //125;
	double sum_tmp = 0;

	// getting all model concentrations for all sample positions and then calculate the likelihood
	// printf("likelihood... history size %d\n", history.size);
	vector<float> model_c_avg(history.size); 
	vector<float> model_c_std(history.size);
	//vector<vector<float>> true_model_c_avg(history.size, vector<float> (1)); 
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);
	//model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, true_source_location, true_model_c_avg);
	for (int i = 0; i < history.size; i++){
		// float sigmaD = history.measurement_std[i];
		// if (sigmaD > 125){
		// 	sigmaD = 125;
		// }
		sum_tmp += pow(history.measurement_avg[i] - model_c_avg[i], 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
	}
	
	return (exp(-0.5 * sum_tmp));
}


// Likelihood with concentration values from precomputed gas maps
double likelihood_precomp(struct history history, double *parameters, int source_pos_idx, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;
	// if (sampleBuffer.average_wind_intensity == 0)
	// 	printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		idx[i] = floor((px[i] - source_x_min) / x_resol);
		idy[i] = floor((py[i] - source_y_min) / y_resol);
		// idx should be between 0 and x_nbins-1
		if (idx[i] < 0) idx[i] = 0;
		if (idx[i] >= x_nbins) idx[i] = x_nbins - 1;
		// idy should be between 0 and y_nbins-1
		if (idy[i] < 0) idy[i] = 0;
		if (idy[i] >= y_nbins) idy[i] = y_nbins - 1;
	}

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, source_pos_idx, model_c_avg);

	// absolute concentration error
	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.measurement_avg[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
	}

	return (exp(-0.5 * sum_tmp));
}

// Likelihood for relative concentration with precomputed gas maps
double likelihood_rel_con(struct history &history, vector<double> rel_history_list, double *parameters, int source_pos_idx, PlumeModel *model)
{
	//float sigma = 0.22;// works well for Gaussian CDF
	//float sigma = 0.35;// works well for Empirical CDF
	double sum_tmp = 0;
	// if (sampleBuffer.average_wind_intensity == 0)
	// 	printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		idx[i] = floor((px[i] - source_x_min) / x_resol);
		idy[i] = floor((py[i] - source_y_min) / y_resol);
	}

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, source_pos_idx, model_c_avg);
      // // print the source position
	// double candidate_source_pos[2] = {0};
	// posterior.index_to_param_values(source_pos_idx, candidate_source_pos);
	// printf("source position: %f, %f\n", candidate_source_pos[0], candidate_source_pos[1]);
	vector<double> rel_model_list;
	vector<double> pdf_model_list;
	double rel_model_val, rel_hist_val; // mean, stddev,
	if (simulation_parameters.relativeMethod == NORMALIZATION)
	{
		calculate_normalized_relative_measurements(rel_model_list, model_c_avg);
		//sigma = 0.49; //works well for Normalization
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF)
	{
		calculate_measurements_list_empirical_CDF(rel_model_list, model_c_avg); 
		// // print the relative measurements
		// printf("model_c_avg:");
		// for (int i = 0; i < model_c_avg.size(); i++)
		// {	
		// 	printf("%f, ",model_c_avg[i]);
		// }
		// printf("\n");
		// printf("rel_model_list:");
		// for (int i = 0; i < rel_model_list.size(); i++)
		// {	
		// 	printf("%f, ",rel_model_list[i]);
		// }
		// printf("\n");
		//sigma = 0.45;// works well for Empirical CDF
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = rel_history_list[i];
			rel_model_val = rel_model_list[i];
			// if (real_source_idx == soure_pos_idx)
			// 	printf("hist_val:%f, rel_hist_val: %f, model_val: %f, rel_model_val: %f @ pos (%f, %f)\n", history.measurement[i],rel_hist_val, model_c_avg[i], rel_model_val, history.position[i].x, history.position[i].y);
			// sum_tmp += pow(rel_model_val - rel_hist_val, 2)/(sigma * sigma);
			sum_tmp += pow(rel_model_val - rel_hist_val, 2);
		}
		sum_tmp = history.size * sum_tmp/(simulation_parameters.sigma * simulation_parameters.sigma);
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF_NUM){
		calculate_measurements_list_empirical_CDF(rel_model_list, model_c_avg); 
		//sigma = 0.48;// works well for Empirical CDF NUM
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = rel_history_list[i];
			rel_model_val = rel_model_list[i];
			// NOTE: take into account the measurement number information, to avoid the algorithm converge to fast to local minima 
			sum_tmp += history.size * pow(rel_model_val - rel_hist_val, 2)/(history.size + 1);
		}
		sum_tmp /= simulation_parameters.sigma * simulation_parameters.sigma;
	}
     
	return (exp(-0.5 * sum_tmp));
}


void calculate_measurements_list_empirical_CDF(vector<double> &rel_meas_list, vector<double> data_list)
{
	rel_meas_list.resize(data_list.size());
	if (data_list.size() == 1) {
		rel_meas_list[0] = 0.0;
		return;
	}
	vector<double> sortedData = data_list;
	sort(sortedData.begin(), sortedData.end());

	// Calculate ECDF
	vector<double> ecdf(data_list.size());
	for (size_t i = 0; i < data_list.size(); ++i) {
		// Count the number of observations less than or equal to the current value
		size_t count = count_if(sortedData.begin(), sortedData.end(),
							[&](double x) { return data_list[i] - x > 0.1; });
		// Calculate proportion
		rel_meas_list[i] = static_cast<double>(count) / data_list.size();
	}
}

void calculate_normalized_relative_measurements(vector<double> &rel_meas_list , vector<double> data_list)
{
	// normalize the measurements with max and min values
	double max_val = *max_element(data_list.begin(), data_list.end());
	double min_val = *min_element(data_list.begin(), data_list.end());
	// printf("max: %f, min: %f\n", max_val, min_val);

	for (int i = 0; i < data_list.size(); i++)
	{
		if (abs(max_val-min_val)< 0.1)
		{
			rel_meas_list.push_back(0);
		}
		else
		{
			rel_meas_list.push_back((data_list[i] - min_val) / (max_val - min_val));
		}
		// printf("data: %f, rel: %f\n", data_list[i], relative_history_meas[i]);
	}
}

double likelihood_gas_hit(struct history &history, double *parameters, int soure_pos_idx, PlumeModel *model)
{
	double sum_tmp = 0;
	// if (sampleBuffer.average_wind_intensity == 0)
	// 	printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		idx[i] = floor((px[i] - source_x_min) / x_resol);
		idy[i] = floor((py[i] - source_y_min) / y_resol);
	}

	// extract predicted concentration values by model
	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	int history_hit;
	int model_hit;

	double likelihood;
	
	//-----model and measurement fixed threshold, penalty on gas hit event difference-------
	if (simulation_parameters.gasHitMethod == FIXED_THRES){
		for (int i = 0; i < history.size; i++){
			if (history.measurement_avg[i] < simulation_parameters.gas_hit_threshold)
				history_hit = 0;
			else
				history_hit = 1;
			history.gas_hit_event[i] = history_hit;
			if (model_c_avg[i] < simulation_parameters.gas_hit_threshold)
				model_hit = 0;
			else
				model_hit = 1;
		if (history_hit != model_hit)
			sum_tmp += 1;
		}
		sum_tmp *= 0.8; //best result with exp 0.5
		likelihood = (exp(-sum_tmp));
	}


	//-------------adaptive threshold- based on recent measurements------------------------------
	if (simulation_parameters.gasHitMethod == SLIDING_WINDOW_THRES)
	{
		//double lamda = 0.8; //0.5;
            printf("Enter likelihood_gas_hit with sliding window\n");
		double his_thrs = 0;
		double mod_thrs = 0;
		double prev_mod_thrs = 0.0;
		double prev_his_thrs = 0.0;
		// double lambda = 0.8;

		//Dynamic thresholds computation
		for (int i = 0; i < history.size; i++){
			if (i == 0){
				prev_his_thrs = history.measurement_avg[i];
				his_thrs = history.measurement_avg[i];
				prev_mod_thrs = model_c_avg[i];
				mod_thrs = model_c_avg[i];
			}
			his_thrs = simulation_parameters.lambda * prev_his_thrs + (1 - simulation_parameters.lambda) * history.measurement_avg[i];
			mod_thrs = simulation_parameters.lambda * prev_mod_thrs + (1 - simulation_parameters.lambda) * model_c_avg[i];
			//store history threshold for next iteration
			prev_his_thrs = his_thrs;
			prev_mod_thrs = mod_thrs;

			// decide whether the gas hit event happens
			history_hit = (history.measurement_avg[i] > his_thrs) ? 1 : 0;
			history.gas_hit_event[i] = history_hit;
			model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
			// update the likelihood based on the similarity of the hit event
			if (history_hit != model_hit){
				sum_tmp +=1;
			}
		}
		sum_tmp *= 0.8;
		likelihood = (exp(-sum_tmp));
	}

	return likelihood;
}

bool prior(double *parameters, int nbr_dimension, PlumeModel *model){
	int i = 0;
	bool output = 1;
	
	// verify if all the parameters are between the right intervals
	for(i = 0; i < nbr_dimension; i++)
		output = output && (parameters[i] < float(model->param_list[i].upper_limit) && parameters[i] >= float(model->param_list[i].lower_limit));
	
	// verify the x and y elements of the parameters are not inside obstacles
	Position tmp_p(parameters[0], parameters[1], 0);
	if(tmp_p.is_in_obstacle_list(obstacles, nbr_obs) != -1){
		//printf("point %f %f is in obstacle\n", parameters[0], parameters[1]);
		output = 0;
	}

	//if(output == 0)
	//	printf("parameters: %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4]);
	
	return(output);
}


// ======================= NAVIGATION functions ========================= //
void random_planning()
{
	printf("Enter random planning \n");
	fprintf(console_logfile, "Enter random planning \n");
	do
	{
		goal_position.heading = 0;
		goal_position.x = (((double)rand() / (double)RAND_MAX) * (XMAX - XMIN - SAFETY_MARGIN * 2)) + SAFETY_MARGIN + XMIN;
		goal_position.y = (((double)rand() / (double)RAND_MAX) * (YMAX - YMIN - SAFETY_MARGIN * 2)) + SAFETY_MARGIN + YMIN;
	} while (goal_position.is_in_obstacle_list(obstacles, nbr_obs) != -1);
}

bool position_outside_arena(Position p){
	bool outside = 0;
	if(p.x < float(XMIN) || p.x > float(XMAX)) outside = 1;
	if(p.y < float(YMIN) || p.y > float(YMAX)) outside = 1;
	//if(position[2] < ZMIN || position[2] > ZMAX) outside = 1;
	return outside;
}

void information_gain_navigation_approx(){
	float p0 = 0.5; //50 cm
	double KLD=0, max_KLD=-100, max_meas=-100;
	//double position[1][3] = {{0}};
	Position tmp_position;
	struct history fake_history;
	fake_history.position = history.position;
	fake_history.measurement_avg = history.measurement_avg;
	fake_history.size = history.size;

	// add fake prediction for friends goal position, so we don't choose the same ones
	for(int i = 0; i < simulation_parameters.number_of_robots; i++){
		if(friends[i].ID == 0 || friends[i].ID  == ID) continue;
		fake_history.position[fake_history.size].copy_position(friends[i].goal);
		fake_history.measurement_avg[fake_history.size] = plume_model.approx_concentration(friends[i].goal, sampleBuffer.average_wind_intensity);
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
			//calculate the expected concentration on position
			fake_history.measurement_avg.push_back(plume_model.approx_concentration(tmp_position, sampleBuffer.average_wind_intensity));
			//std::cout<<"fake_history.measurement = "<<fake_history.measurement_avg[fake_history.size]<<std::endl;
			fake_history.position.push_back(tmp_position);
			(fake_history.size)++;
			DeepCFDPlumeModel2D plume_model_fake;
			Posterior future_posterior(&plume_model_fake, 0);
			//future_posterior.copy_posterior(zero);
			future_posterior.MCMC_evaluation(fake_history);
			//calculate the expected information gain
			KLD = posterior.Kullback_Leibler_divergence(future_posterior);
			// printf("KLD: %f _______________________\n", KLD);
			if(KLD > max_KLD ){
				max_KLD = KLD;
				goal_position.x = tmp_position.x; // + (randu()-0.5)/5.0
				goal_position.y = tmp_position.y;
				goal_position.heading = 0;
			}
            fake_history.measurement_avg.pop_back();
			fake_history.position.pop_back();
			fake_history.size--;
	}
}

void balance_exploration_exploitation(float global_local_search_ratio){
	float ratio = global_local_search_ratio;//simulation_parameters.global_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
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


void calculate_information_gain()
{
	if (simulation_parameters.infoFeature == VARIANCE)
	{
		plume_model.calculate_weighted_gas_con_variance_map(gas_information_map, posterior.pdf);
	}
	if (simulation_parameters.infoFeature == MEAN)
	{
		plume_model.calculate_weighted_gas_con_mean_map(gas_information_map, posterior.pdf);
	}
	if (simulation_parameters.infoFeature == PDF)
	{
		for (int i = 0; i < x_nbins * y_nbins; i++)
		{
			gas_information_map[i] = posterior.pdf[i];
		}
	}
	// printf("gas information map is: \n");
	// for (int i = 0; i < x_nbins * y_nbins; i++)
	// {
	// 	printf("%f, ", gas_information_map[i]);
	// }
	// printf("\n");
}

void reset_metrics(){
    for (int i = 0; i < x_nbins * y_nbins; i++)
    {
	  acc_info_map[i] = 0.0;
	  distance_map[i] = 0.0;
	  scores_map[i] = 0.0;
    }
}

void maximal_information_planning()
{
	printf("Enter maximal information planning \n");
	fprintf(console_logfile, "Enter maximal information planning \n");
	calculate_information_gain();
	double best_point[2] = {0};
	// find the maximal variance in gas variance map
	double max_info = 0;
	int max_info_index = 0;
	int sum = 0;
	for (int i = 0; i < x_nbins * y_nbins; i++)
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			if (gas_information_map[i] > max_info && cell_visited_list[i] == 0)
			{ 
				max_info = gas_information_map[i];
				max_info_index = i;
			}		
		}
		else
		{
			if (gas_information_map[i] > max_info)
			{
				max_info = gas_information_map[i];
				max_info_index = i;
			}		
		}
	}
	fprintf(console_logfile, "max_info : %f , max_info_iter: %d\n", max_info, max_info_index);

      double epsilon = 1e-9; 
      if (fabs(max_info - 0.0) < epsilon) // all the informative points have been visited
	{
		printf("all the informative points have been visited \n");
		fprintf(console_logfile, "all the informative points have been visited \n");
		random_planning();
		printf("goal position : %f %f \n", goal_position.x, goal_position.y);
		return;
	}

	posterior.index_to_param_values(max_info_index, best_point);

	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;
}

void maximal_belief_planning()
{
	printf("Enter maximal belief planning \n");
	fprintf(console_logfile, "Enter maximal belief planning \n");
	double best_point[2] = {0};
	// find the maximal variance in gas variance map
	double max_pdf = 0;
	int max_pdf_index = 0;
	int sum = 0;
	for (int i = 0; i < x_nbins * y_nbins; i++)
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			if (posterior.pdf[i] > max_pdf && cell_visited_list[i] == 0)
			{ 
				max_pdf = posterior.pdf[i];
				max_pdf_index = i;
			}		
		}
		else
		{
			if (posterior.pdf[i] > max_pdf)
			{
				max_pdf = posterior.pdf[i];
				max_pdf_index = i;
			}		
		}
	}
	fprintf(console_logfile, "max_pdf : %f , max_pdf_iter: %d\n", max_pdf, max_pdf_index);

    double epsilon = 1e-9; 
    if (fabs(max_pdf - 0.0) < epsilon) // all the possible points have been visited
	{
		printf("all the probable points have been visited \n");
		fprintf(console_logfile, "all the probable points have been visited, use maximal information planning \n");
		maximal_information_planning();
		printf("goal position : %f %f \n", goal_position.x, goal_position.y);
		return;
	}
	posterior.index_to_param_values(max_pdf_index, best_point);

	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;
}


vector<pair<CellPoint, CellPoint>> clustering_information_blocks(double gas_information_map[]){
	// dbscan clustering
	// param: x_eps, y_eps, minPts, ratio
	DBSCAN dbscan(x_resol + 0.1, y_resol + 0.1, 4, (1.0-posterior.entropy/simulation_parameters.cluster_entropy_ratio));
	dbscan.run(gas_information_map);
	return dbscan.cluster_bounding_boxes;
}

void belief_clustering_planning()
{
	printf("Enter belief clustering planning \n");
	
	vector<pair<CellPoint, CellPoint>> clusters_bounding_box;
	clusters_bounding_box = clustering_information_blocks(posterior.pdf);
	if (clusters_bounding_box.size() == 0)
	{
		printf("no cluster found \n");
		fprintf(console_logfile, "no cluster found \n");
		maximal_belief_planning();
		//maximal_information_planning();
		return;
	}
	else
	{
		// fprintf(console_logfile, "clusters found \n");
		// Find furthest vertex of each cluster
		double max_dist_to_robot;
		int max_dist_to_robot_index;
		vector<Position> candidate_goal;
		for (int i = 0; i < clusters_bounding_box.size(); i++)
		{
			max_dist_to_robot = 0;
			max_dist_to_robot_index = 0;
			// calculate the distance of each vertex to robot
			Position cluster_vertex[4] = {Position(clusters_bounding_box[i].first.x, clusters_bounding_box[i].first.y, 0),
								Position(clusters_bounding_box[i].first.x, clusters_bounding_box[i].second.y, 0),
								Position(clusters_bounding_box[i].second.x, clusters_bounding_box[i].first.y, 0),
								Position(clusters_bounding_box[i].second.x, clusters_bounding_box[i].second.y, 0)};
			for (int i = 0; i < 4; i++)
			{
				double dist = dist_to_goal(cluster_vertex[i]);
				if (dist > max_dist_to_robot)
				{
					max_dist_to_robot = dist;
					max_dist_to_robot_index = i;
				}
			}
			candidate_goal.push_back(cluster_vertex[max_dist_to_robot_index]);
		}
		// find the closest unvisited point among candidate goals
		double min_dist_to_robot = 1000;
		int min_dist_to_robot_index = 0;
		bool find_goal = false;
		for (int j = 0; j < candidate_goal.size(); j++)
		{
			double dist = dist_to_goal(candidate_goal[j]);
			if (dist < min_dist_to_robot)
			{
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int pos_coord[2];
					pos_coord[0] = floor((candidate_goal[j].x - source_x_min) / x_resol);
					pos_coord[1] = floor((candidate_goal[j].y - source_y_min) / y_resol);
					int idx = posterior.indices_to_index(pos_coord);
					if (cell_visited_list[idx] == 0)
					{
						min_dist_to_robot = dist;
						min_dist_to_robot_index = j;
						find_goal = true;
					}
				}
				else
				{
					min_dist_to_robot = dist;
					min_dist_to_robot_index = j;
					find_goal = true;
				}
			}
		}
		if (!find_goal)
		{
			fprintf(console_logfile, "all the vertex of clusters are visited, use maximal belief planning \n");
			//maximal_information_planning();
			maximal_belief_planning();
			return;
		}
		else
		{
			goal_position.x = candidate_goal[min_dist_to_robot_index].x;
			goal_position.y = candidate_goal[min_dist_to_robot_index].y;
			goal_position.heading = 0;
		}

		// fprintf(console_logfile, "clustering goal position : %f %f \n", goal_position.x, goal_position.y);
		// fprintf(console_logfile, "current position : %f %f \n", current_position.x, current_position.y);
	}
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

// **Dijkstra’s Algorithm with Path Tracking**

// Custom Comparator for Priority Queue
struct ComparePosition {
    bool operator()(const std::pair<double, Position>& a, const std::pair<double, Position>& b) {
        return a.first > b.first; // Min-heap: prioritize lower cost
    }
};

std::pair<std::vector<double>, std::unordered_map<int, Position>> 
computeDijkstra(Position start, std::vector<std::tuple<int, int, double>> directions) {
    
    std::unordered_map<int, Position> cameFrom; // Stores the parent of each cell
    std::priority_queue<std::pair<double, Position>, 
                        std::vector<std::pair<double, Position>>, 
                        ComparePosition> pq;

    std::vector<double> dist_map(x_nbins * y_nbins, 1e3);
    dist_map[start.index] = 0;
    pq.push({0, start});
    while (!pq.empty()) {
        auto [currentDist, pos] = pq.top();
        pq.pop();
	
        int x = static_cast<int>(pos.x_idx);
        int y = static_cast<int>(pos.y_idx);
        
        // Expand neighbors
        for (auto [dx, dy, rel_dist] : directions) {
            int nx = x + dx, ny = y + dy;
		Position neg(nx, ny); 
            if (nx >= 0 && ny >= 0 && nx < x_nbins && ny < y_nbins && neg.is_in_obstacle_list(obstacles, nbr_obs)==-1) {
                double newDist = currentDist + rel_dist;
		    if (newDist < dist_map[neg.index]) {
                    dist_map[neg.index] = newDist;
                    pq.push({newDist, neg});
                    cameFrom[neg.index] = {x, y};  // Store parent
                }
            }
        }
    }
    return {dist_map, cameFrom};
}

// **Reconstruct Path from Dijkstra Results**
std::list<Position> reconstructPath(Position goal, const std::unordered_map<int, Position>& cameFrom) {
    std::list<Position> dij_path;
    Position current = goal;
    while (cameFrom.find(current.index) != cameFrom.end()) {
        dij_path.push_back(current);
        current = cameFrom.at(current.index);
    }
    dij_path.push_back(current);  // Add the starting position
    std::reverse(dij_path.begin(), dij_path.end()); // Reverse to get the correct order
    return dij_path;
}

void markNeighboringCellsAsVisited(int x, int y, int x_nbins_, int y_nbins_, int radius_neighbor, int *cell_visited_list)
{
	// Parcours des cellules dans le voisinage
	for (int dx = -radius_neighbor; dx <= radius_neighbor; ++dx)
	{
		for (int dy = -radius_neighbor; dy <= radius_neighbor; ++dy)
		{
			int neighbor_x = x + dx;
			int neighbor_y = y + dy;

			// Vérifier si la cellule voisine est dans les limites de la grille
			if (neighbor_x >= 0 && neighbor_x < x_nbins_ && neighbor_y >= 0 && neighbor_y < y_nbins_)
			{
				// Calculer l'index dans le tableau
				int index = neighbor_y * x_nbins_ + neighbor_x;
				// Marquer la cellule voisine comme visitée
				cell_visited_list[index] = 1;
			}
		}
	}
}

// **Main Function: Compute Accumulative Information Gain Using Dijkstra**
void accumulative_Dijkstra() {
     calculate_information_gain();
     reset_metrics();
    // Directions for a 8-connected grid
    std::vector<std::tuple<int, int, double>> directions = {{-1, 0, index_distance(-1,0)}, {1, 0, index_distance(1,0)},
     {0, -1, index_distance(0,-1)}, {0, 1, index_distance(0,1)}, {-1, -1, index_distance(-1,-1)},
     {-1, 1, index_distance(-1,1)}, {1, -1, index_distance(1,-1)}, {1, 1, index_distance(1,1)}};
    // Compute shortest distances and paths from current position to all cells
    auto [distanceMap, cameFrom] = computeDijkstra(current_position, directions);

    double bestScore = -1000; // Initialize the best score
    Position bestTarget;
    std::list<Position> bestPath;

    double min_acc_info = 1e9, max_acc_info = -1e9;
    double min_dist = 1e9, max_dist = -1e9;

    int radius_neighbor = 2;

    for (int y_idx = 0; y_idx < y_nbins; y_idx++) {
        for (int x_idx = 0; x_idx < x_nbins; x_idx++) {
		Position cell(x_idx,y_idx);
            if (cell.is_in_obstacle_list(obstacles, nbr_obs)!=-1) continue; // Skip obstacles

            double pathLength = distanceMap[cell.index];

            if (pathLength == 1e3) continue; // Unreachable cell
 
            // **Retrieve the path to this cell**
            std::list<Position> dij_path = reconstructPath(cell, cameFrom);
            if (dij_path.empty()){
			std::cout<<"no valid path exists";
			continue; // Skip if no path exists	
		} 

            // **Accumulate total information gain along the path**
            double totalInfo = 0.0;
		if (simulation_parameters.infoSum == LINEAR_SUM){
			for (const auto& pos : dij_path) {		    
				totalInfo += gas_information_map[pos.index]; // Sum information along the path
			}
		}
		if (simulation_parameters.infoSum == EXP_SUM){
			for (const auto& pos : dij_path) {		    
				totalInfo += exp(3*gas_information_map[pos.index])-1; // Sum information along the path
			}
		}
		
		acc_info_map[cell.index] = totalInfo;
		distance_map[cell.index] = pathLength;

		// Update min/max values
		min_acc_info = std::min(min_acc_info, totalInfo);
		max_acc_info = std::max(max_acc_info, totalInfo);
		min_dist = std::min(min_dist, pathLength);
		max_dist = std::max(max_dist, pathLength);
        }
    }

    for (int y_idx = 0; y_idx < y_nbins; y_idx++) {
        for (int x_idx = 0; x_idx < x_nbins; x_idx++) {
		Position cell(x_idx,y_idx);
            if (cell.is_in_obstacle_list(obstacles, nbr_obs)!=-1) continue; // Skip obstacles

            if (max_acc_info != min_acc_info) {
            	acc_info_map[cell.index] = (acc_info_map[cell.index] - min_acc_info) / (max_acc_info - min_acc_info);
        	} else {
            	acc_info_map[cell.index] = 0; // Avoid division by zero
       	}
		double totalInfo = acc_info_map[cell.index];

        	if (max_dist != min_dist) {
            	distance_map[cell.index] = (distance_map[cell.index] - min_dist) / (max_dist - min_dist);
       	} else {
            	distance_map[cell.index] = 0; // Avoid division by zero
        	}
		double pathLength = distance_map[cell.index];
            
            if (totalInfo < 1e-2) continue; // Skip no information gain cells
		// skip visited cells
		if (simulation_parameters.visitedList == visitedList_ON){
			if (cell_visited_list[cell.index] == 1)
				continue;
		}

		// **Compute the score based on information-to-distance trade-off**
            double score;
            if (simulation_parameters.scoreFun == RATIO)
		{
			score = (simulation_parameters.scoreSigma * totalInfo) / std::max(0.0, pathLength);
		}
		if (simulation_parameters.scoreFun == LOG)
		{
			score = (simulation_parameters.scoreSigma * totalInfo) / std::log(std::max(0.0, pathLength) + 1);
		}
		if (simulation_parameters.scoreFun == SIGMOID)
		{
			score = 1 / (1 + exp(-simulation_parameters.scoreSigma * totalInfo)) - 1 / (1 + exp(-0.1 * std::max(0.0, pathLength)));
		}
		if (simulation_parameters.scoreFun == SUM)
		{
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_FIX)
			{
				score = simulation_parameters.scoreSigma * totalInfo - (1.0 - simulation_parameters.scoreSigma) * pathLength;
			}
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_ADP){
				// in the beginning, when the uncertainty is high, the dynamic weight is smaller
				double dynamic_weight = 1.0 - posterior.entropy / simulation_parameters.cluster_entropy_ratio;
				if (dynamic_weight < 0.0)
					dynamic_weight = 0.0;
				score = dynamic_weight * totalInfo - (1.0 - dynamic_weight) * pathLength;
			}
		}
		if (simulation_parameters.scoreFun == EXP)
		{
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_FIX)
			{
				score = simulation_parameters.scoreSigma * (exp(totalInfo) - 1.0) - (1.0 - simulation_parameters.scoreSigma) * pathLength;
			}
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_ADP){
				// in the beginning, when the uncertainty is high, the dynamic weight is smaller
				double dynamic_weight = 1.0 - posterior.entropy / simulation_parameters.cluster_entropy_ratio;
				if (dynamic_weight < 0.0)
					dynamic_weight = 0.0;
				score = dynamic_weight * (exp(totalInfo) - 1.0) - (1.0 - dynamic_weight) * pathLength;
			}
		}

            scores_map[cell.index] = score;

            if (score > bestScore) {
		    std::list<Position> dij_path = reconstructPath(cell, cameFrom);
		    if (dij_path.empty()){
				std::cout<<"no valid path exists";
				continue; // Skip if no path exists	
		    } 
                bestScore = score;
                bestTarget.copy_position(cell);
                bestPath = dij_path;
            }
	  }
    }

    log_scores_map();
    log_distance_map();
    log_acc_info_map();

    if (bestScore > -1e3) {
        std::cout << "Best cell: (" << bestTarget.x_idx << ", " << bestTarget.y_idx << ") with score: " << bestScore << std::endl;
        std::cout << "Optimal Path: ";
        for (const auto& pos : bestPath) {
            std::cout << "(" << pos.x << ", " << pos.y << ") -> ";
        }
        std::cout << "Goal\n";
	  goal_position.x = bestTarget.x;
	  goal_position.y = bestTarget.y;
	  cell_visited_list[bestTarget.index] = 1;
	  markNeighboringCellsAsVisited(bestTarget.x_idx, bestTarget.y_idx, x_nbins, y_nbins, radius_neighbor, cell_visited_list);
    } else {
        std::cout << "No valid path found.\n";
    }
}


void KLD_MAP_distance(float global_local_search_ratio){
	double best_point[2]={0};
	posterior.XYmarginal_distance_combined(current_position, global_local_search_ratio, best_point);
	printf("greedy goal %f %f\n", goal_position.x, goal_position.y);
	printf("best point %f %f\n", best_point[0], best_point[1]);

	float ratio = 0.3;//global_local_search_ratio;//simulation_parameters.global_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
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

void goal_outside_obstacles(Position *goal)
{

	double min_dist_to_current = 0.05 + simulation_parameters.sampling_dist_thresh;
	// is the goal position in an obstacle or out of arena or too close to current position?
	int idx = goal->is_in_obstacle_list(obstacles, nbr_obs);
	if (idx == -1 && !position_outside_arena(*goal) && goal->distance_from(current_position) > min_dist_to_current)
	{
		printf(">>> the goal is not in any obstacles or outside of the region\n");
		return;
	}

	// goal inside obstacle
	if (idx != -1)
	{
		// mark the cell as visited so that never plans to go there again
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			int pos_coord[2];
			pos_coord[0] = floor((goal->x - source_x_min) / x_resol);
			pos_coord[1] = floor((goal->y - source_y_min) / y_resol);
			int idx = posterior.indices_to_index(pos_coord);
			cell_visited_list[idx] = 1;
		}
		printf(">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);
            fprintf(console_logfile, ">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);	
		// otherwise, lets try 4 different candidates
		float min_dist = 10000.0;
		float min_dist_unvisited = 10000.0;
		Position nearest_position(*goal);
		Position nearest_unvisited_position(*goal); // for testing priority to unvisited cells
		Position candidates[4] = {Position(obstacles[idx][0], goal->y, 0),
						  Position(obstacles[idx][2], goal->y, 0),
						  Position(goal->x, obstacles[idx][1], 0),
						  Position(goal->x, obstacles[idx][3], 0)};

		int unvisited = 0;

		// for each candidate position
		for (int i = 0; i < 4; i++)
		{
			// check if the distance to candidate is smaller than others
			//  printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
			if (goal->distance_from(candidates[i]) < min_dist)
			{
				if (candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1)
				{
					// not in obstacle
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
				}
				else
				{
					if (simulation_parameters.visitedList == visitedList_ON)
					{
						// mark the cell as visited so that never plans to go there again
						int pos_coord[2];
						pos_coord[0] = floor((candidates[i].x - source_x_min) / x_resol);
						pos_coord[1] = floor((candidates[i].y - source_y_min) / y_resol);
						int idx = posterior.indices_to_index(pos_coord);
						cell_visited_list[idx] = 1;
					}
				}
			}
		}
		// change the goal position using the pointer
		printf(">>> Obstacle Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		// fprintf(console_logfile, ">>> Obstacle Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);

		goal->copy_position(nearest_position);
	}

	// oustide arena
	if (position_outside_arena(*goal))
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			// mark the cell as visited so that never plans to go there again
			int pos_coord[2];
			pos_coord[0] = floor((goal->x - source_x_min) / x_resol);
			pos_coord[1] = floor((goal->y - source_y_min) / y_resol);
			int idx = posterior.indices_to_index(pos_coord);
			cell_visited_list[idx] = 1;
		}
		printf(">>> ! position %f %f is outside the region\n", goal->x, goal->y);
		fprintf(console_logfile, ">>> ! position %f %f is outside the region\n", goal->x, goal->y);
		// otherwise, lets try 8 different candidates
		float min_dist = 10000.0;
		float min_dist_unvisited = 10000.0;
		Position nearest_position(*goal);
		Position nearest_unvisited_position(*goal); // to give priority to unvisited cells
		Position candidates[8] = {Position(float(XMIN + SAFETY_MARGIN), goal->y, 0),
						  Position(float(XMAX - SAFETY_MARGIN), goal->y, 0),
						  Position(goal->x, float(YMIN + SAFETY_MARGIN), 0),
						  Position(goal->x, float(YMAX - SAFETY_MARGIN), 0),
						  Position(float(XMIN + SAFETY_MARGIN), float(YMIN + SAFETY_MARGIN), 0),
						  Position(float(XMIN + SAFETY_MARGIN), float(YMAX - SAFETY_MARGIN), 0),
						  Position(float(XMAX - SAFETY_MARGIN), float(YMIN + SAFETY_MARGIN), 0),
						  Position(float(XMAX - SAFETY_MARGIN), float(YMAX - SAFETY_MARGIN), 0)};
		// for each candidate position
		for (int i = 0; i < 8; i++)
		{

			// check if the distance to candidate is smaller than others
			if (goal->distance_from(candidates[i]) < min_dist)
			{
				if (candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i]))
				{
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
				}
				else
				{
					if (simulation_parameters.visitedList == visitedList_ON)
					{
						int pos_coord[2];
						pos_coord[0] = floor((candidates[i].x - source_x_min) / x_resol);
						pos_coord[1] = floor((candidates[i].y - source_y_min) / y_resol);
						int idx = posterior.indices_to_index(pos_coord);
						cell_visited_list[idx] = 1;
					}
				}
			}
		}
		// fprintf(console_logfile, ">>> Outside Arena Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}
	// make sure that the chosen goal is not too close from the robot current position so that it will not instantaneously be reached
	double dist_candidate_goal = goal->distance_from(current_position);
	if (goal->distance_from(current_position) < min_dist_to_current)
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			// mark the cell as visited so that never plans to go there again
			int pos_coord[2];
			pos_coord[0] = floor((goal->x - source_x_min) / x_resol);
			pos_coord[1] = floor((goal->y - source_y_min) / y_resol);
			int idx = posterior.indices_to_index(pos_coord);
			cell_visited_list[idx] = 1;
		}
		printf(">>> ! position %f %f is too close to current position\n", goal->x, goal->y);
		fprintf(console_logfile, ">>> ! position %f %f is too close to current position\n", goal->x, goal->y);
		//  otherwise, lets try 4 different candidates
		Position nearest_position(*goal);
		Position candidates[8] = {Position(goal->x + min_dist_to_current, goal->y + min_dist_to_current, 0),
						  Position(goal->x + min_dist_to_current, goal->y - min_dist_to_current, 0),
						  Position(goal->x - min_dist_to_current, goal->y - min_dist_to_current, 0),
						  Position(goal->x - min_dist_to_current, goal->y + min_dist_to_current, 0),
						  Position(goal->x + min_dist_to_current, goal->y, 0),
						  Position(goal->x - min_dist_to_current, goal->y, 0),
						  Position(goal->x, goal->y + min_dist_to_current, 0),
						  Position(goal->x, goal->y - min_dist_to_current, 0)};
		// for each candidate position
		for (int i = 0; i < 8; i++)
		{
			if (candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i]) && current_position.distance_from(candidates[i]) > min_dist_to_current)
			{
				// Take the first candidate that satisfies the condition
				nearest_position.copy_position(candidates[i]);
				int pos_coord[2];
				pos_coord[0] = floor((candidates[i].x - source_x_min) / x_resol);
				pos_coord[1] = floor((candidates[i].y - source_y_min) / y_resol);
				int idx = posterior.indices_to_index(pos_coord);
				if (cell_visited_list[idx] == 0)
				{
					break; // give priority to unvisited cell
				}
				// printf(">>> chose it as new position !!! \n");
			}
			else
			{
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int pos_coord[2];
					pos_coord[0] = floor((candidates[i].x - source_x_min) / x_resol);
					pos_coord[1] = floor((candidates[i].y - source_y_min) / y_resol);
					int idx = posterior.indices_to_index(pos_coord);
					cell_visited_list[idx] = 1;
				}
			}
		}
		fprintf(console_logfile, ">>> Too close Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
		fprintf(console_logfile, ">>> Outside arena Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
	}
	
}

// ============================== STATES ================================= //
void state_waiting(){
	// initial waiting for the plume to get established
	static int counter = 0;
	counter++;
	// printf("waiting\n");

	if(counter > 3){
		FSM.state = STATE_LISTEN;
		FSM.hook  = state_listen;
	}
}

void state_estimating(){
	if(VERBOSE) printf("[%d] estimating...\n", ID);
	time_before_estimation = get_exp_time();
	//reinitialize
	posterior.pdf_reset();
	//estimation
	if(simulation_parameters.MCMC_or_exhaustive_eval == MCMC){
		printf("going for MCMC\n");
		posterior.MCMC_evaluation(history);
	}
	if(simulation_parameters.MCMC_or_exhaustive_eval == EXHAUSTIVE){
		printf("going for exhaustive evaluation \n");
		posterior.exhaustive_evaluation(history);
	}
		
	if(posterior.dirac_flag){
		if(VERBOSE) printf("[%d] exhaustive search!\n", ID);
		posterior.exhaustive_evaluation(history);
	}
	
	printf("sum posterior : %f \n", posterior.sum);
	posterior.process();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	//posterior.log_pdf("/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/current/101/posterior.csv",'a');

	FSM.n_cycles++;
	if(VERBOSE) printf("[%d] cycle number %d\n", ID, FSM.n_cycles);
	if(VERBOSE) printf("[%d] entropy : %f\n", ID, posterior.entropy);
	
	//check if time is up
	//if(FSM.n_cycles == simulation_parameters.max_iter || posterior.entropy < 0.1){
	if(FSM.n_cycles == simulation_parameters.max_iter || (posterior.entropy < 0.3 && posterior.x_spatial_spread < 2 && posterior.y_spatial_spread < 2)){
		//the end
		FSM.state = STATE_FINISHED;
		end_timestamp = get_exp_time();
	} else {
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;
		printf("go to update goal\n");
	}
	time_after_estimation = get_exp_time();
	estimation_computation_time_sum += time_after_estimation - time_before_estimation;
	printf("estimation time: %f\n", time_after_estimation - time_before_estimation);
}

void state_updategoal(){
	time_before_planning = get_exp_time();
	if(VERBOSE) printf("[%d] updating the goal\n", ID);
      
      switch (simulation_parameters.planningStrategy)
	{
	case GREEDY:
		maximal_belief_planning();
		break;
	case RANDOM:
		random_planning();
		break;
	case BELIEF_CLUSTERING:
		belief_clustering_planning();
		break;
	case INFORMATIVE_KLD:
		information_gain_navigation_approx();
	      KLD_MAP_distance(1.0);
		break;
    case SUM_DIJKSTRA_FIX:
	      accumulative_Dijkstra();
		break;
    case SUM_DIJKSTRA_ADP:
	      accumulative_Dijkstra();
		break;
	default:
		break;
	}
      // The goal position is very close to current postion, swith to random planning
	// if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<double>(simulation_parameters.sampling_dist_thresh)))
	// {
	// 	printf("The goal position is very close to current postion, switch to random planning \n");
	// 	fprintf(console_logfile, "The goal position is very close to current postion, switch to random planning \n");
	// 	random_planning();
	// }

	goal_outside_obstacles(&goal_position);

	if (simulation_parameters.workingMode == VIDEO){
		state_read_goal();
	}

      printf("from %f %f to goal %f %f \n", current_position.x, current_position.y, goal_position.x, goal_position.y);
	switch(simulation_parameters.workingMode){
		case EXPERIMENT:
		{
			//send the goal to the supervisor
			Message msg(ID, 0, goal_position, -1, goal_position, 0);
			send_msg(msg);

			// send message to client
			double g_buffer[2] = {0};
			int timeout = 10;
			int success = -1;

			//send the message
			send_goal_to_robot(goal_position.x, goal_position.y);
			time_after_planning = get_exp_time();
			printf("planning time: %f\n", time_after_planning - time_before_planning);
			planning_computation_time_sum += time_after_planning - time_before_planning;
		
			sent_first_goal = true;

			//move to listen
			FSM.state = STATE_LISTEN;
			FSM.hook = state_listen;
			break;

		}
		case VIDEO:
		{
			//send the goal to the supervisor
			Message msg(ID, 0, goal_position, -1, goal_position, 0);
			send_msg(msg);

			// send message to client
			double g_buffer[2] = {0};
			int timeout = 10;
			int success = -1;

			//send the message
			send_goal_to_robot(goal_position.x, goal_position.y);
			time_before_moving_and_measuring = get_exp_time();
			sent_first_goal = true;

			//move to listen
			FSM.state = STATE_LISTEN;
			FSM.hook = state_listen;
			break;
		}

			
		case OFFLINE:
			FSM.state = STATE_READ_ALL;
			FSM.hook  = state_read_goal_measurement;
			break;
		default:
			break;
	}

    
	 
} 


void state_listen(){
	time_before_moving_and_measuring = get_exp_time();
	int success_recv_meas = -1;
	int success_recv_size = -1;
      double buffer[5] = {0};
	int measurement_size = 0; 
    // first recive the size of the message queue
	success_recv_size = read(sock, &measurement_size, sizeof(int));
	printf("received Gas measurement size is: %d \n", measurement_size);
	if (success_recv_size > 0){
		//TODO - check here: whether should be > 0 or > -1
		while (measurement_size-- > 0){
			// wait for gas measurements
			success_recv_meas = read(sock, buffer, sizeof(double)*5); 
			//printf("received Gas value %f %f %f %f\n", buffer[0], buffer[1], buffer[2], buffer[3]); 
			// printf("listening..\n");
			 
			if(success_recv_meas > 0){
				printf("received Gas value %f: (%f, %f) %f +- %f\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]); 
				// save in history
				history.position.push_back({float(buffer[1]), float(buffer[2]), 0});
				history.measurement_avg.push_back(buffer[3]);
				history.measurement_std.push_back(buffer[4]);
				history.size++;
				
				// check if all was received so far
				if(history.size != buffer[0] + 1){
					printf("Something wrong in history...\n");
					printf("history size is %d, buffer[0] is %f \n", history.size, buffer[0]);
				}
			}
			//save current position
			current_position.x = buffer[1];
			current_position.y = buffer[2];
			current_position.heading = 0;
	      } 
		current_position.calculate_index();
		//move to estimating
		history_print(history);
		FSM.state = STATE_ESTIMATING;
		FSM.hook  = state_estimating;
		time_after_moving_and_measuring = get_exp_time();
            if (sent_first_goal) {
			printf("navigation sampling time: %f\n", time_after_moving_and_measuring - time_before_moving_and_measuring);
			navigation_sampling_time_sum += time_after_moving_and_measuring - time_before_moving_and_measuring;
		}
	}
	else{
		printf("nothing yet .. \n");
	}
}

void state_read_goal_measurement(){
	printf("Enter state read goal and measurment\n");
    int d;
	float f1, f2, f3, f4, f5, f6, f7, f8; 
    double t;
	char line[256];

    // Read the goal position
	if (!feof(experiment_goalposition_logfile)){
		fscanf(experiment_goalposition_logfile, "%d, %f, %f, %f, %lf", &d, &f1, &f2, &f3, &t);
		goal_position.x = f1;
		goal_position.y = f2;
		goal_position.heading = 0;
		printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);

		while (dist_to_goal(goal_position) > 0.05){
			fscanf(experiment_measurements_logfile, "%d, %f, %f", &d, &f4, &f5);
			history.size ++;
			history.measurement_avg.push_back(f4);
			history.measurement_std.push_back(f5);
			fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f6, &f7, &f8);
			history.position.push_back({f6, f7, f8});
			current_position.x = f6;
			current_position.y = f7;
			current_position.heading = f8;
		}

		// NOTE Read from a scan
		// printf("I am here \n");
		// fscanf(scan_measurements_logfile, "%d, %f, %f, %f", &d, &f1, &f2, &f4);
		// history.size ++;
		// history.measurement_avg[history.size] = f2;
		// if (history.measurement_avg[history.size] < 0.0)
		// 	history.measurement_avg[history.size] = 0.0;
		// history.measurement_std[history.size] = 0.0;

		// fscanf(scan_position_logfile, "%d, %f, %f, %f, %f", &d, &f1, &f2, &f3, &f4);
		// history.position[history.size].x = f1;
		// history.position[history.size].y = f2;

		// // Move to estimating
		history_print(history);
		FSM.state = STATE_ESTIMATING;
		FSM.hook = state_estimating;

	} else{
		printf("End of experiment measurements file\n");
		FSM.state = STATE_FINISHED;

	}
	

}

void state_read_goal(){
	printf("Enter state read goal and measurment\n");
	int d;
	float f1, f2, f3; 
	double t;
	char line[256];

    	// Read the goal position
	if (!feof(experiment_goalposition_logfile)){
		fscanf(experiment_goalposition_logfile, "%d, %f, %f, %f, %lf", &d, &f1, &f2, &f3, &t);
		goal_position.x = f1;
		goal_position.y = f2;
		goal_position.heading = 0;
		printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);
	} else{
		printf("End of experiment measurements file\n");
		FSM.state = STATE_FINISHED;

	}

}


// =========================== MAIN ============================== //


int main(int argc, char **argv){
	// python instance (should be called ONLY ONCE and only in main)
	//CPyInstance hInstance;

	init();
	if (simulation_parameters.workingMode == EXPERIMENT || simulation_parameters.workingMode == VIDEO){
		init_TCP_socket();
		printf("finish init TCP. \n");
	}

	read_obstacle_list();

	// visibility graph (visilibity library)
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
    	exit(1);
  	}
  	VisiLibity::Visibility_Graph vg(my_environment, epsilon);
	
	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	plume_model.initialize_information_matrix();
	//plume_model.log_gas_concentration_map(gas_concentration_map_logfile_name, 'a');

	// File for saving console messages
	console_logfile = fopen(console_logfile_name, "a");

	// main loop
	start_timestamp = get_exp_time();
	while (FSM.state != STATE_FINISHED){
	   	//change state
		//printf("state %d\n", FSM.state);
		FSM.hook();
		//next
		next_iteration_time();
		n_time_step++;
	}
	// final logs
	printf("Find the source!\n");
	log_history();
	log_experiment_time();
	cleaning_up();
	//free the allocated memory to obstacle array
	for(int i = 0; i < nbr_obs; i++)
		delete[] obstacles[i];
		delete[] obstacles;
		exit(0);
}

