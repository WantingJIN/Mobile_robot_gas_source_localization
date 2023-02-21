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


// read data from file
char logfile_folder_name[256];
FILE* experiment_measurements_logfile;
char experiment_measurements_logfile_name[300];
FILE* experiment_position_logfile;
char experiment_position_logfile_name[300];

char scan_folder_name[256];
FILE* scan_measurements_logfile;
char scan_measurements_logfile_name[300];
FILE* scan_position_logfile;
char scan_position_logfile_name[300];

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

std::string environment_file("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/test_map.environment");
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

int sock = 0; 

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
	std::ifstream infile("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/obstacle_list.txt");
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

void log_trajectory(Position p){
	double time = get_exp_time();
	trajectory_logfile = fopen(trajectory_logfile_name, "a");
	fprintf(trajectory_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
	fclose(trajectory_logfile);
}


// ====================== ESTIMATIOM functions ========================= //

//ASSUMPTION : Using the history of measurements, but only one robot is operating
double likelihood(struct history history, double *parameters, PlumeModel *model, bool verbose){ 
	//float sigmaE = simulation_parameters.likelihood_error;
	float sigmaE = 775;
	//float sigmaD = 250; //125;
	double sum_tmp = 0;

	double true_source_location[2] = {11.0,1.0};

	// if(sampleBuffer.average_wind_intensity == 0) printf("NO WIND, BYE BYE!\n");
	
	// for each sample, getting the model concentration and calculate the likelihood
	// double model_concentration = 0;
	// for (int i = 0; i < history.size; i++){
	// 	model_concentration = model->concentration(history.position[i], sampleBuffer.average_wind_intensity, parameters);
	// 	sum_tmp += pow(history.measurement_avg[i] - model_concentration, 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
	// }

	// getting all model concentrations for all sample positions and then calculate the likelihood
	// printf("likelihood... history size %d\n", history.size);
	vector<float> model_c_avg(history.size); 
	//vector<vector<float>> true_model_c_avg(history.size, vector<float> (1)); 
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg);
	//model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, true_source_location, true_model_c_avg);
	for (int i = 0; i < history.size; i++){
		// float sigmaD = history.measurement_std[i];
		// if (sigmaD > 125){
		// 	sigmaD = 125;
		// }
		float sigmaD = 0;
		sum_tmp += pow(history.measurement_avg[i] - model_c_avg[i], 2) / (sigmaE*sigmaE + sigmaD*sigmaD);
		if (verbose){
			//printf("Parameters is : %f, %f\n", parameters[0], parameters[1]);
			//printf("For %d, estimation: %f, measurement: %f \n", i, true_model_c_avg[i][0], history.measurement_avg[i]);
		}
	}
	
	return (exp(-0.5 * sum_tmp));
}

double likelihood(struct history history, double *parameters, vector<float> model_c_avg){ 
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; //125;
	double sum_tmp = 0;
	// if(sampleBuffer.average_wind_intensity == 0) printf("NO WIND, BYE BYE!\n");
	
	for (int i = 0; i < history.size; i++){
		float sigmaE_std = sigmaE + history.measurement_std[i];
		sum_tmp += pow(history.measurement_avg[i] - model_c_avg[i], 2) / (sigmaE_std*sigmaE_std + sigmaD*sigmaD);
	}
	
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
		//printf("point %f %f is in obstacle\n", parameters[0], parameters[1]);
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
	if(p.x < XMIN || p.x > XMAX) outside = 1;
	if(p.y < YMIN || p.y > YMAX) outside = 1;
	//if(position[2] < ZMIN || position[2] > ZMAX) outside = 1;
	return outside;
}

void information_gain_navigation_approx(Position current_position){
	float p0 = 0.5; //50 cm
	double DKL=0, max_DKL=-100;
	//double position[1][3] = {{0}};
	Position tmp_position;
	struct history fake_history;
	fake_history.position = history.position;
	fake_history.measurement_avg = history.measurement_avg;
	fake_history.measurement_std = history.measurement_std;
	fake_history.size = history.size;

	// add fake prediction for friends goal position, so we don't choose the same ones
	for(int i = 0; i < simulation_parameters.number_of_robots; i++){
		if(friends[i].ID == 0 || friends[i].ID  == ID) continue;
		fake_history.position[fake_history.size].copy_position(friends[i].goal);
		fake_history.measurement_avg[fake_history.size] = plume_model.approx_concentration(friends[i].goal, sampleBuffer.average_wind_intensity);
		fake_history.measurement_std[fake_history.size] = 300; //FIXME: to fix later
		(fake_history.size)++;
	}

	for(int i =- 1; i <= 1; i++){
		for (int j = -1; j <= 1; j++){
			// for each possible action
			tmp_position.x = current_position.x + i*p0;
			tmp_position.y = current_position.y + j*p0;
			tmp_position.heading = 0;
			// printf("position : %f %f\n", tmp_position.x, tmp_position.y);
			//printf("position : %f %f %f\n", j*p0 * cos(l*theta0) * sin(k*phi0),j*p0 * sin(l*theta0) * sin(k*phi0),j*p0 * cos(k*phi0));
			if(position_outside_arena(tmp_position)) continue;
			//calculate the expected concentration on position
			fake_history.measurement_avg[fake_history.size] = plume_model.approx_concentration(tmp_position, sampleBuffer.average_wind_intensity);
			fake_history.measurement_std[fake_history.size] = 300;
			fake_history.position[fake_history.size].copy_position(tmp_position);
			(fake_history.size)++;
			// printf("fake_history.measurement[%d] = %f \n", fake_history.size-1, fake_history.measurement[fake_history.size-1]);
			//memcpy(future_posterior.pdf, zero.pdf, sizeof zero.pdf); //memcpy(future_posterior_3D, zero_posterior, sizeof zero_posterior);
			//future_posterior.sum = 0;
			DeepCFDPlumeModel2D plume_model_fake;
			Posterior future_posterior(&plume_model_fake, 0);
			//future_posterior.copy_posterior(zero);
			future_posterior.MCMC_evaluation(fake_history);
			//calculate the expected information gain
			DKL = posterior.Kullback_Leibler_divergence(future_posterior);
			// printf("DKL: %f _______________________\n", DKL);
			if(DKL > max_DKL ){
				max_DKL = DKL;
				goal_position.x = tmp_position.x; // + (randu()-0.5)/5.0
				goal_position.y = tmp_position.y;
				goal_position.heading = 0;
			}
		}
	}
}

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
	// printf("best point %f %f\n", best_point[0], best_point[1]);

	float ratio = 0.5;//gloabl_local_search_ratio;//simulation_parameters.gloabl_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
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
		// printf(">>> the goal is not in any obstacles\n");
		return;
	}

	//set concentration to 0 in the original goal position
	//FIXME: sometime the goal position is nan; which cost problem in the estimation
	// history.position[history.size].x = goal->x;
	// history.position[history.size].y = goal->y;
	// history.position[history.size].heading = 0;
	// history.measurement_avg[history.size] = 0;
	// history.size++;

	// otherwise, lets try 4 different candidates
	// for each candidate position
	if (idx != -1){
		printf(">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);
		float min_dist = 10000.0;
	    Position nearest_position(*goal);
	    Position candidates[4] = {	Position(obstacles[idx][0], goal->y, 0), 
								Position(obstacles[idx][2], goal->y, 0),
								Position(goal->x, obstacles[idx][1], 0),
								Position(goal->x, obstacles[idx][3], 0)};
		for(int i=0; i<4; i++){
			//check if the distance to candidate is smaller than others
			//printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
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
			//change the goal position using the pointer
			//printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
	    }
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

		for(int i=0; i<4; i++){
			//check if the distance to candidate is smaller than others
			//printf("XMIN :%f, XMAX:%f, YMIN:%f, YMAX:%f\n", XMIN, XMAX, YMIN, YMAX);
			printf("candidates x: %f, y:%f\n", candidates[i].x, candidates[i].y);
			printf("goal is: %f, %f\n", goal->x, goal->y);
			printf("distance is %f\n", goal->distance_from(candidates[i]));
			if(goal->distance_from(candidates[i]) < min_dist){
				printf(">>> best sor far\n");
				//if so, then check if that position is not in another obstacle and it's inside the arena
				printf(">>> inside another obstacle ? %d\n", candidates[i].is_in_obstacle_list(obstacles, nbr_obs));
				printf(">>> outside the arena ? %d \n", position_outside_arena(candidates[i]));
				if(candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i])){
					//if so, then this is the nearest position
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
					printf(">>> chose it as new position !!! \n");
				}
			}
			//change the goal position using the pointer
	    }
		printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
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
	//reinitialize
	posterior.pdf_reset();
	printf("going for MCMC\n");
	//estimation
	if(simulation_parameters.MCMC_or_exhaustive_eval)
		posterior.MCMC_evaluation(history);
	else
		posterior.exhaustive_evaluation(history);
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
	//if(FSM.n_cycles == MAX_SAMPLE_NUMBER || posterior.entropy < 0.1 || posterior.dirac_flag){
	if(FSM.n_cycles == MAX_SAMPLE_NUMBER || posterior.entropy < 0.1){
		//the end
		FSM.state = STATE_FINISHED;
	}else{
        // Mode: Experiment
		// update goal
		switch(WORKING_MODE){
			case EXPERIMENT:
				FSM.state = STATE_UPDATEGOAL;
		        FSM.hook = state_updategoal;
				break;
			case OFFLINE:
			    FSM.state = STATE_READ;
		        FSM.hook  = state_read;
				break;
		}
	}
}

void state_updategoal(){
	if(VERBOSE) printf("[%d] updating the goal\n", ID);

	// //find the best point to move to 
	// switch(simulation_parameters.scenario){
	// 	case NO_COM_NO_COOR :
	// 		information_gain_navigation_approx(current_position);
	// 		printf("goal position here 1 %f %f\n", goal_position.x, goal_position.y);
    // 		//information_gain_navigation_predictive();
    // 		//balance_exploration_exploitation(simulation_parameters.gloabl_local_search_ratio);
    // 		DKL_MAP_distance(0.75);
    // 		printf("goal position here 2 %f %f\n", goal_position.x, goal_position.y);
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
    // goal_outside_obstacles(&goal_position);
	 int d;
	 float f1, f2, f3;
     if (access(experiment_position_logfile_name, F_OK) != -1 && fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f1, &f2, &f3) == 4){
        if (d == 0){
			fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f1, &f2, &f3);
		}
		goal_position.x = f1;
		goal_position.y = f2;
	 



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
    // VisiLibity::Point start(current_position.x, current_position.y);
    // VisiLibity::Point finish(goal_position.x, goal_position.y);
    // VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);
    // // copy the path in another variable
    // for(unsigned k=0; k<Polyline_path.size(); k++){
    // 	Position p(Polyline_path[k].x(), Polyline_path[k].y(),0);
    //     path.push_back(p);
    //		log_trajectory(p);
    // }
    // i = path.begin();

    //send the goal to the supervisor
    Message msg(ID, 0, goal_position, -1, goal_position, 0);
    send_msg(msg);

    // send message to client
    double g_buffer[2] = {0};
	int timeout = 10;
	int success = -1;

	//send the message
	send_goal_to_robot(goal_position.x, goal_position.y);

	//move to listen
	FSM.state = STATE_LISTEN;
	FSM.hook = state_listen;
	 } else {
		 FSM.state = STATE_FINISHED;

	 }
}

void state_listen(){
	int success = -1;
    double buffer[5] = {0}; 

	// wait for gas measurements
    success = read(sock, buffer, sizeof(double)*5); 
    //printf("received Gas value %f %f %f %f\n", buffer[0], buffer[1], buffer[2], buffer[3]); 
    // printf("listening..\n");
     
	//TODO - extend current version to receive multiple measurements
    if(success > 0){
    	printf("received Gas value %f: (%f, %f) %f +- %f\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]); 

    	// save in history
		history.position.push_back({float(buffer[1]), float(buffer[2]), 0});
		history.measurement_avg.push_back(buffer[3]);
		history.measurement_std.push_back(buffer[4]);
		history.size++;

		//save current position
		current_position.x = buffer[1];
		current_position.y = buffer[2];
		current_position.heading = 0;
		 
		// check if all was received so far
		if(history.size != buffer[0])
			printf("Something wrong in history...\n");

		//move to estimating
		history_print(history);
		FSM.state = STATE_ESTIMATING;
		FSM.hook  = state_estimating;
	}
	else{
		printf("nothing yet .. \n");
	}
}

void state_read(){
    int d;
	float f1, f2, f3, f4; 
	unsigned long t;
	char line[256];

	// NOTE Read from an experiment
	if (history.size < 7){
		fscanf(experiment_measurements_logfile, "%d, %f %f", &d, &f1, &f2);
    history.size = d;
	history.measurement_avg[history.size] = f1;
	if (history.measurement_avg[history.size] < 0) history.measurement_avg[history.size]=0;
	history.measurement_std[history.size] = f2;
	//history.measurement_std[history.size] = 0;

	fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f1, &f2, &f3);
	history.position[history.size].x = f1;
	history.position[history.size].y = f2;

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

	}
	

}


// =========================== MAIN ============================== //


int main(int argc, char **argv){
	// python instance (should be called ONLY ONCE and only in main)
	//CPyInstance hInstance;

	init();
	if (WORKING_MODE == EXPERIMENT){
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

	// main loop
	while (FSM.state != STATE_FINISHED){
		//update_position();
		//if we are not stopped to sample, let's log the trajectory
		/*if(speed[0]!=0 && speed[1]!=0){
			log_trajectory();
		}*/
		//send info (current position and goal position) to friends for joint planning
		/*if(simulation_parameters.scenario == COM_COOR){
			// if a robot is already sampling (i.e. STATE_MEASURING), its goal should not be swapped with anyone. Therefore we send 0 as goal and current position
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
		}*/
	   	//change state
    	FSM.hook();
    	//next
    	next_iteration_time();
    	n_time_step++;
	}
	// final logs
	log_history();
	cleaning_up();
	//free the allocated memory to obstacle array
    for(int i = 0; i < nbr_obs; i++)
    	delete[] obstacles[i];
	delete[] obstacles;
	exit(0);
}

