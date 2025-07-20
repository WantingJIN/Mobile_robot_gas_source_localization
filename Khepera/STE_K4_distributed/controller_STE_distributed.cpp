/*
* File:          controller_STE_distributed.cpp
* Date:          May 2025
* Description:   The goal is to estimate the source terms including the position
* Author:        Wanting Jin
* Modifications: 26/01/2019
*/

//custom libraries
#include <fstream>
#include "controller_STE_distributed.h"
#include "Posterior.h"
#include "khepera_functions.h"
#include "constant_and_helper.h"
#include "SimulationParameters.h"
#include "path_planner.h"
#include "Position.h"

#define PORT 3011
#define SA struct sockaddr 

//using namespace std; //can't use it because of isnan !

//Global Variables
struct sOdometryTrack ot;

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
FILE* visited_list_logfile;
FILE* max_a_posteriori_logfile;
FILE* posterior_pdf_3D;
FILE* entropy_logfile;
FILE* tmp_file;
FILE* config_logfile;
FILE* trajectory_logfile;
FILE* baseline_measurements_logfile;

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
char baseline_measurements_logfile_name[256];
char visited_list_logfile_name[256];
char clustering_bounding_boxes_logfile_name[512];
char gas_information_map_logfile_name[512];
char scores_map_logfile_name[512];
char distance_map_logfile_name[512];
char acc_info_map_logfile_name[512];
// text file for storing some prints for debugging
char console_logfile_name[512];
char gas_concentration_map_logfile_name[512];
char experiment_time_logfile_name[512];


// finite state machine
struct FSM FSM;
// robot's measurements
SampleBuffer sampleBuffer(ODOR_FILTER_LENGTH);

Position goal_position;
Position goal_position_team[3];
Position current_position;
float speed[2] = {0};
// robot's position and measurement history
struct history history;
// robot's postion and meausrement during motion
struct history sample_in_motion;
//wind vector
sVector3d wind_vect;
// current posterior distribution 
// Posterior posterior;

// predicted posterior distribution
// Posterior future_posterior;

// posterior set to zero
// Posterior zero(0);

//robot's ID
int ID;
int self_index;

//message structure
struct message message;

SimulationParameters simulation_parameters;

//list of other robots
struct friends friends[MAX_NB_ROBOTS];
// pair of friends ID and socket
vector<pair<int, int>> sock_list;
int message_counter = 0;
int n_time_step = 0;

float previous_sensor_reading = 0.0;

int cell_visited_list[MAP_SIZE] = {0};

// plume model
DeepCFDPlumeModel2D plume_model;

// current posterior distribution
Posterior posterior(&plume_model);

// path planner
PathPlanner path_planner(&plume_model, &posterior, &history);

int sample_counter = 0;

// gas map initialization
// param: sigma, gamma, scaling_param, wind_speed, rco
GasMap2D gas_map(0.1, 0.1, 1, 0.75, 0.3);

std::vector<Message> ack_msg_vec;
std::vector<int> received_msgs;

// obstacles definitions
int nbr_obs = 4;
float margin = 0.2;
float ** obstacles = NULL;
// float obstacles[][4] = {{5-margin, static_cast<float>(5.5+margin), static_cast<float>(2.5-margin), 4+margin}, 
// 							{5-margin, 8+margin, static_cast<float>(2.5-margin), 3+margin}, 
// 							{9-margin, 12+margin, 1-margin, static_cast<float>(1.5+margin)}, 
// 							{static_cast<float>(11.5-margin), 12+margin, 0-margin, static_cast<float>(1.5+margin)}};
std::list<Position> path;
std::list<Position>::iterator i;

std::string environment_file("/home/root/wanting/test_map.environment");
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

int finished = 0;

bool set_odometry_pos_from_MCS = false;
bool MCS_data_available = true;

// store last position for minimum distance threshold in measurement update
Position last_measurement_pos;

//############################################### to fix
int nb_robots = 1;
int robots_ID_list[] = {101};

bool use_visibility_path = true;

// log time
static double start_timestamp;
static double end_timestamp;
double time_before_moving;
double time_after_moving;
double time_before_measuring;
double time_after_measuring;
double navigation_sampling_time_sum = 0;
long long estimating_time_sum = 0; // computation time of the estimation part
long long planning_time_sum = 0;   // computation time of the planning part

static float belief_maps[MAX_NB_ROBOTS][MAP_SIZE] = {0};
static int received_flags[MAX_NB_ROBOTS] = {0};
static float belief_map_merged[MAP_SIZE] = {0};

bool printed = false;
bool entered_state_receive_pdf = false;

// =========================== FUNCTIONS ================================ //
//random value sampled from a uniform distribution
float randu(){
	//random variable between 0 and 1
    return (float)rand() / (float)RAND_MAX ;
}

long long current_timestamp(){
	struct timeval te; 
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

void do_odometry(Position *position) {
    odometry_track_step(&ot);  
    position->x = ot.result.x;// + init_pos_x;
    position->y = -ot.result.y;// + init_pos_y;
    position->heading = -ot.result.theta;// + init_pos_theta;
}

void odometry_set_absolute_position_from_external(const float ex_x, const float ex_y, const float ex_heading){
	ot.result.x = ex_x;
	ot.result.y = -ex_y;
	ot.result.theta = -ex_heading;

}

float dist_to_goal(Position goal_position){
	//Euclidean distance between current position to the goal
	float dx, dy;
	dx = goal_position.x-current_position.x;
	dy = goal_position.y-current_position.y;
	return sqrt(dx*dx+dy*dy);
}

//show the measurement history on screen
void history_print(struct history history){
	printf("_____ Sample History _____ \n");
	for(int i=0; i<history.size; i++){
		printf("%d : %f (+/- %f) @ (%f, %f)\n", i, history.measurement[i], history.measurement_std[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list(){
	// open file
	std::ifstream infile("/home/root/wanting/obstacle_list.txt");
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

// add a message to ack required msgs or remove one from the list
// void ack_msg(Message msg){
//     //check if it's an ack or a new message
//     if(msg.ack_flag == 2){
//         //i'ts an ack => find the corresponding msg in the vector and remove it
//         //for(std::vector<Message>::iterator it = ack_msg_vec.begin(); it != ack_msg_vec.end(); it++){
//         //    if(it->msg_ID == msg.msg_ID){
//         //        //remove the acked message
//         //        printf("%d acked and removed\n", (int)(it->concentration));
//         //        ack_msg_vec.erase(it);
//         //   }
//         //} // --> makes seg fault because of erase
//         std::vector<Message>::iterator it = ack_msg_vec.begin();
//         while(it != ack_msg_vec.end()){
//     		if(it->msg_ID == msg.msg_ID && it->receiver_ID == msg.sender_ID){
//       			//remove the acked message
//                 //printf("[%lld] %d acked by %d and removed\n", current_timestamp(), (int)(it->msg_ID), msg.sender_ID);
//                 ack_msg_vec.erase(it);
//     		}else
//       			it++;
//   		}
//     }else{
//         //it has to be added to the table with all IP addresses
//         if(msg.receiver_ID == 1){
//             for(int i=0; i<nb_robots; i++){
//                 if(robots_ID_list[i] == ID) continue;
//                 Message tmp(msg.msg_ID, ID, robots_ID_list[i], msg.position, msg.concentration, msg.goal, msg.measurement0_goal1, msg.ack_flag);
//                 //msg.receiver_ID = robots_ID_list[i];
//                 //printf("[%lld] %d waiting for ack by %d\n", current_timestamp(),(int)(msg.msg_ID), tmp.receiver_ID);
//                 ack_msg_vec.push_back(tmp);
//             }
//         }else{
//         	ack_msg_vec.push_back(msg);
//         }
//     }
//     //send all the messages in the table
//     printf("----> waiting for ack: ");
//     for(std::vector<Message>::iterator it = ack_msg_vec.begin(); it != ack_msg_vec.end(); it++){
//         printf("%d(%d) ", (int)(it->msg_ID), (int)(it->receiver_ID));

//     }
//     printf("\n");
// }

int init_TCP_socket(){
	struct sockaddr_in address; 
	struct sockaddr_in serv_addr; 
      int sock = 0;
	for (size_t i = 0; i < simulation_parameters.ID_list.size(); i++) {
		// skip itself
		if (i == self_index) continue;
		// Creating socket file descriptor 
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) { 
			printf("\n Socket creation error \n"); 
			return -1; 
		} 

		memset(&serv_addr, '0', sizeof(serv_addr)); 

		serv_addr.sin_family = AF_INET; 
		serv_addr.sin_port = htons(PORT); 
		//serv_addr.sin_addr.s_addr = INADDR_ANY; 
		
		// Convert IPv4 and IPv6 addresses from text to binary form
		string IP_address = "192.168.1." + std::to_string(simulation_parameters.ID_list[i]); 
		if(inet_pton(AF_INET, IP_address.c_str(), &serv_addr.sin_addr)<=0) { //Server's IP
			printf("\nInvalid address/ Address not supported \n"); 
			return -1; 
		} 
		// connect
		if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) { 
			printf("\nConnection Failed \n"); 
			return -1; 
		}
      	sock_list.push_back(make_pair(simulation_parameters.ID_list[i], sock));
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

void log_expected_a_posteriori()
{
	expected_a_posteriori_logfile = fopen(expected_a_posteriori_logfile_name, "a");
	fprintf(expected_a_posteriori_logfile, "%d, %f, %f, %f, %f, %f\n", FSM.n_cycles,
		  plume_model.expected_value[0], plume_model.expected_value[1], plume_model.expected_value[2],
		  plume_model.expected_value[3], plume_model.expected_value[4]);
	fclose(expected_a_posteriori_logfile);
}

void log_max_a_posteriori()
{
	max_a_posteriori_logfile = fopen(max_a_posteriori_logfile_name, "a");
	fprintf(max_a_posteriori_logfile, "%d, %f, %f, %f, %f, %f\n", FSM.n_cycles,
		  plume_model.max_a_post_value[0], plume_model.max_a_post_value[1], plume_model.max_a_post_value[2],
		  plume_model.max_a_post_value[3], plume_model.max_a_post_value[4]);
	fclose(max_a_posteriori_logfile);
}

void log_entropy()
{
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
		fprintf(measurement_history_logfile,"%d, %f, %f\n", i, history.measurement[i], history.measurement_std[i]);
	}
    	fclose(measurement_history_logfile);
   
	//posterior 3D
	//posterior.log_pdf(posterior_pdf_3D_name, 'w');
}

void log_visited_list(){
	visited_list_logfile = fopen(visited_list_logfile_name, "a");

	for (int i = 0; i < MAP_SIZE; i++)
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
      printf("Robot ID [%d]: STATE_MEASURING \n", ID);
	std::vector<float> sample_from_map;
	int history_size_before_sampling = history.size;
      if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
	{
		// if the kernel mapping is enabled, then retrieve the measurement from the gas map
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			gas_map.retrieve_readings_from_gasmap(sample_in_motion.position, sample_from_map);
		}

		for (int i = 0; i < sample_in_motion.size; i++)
		{
			history.position.push_back(sample_in_motion.position[i]);
			// if there is some post-processing, then use the sample from the gas map
			// otherwise, use the raw sample from the robot
			if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
				history.measurement.push_back(sample_from_map[i]);
			else
				history.measurement.push_back(sample_in_motion.measurement[i]);
			history.measurement_std.push_back(0.0);
			history.FSM_iter.push_back(FSM.n_cycles);
			history.size++;
			// mark the cell as visited
			if (simulation_parameters.visitedList == visitedList_ON)
			{
				int pos_coord[2];
				position_to_indices_2D(sample_in_motion.position[i].x, sample_in_motion.position[i].y, &pos_coord[0], &pos_coord[1]);
				int idx = posterior.indices_to_index(pos_coord);
				cell_visited_list[idx] = 1;
			}
		}
		sample_in_motion.position.clear();
		sample_in_motion.measurement.clear();
		sample_in_motion.size = 0;
		sample_from_map.clear();
	}
	if (simulation_parameters.samplingMode == STOP_AND_GO_SAMPLING){
		while(sampleBuffer.n_samples < ODOR_FILTER_LENGTH){
			//continue measuring odor concentration
			sampleBuffer.instant_concentration = odor_read();
			//measuring wind
			//wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
			//add samples to buffers
			sampleBuffer.add_all_to_buffer();
			next_iteration_time();
			//printf("n %d wind intensity: %f, angle %f \n", sampleBuffer.n_samples, sampleBuffer.instant_wind_intensity, sampleBuffer.instant_wind_angle);
		}
		//average all samples
		sampleBuffer.average_all();
	} else if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING || simulation_parameters.samplingMode == SENSE_WITHOUT_STOP){
		sampleBuffer.instant_concentration = odor_read();
		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
	}
	
	//log and save the measurements at the goal position
	if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
	log_position();
	log_measurements();
	history.position.push_back({current_position.x, current_position.y, 0.0});
	float calibrated_sensor_reading = (sampleBuffer.average_concentration - odor_baseline_value < 0) ? 0 : calibrate_sensor_reading(sampleBuffer.average_concentration - odor_baseline_value);
	if (calibrated_sensor_reading < 0) calibrated_sensor_reading = 0.0;
	history.measurement.push_back(calibrated_sensor_reading);
	// if(history.measurement[history.size] < 0) history.measurement[history.size] = 0;
	history.measurement_std.push_back(sampleBuffer.stdDev_concentration);
	history.size++;
	history_print(history);

	if (simulation_parameters.visitedList == visitedList_ON)
	{
		int pos_coord[2];
		position_to_indices_2D(current_position.x, current_position.y, &pos_coord[0], &pos_coord[1]);
		int idx = posterior.indices_to_index(pos_coord);
		cell_visited_list[idx] = 1;
	}

	// send the measurement to the server
	// float m_buffer[5] = {0};
	// int timeout = 10;
	// int success = -1;

	// //send the message
	// int history_size_after_sampling = history.size;

	// printf("history size before : %d, after ; %d \n", history_size_before_sampling, history_size_after_sampling);
	// // first send the size of the message
	// int measurement_size = history_size_after_sampling -  history_size_before_sampling;
	// do{
	// 	success = send(sock, &measurement_size, sizeof(measurement_size), 0);
	// 	--timeout;
	// } while(success == -1 && timeout > 0);
	// if(success > 0){
	// 	printf("successfully sent measurement_size %d \n", measurement_size);
	// 	timeout = 10;
	// }
	// else
	// 	printf("failed to send measurement_size %d \n", measurement_size);


	// for (int i = history_size_before_sampling; i < history_size_after_sampling; i++) {
	// 	m_buffer[0] = i;
	// 	m_buffer[1] = history.position[i].x;
	// 	m_buffer[2] = history.position[i].y;
	// 	m_buffer[3] = history.measurement[i];
	// 	m_buffer[4] = history.measurement_std[i];
	// 	//variable = (condition) ? expressionTrue : expressionFalse;
	// 	do{
	// 		success = send(sock, m_buffer, sizeof(m_buffer), 0);
	// 		--timeout;
	// 	} while(success == -1 && timeout > 0);
	// 	if(success > 0){
	// 		printf("successfully sent measurement %f: (%f, %f) %f +- %f\n", m_buffer[0], m_buffer[1], m_buffer[2], m_buffer[3], m_buffer[4]);
	// 		timeout = 10;

	// 	}
	// 	else
	// 		printf("failed to send measurement %f: (%f, %f) %f +- %f\n", m_buffer[0], m_buffer[1], m_buffer[2], m_buffer[3], m_buffer[4]);
	

	// // send the measurement to the server and wait for new orders
	// FSM.state = STATE_LISTEN;
	// FSM.hook  = state_listen;
	// }

	FSM.state = STATE_ESTIMATING;
	FSM.hook  = state_estimating;
}
void state_estimating(){
	printf("Robot ID [%d]: STATE_ESTIMATING!!! \n", ID);
	// if (VERBOSE)
	// 	printf("[%d] estimating...\n", ID);

	long long time_before_estimating = current_timestamp();

	// reinitialize
	posterior.pdf_reset();
	// estimation
	if (simulation_parameters.BayesianModelingMethod == MCMC)
	{
		printf("going for MCMC\n");
		posterior.MCMC_evaluation(history);
	}
	if (simulation_parameters.BayesianModelingMethod == EXHAUSTIVE)
	{
		printf("going for exhaustive\n");
		posterior.exhaustive_evaluation(history);
	}
	// only when using MCMC
	if (posterior.dirac_flag)
	{
		if (VERBOSE)
			printf("[%d] exhaustive search!\n", ID);
		posterior.exhaustive_evaluation(history);
	}

	// printf("sum posterior : %f \n", posterior.sum);
	posterior.normalize();
	posterior.log_pdf(posterior_pdf_3D_name, 'a');

	FSM.n_cycles++;
	if (VERBOSE)
		printf("[%d] cycle number %d\n", ID, FSM.n_cycles);

	long long current_estimation_time = current_timestamp() - time_before_estimating;

	estimating_time_sum += current_estimation_time;

	// check if stopping condition is reached
	if ((simulation_parameters.number_of_robots == 1)||(simulation_parameters.teamStrategy == INDIVIDUAL))
	{
		// calculate entropy and spread
		posterior.process();
      	posterior.calculate_entropy_and_spread();
		log_expected_a_posteriori();
		log_max_a_posteriori();
		log_entropy();
		
		bool is_stop_condition_reached = check_stop_condition();
		if (is_stop_condition_reached)
		{
			FSM.state = STATE_FINISHED;
			return;
		} else {
			FSM.state = STATE_UPDATEGOAL;
			FSM.hook = state_updategoal;
		}
	}
	else if (simulation_parameters.teamStrategy == DISTRIBUTED)
	{
		FSM.state = STATE_SHARE_PDF;
		FSM.hook = state_share_pdf;
	}
}

void state_updategoal()
{
	// computation time of path planning
	long long time_before_path_planning = current_timestamp();
	printf("Robot ID [%d]: STATE_UPDATEGOAL \n", ID);
	//fprintf(console_logfile, "FSM iter: %d \n", FSM.n_cycles);
	if (VERBOSE)
		printf("[%d] updating the goal\n", ID);
	switch (simulation_parameters.planningStrategy)
	{
	case GREEDY:
		path_planner.maximal_belief_planning();
		break;
	case BALANCE_MAXIMAL_EXPECTED:
		path_planner.balance_expected_maximal_pdf();
		break;
	case RANDOM:
		path_planner.random_planning();
		break;
	case LAWNMOWER:
		path_planner.lawnmower_planning();
		break;
	case BELIEF_CLUSTERING:
		path_planner.belief_clustering_planning();
		break;
	case WEIGHTED_CLUSTERING:
		path_planner.weighted_belief_clustering();
		break;
	case INFO_CLUSTERING:
		path_planner.informative_clustering_planning();
		break;
	case INFORMATIVE_KLD:
		path_planner.KLD_informative_goal_selection();
		path_planner.KLD_distance_balance();
		break;
	case SUM_DIJKSTRA_FIX:
		path_planner.accumulative_Dijkstra();
		break;
	case SUM_DIJKSTRA_ADP:
		path_planner.accumulative_Dijkstra();
		break;
	case MULTI_DIJKSTRA:
		path = path_planner.multi_objective_Dijkstra();
		use_visibility_path = false;
		break;
	default:
		break;
	}
	if (FSM.state != STATE_FINISHED)
	{
		// The goal position is very close to current postion, swith to random planning
		// if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<float>(simulation_parameters.sampling_dist_thresh)))
		// {
		// 	printf("The goal position is very close to current postion, switch to random planning \n");
		// 	//fprintf(console_logfile, "The goal position is very close to current postion, switch to random planning \n");
		// 	random_planning();
		// }
		if (use_visibility_path)
		{

			printf("goal position : %f %f \n", goal_position.x, goal_position.y);
			path_planner.goal_outside_obstacles(&goal_position);
			printf("from %f %f to goal %f %f \n", current_position.x, current_position.y, goal_position.x, goal_position.y);

			// find the path using visibility graph
			VisiLibity::Point start(current_position.x, current_position.y);
			VisiLibity::Point finish(goal_position.x, goal_position.y);
			VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);
			// copy the path in another variable
			for (size_t k = 0; k < Polyline_path.size(); k++)
			{
				Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
				path.push_back(p);
				log_trajectory(p);
			}
		}
		i = path.begin();
		// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		// update goal
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		time_before_moving = current_timestamp();
		long long current_path_planning_time = current_timestamp() - time_before_path_planning;
		planning_time_sum += current_path_planning_time;
		
	}
}

void state_listen(){
	// int result;
	// float g_buffer[2] = {0}; 

	// //wait for new orders
	// result = read(sock, g_buffer, sizeof(g_buffer));
	// if(result > 0){
	// 	printf("received a goal position : (%f, %f)\n", g_buffer[0], g_buffer[1]);
	// 	// check if it was a termination message
	// 	if(g_buffer[0] == -1 && g_buffer[1] == -1){
	// 		goal_position.x = 3;
    	// 	goal_position.y = float(ID-100)/2.0;
    	// 	finished = 1;
	// 	}else{
	// 		//set the goal position
	// 		goal_position.x = g_buffer[0];
	// 		goal_position.y = g_buffer[1];
	// 	}
	// 	//find the path using visibility graph
	// 	VisiLibity::Point start(current_position.x, current_position.y);
	// 	VisiLibity::Point finish(goal_position.x, goal_position.y);
	// 	VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);
	// 	// copy the path in another variable
	// 	for(size_t k=0; k<Polyline_path.size(); k++){
	// 		Position p(Polyline_path[k].x(), Polyline_path[k].y(),0);
	// 		path.push_back(p);
	// 		log_trajectory(p);
	// 	}
	// 	i = path.begin();

	// 	//move to the goal position
	// 	FSM.state = STATE_MOVING;
	// 	FSM.hook = state_moving;
	// }
}

bool entered_state_moving = false;

void state_moving(){

	if (!entered_state_moving)
	{
		printf("Robot ID [%d]: STATE_MOVING \n", ID);
		entered_state_moving = true;
	}

	if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
	{
		sample_counter++;
		//sample and run Kernel
		float sensor_reading = odor_read();
		float instant_concentration = (sensor_reading - odor_baseline_value < 0) ? 0 : calibrate_sensor_reading(sensor_reading - odor_baseline_value);
		if (instant_concentration < 0) instant_concentration = 0;
		if (simulation_parameters.processingMode == DECONVOLUTION || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			reverse_sensor_model(instant_concentration, previous_sensor_reading);
		}
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			gas_map.runKernelAlgorithm(current_position, instant_concentration);
		}
		if (sample_counter > simulation_parameters.sampling_period){
			// Sampling distance threshold
			// float dist_to_last_measurement = sqrt(pow(current_position.x - last_measurement_pos.x, 2) + pow(current_position.y - last_measurement_pos.y, 2));
			// if ((sample_in_motion.size == 0) || dist_to_last_measurement > simulation_parameters.sampling_dist_thresh)
			// {
				// only record the measurement if the localization result is from MCS
				// sometime the odometry result can be very wrong when the robot gets stuck with the obstacle (wheel slip)
				if (MCS_data_available){
					sample_in_motion.position.push_back({current_position.x, current_position.y, 0});
				      sample_in_motion.measurement.push_back(instant_concentration);
				      sample_in_motion.size++;

				      last_measurement_pos.x = current_position.x;
				      last_measurement_pos.y = current_position.y;
				}
				
			//}
			sample_counter = 0;
		}
	}
	if(dist_to_goal(goal_position) > 0.05){
		//move
		if(dist_to_goal(*i) <= 0.05 && i != path.end()){
			i++;
			printf("current goal is: %f, %f\n", i->x, i->y);
		}
		go_to_goal(*i);
		//obstacle avoidance if necessary
		if(obstacle_or_not() == 1){
			FSM.state = STATE_OBSTACLE_AVOIDANCE;
			FSM.hook  = state_obstacle_avoidance;
			return;
		}
		//wb_differential_wheels_set_speed(speed[0],speed[1]);
		set_wheels_speed(speed[0],speed[1]);
		int update_goal_iterations = static_cast<int>(round(simulation_parameters.update_goal_time * 1000.0 / CONTROLLER_TIMESTAMP));

		if (simulation_parameters.updateGoalPolicy == FIXED_FREQ){
			if (n_time_step % update_goal_iterations == 0 && n_time_step != 0)
				{
				// set_wheels_speed(0, 0);
				sampleBuffer.reset_all();
				path.clear();
				// printf("Robot ID [%d]. Number of iteration is %d\n", ID, n_time_step);
				FSM.state = STATE_MEASURING;
				FSM.hook = state_measuring;
				time_after_moving = current_timestamp();
				entered_state_moving = false;
			}
		}
	}else{
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		sampleBuffer.reset_all();
		path.clear();
		if(finished){
			FSM.state = STATE_FINISHED;
		}else{
			//back to measuring
			FSM.state = STATE_MEASURING;
			FSM.hook  = state_measuring;
			time_after_moving = current_timestamp();
			navigation_sampling_time_sum += time_after_moving - time_before_moving;
			time_before_measuring = current_timestamp();
			printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
			entered_state_moving = false;
		}
	}
}

void state_obstacle_avoidance(){
	if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	
	//if no obstacle,
	if(obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05){
		//printf("no obstacle anymore..XXXXXXX\n");
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}
	braitenberg_run(&braitenberg);
}

void state_send_goal()
{
	if(VERBOSE) printf("Robot ID [%d]: STATE_SEND_GOAL \n", ID);

	if (FSM.state != STATE_FINISHED)
	{
		// The goal position is very close to current postion, swith to random planning
		// if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<float>(simulation_parameters.sampling_dist_thresh)))
		// {
		// 	printf("The goal position is very close to current postion, switch to random planning \n");
		// 	//fprintf(console_logfile, "The goal position is very close to current postion, switch to random planning \n");
		// 	random_planning();
		// }

		// goal position for Robot [101]
		goal_position = goal_position_team[0];
		printf("Robot [%d]: Goal position : %f %f \n", ID, goal_position.x, goal_position.y);
		path_planner.goal_outside_obstacles(&goal_position);
		printf("from %f %f to goal %f %f \n", current_position.x, current_position.y, goal_position.x, goal_position.y);

		// find the path using visibility graph
		VisiLibity::Point start(current_position.x, current_position.y);
		VisiLibity::Point finish(goal_position.x, goal_position.y);
		VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);

		// copy the path in another variable
		for (size_t k = 0; k < Polyline_path.size(); k++)
		{
			Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
			path.push_back(p);
			log_trajectory(p);
		}
		i = path.begin();

		for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
			int target_ID = 101 + i;
			if (target_ID == ID) continue;  // Skip self
	    
			// Message msg(ID, target_ID, goal_position_team[i], -1, goal_position_team[i], 0); // -1 for STATE_FINISHED flag
			// send_udp_msg(msg);
			// TODO: Implement the logic to send goal position to other robots
		}

		// move to the goal position
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		time_before_moving = current_timestamp();
	}
}

bool entered_state_receive_goal = false;

void state_receive_goal()
{

	if (!entered_state_receive_goal)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_GOAL \n", ID);
		entered_state_receive_goal = true;
	}

	bool goal_received = false;
      // TODO: Implement the logic to receive goal position from other robots

	// in case no goals received yet, remain in this state
	if (!goal_received)
	{
		// printf("Robot ID [%d]: No goal received, staying in STATE_RECEIVE_GOAL.\n", ID);
		return;
	}

	// print goal position for some of the robots
	// printf("Robot ID [%d]: Final goal position: %f %f\n", ID, goal_position.x, goal_position.y);
	path_planner.goal_outside_obstacles(&goal_position);
	printf("Robot ID [%d]: from %f %f to goal %f %f\n", ID, current_position.x, current_position.y, goal_position.x, goal_position.y);

	VisiLibity::Point start(current_position.x, current_position.y);
	VisiLibity::Point finish(goal_position.x, goal_position.y);
	VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);

	for (size_t k = 0; k < Polyline_path.size(); k++)
	{
		Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
		path.push_back(p);
		log_trajectory(p);
	}

	i = path.begin();

	FSM.state = STATE_MOVING;
	FSM.hook = state_moving;
	entered_state_receive_goal = true;
	time_before_moving = current_timestamp();
}

void state_receive_pdf()
{
	if (!entered_state_receive_pdf)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_PDF \n", ID);
		entered_state_receive_pdf = true;
	}

	if (simulation_parameters.teamStrategy == DISTRIBUTED)
	{
		if (!printed && posterior.pdf != nullptr)
		{
			memcpy(belief_maps[self_index], posterior.pdf, MAP_SIZE * sizeof(float));
			printed = true;
		}

		Message msg;
		for (size_t i = 0; i < sock_list.size(); i++)
		{
			receive_tcp_msg(sock_list[i].second, &msg);		
		}
		if (msg.finish_flag == -1) {
			// finish the task
			FSM.state = STATE_FINISHED;
			printf("Robot ID [%d]: STATE_FINISHED\n", ID);
			return;
		} else {
			int sender_index = find_index_in_ID_list(msg.sender_ID, simulation_parameters.ID_list);
			memcpy(belief_maps[sender_index], msg.belief_map, sizeof(float)*MAP_SIZE);
			received_flags[sender_index] = 1;
		}
		
		// Check if all others have sent their belief maps
		int all_received = 1;
		for (int i = 0; i < simulation_parameters.number_of_robots; i++)
		{
			if (i == self_index)
				continue;
			if (!received_flags[i])
			{
				all_received = 0;
				printf("Robot ID [%d]: Waiting for belief map from Robot ID [%d]\n", ID, 101 + i);
				break;
			}
		}
	
		if (all_received)
		{

			// Reset state for next round
			for (int i = 0; i < simulation_parameters.number_of_robots; i++)
			{
				received_flags[i] = 0;
			}
			printed = false;
			FSM.state = STATE_MERGE_PDF;
			FSM.hook = state_merge_pdf;
			entered_state_receive_pdf = false;
		}
	}
}

void state_merge_pdf()
{
	printf("Robot ID [%d]: STATE_MERGE_PDF \n", ID);
	// Initialize to 1.0 for multiplicative identity
	for (int i = 0; i < MAP_SIZE; i++)
	{
		belief_map_merged[i] = 1.0;
	}

	// Multiply belief maps from other robots (excluding self)
	for (int r = 0; r < simulation_parameters.number_of_robots; r++)
	{
		if (r == self_index)
			continue; // Skip self

		for (int i = 0; i < MAP_SIZE; i++)
		{
			belief_map_merged[i] *= belief_maps[r][i];
		}
	}

	// normalize belief map
	float sum = 0.0;
	for (int i = 0; i < MAP_SIZE; i++)
	{
		sum += belief_map_merged[i];
	}

	if (sum > 0)
	{
		for (int i = 0; i < MAP_SIZE; i++)
		{
			belief_map_merged[i] /= sum;
		}
	}

	// printf("Robot ID [%d]: Belief map merged successfully.\n", ID);
	printf("Robot ID [%d]: Merged belief map is: \n", ID);
	for (int i = 0; i < MAP_SIZE; i++)
	{
		printf("%f ", belief_map_merged[i]);
	}
	printf("\n");
	memcpy(posterior.pdf, belief_map_merged, MAP_SIZE * sizeof(float));
      // Check the stopping condition after merging
	posterior.process();
      posterior.calculate_entropy_and_spread();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	bool is_stop_condition_reached = check_stop_condition();
	if (is_stop_condition_reached)
	{
		FSM.state = STATE_FINISHED;
		printf("Robot ID [%d]: STATE_FINISHED\n", ID);
		//fprintf(console_logfile, "Robot ID [%d]: STATE_FINISHED\n", ID);
		broadcast_state_finished();
		return;
	} else {
		if (simulation_parameters.teamStrategy == CENTRALIZED)
		{
				FSM.state = STATE_UPDATEGOAL_TEAM;
				FSM.hook = state_updategoal_team;
			
		}
		else if (simulation_parameters.teamStrategy == DISTRIBUTED)
		{
			FSM.state = STATE_REGION_ALLOCATION;
			FSM.hook = state_region_allocation;
		}
	}
	
}

void state_updategoal_team()
{
	printf("Robot ID [%d]: STATE_UPDATEGOAL_TEAM \n", ID);

	path_planner.maximal_merged_belief_planning();

	printf("Assigned goal positions to Robot [101], [102] and [103].\n");

	for (int i = 0; i < 3; i++)
	{
		printf("Robot [%d]: Goal position -> x: %f, y: %f\n", 101 + i, goal_position_team[i].x, goal_position_team[i].y);
	}

	FSM.state = STATE_SEND_GOAL;
	FSM.hook = state_send_goal;
}

void state_share_pdf()
{
    printf("Robot ID [%d]: STATE_SHARE_PDF\n", ID);

    if (posterior.pdf == NULL) {
        printf("Robot ID [%d]: No posterior PDF to share.\n", ID);
    } else {
        for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
		if (i == self_index) continue; // Skip self
            int target_ID = simulation_parameters.ID_list[i];
		int message_ID = ID + message_counter++;
		int finish_flag = 0; // -1 indicates that this is not a finish message
            Message msg(message_ID, ID, target_ID, current_position, finish_flag, posterior.pdf);
		int target_sock = -1;
		for(size_t i=0; i<sock_list.size(); i++){
			if(sock_list[i].first == target_ID){
				target_sock = sock_list[i].second;
				break;
			}
		}
            if (target_sock != -1) send_tcp_msg(target_sock, msg);
            printf("Robot ID [%d]: Sent belief map to Robot ID [%d]\n", ID, target_ID);
        }
    }

    FSM.state = STATE_RECEIVE_PDF;
    FSM.hook = state_receive_pdf;
}

void state_region_allocation()
{
	printf("Robot ID [%d]: STATE_REGION_ALLOCATION \n", ID);
	if (simulation_parameters.teamStrategy == DISTRIBUTED)
	{
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;
	}
}

// Broadcast STATE_FINISHED to all teammates
void broadcast_state_finished()
{
    printf("Robot ID [%d]: Broadcasting STATE_FINISHED\n", ID);

    for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
        int target_ID = 101 + i;
        if (target_ID == ID) continue;  // Skip self
        int message_ID = ID + message_counter++;
	  //send STATE_FINISHED message to other robots
        Message msg(message_ID, ID, target_ID, current_position, -1, nullptr); // -1 for STATE_FINISHED flag
	  int target_sock = -1;
	  for(size_t j=0; j<sock_list.size(); j++){
		if(sock_list[j].first == target_ID){
		    target_sock = sock_list[j].second;
		    break;
		}
	  }
        if (target_sock != -1) send_tcp_msg(target_sock, msg);
        printf("Robot ID [%d]: Sent STATE_FINISHED to Robot ID [%d]\n", ID, target_ID);
    }
}

bool check_stop_condition()
{
	bool is_stop_condition_reached = false;
	printf("current_entropy is %f\n", posterior.entropy);
	if (simulation_parameters.stopCondition == ENTROPY_ONLY){
		if (FSM.n_cycles == MAX_ITER || posterior.entropy < 0.1 ){
			is_stop_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == SPREAD_ONLY){
		if (FSM.n_cycles == MAX_ITER || (posterior.x_spatial_spread < 2 && posterior.y_spatial_spread < 2)){
			is_stop_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == ENTROPY_SPREAD){
		if (FSM.n_cycles == MAX_ITER || (posterior.entropy < 0.3 && posterior.x_spatial_spread < 2 && posterior.y_spatial_spread < 2)){
			is_stop_condition_reached = true;
		}
	}
	printf("entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior.entropy, posterior.x_spatial_spread, posterior.y_spatial_spread);
	//fprintf(console_logfile, "entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior.entropy, posterior.x_spatial_spread, posterior.y_spatial_spread);
	if (is_stop_condition_reached)
	{
		// log the finish time stamp
		end_timestamp = current_timestamp();
		printf("Robot ID [%d]: Stop condition reached!\n", ID);
		//fprintf(console_logfile, "Robot ID [%d]: Stop condition reached!\n", ID);
	}
	return is_stop_condition_reached;
}

// =========================== MAIN ============================== //


int main(int argc, char **argv){
	
	init();
	init_TCP_socket();
	establish_baseline();
	read_obstacle_list();
	//odor_baseline_value = 30000
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
		update_position();
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

