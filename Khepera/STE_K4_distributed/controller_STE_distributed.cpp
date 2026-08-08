/*
* File:          controller_STE_distributed.cpp
* Date:          May 2025
* Description:   The goal is to estimate the source terms including the position
* Author:        Wanting Jin
* Modifications: 26/01/2019
*/

//custom libraries
#include <fstream>
#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include "controller_STE_distributed.h"
#include "Posterior.h"
#include "khepera_functions.h"
#include "constant_and_helper.h"
#include "SimulationParameters.h"
#include "path_planner.h"
#include "Position.h"
#include "BeliefCommManager.h"
#include "region_allocator.h"
#include "BeliefMapFusion.h"

//#define PORT 3011
#define SA struct sockaddr 
#define MSG_STATE_BELIEF_MAP 1
#define MSG_STATE_SHARE_PDF -99
#define MSG_STATE_WIND_SAMPLE -97
#define MSG_STATE_FINISHED -100

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
FILE* posterior_pdf;
FILE* merged_pdf;
FILE* entropy_logfile;
FILE* tmp_file;
FILE* config_logfile;
FILE* trajectory_logfile;
FILE* baseline_measurements_logfile;
FILE* region_allocation_logfile;

// text file for storing some prints for debugging
FILE *console_logfile;
FILE *clustering_debugging_logfile;

char goalposition_logfile_name[256];
char measurements_logfile_name[256];
char posterior_logfile_name[256];
char posteriorpdf_logfile_name[256];
char expected_a_posteriori_logfile_name[256];
char max_a_posteriori_logfile_name[256];
char likelihood_logfile_name[256];
char measurement_history_logfile_name[256];
char position_history_logfile_name[256];
char posterior_pdf_logfile_name[256];
char merged_pdf_logfile_name[256];
char entropy_logfile_name[256];
char config_logfile_name[256];
char trajectory_logfile_name[256];
char baseline_measurements_logfile_name[256];
char visited_list_logfile_name[256];
char region_allocation_logfile_name[512];
char clustering_bounding_boxes_logfile_name[512];
char gas_information_map_logfile_name[512];
char scores_map_logfile_name[512];
char distance_map_logfile_name[512];
char acc_info_map_logfile_name[512];
// text file for storing some prints for debugging
char console_logfile_name[512];
char clustering_debugging_name[512];
char gas_concentration_map_logfile_name[512];
char gas_concentration_maps_log_root[512];
char wind_field_u_logfile_name[512];
char wind_field_v_logfile_name[512];
char wind_field_w_logfile_name[512];
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

//communication structure
BeliefCommManager inter_robot_comm;

SimulationParameters simulation_parameters;

//list of other robots
struct friends friends[MAX_NB_ROBOTS];
// pair of friends ID and socket
std::vector<int> server_sockets;
std::vector<std::pair<int, int>> client_sockets;  // {peer_ID, sock}
vector<pair<int, int>> sock_list;
int message_counter = 0;
int n_time_step = 0;

float previous_sensor_reading = 0.0;

int cell_visited_list[MAP_SIZE] = {0};

PlumeModel *plume_model = NULL;
Posterior *posterior = NULL;
PathPlanner *path_planner = NULL;

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

static VisiLibity::Environment load_environment_or_exit(const std::string &filename)
{
	if (access(filename.c_str(), F_OK) == -1)
	{
		fprintf(stderr, "Error opening environment file: %s\n", filename.c_str());
		exit(1);
	}
	return VisiLibity::Environment(filename);
}

std::string environment_file("/home/root/wanting/test_map.environment");
VisiLibity::Environment my_environment(load_environment_or_exit(environment_file));
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

int finished = 0;

bool set_odometry_pos_from_MCS = false;
bool MCS_data_available = true;

// store last position for minimum distance threshold in measurement update
Position last_measurement_pos;

//############################################### to fix
int nb_robots = 1;

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
long long wind_gas_prediction_time_sum = 0;
long long belief_update_time_sum = 0;
long long belief_fusion_time_sum = 0;
long long region_allocation_time_sum = 0;
long long path_planning_time_sum = 0;
long long robot_moving_time_sum = 0;

static float belief_maps[MAX_NB_ROBOTS][MAP_SIZE] = {0};
static int received_flags[MAX_NB_ROBOTS] = {0};
static float belief_map_merged[MAP_SIZE] = {0};
static float received_current_pos[MAX_NB_ROBOTS][2] = {0};
static int ready_from[MAX_NB_ROBOTS] = {0};
static int i_am_ready = 0;
static std::array<int, MAX_NB_ROBOTS> latest_team_wind_sample_id = [] {
	std::array<int, MAX_NB_ROBOTS> ids;
	ids.fill(-1);
	return ids;
}();
static int local_wind_sample_id = 0;

std::vector<int> obstacle_map(MAP_SIZE, 0);
RegionAllocator *region_allocator = NULL;

bool printed = false;
bool entered_state_receive_pdf = false;
bool initial_point_calculated = false;

// =========================== FUNCTIONS ================================ //
static FILE *open_log_file(const char *filename, const char *mode)
{
	FILE *file = fopen(filename, mode);
	if (file == NULL)
		fprintf(stderr, "Error opening log file: %s\n", filename);
	return file;
}

static bool ensure_current_path_target(const char *state_name)
{
	if (path.empty())
	{
		printf("Robot ID [%d]: %s has empty path; falling back to direct goal.\n", ID, state_name);
		path.push_back(goal_position);
		i = path.begin();
	}

	while (i != path.end() && dist_to_goal(*i) <= 0.05f)
		i++;

	if (i == path.end())
	{
		if (dist_to_goal(goal_position) <= 0.05f)
			return false;

		printf("Robot ID [%d]: %s path exhausted before goal; falling back to direct goal.\n", ID, state_name);
		path.push_back(goal_position);
		i = path.end();
		i--;
	}

	return true;
}

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

void add_wind_gas_prediction_time_ms(long long elapsed_ms)
{
	wind_gas_prediction_time_sum += elapsed_ms;
}

static void log_experiment_time()
{
	end_timestamp = current_timestamp();
	if (start_timestamp <= 0)
		start_timestamp = end_timestamp;

	long long total_experiment_time = static_cast<long long>(end_timestamp - start_timestamp);
	long long region_and_path_time = region_allocation_time_sum + path_planning_time_sum;
	long long measured_total_time =
		wind_gas_prediction_time_sum +
		belief_update_time_sum +
		belief_fusion_time_sum +
		region_and_path_time +
		robot_moving_time_sum;

	FILE *file = open_log_file(experiment_time_logfile_name, "w");
	if (file == NULL)
		return;

	fprintf(file,
	        "robot_id,total_experiment_ms,wind_estimation_and_gas_dispersion_prediction_ms,"
	        "belief_update_ms,belief_fusion_ms,region_allocation_and_path_planning_ms,"
	        "region_allocation_ms,path_planning_ms,robot_moving_ms,measured_total_ms,"
	        "unaccounted_ms,n_cycles,n_time_step\n");
	fprintf(file,
	        "%d,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%d,%d\n",
	        ID,
	        total_experiment_time,
	        wind_gas_prediction_time_sum,
	        belief_update_time_sum,
	        belief_fusion_time_sum,
	        region_and_path_time,
	        region_allocation_time_sum,
	        path_planning_time_sum,
	        robot_moving_time_sum,
	        measured_total_time,
	        total_experiment_time - measured_total_time,
	        FSM.n_cycles,
	        n_time_step);
	fclose(file);
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
		printf("%d : gas=%f (+/- %f), wind=(%f, %f), uv=(%f, %f) @ (%f, %f)\n",
		       i, history.gas_meas_avg[i], history.gas_meas_std[i],
		       history.wind_intensity[i], history.wind_angle[i],
		       history.wind_meas_u[i], history.wind_meas_v[i],
		       history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list(){
	// open file
	std::ifstream infile("/home/root/wanting/obstacle_list.txt");
	if (!infile.is_open())
	{
		fprintf(stderr, "Error opening obstacle list file: /home/root/wanting/obstacle_list.txt\n");
		nbr_obs = 0;
		obstacles = NULL;
		std::fill(obstacle_map.begin(), obstacle_map.end(), 0);
		return;
	}
	// first line is the number of obstacles
	if (!(infile >> nbr_obs) || nbr_obs < 0)
	{
		fprintf(stderr, "Error reading obstacle count from /home/root/wanting/obstacle_list.txt\n");
		nbr_obs = 0;
		obstacles = NULL;
		std::fill(obstacle_map.begin(), obstacle_map.end(), 0);
		return;
	}
	// allocate memory for obstacle list array
	obstacles = new float*[nbr_obs];
	for(int i = 0; i < nbr_obs; i++)
		obstacles[i] = NULL;
	// read all obstacles now
	int loaded_obs = 0;
	for(int i = 0; i < nbr_obs; i++){
		obstacles[i] = new float[4];
		if (!(infile >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3]))
		{
			fprintf(stderr, "Error reading obstacle %d from /home/root/wanting/obstacle_list.txt\n", i);
			delete[] obstacles[i];
			obstacles[i] = NULL;
			break;
		}
		obstacles[i][0] -= margin;
		obstacles[i][1] -= margin;
		obstacles[i][2] += margin;
		obstacles[i][3] += margin;
		loaded_obs++;
	}
	nbr_obs = loaded_obs;

	for(int i = 0; i < nbr_obs; i++){
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}

	for (int idx = 0; idx < MAP_SIZE; ++idx)
	{
		float x = 0.0f;
		float y = 0.0f;
		index_to_position_2D(idx, &x, &y);
		Position p(x, y, 0);
		obstacle_map[idx] = (p.is_in_obstacle_list(obstacles, nbr_obs) == -1) ? 0 : 1;
	}
}

bool should_process_team_messages()
{
	return simulation_parameters.number_of_robots > 1 &&
	       simulation_parameters.teamStrategy == DISTRIBUTED;
}

bool should_share_team_wind()
{
	return simulation_parameters.number_of_robots > 1 &&
	       simulation_parameters.teamStrategy == DISTRIBUTED &&
	       simulation_parameters.windInfo == WIND_ONLINE;
}

bool should_use_log_posterior()
{
	return simulation_parameters.posteriorRepresentation == LOG_P;
}

void prepare_belief_map_for_sharing(float belief_map[MAP_SIZE])
{
	if (should_use_log_posterior())
	{
		if (posterior && posterior->pdf_is_log)
			BeliefMapFusion::sanitize_log_map(posterior->pdf, belief_map, obstacle_map);
		else if (posterior)
			BeliefMapFusion::pdf_to_log_map(posterior->pdf, belief_map, obstacle_map);
		return;
	}

	if (posterior)
		memcpy(belief_map, posterior->pdf, MAP_SIZE * sizeof(float));
}

void record_team_wind_sample(Position sample_position, float wind_u, float wind_v, int sender_ID, int sample_id)
{
	int sender_index = find_index_in_ID_list(sender_ID, simulation_parameters.ID_list);
	if (sender_index < 0 || sender_index >= MAX_NB_ROBOTS)
		return;

	if (sample_id <= latest_team_wind_sample_id[sender_index])
		return;

	latest_team_wind_sample_id[sender_index] = sample_id;
	history.team_wind_position.push_back(sample_position);
	history.team_wind_meas_u.push_back(wind_u);
	history.team_wind_meas_v.push_back(wind_v);
	history.team_wind_sender_id.push_back(sender_ID);
	history.team_wind_sample_id.push_back(sample_id);
}

void share_team_wind_sample(Position sample_position, float wind_u, float wind_v)
{
	if (!should_share_team_wind())
		return;

	int sample_id = ++local_wind_sample_id;
	record_team_wind_sample(sample_position, wind_u, wind_v, ID, sample_id);

	Position wind_payload(wind_v, static_cast<float>(sample_id), 0.0f);
	Message msg(MSG_STATE_WIND_SAMPLE, ID, 0, sample_position, wind_u, wind_payload, 0);
	inter_robot_comm.send_message_to_all(msg);
}

void process_team_messages()
{
	if (!should_process_team_messages())
		return;

	Message msg;
	int from_peer = 0;
	while (inter_robot_comm.poll_message(msg, from_peer, 0))
	{
		if (msg.sender_ID == ID)
			continue;
		if (msg.receiver_ID != 0 && msg.receiver_ID != ID)
			continue;

		int sender_index = find_index_in_ID_list(msg.sender_ID, simulation_parameters.ID_list);
		if (sender_index < 0 || sender_index >= MAX_NB_ROBOTS)
			continue;

		if (msg.msg_ID == MSG_STATE_WIND_SAMPLE)
		{
			if (should_share_team_wind())
			{
				int sample_id = static_cast<int>(msg.goal.y);
				record_team_wind_sample(msg.position, msg.concentration, msg.goal.x, msg.sender_ID, sample_id);
			}
			continue;
		}

		if (msg.finish_flag == -1 || msg.msg_ID == MSG_STATE_FINISHED)
		{
			FSM.state = STATE_FINISHED;
			printf("Robot ID [%d]: Received STATE_FINISHED from Robot ID [%d]\n", ID, msg.sender_ID);
			return;
		}

		if (msg.msg_ID == MSG_STATE_SHARE_PDF)
		{
			ready_from[sender_index] = 1;
			printf("Robot ID [%d]: Received READY from Robot ID [%d]\n", ID, msg.sender_ID);
			continue;
		}

		if (msg.msg_ID == MSG_STATE_BELIEF_MAP)
		{
			memcpy(belief_maps[sender_index], msg.belief_map, sizeof(float) * MAP_SIZE);
			received_current_pos[sender_index][0] = msg.position.x;
			received_current_pos[sender_index][1] = msg.position.y;
			received_flags[sender_index] = 1;
			printf("Robot ID [%d]: Received belief map from Robot ID [%d]\n", ID, msg.sender_ID);
			continue;
		}
	}
}


// ===================== LOG FILES FUNCTIONS ========================== //

void log_position(){
	double time = get_exp_time();
	goalposition_logfile = open_log_file(goalposition_logfile_name, "a");
	if (goalposition_logfile == NULL)
		return;
	fprintf(goalposition_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(){
	//int i = 0;
	double time = get_exp_time();
	measurements_logfile = open_log_file(measurements_logfile_name, "a");
	if (measurements_logfile == NULL)
		return;
	fprintf(measurements_logfile,"%d, %f, %f, %f, %f, %f\n",
	        FSM.n_cycles, sampleBuffer.average_wind_intensity,
	        sampleBuffer.average_wind_vx, sampleBuffer.average_wind_vy,
	        sampleBuffer.average_concentration, time);
	//for (i = 0; i < ODOR_FILTER_LENGTH; i++)
	//	fprintf(measurements_logfile,"%f, ", measurements.odor_sample_memory.buffer[i]);
	//fprintf(measurements_logfile,"\n");
	fclose(measurements_logfile);
}

void log_expected_a_posteriori()
{
	expected_a_posteriori_logfile = open_log_file(expected_a_posteriori_logfile_name, "a");
	if (expected_a_posteriori_logfile == NULL)
		return;
	fprintf(expected_a_posteriori_logfile, "%d", FSM.n_cycles);
	for (int i = 0; i < plume_model->nbr_param; ++i)
		fprintf(expected_a_posteriori_logfile, ", %f", plume_model->expected_value[i]);
	fprintf(expected_a_posteriori_logfile, "\n");
	fclose(expected_a_posteriori_logfile);
}

void log_max_a_posteriori()
{
	max_a_posteriori_logfile = open_log_file(max_a_posteriori_logfile_name, "a");
	if (max_a_posteriori_logfile == NULL)
		return;
	fprintf(max_a_posteriori_logfile, "%d", FSM.n_cycles);
	for (int i = 0; i < plume_model->nbr_param; ++i)
		fprintf(max_a_posteriori_logfile, ", %f", plume_model->max_a_post_value[i]);
	fprintf(max_a_posteriori_logfile, "\n");
	fclose(max_a_posteriori_logfile);
}

void log_entropy()
{
	double time = get_exp_time();
	entropy_logfile = open_log_file(entropy_logfile_name, "a");
	if (entropy_logfile == NULL)
		return;
	fprintf(entropy_logfile, "%f, %f\n", posterior->entropy, time);
	fclose(entropy_logfile);
}

void log_history(){
	static int logged_history_size = 0;

	if (logged_history_size > history.size)
		logged_history_size = 0;

	if (logged_history_size == 0){
		position_history_logfile = fopen(position_history_logfile_name, "w");
		if (position_history_logfile != NULL)
			fclose(position_history_logfile);

		measurement_history_logfile = fopen(measurement_history_logfile_name, "w");
		if (measurement_history_logfile != NULL)
			fclose(measurement_history_logfile);
	}

	if (logged_history_size >= history.size)
		return;

	position_history_logfile = fopen(position_history_logfile_name, "a");
	measurement_history_logfile = fopen(measurement_history_logfile_name, "a");

	if (position_history_logfile == NULL || measurement_history_logfile == NULL){
		if (position_history_logfile != NULL)
			fclose(position_history_logfile);
		if (measurement_history_logfile != NULL)
			fclose(measurement_history_logfile);
		return;
	}

	for (int i = logged_history_size; i < history.size; i++){
		fprintf(position_history_logfile, "%d, %f, %f, %f\n", i, history.position[i].x, history.position[i].y, history.position[i].heading);
		fprintf(measurement_history_logfile, "%d, %f, %f, %f, %f, %f, %f\n",
		        i, history.gas_meas_avg[i], history.gas_meas_std[i],
		        history.wind_intensity[i], history.wind_angle[i],
		        history.wind_meas_u[i], history.wind_meas_v[i]);
	}

	fclose(position_history_logfile);
	fclose(measurement_history_logfile);
	logged_history_size = history.size;
   
}

void log_visited_list(){
	visited_list_logfile = open_log_file(visited_list_logfile_name, "a");
	if (visited_list_logfile == NULL)
		return;

	for (int i = 0; i < MAP_SIZE; i++)
		fprintf(visited_list_logfile, "%d, ", cell_visited_list[i]);

	fprintf(visited_list_logfile, "\n");

	fclose(visited_list_logfile);
}

void log_trajectory(Position p){
 	double time = get_exp_time();
	trajectory_logfile = open_log_file(trajectory_logfile_name, "a");
	if (trajectory_logfile == NULL)
		return;
	fprintf(trajectory_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
 	fclose(trajectory_logfile);
}

// ============================== STATES ================================= //
void state_initializing(){

	 // Compute this robot's Y sub-region using sorted ID ranking
    if (initial_point_calculated == false) {

		int n_robots = simulation_parameters.number_of_robots;
		float y_range_min = YMIN + SAFETY_MARGIN;
		float y_range_max = YMAX - SAFETY_MARGIN;
		int robot_rank = 0;
		if (n_robots > 1) {
			std::vector<int> sorted_ids = simulation_parameters.ID_list;
			std::sort(sorted_ids.begin(), sorted_ids.end());
			for (int j = 0; j < (int)sorted_ids.size(); j++) {
				if (sorted_ids[j] == ID) { robot_rank = j; break; }
			}
		}
		float sub_height = (y_range_max - y_range_min) / n_robots;
		float y_sub_min = y_range_min + robot_rank * sub_height;
		float y_sub_max = y_range_min + (robot_rank + 1) * sub_height;
		do
		{
			//goal_position.x = randu() * (XMAX - XMIN - 2 * SAFETY_MARGIN) + XMIN;
			goal_position.x = XMIN;
			goal_position.y = y_sub_min + randu() * (y_sub_max - y_sub_min);
		} while (goal_position.is_in_obstacle_list(obstacles, nbr_obs) != -1);

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
		initial_point_calculated = true;
    }
    

    if(dist_to_goal(goal_position) > 0.05){
		if (!ensure_current_path_target("STATE_INITIALIZATION"))
		{
			set_wheels_speed(0,0);
			return;
		}
		//move
		if(obstacle_or_not() == 0){
			go_to_goal(*i);
			set_wheels_speed(speed[0],speed[1]);
		} else {
			//obstacle avoidance if necessary
			braitenberg_run(&braitenberg);
		}
		
	}else{
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		path.clear();
		khepera4_drive_set_speed(0, 0);
    	printf("[%d] Reached initial position (%.2f, %.2f). Starting FSM.\n",
           ID, current_position.x, current_position.y);
		FSM.state = STATE_MEASURING;
		FSM.hook  = state_measuring;
	}
}

void state_measuring(){
    printf("Robot ID [%d]: STATE_MEASURING \n", ID);
	std::vector<float> sample_from_map;
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
				history.gas_meas_avg.push_back(sample_from_map[i]);
			else
				history.gas_meas_avg.push_back(sample_in_motion.gas_meas_avg[i]);
			history.gas_meas_std.push_back(0.0);
			history.wind_intensity.push_back(sample_in_motion.wind_intensity[i]);
			history.wind_angle.push_back(sample_in_motion.wind_angle[i]);
			history.wind_meas_u.push_back(sample_in_motion.wind_meas_u[i]);
			history.wind_meas_v.push_back(sample_in_motion.wind_meas_v[i]);
			history.FSM_iter.push_back(FSM.n_cycles);
			history.size++;
			// mark the cell as visited
			if (simulation_parameters.visitedList == visitedList_ON)
			{
				int pos_coord[2];
				position_to_indices_2D(sample_in_motion.position[i].x, sample_in_motion.position[i].y, &pos_coord[0], &pos_coord[1]);
				int idx = posterior->indices_to_index(pos_coord);
				cell_visited_list[idx] = 1;
			}
		}
		sample_in_motion.position.clear();
		sample_in_motion.gas_meas_avg.clear();
		sample_in_motion.wind_intensity.clear();
		sample_in_motion.wind_angle.clear();
		sample_in_motion.wind_meas_u.clear();
		sample_in_motion.wind_meas_v.clear();
		sample_in_motion.size = 0;
		sample_from_map.clear();
	}
	if (simulation_parameters.samplingMode == STOP_AND_GO_SAMPLING){
		while(sampleBuffer.n_samples < ODOR_FILTER_LENGTH){
			//continue measuring odor concentration
			sampleBuffer.instant_concentration = odor_read();
			//measuring wind
			bool valid_wind = wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle,
										&sampleBuffer.instant_wind_vx, &sampleBuffer.instant_wind_vy, &sampleBuffer.instant_wind_vz);
			if (!valid_wind)
			{
				next_iteration_time();
				continue;
			}
			//add samples to buffers
			sampleBuffer.add_all_to_buffer();
			next_iteration_time();
			//printf("n %d wind intensity: %f, angle %f \n", sampleBuffer.n_samples, sampleBuffer.instant_wind_intensity, sampleBuffer.instant_wind_angle);
		}
		//average all samples
		sampleBuffer.average_all();
	} else if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING || simulation_parameters.samplingMode == SENSE_WITHOUT_STOP){
		sampleBuffer.instant_concentration = odor_read();
		bool valid_wind = wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle,
									&sampleBuffer.instant_wind_vx, &sampleBuffer.instant_wind_vy, &sampleBuffer.instant_wind_vz);
		if (!valid_wind)
			return;
		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
	}
	
	//log and save the measurements at the goal position
	if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
	log_position();
	log_measurements();
	history.position.push_back({current_position.x, current_position.y, current_position.heading});
	float calibrated_sensor_reading = (sampleBuffer.average_concentration - odor_baseline_value < 0) ? 0 : calibrate_sensor_reading(sampleBuffer.average_concentration - odor_baseline_value);
	if (calibrated_sensor_reading < 0) calibrated_sensor_reading = 0.0;
	history.gas_meas_avg.push_back(calibrated_sensor_reading);
	// if(history.gas_meas_avg[history.size] < 0) history.gas_meas_avg[history.size] = 0;
	history.gas_meas_std.push_back(sampleBuffer.stdDev_concentration);
	history.wind_intensity.push_back(sampleBuffer.average_wind_intensity);
	history.wind_angle.push_back(sampleBuffer.average_wind_angle);
	history.wind_meas_u.push_back(sampleBuffer.average_wind_vx);
	history.wind_meas_v.push_back(sampleBuffer.average_wind_vy);
	share_team_wind_sample(Position(current_position.x, current_position.y, current_position.heading),
	                       sampleBuffer.average_wind_vx, sampleBuffer.average_wind_vy);
	history.size++;
	//history_print(history);

	if (simulation_parameters.visitedList == visitedList_ON)
	{
		int pos_coord[2];
		position_to_indices_2D(current_position.x, current_position.y, &pos_coord[0], &pos_coord[1]);
		int idx = posterior->indices_to_index(pos_coord);
		cell_visited_list[idx] = 1;
	}
	FSM.state = STATE_ESTIMATING;
	FSM.hook  = state_estimating;
}
void state_estimating(){
	printf("Robot ID [%d]: STATE_ESTIMATING!!! \n", ID);
	// if (VERBOSE)
	// 	printf("[%d] estimating...\n", ID);

	long long time_before_estimating = current_timestamp();
	long long wind_gas_time_before_estimating = wind_gas_prediction_time_sum;

	// reinitialize
	posterior->pdf_reset();
	// estimation
	if (simulation_parameters.BayesianModelingMethod == MCMC)
	{
		//printf("going for MCMC\n");
		posterior->MCMC_evaluation(history);
	}
	if (simulation_parameters.BayesianModelingMethod == EXHAUSTIVE)
	{
		//printf("going for exhaustive\n");
		posterior->exhaustive_evaluation(history);
	}
	// only when using MCMC
	if (posterior->dirac_flag)
	{
		if (VERBOSE)
			printf("[%d] exhaustive search!\n", ID);
		posterior->exhaustive_evaluation(history);
	}

	// printf("sum posterior : %f \n", posterior->sum);
	posterior->normalize();
	posterior->log_pdf(posterior_pdf_logfile_name, 'a');

	FSM.n_cycles++;
	if (VERBOSE)
		printf("[%d] cycle number %d\n", ID, FSM.n_cycles);

	long long current_estimation_time = current_timestamp() - time_before_estimating;
	long long current_wind_gas_time = wind_gas_prediction_time_sum - wind_gas_time_before_estimating;
	long long current_belief_update_time = current_estimation_time - current_wind_gas_time;
	if (current_belief_update_time < 0)
		current_belief_update_time = 0;

	estimating_time_sum += current_estimation_time;
	belief_update_time_sum += current_belief_update_time;

	// check if stopping condition is reached
	if ((simulation_parameters.number_of_robots == 1)||(simulation_parameters.teamStrategy == INDIVIDUAL))
	{
		posterior->convert_log_pdf_to_pdf();
		// calculate entropy and spread
		posterior->process();
      	posterior->calculate_entropy_and_spread();
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
	path.clear();
	use_visibility_path = true;
	switch (simulation_parameters.planningStrategy)
	{
	case GREEDY:
		path_planner->maximal_belief_planning();
		break;
	case INFO_CENTER:
		path_planner->information_center_planning();
		break;
	case BALANCE_MAXIMAL_EXPECTED:
		path_planner->balance_expected_maximal_pdf();
		break;
	case RANDOM:
		path_planner->random_planning();
		break;
	case LAWNMOWER:
		path_planner->lawnmower_planning();
		break;
	case BELIEF_CLUSTERING:
		path_planner->belief_clustering_planning();
		break;
	case WEIGHTED_CLUSTERING:
		path_planner->weighted_belief_clustering();
		break;
	case INFO_CLUSTERING:
		path_planner->informative_clustering_planning();
		break;
	case INFORMATIVE_KLD:
		path_planner->KLD_informative_goal_selection();
		path_planner->KLD_distance_balance();
		break;
	case SUM_DIJKSTRA_FIX:
		path_planner->accumulative_Dijkstra();
		break;
	case SUM_DIJKSTRA_ADP:
		path_planner->accumulative_Dijkstra();
		break;
	case MULTI_DIJKSTRA:
		path = path_planner->multi_objective_Dijkstra();
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
			path_planner->goal_outside_obstacles(&goal_position);
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
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			int idx = position_to_index_2D(goal_position.x, goal_position.y);
			if (idx >= 0 && idx < MAP_SIZE)
				cell_visited_list[idx] = 1;
		}
		// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		// update goal
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		time_before_moving = current_timestamp();
		long long current_path_planning_time = current_timestamp() - time_before_path_planning;
		planning_time_sum += current_path_planning_time;
		path_planning_time_sum += current_path_planning_time;
		
	}
}
bool entered_state_moving = false;
static constexpr float GOAL_REACHED_WITH_FRONT_OBSTACLE_DIST = 0.25f;

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
		float wind_intensity = 0.0;
		float wind_angle = 0.0;
		float wind_vx = 0.0;
		float wind_vy = 0.0;
		float wind_vz = 0.0;
		bool valid_wind = wind_read(&wind_intensity, &wind_angle, &wind_vx, &wind_vy, &wind_vz);
		if (valid_wind)
		{
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
						sample_in_motion.position.push_back({current_position.x, current_position.y, current_position.heading});
						sample_in_motion.gas_meas_avg.push_back(instant_concentration);
						sample_in_motion.wind_intensity.push_back(wind_intensity);
						sample_in_motion.wind_angle.push_back(wind_angle);
						sample_in_motion.wind_meas_u.push_back(wind_vx);
						sample_in_motion.wind_meas_v.push_back(wind_vy);
						share_team_wind_sample(Position(current_position.x, current_position.y, current_position.heading), wind_vx, wind_vy);
						sample_in_motion.size++;

					    last_measurement_pos.x = current_position.x;
					    last_measurement_pos.y = current_position.y;
						//printf("sample in motion size: %d, current position: (%f, %f), measurement: %f \n", sample_in_motion.size, current_position.x, current_position.y, instant_concentration);
					}
					
				//}
				sample_counter = 0;
			}
		}
	}
	float goal_dist = dist_to_goal(goal_position);
	if(goal_dist > 0.05){
		if (!ensure_current_path_target("STATE_MOVING"))
		{
			set_wheels_speed(0,0);
			sampleBuffer.reset_all();
			path.clear();
			FSM.state = STATE_MEASURING;
			FSM.hook = state_measuring;
			time_after_moving = current_timestamp();
			navigation_sampling_time_sum += time_after_moving - time_before_moving;
			robot_moving_time_sum += static_cast<long long>(time_after_moving - time_before_moving);
			time_before_measuring = current_timestamp();
			entered_state_moving = false;
			return;
		}
		//move
		go_to_goal(*i);
		//obstacle avoidance if necessary
		if(obstacle_in_front_enter() == 1 && goal_dist <= GOAL_REACHED_WITH_FRONT_OBSTACLE_DIST){
			printf("Robot ID [%d]: Front obstacle within %.2f m of goal; treating goal as reached.\n",
			       ID, GOAL_REACHED_WITH_FRONT_OBSTACLE_DIST);
			set_wheels_speed(0,0);
			sampleBuffer.reset_all();
			path.clear();
			if(finished){
				time_after_moving = current_timestamp();
				navigation_sampling_time_sum += time_after_moving - time_before_moving;
				robot_moving_time_sum += static_cast<long long>(time_after_moving - time_before_moving);
				entered_state_moving = false;
				FSM.state = STATE_FINISHED;
			}else{
				FSM.state = STATE_MEASURING;
				FSM.hook  = state_measuring;
				time_after_moving = current_timestamp();
				navigation_sampling_time_sum += time_after_moving - time_before_moving;
				robot_moving_time_sum += static_cast<long long>(time_after_moving - time_before_moving);
				time_before_measuring = current_timestamp();
				entered_state_moving = false;
			}
			return;
		}
		if(obstacle_or_not_enter() == 1){
			FSM.state = STATE_OBSTACLE_AVOIDANCE;
			FSM.hook  = state_obstacle_avoidance;
			return;
		}
		//wb_differential_wheels_set_speed(speed[0],speed[1]);
		set_wheels_speed(speed[0],speed[1]);
	}else{
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		sampleBuffer.reset_all();
		path.clear();
		if(finished){
			time_after_moving = current_timestamp();
			navigation_sampling_time_sum += time_after_moving - time_before_moving;
			robot_moving_time_sum += static_cast<long long>(time_after_moving - time_before_moving);
			entered_state_moving = false;
			FSM.state = STATE_FINISHED;
		}else{
			//back to measuring
			FSM.state = STATE_MEASURING;
			FSM.hook  = state_measuring;
			time_after_moving = current_timestamp();
			navigation_sampling_time_sum += time_after_moving - time_before_moving;
			robot_moving_time_sum += static_cast<long long>(time_after_moving - time_before_moving);
			time_before_measuring = current_timestamp();
			printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
			entered_state_moving = false;
		}
	}
}

void state_obstacle_avoidance(){
	if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	
	//if no obstacle,
	if(obstacle_or_not_exit() == 0 || dist_to_goal(goal_position) <= 0.05){
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}
	braitenberg_run(&braitenberg);
}

void state_receive_pdf()
{
	if (!entered_state_receive_pdf)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_PDF \n", ID);
		entered_state_receive_pdf = true;
	}

	if (!printed && posterior->pdf != nullptr)
	{
		prepare_belief_map_for_sharing(belief_maps[self_index]);
		printed = true;
	}

	process_team_messages();
	if (FSM.state == STATE_FINISHED)
		return;

	int all_received = 1;
	for (int i = 0; i < simulation_parameters.number_of_robots; i++)
	{
		if (i == self_index)
			continue;
		if (!received_flags[i])
		{
			all_received = 0;
			break;
		}
	}

	if (all_received && !i_am_ready)
	{
		Message ready_msg(MSG_STATE_SHARE_PDF, ID, 0, Position(), 0.0f, Position(), 1);
		inter_robot_comm.send_message_to_all(ready_msg);
		i_am_ready = 1;
		printf("Robot ID [%d]: Sent READY message.\n", ID);
	}

	int ready_count = 0;
	for (int i = 0; i < simulation_parameters.number_of_robots; i++)
	{
		if (i == self_index)
			continue;
		if (ready_from[i])
			ready_count++;
	}

	if (i_am_ready && ready_count == simulation_parameters.number_of_robots - 1)
	{
		entered_state_receive_pdf = false;
		printed = false;
		i_am_ready = 0;

		for (int i = 0; i < simulation_parameters.number_of_robots; i++)
		{
			received_flags[i] = 0;
			ready_from[i] = 0;
		}

		printf("Robot ID [%d]: All robots READY. Transitioning to STATE_MERGE_PDF.\n", ID);
		FSM.state = STATE_MERGE_PDF;
		FSM.hook = state_merge_pdf;
	}
}

void state_merge_pdf()
{
	printf("Robot ID [%d]: STATE_MERGE_PDF \n", ID);
	long long time_before_belief_fusion = current_timestamp();
	bool merge_ok = false;
	if (should_use_log_posterior())
	{
		merge_ok = BeliefMapFusion::fuse_log_maps(belief_maps, simulation_parameters.number_of_robots, belief_map_merged);
		if (!merge_ok)
			printf("Robot ID [%d]: Failed to normalize merged log belief map.\n", ID);
		BeliefMapFusion::log_map_to_pdf(belief_map_merged, belief_map_merged);
	}
	else
	{
		merge_ok = BeliefMapFusion::fuse_normal_maps(belief_maps, simulation_parameters.number_of_robots, belief_map_merged);
		if (!merge_ok)
			printf("Robot ID [%d]: Failed to normalize merged belief map.\n", ID);
	}

	// printf("Robot ID [%d]: Belief map merged successfully.\n", ID);
	printf("Robot ID [%d]: Merged belief map is: \n", ID);
	for (int i = 0; i < MAP_SIZE; i++)
	{
		printf("%f ", belief_map_merged[i]);
	}
	printf("\n");
	memcpy(posterior->pdf, belief_map_merged, MAP_SIZE * sizeof(float));
	posterior->pdf_is_log = false;
	posterior->sum = 0.0f;
	for (int i = 0; i < MAP_SIZE; ++i)
		posterior->sum += posterior->pdf[i];
      // Check the stopping condition after merging
	posterior->process();
      posterior->calculate_entropy_and_spread();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	bool is_stop_condition_reached = check_stop_condition();
	if (is_stop_condition_reached)
	{
		if (posterior->sum > 0.0f && merge_ok)
		{
			belief_fusion_time_sum += current_timestamp() - time_before_belief_fusion;
			FSM.state = STATE_FINISHED;
			printf("Robot ID [%d]: STATE_FINISHED\n", ID);
			broadcast_state_finished();
			return;
		}
		printf("Robot ID [%d]: Avoided STATE_FINISHED because merged belief map is zero\n", ID);
		FSM.state = STATE_MEASURING;
		FSM.hook = state_measuring;
		belief_fusion_time_sum += current_timestamp() - time_before_belief_fusion;
		return;
	} else {
		if (simulation_parameters.teamStrategy == DISTRIBUTED)
		{
			if (simulation_parameters.mapCoverageStrategy != NONE)
			{
				FSM.state = STATE_REGION_ALLOCATION;
				FSM.hook = state_region_allocation;
			}
			else
			{
				FSM.state = STATE_UPDATEGOAL;
				FSM.hook = state_updategoal;
			}
		}
	}
	belief_fusion_time_sum += current_timestamp() - time_before_belief_fusion;
	
}

void state_share_pdf()
{
    printf("Robot ID [%d]: STATE_SHARE_PDF\n", ID);

    if (posterior->pdf == NULL) {
        printf("Robot ID [%d]: No posterior PDF to share.\n", ID);
    } else {
		received_current_pos[self_index][0] = current_position.x;
		received_current_pos[self_index][1] = current_position.y;
		float belief_map_to_share[MAP_SIZE];
		prepare_belief_map_for_sharing(belief_map_to_share);
		Message msg(MSG_STATE_BELIEF_MAP, ID, 0, current_position, 0.0f, Position(), 0, belief_map_to_share);
		inter_robot_comm.send_message_to_all(msg);
		printf("Robot ID [%d]: Broadcast belief map.\n", ID);
    }

    FSM.state = STATE_RECEIVE_PDF;
    FSM.hook = state_receive_pdf;
}

void state_region_allocation()
{
	printf("Robot ID [%d]: STATE_REGION_ALLOCATION \n", ID);
	long long time_before_region_allocation = current_timestamp();
	if (simulation_parameters.teamStrategy == DISTRIBUTED)
	{
		posterior->log_pdf(merged_pdf_logfile_name, 'a');
		if (region_allocator)
			region_allocator->region_allocation_algorithm();
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;
	}
	region_allocation_time_sum += current_timestamp() - time_before_region_allocation;
}

// Broadcast STATE_FINISHED to all teammates
void broadcast_state_finished()
{
    printf("Robot ID [%d]: Broadcasting STATE_FINISHED\n", ID);

	Message msg(MSG_STATE_FINISHED, ID, 0, current_position, 0.0f, Position(), -1);
	inter_robot_comm.send_message_to_all(msg);
}

bool check_stop_condition()
{
	bool is_stop_condition_reached = false;
	printf("current_entropy is %f\n", posterior->entropy);
	if (simulation_parameters.stopCondition == ENTROPY_ONLY){
		if (FSM.n_cycles == MAX_ITER || posterior->entropy < 0.1 ){
			is_stop_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == SPREAD_ONLY){
		if (FSM.n_cycles == MAX_ITER || (posterior->x_spatial_spread < 2 && posterior->y_spatial_spread < 2)){
			is_stop_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == ENTROPY_SPREAD){
		if (FSM.n_cycles == MAX_ITER || (posterior->entropy < 0.3 && posterior->x_spatial_spread < 2 && posterior->y_spatial_spread < 2)){
			is_stop_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == ITERATION_ONLY){
		if (FSM.n_cycles == MAX_ITER){
			is_stop_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == END_OF_FIX_PATH){
		if (path_planner->reach_end_of_fix_path){
			is_stop_condition_reached = true;
		}
	}
	printf("entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior->entropy, posterior->x_spatial_spread, posterior->y_spatial_spread);
	//fprintf(console_logfile, "entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior->entropy, posterior->x_spatial_spread, posterior->y_spatial_spread);
	if (is_stop_condition_reached)
	{
		// log the finish time stamp
		end_timestamp = current_timestamp();
		printf("Robot ID [%d]: Stop condition reached!\n", ID);
		//fprintf(console_logfile, "Robot ID [%d]: Stop condition reached!\n", ID);
	}
	return is_stop_condition_reached;
}

PlumeModel* initialize_plume_model()
{
	if (simulation_parameters.plumeModel == FDM_PDE)
	{
		printf("Initializing ADE-FDM plume model.\n");
		return new AdeFDMPlumeModel2D();
	}
	if (simulation_parameters.plumeModel == GAUSSIAN_PLUME)
	{
		printf("Initializing Gaussian plume model.\n");
		return new GaussianPlumeModel2D();
	}

	printf("Initializing precomputed DDPM plume model.\n");
	return new DeepCFDPlumeModel2D();
}

void initialize_plume_stack()
{
	if (plume_model || posterior || path_planner || region_allocator)
		return;

	try
	{
		plume_model = initialize_plume_model();
		posterior = new Posterior(plume_model);
		path_planner = new PathPlanner(plume_model, posterior, &history);
		region_allocator = new RegionAllocator(posterior, received_current_pos);
	}
	catch (const std::exception &e)
	{
		fprintf(stderr, "Failed to initialize plume stack: %s\n", e.what());
		exit(1);
	}
}

// =========================== MAIN ============================== //


int main(int argc, char **argv){
	
	init();
	inter_robot_comm.initialize(ID, simulation_parameters.ID_list);

	read_obstacle_list();

	establish_baseline();
    initialize_plume_stack();

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
	start_timestamp = current_timestamp();
	while (FSM.state != STATE_FINISHED){
		update_position();
		process_team_messages();
		if (FSM.state == STATE_FINISHED)
			break;
	   	//change state
		FSM.hook();
		log_history();
		//next
		next_iteration_time();
		n_time_step++;
	}
	log_experiment_time();
	if (clustering_debugging_logfile && clustering_debugging_logfile != stderr)
		fclose(clustering_debugging_logfile);
	cleaning_up();
	delete region_allocator;
	delete path_planner;
	delete posterior;
	delete plume_model;
	//free the allocated memory to obstacle array
	for(int i = 0; i < nbr_obs; i++)
		delete[] obstacles[i];
		delete[] obstacles;
		exit(0);
}
