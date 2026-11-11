/*
 * File:          controller_STE_multi-robot.cpp
 * Date:          Jan 2019
 * Description:   The goal is to estimate the source terms including the position
 * Author:        Faezeh Rahbar; Wanting Jin
 * Modifications: 30/07/2024
 */

// custom libraries
#include <fstream>
#include <iostream>
#include <chrono>
#include <memory>
#include "ste_package/ste_node.h"
// #include "nvwa/debug_new.h"
#include "ste_package/Posterior.h"
#include "ste_package/Message.h"
#include "ste_package/robot_functions.h"
#include "ste_package/path_planner.h"
// #include "aStar.h"
#include "ste_package/visilibity.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <string>

using namespace std;
using namespace std::chrono_literals;

// Global Variables

// log files
char folder_name[256];
FILE *goalposition_logfile;
FILE *measurements_logfile;
FILE *expected_a_posteriori_logfile;
FILE *likelihood_logfile;
FILE *measurement_history_logfile;
FILE *position_history_logfile;
FILE *max_a_posteriori_logfile;
FILE *posterior_pdf_3D;
FILE *entropy_logfile;
FILE *config_logfile;
FILE *trajectory_logfile;
FILE *gas_concentration_map_logfile;
FILE *experiment_time_logfile;
FILE *visited_list_logfile;

// text file for storing some prints for debugging
FILE *console_logfile;

char goalposition_logfile_name[512];
char measurements_logfile_name[512];
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
char gas_concentration_maps_log_root[512];
char experiment_time_logfile_name[512];
char visited_list_logfile_name[512];
char clustering_bounding_boxes_logfile_name[512];
char gas_information_map_logfile_name[512];
char scores_map_logfile_name[512];
char distance_map_logfile_name[512];
char acc_info_map_logfile_name[512];
char weighted_belief_map_logfile_name[512];
char relative_history_meas_logfile_name[512];
char wind_field_u_logfile_name[512];
char wind_field_v_logfile_name[512];
char wind_field_w_logfile_name[512];

// text file for storing some prints for debugging
char console_logfile_name[512];

// read data from file
char logfile_folder_name[256];
FILE *experiment_measurements_logfile;
char experiment_measurements_logfile_name[512];
FILE *experiment_position_logfile;
char experiment_position_logfile_name[512];
FILE *experiment_goalposition_logfile;
char experiment_goalposition_logfile_name[512];

// define the global parameter - the root path of the project
const char *workspacePath = getenv("STE_DDPM_ROOT");
const std::string config_folder_path = ament_index_cpp::get_package_share_directory("ste_package");

// finite state machine
struct FSM FSM;

// robot's measurements
SampleBuffer sampleBuffer(ODOR_FILTER_LENGTH);

// position in m
Position goal_position;
Position goal_position_team[3];
Position current_position;
double speed[2] = {0};

// robot's position and measurement history
struct History history;
struct History sample_in_motion;
int history_size_last_cycle = 0;

int sample_counter = 0;

// simulation parameters
SimulationParameters simulation_parameters;

// Declare a pointer to the base class or common interface for plume models
PlumeModel* plume_model = nullptr;

// current posterior distribution
// Posterior posterior(plume_model);
Posterior* posterior= nullptr;
PathPlanner* path_planner = nullptr;

RosBridge* ros = nullptr;

// robot's ID
int ID;

// message structure
struct message message;

// list of other robots
struct friends friends[MAX_NB_ROBOTS];

int n_time_step = 0;

double previous_sensor_reading = 0.0;

// gas map
// param: sigma, gamma, scaling_param, wind_speed, rco
GasMap2D gas_map(0.2, 4, 1, 0.75, 0.3);

// obstacles definitions
int nbr_obs = 4;
float margin = 0.16;
// std::vector<std::array<float, 4>> obstacles;
float **obstacles = NULL;
std::list<Position> path;
std::list<Position>::iterator i;

// visibility graph
double epsilon = 0.000000001;
VisiLibity::Environment *my_environment = nullptr;
VisiLibity::Visibility_Graph *vg = nullptr;

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

// store last position for minimum distance threshold in measurement update
Position last_measurement_pos;

// real source position
double real_source_pos[3] = {0, 0, 0};

bool use_visibility_path = true;

// define the source bound for the estimation
SourceBounds source_bounds;

// store the belief maps of the whole team
std::vector<std::vector<double>> belief_maps;
// visited list
std::vector<int> cell_visited_list;


// =========================== FUNCTIONS ================================ //

// get the current time in timestamp (ms)
long long current_timestamp()
{
	struct timeval te;
	gettimeofday(&te, NULL);						     // get current time
	long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

// show the measurement history on screen
void history_print(struct History history)
{
	printf("_____ Sample History _____ \n");
	for (int i = history_size_last_cycle; i < history.size; i++)
	{
		printf("%d : %f @ (%f, %f)\n", i, history.gas_meas[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
	history_size_last_cycle = history.size;
}

void read_obstacle_list()
{
	// open file
	
	std::string obstacle_file = config_folder_path + "/model/test_maps/map" + std::to_string(simulation_parameters.map_nbr) + "/" + std::to_string(simulation_parameters.map_nbr) + "_obstacles.txt";
	printf("Obstacle file path: %s\n", obstacle_file.c_str());
	std::ifstream infile(obstacle_file.c_str());
	printf("Reading obstacle list from %s\n", obstacle_file.c_str());
	// first line is the number of obstacles
	infile >> nbr_obs;
	printf("There is %d obsatcle in total:\n", nbr_obs);
	obstacles = new float*[nbr_obs];
	for (int i = 0; i < nbr_obs; i++)
	{
		obstacles[i] = new float[4];
		infile >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3];
	}

	for (int i = 0; i < nbr_obs; i++)
	{
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}
}

// ===================== LOG FILES FUNCTIONS ========================== //

void log_position()
{
	// get current clock time
	double time = current_timestamp();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile, "%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(int n_cycle, float x, float y, double wind_u, double wind_v, double concentration)
{
	// int i = 0;
	measurements_logfile = fopen(measurements_logfile_name, "a");
	fprintf(measurements_logfile, "%d, %f, %f, %lf, %lf, %lf\n", n_cycle, x, y, wind_u, wind_v, concentration);
	fclose(measurements_logfile);
}

void log_expected_a_posteriori()
{
	expected_a_posteriori_logfile = fopen(expected_a_posteriori_logfile_name, "a");
	fprintf(expected_a_posteriori_logfile, "%d, %f, %f, %f, %f, %f\n", FSM.n_cycles,
		  plume_model->expected_value[0], plume_model->expected_value[1], plume_model->expected_value[2],
		  plume_model->expected_value[3], plume_model->expected_value[4]);
	fclose(expected_a_posteriori_logfile);
}

void log_max_a_posteriori()
{
	max_a_posteriori_logfile = fopen(max_a_posteriori_logfile_name, "a");
	fprintf(max_a_posteriori_logfile, "%d, %f, %f, %f, %f, %f\n", FSM.n_cycles,
		  plume_model->max_a_post_value[0], plume_model->max_a_post_value[1], plume_model->max_a_post_value[2],
		  plume_model->max_a_post_value[3], plume_model->max_a_post_value[4]);
	fclose(max_a_posteriori_logfile);
}

void log_entropy()
{
	double time = current_timestamp();
	entropy_logfile = fopen(entropy_logfile_name, "a");
	fprintf(entropy_logfile, "%f, %f\n", posterior->entropy, time);
	fclose(entropy_logfile);
}

void log_history()
{
	// position
	// FILE* position_history_logfile;
	position_history_logfile = fopen(position_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++)
	{
		fprintf(position_history_logfile, "%d, %f, %f, %f\n", i, history.position[i].x, history.position[i].y, history.position[i].heading);
	}
	fclose(position_history_logfile);
	// measurement
	// FILE * measurement_history_logfile;
	measurement_history_logfile = fopen(measurement_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++)
	{
		if (simulation_parameters.gasFeature == ABSOLUTE_VALUE)
		{
			fprintf(measurement_history_logfile, "%d, %f, %d, %f, %f\n", i, history.gas_meas[i], history.FSM_iter[i], history.wind_meas_u[i], history.wind_meas_v[i]);
		}
		if (simulation_parameters.gasFeature == RELATIVE_VALUE)
		{
			fprintf(measurement_history_logfile, "%d, %f, %f, %d, %f, %f\n", i, history.gas_meas[i], history.relative_meas[i], history.FSM_iter[i], history.wind_meas_u[i], history.wind_meas_v[i]);
		}
		if (simulation_parameters.gasFeature == GASHIT)
		{
			fprintf(measurement_history_logfile, "%d, %f, %d, %d, %f, %f\n", i, history.gas_meas[i], history.gas_hit_event[i], history.FSM_iter[i], history.wind_meas_u[i], history.wind_meas_v[i]);
		}
	}

	fclose(measurement_history_logfile);
}

void log_visited_list()
{
	visited_list_logfile = fopen(visited_list_logfile_name, "a");

	for (int i = 0; i < source_bounds.map_size; i++)
		fprintf(visited_list_logfile, "%d, ", cell_visited_list[i]);

	fprintf(visited_list_logfile, "\n");

	fclose(visited_list_logfile);
}

void log_trajectory(Position p)
{
	double time = current_timestamp();
	trajectory_logfile = fopen(trajectory_logfile_name, "a");
	fprintf(trajectory_logfile, "%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
	fclose(trajectory_logfile);
}

void log_experiment_time()
{
	double experiment_time = end_timestamp - start_timestamp;
	experiment_time_logfile = fopen(experiment_time_logfile_name, "w");

	double computation_time = (estimating_time_sum + planning_time_sum) / 1000.0;
	double total_time = experiment_time + computation_time;

	fprintf(experiment_time_logfile, "total experiment time (s): %f\n", experiment_time);
	fprintf(experiment_time_logfile, "navigation and sampling time (s): %f\n", navigation_sampling_time_sum);
	fprintf(experiment_time_logfile, "estimating time (s): %f\n", estimating_time_sum / 1000.0);
	fprintf(experiment_time_logfile, "planning time (s): %f\n", planning_time_sum / 1000.0);
	fprintf(experiment_time_logfile, "computation time (s): %f\n", computation_time);
	fprintf(experiment_time_logfile, "total time (s): %f\n", total_time);
	fclose(experiment_time_logfile);
}

// ============================== STATES ================================= //

void state_initializing(){
	printf("Robot ID [%d]: STATE_INITIALIZING \n", ID);
	
// waiting for some time to let the robot stabilize; and the message to confirm connection
	if (ros->shakehand_with_robot() == false){
		printf("Robot ID [%d]: waiting for handshake...\n", ID);
		FSM.state = STATE_INITIALIZING;
		FSM.hook  = state_initializing;
		//next_iteration_time();
	
	} else {
		std::list<Position> start_path;
		float start_x, start_y;
		if (simulation_parameters.workingMode == EXPERIMENT){
			// set the start position based on the configuration parameter
			switch (simulation_parameters.robot_start_position_config){
				case 0:
					{
						start_x = XMAX -0.5 ;
						start_y = YMIN + 0.5;
						break;
					}
				case 1:
					{
						start_x = XMAX -0.5 ;
						start_y = YMAX -0.5;
						break;	
					}
				case 2:
					{
						start_x = XMIN +0.5 ;
						start_y = YMAX -0.5;
						break;	
					}
				case 3:
					{
						start_x = XMIN +0.5 ;
						start_y = YMIN +0.5;
						break;	
					}
				case 4:
					{
						start_x = -2;//(XMIN + XMAX) / 2;
						start_y = (YMIN + YMAX) / 2;
						break;	
					}
			}
		}
		if (simulation_parameters.workingMode == VIDEO){
			read_goal_position();
			start_x = goal_position.x;
			start_y = goal_position.y;
		}

		printf("Robot ID [%d]: start position from configuration %d: (%.2f, %.2f)\n", ID, simulation_parameters.robot_start_position_config, start_x, start_y);
		// FIXME: some bug here to get the current Pose, always return 0, maybe need another ros spin
		//ros->getCurrentPose(&current_position.x, &current_position.y, &current_position.heading);
		// temporary fix
		current_position.x = XMAX-1.0;
		current_position.y = YMIN+1.0;
		printf("Robot ID [%d]: current position: (%.2f, %.2f)\n", ID, current_position.x, current_position.y);
		// find the path using visibility graph
		VisiLibity::Point start(current_position.x, current_position.y);
		VisiLibity::Point finish(start_x, start_y);
		VisiLibity::Polyline Polyline_path = my_environment->shortest_path(start, finish, *vg, epsilon);
		// copy the path in another variable
		// skip the first point (current position)
		for (unsigned k = 1; k < Polyline_path.size(); k++)
		{
			Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
			start_path.push_back(p);
			printf(" waypoint %d: (%f, %f) \n", k, p.x, p.y);
		
		}
		ros->send_waypoints(start_path);
		FSM.state = STATE_LISTENING;
		FSM.hook  = state_listening;
	}
}
	


void state_listening(){
	// printf("Receiving measurements... \n");
    // if (ros->getCurrentPose(&current_position.x, &current_position.y, &current_position.heading)){
	// 	printf("Current Position: x = %f, y = %f, heading = %f\n", current_position.x, current_position.y, current_position.heading);
	// 	// while (current_position.dist_to_goal(goal_position) > 0.05){
	// 	// 	current_position.calculate_index();
	// 	//     // pushback the result into history
	// 	// }
	// }
		
	// FSM.state = STATE_LISTENING;
	// FSM.hook  = state_listening;

	if (!ros->robot_reached_goal){
		//printf("Robot ID [%d]: waiting to reach position...\n", ID);
		FSM.state = STATE_LISTENING;
		FSM.hook  = state_listening;
	}
	else {
		//copy the measurement history from rosbridge
		printf("Robot ID [%d]: reached position.\n", ID);
		struct History last_history = *(ros->get_meas_history());
        printf("Received history size: %d\n", last_history.size);
		// add the new measurements to history
		if (FSM.n_cycles == 0){
			// for the first cycle, only copy the value of the goal position
			history.position.push_back(last_history.position.back());
			history.gas_meas.push_back(last_history.gas_meas.back());
			history.wind_meas_u.push_back(last_history.wind_meas_u.back());
			history.wind_meas_v.push_back(last_history.wind_meas_v.back());
			history.FSM_iter.push_back(FSM.n_cycles);	
			history.size++;
		}else{
			for (int i = 0; i < last_history.size; i++){
				history.position.push_back(last_history.position[i]);
				history.gas_meas.push_back(last_history.gas_meas[i]);
				history.wind_meas_u.push_back(last_history.wind_meas_u[i]);
				history.wind_meas_v.push_back(last_history.wind_meas_v[i]);
				history.FSM_iter.push_back(FSM.n_cycles);	
				history.size++;
			}
		}
		current_position = last_history.position.back();
		history_print(history);
		FSM.state = STATE_ESTIMATING;
		FSM.hook  = state_estimating;
		log_position();
	}
}


void state_estimating()
{
	printf("Robot ID [%d]: STATE_ESTIMATING \n", ID);
	// if (VERBOSE)
	// 	printf("[%d] estimating...\n", ID);

	long long time_before_estimating = current_timestamp();

	// reinitialize
	posterior->pdf_reset();
	// estimation
	if (simulation_parameters.BayesianModelingMethod == MCMC)
	{
		printf("going for MCMC\n");
		posterior->MCMC_evaluation(history);
	}
	if (simulation_parameters.BayesianModelingMethod == EXHAUSTIVE)
	{
		printf("going for exhaustive\n");
		posterior->exhaustive_evaluation(history);
	}
	// only when using MCMC
	if (posterior->dirac_flag)
	{
		if (VERBOSE)
			printf("[%d] exhaustive search!\n", ID);
		posterior->exhaustive_evaluation(history);
	}

	//printf("sum posterior : %f \n", posterior->sum);
	posterior->normalize();
	//printf("sum posterior after normalization : %f \n", posterior->sum);
	posterior->log_pdf(posterior_pdf_3D_name, 'a');
	if (simulation_parameters.gasFeature == RELATIVE_VALUE)
		posterior->log_relative_history_meas(relative_history_meas_logfile_name, 'a', history.relative_meas);

	FSM.n_cycles++;
	if (VERBOSE)
		printf("[%d] cycle number %d\n", ID, FSM.n_cycles);
	// if (VERBOSE)
	// 	printf("[%d] entropy : %f\n", ID, posterior.entropy);

	long long current_estimation_time = current_timestamp() - time_before_estimating;
	// printf("Current estimation time is : %lld\n", current_estimation_time);

	estimating_time_sum += current_estimation_time;
	// printf("estimating_time_sum : %lld\n", estimating_time_sum);

	// check if stopping condition is reached
	// calculate entropy and spread
	posterior->process();
	posterior->calculate_entropy_and_spread();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();

	printf("Max a posteriori: %f, %f\n", plume_model->max_a_post_value[0], plume_model->max_a_post_value[1]);
	
	
	bool is_converge_condition_reached = check_converge_condition();
	bool is_path_end = path_planner->reach_end_of_fix_path; // only for fixed path like lawnmower
	if (is_converge_condition_reached || is_path_end || FSM.n_cycles == MAX_ITER)
	{
		// FSM.state = STATE_FINISHED;
		// return;
		FSM.state = STATE_FINISHED;
		printf("Robot ID [%d]: STATE_FINISHED\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: STATE_FINISHED\n", ID);
		return;
	
	} else {
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;

	}
}

void state_updategoal()
{
	// computation time of path planning
	long long time_before_path_planning = current_timestamp();
	printf("Robot ID [%d]: STATE_UPDATEGOAL \n", ID);
	fprintf(console_logfile, "FSM iter: %d \n", FSM.n_cycles);
	if (VERBOSE)
		printf("[%d] updating the goal\n", ID);
	switch (simulation_parameters.planningStrategy)
	{
	case GREEDY:
		path_planner->maximal_belief_planning();
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

	if (path_planner->reach_end_of_fix_path){
		printf("Robot ID [%d]: reached the end of fixed path\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: reached the end of fixed path\n", ID);
		FSM.state = STATE_FINISHED;
		return;
	}

	if (FSM.state != STATE_FINISHED)
	{
		// If the goal position is very close to current postion, swith to random planning
		// if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<double>(simulation_parameters.sampling_dist_thresh)))
		// {
		// 	printf("The goal position is very close to current postion, switch to random planning \n");
		// 	fprintf(console_logfile, "The goal position is very close to current postion, switch to random planning \n");
		// 	random_planning();
		// }
		path.clear();
		if (simulation_parameters.workingMode == VIDEO){
			// use the goal position from the file instead
			read_goal_position();
			if (FSM.state == STATE_FINISHED){
				return;
			}
		}
		if (use_visibility_path){
			printf("goal position : %f %f \n", goal_position.x, goal_position.y);
			path_planner->goal_outside_obstacles(&goal_position);
			printf("from %f %f to goal %f %f \n", current_position.x, current_position.y, goal_position.x, goal_position.y);

			// find the path using visibility graph
			VisiLibity::Point start(current_position.x, current_position.y);
			VisiLibity::Point finish(goal_position.x, goal_position.y);
			VisiLibity::Polyline Polyline_path = my_environment->shortest_path(start, finish, *vg, epsilon);
			// copy the path in another variable
			// skip the first point (current position)
			for (unsigned k = 1; k < Polyline_path.size(); k++)
			{
				Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
				path.push_back(p);
				log_trajectory(p);
			}
		
		// i = path.begin();
		// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		// send the goal to the supervisor
		}

		// mark the goal position as visited instead of the actually reach position, to avoid the problem of reselecting the same goal
		int idx = position_to_index_2D(goal_position.x, goal_position.y);
		cell_visited_list[idx] = 1;
            printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, goal_position.x, goal_position.y);
		

		if (simulation_parameters.workingMode == EXPERIMENT || simulation_parameters.workingMode == VIDEO){
			printf("Sending waypoints to robot ID [%d]...\n", ID);
			ros->send_waypoints(path);
			// update goal
			FSM.state = STATE_LISTENING;
			FSM.hook = state_listening;
			time_before_moving = current_timestamp();
			long long current_path_planning_time = current_timestamp() - time_before_path_planning;
			planning_time_sum += current_path_planning_time;

		}
		if (simulation_parameters.workingMode == OFFLINE){
			FSM.state = STATE_READ_ALL;
			FSM.hook  = state_read_goal_measurement;
		}

	}
}



void state_read_goal_measurement(){
	printf("Enter state read goal and measurment\n");
    int d, FSM_iter;
	float f1, f2, f3, f4, f5, f6, f7, f8, f9, f10; 
    double t;
	char line[256];
    int counter = 0;

    // Read the goal position
	if (fscanf(experiment_goalposition_logfile, "%d, %f, %f, %f, %lf", &d, &f1, &f2, &f3, &t) == 5){
		printf("Read goal position from file \n");
		goal_position.x = f1;
		goal_position.y = f2;
		goal_position.heading = 0;
		printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);
        float average_x = 0;
		float average_y = 0;
		float average_heading = 0;
		float average_gas = 0;
		float average_wind_u = 0;
		float average_wind_v = 0;

		while (current_position.dist_to_goal(goal_position) > 0.05){
			// keep 1 out of 2 measurements to reduce the data size
			fscanf(experiment_measurements_logfile, "%d, %f, %f, %d, %f, %f", &d, &f4, &f5, &FSM_iter, &f9, &f10);
			fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f6, &f7, &f8);
			// use the average of sampling_period measurements
            average_gas += f4;
			average_wind_u += f9;
			average_wind_v += f10;
			average_x += f6;
			average_y += f7;
			average_heading += f8;
			if (counter % simulation_parameters.sampling_period == 0){
				if (counter != 0){
					// to avoid adding the first measurement when reaching the goal position
					average_gas /= simulation_parameters.sampling_period;
					average_wind_u /= simulation_parameters.sampling_period;
					average_wind_v /= simulation_parameters.sampling_period;
					average_x /= simulation_parameters.sampling_period;
					average_y /= simulation_parameters.sampling_period;
					average_heading /= simulation_parameters.sampling_period;

				}
				
				
				history.size ++;
				history.gas_meas.push_back(average_gas);
				history.wind_meas_u.push_back(average_wind_u);
				history.wind_meas_v.push_back(average_wind_v);
				history.FSM_iter.push_back(FSM_iter);				
				history.position.push_back({average_x, average_y, average_heading});
				average_gas = 0;
				average_wind_u = 0;
				average_wind_v = 0;
				average_x = 0;
				average_y = 0;
				average_heading = 0;
			}
			current_position.x = f6;
			current_position.y = f7;
			current_position.heading = f8;
			counter ++;
		}


		// Move to estimating
		history_print(history);
		log_position();
		log_trajectory(current_position);
		FSM.state = STATE_ESTIMATING;
		FSM.hook = state_estimating;

	} else{
		printf("End of experiment measurements file\n");
		FSM.state = STATE_FINISHED;
		//close the files
		fclose(experiment_goalposition_logfile);
		fclose(experiment_measurements_logfile);
		fclose(experiment_position_logfile);

	}
}

void read_goal_position(){
	printf("Enter state read goals\n");
    int d;
	float f1, f2, f3; 
    double t;
	char line[256];

    // Read the goal position
	if (fscanf(experiment_goalposition_logfile, "%d, %f, %f, %f, %lf", &d, &f1, &f2, &f3, &t) == 5){
		goal_position.x = f1;
		goal_position.y = f2;
		goal_position.heading = 0;
		printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);
	} else{
		printf("End of experiment measurements file\n");
		//close the files
		FSM.state = STATE_FINISHED;
		fclose(experiment_goalposition_logfile);
	}
}



bool check_converge_condition()
{
	bool is_converge_condition_reached = false;
	if (simulation_parameters.stopCondition == ENTROPY_ONLY){
		if (posterior->entropy < 0.1 ){
			is_converge_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == SPREAD_ONLY){
		if ((posterior->x_spatial_spread < 2 && posterior->y_spatial_spread < 2)){
			is_converge_condition_reached = true;
		}
	} else if (simulation_parameters.stopCondition == ENTROPY_SPREAD){
		if (FSM.n_cycles == MAX_ITER || (posterior->entropy < 0.3 && posterior->x_spatial_spread < 2 && posterior->y_spatial_spread < 2)){
			is_converge_condition_reached = true;
		}
	}
	printf("entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior->entropy, posterior->x_spatial_spread, posterior->y_spatial_spread);
	fprintf(console_logfile, "entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior->entropy, posterior->x_spatial_spread, posterior->y_spatial_spread);
	if (is_converge_condition_reached)
	{
		// log the finish time stamp
		end_timestamp = current_timestamp();
		printf("Robot ID [%d]: Stop condition reached!\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: Stop condition reached!\n", ID);
	}
	return is_converge_condition_reached;
}

PlumeModel* initializePlumeModel() {
    if (simulation_parameters.plumeModel == DDPM) {
		printf("Initializing DeepCFD plume model...\n");
        return new DeepCFDPlumeModel2D();
    } else if (simulation_parameters.plumeModel == FDM_PDE) {
		printf("Initializing FDM_PDE plume model...\n");
        return new AdeFDMPlumeModel2D();
    } else if (simulation_parameters.plumeModel == GAUSSIAN_PLUME){
        return new GaussianPlumeModel2D();
    } else {
        printf("Unknown plume model. Exit.\n");
        exit(1);
    }
}

void check_visibility_environment() {
	// visibility graph (visilibity library)
	std::cout << "Validating environment model . . . ";
	if (my_environment->is_valid(epsilon))
		std::cout << "OK" << std::endl;
	else {
		std::cout << std::endl
			    << "Warning:  Environment model "
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
}


void set_source_bounds(SourceBounds *bounds,
                        double x_min, double y_min,
                        double x_max, double y_max,
                        int x_nbins, int y_nbins)
{
    bounds->x_min = x_min;
    bounds->y_min = y_min;
    bounds->x_max = x_max;
    bounds->y_max = y_max;
    bounds->x_nbins = x_nbins;
    bounds->y_nbins = y_nbins;

	bounds->map_size = x_nbins * y_nbins;

    // Compute resolutions safely (avoid division by zero)
    bounds->x_resol = (x_nbins > 0) ? (x_max - x_min) / x_nbins : 0.0;
    bounds->y_resol = (y_nbins > 0) ? (y_max - y_min) / y_nbins : 0.0;
}

void init_estimation_bound(){
	// in case of global estimation
		set_source_bounds(&source_bounds, 
						XMIN, YMIN, XMAX, YMAX, 20, 20);
		FILE * sim_param_file = fopen(config_logfile_name, "a");
	    fprintf(sim_param_file, "nbins %d %d %d %d %d\n", Q_nbins, source_bounds.x_nbins, source_bounds.y_nbins, a_y_nbins, b_y_nbins);
	    fprintf(sim_param_file, "min_maxs %f %f %f %f %f %f %f %f %f %f\n", Q_min, Q_max, source_bounds.x_min, source_bounds.x_max, source_bounds.y_min, source_bounds.y_max, a_y_min, a_y_max, b_y_min, b_y_max);
	    fclose(sim_param_file);
}


void init_visited_list(){
	cell_visited_list.assign(source_bounds.map_size, 0.0);
}


// =========================== MAIN ============================== //

int main(int argc, char **argv)
{
	printf("Starting STE Node... \n");
	rclcpp::init(argc, argv);
	ros = new RosBridge();

	init();
	read_obstacle_list();
    int m = simulation_parameters.map_nbr;
	std::string environment_file = config_folder_path + "/model/test_maps/map" + std::to_string(m) + "/test_map" + std::to_string(m) + ".environment";
    my_environment = new VisiLibity::Environment(environment_file);
    vg = new VisiLibity::Visibility_Graph(*my_environment, epsilon);
    check_visibility_environment();
    
	// init the estimation bound depending on global or local estimation
	init_estimation_bound();
    // if using visited list, initialize the visited list
	if (simulation_parameters.visitedList == visitedList_ON){
		init_visited_list();
	}
    // File for saving console messages
	console_logfile = fopen(console_logfile_name, "a");
	// Initialize the appropriate plume model based on the simulation parameters
	plume_model = initializePlumeModel();
	posterior = new Posterior(plume_model);
	path_planner = new PathPlanner(plume_model, posterior, &history);

	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	plume_model->calculate_gas_map_and_information_matrix();
	plume_model->log_gas_concentration_map(gas_concentration_map_logfile_name, 'a');

	auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  	exec->add_node(ros->node());
    
	// //main loop
	while (FSM.state != STATE_FINISHED && rclcpp::ok())
	{
		exec->spin_some(); 
		
		// update_position();
	
		// change state
		FSM.hook();
		// next
		next_iteration_time();
		// simple sleep to keep timing reasonable
  		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		n_time_step++;
	}

	fclose(console_logfile);

	log_history();
	log_experiment_time();
	log_visited_list();

	// free the allocated memory to obstacle array
	for (int i = 0; i < nbr_obs; i++)
		delete[] obstacles[i];
	delete[] obstacles;
    
	if (simulation_parameters.workingMode == EXPERIMENT || simulation_parameters.workingMode == VIDEO){
		printf("STE Node finished. Returning to home position... \n");
		float end_x = XMAX-1.0;
		float end_y = YMIN+1.0;
		std::list<Position> end_goal;
		// find the path using visibility graph
		VisiLibity::Point start(current_position.x, current_position.y);
		VisiLibity::Point finish(end_x, end_y);
		VisiLibity::Polyline Polyline_path = my_environment->shortest_path(start, finish, *vg, epsilon);
		// copy the path in another variable
		// skip the first point (current position)
		for (unsigned k = 1; k < Polyline_path.size(); k++)
		{
			Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
			end_goal.push_back(p);
		
		}

		ros->send_waypoints(end_goal);
		printf("STE Node finished. Homing... \n");
	}

	rclcpp::shutdown();

	exit(0);
}