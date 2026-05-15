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
#include "controller_STE_multi-robot.h"
// #include "nvwa/debug_new.h"
#include "Posterior.h"
#include "Message.h"
#include "webots_functions.h"
#include "path_planner.h"
#include "region_allocator.h"
// #include "aStar.h"
#include "visilibity.hpp"
// #include <webots/supervisor.h>
#include "constant_and_helper.h"
#include <unistd.h>

#define MSG_STATE_SHARE_PDF -99
#define MSG_STATE_SHARE_DIST -98


using namespace std;

// Global Variables

// log files
char folder_name[256];
FILE *goalposition_logfile;
FILE *measurements_logfile;
FILE *expected_a_posteriori_logfile;
FILE *likelihood_logfile;
FILE *measurement_history_logfile;
FILE *position_history_logfile;
FILE *whole_position_history_logfile; //new: for MATLAB Distributed clustering visualization
FILE *time_logfile;
FILE *max_a_posteriori_logfile;
FILE *posterior_pdf_3D;
FILE *posterior_pdf_3D_cluster; //new: for MATLAB Distributed clustering visualization
FILE *cluster_sizes_geometric;
FILE *cluster_sizes_informative;
FILE *entropy_logfile;
FILE *entropy_individual_logfile;
FILE *config_logfile;
FILE *trajectory_logfile;
FILE *gas_concentration_map_logfile;
FILE *experiment_time_logfile;
FILE *visited_list_logfile;

// text file for storing some prints for debugging
// debugging purpose
FILE *console_logfile;
FILE *clustering_debugging_logfile; //new: for debugging purpose (Distributed architecture)
FILE *cumulative_path_logfile;


char goalposition_logfile_name[512];
char measurements_logfile_name[512];
char expected_a_posteriori_logfile_name[512];
char max_a_posteriori_logfile_name[512];
char likelihood_logfile_name[512];
char measurement_history_logfile_name[512];
char position_history_logfile_name[512];
char whole_position_history_logfile_name[512];
char time_logfile_name[512];
char posterior_pdf_3D_name[512];
char posterior_pdf_pre_merge_name[512];
char posterior_pdf_3D_cluster_name[512];
char cluster_sizes_geometric_name[512];
char cluster_sizes_informative_name[512];
char entropy_logfile_name[512];
char entropy_individual_logfile_name[512];
char config_logfile_name[512];
char trajectory_logfile_name[512];
char gas_concentration_map_logfile_name[512];
char experiment_time_logfile_name[512];
char visited_list_logfile_name[512];
char clustering_bounding_boxes_logfile_name[512];
char gas_information_map_logfile_name[512];
char scores_map_logfile_name[512];
char distance_map_logfile_name[512];
char acc_info_map_logfile_name[512];

// text file for storing some prints for debugging
char console_logfile_name[512];
char clustering_debugging_name[512];
char cumulative_path_logfile_name[512];

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
struct history history;
struct history sample_in_motion;

int sample_counter = 0;
// plume model
DeepCFDPlumeModel2D plume_model;

int cell_visited_list[MAP_SIZE] = {0};

// current posterior distribution
Posterior posterior(&plume_model);

// path planner
PathPlanner path_planner(&plume_model, &posterior, &history);

// robot's ID
int ID;
int self_index;

// message structure
struct message message;

// list of other robots
struct friends friends[MAX_NB_ROBOTS];

SimulationParameters simulation_parameters;

double previous_sensor_reading = 0.0;

// gas map
// param: sigma, gamma, scaling_param, wind_speed, rco
GasMap2D gas_map(0.2, 4, 1, 0.75, 0.3);

// obstacles definitions
int nbr_obs = 4;
float margin = 0.16;
float **obstacles = NULL;
std::list<Position> path;
std::list<Position>::iterator i;

std::string environment_file("../../data/test_map.environment");
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

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

bool use_visibility_path = true;

// store the belief maps of the whole team
static double belief_maps[MAX_NB_ROBOTS][MAP_SIZE] = {0};
static int received_flags[MAX_NB_ROBOTS] = {0};
static double belief_map_merged[MAP_SIZE] = {0};
static int cluster_informative_pdf[x_nbins*y_nbins] = {0};
static float received_current_pos[MAX_NB_ROBOTS][2] = {0};
static double received_current_dist[MAX_NB_ROBOTS] = {0};
static int ready_from[MAX_NB_ROBOTS] = {0};
static int i_am_ready = 0;

// region allocator
RegionAllocator region_allocator(&posterior, received_current_pos);


// print variables
bool printed = false;
bool entered_state_receive_pdf = false;
bool entered_state_receive_dist = false;
bool entered_state_moving = false;

// preallocate obstacle map
std::vector<int> obstacle_map(x_nbins * y_nbins, 0);  // GLOBAL


// =========================== FUNCTIONS ================================ //


// random value sampled from a normal distribution
double randn(double mu, double sigma)
{
	// taken from https://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c/
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;

	if (call == 1)
	{
		call = !call;
		return (mu + sigma * (double)X2);
	}

	do
	{
		U1 = -1 + ((double)rand() / RAND_MAX) * 2;
		U2 = -1 + ((double)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	mult = sqrt((-2 * log(W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

	call = !call;

	return (mu + sigma * (double)X1);
}

// random value sampled from a uniform distribution
double randu()
{
	// random variable between 0 and 1
	return (double)rand() / (double)RAND_MAX;
}

// get the current time in timestamp (ms)
long long current_timestamp()
{
	struct timeval te;
	gettimeofday(&te, NULL);						     // get current time
	long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

// Euclidean distance between two points
double pt_distance(Position p1, Position p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
// min distance between two robots position (p1 and p2) and their goals (g1 and g2)
double min_distance(Position p1, Position g1, Position p2, Position g2)
{
	double d_original = pt_distance(p1, g1) + pt_distance(p2, g2);
	double d_swap = pt_distance(p1, g2) + pt_distance(p2, g1);
	if (d_swap < d_original)
		return d_swap;
	else
		return d_original;
}

// Euclidean distance between current position to the goal position
double dist_to_goal(Position goal)
{
	double dx, dy;
	dx = goal.x - current_position.x;
	dy = goal.y - current_position.y;
	return sqrt(dx * dx + dy * dy);
}

// calculate wheel speeds to go toward goal position
void go_to_goal(Position goal)
{
	// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, goal.x,goal.y);
	// vector to the goal
	double movement[2];
	movement[0] = (goal.x - current_position.x);
	movement[1] = (goal.y - current_position.y);
	// printf("movement : %f %f \n", movement[0], movement[1]);

	// vector to the goal in robot coordinates
	float x = movement[0] * cosf(current_position.heading) + movement[1] * sinf(current_position.heading);  // x in robot coordinates
	float z = -movement[0] * sinf(current_position.heading) + movement[1] * cosf(current_position.heading); // z in robot coordinates
	float bearing = atan2(z, x) - PI / 2;
	// printf("bearing : %f \n", bearing/PI*180);

	double k_rho = 0.2;		// gain for linear velocity
	double k_theta = k_rho * 5.0; // gain for angular velocity; 5.0 is the ratio between the two gains, where is a good value to reach the goal
	// angular and linear velocity
	// the maximal speed of Khepera is 0.8m/s, so that the maximal velocity factor is 4
	float w = simulation_parameters.vel_factor * k_theta * sinf(bearing);

	// ORIGINAL CONTROLLER
	float u = simulation_parameters.vel_factor * k_rho * cosf(bearing); // default vel factor is 1

	// LINEAR VELOCITY CONTROLLER
	// int dir = (cosf(bearing)>0)-(cosf(bearing)<0);
	// float u = simulation_parameters.vel_factor * dir * (abs(bearing)<=0.2) + (abs(bearing)>0.2) * 0.2 * cosf(bearing); //simulation_parameters.vel_factor * 0.2 * cosf(bearing);

	// printf("linear v : %f; angular w : %f \n", u, w);
	//  wheels speed
	speed[0] = (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
	speed[1] = (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
	// printf("speed : %f %f \n", speed[0], speed[1]);
}

// show the measurement history on screen
void history_print(struct history history)
{
	// printf("_____ Sample History _____ \n");
	// for (int i = 0; i < history.size; i++)
	// {
	// 	printf("%d : %f @ (%f, %f)\n", i, history.measurement[i], history.position[i].x, history.position[i].y);
	// }
	// printf("__________________________ \n");
}

void read_obstacle_list()
{
	// open file
	std::ifstream infile("../../data/obstacle_list.txt");
	// first line is the number of obstacles
	infile >> nbr_obs;
	printf("There is %d obsatcle in total:\n", nbr_obs);
	// allocate memory for obstacle list array
	obstacles = new float *[nbr_obs];
	// read all obstacles now
	for (int i = 0; i < nbr_obs; i++)
	{
		obstacles[i] = new float[4];
		infile >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3];
		obstacles[i][0] -= margin;
		obstacles[i][1] -= margin;
		obstacles[i][2] += margin;
		obstacles[i][3] += margin;
	}

	for (int i = 0; i < nbr_obs; i++)
	{
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}

	float dx = (XMAX - XMIN) / x_nbins;
	float dy = (YMAX - YMIN) / y_nbins;

	for (int i = 0; i < nbr_obs; ++i) {
		float x1, y1, x2, y2;
		infile >> x1 >> y1 >> x2 >> y2;

		int col_start = std::max(0, static_cast<int>(std::floor((x1 - XMIN) / dx)));
		int col_end   = std::min(x_nbins - 1, static_cast<int>(std::floor((x2 - XMIN) / dx)));
		int row_start = std::max(0, static_cast<int>(std::floor((y1 - YMIN) / dy)));
		int row_end   = std::min(y_nbins - 1, static_cast<int>(std::floor((y2 - YMIN) / dy)));

		for (int row = row_start; row <= row_end; ++row) {
			for (int col = col_start; col <= col_end; ++col) {
				int idx = row * x_nbins + col;
				obstacle_map[idx] = 1;
			}
		}

	}
}


// ===================== LOG FILES FUNCTIONS ========================== //
void log_position()
{
	double time = wb_robot_get_time();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile, "%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements()
{
	// int i = 0;
	double time = wb_robot_get_time();
	measurements_logfile = fopen(measurements_logfile_name, "a");
	fprintf(measurements_logfile, "%d, %f, %f, %f\n", FSM.n_cycles, sampleBuffer.average_wind_intensity, sampleBuffer.average_concentration, time);
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
			fprintf(measurement_history_logfile, "%d, %f, %d\n", i, history.measurement[i], history.FSM_iter[i]);
		}
		if (simulation_parameters.gasFeature == RELATIVE_VALUE)
		{
			fprintf(measurement_history_logfile, "%d, %f, %f, %d\n", i, history.measurement[i], history.relative_meas[i], history.FSM_iter[i]);
		}
		if (simulation_parameters.gasFeature == GASHIT)
		{
			fprintf(measurement_history_logfile, "%d, %f, %d, %d\n", i, history.measurement[i], history.gas_hit_event[i], history.FSM_iter[i]);
		}
	}

	fclose(measurement_history_logfile);
}

void log_whole_position_history()
{
    whole_position_history_logfile = fopen(whole_position_history_logfile_name, "a"); // append mode, to add new rows without removing the previous ones

    fprintf(whole_position_history_logfile, "%f,%f,", current_position.x, current_position.y);

    fprintf(whole_position_history_logfile, "\n");
    fclose(whole_position_history_logfile);
}
void log_visited_list()
{
	visited_list_logfile = fopen(visited_list_logfile_name, "a");

	for (int i = 0; i < MAP_SIZE; i++)
		fprintf(visited_list_logfile, "%d, ", cell_visited_list[i]);

	fprintf(visited_list_logfile, "\n");

	fclose(visited_list_logfile);
}

void log_trajectory(Position p)
{
	double time = wb_robot_get_time();
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
void state_waiting()
{
	// initial waiting for the plume to get established
	static int counter = 0;
	counter++;

	if (counter > 300)
	{
		FSM.state = STATE_MEASURING;
		FSM.hook = state_measuring;
		// log start timestamp of the first measuring state (static variable)
		start_timestamp = wb_robot_get_time();
		printf("start timestamp: %f\n", start_timestamp);
		time_before_measuring = start_timestamp;
	}
}

void state_read_initial_position_measurement()
{
	printf("Enter state read initial position\n");
	int d, iter;
	float f1, f2, f3, f4, f5, f6, f7, f8;
	double t;
	char line[256];
	start_timestamp = wb_robot_get_time();

	// Read the goal position
	if (!feof(experiment_goalposition_logfile))
	{
		fscanf(experiment_goalposition_logfile, "%d, %f, %f, %f, %lf", &d, &f1, &f2, &f3, &t);
		goal_position.x = f1;
		goal_position.y = f2;
		goal_position.heading = 0;
		printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);
		fscanf(experiment_measurements_logfile, "%d, %f, %f, %d", &d, &f4, &f5, &iter);
		history.size++;
		history.measurement.push_back(f4);
		fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f6, &f7, &f8);
		history.position.push_back({f6, f7, f8});
		current_position.x = f6;
		current_position.y = f7;
		current_position.heading = f8;
		printf("measurement @ position (%f, %f, %f) is %f\n", current_position.x, current_position.y, current_position.heading, f4);
	}
	sampleBuffer.average_wind_intensity = 1;
	FSM.state = STATE_ESTIMATING;
	FSM.hook = state_estimating;
}

void state_read_positions_measurements()
{
	printf("Enter state read goal and measurment\n");
	int d, iter;
	float f1, f2, f3, f4, f5, f6, f7, f8;
	double t;
	char line[256];

	// Read the goal position
	if (!feof(experiment_goalposition_logfile))
	{
		fscanf(experiment_goalposition_logfile, "%d, %f, %f, %f, %lf", &d, &f1, &f2, &f3, &t);
		goal_position.x = f1;
		goal_position.y = f2;
		goal_position.heading = 0;
		int pos_coord[2];
		position_to_indices_2D(goal_position.x, goal_position.y, &pos_coord[0], &pos_coord[1]);
		int idx = posterior.indices_to_index(pos_coord);
		cell_visited_list[idx] = 1;
		// printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);

		while (dist_to_goal(goal_position) > 0.05)
		{
			fscanf(experiment_measurements_logfile, "%d, %f, %f, %d", &d, &f4, &f5, &iter);
			history.size++;
			history.measurement.push_back(f4);
			fscanf(experiment_position_logfile, "%d, %f, %f, %f", &d, &f6, &f7, &f8);
			history.position.push_back({f6, f7, f8});
			current_position.x = f6;
			current_position.y = f7;
			current_position.heading = f8;
			// printf("measurement @ position (%f, %f, %f) is %f\n", current_position.x, current_position.y, current_position.heading,f4);
		}
	}
	sampleBuffer.average_wind_intensity = 1;
	FSM.state = STATE_ESTIMATING;
	FSM.hook = state_estimating;
}

void state_measuring()
{

	printf("Robot ID [%d]: STATE_MEASURING \n", ID);
	// printf("measuring sample number %d\n", measurements.n_samples);
	vector<double> sample_from_map;
	// if robot is sensing while moving, saving the measurement in motion
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
	// if the sampling mode is stop and go, then sample for several seconds and take the average value
	// Only the average value is used for the estimation
	if (simulation_parameters.samplingMode == STOP_AND_GO_SAMPLING)
	{
		// Sample for several seconds and take the average value
		if (sampleBuffer.n_samples < ODOR_FILTER_LENGTH)
		{
			// continue measuring odor concentration
			double odor_concentration = 0.0;
			odor_read(&sampleBuffer.instant_concentration, odor_concentration);
			// measuring wind
			wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
			// printf("instant concentration: %f, wind intensity: %f\n", sampleBuffer.instant_concentration, sampleBuffer.instant_wind_intensity);
			// add samples to buffers
			sampleBuffer.add_all_to_buffer();
			// printf("n %d wind intensity: %f, angle %f \n", sampleBuffer.n_samples, sampleBuffer.instant_wind_intensity, sampleBuffer.instant_wind_angle);
			return;
		}
		else
		{
			// average all samples
			sampleBuffer.average_all();
		}
	}
	if (simulation_parameters.samplingMode == SENSE_WITHOUT_STOP || simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
	{
		// continue measuring odor concentration
		double odor_concentration = 0.0;
		odor_read(&sampleBuffer.instant_concentration, odor_concentration);
		// measuring wind
		wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
		// printf("instant concentration: %f, wind intensity: %f\n", sampleBuffer.instant_concentration, sampleBuffer.instant_wind_intensity);
		// add samples to buffers
		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
	}

	history.position.push_back({current_position.x, current_position.y, 0});
	history.measurement.push_back(sampleBuffer.average_concentration);
	history.FSM_iter.push_back(FSM.n_cycles);
	history.size++;
	history_print(history);

	// mark the current cell as visited
	if (simulation_parameters.visitedList == visitedList_ON)
	{
		int pos_coord[2];
		position_to_indices_2D(current_position.x, current_position.y, &pos_coord[0], &pos_coord[1]);
		int idx = posterior.indices_to_index(pos_coord);
		cell_visited_list[idx] = 1;
	}
	// log and save
	if (VERBOSE)
		printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
	//log_position();
	log_measurements();


	// // send it to friends
	// if (simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR)
	// {
	// 	Message msg(ID, 1, history.position[(history.size) - 1], sampleBuffer.average_concentration, goal_position, 0);
	// 	send_msg(msg);
	// }

	// go to estimation
	FSM.state = STATE_ESTIMATING;
	FSM.hook = state_estimating;
	time_after_measuring = wb_robot_get_time();
	navigation_sampling_time_sum += time_after_measuring - time_before_measuring;
	printf("navigation sampling time: %f\n", navigation_sampling_time_sum);

}

void state_estimating()
{
	printf("Robot ID [%d]: STATE_ESTIMATING \n", ID);
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
		//printf("going for exhaustive\n");
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
	posterior.log_pdf(posterior_pdf_pre_merge_name, 'a');

	if (simulation_parameters.mapCoverageStrategy != NONE)
	{
		FSM.n_cycles++;
	}

	fprintf(console_logfile, "FSM iter: %d \n", FSM.n_cycles);

	// debugging purpose
	fprintf(clustering_debugging_logfile, "Robot ID [%d]: FSM iter number %d \n", ID, FSM.n_cycles);

	// debugging purpose
	fprintf(clustering_debugging_logfile, "Individual original pdf: \n");
	for (int i = 0; i < MAP_SIZE; i++) {
		fprintf(clustering_debugging_logfile, "%.8f ", posterior.pdf[i]);
	}
	fprintf(clustering_debugging_logfile, "\n");


	if (VERBOSE)
		printf("[%d] cycle number %d\n", ID, FSM.n_cycles);

	long long current_estimation_time = current_timestamp() - time_before_estimating;
	// printf("Current estimation time is : %lld\n", current_estimation_time);

	estimating_time_sum += current_estimation_time;
	// printf("estimating_time_sum : %lld\n", estimating_time_sum);

	// check if stopping condition is reached
	if ((simulation_parameters.number_of_robots == 1)||(simulation_parameters.controlArchitecture == INDIVIDUAL))
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
	else if (simulation_parameters.controlArchitecture == DISTRIBUTED)
	{
		FSM.state = STATE_SHARE_PDF;
		FSM.hook = state_share_pdf;
	}
}

void state_updategoal()
{
	if (simulation_parameters.mapCoverageStrategy == NONE && simulation_parameters.number_of_robots > 1)
	{
		log_position();
		log_whole_position_history();
		posterior.log_pdf(posterior_pdf_3D_name, 'a');

		FSM.n_cycles++;
	}
	
	double sum = 0.0;
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        sum += posterior.pdf[i];
    }
	if (sum < 0.1)
	{
		goal_position.x = current_position.x;
		goal_position.y = current_position.y;

		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
	}
	else{
		// computation time of path planning
		long long time_before_path_planning = current_timestamp();
		printf("Robot ID [%d]: STATE_UPDATEGOAL \n", ID);
		if (VERBOSE)
			printf("[%d] updating the goal\n", ID);
		switch (simulation_parameters.planningStrategy)
		{
		case GREEDY:
			path_planner.maximal_belief_planning();
			break;
		case INFO_CENTER:
			path_planner.information_center_planning();
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
			path=path_planner.accumulative_Dijkstra();
			use_visibility_path = false;
			break;
		case SUM_DIJKSTRA_ADP:
			path=path_planner.accumulative_Dijkstra();
			use_visibility_path = false;
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
				for (unsigned k = 0; k < Polyline_path.size(); k++)
				{
					Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
					path.push_back(p);
					log_trajectory(p);
				}
			}
			i = path.begin();

			Message msg(ID, 0, goal_position, -1, goal_position, 0);
			send_msg(msg);
		
			if (simulation_parameters.simuMode == ONLINE)
			{
				// update goal
				FSM.state = STATE_MOVING;
				FSM.hook = state_moving;

				time_before_moving = wb_robot_get_time();
				long long current_path_planning_time = current_timestamp() - time_before_path_planning;
				planning_time_sum += current_path_planning_time;
			}
			else
			{
				FSM.state = STATE_READ_ALL;
				FSM.hook = state_read_positions_measurements;
			}
		}
	}

}

void state_moving()
{

	if (!entered_state_moving)
	{
		printf("Robot ID [%d]: STATE_MOVING \n", ID);
		entered_state_moving = true;
	}
	// printf("moving toward the goal\n");
	if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
	{
		sample_counter++;
		// sample and run kernel
		double sensor_reading = 0.0;
		double odor_concentration = 0.0;
		odor_read(&sensor_reading, odor_concentration);
		if (simulation_parameters.processingMode == DECONVOLUTION || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			reverse_sensor_model(sensor_reading, previous_sensor_reading);
		}
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			gas_map.runKernelAlgorithm(current_position, sensor_reading);
		}
		if (sample_counter > simulation_parameters.sampling_period)
		{
			// Sampling distance threshold
			// float dist_to_last_measurement = sqrt(pow(current_position.x - last_measurement_pos.x, 2) + pow(current_position.y - last_measurement_pos.y, 2));
			// if ((sample_in_motion.size == 0) || dist_to_last_measurement > simulation_parameters.sampling_dist_thresh)
			//{
			sample_in_motion.position.push_back({current_position.x, current_position.y, 0});
			sample_in_motion.measurement.push_back(sensor_reading);
			sample_in_motion.size++;
			last_measurement_pos.x = current_position.x;
			last_measurement_pos.y = current_position.y;
			sample_counter = 0;
		}
	}
	// verify if arrived to the goal if applicable
	// fprintf(console_logfile, "check dist to end goal \n");
	// fprintf(console_logfile,"my position (%f, %f) - my end goal (%f, %f)\n", current_position.x, current_position.y, goal_position.x, goal_position.y);
	double dist_threshold = 0.1;
	if (dist_to_goal(goal_position) > dist_threshold)
	{
		if (i == path.end())
		{
			fprintf(console_logfile, "path end goal reached \n");
		}

		// move
		if (dist_to_goal(*i) <= dist_threshold && i != path.end())
		{
			fprintf(console_logfile, "my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
			i++;
			// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		}
		go_to_goal(*i);
		// obstacle avoidance if necessary
		if (obstacle_or_not() == 1)
		{
			FSM.state = STATE_OBSTACLE_AVOIDANCE;
			FSM.hook = state_obstacle_avoidance;
			return;
		}
		set_wheels_speed(speed[0], speed[1]);

		// save the current position in logfile
		log_whole_position_history();
		

	}
	else 
	{
		set_wheels_speed(0, 0);
		sampleBuffer.reset_all();
		path.clear();
		// back to measuring

		FSM.state = STATE_MEASURING;
		FSM.hook = state_measuring;
		entered_state_moving = false;
	}

}


void state_obstacle_avoidance()
{
	// printf("avoiding obstacle\n");
	obstacle_avoidance_k4();
}

void state_merge_pdf()
{
	printf("Robot ID [%d]: STATE_MERGE_PDF \n", ID);

	// Original merge without weights
	for (int i = 0; i < MAP_SIZE; i++)
	{
		belief_map_merged[i] = 1.0;
	}

	for (int r = 0; r < simulation_parameters.number_of_robots; r++)
	{
		for (int i = 0; i < MAP_SIZE; i++)
		{
			belief_map_merged[i] *= belief_maps[r][i];
		}
	}

	fprintf(clustering_debugging_logfile, "NO Normalized Full Merged BM: \n");
	for (int i = 0; i < MAP_SIZE; i++) {
		fprintf(clustering_debugging_logfile, "%.8f ", belief_map_merged[i]);
	}
	fprintf(clustering_debugging_logfile, "\n");

	// normalize belief map
	double sum = 0.0;
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

	// debugging purpose
	//fprintf(clustering_debugging_logfile, "FSM iter: %d \n", FSM.n_cycles);
	fprintf(clustering_debugging_logfile, "Normalized Full Merged BM: \n");
	for (int i = 0; i < MAP_SIZE; i++) {
		fprintf(clustering_debugging_logfile, "%.8f ", belief_map_merged[i]);
	}
	fprintf(clustering_debugging_logfile, "\n");


	// printf("Robot ID [%d]: Belief map merged successfully.\n", ID);
	printf("Robot ID [%d]: Merged belief map is: \n", ID);
	for (int i = 0; i < MAP_SIZE; i++)
	{
		printf("%f ", belief_map_merged[i]);
	}
	printf("\n");

	memcpy(posterior.pdf, belief_map_merged, MAP_SIZE * sizeof(double));


      // Check the stopping condition after merging
	posterior.process();
	posterior.calculate_entropy_and_spread();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();


	// Check if Merged Belief Map equal 0
	double sum_merged_pdf = 0.0;
	for (int i = 0; i < x_nbins * y_nbins; ++i) {
		sum_merged_pdf += posterior.pdf[i];
	}

	bool is_stop_condition_reached = check_stop_condition();
	if (is_stop_condition_reached)
	{
		// Avoid STATE_FINISHED if Merged Belief Map equal 0
		if (sum_merged_pdf > 0)
		{
			log_position();
			log_whole_position_history();
			posterior.log_pdf(posterior_pdf_3D_name, 'a');
			broadcast_state_finished();

			FSM.state = STATE_FINISHED;
			printf("Robot ID [%d]: STATE_FINISHED\n", ID);
			fprintf(console_logfile, "Robot ID [%d]: STATE_FINISHED\n", ID);
			return;
		}
		else
		{
			printf("Robot ID [%d]: Avoided STATE_FINISHED because Merged Belief Map = 0\n", ID);
			fprintf(clustering_debugging_logfile, "Robot ID [%d]: Avoided STATE_FINISHED because Merged Belief Map = 0\n", ID);
			FSM.state = STATE_MEASURING;
			FSM.hook = state_measuring;
			return;
		}
	}
	else
	{
		if (simulation_parameters.controlArchitecture == DISTRIBUTED && simulation_parameters.mapCoverageStrategy != NONE)
		{
			FSM.state = STATE_REGION_ALLOCATION;
			FSM.hook = state_region_allocation;
		}
		else if (simulation_parameters.controlArchitecture == DISTRIBUTED && simulation_parameters.mapCoverageStrategy == NONE)
		{
			FSM.state = STATE_UPDATEGOAL;
			FSM.hook = state_updategoal;
		}
	}

}

void state_receive_pdf()
{
	if (!entered_state_receive_pdf)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_PDF \n", ID);
		entered_state_receive_pdf = true;
	}

	
	if (!printed && posterior.pdf != nullptr)
	{
		memcpy(belief_maps[self_index], posterior.pdf, MAP_SIZE * sizeof(double));
		printed = true;
	}

	while (wb_receiver_get_queue_length(receiver) > 0)
	{
		const void *data = wb_receiver_get_data(receiver);
		const double *buffer = (const double *)data;

		int sender_ID = (int)buffer[0];
		int message_type = (int)buffer[1];
		float received_current_pos_x = buffer[2];
		float received_current_pos_y = buffer[3];

		int sender_index = sender_ID - 101;

		if (message_type == MSG_STATE_SHARE_PDF)
		{
			ready_from[sender_index] = 1;
			printf("Robot ID [%d]: Received READY from Robot ID [%d]\n", ID, sender_ID);
			wb_receiver_next_packet(receiver);
			continue;
		}

		if (sender_ID != ID && sender_index >= 0 && sender_index < MAX_NB_ROBOTS)
		{
			memcpy(belief_maps[sender_index], &buffer[10], MAP_SIZE * sizeof(double));
			received_current_pos[sender_index][0] = received_current_pos_x;
			received_current_pos[sender_index][1] = received_current_pos_y;
			received_flags[sender_index] = 1;
		}

		wb_receiver_next_packet(receiver);
	}

	// Send READY message if received all maps
	int all_received = 1;
	for (int i = 0; i < simulation_parameters.number_of_robots; i++)
	{
		if (i == self_index) continue;
		if (!received_flags[i])
		{
			all_received = 0;
			break;
		}
	}

	if (all_received && !i_am_ready)
	{
		Message msg(ID, MSG_STATE_SHARE_PDF, Position(), -1, Position(), 1);
		send_msg(msg);
		i_am_ready = 1;

		printf("Robot ID [%d]: Sent READY message.\n", ID);
	}

	// Check if all READY received
	int ready_count = 0;
	for (int i = 0; i < simulation_parameters.number_of_robots; i++)
	{
		if (i == self_index) continue;
		if (ready_from[i]) ready_count++;
	}

	if (i_am_ready && ready_count == simulation_parameters.number_of_robots - 1)
	{

		entered_state_receive_pdf = false;
		printed = false;
		i_am_ready = 0;

		// Reset flags
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

void state_share_pdf()
{
    printf("Robot ID [%d]: STATE_SHARE_PDF\n", ID);

    if (posterior.pdf == NULL) {
        printf("Robot ID [%d]: No posterior PDF to share.\n", ID);
    } else {
        for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
            int target_ID = 101 + i;
            if (target_ID == ID) continue; // skip self

			// save my current position
			received_current_pos[self_index][0] = current_position.x;
			received_current_pos[self_index][1] = current_position.y;

			// share my current position & belief map with teammates
            Message msg(ID, target_ID, current_position, -1, Position(), 0, posterior.pdf);
            send_msg(msg);

            printf("Robot ID [%d]: Sent belief map to Robot ID [%d]\n", ID, target_ID);
        }
    }


    FSM.state = STATE_RECEIVE_PDF;
    FSM.hook = state_receive_pdf;
}

void state_region_allocation()
{
	printf("Robot ID [%d]: STATE_REGION_ALLOCATION \n", ID);
	log_position();
	log_whole_position_history();
	posterior.log_pdf(posterior_pdf_3D_name, 'a');

	region_allocator.region_allocation_algorithm();

	FSM.state = STATE_UPDATEGOAL;
	FSM.hook = state_updategoal;
	
}


// =========================== MAIN ============================== //
int main(int argc, char **argv)
{
	init();
	read_obstacle_list();
	// Debugging
	for (int row = 0; row < y_nbins; ++row) {
		for (int col = 0; col < x_nbins; ++col) {
			std::cout << obstacle_map[row * x_nbins + col] << " ";
		}
		std::cout << "\n";
	}

	// visibility graph (visilibity library)
	std::cout << "Validating environment model . . . ";
	if (my_environment.is_valid(epsilon))
		std::cout << "OK" << std::endl;
	else
	{
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
	VisiLibity::Visibility_Graph vg(my_environment, epsilon);

	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	plume_model.initialize_potential_gas_maps();
	// plume_model.log_gas_concentration_map(gas_concentration_map_logfile_name, 'a');

	// File for saving console messages
	console_logfile = fopen(console_logfile_name, "a");

	// debugging purpose
	clustering_debugging_logfile = fopen(clustering_debugging_name, "a");
	cumulative_path_logfile = fopen(cumulative_path_logfile_name, "a");


	// initialize the receiver
	init_receiver(TIME_STEP);

	// //main loop
	while (FSM.state != STATE_FINISHED)
	{
		if (simulation_parameters.simuMode == ONLINE)
		{
			update_position();
		}
		// if we are not stopped to sample, let's log the trajectory
		/*if(speed[0]!=0 && speed[1]!=0){
			log_trajectory();
		}*/
		// send info (current position and goal position) to friends for joint planning
		if (simulation_parameters.scenario == COM_COOR)
		{
			// //if a robot is already sampling (i.e. STATE_MEASURING), its goal should not be swapped with anyone. Therefore we send 0 as goal and current position
			if (FSM.state == STATE_MEASURING)
			{
				Position tmp;
				Message msg(ID, 1, tmp, sampleBuffer.average_concentration, tmp, 1);
				send_msg(msg);
			}
			else
			{
				Message msg(ID, 1, current_position, sampleBuffer.average_concentration, goal_position, 1);
				// printf("[%d] I am sendig my position %f %f %f and my goal %f %f %f\n", msg.sender_ID, msg.position[0],msg.position[1], msg.position[2], msg.goal[0],msg.goal[1],msg.goal[2]);
				send_msg(msg);
			}
		}
		// receive info from colleagues for joint planning and history
		if (simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR)
		{
			// printf("Robot %d has %d unread messages! \n", ID, wb_receiver_get_queue_length(receiver));
			receive_from_friends();
			// printf("[%d] my history size is %d\n", ID, history.size);
		}
		// change state
		FSM.hook();
		// next
		next_iteration_time();
	}

	fclose(console_logfile);

	// debugging purpose
	fclose(clustering_debugging_logfile);

	fclose(cumulative_path_logfile);

	// final logs
	if (simulation_parameters.simuMode == ONLINE)
	{
		log_history();
		log_experiment_time();
		log_visited_list();
		cleaning_up();
	}
	// free the allocated memory to obstacle array
	for (int i = 0; i < nbr_obs; i++)
		delete[] obstacles[i];
	delete[] obstacles;

	exit(0);
}

// Broadcast STATE_FINISHED to all teammates
void broadcast_state_finished()
{
    printf("Robot ID [%d]: Broadcasting STATE_FINISHED\n", ID);


    for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
        int target_ID = 101 + i;
        if (target_ID == ID) continue;  // Skip self

		// Notify the TEAMMATES
        Message msg(ID, target_ID, Position(), -1, Position(), -1); // -1 for STATE_FINISHED flag
        send_msg(msg);

		// Notify the SUPERVISOR
		current_position.x = -1;
		current_position.y = -1;

		Message msg2(ID, 0, current_position, -1, current_position, -1); // -1 for STATE_FINISHED flag
        send_msg(msg2);

        printf("Robot ID [%d]: Sent STATE_FINISHED to Robot ID [%d]\n", ID, target_ID);
    }
}

bool check_stop_condition()
{
	bool is_stop_condition_reached = false;
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
	// printf("entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior.entropy, posterior.x_spatial_spread, posterior.y_spatial_spread);
	fprintf(console_logfile, "entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior.entropy, posterior.x_spatial_spread, posterior.y_spatial_spread);
	if (is_stop_condition_reached)
	{
		// log the finish time stamp
		end_timestamp = wb_robot_get_time();
		printf("Robot ID [%d]: Stop condition reached!\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: Stop condition reached!\n", ID);
	}
	return is_stop_condition_reached;
}
