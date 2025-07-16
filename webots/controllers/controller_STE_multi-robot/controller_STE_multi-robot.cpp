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
// #include "aStar.h"
#include "visilibity.hpp"
// #include <webots/supervisor.h>
#include "constant_and_helper.h"
#include <unistd.h>

#define MSG_STATE_SHARE_PDF -99
#define MSG_STATE_SHARE_DIST -98
#define MSG_SUBSTATE_SHARE_PDF -97
#define MSG_SUBSTATE_SHARE_KLD -96


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
FILE *kld_value_logfile;
FILE *time_logfile;
FILE *max_a_posteriori_logfile;
FILE *posterior_pdf_3D;
FILE *posterior_pdf_3D_cluster; //new: for MATLAB Distributed clustering visualization
FILE *posterior_pdf_3D_cluster_lloyd;
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
FILE *KLD_debugging_logfile; //new: for debugging purpose (KLD)
FILE *cumulative_path_logfile;


char goalposition_logfile_name[512];
char measurements_logfile_name[512];
char expected_a_posteriori_logfile_name[512];
char max_a_posteriori_logfile_name[512];
char likelihood_logfile_name[512];
char measurement_history_logfile_name[512];
char position_history_logfile_name[512];
char whole_position_history_logfile_name[512];
char kld_value_logfile_name[512];
char time_logfile_name[512];
char posterior_pdf_3D_name[512];
char posterior_pdf_3D_cluster_name[512];
char posterior_pdf_3D_cluster_lloyd_name[512];
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
char KLD_debugging_name[512];
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

// nested finite state machine
struct FSM_Sub FSM_Sub;

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

// iteration number
int n_time_step = 0;

// update pdf cluster
int upd_pdf_cluster = 0;

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

double initial_time_results;  // Variable global


// precomputed gas maps
double c_avg[MAP_SIZE][MAP_SIZE]; // left: source position, right: estimated cell position

// store last position for minimum distance threshold in measurement update
Position last_measurement_pos;

// real source position
double real_source_pos[3] = {0, 0, 0};

bool use_visibility_path = true;

// store the belief maps of the whole team
static double belief_maps[MAX_NB_ROBOTS][MAP_SIZE] = {0};
static int received_flags[MAX_NB_ROBOTS] = {0};
static double belief_map_merged[MAP_SIZE] = {0};
static int belief_clusters[x_nbins*y_nbins] = {0};
static float received_current_pos[MAX_NB_ROBOTS][2] = {0};
static double received_current_dist[MAX_NB_ROBOTS] = {0};
static double received_current_kld[MAX_NB_ROBOTS] = {0};
static int ready_from[MAX_NB_ROBOTS] = {0};
static int i_am_ready = 0;
static double epsilon_clusterize = 0;
double start_substates;

// KLD applied on consecutive Belief Maps
static double actual_belief_map[MAP_SIZE] = {0};
static int n_meas = 0;
static double actual_sum = 0.0;
static double distance_to_goal = 0;
double KLD = 0;
int n_kld = 0;
int kld_replan = 0;

// print variables
bool printed = false;
bool entered_state_receive_pdf = false;
bool entered_state_receive_dist = false;
bool entered_state_receive_kld = false;
bool entered_substate_receive_pdf = false;
bool entered_state_moving = false;
bool entered_state_receive_goal = false;
bool kld_subloop_upd = false;
bool movesting_initialized = false;

// transforming update_time to number of iterations
int compare_pdf_iterations = static_cast<int>(round(simulation_parameters.update_time * 1000.0 / TIME_STEP));

// obstacle map
std::vector<int> obstacle_map = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

int n_combine_pathplanning = 0;

// =========================== FUNCTIONS ================================ //
// debugging function
const int MAP_WIDTHX = 20;
const int MAP_HEIGHTY = 20;
const int MAP_SIZEXY = MAP_WIDTHX * MAP_HEIGHTY;

std::vector<std::pair<int, int>> find_top_3(const double* map) {
    double top_vals[3] = {-1.0, -1.0, -1.0};
    int top_idxs[3] = {-1, -1, -1};

    for (int i = 0; i < MAP_SIZEXY; ++i) {
        double val = map[i];

        if (val > top_vals[0]) {
            top_vals[2] = top_vals[1]; top_idxs[2] = top_idxs[1];
            top_vals[1] = top_vals[0]; top_idxs[1] = top_idxs[0];
            top_vals[0] = val;         top_idxs[0] = i;
        } else if (val > top_vals[1]) {
            top_vals[2] = top_vals[1]; top_idxs[2] = top_idxs[1];
            top_vals[1] = val;         top_idxs[1] = i;
        } else if (val > top_vals[2]) {
            top_vals[2] = val;         top_idxs[2] = i;
        }
    }

    std::vector<std::pair<int, int>> result;

    for (int i = 0; i < 3; ++i) {
        int row = top_idxs[i] / MAP_WIDTHX;
        int col = top_idxs[i] % MAP_WIDTHX;
        result.emplace_back(row, col);
    }

    return result;
}


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
		upd_pdf_cluster = i;
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

void log_whole_position_history(bool real_position)
{
    whole_position_history_logfile = fopen(whole_position_history_logfile_name, "a"); // append mode, to add new rows without removing the previous ones

    fprintf(whole_position_history_logfile, "%f,%f,", current_position.x, current_position.y);

    fprintf(whole_position_history_logfile, "\n");
    fclose(whole_position_history_logfile);
}

void log_kld_value()
{
	double time = wb_robot_get_time();
    kld_value_logfile = fopen(kld_value_logfile_name, "a"); // append mode, to add new rows without removing the previous ones

    fprintf(kld_value_logfile, "%i,%f,%i,%f,%f,%f,", FSM.n_cycles, KLD, kld_replan, current_position.x, current_position.y, time);

    fprintf(kld_value_logfile, "\n");
    fclose(kld_value_logfile);
}

void log_time(double simulation_time_results) {
    time_logfile = fopen(time_logfile_name, "a"); // modo 'w' para sobrescribir (1x1 casilla)



    fprintf(time_logfile, "%f\n", simulation_time_results);  // una sola fila, una sola columna
    fprintf(time_logfile, "\n");

    fclose(time_logfile);
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

void log_posterior_pdf_3d_cluster(const int* belief_clusters)
{
	posterior_pdf_3D_cluster = fopen(posterior_pdf_3D_cluster_name, "a"); // append mode, to add new rows without removing the previous ones

	// Save current position of the robot
	fprintf(posterior_pdf_3D_cluster, "%f,%f,", current_position.x, current_position.y);

	// Save clusterized belief map in one row of the .csv
	for (int y = 0; y < y_nbins; ++y) {
		for (int x = 0; x < x_nbins; ++x) {
			int idx = y * x_nbins + x;
			fprintf(posterior_pdf_3D_cluster, "%d", belief_clusters[idx]);

			if (x != x_nbins - 1 || y != y_nbins - 1) // no coma at the end
				fprintf(posterior_pdf_3D_cluster, ",");
		}
	}

	fprintf(posterior_pdf_3D_cluster, "\n");
	fclose(posterior_pdf_3D_cluster);
}

void log_posterior_pdf_3d_cluster_lloyd(const int* belief_clusters)
{
	posterior_pdf_3D_cluster_lloyd = fopen(posterior_pdf_3D_cluster_lloyd_name, "a"); // append mode, to add new rows without removing the previous ones

	// Save current position of the robot
	fprintf(posterior_pdf_3D_cluster_lloyd, "%f,%f,", current_position.x, current_position.y);

	// Save clusterized belief map in one row of the .csv
	for (int y = 0; y < y_nbins; ++y) {
		for (int x = 0; x < x_nbins; ++x) {
			int idx = y * x_nbins + x;
			fprintf(posterior_pdf_3D_cluster_lloyd, "%d", belief_clusters[idx]);

			if (x != x_nbins - 1 || y != y_nbins - 1) // no coma at the end
				fprintf(posterior_pdf_3D_cluster_lloyd, ",");
		}
	}

	fprintf(posterior_pdf_3D_cluster_lloyd, "\n");
	fclose(posterior_pdf_3D_cluster_lloyd);
}



// ================== CLUSTERING APPROACHES FUNCTIONS ====================== //
void index_to_xy(int index, int &x, int &y) {
	y = index / x_nbins;
	x = index % x_nbins;
}

int xy_to_index(double x, double y) {
    double dx = (XMAX - XMIN) / x_nbins;
    double dy = (YMAX - YMIN) / y_nbins;

    int ix = static_cast<int>(std::floor((x - XMIN) / dx));
    int iy = static_cast<int>(std::floor((y - YMIN) / dy));
    if (ix < 0) ix = 0; else if (ix >= x_nbins) ix = x_nbins - 1;
    if (iy < 0) iy = 0; else if (iy >= y_nbins) iy = y_nbins - 1;
    return iy * x_nbins + ix;
}

// void clusterize_belief_map(const std::vector<int>& selected_indices) {


//     // Binary masks for cutting regions from individual pdfs
//     std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots,
//                                             std::vector<int>(x_nbins * y_nbins, 0));

//     // Are the (x,y) coordinates for the centroids
//     std::vector<int> seeds_x(simulation_parameters.number_of_robots),
//                      seeds_y(simulation_parameters.number_of_robots);

//     // Distance matrices
//     std::vector<std::vector<double>> dist_map(simulation_parameters.number_of_robots,
//                                               std::vector<double>(x_nbins * y_nbins));

//     // Convert bins to (x,y) coordinates
//     for (int i = 0; i < simulation_parameters.number_of_robots; ++i) {
//         index_to_xy(selected_indices[i], seeds_x[i], seeds_y[i]);
//     }

//     // Compute distances from each cell to each centroid
//     for (int i = 0; i < x_nbins * y_nbins; ++i) {
// 		if (posterior.pdf[i] <= epsilon_clusterize) continue;
//         int x, y;
//         index_to_xy(i, x, y);
//         for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
//             double dx = x - seeds_x[k];
//             double dy = y - seeds_y[k];
//             dist_map[k][i] = dx * dx + dy * dy;
//         }
//     }

//     // Assign cells to the nearest centroid
//     std::vector<int> cluster_sizes(simulation_parameters.number_of_robots, 0);
//     std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0); // Inicializar

//     for (int i = 0; i < x_nbins * y_nbins; ++i) {
// 		if (posterior.pdf[i] <= epsilon_clusterize) continue;
//         double min_dist = dist_map[0][i];
//         int min_k = 0;
//         for (int k = 1; k < simulation_parameters.number_of_robots; ++k) {
//             if (dist_map[k][i] < min_dist) {
//                 min_dist = dist_map[k][i];
//                 min_k = k;
//             }
//         }
//         belief_clusters[i] = min_k + 1; // Labeled pdf according to clusters (values 1,2,3..)
//         cluster_sizes[min_k]++;
//     }

//     log_posterior_pdf_3d_cluster(belief_clusters);

//     // Fill binary masks
//     for (int i = 0; i < x_nbins * y_nbins; ++i) {
// 		if (posterior.pdf[i] <= epsilon_clusterize) continue;
//         for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
//             pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
//         }
//     }

//     // Apply each binary mask to the merged pdf, to obtain individual pdf corresponding to the cutted region
//     for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
//         if (ID == 101 + k)
// 		{
//             for (int i = 0; i < x_nbins * y_nbins; ++i) {
//                 posterior.pdf[i] *= pdf_masks[k][i];
//             }

// 			// debugging purpose
// 			// fprintf(clustering_debugging_logfile, "Robot ID [%d]: Individual pdf after mask:\n", ID);
// 			// for (int y = 0; y < y_nbins; ++y) {
// 			// 	for (int x = 0; x < x_nbins; ++x) {
// 			// 		int idx = y * x_nbins + x;
// 			// 		fprintf(clustering_debugging_logfile, "%.9f ", posterior.pdf[idx]);
// 			// 	}
// 			// 	fprintf(clustering_debugging_logfile, "\n");
// 			// }
//         }
//     }

// 	fprintf(clustering_debugging_logfile, "Robot ID [%d]: NO Normalized Individual pdf after mask:\n", ID);
// 	for (int y = 0; y < y_nbins; ++y) {
// 		for (int x = 0; x < x_nbins; ++x) {
// 			int idx = y * x_nbins + x;
// 			fprintf(clustering_debugging_logfile, "%.8f ", posterior.pdf[idx]);
// 		}
// 		fprintf(clustering_debugging_logfile, "\n");
// 	}

//     // Normalize individual pdf
//     double sum = 0.0;
//     for (int i = 0; i < x_nbins * y_nbins; ++i) {
//         sum += posterior.pdf[i];
//     }
//     if (sum > 0) {
//         for (int i = 0; i < x_nbins * y_nbins; ++i) {
//             posterior.pdf[i] /= sum;
//         }
//     }

//     double sum_def = 0.0;
//     for (int i = 0; i < x_nbins * y_nbins; ++i) {
//         sum_def += posterior.pdf[i];
//     }
//     printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

//     // debugging purpose
//     for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
//         int self_index = 101 + k;
// 		if (ID == self_index)
// 		{
// 			fprintf(clustering_debugging_logfile, "\n Clusterized Belief Map (1, 2, 3):\n");
// 			for (int y = 0; y < y_nbins; ++y) {
// 				for (int x = 0; x < x_nbins; ++x) {
// 					int idx = y * x_nbins + x;
// 					fprintf(clustering_debugging_logfile, "%d ", belief_clusters[idx]);
// 				}
// 				fprintf(clustering_debugging_logfile, "\n");
// 			}
// 			fprintf(clustering_debugging_logfile, "\nRobot ID [%d]: Cluster binary mask:\n", ID);
// 			for (int y = 0; y < y_nbins; ++y) {
// 				for (int x = 0; x < x_nbins; ++x) {
// 					int idx = y * x_nbins + x;
// 					fprintf(clustering_debugging_logfile, "%d ", pdf_masks[k][idx]);
// 				}
// 				fprintf(clustering_debugging_logfile, "\n");
// 			}
// 			// debugging purpose
// 			fprintf(clustering_debugging_logfile, "Robot ID [%d]: Normalized Individual pdf after mask:\n", ID);
// 			for (int y = 0; y < y_nbins; ++y) {
// 				for (int x = 0; x < x_nbins; ++x) {
// 					int idx = y * x_nbins + x;
// 					fprintf(clustering_debugging_logfile, "%.9f ", posterior.pdf[idx]);
// 				}
// 				fprintf(clustering_debugging_logfile, "\n");
// 			}
// 			fprintf(clustering_debugging_logfile, "\nRobot ID [%d]: Bins:\n", ID);
// 			for (int y = 0; y < y_nbins; ++y) {
// 				for (int x = 0; x < x_nbins; ++x) {
// 					int idx = y * x_nbins + x;
// 					if ((idx+1)<10)
// 					{
// 						fprintf(clustering_debugging_logfile, "00%i ", idx+1);
// 					}
// 					else if ((idx+1)>=10 && (idx+1)<100)
// 					{
// 						fprintf(clustering_debugging_logfile, "0%i ", idx+1);
// 					}
// 					else{
// 						fprintf(clustering_debugging_logfile, "%i ", idx+1);
// 					}

// 				}
// 				fprintf(clustering_debugging_logfile, "\n");
// 			}
// 		}
//     }
// }
void prova_3maxcells_clustering_continuo(const std::vector<int>& selected_indices) {

    std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots, std::vector<int>(x_nbins * y_nbins, 0));
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));

    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    // Buscar las 3 celdas de mayor peso
    std::vector<std::pair<double, int>> top_cells;
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] > epsilon_clusterize)
            top_cells.push_back({posterior.pdf[i], i});
    }
    std::sort(top_cells.begin(), top_cells.end(), std::greater<>());

    std::vector<int> seeds_x(3), seeds_y(3);
    for (int i = 0; i < 3 && i < top_cells.size(); ++i)
        index_to_xy(top_cells[i].second, seeds_x[i], seeds_y[i]);

    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);

    // Region Growing alternado y balanceado
    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::queue<std::pair<int, int>> queues[3];
    int cluster_sizes[3] = {0, 0, 0};

    int total_active_cells = 0;
    for (int i = 0; i < x_nbins * y_nbins; ++i)
        if (posterior.pdf[i] > epsilon_clusterize && obstacle_map[i] == 0)
            total_active_cells++;

    int target_size = total_active_cells / 3;
    int max_size_margin = target_size * 1.2; // Permite un 20% de margen

    for (int k = 0; k < 3; ++k) {
        queues[k].push({seeds_x[k], seeds_y[k]});
        visited[seeds_y[k]][seeds_x[k]] = true;
        belief_clusters[seeds_y[k] * x_nbins + seeds_x[k]] = k + 1;
        cluster_sizes[k]++;
    }

    bool growing = true;
    while (growing) {
        growing = false;
        for (int k = 0; k < 3; ++k) {
            if (cluster_sizes[k] >= max_size_margin) continue;

            int q_size = queues[k].size();
            for (int step = 0; step < q_size; ++step) {
                auto [x, y] = queues[k].front(); queues[k].pop();

                for (const auto& [dx, dy] : directions) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                    if (obstacle_map_2d[ny][nx] == 1) continue;
                    if (visited[ny][nx]) continue;
                    int idx = ny * x_nbins + nx;
                    if (posterior.pdf[idx] <= epsilon_clusterize) continue;

                    visited[ny][nx] = true;
                    belief_clusters[idx] = k + 1;
                    queues[k].push({nx, ny});
                    cluster_sizes[k]++;
                    growing = true;

                    if (cluster_sizes[k] >= max_size_margin)
                        break;
                }
                if (cluster_sizes[k] >= max_size_margin)
                    break;
            }
        }
    }

    log_posterior_pdf_3d_cluster(belief_clusters);

    // Asignar pdf_masks
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= epsilon_clusterize) continue;
        for (int k = 0; k < 3; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

    // Modificar posterior para el robot correspondiente
    for (int k = 0; k < 3; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior.pdf[i] *= pdf_masks[k][i];
        }
    }

    double sum = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
    if (sum > 0)
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            posterior.pdf[i] /= sum;

    double sum_def = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
    printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

    // Debug opcional
    for (int k = 0; k < 3; ++k) {
        if (ID == 101 + k) {
            fprintf(clustering_debugging_logfile, "\n Clusterized Belief Map (balanceado):\n");
            for (int y = 0; y < y_nbins; ++y) {
                for (int x = 0; x < x_nbins; ++x) {
                    int idx = y * x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%d ", belief_clusters[idx]);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }
        }
    }
}




void clusterize_belief_map_obstacle_dist(const std::vector<int>& selected_indices) {

    std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots, std::vector<int>(x_nbins * y_nbins, 0));
    std::vector<int> seeds_x(simulation_parameters.number_of_robots), seeds_y(simulation_parameters.number_of_robots);
    std::vector<std::vector<double>> dist_map(simulation_parameters.number_of_robots, std::vector<double>(x_nbins * y_nbins, std::numeric_limits<double>::max()));

    // Convert bins to (x,y)
    for (int i = 0; i < simulation_parameters.number_of_robots; ++i)
        index_to_xy(selected_indices[i], seeds_x[i], seeds_y[i]);

    // Convert flat obstacle map to 2D
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));
    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    // --- BFS Distance Map (Obstacle aware)
	double cell_size_x = (XMAX - XMIN) / static_cast<double>(x_nbins);
	double cell_size_y = (YMAX - YMIN) / static_cast<double>(y_nbins);

	// Von Neumann (4 directions)
	// const std::vector<std::pair<int, int>> directions = {{1,0},{-1,0},{0,1},{0,-1}};

	// Moore (8 directions)
	const std::vector<std::pair<int, int>> directions = {
		{1, 0}, {-1, 0}, {0, 1}, {0, -1},
		{1, 1}, {1, -1}, {-1, 1}, {-1, -1}
	};

	for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
		std::vector<std::vector<double>> local_dist(y_nbins, std::vector<double>(x_nbins, std::numeric_limits<double>::max()));
		std::queue<std::pair<int, int>> q;

		int sx = seeds_x[k], sy = seeds_y[k];

		local_dist[sy][sx] = 0.0;
		q.push({sx, sy});

		while (!q.empty()) {
			auto [x, y] = q.front(); q.pop();
			double d = local_dist[y][x];

			for (const auto& [dx, dy] : directions) {
				int nx = x + dx, ny = y + dy;
				if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
				if (obstacle_map_2d[ny][nx] == 1) continue;

				// Real cost of moving on each direction (meters)
				double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));

				if (local_dist[ny][nx] > d + cost) {
					local_dist[ny][nx] = d + cost;
					q.push({nx, ny});
				}
			}
		}

		// Flatten map from 2D to 1D
		for (int y = 0; y < y_nbins; ++y)
			for (int x = 0; x < x_nbins; ++x)
				dist_map[k][y * x_nbins + x] = local_dist[y][x];


		// debugging purpose
		fprintf(clustering_debugging_logfile, "\nRobot ID [%d]: Distances map (for Robot ID [%d])\n", ID, k+101);
		for (int y = 0; y < y_nbins; ++y) {
			for (int x = 0; x < x_nbins; ++x) {
				int idx = y * x_nbins + x;
				double val = dist_map[k][idx];
				if (val == std::numeric_limits<double>::max())
					fprintf(clustering_debugging_logfile, "      ");
				else
					fprintf(clustering_debugging_logfile, "%5.2f ", val);
			}
			fprintf(clustering_debugging_logfile, "\n");
		}

    }




    // Clusters assignation
    std::vector<int> cluster_sizes(simulation_parameters.number_of_robots, 0);
    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= epsilon_clusterize) continue;

		// No weighted
		double min_dist = dist_map[0][i];
        int min_k = 0;
        for (int k = 1; k < simulation_parameters.number_of_robots; ++k) {
            if (dist_map[k][i] < min_dist) {
                min_dist = dist_map[k][i];
                min_k = k;
            }
        }


        belief_clusters[i] = min_k + 1;
        cluster_sizes[min_k]++;
    }

    log_posterior_pdf_3d_cluster(belief_clusters);

    // Binary masks and cutting merged pdf for each robot
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= epsilon_clusterize) continue;
        for (int k = 0; k < simulation_parameters.number_of_robots; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior.pdf[i] *= pdf_masks[k][i];
        }
    }

    fprintf(clustering_debugging_logfile, "Robot ID [%d]: NO Normalized Individual pdf after mask:\n", ID);
    for (int y = 0; y < y_nbins; ++y) {
        for (int x = 0; x < x_nbins; ++x) {
            int idx = y * x_nbins + x;
            fprintf(clustering_debugging_logfile, "%.9f ", posterior.pdf[idx]);
        }
        fprintf(clustering_debugging_logfile, "\n");
    }

    double sum = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
    if (sum > 0)
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            posterior.pdf[i] /= sum;

    double sum_def = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
    printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

    // debugging purpose
    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        if (ID == 101 + k) {
            fprintf(clustering_debugging_logfile, "\n Clusterized Belief Map (1, 2, 3):\n");
            for (int y = 0; y < y_nbins; ++y) {
                for (int x = 0; x < x_nbins; ++x) {
                    int idx = y * x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%d ", belief_clusters[idx]);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }

            fprintf(clustering_debugging_logfile, "\nRobot ID [%d]: Cluster binary mask:\n", ID);
            for (int y = 0; y < y_nbins; ++y) {
                for (int x = 0; x < x_nbins; ++x) {
                    int idx = y * x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%d ", pdf_masks[k][idx]);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }

            fprintf(clustering_debugging_logfile, "Robot ID [%d]: Normalized Individual pdf after mask:\n", ID);
            for (int y = 0; y < y_nbins; ++y) {
                for (int x = 0; x < x_nbins; ++x) {
                    int idx = y * x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%.9f ", posterior.pdf[idx]);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }

            fprintf(clustering_debugging_logfile, "\nRobot ID [%d]: Bins:\n", ID);
            for (int y = 0; y < y_nbins; ++y) {
                for (int x = 0; x < x_nbins; ++x) {
                    int idx = y * x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%03d ", idx + 1);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }
        }
    }
}

void clusterize_belief_map_lloyd(const std::vector<int>& selected_indices) {
    const int K = simulation_parameters.number_of_robots;
    const int max_iterations = 10;

    std::vector<std::pair<double, double>> centroids(K);
    std::vector<int> seeds_x(K), seeds_y(K);
    for (int i = 0; i < K; ++i) {
        index_to_xy(selected_indices[i], seeds_x[i], seeds_y[i]);
        centroids[i] = { static_cast<double>(seeds_x[i]), static_cast<double>(seeds_y[i]) };
    }

    std::vector<int> cluster_assignments(x_nbins * y_nbins, -1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // STEP 1: Assign each cell to the nearest cluster (without considering obstacles)
        std::vector<int> temp_cluster_sizes(K, 0);

        for (int i = 0; i < x_nbins * y_nbins; ++i) {
            if (posterior.pdf[i] <= epsilon_clusterize) continue;
            int x, y;
            index_to_xy(i, x, y);

            double min_dist = std::numeric_limits<double>::max();
            int best_k = -1;
            for (int k = 0; k < K; ++k) {
                double dx = x - centroids[k].first;
                double dy = y - centroids[k].second;
                double dist = dx * dx + dy * dy;
                if (dist < min_dist || (dist == min_dist && temp_cluster_sizes[k] < temp_cluster_sizes[best_k])) {
                    min_dist = dist;
                    best_k = k;
                }
            }
            cluster_assignments[i] = best_k;
            temp_cluster_sizes[best_k]++;
        }

        // STEP 2: Recompute GEOMETRICAL centroids
        // std::vector<double> sum_x(K, 0.0), sum_y(K, 0.0);
        // std::vector<int> count(K, 0);

        // for (int i = 0; i < x_nbins * y_nbins; ++i) {
        //     int cluster = cluster_assignments[i];
        //     if (cluster < 0) continue;
        //     int x, y;
        //     index_to_xy(i, x, y);
        //     sum_x[cluster] += x;
        //     sum_y[cluster] += y;
        //     count[cluster]++;
        // }

        // for (int k = 0; k < K; ++k) {
        //     if (count[k] > 0) {
        //         centroids[k].first  = sum_x[k] / count[k];
        //         centroids[k].second = sum_y[k] / count[k];
        //     }
        // }

		// STEP 2: Recompute INFORMATIVE centroids
		std::vector<double> weighted_sum_x(K, 0.0), weighted_sum_y(K, 0.0), total_weight(K, 0.0);
		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			int cluster = cluster_assignments[i];
			if (cluster < 0) continue;

			double weight;

			if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_DIST) 
			{
				weight = 1;
			}
			else if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_PROB)
			{
				weight = posterior.pdf[i];
			}		

			if (weight < epsilon_clusterize) continue;

			int x, y;
			index_to_xy(i, x, y);
			weighted_sum_x[cluster] += x * weight;
			weighted_sum_y[cluster] += y * weight;
			total_weight[cluster] += weight;
		}

		for (int k = 0; k < K; ++k) {
			if (total_weight[k] > epsilon_clusterize) {
				centroids[k].first  = static_cast<int>(weighted_sum_x[k] / total_weight[k]);
				centroids[k].second = static_cast<int>(weighted_sum_y[k] / total_weight[k]);
			}
		}

        // If we want to visualize the evolution of the BM along the 10 iterations refining
        for (int i = 0; i < x_nbins * y_nbins; ++i) {
            if (posterior.pdf[i] <= epsilon_clusterize) continue;
            belief_clusters[i] = cluster_assignments[i] + 1;
        }
        log_posterior_pdf_3d_cluster_lloyd(belief_clusters);

    }

    // Assign cluster labels
    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);
    std::vector<std::vector<int>> pdf_masks(K, std::vector<int>(x_nbins * y_nbins, 0));

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= epsilon_clusterize) continue;
        int cluster = cluster_assignments[i];
        belief_clusters[i] = cluster + 1;
        for (int k = 0; k < K; ++k) {
            pdf_masks[k][i] = (cluster == k) ? 1 : 0;
        }
    }

    log_posterior_pdf_3d_cluster(belief_clusters);

    for (int k = 0; k < K; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior.pdf[i] *= pdf_masks[k][i];

            double sum = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
            if (sum > 0.0) {
                for (int i = 0; i < x_nbins * y_nbins; ++i)
                    posterior.pdf[i] /= sum;
            }

            double sum_def = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
            printf("Robot ID [%d]: Sum of Lloyd-clustered pdf is %f \n", ID, sum_def);
        }
    }
}

void clusterize_belief_map_obstacle_dist_lloyd(const std::vector<int>& selected_indices)
{
    const int K = simulation_parameters.number_of_robots;
    const int max_iterations = 10;

    std::vector<std::pair<int, int>> seeds(K);
    for (int i = 0; i < K; ++i)
        index_to_xy(selected_indices[i], seeds[i].first, seeds[i].second);

    // Distance maps (considering obstacles)
    std::vector<std::vector<double>> dist_map(K, std::vector<double>(x_nbins * y_nbins, std::numeric_limits<double>::max()));

    // Convert flat map to 2D
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));
    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    // Distance Map (Obstacle aware)
    double cell_size_x = (XMAX - XMIN) / static_cast<double>(x_nbins);
    double cell_size_y = (YMAX - YMIN) / static_cast<double>(y_nbins);

	// Moore
    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    for (int k = 0; k < K; ++k) {
        std::vector<std::vector<double>> local_dist(y_nbins, std::vector<double>(x_nbins, std::numeric_limits<double>::max()));
        std::queue<std::pair<int, int>> q;

        int sx = seeds[k].first, sy = seeds[k].second;
        local_dist[sy][sx] = 0.0;
        q.push({sx, sy});

        while (!q.empty()) {
            auto [x, y] = q.front(); q.pop();
            double d = local_dist[y][x];

            for (const auto& [dx, dy] : directions) {
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                if (obstacle_map_2d[ny][nx] == 1) continue;

                double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));
                if (local_dist[ny][nx] > d + cost) {
                    local_dist[ny][nx] = d + cost;
                    q.push({nx, ny});
                }
            }
        }

        for (int y = 0; y < y_nbins; ++y)
            for (int x = 0; x < x_nbins; ++x)
                dist_map[k][y * x_nbins + x] = local_dist[y][x];

		// debugging purpose
		fprintf(clustering_debugging_logfile, "\nRobot ID [%d]: Distances map (for Robot ID [%d])\n", ID, k+101);
		for (int y = 0; y < y_nbins; ++y) {
			for (int x = 0; x < x_nbins; ++x) {
				int idx = y * x_nbins + x;
				double val = dist_map[k][idx];
				if (val == std::numeric_limits<double>::max())
					fprintf(clustering_debugging_logfile, "      ");
				else
					fprintf(clustering_debugging_logfile, "%5.2f ", val);
			}
			fprintf(clustering_debugging_logfile, "\n");
		}
    }

    // Lloyd refining algorithm (considering obstacles in distances matrix)
    std::vector<int> cluster_assignments(x_nbins * y_nbins, -1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<int> temp_cluster_sizes(K, 0);

        // STEP 1: Assign each cell to the nearest cluster (considering obstacles)
		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			if (posterior.pdf[i] <= epsilon_clusterize) continue;

			double min_dist = dist_map[0][i];
			int best_k = 0;

			for (int k = 1; k < K; ++k) {
				double dist = dist_map[k][i];
				if (dist < min_dist || (dist == min_dist && temp_cluster_sizes[k] < temp_cluster_sizes[best_k])) {
					min_dist = dist;
					best_k = k;
				}
			}

			cluster_assignments[i] = best_k;
			temp_cluster_sizes[best_k]++;
		}

        // STEP 2: Recompute GEOMETRICAL centroids
        // std::vector<double> sum_x(K, 0.0), sum_y(K, 0.0);
        // std::vector<int> count(K, 0);
        // for (int i = 0; i < x_nbins * y_nbins; ++i) {
        //     int cluster = cluster_assignments[i];
        //     if (cluster < 0) continue;
        //     int x, y;
        //     index_to_xy(i, x, y);
        //     sum_x[cluster] += x;
        //     sum_y[cluster] += y;
        //     count[cluster]++;
        // }

        // for (int k = 0; k < K; ++k) {
        //     if (count[k] > 0) {
        //         seeds[k].first  = static_cast<int>(sum_x[k] / count[k]);
        //         seeds[k].second = static_cast<int>(sum_y[k] / count[k]);
        //     }
        // }

		// STEP 2: Recompute INFORMATIVE centroids
		std::vector<double> weighted_sum_x(K, 0.0), weighted_sum_y(K, 0.0), total_weight(K, 0.0);
		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			int cluster = cluster_assignments[i];
			if (cluster < 0) continue;
			
			double weight;

			if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_DIST) 
			{
				weight = 1;
			}
			else if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_PROB)
			{
				weight = posterior.pdf[i];
			}					

			if (weight < epsilon_clusterize) continue;

			int x, y;
			index_to_xy(i, x, y);
			weighted_sum_x[cluster] += x * weight;
			weighted_sum_y[cluster] += y * weight;
			total_weight[cluster] += weight;
		}

		for (int k = 0; k < K; ++k) {
			if (total_weight[k] > epsilon_clusterize) {
				seeds[k].first  = static_cast<int>(weighted_sum_x[k] / total_weight[k]);
				seeds[k].second = static_cast<int>(weighted_sum_y[k] / total_weight[k]);
			}
		}

        // Generate again distances maps with the new centroids
        for (int k = 0; k < K; ++k) {
            std::vector<std::vector<double>> local_dist(y_nbins, std::vector<double>(x_nbins, std::numeric_limits<double>::max()));
            std::queue<std::pair<int, int>> q;

            int sx = seeds[k].first, sy = seeds[k].second;
            local_dist[sy][sx] = 0.0;
            q.push({sx, sy});

            while (!q.empty()) {
                auto [x, y] = q.front(); q.pop();
                double d = local_dist[y][x];

                for (const auto& [dx, dy] : directions) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                    if (obstacle_map_2d[ny][nx] == 1) continue;

                    double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));
                    if (local_dist[ny][nx] > d + cost) {
                        local_dist[ny][nx] = d + cost;
                        q.push({nx, ny});
                    }
                }
            }

            for (int y = 0; y < y_nbins; ++y)
                for (int x = 0; x < x_nbins; ++x)
                    dist_map[k][y * x_nbins + x] = local_dist[y][x];
        }

        // debugging purpose
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            belief_clusters[i] = (posterior.pdf[i] > epsilon_clusterize) ? cluster_assignments[i] + 1 : 0;

        log_posterior_pdf_3d_cluster_lloyd(belief_clusters);
    }

    // Cut Belief Map according to the assigned cluster
	// Assign labels to the cells (clustering)
    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);
    std::vector<std::vector<int>> pdf_masks(K, std::vector<int>(x_nbins * y_nbins, 0));

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= epsilon_clusterize) continue;
        int cluster = cluster_assignments[i];
        belief_clusters[i] = cluster + 1;
        pdf_masks[cluster][i] = 1;
    }

    log_posterior_pdf_3d_cluster(belief_clusters);

    for (int k = 0; k < K; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior.pdf[i] *= pdf_masks[k][i];

            double sum = std::accumulate(posterior.pdf, posterior.pdf + x_nbins * y_nbins, 0.0);
            if (sum > 0.0) {
                for (int i = 0; i < x_nbins * y_nbins; ++i)
                    posterior.pdf[i] /= sum;
            }

            printf("Robot ID [%d]: Sum of Lloyd-obstacle-aware pdf is %f \n", ID, sum);
        }
    }

}



// void approach1_centroid_max_pdf()
// {
//     int n = simulation_parameters.number_of_robots;
//     std::vector<std::vector<int>> top_indices_list(n);
//     std::vector<double> top_values(n, -1);
//     double epsilon = 1e-6;

//     for (int i = 0; i < x_nbins * y_nbins; i++) {

//         if (simulation_parameters.visitedList == visitedList_ON && cell_visited_list[i] != 0)
//             continue;

//         double value = posterior.pdf[i];

//         for (int j = 0; j < n; j++) {
//             if (value > top_values[j] + epsilon) {
//                 // Shift down other top values and indices
//                 for (int k = n - 1; k > j; k--) {
//                     top_values[k] = top_values[k - 1];
//                     top_indices_list[k] = top_indices_list[k - 1];
//                 }
//                 top_values[j] = value;
//                 top_indices_list[j] = {i};
//                 break;
//             } else if (fabs(value - top_values[j]) < epsilon) {
//                 top_indices_list[j].push_back(i);
//                 break;
//             }
//         }
//     }

//     std::vector<int> selected_indices(n, -1);
//     for (int j = 0; j < n; j++) {
//         if (!top_indices_list[j].empty()) {
//             selected_indices[j] = top_indices_list[j][0];
//         }
//     }

//     printf("Robot ID [%d]: Top %d maximal merged belief map values:\n", ID, n);
//     for (int j = 0; j < n; j++) {
//         printf("%d) Value: %f at index %d\n", j + 1, top_values[j], selected_indices[j]);
//     }

//     int self_index = ID - 101;
//     if (self_index >= 0 && self_index < simulation_parameters.number_of_robots) {
// 		fprintf(clustering_debugging_logfile, "Robot ID [%d]: Top %d maximal merged belief map values:\n", ID, n);
// 		for (int j = 0; j < n; j++) {
// 			fprintf(clustering_debugging_logfile, "%d) Value: %f at index %d\n", j + 1, top_values[j], selected_indices[j]);
// 		}
//     }

//     for (int j = 0; j < n; j++) {
//         if (selected_indices[j] != -1) {
//             cell_visited_list[selected_indices[j]] = 1;
//         }
//     }

//     clusterize_belief_map(selected_indices);
// }


// void approach2_centroid_curr_pos()
// {
//     std::vector<int> selected_indices(simulation_parameters.number_of_robots);

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         double x = received_current_pos[i][0];
//         double y = received_current_pos[i][1];
//         selected_indices[i] = xy_to_index(x, y);
//     }

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         cell_visited_list[selected_indices[i]] = 1;
//     }

//     printf("Robot positions and bins:\n");
//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         printf("%d) Robot ID [%d]: (%f, %f) -> Bin %d\n", i + 1, 101 + i, received_current_pos[i][0], received_current_pos[i][1], selected_indices[i]);
//     }

//     clusterize_belief_map(selected_indices);
// }


// void approach3_centroid_curr_pos_obstacle_dist()
// {
//     std::vector<int> selected_indices(simulation_parameters.number_of_robots);

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         double x = received_current_pos[i][0];
//         double y = received_current_pos[i][1];
//         selected_indices[i] = xy_to_index(x, y);
//     }

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         cell_visited_list[selected_indices[i]] = 1;
//     }

//     printf("Robot positions and bins:\n");
//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         printf("%d) Robot ID [%d]: (%f, %f) -> Bin %d\n", i + 1, 101 + i, received_current_pos[i][0], received_current_pos[i][1], selected_indices[i]);
//     }

//     clusterize_belief_map_obstacle_dist(selected_indices);
// }

// void approach4_centroid_curr_pos_lloyd()
// {
//     std::vector<int> selected_indices(simulation_parameters.number_of_robots);

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         double x = received_current_pos[i][0];
//         double y = received_current_pos[i][1];
//         selected_indices[i] = xy_to_index(x, y);
//     }

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         cell_visited_list[selected_indices[i]] = 1;
//     }

//     printf("Robot positions and bins:\n");
//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         printf("%d) Robot ID [%d]: (%f, %f) -> Bin %d\n", i + 1, 101 + i, received_current_pos[i][0], received_current_pos[i][1], selected_indices[i]);
//     }

//     clusterize_belief_map_lloyd(selected_indices);
// }

// void approach5_centroid_curr_pos_obstacle_dist_lloyd()
// {
//     std::vector<int> selected_indices(simulation_parameters.number_of_robots);

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         double x = received_current_pos[i][0];
//         double y = received_current_pos[i][1];
//         selected_indices[i] = xy_to_index(x, y);
//     }

//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         cell_visited_list[selected_indices[i]] = 1;
//     }

//     printf("Robot positions and bins:\n");
//     for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
//         printf("%d) Robot ID [%d]: (%f, %f) -> Bin %d\n", i + 1, 101 + i, received_current_pos[i][0], received_current_pos[i][1], selected_indices[i]);
//     }

//     clusterize_belief_map_obstacle_dist_lloyd(selected_indices);
// }

void cluster_algorithm()
{
    std::vector<int> selected_indices(simulation_parameters.number_of_robots);

    for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
        double x = received_current_pos[i][0];
        double y = received_current_pos[i][1];
        selected_indices[i] = xy_to_index(x, y);
    }

    for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
        cell_visited_list[selected_indices[i]] = 1;
    }

    printf("Robot positions and bins:\n");
    for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
        printf("%d) Robot ID [%d]: (%f, %f) -> Bin %d\n", i + 1, 101 + i, received_current_pos[i][0], received_current_pos[i][1], selected_indices[i]);
    }

	if (simulation_parameters.mapCoverageStrategy == PROVA) prova_3maxcells_clustering_continuo(selected_indices);
	if (simulation_parameters.mapCoverageStrategy == VORONOI) clusterize_belief_map_obstacle_dist(selected_indices);
	if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_DIST) clusterize_belief_map_obstacle_dist_lloyd(selected_indices);
	if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_PROB) clusterize_belief_map_obstacle_dist_lloyd(selected_indices);
	//if (simulation_parameters.mapCoverageStrategy == VORONOI_LLOYD_DIST) clusterize_belief_map_lloyd(selected_indices);
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
	
	if (FSM.n_cycles == 0)
	{
		initial_time_results = wb_robot_get_time();
	}

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
	//posterior.log_pdf(posterior_pdf_3D_name, 'a');

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
	// if (VERBOSE)
	// 	printf("[%d] entropy : %f\n", ID, posterior.entropy);

	char label[255];
	sprintf(label, "Iteration: %d\n", FSM.n_cycles);
	//wb_supervisor_set_label(2, label, 0.1, 0.8, 0.1, 0x61A8E8, 0, "Impact");
	char label2[255];
	sprintf(label2, "Reading: %f\n", sampleBuffer.average_concentration);
	//wb_supervisor_set_label(1, label2, 0.1, 0.85, 0.1, 0xFF0000, 0, "Impact");
	char label3[255];
	sprintf(label3, "Entropy: %.3f\n", posterior.entropy);
	//wb_supervisor_set_label(3, label3, 0.1, 0.9, 0.1, 0xFFD966, 0, "Impact");

	long long current_estimation_time = current_timestamp() - time_before_estimating;
	// printf("Current estimation time is : %lld\n", current_estimation_time);

	estimating_time_sum += current_estimation_time;
	// printf("estimating_time_sum : %lld\n", estimating_time_sum);

	// check if stopping condition is reached
	if ((simulation_parameters.number_of_robots == 1)||(simulation_parameters.number_of_robots == 1))
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
			double end_time_results = wb_robot_get_time();
			double simulation_time_results = end_time_results - initial_time_results;
			log_time(simulation_time_results);

			FSM.state = STATE_FINISHED;
			return;
		} else {
			FSM.state = STATE_UPDATEGOAL;
			FSM.hook = state_updategoal;
		}
	}
	else if (simulation_parameters.controlArchitecture == CENTRALIZED)
	{
		if (ID == CENTRAL_MASTER_ID)
		{
			FSM.state = STATE_RECEIVE_PDF;
			FSM.hook = state_receive_pdf;
		}
		else
		{
			FSM.state = STATE_SEND_PDF;
			FSM.hook = state_send_pdf;
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
		log_kld_value();
		log_whole_position_history(true);
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
		case COMBINED:
			// Cicla entre 1, 2, 3
			n_combine_pathplanning++;

			// if (n_combine_pathplanning % 2 == 0)
			// {
			// 	path_planner.information_center_planning();
			// }
			// else
			// {
			// 	path_planner.accumulative_Dijkstra();
					
			// }	
			if (FSM.n_cycles == 1)
			{
				path_planner.farthest_belief_point_planning();
			}
			else if (n_combine_pathplanning % 2 == 0)
			{
				path_planner.belief_clustering_planning();
			}
			else
			{
				path_planner.information_center_planning();
			}
			break;	
		case GREEDY:
			if (simulation_parameters.mapCoverageStrategy == NONE && simulation_parameters.number_of_robots > 1)
			{
				path_planner.debug_relative_value_maximal_merged_belief_planning();
			}
			else if (simulation_parameters.mapCoverageStrategy != NONE || simulation_parameters.number_of_robots == 1)
			{
				path_planner.maximal_belief_planning();
			}		
			break;
		case INFO_CENTER:
			path_planner.information_center_planning();
			break;
		case FARTHEST_CELL:
			path_planner.farthest_belief_point_planning();
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
			// The goal position is very close to current postion, swith to random planning
			// if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<double>(simulation_parameters.sampling_dist_thresh)))
			// {
			// 	printf("The goal position is very close to current postion, switch to random planning \n");
			// 	fprintf(console_logfile, "The goal position is very close to current postion, switch to random planning \n");
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
				for (unsigned k = 0; k < Polyline_path.size(); k++)
				{
					Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
					path.push_back(p);
					log_trajectory(p);
				}
			}
			i = path.begin();
			// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
			// send the goal to the supervisor
			//if (kld_subloop_upd) log_position();
			//if (kld_subloop_upd) posterior.log_pdf(posterior_pdf_3D_name, 'a');
			//if (kld_subloop_upd) log_posterior_pdf_3d_cluster();
			
			Message msg(ID, 0, goal_position, -1, goal_position, 0);
			send_msg(msg);
			// if online simulation: move to the goal position
			// if offline simulation: read the next goal position and measurements from the file
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
	//double start_state_moving = wb_robot_get_time();

	//double substates_time = start_state_moving - start_substates;
	//fprintf(KLD_debugging_logfile, "Time in SUBSTATES was %f seconds \n", substates_time);
	kld_subloop_upd = false;
	kld_replan = 0;

	if (!entered_state_moving)
	{
		printf("Robot ID [%d]: STATE_MOVING \n", ID);

		// this one doesnt make sense because it enters again in state_moving quite fast (0.288 sec) but forces waiting for n_iter=frequency or distance to the goal < 0.05
		// double time_now = wb_robot_get_time();
		// double time_between_movements = time_now - time_after_moving;
		// printf("Robot ID [%d]: Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements);

		entered_state_moving = true;
	}


	bool is_centralized = (simulation_parameters.controlArchitecture == CENTRALIZED && ID == CENTRAL_MASTER_ID);
	bool is_distributed = simulation_parameters.controlArchitecture == DISTRIBUTED;

	if ((is_distributed && simulation_parameters.increaseFreqBM == NO) || simulation_parameters.number_of_robots == 1)
	{
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
			log_whole_position_history(true);

			// int update_goal_iterations = static_cast<int>(round(simulation_parameters.update_time * 1000.0 / TIME_STEP));

			// if (simulation_parameters.updateGoalPolicy == FIXED_FREQ){
			// 	if (n_time_step % update_goal_iterations == 0 && n_time_step != 0)
			// 		{
			// 		double time_now_time_step_far = wb_robot_get_time();
			// 		double time_between_movements_time_step_far = time_now_time_step_far - time_after_moving;
			// 		printf("Robot ID [%d]: Far from goal. Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements_time_step_far);
			// 		fprintf(console_logfile, "Robot ID [%d]: Far from goal. Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements_time_step_far);
			// 		// set_wheels_speed(0, 0);
			// 		sampleBuffer.reset_all();
			// 		path.clear();
			// 		// printf("Robot ID [%d]. Number of iteration is %d\n", ID, n_time_step);
			// 		FSM.state = STATE_MEASURING;
			// 		FSM.hook = state_measuring;
			// 		time_after_moving = wb_robot_get_time();
			// 		entered_state_moving = false;
			// 		return;
			// 	}
			// }
		}
		else 
		{
			double time_now_time_step_near = wb_robot_get_time();
			double time_between_movements_time_step_near = time_now_time_step_near - time_after_moving;
			printf("Robot ID [%d]: Near from goal. Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements_time_step_near);
			fprintf(console_logfile, "Robot ID [%d]: Near from goal. Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements_time_step_near);
			set_wheels_speed(0, 0);
			sampleBuffer.reset_all();
			path.clear();
			// back to measuring
			FSM.state = STATE_MEASURING;
			FSM.hook = state_measuring;
			time_after_moving = wb_robot_get_time();
			navigation_sampling_time_sum += time_after_moving - time_before_moving;
			time_before_measuring = wb_robot_get_time();
			printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
			entered_state_moving = false;
		}

	}
	else if (is_distributed && simulation_parameters.increaseFreqBM == YES)
	{
		// ====== SAMPLING ======
		// printf("moving toward the goal\n");
		if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
		{
			sample_counter++;
			n_meas++;
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

		double dist_threshold_movesting = 0.05;
		distance_to_goal = dist_to_goal(goal_position);
		if (dist_to_goal(goal_position) > dist_threshold_movesting)
		{
			if (i == path.end())
			{
				fprintf(console_logfile, "path end goal reached \n");
			}

			// move
			if (dist_to_goal(*i) <= dist_threshold_movesting && i != path.end())
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
			log_whole_position_history(true);
		}
		else
		{
			// double time_now_time_step_near = wb_robot_get_time();
			// double time_between_movements_time_step_near = time_now_time_step_near - time_after_moving;
			// printf("Robot ID [%d]: Near from goal. Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements_time_step_near);
			// fprintf(console_logfile, "Robot ID [%d]: Near from goal. Time outside STATE_MOVING: %f seconds\n", ID, time_between_movements_time_step_near);
			set_wheels_speed(0, 0);

			sampleBuffer.reset_all();
			path.clear();
			// back to measuring
			// FSM.state = STATE_MEASURING;
			// FSM.hook = state_measuring;
			// time_after_moving = wb_robot_get_time();
			// navigation_sampling_time_sum += time_after_moving - time_before_moving;
			// time_before_measuring = wb_robot_get_time();
			// printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
			// entered_state_moving = false;
			FSM.state = STATE_SHARE_DIST;
			FSM.hook = state_share_dist;
			return;
		}
		
		// 
		int compare_pdf_iterations = static_cast<int>(round(simulation_parameters.update_time * 1000.0 / TIME_STEP));
		// 
		if ((n_time_step % compare_pdf_iterations == 0 && n_time_step != 0))
		{
			//start_substates = wb_robot_get_time();

			FSM.state = STATE_SHARE_DIST;
			FSM.hook = state_share_dist;
			return;
		}
	}


}

void state_share_dist()
{

    printf("Robot ID [%d]: STATE_SHARE_DIST\n", ID);
	fprintf(clustering_debugging_logfile, "Robot ID [%d]: Entered STATE_SHARE_DIST\n", ID);


    if (posterior.pdf == NULL) {
        printf("Robot ID [%d]: No posterior PDF to share.\n", ID);
    } else {
        for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
            int target_ID = 101 + i;
            if (target_ID == ID) continue; // skip self

			// save my current distance to the goal
			received_current_dist[self_index] = distance_to_goal;

			// share my current distance to the goal
            Message msg(ID, target_ID, Position(), distance_to_goal, Position(), 0);
            send_msg(msg);

            printf("Robot ID [%d]: Sent distance to Robot ID [%d]\n", ID, target_ID);
        }
    }

	FSM.state = STATE_RECEIVE_DIST;
	FSM.hook = state_receive_dist;
}

void state_receive_dist()
{

	if (!entered_state_receive_dist)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_DIST \n", ID);
		fprintf(clustering_debugging_logfile, "Robot ID [%d]: Entered STATE_RECEIVE_DIST\n", ID);
		entered_state_receive_dist = true;
	}

	while (wb_receiver_get_queue_length(receiver) > 0)
	{
		const void *data = wb_receiver_get_data(receiver);
		const double *buffer = (const double *)data;
		int sender_ID = (int)buffer[0];
		int message_type = (int)buffer[1];
		double received_current_dist_teammate = buffer[5];

		int sender_index = sender_ID - 101;

		if (message_type == MSG_STATE_SHARE_DIST)
		{
			ready_from[sender_index] = 1;
			printf("Robot ID [%d]: Received READY from Robot ID [%d]\n", ID, sender_ID);
			wb_receiver_next_packet(receiver);
			continue;
		}

		if (sender_ID != ID && sender_index >= 0 && sender_index < MAX_NB_ROBOTS)
		{

			received_current_dist[sender_index] = received_current_dist_teammate;
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
		Message msg(ID, MSG_STATE_SHARE_DIST, Position(), -1, Position(), 1);
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

		bool all_ready = true;
		for (int i = 0; i < simulation_parameters.number_of_robots; i++)
		{
			if (received_current_dist[i] > 0.05) {
				all_ready = false;
				break;
			}
		}

		if (all_ready)
		{
			printf("Robot ID [%d]: All robots READY. Transitioning to STATE_MEASURING.\n", ID);
			FSM.state = STATE_MEASURING;
			FSM.hook = state_measuring;
		}
		else
		{
			printf("Robot ID [%d]: All robots READY. Transitioning to STATE_MOVESTING.\n", ID);
			FSM.state = STATE_MOVESTING;
			FSM.hook = state_movesting;
		}
	}

}

void state_movesting()
{



	if (!movesting_initialized)
	{
		FSM_Sub.substate = SUBSTATE_MEASURING;
		FSM_Sub.hook = substate_measuring;

		FSM.subfsm = &FSM_Sub; // remember that in orthe main FSM we declared subfsm as a pointer to the struct FSM_SUb (generic)
							// then, here we introduce the adress of the actual concrete FSM_Sub which is being used on this code	

		movesting_initialized = true;	
	}

	// Ejecute sub-FSM
	if (FSM.subfsm && FSM.subfsm->hook) {
		FSM.subfsm->hook();  // Executes the logic of the current state (which is SUBSTATE_MEASURING!!!)
	}

	// FSM.state = SUBSTATE_ESTIMATING;
	// FSM.hook = substate_estimating;
}

void substate_measuring()
{

	printf("Robot ID [%d]: STATE_MOVESTING\n", ID);
	printf("Robot ID [%d]: Checking comparison between PDFs after %d seconds (%i iterations, %i measurements)\n", ID, simulation_parameters.update_time, compare_pdf_iterations, history.size);
	fprintf(KLD_debugging_logfile, "FSM cycle: %i\n", FSM.n_cycles);
	fprintf(KLD_debugging_logfile, "Robot ID [%d]: Checking comparison between PDFs after %d seconds (%i iterations, %i measurements)\n", ID, simulation_parameters.update_time, compare_pdf_iterations, history.size);


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
		fprintf(KLD_debugging_logfile, "Robot ID [%d]: Gathered %ld measurements while moving!\n", ID, sample_in_motion.measurement.size());
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
    FSM.subfsm->substate = SUBSTATE_ESTIMATING;
    FSM.subfsm->hook = substate_estimating;

}

void substate_estimating()
{
	printf("Robot ID [%d]: SUBSTATE_ESTIMATING \n", ID);


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

    FSM.subfsm->substate = SUBSTATE_SHARE_PDF;
    FSM.subfsm->hook = substate_share_pdf;

	// FSM.state = SUBSTATE_SHARE_PDF;
	// FSM.hook = substate_share_pdf;
}

void substate_share_pdf()
{
	printf("Robot ID [%d]: SUBSTATE_SHARE_PDF \n", ID);


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

    FSM.subfsm->substate = SUBSTATE_RECEIVE_PDF;
    FSM.subfsm->hook = substate_receive_pdf;

	// FSM.state = SUBSTATE_RECEIVE_PDF;
	// FSM.hook = substate_receive_pdf;
}

void substate_receive_pdf()
{



	if (!entered_substate_receive_pdf)
	{
		printf("Robot ID [%d]: SUBSTATE_RECEIVE_PDF \n", ID);

		entered_substate_receive_pdf = true;
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

		if (message_type == MSG_SUBSTATE_SHARE_PDF)
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
		Message msg(ID, MSG_SUBSTATE_SHARE_PDF, Position(), -1, Position(), 1);
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

		FSM.subfsm->substate = SUBSTATE_MERGE_PDF;
		FSM.subfsm->hook = substate_merge_pdf;
				
		// FSM.state = SUBSTATE_MERGE_PDF;
		// FSM.hook = substate_merge_pdf;

	}

}

void substate_merge_pdf()
{
	printf("Robot ID [%d]: SUBSTATE_MERGE_PDF \n", ID);


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

	memcpy(posterior.pdf, belief_map_merged, MAP_SIZE * sizeof(double));
	posterior.process();
    posterior.calculate_entropy_and_spread();


	// Check if Merged Belief Map equal 0
	double sum_merged_pdf_substate = 0.0;
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        sum_merged_pdf_substate += posterior.pdf[i];
    }

	bool is_stop_condition_reached = check_stop_condition();
	if (is_stop_condition_reached)
	{
		// Avoid STATE_FINISHED if Merged Belief Map equal 0
		if (sum_merged_pdf_substate > 0)
		{
			log_position();
			//log_kld_value();
			log_whole_position_history(true);

			double end_time_results = wb_robot_get_time();
			double simulation_time_results = end_time_results - initial_time_results;
			log_time(simulation_time_results);

			broadcast_state_finished();

			FSM.state = STATE_FINISHED;
			printf("Robot ID [%d]: STATE_FINISHED inside of KLD_SUBLOOP\n", ID);
			fprintf(KLD_debugging_logfile, "Robot ID [%d]: STATE_FINISHED inside of KLD_SUBLOOP\n", ID);
			return;
		}
		else
		{
			printf("Robot ID [%d]: Avoided STATE_FINISHED inside of KLD_SUBLOOP because Merged Belief Map = 0\n", ID);
			fprintf(KLD_debugging_logfile, "Robot ID [%d]: Avoided STATE_FINISHED  inside of KLD_SUBLOOP because Merged Belief Map = 0\n", ID);
			FSM.state = STATE_MOVING;
			FSM.hook = state_moving;
			return;
		}
	}
	else
	{

		FSM.subfsm->substate = SUBSTATE_COMPARE_PDF;
		FSM.subfsm->hook = substate_compare_pdf;
				

		// FSM.state = SUBSTATE_COMPARE_PDF;
		// FSM.hook = substate_compare_pdf;
	}


}

void substate_compare_pdf()
{
	printf("Robot ID [%d]: SUBSTATE_COMPARE_PDF \n", ID);

	n_meas = 0;

	n_kld++;
	KLD = abs(posterior.KLD_BM_Update(actual_belief_map, actual_sum));
	fprintf(KLD_debugging_logfile, "Robot [%d]: KLD #: %i\n", ID, n_kld);
	fprintf(KLD_debugging_logfile, "Robot [%d]: KLD Value: %f\n", ID, KLD);

	std::vector<std::pair<int, int>> maximum_values_actual_belief = find_top_3(actual_belief_map);
	std::vector<std::pair<int, int>> maximum_values_proposed_pdf = find_top_3(posterior.pdf);

	fprintf(KLD_debugging_logfile, "Robot ID [%d]: Gathered %ld measurements while moving!\n", ID, sample_in_motion.measurement.size());


	if (ID==101)
	{
		printf("Robot ID [%d]: KLD COMPARISON Between belief maps corresponding to:\n", ID);
		printf("Robot ID [%d]: ACTUAL BELIEF MAP:\n", ID);
		for (int i = 0; i < maximum_values_actual_belief.size(); ++i) {
			int row = maximum_values_actual_belief[i].first;
			int col = maximum_values_actual_belief[i].second;
			printf("Top %d: row = %d, column = %d\n", i + 1, row, col);
		}
		printf("Robot ID [%d]: PROPOSED PDF:\n", ID);
		for (int i = 0; i < maximum_values_proposed_pdf.size(); ++i) {
			int row = maximum_values_proposed_pdf[i].first;
			int col = maximum_values_proposed_pdf[i].second;
			printf("Top %d: row = %d, column = %d\n", i + 1, row, col);
		}
	}
	fprintf(KLD_debugging_logfile, "=========================================\n");
	fprintf(KLD_debugging_logfile, "Actual PDF:\n");
	for (int i = 0; i < maximum_values_actual_belief.size(); ++i) {
		int row = maximum_values_actual_belief[i].first;
		int col = maximum_values_actual_belief[i].second;
		fprintf(KLD_debugging_logfile, "Top %d: row = %d, column = %d\n", i + 1, row, col);
	}
	fprintf(KLD_debugging_logfile, "Proposed PDF:\n");
	for (int i = 0; i < maximum_values_proposed_pdf.size(); ++i) {
		int row = maximum_values_proposed_pdf[i].first;
		int col = maximum_values_proposed_pdf[i].second;
		fprintf(KLD_debugging_logfile, "Top %d: row = %d, column = %d\n", i + 1, row, col);
	}	
	fprintf(KLD_debugging_logfile, "=========================================\n");


	//log_kld_value();


	FSM.subfsm->substate = SUBSTATE_SHARE_KLD;
	FSM.subfsm->hook = substate_share_kld;
				
	// FSM.state = SUBSTATE_SHARE_KLD;
	// FSM.hook = substate_share_kld;
}

void substate_share_kld()
{

	printf("Robot ID [%d]: SUBSTATE_SHARE_KLD \n", ID);



    if (posterior.pdf == NULL) {
        printf("Robot ID [%d]: No posterior PDF to share.\n", ID);
    } else {
        for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
            int target_ID = 101 + i;
            if (target_ID == ID) continue; // skip self

			// save my current distance to the goal
			received_current_kld[self_index] = KLD;

			// share my current distance to the goal
            Message msg(ID, target_ID, Position(), KLD, Position(), 0);
            send_msg(msg);

            printf("Robot ID [%d]: Sent KLD to Robot ID [%d]\n", ID, target_ID);
        }
    }

	FSM.state = SUBSTATE_RECEIVE_KLD;
	FSM.hook = substate_receive_kld;
}

void substate_receive_kld()
{



	if (!entered_state_receive_kld)
	{
		printf("Robot ID [%d]: SUBSTATE_RECEIVE_KLD \n", ID);
		entered_state_receive_kld = true;
	}

	while (wb_receiver_get_queue_length(receiver) > 0)
	{
		const void *data = wb_receiver_get_data(receiver);
		const double *buffer = (const double *)data;
		int sender_ID = (int)buffer[0];
		int message_type = (int)buffer[1];
		double received_current_kld_teammate = buffer[5];

		int sender_index = sender_ID - 101;

		if (message_type == MSG_SUBSTATE_SHARE_KLD)
		{
			ready_from[sender_index] = 1;
			printf("Robot ID [%d]: Received READY from Robot ID [%d]\n", ID, sender_ID);
			wb_receiver_next_packet(receiver);
			continue;
		}

		if (sender_ID != ID && sender_index >= 0 && sender_index < MAX_NB_ROBOTS)
		{

			received_current_kld[sender_index] = received_current_kld_teammate;
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
		Message msg(ID, MSG_SUBSTATE_SHARE_KLD, Position(), -1, Position(), 1);
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

		entered_state_moving = false;

		fprintf(KLD_debugging_logfile, "Robot ID [%d]: Received KLDs: %f %f %f\n", ID, received_current_kld[0], received_current_kld[1], received_current_kld[2]);
		if (abs(received_current_kld[self_index]) > simulation_parameters.KLD_thresh)
		{
			fprintf(KLD_debugging_logfile, "Robot ID [%d]: Forcing transition to REGION_ALLOCATION since KLD > %f\n",ID, simulation_parameters.KLD_thresh);


			// Keep the current BM
			std::memcpy(actual_belief_map, posterior.pdf, MAP_SIZE * sizeof(double));
			actual_sum = 0.0;
			for (int i = 0; i < MAP_SIZE; ++i)
			actual_sum += actual_belief_map[i];


			set_wheels_speed(0, 0);
			sampleBuffer.reset_all();
			path.clear();

			//FSM.n_cycles++;
			kld_subloop_upd = true;

			kld_replan = 1;
			log_kld_value();


			// Clean sub-FSM
    		FSM.subfsm = nullptr;  
			movesting_initialized = false;

			FSM.state = STATE_REGION_ALLOCATION;
			FSM.hook = state_region_allocation;
		} 
		else
		{
			kld_replan = 0;
			log_kld_value();


			// Clean sub-FSM
    		FSM.subfsm = nullptr;  
			movesting_initialized = false;

			FSM.state = STATE_MOVING;
			FSM.hook = state_moving;
		}	
	}
}

void state_obstacle_avoidance()
{
	// printf("avoiding obstacle\n");
	obstacle_avoidance_k4();
}

void state_send_goal()
{
	printf("Robot ID [%d]: STATE_SEND_GOAL \n", ID);

	if (FSM.state != STATE_FINISHED)
	{
		// The goal position is very close to current postion, swith to random planning
		// if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<double>(simulation_parameters.sampling_dist_thresh)))
		// {
		// 	printf("The goal position is very close to current postion, switch to random planning \n");
		// 	fprintf(console_logfile, "The goal position is very close to current postion, switch to random planning \n");
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
		for (unsigned k = 0; k < Polyline_path.size(); k++)
		{
			Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
			path.push_back(p);
			log_trajectory(p);
		}
		i = path.begin();
		// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		// send the goal to the supervisor
		Message msg(ID, 0, goal_position, -1, goal_position, 0);
		send_msg(msg);

		for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
			int target_ID = 101 + i;
			if (target_ID == ID) continue;  // Skip self

			Message msg(ID, target_ID, goal_position_team[i], -1, goal_position_team[i], 0);
			send_msg(msg);
		}

		// move to the goal position
		if (simulation_parameters.simuMode == ONLINE)
		{
			// update goal
			FSM.state = STATE_MOVING;
			FSM.hook = state_moving;
			time_before_moving = wb_robot_get_time();
		}
		else
		{
			FSM.state = STATE_READ_ALL;
			FSM.hook = state_read_positions_measurements;
		}
	}
}

void state_receive_goal()
{

	if (!entered_state_receive_goal)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_GOAL \n", ID);
		entered_state_receive_goal = true;
	}

	bool goal_received = false;

	// message queue lenght
	int queue_length = wb_receiver_get_queue_length(receiver);
	// printf("Robot ID [%d]: Receiver queue length = %d\n", ID, queue_length);

	if (queue_length == 0)
		return;

	while (wb_receiver_get_queue_length(receiver) > 0)
	{
		const void *data = wb_receiver_get_data(receiver);
		const double *buffer = (const double *)data;

		// link sender and receiver with buffer
		int sender_ID = (int)buffer[0];
		int receiver_ID = (int)buffer[1];
		int flag_finished = (int)buffer[9];
		// printf("Robot ID [%d]: Checking message - sender_ID: %d, receiver_ID: %d\n", ID, sender_ID, receiver_ID);

		// check if state_finished
		if (flag_finished == -1)
		{
			printf("Robot ID [%d]: Imposing STATE_FINISHED as requested from robot %d!\n", ID, sender_ID);
			FSM.state = STATE_FINISHED;
			printf("Robot ID [%d]: STATE_FINISHED\n", ID);
			return;
		}

		// check if message comes from robot 101
		if (sender_ID == CENTRAL_MASTER_ID)
		{
			if (receiver_ID == ID)
			{ // this message is for this robot?
				// printf("Robot ID [%d]: Valid goal received from sender_ID %d!\n", ID, sender_ID);

				// assign the goal to the robot specified in the receiver ID
				goal_position.x = buffer[6];
				goal_position.y = buffer[7];
				goal_position.heading = buffer[8];

				// print new goal position
				// printf("Robot ID [%d]: Set goal_position = %f %f\n", ID, goal_position.x, goal_position.y);

				goal_received = true;
			}
		}

		// delete current message and go to check the next message in the queue
		wb_receiver_next_packet(receiver);

		// if goal position for some robot already received, go out from the while
		if (goal_received)
			break;
	}

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

	for (unsigned k = 0; k < Polyline_path.size(); k++)
	{
		Position p(Polyline_path[k].x(), Polyline_path[k].y(), 0);
		path.push_back(p);
		log_trajectory(p);
	}

	i = path.begin();

	// we send the msg containing the supervisor to visulize the goal positions
	Message msg(ID, 0, goal_position, -1, goal_position, 0);
	send_msg(msg);

	if (simulation_parameters.simuMode == ONLINE)
	{
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		entered_state_receive_goal = true;
		time_before_moving = wb_robot_get_time();
	}
	else
	{
		FSM.state = STATE_READ_ALL;
		FSM.hook = state_read_positions_measurements;
		entered_state_receive_goal = true;
	}
}

void state_send_pdf()
{
	printf("Robot ID [%d]: STATE_SEND_PDF \n", ID);

	if (posterior.pdf != nullptr)
	{
		// printf("Robot ID [%d]: Belief map ready to be sent.\n", ID);
		Message msg_send(ID, CENTRAL_MASTER_ID, Position(), -1, Position(), 0, posterior.pdf);
		send_msg(msg_send);
		// printf("Robot ID [%d]: Sending belief map to STATE_RECEIVE_PDF.\n", ID);

		FSM.state = STATE_RECEIVE_GOAL;
		FSM.hook = state_receive_goal;
	}
}

void state_receive_pdf()
{
	if (!entered_state_receive_pdf)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_PDF \n", ID);
		entered_state_receive_pdf = true;
	}

	// if the team strategy is centralized, then the robot 101 receives the belief maps from the other robots
	// if the team strategy is distributed, then all robots receive the belief maps from each other
	bool is_centralized = (simulation_parameters.controlArchitecture == CENTRALIZED && ID == CENTRAL_MASTER_ID);
	bool is_distributed = simulation_parameters.controlArchitecture == DISTRIBUTED;

	if (is_centralized)
	{
		if (!printed && posterior.pdf != nullptr)
		{
			memcpy(belief_maps[self_index], posterior.pdf, MAP_SIZE * sizeof(double));
			printed = true;
		}

		int queue_length = wb_receiver_get_queue_length(receiver);
		// printf("Robot ID [%d]: Receiver queue length = %d\n", ID, queue_length);

		while (wb_receiver_get_queue_length(receiver) > 0)
		{
			const void *data = wb_receiver_get_data(receiver);
			const double *buffer = (const double *)data;

			int sender_ID = (int)buffer[0];
			int receiver_ID = (int)buffer[1];
			int flag_finished = (int)buffer[9];

			printf("Robot ID [%d]: Received message - sender_ID: %d, receiver_ID: %d\n", ID, sender_ID, receiver_ID);

			int sender_index = sender_ID - 101;

			// Skip self and invalid indices
			if (sender_ID != ID && sender_index >= 0 && sender_index < MAX_NB_ROBOTS)
			{
				memcpy(belief_maps[sender_index], &buffer[10], MAP_SIZE * sizeof(double));
				received_flags[sender_index] = 1;
			}

			wb_receiver_next_packet(receiver);
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

	if (is_distributed)
	{
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


	// // debugging purpose
// 	fprintf(clustering_debugging_logfile, "\nRobot ID [%d] Belief Maps (cycle %d):\n", ID, FSM.n_cycles);
// 	fprintf(clustering_debugging_logfile, "Robot 101 belief map:\n");
// 	for (int i = 0; i < MAP_SIZE; i++)
// 		fprintf(clustering_debugging_logfile, "%.8f ", belief_maps[0][i]);
// 	fprintf(clustering_debugging_logfile, "\nRobot 102 belief map:\n");
// 	for (int i = 0; i < MAP_SIZE; i++)
// 		fprintf(clustering_debugging_logfile, "%.8f ", belief_maps[1][i]);
// 	fprintf(clustering_debugging_logfile, "\nRobot 103 belief map:\n");
// 	for (int i = 0; i < MAP_SIZE; i++)
// 		fprintf(clustering_debugging_logfile, "%.8f ", belief_maps[2][i]);
// 	fprintf(clustering_debugging_logfile, "\n");

	//debugging purpose
	//fprintf(clustering_debugging_logfile, "FSM iter: %d \n", FSM.n_cycles);
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

	if (FSM.n_cycles == 1 && simulation_parameters.increaseFreqBM == YES && simulation_parameters.controlArchitecture == DISTRIBUTED)
	{
		memcpy(actual_belief_map, belief_map_merged, MAP_SIZE * sizeof(double));
		for (int i = 0; i < MAP_SIZE; ++i)
			actual_sum += actual_belief_map[i];
	}

	//posterior.log_pdf(posterior_pdf_3D_name, 'a');

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
			log_kld_value();
			log_whole_position_history(true);
			posterior.log_pdf(posterior_pdf_3D_name, 'a');

			double end_time_results = wb_robot_get_time();
			double simulation_time_results = end_time_results - initial_time_results;
			log_time(simulation_time_results);

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
		if (simulation_parameters.controlArchitecture == CENTRALIZED)
		{
				FSM.state = STATE_UPDATEGOAL_TEAM;
				FSM.hook = state_updategoal_team;

		}
		else if (simulation_parameters.controlArchitecture == DISTRIBUTED && simulation_parameters.mapCoverageStrategy != NONE)
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

	if (simulation_parameters.controlArchitecture == DISTRIBUTED)
	{

		// debugging purpose
			fprintf(clustering_debugging_logfile, "Robot ID [%d]: Received positions \n", ID);
			fprintf(clustering_debugging_logfile, "received_pos_101: (%.2f, %.2f)\n",
					received_current_pos[0][0], received_current_pos[0][1]);
			fprintf(clustering_debugging_logfile, "received_pos_102: (%.2f, %.2f)\n",
					received_current_pos[1][0], received_current_pos[1][1]);
			fprintf(clustering_debugging_logfile, "received_pos_103: (%.2f, %.2f)\n",
					received_current_pos[2][0], received_current_pos[2][1]);

		log_position();
		//log_kld_value();
		log_whole_position_history(true);
		posterior.log_pdf(posterior_pdf_3D_name, 'a');

		cluster_algorithm();

		//FSM.n_cycles++;

		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;
	}
}





// =========================== MAIN ============================== //
int main(int argc, char **argv)
{
	init();
	read_obstacle_list();
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
	// char label[255];
	// sprintf(label, "Iteration: %d\n", 0);
	// wb_supervisor_set_label(2, label, 0.1, 0.8, 0.1, 0x61A8E8, 0, "Impact");
	// char label2[255];
	// sprintf(label2, "Reading: %d\n", 0);
	// wb_supervisor_set_label(1, label2, 0.1, 0.85, 0.1, 0xFF0000, 0, "Impact");

	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	plume_model.initialize_potential_gas_maps();
	// plume_model.log_gas_concentration_map(gas_concentration_map_logfile_name, 'a');

	// File for saving console messages
	console_logfile = fopen(console_logfile_name, "a");

	// debugging purpose
	clustering_debugging_logfile = fopen(clustering_debugging_name, "a");
	KLD_debugging_logfile = fopen(KLD_debugging_name, "a");
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
		n_time_step++;
	}

	fclose(console_logfile);

	// debugging purpose
	fclose(clustering_debugging_logfile);

	fclose(KLD_debugging_logfile);
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
