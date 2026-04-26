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
#include <webots/supervisor.h>

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

// simulation parameters
SimulationParameters simulation_parameters("../../data/config.yaml");

// Declare a pointer to the base class or common interface for plume models
PlumeModel* plume_model = nullptr;

// current posterior distribution
// Posterior posterior(plume_model);
Posterior* posterior= nullptr;
PathPlanner* path_planner = nullptr;
// robot's ID
int ID;
int self_index;

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

// real source position
double real_source_pos[3] = {0, 0, 0};

bool use_visibility_path = true;

// multi-robot belief sharing
// store the belief maps of the whole team
std::vector<std::vector<double>> belief_maps;
std::vector<int> received_flags;
// merged belief map
std::vector<double> belief_map_merged;
static float received_current_pos[MAX_NB_ROBOTS][2] = {0};
static int i_am_ready = 0;
static int ready_from[MAX_NB_ROBOTS] = {0};
static int belief_clusters[x_nbins*y_nbins] = {0};

//FIXME: Temporary put it here 

#define MSG_STATE_SHARE_PDF -99

// visited list
std::vector<int> cell_visited_list;

bool printed = false;
bool entered_state_receive_pdf = false;
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

// calculate wheel speeds to go toward goal position
void go_to_goal(Position goal)
{
	
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
	printf("_____ Sample History _____ \n");
	for (int i = 0; i < history.size; i++)
	{
		printf("%d : %f @ (%f, %f)\n", i, history.gas_meas[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list()
{
	// open file
	std::ifstream infile("../../data/obstacle_list.txt");
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
	double time = wb_robot_get_time();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile, "%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(int n_cycle, float x, float y, double wind_u, double wind_v, double concentration)
{
	// int i = 0;
	double time = wb_robot_get_time();
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
	double time = get_exp_time();
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

	if (counter > 400)
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
		history.gas_meas.push_back(f4);
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
		int idx = position_to_index_2D(goal_position.x, goal_position.y);
		cell_visited_list[idx] = 1;
		// printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);

		while (current_position.dist_to_goal(goal_position) > 0.05)
		{
			fscanf(experiment_measurements_logfile, "%d, %f, %f, %d", &d, &f4, &f5, &iter);
			history.size++;
			history.gas_meas.push_back(f4);
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
				history.gas_meas.push_back(sample_from_map[i]);
			else
				history.gas_meas.push_back(sample_in_motion.gas_meas[i]);
			// store wind measurements
			history.wind_meas_u.push_back(sample_in_motion.wind_meas_u[i]);
			history.wind_meas_v.push_back(sample_in_motion.wind_meas_v[i]);

			history.FSM_iter.push_back(FSM.n_cycles);
			history.size++;
			// mark the cell as visited
			if (simulation_parameters.visitedList == visitedList_ON)
			{
				int idx = position_to_index_2D(sample_in_motion.position[i].x, sample_in_motion.position[i].y);
				// do not mark the cells passed during motion as visited; only mark the final goal position
				//cell_visited_list[idx] = 1;
			}
			log_measurements(FSM.n_cycles, sample_in_motion.position[i].x, sample_in_motion.position[i].y,
							 sample_in_motion.wind_meas_u[i], sample_in_motion.wind_meas_v[i], sample_in_motion.gas_meas[i]);
		}

		sample_in_motion.position.clear();
		sample_in_motion.gas_meas.clear();
		sample_in_motion.wind_meas_u.clear();
		sample_in_motion.wind_meas_v.clear();
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
			wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle, &sampleBuffer.instant_wind_vx, &sampleBuffer.instant_wind_vy, &sampleBuffer.instant_wind_vz);
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
		wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle, &sampleBuffer.instant_wind_vx, &sampleBuffer.instant_wind_vy, &sampleBuffer.instant_wind_vz);
		// add samples to buffers
		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
	}

	history.position.push_back({current_position.x, current_position.y, 0});
	history.gas_meas.push_back(sampleBuffer.average_concentration);
	history.wind_meas_u.push_back(sampleBuffer.average_wind_vx);
	history.wind_meas_v.push_back(sampleBuffer.average_wind_vy);
	history.FSM_iter.push_back(FSM.n_cycles);
	history.size++;
	//history_print(history);
	log_measurements(FSM.n_cycles, current_position.x, current_position.y,
						sampleBuffer.average_wind_vx, sampleBuffer.average_wind_vy, sampleBuffer.average_concentration);
	// mark the current cell as visited
	if (simulation_parameters.visitedList == visitedList_ON)
	{
		int idx = position_to_index_2D(current_position.x, current_position.y);
		fprintf(console_logfile, "mark cell %d as visited\n", idx);
		cell_visited_list[idx] = 1;
		fprintf(console_logfile, "mark cell %d, position (%f, %f) as visited\n", idx, current_position.x, current_position.y);
	}
	// log and save
	if (VERBOSE)
		printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
	log_position();

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

	printf("sum posterior : %f \n", posterior->sum);
	posterior->normalize();
	printf("sum posterior after normalization : %f \n", posterior->sum);
	posterior->log_pdf(posterior_pdf_3D_name, 'a');
	if (simulation_parameters.gasFeature == RELATIVE_VALUE)
		posterior->log_relative_history_meas(relative_history_meas_logfile_name, 'a', history.relative_meas);

	FSM.n_cycles++;
	if (VERBOSE)
		printf("[%d] cycle number %d\n", ID, FSM.n_cycles);
	// if (VERBOSE)
	// 	printf("[%d] entropy : %f\n", ID, posterior.entropy);

	long long current_estimation_time = current_timestamp() - time_before_estimating;

	estimating_time_sum += current_estimation_time;


	// check if stopping condition is reached
	if ((simulation_parameters.number_of_robots == 1)||(simulation_parameters.teamStrategy == INDIVIDUAL))
	{
		// calculate entropy and spread
		posterior->process();
      	posterior->calculate_entropy_and_spread();
		log_expected_a_posteriori();
		log_max_a_posteriori();
		log_entropy();

		char label[255];
		sprintf(label, "Iteration: %d\n", FSM.n_cycles);
		wb_supervisor_set_label(1, label, 0.1, 0.8, 0.1, 0x61A8E8, 0, "Impact");
		char label2[255];
		sprintf(label2, "Entropy: %.3f\n", posterior->entropy);
		wb_supervisor_set_label(2, label2, 0.1, 0.85, 0.1, 0xFF0000, 0, "Impact");

		
		bool is_converge_condition_reached = check_converge_condition();
		bool is_path_end = path_planner->reach_end_of_fix_path; // only for fixed path like lawnmower
		if (is_converge_condition_reached || is_path_end || FSM.n_cycles == MAX_ITER)
		{
			// FSM.state = STATE_FINISHED;
			// return;
			if (simulation_parameters.estimationScale == GLOBAL){
				FSM.state = STATE_FINISHED;
				printf("Robot ID [%d]: STATE_FINISHED\n", ID);
				fprintf(console_logfile, "Robot ID [%d]: STATE_FINISHED\n", ID);
				broadcast_state_finished();
				return;
			} 
			else {
				// Change the estimation range and move robot to new area
				// reset the posterior and plume range
				// if the estimation converge inside the estimation range, then stop
				// if the estimation converge on the boundary, then it need to be checked
				// Also set a maximum number of relocations
				if (posterior->is_converged_on_boundary() && (FSM.n_relocations < MAX_RELOCATIONS)){
					FSM.n_relocations++;
					// move the robot to the converged position
					goal_position.x = plume_model->max_a_post_value[0];
					goal_position.y = plume_model->max_a_post_value[1];
					goal_position.heading = 0;
					Message msg(ID, 0, goal_position, -1, goal_position, 0);
					send_msg(msg);
					printf("Robot ID [%d]: relocating to new area (%f, %f) \n", ID, goal_position.x, goal_position.y);
					fprintf(console_logfile, "Robot ID [%d]: relocating to new area (%f, %f) \n", ID, goal_position.x, goal_position.y);
					FSM.state = STATE_MOVING;
					FSM.hook = state_moving;

					printf("The estimation converged on the boundary, change the estimation range and relocate robot \n");
					fprintf(console_logfile, "The estimation converged on the boundary, change the estimation range and relocate robot \n");
				} else {
					FSM.state = STATE_FINISHED;
					printf("Robot ID [%d]: STATE_FINISHED\n", ID);
					fprintf(console_logfile, "Robot ID [%d]: STATE_FINISHED\n", ID);
					broadcast_state_finished();
					return;
				}
				
			}
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
	if (FSM.state != STATE_FINISHED)
	{
		if (use_visibility_path)
		{

			printf("goal position : %f %f \n", goal_position.x, goal_position.y);
			path_planner->find_feasible_goal(&goal_position);
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
		// mark the goal position as visited
		int idx = position_to_index_2D(goal_position.x, goal_position.y);
		cell_visited_list[idx] = 1;
            printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, goal_position.x, goal_position.y);
		i = path.begin();
		
		// send the goal to the supervisor
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

bool entered_state_moving = false;

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
		double wind_intensity = 0.0;
		double wind_angle = 0.0;
		double wind_vx = 0.0;
		double wind_vy = 0.0;
		double wind_vz = 0.0;
		odor_read(&sensor_reading, odor_concentration);
		wind_read(&wind_intensity, &wind_angle, &wind_vx, &wind_vy, &wind_vz);
		//printf("instant wind intensity: %f, wind angle: %f, wind vx: %f, wind vy: %f, wind vz: %f\n", wind_intensity, wind_angle, wind_vx, wind_vy, wind_vz);
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
			sample_in_motion.gas_meas.push_back(sensor_reading);
			sample_in_motion.wind_meas_u.push_back(wind_vx);
			sample_in_motion.wind_meas_v.push_back(wind_vy);
			sample_in_motion.size++;
			last_measurement_pos.x = current_position.x;
			last_measurement_pos.y = current_position.y;
			sample_counter = 0;
		}
	}
	// verify if arrived to the goal if applicable
	// fprintf(console_logfile, "check dist to end goal \n");
	// fprintf(console_logfile,"my position (%f, %f) - my end goal (%f, %f)\n", current_position.x, current_position.y, goal_position.x, goal_position.y);
	if (current_position.dist_to_goal(goal_position) > 0.05)
	{

		if (i == path.end())
		{
			fprintf(console_logfile, "path end goal reached \n");
		}

		// move
		if (current_position.dist_to_goal(*i) <= 0.05 && i != path.end())
		{
			//fprintf(console_logfile, "my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
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

		int update_goal_iterations = static_cast<int>(round(simulation_parameters.update_goal_time * 1000.0 / TIME_STEP));

		if (simulation_parameters.updateGoalPolicy == FIXED_FREQ){
			if (n_time_step % update_goal_iterations == 0 && n_time_step != 0)
				{
				double time_now_time_step_far = wb_robot_get_time();
				// set_wheels_speed(0, 0);
				sampleBuffer.reset_all();
				path.clear();
				// printf("Robot ID [%d]. Number of iteration is %d\n", ID, n_time_step);
				FSM.state = STATE_MEASURING;
				FSM.hook = state_measuring;
				time_after_moving = wb_robot_get_time();
				entered_state_moving = false;
			}
		}
	}
	else 
	{
		double time_now_time_step_near = wb_robot_get_time();
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

		// goal position for Robot [101]
		goal_position = goal_position_team[0];
		printf("Robot [%d]: Goal position : %f %f \n", ID, goal_position.x, goal_position.y);
		path_planner->find_feasible_goal(&goal_position);
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
	    
			Message msg(ID, target_ID, goal_position_team[i], -1, goal_position_team[i], 0); // -1 for STATE_FINISHED flag
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


void state_receive_pdf()
{
	if (!entered_state_receive_pdf)
	{
		printf("Robot ID [%d]: STATE_RECEIVE_PDF \n", ID);
		entered_state_receive_pdf = true;
	}
	
		if (!printed && posterior->pdf != nullptr)
		{
			belief_maps[self_index].assign(posterior->pdf, posterior->pdf + MAP_SIZE);
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
                  int message_type = (int)buffer[1];
			float received_current_pos_x = buffer[2];
			float received_current_pos_y = buffer[3];
                  int sender_index = sender_ID - 101;

			printf("Robot ID [%d]: Received message - sender_ID: %d, receiver_ID: %d\n", ID, sender_ID, receiver_ID);
                  // int flag_finished = (int)buffer[9];
			// // check if state_finished
			// if (flag_finished == -1)
			// {
			// 	// printf("Robot ID [%d]: Imposing STATE_FINISHED as requested from robot %d!\n", ID, sender_ID);
			// 	FSM.state = STATE_FINISHED;
			// 	printf("Robot ID [%d]: STATE_FINISHED\n", ID);
			// 	return;
			// }

                  // TODO: figure out what is message type and when we share PDF and when finish state
			if (message_type == MSG_STATE_SHARE_PDF)
			{
				ready_from[sender_index] = 1;
				printf("Robot ID [%d]: Received READY from Robot ID [%d]\n", ID, sender_ID);
				wb_receiver_next_packet(receiver);
				continue;
			}
			
			// Skip self and invalid indices
			if (sender_ID != ID && sender_index >= 0 && sender_index < MAX_NB_ROBOTS)
			{
				belief_maps[sender_index].assign(&buffer[10], &buffer[10] + MAP_SIZE);
				received_current_pos[sender_index][0] = received_current_pos_x;
				received_current_pos[sender_index][1] = received_current_pos_y;
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
				//printf("Robot ID [%d]: Waiting for belief map from Robot ID [%d]\n", ID, 101 + i);
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

	// printf("Robot ID [%d]: Belief map merged successfully.\n", ID);
	printf("Robot ID [%d]: Merged belief map is: \n", ID);
	for (int i = 0; i < MAP_SIZE; i++)
	{
		printf("%f ", belief_map_merged[i]);
	}
	printf("\n");
	belief_map_merged.assign(posterior->pdf, posterior->pdf + MAP_SIZE);
    // Check the stopping condition after merging
	posterior->process();
      posterior->calculate_entropy_and_spread();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	bool is_stop_condition_reached = check_stop_condition();
	if (is_stop_condition_reached)
	{
		FSM.state = STATE_FINISHED;
		printf("Robot ID [%d]: STATE_FINISHED\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: STATE_FINISHED\n", ID);
		broadcast_state_finished();
		return;
	} else {
		if (simulation_parameters.teamStrategy == DISTRIBUTED)
		{
			FSM.state = STATE_REGION_ALLOCATION;
			FSM.hook = state_region_allocation;
		}
	}
	
}

void state_share_pdf()
{
    printf("Robot ID [%d]: STATE_SHARE_PDF\n", ID);

    if (posterior->pdf == NULL) {
        printf("Robot ID [%d]: No posterior PDF to share.\n", ID);
    } else {
        for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
            int target_ID = 101 + i;
            if (target_ID == ID) continue; // skip self

            Message msg(ID, target_ID, Position(), -1, Position(), 0, posterior->pdf);
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
		log_position();
		//log_kld_value();
		posterior.log_pdf(posterior_pdf_3D_name, 'a');

		region_allocation_algorithm();
		
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;
	}
}



void clusterize_belief_map_obstacle_dist_VORONOI(const std::vector<int>& selected_indices) {

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
        if (posterior.pdf[i] <= simulation_parameters.epsilon_clusterize) continue;

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

	log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster(belief_clusters);

    // Binary masks and cutting merged pdf for each robot
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        for (int k = 0; k < simulation_parameters.number_of_robots; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

	// Bucle para procesar todos los clusters
	std::vector<double> cluster_sizes_info(simulation_parameters.number_of_robots, 0.0);

	for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
		double sum_prob = 0.0;

		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			// Aplicar la máscara correspondiente al cluster k
			double masked_value = posterior.pdf[i] * pdf_masks[k][i];

			// Acumular la suma de probabilidades para este cluster
			sum_prob += masked_value;
		}

		// Guardamos la suma de probabilidades en el vector
		cluster_sizes_info[k] = sum_prob;
	}
	log_cluster_sizes_informative(cluster_sizes_info);

		
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


void clusterize_belief_map_obstacle_dist_LLOYD(const std::vector<int>& selected_indices)
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
			if (posterior.pdf[i] <= simulation_parameters.epsilon_clusterize) continue;

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

		// STEP 2: Recompute GEOMETRICAL or INFORMATIVE centroids
		std::vector<double> weighted_sum_x(K, 0.0), weighted_sum_y(K, 0.0), total_weight(K, 0.0);
		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			int cluster = cluster_assignments[i];
			if (cluster < 0) continue;
			
			double weight;

			if (simulation_parameters.mapCoverageStrategy == LLOYD_D) 
			{
				weight = 1;
			}
			else if (simulation_parameters.mapCoverageStrategy == LLOYD_P)
			{
				weight = posterior.pdf[i];
			}					

			if (weight < simulation_parameters.epsilon_clusterize) continue;

			int x, y;
			index_to_xy(i, x, y);
			weighted_sum_x[cluster] += x * weight;
			weighted_sum_y[cluster] += y * weight;
			total_weight[cluster] += weight;
		}

		for (int k = 0; k < K; ++k) {
			if (total_weight[k] > simulation_parameters.epsilon_clusterize) {
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

        // If we want to visualize the evolution of the BM along the 10 iterations refining
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            belief_clusters[i] = (posterior.pdf[i] > simulation_parameters.epsilon_clusterize) ? cluster_assignments[i] + 1 : 0;

        log_posterior_pdf_3d_cluster_lloyd(belief_clusters);
    }

    // Cut Belief Map according to the assigned cluster
	// Assign labels to the cells (clustering)
	std::vector<int> cluster_sizes(simulation_parameters.number_of_robots, 0);
    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);
    std::vector<std::vector<int>> pdf_masks(K, std::vector<int>(x_nbins * y_nbins, 0));

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        int cluster = cluster_assignments[i];
        belief_clusters[i] = cluster + 1;
        pdf_masks[cluster][i] = 1;
		cluster_sizes[cluster]++;
    }

	log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster(belief_clusters);

	// Bucle para procesar todos los clusters
	std::vector<double> cluster_sizes_info(simulation_parameters.number_of_robots, 0.0);

	for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
		double sum_prob = 0.0;

		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			// Aplicar la máscara correspondiente al cluster k
			double masked_value = posterior.pdf[i] * pdf_masks[k][i];

			// Acumular la suma de probabilidades para este cluster
			sum_prob += masked_value;
		}

		// Guardamos la suma de probabilidades en el vector
		cluster_sizes_info[k] = sum_prob;
	}
	log_cluster_sizes_informative(cluster_sizes_info);

		
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

void clusterize_belief_map_obstacle_dist_REGION_GROWING_D() {

    std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots, std::vector<int>(x_nbins * y_nbins, 0));
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));

    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    std::vector<std::pair<double, int>> top_cells;
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] > simulation_parameters.epsilon_clusterize)
            top_cells.push_back({posterior.pdf[i], i});
    }
    std::sort(top_cells.begin(), top_cells.end(), std::greater<>());

    std::vector<int> seeds_x(3), seeds_y(3);
    for (int i = 0; i < 3 && i < top_cells.size(); ++i)
        index_to_xy(top_cells[i].second, seeds_x[i], seeds_y[i]);

    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);

    // 8 posibles direcciones (cardinales y diagonales)
    const std::vector<std::pair<int, int>> all_directions = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1},
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    std::vector<std::pair<int, int>> robot_directions(3);

    // Función para clasificar ángulo en dirección 8-way
    auto angle_to_direction = [](double angle) -> std::pair<int,int> {
        // Ángulo en radianes en [-pi, pi]
        // Convertir a grados para facilitar (opcional)
        double deg = angle * 180.0 / M_PI;

        // Ajustar grados a [0, 360)
        if (deg < 0) deg += 360.0;

        // Definimos sectores de 45° para cada dirección
        // Range: [center-22.5, center+22.5)
        // 0° -> (1, 0)
        // 45° -> (1, 1)
        // 90° -> (0, 1)
        // 135° -> (-1, 1)
        // 180° -> (-1, 0)
        // 225° -> (-1, -1)
        // 270° -> (0, -1)
        // 315° -> (1, -1)

        if ((deg >= 337.5 && deg < 360) || (deg >= 0 && deg < 22.5)) return {1, 0};
        else if (deg >= 22.5 && deg < 67.5) return {1, 1};
        else if (deg >= 67.5 && deg < 112.5) return {0, 1};
        else if (deg >= 112.5 && deg < 157.5) return {-1, 1};
        else if (deg >= 157.5 && deg < 202.5) return {-1, 0};
        else if (deg >= 202.5 && deg < 247.5) return {-1, -1};
        else if (deg >= 247.5 && deg < 292.5) return {0, -1};
        else if (deg >= 292.5 && deg < 337.5) return {1, -1};

        // Por defecto
        return {0, 0};
    };

    // Clasifica cada semilla en dirección hacia su robot usando atan2
	for (int k = 0; k < 3; ++k) {
		double rx = received_current_pos[k][0];
		double ry = received_current_pos[k][1];
		int sx = seeds_x[k];
		int sy = seeds_y[k];
	
		double sx_cont = XMIN + (sx + 0.5) * (XMAX - XMIN) / x_nbins;
		double sy_cont = YMIN + (sy + 0.5) * (YMAX - YMIN) / y_nbins;
	
		double dx = rx - sx_cont; //-rx+sx_cont si queremos direccion en sentido contrario (sx-robot). Si no, rx - sx_cont
		double dy = ry - sy_cont;
	
		if (dx == 0 && dy == 0) {
			robot_directions[k] = {0, 0};
			continue;
		}
	
		double angle = std::atan2(dy, dx);
		robot_directions[k] = angle_to_direction(angle);
	}
	

    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::queue<std::pair<int, int>> queues[3];
    int cluster_sizes[3] = {0, 0, 0};

    int total_active_cells = 0;
    for (int i = 0; i < x_nbins * y_nbins; ++i)
        if (posterior.pdf[i] > simulation_parameters.epsilon_clusterize && obstacle_map[i] == 0)
            total_active_cells++;

    int target_size = total_active_cells / 3;
    int max_size_margin = target_size * 1.2;

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
            if (queues[k].empty()) goto stop_region_growing;

            auto [x, y] = queues[k].front(); queues[k].pop();
            auto [dx, dy] = robot_directions[k];

            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
            if (obstacle_map_2d[ny][nx] == 1) continue;
            if (visited[ny][nx]) continue;

            int idx = ny * x_nbins + nx;
            if (posterior.pdf[idx] <= simulation_parameters.epsilon_clusterize) continue;

            visited[ny][nx] = true;
            belief_clusters[idx] = k + 1;
            queues[k].push({nx, ny});
            cluster_sizes[k]++;
            growing = true;
        }
    }

stop_region_growing:

    log_cluster_sizes_geometric(std::vector<int>(cluster_sizes, cluster_sizes + 3));
    log_posterior_pdf_3d_cluster(belief_clusters);

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        for (int k = 0; k < 3; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

    std::vector<double> cluster_sizes_info(simulation_parameters.number_of_robots, 0.0);
    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        double sum_prob = 0.0;
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            sum_prob += posterior.pdf[i] * pdf_masks[k][i];
        cluster_sizes_info[k] = sum_prob;
    }
    log_cluster_sizes_informative(cluster_sizes_info);

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



void clusterize_belief_map_obstacle_dist_REGION_GROWING_P() {

    std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots, std::vector<int>(x_nbins * y_nbins, 0));
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));

    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    // Buscar las 3 celdas de mayor peso
    std::vector<std::pair<double, int>> top_cells;
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] > simulation_parameters.epsilon_clusterize)
            top_cells.push_back({posterior.pdf[i], i});
    }
    std::sort(top_cells.begin(), top_cells.end(), std::greater<>());

    std::vector<int> seeds_x(3), seeds_y(3);
    for (int i = 0; i < 3 && i < top_cells.size(); ++i)
        index_to_xy(top_cells[i].second, seeds_x[i], seeds_y[i]);

    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);

    // Region Growing alternado y balanceado
    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::queue<std::pair<int, int>> queues[3];
    int cluster_sizes[3] = {0, 0, 0};



    int total_active_cells = 0;
    for (int i = 0; i < x_nbins * y_nbins; ++i)
        if (posterior.pdf[i] > simulation_parameters.epsilon_clusterize && obstacle_map[i] == 0)
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
			// 1. Si ya alcanzó su margen, lo saltamos
			if (cluster_sizes[k] >= max_size_margin) continue;

			// 2. Si la cola está vacía → break global, como pediste
			if (queues[k].empty()) goto stop_region_growing;

			// Process only 1 cell
			auto [x, y] = queues[k].front(); queues[k].pop();

			int best_nx = -1, best_ny = -1;
			double best_pdf = -1.0;
			
			for (const auto& [dx, dy] : directions) {
				int nx = x + dx, ny = y + dy;
				if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
				if (obstacle_map_2d[ny][nx] == 1) continue;
				if (visited[ny][nx]) continue;
			
				int idx = ny * x_nbins + nx;
				if (posterior.pdf[idx] <= simulation_parameters.epsilon_clusterize) continue;
			
				if (posterior.pdf[idx] > best_pdf) {
					best_pdf = posterior.pdf[idx];
					best_nx = nx;
					best_ny = ny;
				}
			}
			
			// Expandimos si encontramos una buena celda
			if (best_nx != -1 && best_ny != -1) {
				int idx = best_ny * x_nbins + best_nx;
				visited[best_ny][best_nx] = true;
				belief_clusters[idx] = k + 1;
				queues[k].push({best_nx, best_ny});
				cluster_sizes[k]++;
				growing = true;
			}
			
			
		}
	}
	stop_region_growing:

    log_cluster_sizes_geometric(std::vector<int>(cluster_sizes, cluster_sizes + 3));
    log_posterior_pdf_3d_cluster(belief_clusters);

    // Asignar pdf_masks
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior.pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        for (int k = 0; k < 3; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

	// Bucle para procesar todos los clusters
	std::vector<double> cluster_sizes_info(simulation_parameters.number_of_robots, 0.0);

	for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
		double sum_prob = 0.0;

		for (int i = 0; i < x_nbins * y_nbins; ++i) {
			// Aplicar la máscara correspondiente al cluster k
			double masked_value = posterior.pdf[i] * pdf_masks[k][i];

			// Acumular la suma de probabilidades para este cluster
			sum_prob += masked_value;
		}

		// Guardamos la suma de probabilidades en el vector
		cluster_sizes_info[k] = sum_prob;
	}
	log_cluster_sizes_informative(cluster_sizes_info);

		
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







void region_allocation_algorithm()
{
	// Select SEED
    std::vector<int> selected_indices;          // salida final

    if (simulation_parameters.mapCoverageSeed == ROBOT_CELL)  // current robot position as seed
    {
        selected_indices.resize(simulation_parameters.number_of_robots);

        for (int i = 0; i < simulation_parameters.number_of_robots; ++i) {
            double x = received_current_pos[i][0];
            double y = received_current_pos[i][1];
            selected_indices[i] = xy_to_index(x, y);
        }
    }
    else if (simulation_parameters.mapCoverageSeed == MAX_CELL)  // max BM cell  as seed
    {
        constexpr int N_SEEDS = 3;
        std::vector<std::pair<double,int>> top_cells;

        // Recorremos todo el grid en formato 1‑D y guardamos celdas candidatas
        for (int idx = 0; idx < x_nbins * y_nbins; ++idx) {
            if (posterior.pdf[idx] > simulation_parameters.epsilon_clusterize)
                top_cells.emplace_back(posterior.pdf[idx], idx);
        }

        // Orden descendente por probabilidad (los empates quedan por orden natural)
        std::partial_sort(top_cells.begin(),
                          top_cells.begin() + std::min(N_SEEDS, (int)top_cells.size()),
                          top_cells.end(),
                          std::greater<>());

        selected_indices.resize(N_SEEDS, -1);   // -1 si falta alguna semilla

        for (int i = 0; i < N_SEEDS && i < (int)top_cells.size(); ++i)
            selected_indices[i] = top_cells[i].second;
    }


	// Visited cells
      for (int idx : selected_indices)
        if (idx >= 0) cell_visited_list[idx] = 1;
        
	// Select coverage
	if (simulation_parameters.mapCoverageStrategy == VORONOI) 
	{
		clusterize_belief_map_obstacle_dist_VORONOI(selected_indices);
	}
	
	if (simulation_parameters.mapCoverageStrategy == LLOYD_D || simulation_parameters.mapCoverageStrategy == LLOYD_P) 
	{
	       clusterize_belief_map_obstacle_dist_LLOYD(selected_indices);
	}
	
	if (simulation_parameters.mapCoverageStrategy == REGION_GROWING_P) clusterize_belief_map_obstacle_dist_REGION_GROWING_P();
	if (simulation_parameters.mapCoverageStrategy == REGION_GROWING_D) clusterize_belief_map_obstacle_dist_REGION_GROWING_D();

}

// Broadcast STATE_FINISHED to all teammates
void broadcast_state_finished()
{
    printf("Robot ID [%d]: Broadcasting STATE_FINISHED\n", ID);

    for (int i = 0; i < simulation_parameters.number_of_robots; i++) {
        int target_ID = 101 + i;
        if (target_ID == ID) continue;  // Skip self

        Message msg(ID, target_ID, Position(), -1, Position(), -1); // -1 for STATE_FINISHED flag
        send_msg(msg);

        printf("Robot ID [%d]: Sent STATE_FINISHED to Robot ID [%d]\n", ID, target_ID);
    }
}

bool check_stop_condition()
{
	bool is_stop_condition_reached = false;
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
	}
	printf("entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior->entropy, posterior->x_spatial_spread, posterior->y_spatial_spread);
	fprintf(console_logfile, "entropy : %f, x_spatial_spread : %d, y_spatial_spread : %d\n", posterior->entropy, posterior->x_spatial_spread, posterior->y_spatial_spread);
	if (is_stop_condition_reached)
	{
		// log the finish time stamp
		end_timestamp = wb_robot_get_time();
		printf("Robot ID [%d]: Stop condition reached!\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: Stop condition reached!\n", ID);
	}
	return is_stop_condition_reached;
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
		end_timestamp = wb_robot_get_time();
		printf("Robot ID [%d]: Stop condition reached!\n", ID);
		fprintf(console_logfile, "Robot ID [%d]: Stop condition reached!\n", ID);
	}
	return is_converge_condition_reached;
}

PlumeModel* initializePlumeModel() {
    if (simulation_parameters.plumeModel == DDPM) {
        return new DeepCFDPlumeModel2D();
    } else if (simulation_parameters.plumeModel == FDM_PDE) {
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
	VisiLibity::Visibility_Graph vg(my_environment, epsilon);
	std::cout << "Validating environment model . . . ";
	if (my_environment.is_valid(epsilon))
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


void init_belief_maps_for_team(){
	belief_maps.assign(simulation_parameters.number_of_robots, std::vector<double>(MAP_SIZE, 0.0));
	belief_map_merged.assign(MAP_SIZE, 0.0);
	received_flags.assign(simulation_parameters.number_of_robots, 0);
}

void init_visited_list(){
	cell_visited_list.assign(MAP_SIZE, 0.0);
}
	

// =========================== MAIN ============================== //

int main(int argc, char **argv)
{
	init();
	read_obstacle_list();
    check_visibility_environment();
	// if multi-robot and using team strategy, initialize the belief maps for each robot
	if (simulation_parameters.number_of_robots > 1 && (simulation_parameters.teamStrategy == CENTRALIZED || simulation_parameters.teamStrategy == DISTRIBUTED)){
		init_belief_maps_for_team();
	}
    // if using visited list, initialize the visited list
	init_visited_list();

    // File for saving console messages
	console_logfile = fopen(console_logfile_name, "a");
	// Initialize the appropriate plume model based on the simulation parameters
	plume_model = initializePlumeModel();
	posterior = new Posterior(plume_model);
	path_planner = new PathPlanner(plume_model, posterior, &history);

	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	plume_model->calculate_gas_map_and_information_matrix();
	plume_model->log_gas_concentration_map(gas_concentration_map_logfile_name, 'a');

	// initialize the receiver
	init_receiver(TIME_STEP);

	// //main loop
	while (FSM.state != STATE_FINISHED)
	{
		if (simulation_parameters.simuMode == ONLINE)
		{
			update_position();
		}
		// change state
		FSM.hook();
		// next
		next_iteration_time();
		n_time_step++;
	}

	fclose(console_logfile);

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