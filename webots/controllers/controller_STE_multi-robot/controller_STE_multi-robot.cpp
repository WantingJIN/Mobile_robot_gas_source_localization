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
// #include "aStar.h"
#include "visilibity.hpp"
#include <webots/supervisor.h>

using namespace std;

// Global Variables

// log files
char folder_name[256];
FILE *goalposition_logfile;
FILE *measurements_logfile;
FILE *posterior_logfile;
FILE *posteriorpdf_logfile;
FILE *expected_a_posteriori_logfile;
FILE *likelihood_logfile;
FILE *measurement_history_logfile;
FILE *position_history_logfile;
FILE *max_a_posteriori_logfile;
FILE *posterior_pdf_3D;
FILE *entropy_logfile;
FILE *tmp_file;
FILE *config_logfile;
FILE *trajectory_logfile;
FILE *gas_concentration_map_logfile;
FILE *gas_information_map_logfile;
FILE *gas_hit_variance_map_logfile;
FILE *experiment_time_logfile;
FILE *visited_list_logfile;

// text file for storing some prints for debugging
FILE *console_logfile;

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
char gas_information_map_logfile_name[512];
char gas_hit_variance_map_logfile_name[512];
char experiment_time_logfile_name[512];
char visited_list_logfile_name[512];

// text file for storing some prints for debugging
char console_logfile_name[512];

// read data from file
char logfile_folder_name[256];
FILE* experiment_measurements_logfile;
char experiment_measurements_logfile_name[512];
FILE* experiment_position_logfile;
char experiment_position_logfile_name[512];
FILE* experiment_goalposition_logfile;
char experiment_goalposition_logfile_name[512];

// define the global parameter - the root path of the project
const char *workspacePath = getenv("STE_DDPM_ROOT");

// finite state machine
struct FSM FSM;

// robot's measurements
SampleBuffer sampleBuffer(ODOR_FILTER_LENGTH);

// position in m
Position goal_position;
Position current_position;
double speed[2] = {0};
// robot's position and measurement history
struct history history;
struct history sample_in_motion;
vector<double> ground_truth_in_motion;

int sample_counter = 0;
// plume model
DeepCFDPlumeModel2D plume_model;

// gas information map
double gas_information_map[x_nbins * y_nbins] = {0};
double gas_hit_variance_map[x_nbins * y_nbins] = {0};
int cell_visited_list[x_nbins * y_nbins] = {0};

// current posterior distribution
Posterior posterior(&plume_model);

// robot's ID
int ID;

// message structure
struct message message;

// list of other robots
struct friends friends[MAX_NB_ROBOTS];

int n_time_step = 0;

struct simulation_parameters simulation_parameters;

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

// precomputed gas maps
double c_avg[x_nbins * y_nbins][x_nbins * y_nbins]; // left: source position, right: estimated cell position

// store last position for minimum distance threshold in measurement update
Position last_measurement_pos;

// real source position
double real_source_pos[3] = {0, 0, 0};

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
	//printf("bearing : %f \n", bearing/PI*180);
      
	double k_rho = 0.2;   // gain for linear velocity
	double k_theta = k_rho * 5.0; // gain for angular velocity; 5.0 is the ratio between the two gains, where is a good value to reach the goal
	// angular and linear velocity
	// the maximal speed of Khepera is 0.8m/s, so that the maximal velocity factor is 4
	float w = simulation_parameters.vel_factor * k_theta * sinf(bearing);

	// ORIGINAL CONTROLLER
	float u = simulation_parameters.vel_factor * k_rho * cosf(bearing); // default vel factor is 1

	// LINEAR VELOCITY CONTROLLER
	// int dir = (cosf(bearing)>0)-(cosf(bearing)<0);
	// float u = simulation_parameters.vel_factor * dir * (abs(bearing)<=0.2) + (abs(bearing)>0.2) * 0.2 * cosf(bearing); //simulation_parameters.vel_factor * 0.2 * cosf(bearing);

	//printf("linear v : %f; angular w : %f \n", u, w);
	// wheels speed
	speed[0] = (u - AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS; 
	speed[1] = (u + AXLE_LENGTH * w / 2.0) / WHEEL_RADIUS;
	//printf("speed : %f %f \n", speed[0], speed[1]);
}

// show the measurement history on screen
void history_print(struct history history)
{
	printf("_____ Sample History _____ \n");
	for (int i = 0; i < history.size; i++)
	{
		printf("%d : %f @ (%f, %f)\n", i, history.measurement[i], history.position[i].x, history.position[i].y);
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

void log_gas_information_map()
{
	gas_information_map_logfile = fopen(gas_information_map_logfile_name, "a");

	for (int i = 0; i < x_nbins * y_nbins; i++)
		fprintf(gas_information_map_logfile, "%.9f, ", gas_information_map[i]);

	fprintf(gas_information_map_logfile, "\n");

	fclose(gas_information_map_logfile);
}

void log_gas_hit_variance_map()
{
	gas_hit_variance_map_logfile = fopen(gas_hit_variance_map_logfile_name, "a");

	for (int i = 0; i < x_nbins * y_nbins; i++)
		fprintf(gas_hit_variance_map_logfile, "%.9f, ", gas_hit_variance_map[i]);

	fprintf(gas_hit_variance_map_logfile, "\n");

	fclose(gas_hit_variance_map_logfile);
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
		// printf("history.measurement[%d] = %d\n", i, history.gas_hit_event[i]);
		fprintf(measurement_history_logfile, "%d, %f, %d, %d\n", i, history.measurement[i], history.gas_hit_event[i], history.FSM_iter[i]);
	}
	fclose(measurement_history_logfile);

	// posterior 3D
	// posterior.log_pdf(posterior_pdf_3D_name, 'w');
}

void log_visited_list()
{
	visited_list_logfile = fopen(visited_list_logfile_name, "a");

	for (int i = 0; i < x_nbins * y_nbins; i++)
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
	fprintf(experiment_time_logfile, "total experiment time (s): %f\n", experiment_time);
	fprintf(experiment_time_logfile, "navigation and sampling time (s): %f\n", navigation_sampling_time_sum);
	fprintf(experiment_time_logfile, "estimating time (s): %f\n", estimating_time_sum / 1000.0);
	fclose(experiment_time_logfile);
}

// ====================== ESTIMATION functions ========================= //

// Likelihood with concentration values from precomputed gas maps
double likelihood_precomp(struct history history, double *parameters, int soure_pos_idx, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

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

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	// absolute concentration error
	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.measurement[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
	}

	return (exp(-0.5 * sum_tmp));
}

// Likelihood for relative concentration with precomputed gas maps
double likelihood_rel_con(struct history &history, vector<double> rel_history_list, double *parameters, int soure_pos_idx, PlumeModel *model)
{
	int pos_coord[2];
	pos_coord[0] = floor((real_source_pos[0] - source_x_min) / x_resol);
      pos_coord[1] = floor((real_source_pos[1] - source_y_min) / y_resol);
	int real_source_idx = posterior.indices_to_index(pos_coord);
	//float sigma = 0.22;// works well for Gaussian CDF
	float sigma = 0.35;// works well for Empirical CDF
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

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

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	vector<double> rel_model_list;
	vector<double> pdf_model_list;
	double model_mean;
	double model_stddev;
	if (simulation_parameters.relativeMethod == NORMALIZATION)
	{
		calculate_normalized_relative_measurements(rel_model_list, model_c_avg);
		sigma = 0.35; //works well for Normalization
	}
	if (simulation_parameters.relativeMethod == M_DISTANCE)
	{
		calculate_data_distribution(model_c_avg, model_mean, model_stddev);
		calculate_measurements_list_M_distance(rel_model_list, model_c_avg, model_mean, model_stddev);
	}
	if (simulation_parameters.relativeMethod == GAUSSIAN_CDF)
	{
		calculate_data_distribution(model_c_avg, model_mean, model_stddev);
		calculate_measurements_list_gaussian_CDF(rel_model_list, model_c_avg, model_mean, model_stddev);
		sigma = 0.22;// works well for Gaussian CDF
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF)
	{
		calculate_measurements_list_empirical_CDF(rel_model_list, model_c_avg); 
		sigma = 0.35;// works well for Empirical CDF
	}
	double rel_model_val, rel_hist_val; // mean, stddev,
	for (int i = 0; i < history.size; i++)
	{
		rel_hist_val = rel_history_list[i];
		rel_model_val = rel_model_list[i];
		// if (real_source_idx == soure_pos_idx)
		// 	printf("hist_val:%f, rel_hist_val: %f, model_val: %f, rel_model_val: %f @ pos (%f, %f)\n", history.measurement[i],rel_hist_val, model_c_avg[i], rel_model_val, history.position[i].x, history.position[i].y);
		sum_tmp += pow(rel_model_val - rel_hist_val, 2);
	}
	sum_tmp /= (2 * sigma * sigma); // best (2*0.15*0.15);

	return (exp(-0.5 * sum_tmp));
}

double likelihood_pdf(struct history &history, vector<double> pdf_history_list, double *parameters, int soure_pos_idx, PlumeModel *model)
{
	int pos_coord[2];
	pos_coord[0] = floor((real_source_pos[0] - source_x_min) / x_resol);
      pos_coord[1] = floor((real_source_pos[1] - source_y_min) / y_resol);
	int real_source_idx = posterior.indices_to_index(pos_coord);
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);
      float sigma = 0.35;
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

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	vector<double> rel_model_list;
	vector<double> pdf_model_list;
      //printf("enter calculating model concentration pdf!\n");
	calculate_measurement_list_probablistic_distribution(pdf_model_list, model_c_avg);
	// compare between the two pdfs
	// calculate the KL divergence
	//printf("enter calculating kld!\n");
	double kl_divergence = 0;
	for (int i = 0; i < pdf_model_list.size(); i++)
	{	
		if (pdf_model_list[i] > 0.0 && pdf_history_list[i] > 0.0)
		{
			kl_divergence += pdf_model_list[i] * log(pdf_model_list[i] /pdf_history_list[i]);
			
		}
	}
	//printf("kl_divergence: %f\n", kl_divergence);
	return (exp(-pow(kl_divergence,2)/(sigma*sigma)));
}

// ASSUMPTION : Using the history of measurements, but only one robot is operating
double likelihood(struct history history, double *parameters, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

	// getting all model concentrations for all sample positions and then calculate the likelihood
	vector<float> model_c_avg(history.size);
	vector<float> model_c_std(history.size);
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);

	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.measurement[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
		// std::cout<<"size: "<< history.size <<"model_c_avg[i] "<<model_c_avg[i]<<std::endl;
	}

	return (exp(-0.5 * sum_tmp));
}

void calculate_data_distribution(vector<double> data_list, double &mean, double &stddev)
{
	double sum = 0;
	double sum2 = 0;
	for (int i = 0; i < data_list.size(); i++)
	{
		sum += data_list[i];
	}
	mean = sum / data_list.size();
	for (int i = 0; i < data_list.size(); i++)
	{
		sum2 += pow(data_list[i] - mean, 2);
	}
	stddev = sqrt(sum2 / data_list.size());
	// printf("mean: %f, variance: %f\n", mean, variance);
}

double calculate_measurement_gaussian_CDF(double data, double mean, double stddev)
{
	double cdf = 0.5 * (1 + std::erf((data - mean) / (stddev * std::sqrt(2))));
	// printf("data: %f, mean: %f, stddev: %f, cdf: %f\n", data, mean, stddev, cdf);
	return cdf;
}

double calculate_measurement_M_distance(double data, double mean, double stddev)
{
	double m_distance = (data - mean)/ stddev;
	//printf("data: %f, m_distance: %f\n", data, m_distance);
	return m_distance;
}

void calculate_measurements_list_gaussian_CDF(vector<double> &rel_meas_list, vector<double> data_list, double mean, double stddev)
{
	if (stddev == 0)
		stddev = 0.01;
	for (int i = 0; i < data_list.size(); i++)
	{
		rel_meas_list.push_back(calculate_measurement_gaussian_CDF(data_list[i], mean, stddev));
	}
}
void calculate_measurements_list_M_distance(vector<double> &rel_meas_list, vector<double> data_list, double mean, double stddev)
{
	if (stddev == 0)
		stddev = 0.01;
	//printf("mean: %f, stddev: %f\n", mean, stddev);
	for (int i = 0; i < data_list.size(); i++)
	{
		rel_meas_list.push_back(calculate_measurement_M_distance(data_list[i], mean, stddev));
	}


}
void calculate_measurements_list_empirical_CDF(vector<double> &rel_meas_list, vector<double> data_list)
{
	if (data_list.size() == 1) {
		rel_meas_list.push_back(0);
		return;
	}
	vector<double> sortedData = data_list;
	sort(sortedData.begin(), sortedData.end());

	// Calculate ECDF
	vector<double> ecdf(data_list.size());
	for (size_t i = 0; i < data_list.size(); ++i) {
		// Count the number of observations less than or equal to the current value
		size_t count = count_if(sortedData.begin(), sortedData.end(),
							[&](double x) { return x < data_list[i]; });
		// Calculate proportion
		rel_meas_list.push_back(static_cast<double>(count) / data_list.size());
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

void calculate_measurement_list_probablistic_distribution(vector<double> &pdf_meas_list, vector<double> data_list)
{
	// find the max and min values
	double max_val = *max_element(data_list.begin(), data_list.end());
	double min_val = *min_element(data_list.begin(), data_list.end());
	int data_size = data_list.size();
	int n_bins = 10;
	if (abs(max_val - min_val) < 1e-2 )
	{
		printf("max and min are the same! set uniform distribution\n");
		// uniform distribution
		for (int i = 0; i < n_bins; i++){
			pdf_meas_list.push_back(double(1.0)/double(n_bins));
		}
		return;
	}
	// classfiy the data into bins
	double bin_width = (max_val - min_val)/ double(n_bins);
	vector<int> bin_count(n_bins, 0);
	for (int i = 0; i < data_size; i++)
	{
		if (data_list[i] < 0)
			data_list[i] = 0;
		int bin_idx = floor(data_list[i]- min_val / bin_width);
		if (bin_idx > n_bins-1)
			bin_idx = n_bins-1;
		if (bin_idx < 0)
			bin_idx = 0;
		bin_count[bin_idx]++;
	}
	// calculate the probability of each bin
	//printf("pdf is: ");
	for (int i = 0; i < n_bins; i++)
	{

		pdf_meas_list.push_back(double(bin_count[i]) / double(data_size));
		//printf("%f, ", double(bin_count[i]) / double(data_size));
	}
	//printf("\n");
}

double likelihood_gas_hit(struct history &history, double *parameters, int soure_pos_idx, PlumeModel *model, double mean, double stddev)
{
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

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

	// old
	// model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);

	// with precomputed map is much faster
	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	int history_hit;
	int model_hit;

	double likelihood;

	//--------------------------- fixed thresh -----------------------------
	/*
	int gas_hit_threshold  = 1000;
	for (int i = 0; i < history.size; i++){
		if (history.measurement[i] < gas_hit_threshold)
			history_hit = 0;
		else
			history_hit = 1;
	  history.gas_hit_event[i] = history_hit;
		if (model_c_avg[i] < gas_hit_threshold)
			model_hit = 0;
		else
			model_hit = 1;
	    if (history_hit != model_hit)
		    sum_tmp += 1;
	}

	sum_tmp *= 0.5; //best result with exp 0.5
	likelihood = (exp(-sum_tmp));

	//sum_tmp *= 0.5;
	//likelihood = 1-tanh(sum_tmp);
	*/

	//-----model fixed threshold, penalty on relative concentration history -------

	double rel_hist_val = 0.0;
	int gas_hit_threshold = 1000;
	for (int i = 0; i < history.size; i++)
	{
		if (stddev == 0)
			rel_hist_val = calculate_measurement_gaussian_CDF(history.measurement[i], mean, 0.01);
		else
			rel_hist_val = calculate_measurement_gaussian_CDF(history.measurement[i], mean, stddev);
		if (model_c_avg[i] < gas_hit_threshold)
		{
			model_hit = 0;		 // for plotting
			sum_tmp += rel_hist_val; // larger penalty if value detected is large and model has a miss
		}
		else
		{
			model_hit = 1;
			sum_tmp += 1 - rel_hist_val; // small penalty if model hit and high concentration measurement
		}
		history.gas_hit_event[i] = model_hit; // will have to be re-run at the end of the algo with the expected source location as input for plotting
	}

	sum_tmp *= 0.8;
	likelihood = (exp(-sum_tmp));

	//-----------------dynamic thresholds and constant penalty--------------------------

	//-------------------------min max avg threshold-------------------------------
	/*
	double lamda = 0.8; //0.5;
	double his_thrs = 0;
	double mod_thrs = 0;
	static double max_con = 0.0;
	static double min_con = 10000.0;
	static int max_idx = 0;
	static int min_idx = 0;

	//Dynamic threshold
	if (history.size<=1){
		max_con = history.measurement[0];
		min_con = history.measurement[0];
		max_idx = 0;
		min_idx = 0;
	}else{
		if (history.measurement[history.size-1]>max_con ){
			max_con = history.measurement[history.size-1];
			max_idx = history.size-1;
		}else if (history.measurement[history.size-1]<min_con){
			min_con = history.measurement[history.size-1];
			min_idx = history.size-1;
		}
		his_thrs = min_con+(max_con-min_con)*0.5;
		mod_thrs = model_c_avg[min_idx]+(model_c_avg[max_idx]-model_c_avg[min_idx])*0.5;
	}
	for (int i = 0; i < history.size; i++){
	  // decide whether the gas hit event happens
		history_hit = (history.measurement[i] > his_thrs) ? 1 : 0;
		history.gas_hit_event[i] = history_hit;
		model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
	  // update the likelihood based on the similarity of the hit event
		if (history_hit != model_hit){
			sum_tmp +=1;
		}
	}
	//sum_tmp *= 1;
	likelihood = (exp(-sum_tmp));
	*/

	//-------------adaptive threshold- based on recent measurements------------------------------
	/*
	double lamda = 0.5; //0.5;
	double his_thrs = 0;
	double mod_thrs = 0;
	double prev_mod_thrs = 0.0;
	double prev_his_thrs = 0.0;

	//Dynamic thresholds computation
	for (int i = 0; i < history.size; i++){
		if (i == 0){
			prev_his_thrs = history.measurement[i];
			his_thrs = history.measurement[i];
			prev_mod_thrs = model_c_avg[i];
			mod_thrs = model_c_avg[i];
		}
		his_thrs = lamda * prev_his_thrs + (1 - lamda) * history.measurement[i];
		mod_thrs = lamda * prev_mod_thrs + (1 - lamda) * model_c_avg[i];
		//store history threshold for next iteration
		prev_his_thrs = his_thrs;
		prev_mod_thrs = mod_thrs;
	}

	for (int i = 0; i < history.size; i++){
	  // decide whether the gas hit event happens
		history_hit = (history.measurement[i] > his_thrs) ? 1 : 0;
		history.gas_hit_event[i] = history_hit;
		model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
	  // update the likelihood based on the similarity of the hit event
		if (history_hit != model_hit){
			sum_tmp +=1;
		}
	}
	sum_tmp *= 0.5;
	likelihood = (exp(-sum_tmp));
	*/

	//-----------adaptive threshold for future dynamic environment---------------------
	// For this to work, the environmental parameters such as wind speed will have
	// to be stored and used as input of the model concentrations
	/*
	double lamda = 0.5;
	double his_thrs = 0;
	double mod_thrs = 0;
	double prev_mod_thrs = 0.0;
	double prev_his_thrs = 0.0;

	for (int i = 0; i < history.size; i++){
		if (i == 0){
			prev_his_thrs = history.measurement[i];
			his_thrs = history.measurement[i];
			prev_mod_thrs = model_c_avg[i];
			mod_thrs = model_c_avg[i];
		}
		his_thrs = lamda * prev_his_thrs + (1 - lamda) * history.measurement[i];
		mod_thrs = lamda * prev_mod_thrs + (1 - lamda) * model_c_avg[i];
		//store history threshold for next iteration
		prev_his_thrs = his_thrs;
		prev_mod_thrs = mod_thrs;
	  // decide whether the gas hit event happens
		history_hit = (history.measurement[i] > his_thrs) ? 1 : 0;
		history.gas_hit_event[i] = history_hit;
		model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
	  // update the likelihood based on the similarity of the hit event
		if (history_hit != model_hit){
			sum_tmp +=1;
		}
	}
	sum_tmp *= 0.5;
	likelihood = (exp(-sum_tmp));
	*/

	return likelihood;
}

bool prior(double *parameters, int nbr_dimension, PlumeModel *model)
{
	int i = 0;
	bool output = 1;

	// verify if all the parameters are between the right intervals
	for (i = 0; i < nbr_dimension; i++)
		output = output && (parameters[i] < model->param_list[i].upper_limit && parameters[i] >= model->param_list[i].lower_limit);

	// verify the x and y elements of the parameters are not inside obstacles
	Position tmp_p(parameters[0], parameters[1], 0);
	if (tmp_p.is_in_obstacle_list(obstacles, nbr_obs) != -1)
	{
		// printf("point %f %f is in obstacle\n", parameters[0], parameters[1]);
		output = 0;
	}

	// if(output == 0)
	//	printf("parameters: %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4]);

	return (output);
}

// ======================= NAVIGATION functions ========================= //
void random_navigation()
{
	goal_position.x = (randu() * (XMAX - SAFETY_MARGIN * 2)) + SAFETY_MARGIN; // position in m
	goal_position.y = (randu() * (YMAX - SAFETY_MARGIN * 2)) + SAFETY_MARGIN;
	goal_position.heading = 0; //(randu() * (ZMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
}

bool position_outside_arena(Position p)
{
	bool outside = 0;
	if (p.x < float(XMIN) || p.x > float(XMAX))
		outside = 1;
	if (p.y < float(YMIN) || p.y > float(YMAX))
		outside = 1;
	// if(position[2] < ZMIN || position[2] > ZMAX) outside = 1;
	return outside;
}
void information_gain_navigation_approx()
{
	float p0 = 0.5; // 50 cm
	double KLD = 0, max_KLD = -100, max_meas = -100;
	// double position[1][3] = {{0}};
	Position tmp_position;
	struct history fake_history;
	fake_history.position = history.position;
	fake_history.measurement = history.measurement;
	fake_history.size = history.size;

	// add fake prediction for friends goal position, so we don't choose the same ones
	for (int i = 0; i < simulation_parameters.number_of_robots; i++)
	{
		if (friends[i].ID == 0 || friends[i].ID == ID)
			continue;
		fake_history.position[fake_history.size].copy_position(friends[i].goal);
		fake_history.measurement[fake_history.size] = plume_model.approx_concentration(friends[i].goal, sampleBuffer.average_wind_intensity);
		(fake_history.size)++;
	}
	// 8 candidate positions
	Position candidates[8] = {Position(current_position.x + p0, current_position.y, 0),
					  Position(current_position.x - p0, current_position.y, 0),
					  Position(current_position.x, current_position.y + p0, 0),
					  Position(current_position.x, current_position.y - p0, 0),
					  Position(current_position.x + p0, current_position.y + p0, 0),
					  Position(current_position.x - p0, current_position.y + p0, 0),
					  Position(current_position.x + p0, current_position.y - p0, 0),
					  Position(current_position.x - p0, current_position.y - p0, 0)};
	for (auto &candidate_pos : candidates)
	{
		// for each possible action
		tmp_position.x = candidate_pos.x;
		tmp_position.y = candidate_pos.y;
		tmp_position.heading = 0;
		// printf("position : %f %f\n", tmp_position.x, tmp_position.y);
		// printf("position : %f %f %f\n", j*p0 * cos(l*theta0) * sin(k*phi0),j*p0 * sin(l*theta0) * sin(k*phi0),j*p0 * cos(k*phi0));
		if (position_outside_arena(tmp_position))
			continue;
		// calculate the expected concentration on position
		fake_history.measurement.push_back(plume_model.approx_concentration(tmp_position, sampleBuffer.average_wind_intensity));
		std::cout << "fake_history.measurement = " << fake_history.measurement[fake_history.size] << std::endl;
		fake_history.position.push_back(tmp_position);
		(fake_history.size)++;
		DeepCFDPlumeModel2D plume_model_fake;
		Posterior future_posterior(&plume_model_fake, 0);
		// future_posterior.copy_posterior(zero);
		future_posterior.MCMC_evaluation(fake_history);
		// calculate the expected information gain
		KLD = posterior.Kullback_Leibler_divergence(future_posterior);
		// printf("DKL: %f _______________________\n", DKL);
		if (KLD > max_KLD)
		{
			max_KLD = KLD;
			goal_position.x = tmp_position.x; // + (randu()-0.5)/5.0
			goal_position.y = tmp_position.y;
			goal_position.heading = 0;
		}
		fake_history.measurement.pop_back();
		fake_history.position.pop_back();
		fake_history.size--;
	}
}

void balance_exploration_exploitation(float gloabl_local_search_ratio)
{
	float ratio = gloabl_local_search_ratio;
	float vec1[2] = {0}, vec2[2] = {0};
	printf("current_position : %f %f , goal position : %f %f , estimated source : %f %f \n", current_position.x, current_position.y,
		 goal_position.x, goal_position.y, plume_model.max_a_post_value[0], plume_model.max_a_post_value[1]);
	vec1[0] = goal_position.x - current_position.x; // exploration
	vec1[1] = goal_position.y - current_position.y;
	vec2[0] = plume_model.max_a_post_value[0] - current_position.x; // TODO: to improve
	vec2[1] = plume_model.max_a_post_value[1] - current_position.y;
	goal_position.x = current_position.x + ratio * vec1[0] + (1 - ratio) * vec2[0];
	goal_position.y = current_position.y + ratio * vec1[1] + (1 - ratio) * vec2[1];
	goal_position.heading = 0;
	// printf("new goal position : %f %f %f\n", goal_position[0],goal_position[1],goal_position[2]);
}

void greedy_planning()
{
      printf("Enter greedy planning \n");
	fprintf(console_logfile, "Enter greedy planning \n");
	double best_point[2] = {0};
	// Adaptive greedy planning based on entropy
	// if (posterior.entropy > 4){
	// 	posterior.XYmarginal_distance_combined(current_position, 1.0, best_point);
	// } else {
	// 	posterior.XYmarginal_distance_combined(current_position, 0.75, best_point);
	// }
	posterior.XYmarginal_distance_combined(current_position, 1.0, best_point);
	// printf("best point: %f %f \n", best_point[0], best_point[1]);
	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;
}

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

void lawnmower_planning()
{
	printf("Enter lawnmower planning \n");
	int static direction = 1;
	double x_step = 1.0;
	double y_step = 4;
	goal_position.y = current_position.y + direction * y_step;
	goal_position.x = current_position.x;
	goal_position.heading = 0;
	if (current_position.y > YMAX - SAFETY_MARGIN - 0.05 && direction == 1)
	{
		goal_position.x = current_position.x + x_step;
		direction = -1;
	}
	if (current_position.y < YMIN + SAFETY_MARGIN + 0.1 && direction == -1)
	{
		goal_position.x = current_position.x + x_step;
		direction = 1;
	}
	if (goal_position.x > XMAX - SAFETY_MARGIN - 0.05)
	{
		FSM.state = STATE_FINISHED;
		printf("FINISHED \n");
	}
}

void calculate_information_gain()
{
	if (simulation_parameters.infoFeature == VARIANCE)
	{
		plume_model.calculate_weighted_gas_con_variance_map(gas_information_map, posterior.pdf);
	}
	else if (simulation_parameters.infoFeature == MEAN)
	{
		plume_model.calculate_weighted_gas_con_mean_map(gas_information_map, posterior.pdf);
	}
	// printf("gas information map is: \n");
	// for (int i = 0; i < x_nbins * y_nbins; i++)
	// {
	// 	printf("%f, ", gas_information_map[i]);
	// }
	// printf("\n");
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

	// The goal position is very close to current postion, swith to greedy planning
	if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<double>(simulation_parameters.sampling_dist_thresh)))
	{
		printf("The goal position is very close to current postion, switch to greedy planning \n");
		fprintf(console_logfile, "The goal position is very close to current postion, switch to greedy planning \n");
		greedy_planning();
	}
	printf("goal position : %f %f \n", goal_position.x, goal_position.y);
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
      if (fabs(max_pdf - 0.0) < epsilon) // all the probable points have been visited
	{
		printf("all the probable points have been visited \n");
		fprintf(console_logfile, "all the probable points have been visited \n");
		maximal_information_planning();
		printf("goal position : %f %f \n", goal_position.x, goal_position.y);
		return;
	}

	posterior.index_to_param_values(max_pdf_index, best_point);

	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;

	// The goal position is very close to current postion, swith to greedy planning
	if (dist_to_goal(goal_position) <= std::max(0.05, static_cast<double>(simulation_parameters.sampling_dist_thresh)))
	{
		printf("The goal position is very close to current postion, switch to greedy planning \n");
		fprintf(console_logfile, "The goal position is very close to current postion, switch to greedy planning \n");
		greedy_planning();
	}
	printf("goal position : %f %f \n", goal_position.x, goal_position.y);
}


vector<pair<CellPoint, CellPoint>> clustering_information_blocks(double information_map[])
{
	// dbscan clustering
	// param: x_eps, y_eps, minPts, ratio
	DBSCAN dbscan(x_resol + 0.1, y_resol + 0.1, 4, (1.0 - posterior.entropy / simulation_parameters.cluster_entropy_ratio));
	dbscan.run(information_map);
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
		maximal_belief_planning();
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
					}
				}
				else
				{
					min_dist_to_robot = dist;
					min_dist_to_robot_index = j;
				}
			}
		}
		if (min_dist_to_robot == 1000)
		{
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

void balance_expected_maximal_pdf()
{
	double maximal_pdf_point[2] = {0};
	double expected_pdf_point[2] = {0};
	maximal_pdf_point[0] = plume_model.max_a_post_value[0];
	maximal_pdf_point[1] = plume_model.max_a_post_value[1];
	expected_pdf_point[0] = plume_model.expected_value[0];
	expected_pdf_point[1] = plume_model.expected_value[1];
	goal_position.x = (maximal_pdf_point[0] + expected_pdf_point[0]) / 2;
	goal_position.y = (maximal_pdf_point[1] + expected_pdf_point[1]) / 2;
	goal_position.heading = 0;
}

void joint_planning()
{
	// std::cout<<"joint_planning"<<std::endl;
	//  2 robots, 2 possibilities
	if (simulation_parameters.number_of_robots == 2)
	{
		if (pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, goal_position) < pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[0].goal))
		{
			// printf("let's swap ************************************************\n");
			// swap the goal positions
			// inform the other guy
			// wb_emitter_set_channel(emitter, 10);
			Message msg(ID, friends[0].ID, current_position, -1, goal_position, 1);
			// wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
			send_msg(msg);
			// change mine as well
			goal_position.copy_position(friends[0].goal);
			// printf("my new goal %f %f %f , my friend's new goal \n", goal_position[0], goal_position[1], goal_position[2]);
		}
	}
	// 3 robots, 6 possibilites
	if (simulation_parameters.number_of_robots == 3)
	{
		if (std::isnan(friends[0].goal.x) || std::isnan(friends[0].goal.y) || std::isnan(friends[1].goal.x) || std::isnan(friends[1].goal.y))
			return;
		int new_dist = 0, min_dist = 0;
		Position my_new_goal;
		Position friend_1_new_goal;
		Position friend_2_new_goal;
		my_new_goal.copy_position(goal_position);
		friend_1_new_goal.copy_position(friends[0].goal);
		friend_2_new_goal.copy_position(friends[1].goal);
		// printf("our old goals : mine %f %f %f, friend 1: %f %f %f, friend 2: %f %f %f \n", goal_position[0], goal_position[1], goal_position[2], friends[0].goal[0], friends[0].goal[1], friends[0].goal[2], friends[1].goal[0],friends[1].goal[1],friends[1].goal[2]);

		// 0 : no swap
		min_dist = pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[0].goal) + pt_distance(friends[1].position, friends[1].goal);
		// 1 : friends swap
		new_dist = pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[1].goal) + pt_distance(friends[1].position, friends[0].goal);
		if (new_dist < min_dist)
		{
			// printf("we are in 1\n");
			min_dist = new_dist;
			my_new_goal.copy_position(goal_position);
			friend_1_new_goal.copy_position(friends[1].goal);
			friend_2_new_goal.copy_position(friends[0].goal);
		}
		// 2 : I swap with friend1, friend1 swaps with friend2
		new_dist = pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, friends[1].goal) + pt_distance(friends[1].position, goal_position);
		if (new_dist < min_dist)
		{
			// printf("we are in 2\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[0].goal);
			friend_1_new_goal.copy_position(friends[1].goal);
			friend_2_new_goal.copy_position(goal_position);
		}
		// 3 : I swap with friend1
		new_dist = pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, goal_position) + pt_distance(friends[1].position, friends[1].goal);
		if (new_dist < min_dist)
		{
			// printf("we are in 3\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[0].goal);
			friend_1_new_goal.copy_position(goal_position);
			friend_2_new_goal.copy_position(friends[1].goal);
		}
		// 4 : I swap with friend2, friend2 swaps with friend1
		new_dist = pt_distance(current_position, friends[1].goal) + pt_distance(friends[0].position, goal_position) + pt_distance(friends[1].position, friends[0].goal);
		if (new_dist < min_dist)
		{
			// printf("we are in 4\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[1].goal);
			friend_1_new_goal.copy_position(goal_position);
			friend_2_new_goal.copy_position(friends[0].goal);
		}
		// 5 : I swap with friend2
		new_dist = pt_distance(current_position, friends[1].goal) + pt_distance(friends[0].position, friends[0].goal) + pt_distance(friends[1].position, goal_position);
		if (new_dist < min_dist)
		{
			// printf("we are in 5\n");
			min_dist = new_dist;
			my_new_goal.copy_position(friends[1].goal);
			friend_1_new_goal.copy_position(friends[0].goal);
			friend_2_new_goal.copy_position(goal_position);
		}

		// printf("Our new goals : mine %f %f %f, friend 1: %f %f %f, friend 2: %f %f %f \n", goal_position[0], goal_position[1], goal_position[2], friends[0].goal[0], friends[0].goal[1], friends[0].goal[2], friends[1].goal[0],friends[1].goal[1],friends[1].goal[2]);

		// inform the other guys if any changes
		if (friend_1_new_goal.x != friends[0].goal.x || friend_1_new_goal.y != friends[0].goal.y)
		{
			Message msg1(ID, friends[0].ID, current_position, -1, friend_1_new_goal, 1);
			send_msg(msg1);
		} // else printf("no change for friend %d\n", friends[0].ID);
		if (friend_2_new_goal.x != friends[1].goal.x || friend_2_new_goal.y != friends[1].goal.y)
		{
			Message msg2(ID, friends[1].ID, current_position, -1, friend_2_new_goal, 1);
			send_msg(msg2);
		} // else printf("no change for friend %d\n", friends[1].ID);

		// change mine as well
		goal_position.copy_position(my_new_goal);
	}
}

void KLD_MAP_distance(float gloabl_local_search_ratio)
{
	double best_point[2] = {0};
	posterior.XYmarginal_distance_combined(current_position, gloabl_local_search_ratio, best_point);
	printf("greedy goal %f %f\n", goal_position.x, goal_position.y);
	printf("best point %f %f\n", best_point[0], best_point[1]);

	float ratio = 0.3; // gloabl_local_search_ratio;//simulation_parameters.gloabl_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
	float vec1[2] = {0}, vec2[2] = {0};
	// printf("current_position : %f %f %f, goal position : %f %f %f, estimated source : %f %f %f\n", current_position[0],current_position[1],current_position[2],
	//	goal_position[0],goal_position[1],goal_position[2], estimated_source_parameters.source_position[0],estimated_source_parameters.source_position[1],estimated_source_parameters.source_position[2]);
	vec1[0] = goal_position.x - current_position.x; // exploration
	vec1[1] = goal_position.y - current_position.y;
	// vec1[2] = goal_position[2] - current_position[2];
	vec2[0] = best_point[0] - current_position.x; // exploitation (source) //estimated_source_parameters //max_a_post_source_parameters
	vec2[1] = best_point[1] - current_position.y;
	// vec2[2] = estimated_source_parameters.source_position[2] - current_position[2];
	goal_position.x = current_position.x + ratio * vec1[0] + (1 - ratio) * vec2[0];
	goal_position.y = current_position.y + ratio * vec1[1] + (1 - ratio) * vec2[1];
	goal_position.heading = 0; // current_position[2] + ratio*vec1[2] + (1-ratio)*vec2[2];
}

void goal_outside_obstacles(Position *goal)
{

	double min_dist_to_current = 0.05 + simulation_parameters.sampling_dist_thresh;
	// is the goal position in an obstacle or out of arena or too close to current position?
	int idx = goal->is_in_obstacle_list(obstacles, nbr_obs);
	if (idx == -1 && !position_outside_arena(*goal) && goal->distance_from(current_position) < min_dist_to_current)
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

			// give priority to unvisited cells but was much slower
			/*
			if (goal->distance_from(candidates[i]) <= min_dist_unvisited && simulation_parameters.visitedList == visitedList_ON){
				int pos_coord[2];
				pos_coord[0] =  floor((candidates[i].x - source_x_min) / x_resol);
				pos_coord[1] =  floor((candidates[i].y - source_y_min) / y_resol);
				int idx = posterior.indices_to_index(pos_coord);
				if (cell_visited_list[idx] == 0){
					//if so, then this is the nearest position
					nearest_unvisited_position.copy_position(candidates[i]);
					min_dist_unvisited = goal->distance_from(candidates[i]);
				}
			}
			*/
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
	// fprintf(console_logfile, ">>> Candidate goal %f %f \n", goal->x, goal->y);
	// fprintf(console_logfile, ">>> Current position %f %f \n", current_position.x, current_position.y);
	// fprintf(console_logfile, ">>> Dist from current %f \n", dist_candidate_goal);
	if (goal->distance_from(current_position) < min_dist_to_current)
	{
		printf(">>> ! position %f %f is too close to current position\n", goal->x, goal->y);
		// fprintf(console_logfile, ">>> ! position %f %f is too close to current position\n", goal->x, goal->y);
		//  otherwise, lets try 4 different candidates
		float min_dist_unvisited = 10000.0;
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
	}

	// change the goal position using the pointer
	// printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
	// fprintf(console_logfile, ">>> Outside arena Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
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
	int d,iter;
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
		printf("measurement @ position (%f, %f, %f) is %f\n", current_position.x, current_position.y, current_position.heading,f4);
		
	}
	sampleBuffer.average_wind_intensity =1;
	FSM.state = STATE_ESTIMATING;
      FSM.hook = state_estimating;
}

void state_read_positions_measurements()
{
	printf("Enter state read goal and measurment\n");
	int d,iter;
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
		pos_coord[0] = floor((goal_position.x - source_x_min) / x_resol);
		pos_coord[1] = floor((goal_position.y - source_y_min) / y_resol);
		int idx = posterior.indices_to_index(pos_coord);
		cell_visited_list[idx] = 1;
		//printf("goal position is (%f, %f, %f) at %lf\n", goal_position.x, goal_position.y, f3, t);

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
			//printf("measurement @ position (%f, %f, %f) is %f\n", current_position.x, current_position.y, current_position.heading,f4);
		}
	}
	sampleBuffer.average_wind_intensity =1;
	FSM.state = STATE_ESTIMATING;
      FSM.hook = state_estimating;
}

void state_measuring()
{
	// printf("measuring sample number %d\n", measurements.n_samples);
	vector<double> sample_from_map;
	// if robot is sensing while moving, saving the measurement in motion
	if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
	{
		// if the kernel mapping is enabled, then retrieve the measurement from the gas map
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			gas_map.retrieve_readings_from_gasmap(sample_in_motion.position, sample_from_map);
			for (int i = 0; i < sample_in_motion.size; i++)
			{
				// printf("instant measurement at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, sample_in_motion.measurement[i]);
				// printf("measurement from gas map at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, sample_from_map[i]);
				// printf("ground truth at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, ground_truth_in_motion[i]);
				history.position.push_back(sample_in_motion.position[i]);
				history.measurement.push_back(sample_from_map[i]);
				history.gas_hit_event.push_back(0);
				history.FSM_iter.push_back(FSM.n_cycles);
				history.size++;
				// mark the cell as visited
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int pos_coord[2];
					pos_coord[0] = floor((sample_in_motion.position[i].x - source_x_min) / x_resol);
					pos_coord[1] = floor((sample_in_motion.position[i].y - source_y_min) / y_resol);
					int idx = posterior.indices_to_index(pos_coord);
					cell_visited_list[idx] = 1;
				}
			}
		}
		else
		{
			// if else, save the instant measurement directly
			for (int i = 0; i < sample_in_motion.size; i++)
			{
				if (simulation_parameters.gasSensor == MOX)
				{
					// printf("instant measurement at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, sample_in_motion.measurement[i]);
					// printf("ground truth at (%f, %f) is %f\n", sample_in_motion.position[i].x, sample_in_motion.position[i].y, ground_truth_in_motion[i]);
				}
				history.position.push_back(sample_in_motion.position[i]);
				history.measurement.push_back(sample_in_motion.measurement[i]);
				history.gas_hit_event.push_back(0);
				history.FSM_iter.push_back(FSM.n_cycles);
				history.size++;
				// mark the cell as visited
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int pos_coord[2];
					pos_coord[0] = floor((sample_in_motion.position[i].x - source_x_min) / x_resol);
					pos_coord[1] = floor((sample_in_motion.position[i].y - source_y_min) / y_resol);
					int idx = posterior.indices_to_index(pos_coord);
					cell_visited_list[idx] = 1;
				}
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

		printf("wind intensity reading: %f\n", sampleBuffer.instant_wind_intensity);
		printf("wind orientation: %f\n", sampleBuffer.instant_wind_angle);
		printf("Speed 0: %f\n", speed[0]);
		printf("Speed 1: %f\n", speed[1]);
		printf("Robot lin vel: %f\n", ((speed[0] + speed[1]) * WHEEL_RADIUS / 2));
		printf("Robot orientation: %f\n", current_position.heading);

		sampleBuffer.add_all_to_buffer();
		sampleBuffer.average_all();
		// FIXME - add the case for reverse sensor model
		history.position.push_back({current_position.x, current_position.y, 0});
		// history.measurement[history.size] = sampleBuffer.average_concentration;
		history.measurement.push_back(sampleBuffer.average_concentration);
		history.gas_hit_event.push_back(0);
		history.FSM_iter.push_back(FSM.n_cycles);
		history.size++;
		// mark the current cell as visited
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			int pos_coord[2];
			pos_coord[0] = floor((current_position.x - source_x_min) / x_resol);
			pos_coord[1] = floor((current_position.y - source_y_min) / y_resol);
			int idx = posterior.indices_to_index(pos_coord);
			cell_visited_list[idx] = 1;
		}
		if (VERBOSE)
			printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
		log_position();
		log_measurements();
		history_print(history);
		// go to estimation
		FSM.state = STATE_ESTIMATING;
		FSM.hook = state_estimating;
		time_after_measuring = wb_robot_get_time();
		navigation_sampling_time_sum += time_after_measuring - time_before_measuring;
		printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
	}
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
		}
		else
		{
			// average all samples
			sampleBuffer.average_all();
			// log and save
			if (VERBOSE)
				printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
			log_position();
			log_measurements();
			if (!position_outside_arena(current_position))
			{
				history.position.push_back({current_position.x, current_position.y, 0});
				// history.measurement[history.size] = sampleBuffer.average_concentration;
				history.measurement.push_back(sampleBuffer.average_concentration);
				history.gas_hit_event.push_back(0);
				history.FSM_iter.push_back(FSM.n_cycles);
				history.size++;
				history_print(history);
			}
			// send it to friends
			if (simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR)
			{
				Message msg(ID, 1, history.position[(history.size) - 1], sampleBuffer.average_concentration, goal_position, 0);
				send_msg(msg);
			}

			// go to estimation
			FSM.state = STATE_ESTIMATING;
			FSM.hook = state_estimating;
			time_after_measuring = wb_robot_get_time();
			navigation_sampling_time_sum += time_after_measuring - time_before_measuring;
			printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
		}
	}
	if (simulation_parameters.samplingMode == SENSE_WITHOUT_STOP)
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
		// log and save
		if (VERBOSE)
			printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
		log_position();
		log_measurements();
		if (!position_outside_arena(current_position))
		{
			history.position.push_back({current_position.x, current_position.y, 0});
			history.measurement.push_back(sampleBuffer.average_concentration);
			history.gas_hit_event.push_back(0);
			history.FSM_iter.push_back(FSM.n_cycles);
			history.size++;
			history_print(history);
		}
		// send it to friends
		if (simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR)
		{
			Message msg(ID, 1, history.position[(history.size) - 1], sampleBuffer.average_concentration, goal_position, 0);
			send_msg(msg);
		}

		// go to estimation
		FSM.state = STATE_ESTIMATING;
		FSM.hook = state_estimating;
		time_after_measuring = wb_robot_get_time();
		navigation_sampling_time_sum += time_after_measuring - time_before_measuring;
		printf("navigation sampling time: %f\n", navigation_sampling_time_sum);
	}
}

void state_estimating()
{
	if (VERBOSE)
		printf("[%d] estimating...\n", ID);

	long long time_before_estimating = current_timestamp();

	static long long dummy_est_sum = 0;

	// reinitialize
	posterior.pdf_reset();
	// estimation
	if (simulation_parameters.MCMC_or_exhaustive_eval)
	{
		printf("going for MCMC\n");
		posterior.MCMC_evaluation(history);
	}
	else
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
	posterior.process();
	log_expected_a_posteriori();
	log_max_a_posteriori();
	log_entropy();
	posterior.log_pdf(posterior_pdf_3D_name, 'a');

	FSM.n_cycles++;
	if (VERBOSE)
		printf("[%d] cycle number %d\n", ID, FSM.n_cycles);
	if (VERBOSE)
		printf("[%d] entropy : %f\n", ID, posterior.entropy);

	char label[255];
	sprintf(label, "Iteration: %d\n", FSM.n_cycles);
	wb_supervisor_set_label(2, label, 0.1, 0.8, 0.1, 0x61A8E8, 0, "Impact");
	char label2[255];
	sprintf(label2, "Reading: %f\n", sampleBuffer.average_concentration);
	wb_supervisor_set_label(1, label2, 0.1, 0.85, 0.1, 0xFF0000, 0, "Impact");
	char label3[255];
	sprintf(label3, "Entropy: %.3f\n", posterior.entropy);
	wb_supervisor_set_label(3, label3, 0.1, 0.9, 0.1, 0xFFD966, 0, "Impact");

	long long current_estimation_time = current_timestamp() - time_before_estimating;
	// printf("Current estimation time is : %lld\n", current_estimation_time);

	estimating_time_sum += current_estimation_time;
	// printf("estimating_time_sum : %lld\n", estimating_time_sum);

	// check if time is up
	// if(FSM.n_cycles == MAX_ITER || posterior.entropy < 0.1 || posterior.dirac_flag){
	if (FSM.n_cycles == MAX_ITER || posterior.entropy < 0.1)
	{
		// the end
		FSM.state = STATE_FINISHED;
		// log the finish time stamp
		end_timestamp = wb_robot_get_time();
		printf("end time stamp: %f\n", end_timestamp);
		printf("Current state is STATE_FINISHED\n");
	}
	else
	{
		FSM.state = STATE_UPDATEGOAL;
		FSM.hook = state_updategoal;
	}
}

void state_updategoal()
{
	fprintf(console_logfile, "FSM iter: %d \n", FSM.n_cycles);
	if (VERBOSE)
		printf("[%d] updating the goal\n", ID);
	switch (simulation_parameters.planningStrategy)
	{
	case GREEDY:
		greedy_planning();
		break;
	case BALANCE_MAXIMAL_EXPECTED:
		balance_expected_maximal_pdf();
		break;
	case RANDOM:
		random_planning();
		break;
	case LAWNMOWER:
		lawnmower_planning();
		break;
	case BELIEF_CLUSTERING:
		belief_clustering_planning();
		break;
	case INFORMATIVE_KLD:
		information_gain_navigation_approx();
		KLD_MAP_distance(1.0);
		break;
	default:
		break;
	}
	if (FSM.state != STATE_FINISHED)
	{
		goal_outside_obstacles(&goal_position);
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
		// move to the goal position
		if(simulation_parameters.simuMode == ONLINE){
			// update goal
			FSM.state = STATE_MOVING;
			FSM.hook = state_moving;
			time_before_moving = wb_robot_get_time();
		} else {
			FSM.state = STATE_READ_ALL;
			FSM.hook  = state_read_positions_measurements;
		}
		
	}
}

void state_moving()
{
	// printf("moving toward the goal\n");
	if (simulation_parameters.samplingMode == CONTINOUS_SAMPLING)
	{
		sample_counter++;
		// sample and run kernel
		double instant_concentration = 0.0;
		double odor_concentration = 0.0;
		double reverse_sensor_reading = 0.0;
		odor_read(&instant_concentration, odor_concentration);
		if (simulation_parameters.processingMode == DECONVOLUTION || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			double Ts = TIME_STEP / 1000.0; // sampling frequency
			double tau_r = 0, tau_d = 0; 
			if (simulation_parameters.gasSensor == MOX || simulation_parameters.gasSensor == NOISY_MOX){
				tau_r = 0.72;   // time constant for the rise (our default sensor)		  
			      tau_d = 0.92;   // time constant for the decay		  
			}
			if (simulation_parameters.gasSensor == SLOW_MOX || simulation_parameters.gasSensor == NOISY_SLOW_MOX){
				tau_r = 2.06;   // time constant for the rise (slow sensor)	  
			      tau_d = 4.53;   // time constant for the decay	  
			}
			if (simulation_parameters.gasSensor == VERY_SLOW_MOX || simulation_parameters.gasSensor == NOISY_VERY_SLOW_MOX){
				tau_r = 5.19;   // time constant for the rise (very slow MOX TGS2600)	  
			      tau_d = 19.74;  // time constant for the decay	  
			}
			double alpha = 0.0;
			if (instant_concentration > previous_sensor_reading) // rising phase
				alpha = tau_r / Ts;
			else // decaying phase
				alpha = tau_d / Ts;
			reverse_sensor_reading = (1 + alpha) * instant_concentration - alpha * previous_sensor_reading;
			// printf("reverse_sensor_reading %f\n", reverse_sensor_reading);
			// printf("instant_concentration %f\n", instant_concentration);
			// printf("odor_concentration %f\n", odor_concentration);
			previous_sensor_reading = instant_concentration;
			instant_concentration = reverse_sensor_reading;
		}
		if (simulation_parameters.processingMode == KERNEL_MAPPING || simulation_parameters.processingMode == MAPPING_AND_DECONVOLUTION)
		{
			gas_map.runKernelAlgorithm(current_position, instant_concentration);
		}
		if (sample_counter > simulation_parameters.sampling_period)
		{
			if (!position_outside_arena(current_position))
			{
				// Sampling distance threshold
				float dist_to_last_measurement = sqrt(pow(current_position.x - last_measurement_pos.x, 2) + pow(current_position.y - last_measurement_pos.y, 2));
				if ((sample_in_motion.size == 0) || dist_to_last_measurement > simulation_parameters.sampling_dist_thresh)
				{
					sample_in_motion.position.push_back({current_position.x, current_position.y, 0});
					sample_in_motion.measurement.push_back(instant_concentration);
					sample_in_motion.size++;
					ground_truth_in_motion.push_back(odor_concentration);
					last_measurement_pos.x = current_position.x;
					last_measurement_pos.y = current_position.y;
					// printf("Distance between measurements = %f", dist_to_last_measurement);
					// history.position.push_back({current_position.x, current_position.y, 0});
					// history.measurement.push_back(instant_concentration);
					// history.size++;
				}
			}
			sample_counter = 0;
		}
	}
	// verify if arrived to the goal if applicable
	// fprintf(console_logfile, "check dist to end goal \n");
	// fprintf(console_logfile,"my position (%f, %f) - my end goal (%f, %f)\n", current_position.x, current_position.y, goal_position.x, goal_position.y);
	if (dist_to_goal(goal_position) > 0.05)
	{

		if (i == path.end())
		{
			fprintf(console_logfile, "path end goal reached \n");
		}

		// move
		if (dist_to_goal(*i) <= 0.05 && i != path.end())
		{
			fprintf(console_logfile, "my position (%f, %f) - my i goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
			i++;
			// printf("my position (%f, %f) - my goal (%f, %f)\n", current_position.x, current_position.y, i->x, i->y);
		}
		go_to_goal(*i);  
		// obstacle avoidance if necessary
		//  if(obstacle_or_not() == 1){
		//  	FSM.state = STATE_OBSTACLE_AVOIDANCE;
		//  	FSM.hook  = state_obstacle_avoidance;
		//  	return;
		//  }
		set_wheels_speed(speed[0], speed[1]);
	}
	else
	{

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
	}
}

void state_obstacle_avoidance()
{
	// printf("avoiding obstacle\n");
	obstacle_avoidance_k4();
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
	char label[255];
	sprintf(label, "Iteration: %d\n", 0);
	wb_supervisor_set_label(2, label, 0.1, 0.8, 0.1, 0x61A8E8, 0, "Impact");
	char label2[255];
	sprintf(label2, "Reading: %d\n", 0);
	wb_supervisor_set_label(1, label2, 0.1, 0.85, 0.1, 0xFF0000, 0, "Impact");

	// Precompute gas concentration maps for all possible source locations, resolution 20x20
	plume_model.initialize_information_matrix();
	plume_model.log_gas_concentration_map(gas_concentration_map_logfile_name, 'a');

	// File for saving console messages
	console_logfile = fopen(console_logfile_name, "a");

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

	// Quickfix to have correct plots with mode fixed th
	if (simulation_parameters.gasFeature == GASHIT)
	{
		double mean, stddev;
		double sample[2];
		int indices[2];
		sample[0] = plume_model.expected_value[0];
		sample[1] = plume_model.expected_value[1];

		indices[0] = floor((sample[0] - source_x_min) / x_resol);
		indices[1] = floor((sample[1] - source_y_min) / y_resol);
		int source_i = 0;

		source_i = posterior.indices_to_index(indices);
		// fprintf(console_logfile, "exp x: %f\n", sample[0] );
		// fprintf(console_logfile, "exp y: %f\n", sample[1] );
		// fprintf(console_logfile, "source index: %d\n", source_i);

		calculate_data_distribution(history.measurement, mean, stddev);
		likelihood_gas_hit(history, sample, source_i, &plume_model, mean, stddev);
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
