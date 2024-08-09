#ifndef CONTROLLER_STE_MULTI_ROBOT_H
#define CONTROLLER_STE_MULTI_ROBOT_H

// C libraries
#include <assert.h>
#include <math.h>
#include <stdio.h>    //printf
#include <stdlib.h>   //srand
#include <sys/stat.h> // mkdir
#include <sys/time.h>
#include <time.h> //time
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib> // for getenv
#include "PlumeModel.h"
#include "SampleBuffer.h"
#include "Position.h"
#include "gasMap2D.h"
#include "Constant.h"
#include "Dbscan.h"

extern int nbr_obs;
extern float margin;
extern float **obstacles;
// extern float obstacles[][4];

// Mode of sampling
typedef enum samplingMode
{
	CONTINOUS_SAMPLING = 0,
	STOP_AND_GO_SAMPLING = 1,
	SENSE_WITHOUT_STOP = 2,
} eSamplingMode;

// Mode of Processing;
// in order to composente the slow reaction/recovery time of sensing
typedef enum processingMode
{
	NO_PROCESSING = 0,
	KERNEL_MAPPING = 1,
	DECONVOLUTION = 2,
	MAPPING_AND_DECONVOLUTION = 3,
} eProcessingMode;

typedef enum gasSensor
{
	PID = 0,
	MOX = 1,
	NOISY_MOX = 2,
	SLOW_MOX = 3,
	VERY_SLOW_MOX = 4,
	NOISY_SLOW_MOX = 5,
	NOISY_VERY_SLOW_MOX = 6,
} eGasSensor;

typedef enum planningStrategy
{
	GREEDY = 0,
	BALANCE_MAXIMAL_EXPECTED = 2,
	RANDOM = 3,
	LAWNMOWER = 4,
	BELIEF_CLUSTERING = 5,
	INFORMATIVE_KLD = 6,
} ePlanningStrategy;

typedef enum gasFeature
{
	//CONCENTRATION = 0,
	ABSOLUTE_VALUE = 0,
	GASHIT = 1,
	//RELATIVE_CONCENTRATION = 2,
	RELATIVE_VALUE = 2,
	PDF = 3,
} eGasFeature;

typedef enum visitedList
{
	visitedList_ON = 0,
	visitedList_OFF = 1,
} eVisitedList;

typedef enum infoFeature
{
	VARIANCE = 0,
	MEAN = 1,
} eInfoFeature;

typedef enum relativeMethod
{
	NORMALIZATION = 0,
	GAUSSIAN_CDF = 1,
	M_DISTANCE = 2,
	EMPIRICAL_CDF = 3,
} eRelativeMethod;

typedef enum simuMode{
	ONLINE = 0,
	OFFLINE = 1,
}eSimuMode;

typedef enum calibrationMode
{
	OFF = 0,
	LINEAR = 1,
	NOLINEAR = 2,
} eCalibrationMode;

typedef enum gasHitMethod
{
	FIXED_THRES = 0,
	MIN_MAX_THRES = 1,
	SLIDING_WINDOW_THRES = 2,
	FIXED_THRES_AND_GAUSSIAN_CDF = 3,
} eGasHitMethod;

// parameters to estimate
struct parameters_set
{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	// double a_z;
	// double b_z;
};

// simulation paramters
struct simulation_parameters
{
	float likelihood_error;  //1
	float gloabl_local_search_ratio; //2
	int MCMC_iteration_num; //3
	int MCMC_or_exhaustive_eval; // 4: 1 for MCMC, 0 for exhaustive evaluation
	int number_of_robots; //5
	int scenario; //6
	int map_nbr; //7
	eSamplingMode samplingMode; //8
	eProcessingMode processingMode; //9
	eGasSensor gasSensor; //10
	ePlanningStrategy planningStrategy; //11
	eGasFeature gasFeature; //12
	eVisitedList visitedList; //13
	eInfoFeature infoFeature; //14
	int sampling_period; //15
	float cluster_entropy_ratio; //16
	float vel_factor; //17
	float wind_factor; //18
	float sampling_dist_thresh; //19
	eRelativeMethod relativeMethod; //20
	eSimuMode simuMode; //21
	eCalibrationMode calibrationMode; //22
	eGasHitMethod gasHitMethod; //23
};

extern struct simulation_parameters simulation_parameters;

// states of the finite state machine
typedef enum state
{
	STATE_UNKNOWN = -1,
	STATE_WAITFORPLUME = 0,
	STATE_MEASURING = 4,
	STATE_ESTIMATING = 1,
	STATE_UPDATEGOAL = 2,
	STATE_MOVING = 3,
	STATE_FINISHED = 5,
	STATE_OBSTACLE_AVOIDANCE = 6,
	STATE_READ_ALL = 7,
	STATE_READ_INITIAL = 8,
} eState;

// finite state machine
struct FSM
{
	int n_cycles;
	eState state;
	void (*hook)();
};
extern struct FSM FSM;

// measurement history
struct history
{
	int size;
	vector<int> FSM_iter;
	// Position position[MAX_SAMPLE_NUMBER*MAX_NB_ROBOTS];
	// double measurement[MAX_SAMPLE_NUMBER*MAX_NB_ROBOTS];
	vector<Position> position;
	vector<double> measurement;
	vector<int> gas_hit_event;
};

// message structure
struct message
{
	int sender_ID;
	double position[3];
	double concentration;
	int termination_flag;
};

// info from other robots
struct friends
{
	int ID;
	Position position;
	Position goal;
};

// functions
void state_measuring();
void state_estimating();
void state_updategoal();
void state_moving();
void state_waiting();
void state_obstacle_avoidance();
void state_read_positions_measurements();
void state_read_initial_position_measurement();

// old
double likelihood(struct history history, double *parameters, PlumeModel *model);
// new likelihood with precomputed gas maps
double likelihood_precomp(struct history history, double *parameters, int soure_pos_idx, PlumeModel *model);
// new likelihood for relative concentrations
double likelihood_rel_con(struct history &history, vector<double> rel_history_meas, double *parameters, int soure_pos_idx, PlumeModel *model);
double likelihood_gas_hit(struct history &history, double *parameters, int soure_pos_idx, PlumeModel *model, double mean, double stddev);
double likelihood_pdf(struct history &history, vector<double> pdf_history_list, double *parameters, int soure_pos_idx, PlumeModel *model);
bool prior(double *parameters, int nbr_dimension, PlumeModel *model);

double randu();
long long current_timestamp();
double dist_to_goal(Position goal);
void goal_outside_obstacles(Position *goal);

// double target_evaluation(double position[][3]);
// void get_gps_position(Position external_gps_position);
int obstacle_or_not();
void do_odometry();

// calculate the Gaussian distribution of the history and the CDF of the data
double calculate_measurement_gaussian_CDF(double data, double mean, double stddev);
double calculate_measurement_M_distance(double data, double mean, double stddev);
void calculate_measurements_list_gaussian_CDF(vector<double> &rel_meas_list, vector<double> data_list, double mean, double stddev);
void calculate_measurements_list_empirical_CDF(vector<double> &rel_meas_list, vector<double> data_list);
void calculate_measurements_list_M_distance(vector<double> &rel_meas_list, vector<double> data_list, double mean, double stddev);
void calculate_data_distribution(vector<double> data, double &mean, double &stddev);
void calculate_normalized_relative_measurements(vector<double> &rel_meas_list, vector<double> data_list);
void calculate_measurement_list_probablistic_distribution(vector<double> &pdf_meas_list, vector<double> data_list);

#endif