#ifndef CONTROLLER_STE_MULTI_ROBOT_H
#define CONTROLLER_STE_MULTI_ROBOT_H

//C libraries
#include <assert.h>
#include <math.h>
#include <stdio.h> //printf
#include <stdlib.h> //srand
#include <sys/stat.h> // mkdir
#include <sys/time.h>
#include <time.h> //time 
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include "PlumeModel.h"
#include "SampleBuffer.h"
#include "Position.h"
#include "gasMap2D.h"
#include "Constant.h"
#include "Dbscan.h"

extern int nbr_obs;
extern float margin;
extern float ** obstacles;
// extern float obstacles[][4];

// Mode of sampling 
typedef enum samplingMode {
	CONTINOUS_SAMPLING = 0,
	STOP_AND_GO_SAMPLING = 1,
	SENSE_WITHOUT_STOP = 2,
}eSamplingMode;

// Mode of Processing; 
// in order to composente the slow reaction/recovery time of sensing
typedef enum processingMode {
	NO_PROCESSING = 0,
	KERNEL_MAPPING = 1,
	REVERSE_SENSOR_MODEL = 2,
	MAPPING_AND_REVERSE_SENSOR_MODEL = 3,
}eProcessingMode;

typedef enum gasSensor{
	PID = 0,
	MOX = 1,
	NOISY_MOX = 2,
}eGasSensor;

typedef enum planningStrategy{
	GREEDY = 0,
	INFORMATIVE = 1,
	BALANCE_MAXIMAL_EXPECTED = 2,
	RANDOM = 3,
	LAWNMOWER = 4,
	INFORMATIVE_CLUSTERING = 5,
}ePlanningStrategy;

typedef enum gasFeature{
	CONCENTRATION = 0,
	GASHIT = 1,
}eGasFeature;

typedef enum visitedList{
	visitedList_ON = 0,
	visitedList_OFF = 1,
}eVisitedList;

typedef enum infoFeature{
	VARIANCE = 0,
	MEAN = 1,
}eInfoFeature;


//parameters to estimate
struct parameters_set{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	//double a_z;
	//double b_z;
};

// simulation paramters
struct simulation_parameters{
	float likelihood_error;
	float gloabl_local_search_ratio;
	int MCMC_iteration_num;
	int MCMC_or_exhaustive_eval; //1 for MCMC, 0 for exhaustive evaluation
	int number_of_robots;
	int scenario;
	int map_nbr;
	eSamplingMode samplingMode;
	eProcessingMode processingMode;
	eGasSensor gasSensor;
	ePlanningStrategy planningStrategy;
	eGasFeature gasFeature;
	eVisitedList visitedList;
	eInfoFeature infoFeature;
	int sampling_frequency;
};
extern struct simulation_parameters simulation_parameters;

//states of the finite state machine
typedef enum state {
  STATE_UNKNOWN			=	-1,
  STATE_WAITFORPLUME	=   0,
  STATE_MEASURING 		=	4,
  STATE_ESTIMATING		=	1,
  STATE_UPDATEGOAL		=	2,
  STATE_MOVING    		=	3,
  STATE_FINISHED    	=	5,
  STATE_OBSTACLE_AVOIDANCE = 6,
}eState;

//finite state machine
struct FSM{
    int n_cycles;
    eState state;
    void (*hook)();
};
extern struct FSM FSM; 

//measurement history
struct history{
	int size;
	//Position position[MAX_SAMPLE_NUMBER*MAX_NB_ROBOTS];
	//double measurement[MAX_SAMPLE_NUMBER*MAX_NB_ROBOTS];
	vector<Position> position;
	vector<double> measurement;
	vector<int> gas_hit_event;
};

// message structure
struct message{
	int sender_ID;
	double position[3];
	double concentration;
	int termination_flag;
};

// info from other robots
struct friends{
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

//planning strategy
void informative_path_planning_based_on_variance();

double pseudo_gaussian_plume_model(double robot_position[], double wind_intensity, struct parameters_set parameters);
double likelihood(struct history history, double *parameters, PlumeModel *model);
double likelihood_gas_hit(struct history &history, double *parameters, PlumeModel *model, double mean, double stddev);
bool prior(double *parameters, int nbr_dimension, PlumeModel *model);

double randu();
long long current_timestamp();
double dist_to_goal(Position goal);
void goal_outside_obstacles(Position * goal);

//double target_evaluation(double position[][3]);
//void get_gps_position(Position external_gps_position);
int obstacle_or_not();
void do_odometry();

// calculate the Gaussian distribution of the history and the CDF of the data
double calculate_measurement_gaussian_CDF(double data, double mean, double stddev);
void calculate_history_distribution(struct history history, double &mean, double &stddev);

#endif