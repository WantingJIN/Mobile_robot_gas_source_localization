#ifndef CONTROLLER_STE_CLEAN_H
#define CONTROLLER_STE_CLEAN_H




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
#include "Constant.h"
#include "Dbscan.h"

extern int nbr_obs;
extern float margin;
extern float ** obstacles;

//parameters to estimate
struct parameters_set{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	//double a_z;
	//double b_z;
};
typedef enum planningStrategy{
	GREEDY = 0,
	INFORMATIVE_KLD = 1,
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
	BELIEF = 2,
}einfoFeature;

typedef enum workingMode{
	OFFLINE = 0,
	EXPERIMENT = 1,
}eworkingMode;

// simulation paramters
struct simulation_parameters{
	eworkingMode workingMode;
	float likelihood_error;
	float gloabl_local_search_ratio;
	int MCMC_iteration_num;
	int MCMC_or_exhaustive_eval; //1 for MCMC, 0 for exhaustive evaluation
	int number_of_robots;
	int scenario;
	int map_nbr;
	ePlanningStrategy planningStrategy;
	eGasFeature gasFeature;
	eVisitedList visitedList;
	einfoFeature infoFeature;
	float cluster_entropy_ratio;
	float measurement_error;
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
  STATE_LISTEN			=	7,
  STATE_READ            =   8,
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
	vector<Position> position;
	vector<double> measurement_avg;
	vector<double> measurement_std;
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
void state_listen();
void state_read();

double pseudo_gaussian_plume_model(double robot_position[], double wind_intensity, struct parameters_set parameters);
double likelihood(struct history history, double *parameters, PlumeModel *model, bool verbose);
double likelihood(struct history history, double *parameters, vector<float> model_c_avg);
bool prior(double *parameters, int nbr_dimension, PlumeModel *model);

double randu();
long long current_timestamp();
double dist_to_goal(Position goal);
bool send_goal_to_robot(double x, double y);

//double target_evaluation(double position[][3]);
//void get_gps_position(Position external_gps_position);
int obstacle_or_not();
void do_odometry();


#endif