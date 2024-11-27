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
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib> // for getenv
#include "PlumeModel.h"
#include "SampleBuffer.h"
#include "Position.h"
#include "Constant.h"
#include "Dbscan.h"
#include "SimulationParameters.h"

extern int nbr_obs;
extern float margin;
extern float ** obstacles;

// simulation paramters

extern SimulationParameters simulation_parameters;

//parameters to estimate
struct parameters_set{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	//double a_z;
	//double b_z;
};

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
  STATE_READ_ALL            =   8,
  STATE_READ_GOAL           =   9,
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
void state_read_goal();
void state_read_goal_measurement();

double pseudo_gaussian_plume_model(double robot_position[], double wind_intensity, struct parameters_set parameters);
double likelihood(struct history history, double *parameters, PlumeModel *model, bool verbose);
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