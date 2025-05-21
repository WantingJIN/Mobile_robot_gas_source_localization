#ifndef CONTROLLER_STE_DISTRIBUTED_H
#define CONTROLLER_STE_DISTRIBUTED_H


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
#include <vector>

#include "odor_filter.h"
#include "Message.h"
#include "SampleBuffer.h"
#include "visilibity.hpp"
#include "gasMap2D.h"
#include "SimulationParameters.h"


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
  STATE_LISTEN = 7,
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
	vector<int> FSM_iter;
	vector<Position> position;
	vector<double> measurement;
	vector<double> measurement_std;
	vector<int> relative_meas;
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

extern std::list<Position> path;
extern std::list<Position>::iterator i;
// extern VisiLibity::Environment my_environment;
// extern VisiLibity::Visibility_Graph vg();
extern std::string environment_file;
extern VisiLibity::Environment my_environment;
extern double epsilon;
extern VisiLibity::Visibility_Graph vg;
extern float ** obstacles;
extern SimulationParameters simulation_parameters;

// functions
void state_measuring();
void state_estimating();
void state_updategoal();
void state_moving();
void state_waiting();
void state_obstacle_avoidance();
void state_listen();

double pseudo_gaussian_plume_model(double robot_position[], double wind_intensity, struct parameters_set parameters);
double likelihood(struct history history, struct parameters_set parameters);
double prior(struct parameters_set parameters);

double randu();
long long current_timestamp();
void ack_msg(Message msg);

double target_evaluation(double position[][3]);
void get_gps_position(double external_gps_position[3]);
int obstacle_or_not();
void do_odometry(Position *position);
void odometry_set_absolute_position_from_external(const float ex_x, const float ex_y, const float ex_heading);
double dist_to_goal(Position goal);


#endif