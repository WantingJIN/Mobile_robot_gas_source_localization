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
#include "SimulationParameters.h"

extern int nbr_obs;
extern float margin;
extern float **obstacles;
extern SimulationParameters simulation_parameters;
// extern float obstacles[][4];

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
	STATE_RECEIVE_GOAL = 9,
	STATE_RECEIVE_PDF = 10,
	STATE_SEND_GOAL = 11,
	STATE_SEND_PDF = 12,
	STATE_MERGE_PDF = 13,
	STATE_UPDATEGOAL_TEAM = 14,
	STATE_SHARE_PDF = 15,
	STATE_REGION_ALLOCATION = 16,
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
	vector<double> relative_meas;
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
void state_receive_goal();
void state_receive_pdf();
void state_send_goal();
void state_send_pdf();
void state_merge_pdf();
void state_updategoal_team();
void state_share_pdf();
void state_region_allocation();

double randu();
long long current_timestamp();
double dist_to_goal(Position goal);

// double target_evaluation(double position[][3]);
// void get_gps_position(Position external_gps_position);
int obstacle_or_not();
void do_odometry();

void information_clustering_planning();

void broadcast_state_finished();
#endif