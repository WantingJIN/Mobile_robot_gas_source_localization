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
#include "constant_and_helper.h"
#include "Dbscan.h"
#include "SimulationParameters.h"


// states of the finite state machine
typedef enum state
{
	STATE_UNKNOWN = -1,
	STATE_WAITFORPLUME = 0,
	STATE_ESTIMATING = 1,
	STATE_UPDATEGOAL = 2,
	STATE_FINISHED = 3,
	STATE_OBSTACLE_AVOIDANCE = 4,
	STATE_LISTENING = 5,
	STATE_INITIALIZING = 6,
	STATE_READ_ALL = 7,
} eState;

// finite state machine
struct FSM
{
	int n_cycles;
	int n_relocations;
	eState state;
	void (*hook)();
};
extern struct FSM FSM;

// measurement history
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
void state_estimating();
void state_updategoal();
void state_waiting();
void state_obstacle_avoidance();
void state_initializing();
void state_listening();
void state_read_goal_measurement();

void read_goal_position();


PlumeModel* initializePlumeModel();

bool check_stop_condition();
bool check_converge_condition();

long long current_timestamp();

// double target_evaluation(double position[][3]);
// void get_gps_position(Position external_gps_position);
int obstacle_or_not();
void do_odometry();

void information_clustering_planning();

void broadcast_state_finished();

void set_source_bounds_centered_with_location(SourceBounds *bounds, double px, double py);
void set_source_esimation_grid(SourceBounds *bounds, int x_bins, int y_bins);
void set_source_estimation_range(SourceBounds *bounds, double x_min, double x_max, double y_min, double y_max);
std::vector<Position> load_goals_from_file(std::string filename);
#endif