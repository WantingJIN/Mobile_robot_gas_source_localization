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
#include <array>

#include "odor_filter.h"
#include "Message.h"
#include "SampleBuffer.h"
#include "visilibity.hpp"
#include "gasMap2D.h"
#include "SimulationParameters.h"
#include "Dbscan.h"

class PlumeModel;

//states of the finite state machine
typedef enum state {
	STATE_UNKNOWN		=	-1,
	STATE_WAITFORPLUME	=   0,
	STATE_MEASURING 		=	4,
	STATE_ESTIMATING		=	1,
	STATE_UPDATEGOAL		=	2,
	STATE_MOVING    		=	3,
	STATE_FINISHED    	=	5,
	STATE_OBSTACLE_AVOIDANCE = 6,
	STATE_LISTEN = 7,
	STATE_RECEIVE_GOAL = 9,
	STATE_RECEIVE_PDF = 10,
	STATE_SEND_GOAL = 11,
	STATE_SEND_PDF = 12,
	STATE_MERGE_PDF = 13,
	STATE_UPDATEGOAL_TEAM = 14,
	STATE_SHARE_PDF = 15,
	STATE_REGION_ALLOCATION = 16,
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
	vector<float> gas_meas_avg;
	vector<float> gas_meas_std;
	vector<float> wind_intensity;
	vector<float> wind_angle;
	vector<float> wind_meas_u;
	vector<float> wind_meas_v;
	vector<Position> team_wind_position;
	vector<float> team_wind_meas_u;
	vector<float> team_wind_meas_v;
	vector<int> team_wind_sender_id;
	vector<int> team_wind_sample_id;
	vector<float> relative_meas;
	vector<int> gas_hit_event;
};


// message structure
struct message{
	int sender_ID;
	float position[3];
	float concentration;
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
void state_receive_goal();
void state_receive_pdf();
void state_send_goal();
void state_send_pdf();
void state_merge_pdf();
void state_share_pdf();
void state_region_allocation();
void state_updategoal_team();
void initialize_plume_stack();
PlumeModel* initialize_plume_model();

float pseudo_gaussian_plume_model(float robot_position[], float wind_intensity, struct parameters_set parameters);
float likelihood(struct history history, struct parameters_set parameters);
float prior(struct parameters_set parameters);

float randu();
long long current_timestamp();
void ack_msg(Message msg);

float target_evaluation(float position[][3]);
void get_gps_position(float external_gps_position[3]);
int obstacle_or_not();
void do_odometry(Position *position);
void odometry_set_absolute_position_from_external(const float ex_x, const float ex_y, const float ex_heading);
float dist_to_goal(Position goal);

bool check_stop_condition();
void broadcast_state_finished();


#endif
