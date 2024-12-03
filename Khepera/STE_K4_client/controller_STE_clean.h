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
#include <vector>

#include "odor_filter.h"
#include "Message.h"
#include "SampleBuffer.h"
#include "visilibity.hpp"
#include "gasMap2D.h"

// Sampling mode


//Constant variables
#define BOUND(x, a, b) (((x)<(a))?(a):((x)>(b))?(b):(x))
#define PI 					3.14159265358979
#define TIME_STEP 			64 //adjusts the speed (ms)
#define SAFETY_MARGIN 		.2 // how far in meters are you ready to approach the walls
#define WHEEL_RADIUS	        0.0205	  // Wheel radius (meters)
#define AXLE_LENGTH             0.08841 
#define ODOR_FILTER_LENGTH 	50 //~10sec //number of measurements before averaging
//#define ODOR_FILTER_LENGTH 	1 //~10sec //number of measurements before averaging
#define MAX_SAMPLE_NUMBER 	50
#define N_CONCENTRATION_LEVEL 4
#define MAX_SPEED 			35748.4
#define SENSOR_NUMBER  		8
#define MAX_NB_ROBOTS       3


//wind tunnel boundaries
#define XMIN 0.25
#define YMIN 0.15
//#define ZMIN 0.1
#define XMAX 10.25 //12.5
#define YMAX 3.85
//#define ZMAX 1.9

// min and max for each parameter
#define Q_min  1
#define Q_max  800
#define source_x_min  0.25
#define source_y_min  0
//#define source_z_min  0
#define source_x_max  10.25 //12.5
#define source_y_max  4
//#define source_z_max  2
#define a_y_min  0.01//0.005//0.01
#define a_y_max  0.2//0.015//0.2
#define b_y_min  0.1//0.05//0.1
#define b_y_max  1.0//0.15//1.0
//#define a_z_min  0.01
//#define a_z_max  0.05
//#define b_z_min  0.1
//#define b_z_max  0.5

// number of bins for posterior PDF
#define Q_nbins   10 //(Q_max - Q_min) /20
#define x_nbins   40 //(source_x_max - source_x_min)/0.1 //0.1 m is the granuality
#define y_nbins   20 //(source_y_max - source_y_min)/0.1 //0.1 m is the granuality
//#define z_nbins   10 //(source_z_max - source_z_min)/0.1 //0.1 m is the granuality
#define a_y_nbins   10 //(a_y_max - a_y_min) / 0.0025
#define b_y_nbins   10 //(b_y_max - b_y_min) / 0.025
//#define a_z_nbins   10 //(a_z_max - a_z_min) / 0.0025
//#define b_z_nbins   10 //(b_z_max - b_z_min) / 0.025

#define MAX_DISTANCE 10.770329614 //from (2.5, 0) to (12.5,4)

#define VERBOSE 		1

#define NO_COM_NO_COOR	0
#define COM_NO_COOR		1
#define COM_COOR 		2
#define SCENARIO 		NO_COM_NO_COOR


//parameters to estimate
struct parameters_set{
	double Q;
	double source_position[2];
	double a_y;
	double b_y;
	//double a_z;
	//double b_z;
};

typedef enum samplingMode{
	STOP_AND_SENSE = 0,
	SENSE_WITHOUT_STOP = 1,
	CONTINOUS_SAMPLING = 2,
}eSamplingMode;

typedef enum processingMode{
	NO_PROCESSING = 0,
	KERNEL_INTERPOLATION = 1,
	REVERSE_SENSOR_MODEL = 2,
}eProcessingMode;

typedef enum sensorCalibration{
	ON = 0,
	OFF = 1,
}eSensorCalibration;

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
	eSensorCalibration sensorCalibration;
	int sampling_period;
	int wheel_speed;
	float sampling_dist_thresh;
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
	vector<Position> position;
	vector<double> measurement_avg;
	vector<double> measurement_std;

};

// robot's measurements
struct measurements{
	double wind_angle;
	double wind_intensity;
	sMaxFilter wind_sample_memory;
	sMaxFilter odor_sample_memory;
	double instant_concentration;
    double average_concentration;
    int n_samples;
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