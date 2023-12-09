#ifndef KHEPERA_FUNCTIONS_H
#define KHEPERA_FUNCTIONS_H


#include "controller_STE_multi-robot_Gaussian_plume.h"


#include <sys/stat.h>
#include <sys/time.h>
#include <iostream>
#include <algorithm>
#include <vector>
//#include <eigen3/Eigen/Dense>

// extern "C" {
//     #include "khepera4.h"
//     #include "odorboard.h"
//     #include "windsensorboard.h"
//     #include "odometry_track.h"
//     #include "odometry_goto.h"
// }
#include "udp_msg.h"
//#include "wind_filter.h"
// #include "vector3d.h" //defines the sVector3d type used in wind read
#include "Message.h"

// #include "cortex.h"
// #include "CortexHandler.h"

#define WEBOTS 0
#define KHEPERA 1
#define PUMP_SPEED        0     //1024=stop, 0=max
#define COMPUTERID  30
#define SLEEP_PERIOD      50


// extern struct sKhepera4 khepera4;
// extern struct sOdorboard odorboard;
// extern struct sWindSensorBoard windsensorboard;

// extern struct sOdometryTrack ot;

// log files
extern char folder_name[25];
extern FILE* goalposition_logfile;
extern FILE* measurements_logfile;
extern FILE* posterior_logfile;
extern FILE* posteriorpdf_logfile;
extern FILE* expected_a_posteriori_logfile;
extern FILE* likelihood_logfile;
extern FILE* measurement_history_logfile;
extern FILE* position_history_logfile;
extern FILE* max_a_posteriori_logfile;
extern FILE* posterior_pdf_3D;
extern FILE* entropy_logfile;
extern FILE* tmp_file;
extern FILE* config_logfile;
extern FILE* gas_information_map_logfile;
extern FILE* gas_hit_variance_map_logfile;
extern FILE* experiment_time_logfile;
extern char goalposition_logfile_name[256];
extern char measurements_logfile_name[256];
extern char posterior_logfile_name[256];
extern char posteriorpdf_logfile_name[256];
extern char expected_a_posteriori_logfile_name[256];
extern char max_a_posteriori_logfile_name[256];
extern char likelihood_logfile_name[256];
extern char measurement_history_logfile_name[256];
extern char position_history_logfile_name[256];
extern char posterior_pdf_3D_name[256];
extern char entropy_logfile_name[256];
extern char config_logfile_name[256];
extern char gas_information_map_logfile_name[512];
extern char gas_hit_variance_map_logfile_name[512];
extern char experiment_time_logfile_name[512];
// read logfile from the experiment
extern char logfile_folder_name[256];
extern FILE* experiment_measurements_logfile;
extern char experiment_measurements_logfile_name[300];
extern FILE* experiment_goalposition_logfile;
extern char experiment_goalposition_logfile_name[300];
extern FILE* experiment_position_logfile;
extern char experiment_position_logfile_name[300];
// read scan file
extern char scan_folder_name[256];
extern FILE* scan_measurements_logfile;
extern char scan_measurements_logfile_name[300];
extern FILE* scan_position_logfile;
extern char scan_position_logfile_name[300];

// finite state machine
extern struct FSM FSM;
// robot's measurements
extern struct measurements measurements;

extern Position goal_position; //position in m
extern Position current_position; //position in m
extern double speed[2];
// robot's position and measurement history
extern struct history history;
//wind vector
// extern sVector3d wind_vect;

//robot's ID
extern int ID;

extern int msg_counter;

//message structure
extern struct message message;

//list of other robots
extern struct friends friends[MAX_NB_ROBOTS];

extern int n_time_step;

extern struct simulation_parameters simulation_parameters;

extern double odor_baseline_value;

extern std::vector<int> received_msgs;

typedef struct {
  struct {
    int robot_id;
  } header;
  struct {
    float x;
    float y;
    float angle;
  } position;
} sMsg;


void init();
void initCommSwisTrack();
int initCortexComm();
void initCommRobot();
void establish_baseline();
double odor_read();
double wind_read();
void update_position();
void updateSwisTrack(double position[3]);
void updateCortexPos(double position[3]);
int obstacle_or_not();
void receive_from_friends();
double get_exp_time();
void send_msg(Message msg); 
int receive_msg(Message * msg);
void go_to_goal();
void obstacle_avoidance(double speed[2]);
void set_wheels_speed(double speed1, double speed2);
void cleaning_up();
void next_iteration_time();


#endif