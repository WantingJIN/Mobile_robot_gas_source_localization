#ifndef KHEPERA_FUNCTIONS_H
#define KHEPERA_FUNCTIONS_H


#include "controller_STE_distributed.h"


#include <sys/stat.h>
#include <sys/time.h>
#include <iostream>
#include <algorithm>
//#include <eigen3/Eigen/Dense>

extern "C" {
    #include "khepera4.h"
    #include "odorboard.h"
    #include "trisonica.h"
    #include "odometry_track.h"
    #include "odometry_goto.h"
}
#include "udp_msg.h"
//#include "wind_filter.h"
#include "vector3d.h" //defines the sVector3d type used in wind read
#include "Message.h"
#include "Position.h"
#include "cortex.h"
#include "CortexHandler.h"
#include "visilibity.hpp"
#include "constant_and_helper.h"

#define WEBOTS 0
#define KHEPERA 1
#define PUMP_SPEED        0    //1024=stop, 0=max
#define COMPUTERID  30
#define SLEEP_PERIOD      50
#define CONTROLLER_TIMESTAMP SLEEP_PERIOD*200


extern struct sKhepera4 khepera4;
extern struct sOdorboard odorboard;
extern struct sTrisonica trisonica;
extern struct sBraitenberg braitenberg;

extern struct sOdometryTrack ot;

// log files
extern char folder_name[256];
extern FILE* baseline_measurements_logfile;

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
extern char trajectory_logfile_name[256];
extern char baseline_measurements_logfile_name[256];
extern char visited_list_logfile_name[256];

extern char clustering_bounding_boxes_logfile_name[512];
extern char scores_map_logfile_name[512];
extern char distance_map_logfile_name[512];
extern char acc_info_map_logfile_name[512];
extern char console_logfile_name[512];
extern char gas_information_map_logfile_name[512];
extern char gas_concentration_map_logfile_name[512];
extern char experiment_time_logfile_name[512];


// finite state machine
extern struct FSM FSM;

extern Position goal_position; //position in m
extern Position current_position; //position in m
extern float speed[2];
// robot's position and measurement history
extern struct history history;
//wind vector
extern sVector3d wind_vect;

//robot's ID
extern int ID;
extern int self_index;

//message structure
extern struct message message;

//list of other robots
extern struct friends friends[MAX_NB_ROBOTS];
extern vector<int> ID_list; // IDs of the robots

extern int n_time_step;

extern float odor_baseline_value;

extern std::vector<int> received_msgs;

extern std::list<Position> path;
extern std::list<Position>::iterator i;
// extern VisiLibity::Environment my_environment;
// extern VisiLibity::Visibility_Graph vg();
extern std::string environment_file;
extern VisiLibity::Environment my_environment;
extern double epsilon;
extern VisiLibity::Visibility_Graph vg;
// set the odometry initial pos from last frame MCS result
extern bool set_odometry_pos_from_MCS;
extern bool MCS_data_available;


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

struct sBraitenberg {
    struct {
        int left;
        int right;
    } speed;
    int repeat;
    int wait_us;
    int weight_left[12];
    int weight_right[12];
    int bias[12];
};



void init();
void initCommSwisTrack();
int initCortexComm();
void initCommRobot();
void set_logfile_names();
void establish_baseline();
float odor_read();
void wind_read(float* wind_intensity, float* wind_angle);
void update_position();
void updateSwisTrack(float position[3]);
void updateCortexPos(Position *position);
int obstacle_or_not();
void receive_from_friends();
double get_exp_time();
void send_udp_msg(Message msg); 
int receive_udp_msg(Message * msg);
int find_index_in_ID_list(int ID, std::vector<int> &ID_list);

void go_to_goal(Position goal);
void obstacle_avoidance(float speed[2]);
void set_wheels_speed(float speed1, float speed2);
void cleaning_up();
void next_iteration_time();
void log_baseline(float min_c, float max_c, float baseline_c);
float calibrate_sensor_reading (float raw_reading);


void braitenberg_run(sBraitenberg *braitenberg);
void braitenberg_set_obstacle_avoidance(sBraitenberg *braitenberg, int aggressivity);
void braitenberg_init(sBraitenberg *braitenberg);

void reverse_sensor_model(float &sensor_reading, float &previous_sensor_reading);

#endif