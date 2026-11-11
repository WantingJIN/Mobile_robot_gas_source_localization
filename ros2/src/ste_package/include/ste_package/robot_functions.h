#ifndef ROBOT_HEADER_H
#define ROBOT_HEADER_H

#include <random>

#include "ste_node.h"
#include "Message.h"
#include "SimulationParameters.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ste_interfaces/action/navigate_sample.hpp"
#include <list>



#define SLEEP_PERIOD      50 // in ms
// log files
extern char folder_name[256];
extern char goalposition_logfile_name[512];
extern char measurements_logfile_name[512];
extern char expected_a_posteriori_logfile_name[512];
extern char max_a_posteriori_logfile_name[512];
extern char likelihood_logfile_name[512];
extern char measurement_history_logfile_name[512];
extern char position_history_logfile_name[512];
extern char posterior_pdf_3D_name[512];
extern char entropy_logfile_name[512];
extern char config_logfile_name[512];
extern char trajectory_logfile_name[512];
extern char gas_concentration_map_logfile_name[512];
extern char gas_concentration_maps_log_root[512];
extern char gas_information_map_logfile_name[512];
extern char experiment_time_logfile_name[512];
extern char visited_list_logfile_name[512];
extern char clustering_bounding_boxes_logfile_name[512];
extern char scores_map_logfile_name[512];
extern char distance_map_logfile_name[512];
extern char acc_info_map_logfile_name[512];
extern char weighted_belief_map_logfile_name[512];
extern char relative_history_meas_logfile_name[512];
extern char wind_field_u_logfile_name[512];
extern char wind_field_v_logfile_name[512];
extern char wind_field_w_logfile_name[512];

// read logfile from the experiment
extern char logfile_folder_name[256];
extern FILE* experiment_measurements_logfile;
extern char experiment_measurements_logfile_name[512];
extern FILE* experiment_goalposition_logfile;
extern char experiment_goalposition_logfile_name[512];
extern FILE* experiment_position_logfile;
extern char experiment_position_logfile_name[512];

// for storing debugging info
extern FILE *console_logfile;
extern char console_logfile_name[512];

// finite state machine
extern struct FSM FSM;
// robot's measurements
extern struct measurements measurements;

// extern double goal_position[3]; //position in m
extern Position goal_position;
// extern double current_position[3]; //position in m
extern Position current_position;
extern double speed[2];
// robot's position and measurement history
extern struct History history;

// robot's ID
extern int ID;
extern int self_index;

// message structure
extern struct message message;

// list of other robots
extern struct friends friends[MAX_NB_ROBOTS];

extern int n_time_step;

extern double real_source_pos[3];

extern SimulationParameters simulation_parameters;

void init();
void update_position();
double get_exp_time();
void set_logfile_names();
void next_iteration_time();

class RosBridge {
public:
  RosBridge();                      // creates node, pubs, subs
  void spinOnce();                  // process incoming messages
  bool getCurrentPose(float* x, float* y, float* theta);
  void publishWaypoint(double x, double y, double theta);
  rclcpp::Node::SharedPtr node();

  using RobotMission = ste_interfaces::action::NavigateSample;
  using GoalHandleMission = rclcpp_action::ClientGoalHandle<RobotMission>;

  bool shakehand_with_robot();
  void send_waypoints(std::list<Position> path);
  void feedback_callback(GoalHandleMission::SharedPtr, const std::shared_ptr<const RobotMission::Feedback> feedback);
  void result_callback(const GoalHandleMission::WrappedResult & result);
  std::shared_ptr<History> get_meas_history();
  bool robot_reached_goal;
  bool shakehand_success;
  int feedback_count = 0;
private:
  rclcpp::Node::SharedPtr node_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_pose_{false};
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr handshake_client_;
  rclcpp_action::Client<RobotMission>::SharedPtr action_client_;
  // Use a shared_ptr to manage the vector heap memory
  std::shared_ptr<History> history_ptr_ = std::make_shared<History>();
  std::mutex data_mutex_;
  bool action_active_{false};
  
};



#endif
