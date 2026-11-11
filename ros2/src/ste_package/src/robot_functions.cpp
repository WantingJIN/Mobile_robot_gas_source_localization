#include "ste_package/robot_functions.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h> 


// #include "nvwa/debug_new.h"


long long start_time;

void init()
{
	if (VERBOSE)
		printf("initialization\n");

	start_time = current_timestamp();

	// seed the RNG
	srand(time(NULL));
      

    // load the simulation parameters
	std::string config_folder_path = ament_index_cpp::get_package_share_directory("ste_package");
	std::string config_path = config_folder_path + + "/config.txt";
    simulation_parameters.load_simulation_parameters_from_config(config_path.c_str());
    simulation_parameters.read_mapID_from_file();

	// set the log file names
	set_logfile_names();

	FILE *sim_param_file;

	switch (simulation_parameters.workingMode){
		case EXPERIMENT:
			//set the state to wait for stationary plume
			FSM.state = STATE_INITIALIZING;
			FSM.hook  = state_initializing;
			break;
		case VIDEO:
			//set the state to wait for stationary plume
			FSM.state = STATE_INITIALIZING;
			FSM.hook  = state_initializing;
			break;
		case OFFLINE:
			//set the state to read from log files
			FSM.state = STATE_READ_ALL;
			FSM.hook  = state_read_goal_measurement;
			break;
		default:
			break;
	}


	// write the simulation parameters
	
	sprintf(config_logfile_name, "%s/param_log.csv", folder_name);
	sim_param_file = fopen(config_logfile_name, "w");
	// fprintf(sim_param_file, "nbins %d %d %d %d %d\n", Q_nbins, x_nbins, y_nbins, a_y_nbins, b_y_nbins);
	// fprintf(sim_param_file, "min_maxs %f %f %f %f %f %f %f %f %f %f\n", Q_min, Q_max, source_x_min, source_x_max, source_y_min, source_y_max, a_y_min, a_y_max, b_y_min, b_y_max);
	fprintf(sim_param_file, "max_iter_number %d\n", MAX_ITER);
	fclose(sim_param_file);

	simulation_parameters.log_simulation_parameters(config_logfile_name);

	goal_position.copy_position(current_position);
    printf("Initialization done.\n");
}

/* Return value
 * sensor_reading: the reading of the sensor (by taking into account the sensor model)
 * odor_concentration: the concentration of the odor at the robot's position (ground truth)
 */

void set_logfile_names()
{
	// log files names
	char *pPath;
	pPath = getenv("STE_DDPM_ROOT");
	// printf("Workspace path: %s\n", pPath);
	if (pPath != NULL)
	{
		// printf("The current path is: %s\n", pPath);
		char folder_path[64];
		sprintf(folder_path, "%s/%s", pPath, "ros2/Results"); // create a new folder for log files
		mkdir(folder_path, 0755);
		//create a new folder with the time dd_hh_mm
		char date_folder[128];
		time_t now = time(0);
		tm *ltm = localtime(&now);
		sprintf(date_folder, "%s/%02d_%02d", folder_path, ltm->tm_mon+1 ,ltm->tm_mday);
		mkdir(date_folder, 0755);
		char time_folder[64];
		sprintf(time_folder, "%02d_%02d",ltm->tm_hour, ltm->tm_min);
		sprintf(folder_name, "%s/%s", date_folder, time_folder);
		mkdir(folder_name, 0755);
	}
	else
	{
		sprintf(folder_name, "../../../Results/current"); // create a new folder for log files
		mkdir(folder_name, 0755);
		sprintf(folder_name, "../../../Results/current/%d", ID); // create a new folder for log files
		mkdir(folder_name, 0755);
	}
	sprintf(measurements_logfile_name, "%s/measurements.csv", folder_name);
	sprintf(goalposition_logfile_name, "%s/goal_position.csv", folder_name);
	sprintf(expected_a_posteriori_logfile_name, "%s/expected_a_posteriori.csv", folder_name);
	sprintf(max_a_posteriori_logfile_name, "%s/max_a_posteriori.csv", folder_name);
	sprintf(likelihood_logfile_name, "%s/likelihood_comparison.csv", folder_name);
	sprintf(measurement_history_logfile_name, "%s/measurement_history.csv", folder_name);
	sprintf(position_history_logfile_name, "%s/position_history.csv", folder_name);
	sprintf(posterior_pdf_3D_name, "%s/posterior_pdf_3D.csv", folder_name);
	sprintf(entropy_logfile_name, "%s/entropy.csv", folder_name);
	sprintf(config_logfile_name, "%s/config_log.csv", folder_name);
	sprintf(trajectory_logfile_name, "%s/trajectory.csv", folder_name);
	sprintf(gas_concentration_map_logfile_name, "%s/gas_concentration_map.csv", folder_name);
	sprintf(gas_concentration_maps_log_root, "%s/gas_concentration_map", folder_name);
	sprintf(gas_information_map_logfile_name, "%s/gas_information_map.csv", folder_name);
	sprintf(experiment_time_logfile_name, "%s/experiment_time.csv", folder_name);
	sprintf(visited_list_logfile_name, "%s/visited_list.csv", folder_name);
	sprintf(clustering_bounding_boxes_logfile_name, "%s/clustering_bounding_boxes.csv", folder_name);
	sprintf(acc_info_map_logfile_name, "%s/acc_info_map.csv", folder_name);
	sprintf(scores_map_logfile_name, "%s/scores_map.csv", folder_name);
	sprintf(distance_map_logfile_name, "%s/distance_map.csv", folder_name);
	sprintf(weighted_belief_map_logfile_name, "%s/weighted_belief_map.csv", folder_name);
	sprintf(relative_history_meas_logfile_name, "%s/relative_history_meas.csv", folder_name);
	sprintf(wind_field_u_logfile_name, "%s/wind_field_u.csv", folder_name);
	sprintf(wind_field_v_logfile_name, "%s/wind_field_v.csv", folder_name);
	sprintf(wind_field_w_logfile_name, "%s/wind_field_w.csv", folder_name);

	// to save some debugging console input
	sprintf(console_logfile_name, "%s/console.txt", folder_name);
	// printf("Console log file: %s\n", console_logfile_name);

      // Running STE offline : run STE from loaded data of previous simulation
	// log name of the experiment
    sprintf(logfile_folder_name, "%s/ros2/Results/%s", workspacePath, simulation_parameters.experimental_logname);
	sprintf(experiment_measurements_logfile_name, "%s/measurement_history.csv", logfile_folder_name);
	sprintf(experiment_position_logfile_name, "%s/position_history.csv", logfile_folder_name);
	// printf("experiment position log file: %s\n", simulation_parameters.experimental_logname);
	// printf("Experiment measurements log file: %s\n", experiment_measurements_logfile_name);
	sprintf(experiment_goalposition_logfile_name, "%s/goal_position.csv", logfile_folder_name);
	experiment_goalposition_logfile = fopen(experiment_goalposition_logfile_name, "r");
	experiment_measurements_logfile = fopen(experiment_measurements_logfile_name, "r");
	experiment_position_logfile = fopen(experiment_position_logfile_name, "r");

}

void next_iteration_time(){
	usleep(SLEEP_PERIOD * 2000);
}



RosBridge::RosBridge()
  {
    node_ = std::make_shared<rclcpp::Node>("ste_robot_bridge");

	// 1. Initialize Service Client for Handshake
	handshake_client_ = node_->create_client<std_srvs::srv::Trigger>("/check_ready");
	
	// 2. Initialize Action Client for Waypoints
	action_client_ = rclcpp_action::create_client<RobotMission>(node_, "robot_mission");

	robot_reached_goal = false;
	shakehand_success = false;
  }

  rclcpp::Node::SharedPtr RosBridge::node() { return node_; }

  // Called from main loop after spin_some
  bool RosBridge::getCurrentPose(float* x, float* y, float* theta)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pose_) return false;
    *x = current_pose_.pose.position.x;
    *y = current_pose_.pose.position.y;
    *theta = tf2::getYaw(current_pose_.pose.orientation);
    return true;
  }


  void RosBridge::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
	printf("Received pose: x=%f, y=%f, theta=%f\n", msg->pose.position.x, msg->pose.position.y, tf2::getYaw(msg->pose.orientation));
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = *msg;
    has_pose_ = true;
  }

// --- STEP 1: Handshake Logic ---
   bool RosBridge::shakehand_with_robot() {
		RCLCPP_INFO(node_->get_logger(), "Waiting for robot service...");
		while (!handshake_client_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) return false;
			RCLCPP_INFO(node_->get_logger(), "Robot not found on network, retrying...");
		}

		auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
		handshake_client_->async_send_request(request, 
			[this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
				auto response = future.get();
				if (response->success) {
					RCLCPP_INFO(node_->get_logger(), "Handshake Success: %s", response->message.c_str());
					shakehand_success = true;
				} else {
					RCLCPP_ERROR(node_->get_logger(), "Robot rejected handshake: %s", response->message.c_str());
				}
			});
		return shakehand_success;
	}

    // --- STEP 2: Goal Generation ---
    void RosBridge::send_waypoints(std::list<Position> path) {
		
		// if (action_active_) { // 你可以加一个布尔量标记
    	// 	RCLCPP_WARN(node_->get_logger(), "Previous goal is still active, waiting...");
    	// 	return;
		// }
		printf("Preparing to send waypoints to robot...\n");
		robot_reached_goal = false;
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available!");
            return;
        }
		printf("Sending waypoints:");

        auto goal_msg = RobotMission::Goal();
		goal_msg.goal_positions.header.frame_id = "map"; 
		goal_msg.goal_positions.header.stamp = node_->now();
        for (const auto& pos : path) {
			geometry_msgs::msg::Pose waypoint;
			waypoint.position.x = pos.x;
			waypoint.position.y = pos.y;
			waypoint.position.z = 0.0;
			tf2::Quaternion q;
			q.setRPY(0, 0, pos.heading);
			waypoint.orientation = tf2::toMsg(q);
			goal_msg.goal_positions.poses.push_back(waypoint);
			printf("  Waypoint: (%.2f, %.2f)\n", waypoint.position.x, waypoint.position.y);
		}
		printf("\n");

        auto send_goal_options = rclcpp_action::Client<RobotMission>::SendGoalOptions();
        
        // Setup Callbacks
        send_goal_options.feedback_callback = std::bind(&RosBridge::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&RosBridge::result_callback, this, std::placeholders::_1);
        
        try {
			action_client_->async_send_goal(goal_msg, send_goal_options);
		} catch (const std::exception & e) {
			printf("CRASH PREVENTED: %s\n", e.what());
		}

		history_ptr_ = std::make_shared<History>(); 
		feedback_count = 0;

		printf("Action sent to robot.\n");
    }

    // --- STEP 3: Feedback & Result Callbacks ---
    void RosBridge::feedback_callback(GoalHandleMission::SharedPtr, const std::shared_ptr<const RobotMission::Feedback> feedback) {
        // This is where you receive the Gas and Anemometer data from the robot
		// RCLCPP_INFO(node_->get_logger(), "Robot Feedback -> Pos: (%.2f, %.2f) | Gas: %.2f", 
        //             feedback->current_pose.x, 
        //             feedback->current_pose.y, 
        //             feedback->gas_level);

		// only keep one measurement outof every N feedbacks to reduce data amount
		// const int FEEDBACK_SKIP = 5;
		// feedback_count++;
		// if (feedback_count >1 && feedback_count % FEEDBACK_SKIP != 0) {
		// 	return;
		// }
		// Store the feedback data into history
		std::lock_guard<std::mutex> lock(data_mutex_);

		history_ptr_->position.push_back(Position(feedback->current_pose.x, feedback->current_pose.y,
												  feedback->current_pose.theta));
		history_ptr_->gas_meas.push_back(feedback->gas_level);
		history_ptr_->wind_meas_u.push_back(feedback->wind_speed_u);
		history_ptr_->wind_meas_v.push_back(feedback->wind_speed_v);
		history_ptr_->size += 1;
	
    }

    void RosBridge::result_callback(const GoalHandleMission::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Waypoint reached! Waiting for next goal...");
				robot_reached_goal = true;
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Mission failed or was aborted.");
                break;
        }
    }
	std::shared_ptr<History> RosBridge::get_meas_history() {
        //std::lock_guard<std::mutex> lock(data_mutex_);
        // Return the current pointer and create a new empty one for the next mission
        auto snapshot = history_ptr_;
        history_ptr_ = std::make_shared<History>(); 
		action_active_ = false;
        return snapshot;
  }