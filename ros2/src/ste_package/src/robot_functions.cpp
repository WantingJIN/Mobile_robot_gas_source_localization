#include "ste_package/robot_functions.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


// #include "nvwa/debug_new.h"


long long start_time;

void init()
{
	if (VERBOSE)
		printf("initialization\n");

	start_time = current_timestamp();

	// seed the RNG
	srand(time(NULL));
      
	// set the log file names
	set_logfile_names();

    // load the simulation parameters
	std::string config_folder_path = ament_index_cpp::get_package_share_directory("ste_package");
	std::string config_path = config_folder_path + + "/config.txt";
    simulation_parameters.load_simulation_parameters_from_config(config_path.c_str());
    simulation_parameters.read_mapID_from_file();

	FILE *sim_param_file;

	switch (simulation_parameters.workingMode){
		case EXPERIMENT:
			//set the state to wait for stationary plume
			FSM.state = STATE_INITIALIZING;
			FSM.hook  = state_initializing;
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
	printf("Workspace path: %s\n", pPath);
	if (pPath != NULL)
	{
		printf("The current path is: %s\n", pPath);
		char folder_path[128];
		sprintf(folder_path, "%s/%s", pPath, "ros2/Results"); // create a new folder for log files
		mkdir(folder_path, 0755);
		sprintf(folder_name, "%s/%s", folder_path, "logs"); // create a new folder for log files
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
	sprintf(relative_history_meas_logfile_name, "%s/relative_history_meas.csv", folder_name);
	sprintf(wind_field_u_logfile_name, "%s/wind_field_u.csv", folder_name);
	sprintf(wind_field_v_logfile_name, "%s/wind_field_v.csv", folder_name);
	sprintf(wind_field_w_logfile_name, "%s/wind_field_w.csv", folder_name);

	// to save some debugging console input
	sprintf(console_logfile_name, "%s/console.txt", folder_name);
	printf("Console log file: %s\n", console_logfile_name);

      // Running STE offline : run STE from loaded data of previous simulation
	// log name of the experiment
    sprintf(logfile_folder_name, "%s/Results/%s/101", workspacePath, simulation_parameters.experimental_logname);
	sprintf(experiment_measurements_logfile_name, "%s/measurements.csv", logfile_folder_name);
	sprintf(experiment_position_logfile_name, "%s/position_history.csv", logfile_folder_name);
	sprintf(experiment_goalposition_logfile_name, "%s/goal_position.csv", logfile_folder_name);

}

void next_iteration_time(){
	usleep(SLEEP_PERIOD * 2000);
}



RosBridge::RosBridge()
  {
    node_ = std::make_shared<rclcpp::Node>("controller_bridge");

    pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
      "pose_ctrl", 10);

    pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
      "current_Pose", 10,
      std::bind(&RosBridge::currentPoseCallback, this, std::placeholders::_1));
  }

rclcpp::Node::SharedPtr RosBridge::node() { return node_; }

  // Called from main loop after spin_some
bool RosBridge::getCurrentPose(float* x, float* y, float* theta)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pose_) return false;
    *x = current_pose_.x;
    *y = current_pose_.y;
    *theta = current_pose_.theta;
    return true;
  }

  // Called from FSM / controller side to send waypoint
  void RosBridge::publishWaypoint(double x, double y, double theta)
  {
    geometry_msgs::msg::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    pose_pub_->publish(msg);
  }

  void RosBridge::currentPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = *msg;
    has_pose_ = true;
  }

