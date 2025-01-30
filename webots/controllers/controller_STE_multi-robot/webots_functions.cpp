#include "webots_functions.h"
// #include "nvwa/debug_new.h"

// webots libraries

WbDeviceTag compass;
WbDeviceTag emitter;
WbDeviceTag receiver;
WbDeviceTag odor_tag;
WbDeviceTag wind_tag;
WbDeviceTag sensors[SENSOR_NUMBER];
WbDeviceTag infrared_sensors[12];
WbDeviceTag left_motor, right_motor;

long long start_time;

double previous_sensor_value;

void init()
{
	if (VERBOSE)
		printf("initialization\n");

	start_time = current_timestamp();

	// seed the RNG
	srand(time(NULL));

	// robot
	wb_robot_init();
	khepera4_drive_init();
	const char *ID_char = wb_robot_get_name();
	ID = atoi(ID_char);

	// odor and wind sensors
	char tag_name[256];
	sprintf(tag_name, "sensor_odor_%d", ID);
	odor_tag = wb_robot_get_device(tag_name);
	wb_receiver_enable(odor_tag, TIME_STEP);
	sprintf(tag_name, "sensor_wind_%d", ID);
	wind_tag = wb_robot_get_device(tag_name);
	wb_receiver_enable(wind_tag, TIME_STEP);

	// GPS
	gps = wb_robot_get_device("gps");
	wb_gps_enable(gps, TIME_STEP);

	// Compass
	compass = wb_robot_get_device("compass");
	wb_compass_enable(compass, TIME_STEP);

	// Wheel speeds
	// wb_differential_wheels_enable_encoders(TIME_STEP);
	// wb_differential_wheels_set_speed(0,0);
	//  get the motors and set target position to infinity (speed control)
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	set_wheels_speed(0, 0);

	// distance sensors
	/*char sensors_name[] = "ds0";
	  for (int i = 0; i < SENSOR_NUMBER; i++){
		  sensors[i] = wb_robot_get_device(sensors_name);
		  wb_distance_sensor_enable(sensors[i], TIME_STEP);
		  (sensors_name[2])++;
	  }*/
	static const char *infrared_sensors_names[SENSOR_NUMBER + 4] = {
	    // turret sensors
	    "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
	    "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
	    // ground sensors
	    "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
	    "ground right infrared sensor"};
	// get and enable the infrared sensors
	for (int i = 0; i < 12; ++i)
	{
		infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
		wb_distance_sensor_enable(infrared_sensors[i], TIME_STEP);
	}

	// radio emitter and receiver
	emitter = wb_robot_get_device("emitter_K4");
	receiver = wb_robot_get_device("receiver_K4");
	wb_receiver_enable(receiver, TIME_STEP);

	// previous reading initialization
	previous_sensor_value = 0.0;

	// initialize the current position
	get_gps_position(&current_position);
	goal_position.copy_position(current_position);
	printf("initial goal position %f %f %f\n", goal_position.x, goal_position.y, goal_position.heading);
      
	// set the log file names
	set_logfile_names();

      // load the simulation parameters
      simulation_parameters.load_simulation_parameters_from_config("../../data/config.txt");
      simulation_parameters.rewrite_parameters_from_file();

	FILE *sim_param_file;
	int ok = 0;
      int release_amount;
	if (access("../../data/plugin_parameters/releaseAmount.txt", F_OK) != -1)
	{
		sim_param_file = fopen("../../data/plugin_parameters/releaseAmount.txt", "r");
		ok = fscanf(sim_param_file, "%d", &release_amount);
		if (ok)
			printf("release rate param : %d\n", release_amount);
		fclose(sim_param_file);
	}

	if (simulation_parameters.simuMode == ONLINE){
		if (access("../../data/source_pos.txt", F_OK) != -1) {
			FILE * sim_param_file = fopen("../../data/source_pos.txt", "r");
			int ok = fscanf(sim_param_file, "%lf %lf", &real_source_pos[0], &real_source_pos[1]);
			real_source_pos[2] = 0.13;
			if(ok) printf("source of gas : %lf %lf %lf\n", real_source_pos[0], real_source_pos[1], real_source_pos[2]);
			fclose(sim_param_file);  
		}
	}else if (simulation_parameters.simuMode == OFFLINE){
		char experiment_source_pos_logfile_name[512];
		char* source_str = (char*) malloc(100 * sizeof(char)); 
            sprintf(experiment_source_pos_logfile_name, "%s/source_pos.csv", logfile_folder_name);
		printf("%s\n",experiment_source_pos_logfile_name);
		if (access(experiment_source_pos_logfile_name, F_OK) != -1) {
			FILE * sim_param_file = fopen(experiment_source_pos_logfile_name, "r");
			int ok = fscanf(sim_param_file, "%s %lf %lf %lf",source_str, &real_source_pos[0], &real_source_pos[1], &real_source_pos[2]);
			if(ok) printf("source of gas : %lf %lf %lf\n", real_source_pos[0], real_source_pos[1], real_source_pos[2]);
			fclose(sim_param_file);  
		}
	}


      // experiment results folder
	experiment_measurements_logfile = fopen(experiment_measurements_logfile_name,"r");
    	experiment_position_logfile = fopen(experiment_position_logfile_name, "r");
	experiment_goalposition_logfile = fopen(experiment_goalposition_logfile_name, "r");

	// write the simulation parameters
	sprintf(config_logfile_name, "%s/param_log.csv", folder_name);
	sim_param_file = fopen(config_logfile_name, "w");
	fprintf(sim_param_file, "nbins %d %d %d %d %d\n", Q_nbins, x_nbins, y_nbins, a_y_nbins, b_y_nbins);
	fprintf(sim_param_file, "min_maxs %f %f %f %f %f %f %f %f %f %f\n", Q_min, Q_max, source_x_min, source_x_max, source_y_min, source_y_max, a_y_min, a_y_max, b_y_min, b_y_max);
	fprintf(sim_param_file, "max_iter_number %d\n", MAX_ITER);
	fprintf(sim_param_file, "release_amount %d\n", release_amount);
	fclose(sim_param_file);

	simulation_parameters.log_simulation_parameters(config_logfile_name);

	if ((ID - 100) > simulation_parameters.number_of_robots)
	{
		cleaning_up();
		exit(0);
	}
      
	if (simulation_parameters.simuMode == ONLINE){
		// set the state to wait for stationary plume
		FSM.state = STATE_WAITFORPLUME;
		FSM.hook = state_waiting;
		FSM.n_cycles = 0;
	} else if (simulation_parameters.simuMode == OFFLINE){
		// set the state to measuring
		FSM.state = STATE_READ_INITIAL;
		FSM.hook = state_read_initial_position_measurement;
		FSM.n_cycles = 0;
	}
}

/* Return value
 * sensor_reading: the reading of the sensor (by taking into account the sensor model)
 * odor_concentration: the concentration of the odor at the robot's position (ground truth)
 */

void odor_read(double *sensor_reading, double &odor_concentration)
{
	while (wb_receiver_get_queue_length(odor_tag) > 0)
	{
		assert(wb_receiver_get_data_size(odor_tag) == sizeof(double));
		odor_concentration = *(const double *)wb_receiver_get_data(odor_tag);
		wb_receiver_next_packet(odor_tag);
	}
	if (sensor_reading != nullptr)
	{
		// printf("odor measurement : %f \n",odor_measurement);
		//  For PID sensor, the sensor reading is equal to the odor concentration
		if (simulation_parameters.gasSensor == PID)
		{
			*sensor_reading = odor_concentration;
			return;
		}
		// For the MOX sensor, the sensor properties are modeled as a first-order system
		if (simulation_parameters.gasSensor == MOX || simulation_parameters.gasSensor == SLOW_MOX 
		|| simulation_parameters.gasSensor == VERY_SLOW_MOX || simulation_parameters.gasSensor == NOISY_MOX
		|| simulation_parameters.gasSensor == NOISY_SLOW_MOX || simulation_parameters.gasSensor == NOISY_VERY_SLOW_MOX)
		{
			double Ts = TIME_STEP / 1000.0; // sampling frequency
			double tau_r = 0 ,tau_d = 0;
			if (simulation_parameters.gasSensor == MOX || simulation_parameters.gasSensor == NOISY_MOX){
				tau_r = 0.72; // time constant for the rise (our default sensor)
				tau_d = 0.92; // time constant for the decay
			}
			if (simulation_parameters.gasSensor == SLOW_MOX || simulation_parameters.gasSensor == NOISY_SLOW_MOX){
				tau_r = 2.06; // time constant for the rise (slow sensor)
				tau_d = 4.53; // time constant for the decay
			}
			if (simulation_parameters.gasSensor == VERY_SLOW_MOX || simulation_parameters.gasSensor == NOISY_VERY_SLOW_MOX){
				tau_r = 5.19; // time constant for the rise (very slow MOX TGS2600)
			      tau_d = 19.74; // time constant for the decay

			}

			// sensor model: Y[n] = 1/(1 + Tswc) * Y[n-1] + Tswc/(1 + Tswc) * X[n]
			// where wc is the cut-off frequency and Ts is the sampling period
			double alpha;
			if (odor_concentration > previous_sensor_value) // rising phase
				alpha = Ts / (tau_r + Ts);
			else // decaying phase
				alpha = Ts / (tau_d + Ts);
			*sensor_reading = alpha * odor_concentration + previous_sensor_value * (1 - alpha);
			previous_sensor_value = *sensor_reading;
			if (*sensor_reading < 0)
			{
				*sensor_reading = 0;
			}
		}
		if (simulation_parameters.calibrationMode == LINEAR)
		{
			double raw_reading_before_calibration;
			linear_reverse_con_to_raw_reading(sensor_reading, raw_reading_before_calibration);
			*sensor_reading = raw_reading_before_calibration;
		}
		if (simulation_parameters.calibrationMode == NOLINEAR)
		{
			double raw_reading_before_calibration;
			nolinear_reverse_con_to_raw_reading(sensor_reading, raw_reading_before_calibration);
			*sensor_reading = raw_reading_before_calibration;
		}
		if (simulation_parameters.gasSensor == NOISY_MOX ||
		simulation_parameters.gasSensor == NOISY_SLOW_MOX || simulation_parameters.gasSensor == NOISY_VERY_SLOW_MOX)
		{
			// // add some random noise to the sensor reading
			// // define the sensor noise as a Gaussian distribution with mean = 0 
			// double mean = 0.0;
			// double gas_sensor_noise_var = 40000;
			// double sensor_noise = generateGaussianNoise(mean, gas_sensor_noise_var);
			// // gaussian noise
			// *sensor_reading += sensor_noise;
			// // constant error
			// //*sensor_reading += 200.0;
			// // scaling error
			// //*sensor_reading *= 2.0;
			// Heteroscedastic Noise = background noise + noise propotional to the signal (intermittency of the plume)
			addNoise(sensor_reading, 69.33, 10.8);
		}
		if (*sensor_reading < 0)
		{
			*sensor_reading = 0;
		}
	}
}

void linear_reverse_con_to_raw_reading(double* odor_concentration, double& MOX_sensor_reading)
{
	double a = 3.0;
	double b = 99.0;
	MOX_sensor_reading = (*odor_concentration - b)/a;
	if (MOX_sensor_reading < 0)
		MOX_sensor_reading = 0;
}
void nolinear_reverse_con_to_raw_reading(double* odor_concentration, double& MOX_sensor_reading)
{
	// the odor concentration is actually simulating the sensor reading instead of the concentration on [ppm]
	// in the datesheet, the detectable gases concentration range is from 10 to 500
	// divide thhe simulated measurement by 100 to approximate the concentration in ppm
	double Rs = pow(*odor_concentration/(100*1.488),-0.632) * simulation_parameters.R0;
	double RL = 51.0; //kOhm
	double Vc = 3300; //mV
	MOX_sensor_reading = Vc * RL / (RL + Rs);  // the gas sensor output is the voltage of the loading resistor
	if (MOX_sensor_reading < 0)
		MOX_sensor_reading = 0;
}

void wind_read(double *intensity, double *angle)
{
	struct vect3D
	{
		double x;
		double y;
		double z;
	};
	struct vect3D v = {0};
	while (wb_receiver_get_queue_length(wind_tag) > 0)
	{
		assert(wb_receiver_get_data_size(wind_tag) == sizeof(struct vect3D));
		// not confident in the wind sensor, we round up the value to 1 digit
		v = *(const struct vect3D *)wb_receiver_get_data(wind_tag);
		wb_receiver_next_packet(wind_tag);
	}
	// convert the 3D vector from the sensor to intensity and heading
	*intensity = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	*angle = 0;
}

void get_gps_position(Position *external_gps_position)
{
	const double *gps_values = NULL;
	gps_values = wb_gps_get_values(gps);

	external_gps_position->x = gps_values[0];			     // + 1500; //position in cm
	external_gps_position->y = gps_values[1];			     // +  200; //position in cm
	external_gps_position->heading = current_position.heading; // just for it to have a theta to give back...

	// printf("Coord from GPS: x = %lf; y = %lf\n", external_gps_position[0], external_gps_position[1]);
}

void update_position()
{
	const double *compass_heading = NULL;

	get_gps_position(&current_position);

	compass_heading = wb_compass_get_values(compass);
	current_position.heading = (-atan2(compass_heading[0], compass_heading[2]) + M_PI);
}

int obstacle_or_not()
{
	double sensors_value[SENSOR_NUMBER];

	for (int i = 0; i < SENSOR_NUMBER; i++)
	{
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
		// if one of the sensors detect something, then obstacle
		if (sensors_value[i] > 200)
		{
			// printf("[%d] obstacle detected..!!!!! %d %f\n", ID, i, sensors_value[i]);
			return 1;
		}
	}
	// otherwise no obstacle
	return 0;
}

void obstacle_avoidance_k4()
{
	// if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	// IR sensors on K4 are respectively : {back_left, left, front_left, front, front_right, right, back_right, back}
	// double khepera4_matrix[SENSOR_NUMBER][2] = {{2,1},{3,1},{3,0},{1,1},{0,3},{1,3},{1,2},{2,2}};
	// double khepera4_matrix[SENSOR_NUMBER][2] = {{1, 1}, {1, -1}, {2, -2}, {-2, -3}, {-2, 2}, {-1, 1}, {0, 1}, {1, 1}};//{{-2, 4}, {-3, 5}, {-7, 7}, {7, -6}, {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5}};//{{-0.5,-0.5}, {-2, 4}, {-4, 3}, {3,3}, {4, -5}, {4, -2}, {-0.5,-0.5}, {-0.5,-0.5}};
	// double khepera4_matrix[SENSOR_NUMBER][2] ={{-2, 4}, {-3, 5}, {-7, 7}, {7, -6}, {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5}}; //from webots' breitenberg controller
	double khepera4_matrix[SENSOR_NUMBER][2] = {{0, 1}, {1, -1}, {2, -2}, {-3, -2.5}, {-2, 2}, {-1, 1}, {1, 0}, {0, 0}};
	double sensors_value[SENSOR_NUMBER];
	double range = 1024;

	// if no obstacle,
	if (obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05)
	{
		// printf("no obstacle anymore..XXXXXXX\n");
		// wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0, 0);
		// back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}

	for (int i = 0; i < SENSOR_NUMBER; i++)
	{
		sensors_value[i] = wb_distance_sensor_get_value(infrared_sensors[i]);
	}

	// adjust the speed to avoid the obstacle
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < SENSOR_NUMBER; j++)
		{
			speed[i] += (khepera4_matrix[j][i] * -12) * (1.0 - (sensors_value[j] / range));
		}
		speed[i] = BOUND(speed[i], -MAX_SPEED, MAX_SPEED);
	}
	// wb_differential_wheels_set_speed(speed[0],speed[1]);
	// printf("[%d] speed final = %f %f\n", ID, speed[0], speed[1]);
	set_wheels_speed(speed[0], speed[1]);
}

void receive_from_friends()
{
	// printf("I am receiving\n");
	const void *msg_from_friends;
	// get all the messages in the queue
	while (wb_receiver_get_queue_length(receiver))
	{
		// get a message
		msg_from_friends = wb_receiver_get_data(receiver);
		// convert it to a Message instance
		Message msg(msg_from_friends);
		// msg.print_message();

		// if the message contains a measurement value -> put it in history
		if (msg.measurement0_goal1 == 0)
		{
			// std::cout << "COMMUNICATION++++++++++++++++++" << std::endl;
			// put the relelvant values in history
			history.measurement[history.size] = msg.concentration;
			// memcpy(history.position[history.size], msg.position, sizeof(double)*3);
			history.position[history.size].copy_position(msg.position);
			// printf("I, robot %d, received concentration %f at (%f,%f,%f) from robot %d.\n", ID, history.measurement[history.size], history.position[history.size][0], history.position[history.size][1], history.position[history.size][2], msg.sender_ID);
			history.size++;
		}
		else
		{ // if the message contains the current position and goal of another robot -> update the friends list
			// std::cout << "JOINT PLANNING****************" << std::endl;
			// if the message is sent to my ID specifically, then it's a new goal for me!
			if (msg.receiver_ID == ID)
			{
				// std::cout << "JOINT PLANNING****************" << std::endl;
				// std::cout << "Someone swapped with me" << std::endl;
				// memcpy(goal_position, msg.goal, sizeof(double)*3);
				goal_position.copy_position(msg.goal);
				printf("[%d] my goal has changed to %f %f %f \n", ID, goal_position.x, goal_position.y, goal_position.heading);
				// inform the supervisor again to move the target to the new place
				wb_emitter_set_channel(emitter, 600);
				Message msg(ID, 0, goal_position, -1, goal_position, 0);
				wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
			}
			else
			{
				for (int i = 0; i < simulation_parameters.number_of_robots; i++)
				{
					// std::cout << "received news from friends" << std::endl;
					if (friends[i].ID == 0)
						friends[i].ID = msg.sender_ID;
					if (friends[i].ID == msg.sender_ID)
					{
						// printf("message goal %f %f %f\n", msg.goal[0], msg.goal[1], msg.goal[2]);
						// memcpy(friends[i].position, msg.position, sizeof(double)*3);
						friends[i].position.copy_position(msg.position);
						// memcpy(friends[i].goal, msg.goal, sizeof(double)*3);
						friends[i].goal.copy_position(msg.goal);
						// printf("[%d] : robot %d is at %f %f %f and goes to %f %f %f\n", ID, friends[i].ID, friends[i].position[0],friends[i].position[1],friends[i].position[2],friends[i].goal[0],friends[i].goal[1],friends[i].goal[2]);
						break;
					}
					// printf("[%d] : robot %d is at %f %f %f and goes to %f %f %f\n", ID, friends[i].ID, friends[i].position[0],friends[i].position[1],friends[i].position[2],friends[i].goal[0],friends[i].goal[1],friends[i].goal[2]);
				}
			}
		}
		// delete the head packet so the next packet in the queue, if any, becomes the new head packet
		wb_receiver_next_packet(receiver);
	}
}

double get_exp_time()
{
	return wb_robot_get_time();
}

void send_msg(Message msg)
{
	// set the channel
	if (msg.receiver_ID == 0) // msg is meant for supervisor
		wb_emitter_set_channel(emitter, 600);
	else // for other robots
		wb_emitter_set_channel(emitter, 601);
	// send
	wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
}

void set_wheels_speed(double speed1, double speed2)
{
	// wb_differential_wheels_set_speed(speed1,speed2);
	wb_motor_set_velocity(left_motor, speed1);
	wb_motor_set_velocity(right_motor, speed2);
}

void cleaning_up()
{
	int systemRet = 0;
	char file_name[256];
	FILE *ack_file;
	char *pPath;

	// disable the sensors
	wb_receiver_disable(odor_tag);
	wb_receiver_disable(wind_tag);

	// ask the supervisor to quit the simulation
	// wb_emitter_set_channel(emitter, 600);
	// goal_position.x = -1; goal_position.y = -1; goal_position.heading = -1;
	goal_position.set_to(-1, -1, -1);
	Message msg(ID, 0, goal_position, -1, goal_position, 0);
	// wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
	send_msg(msg);

	// delete plume_model;

	wb_robot_cleanup();
}

void next_iteration_time()
{
	wb_robot_step(TIME_STEP);
	// check for timeout
	//  if(current_timestamp()-start_time > TIMEOUT)
	//  	FSM.state = STATE_FINISHED;
}

double generateGaussianNoise(double mean, double variance)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::normal_distribution<double> dist(mean, std::sqrt(variance));
	return dist(gen);
}

void set_logfile_names()
{
	// log files names
	char *pPath;
	pPath = getenv("WB_WORKING_DIR");
	if (pPath != NULL)
	{
		if (VERBOSE)
			printf("The current path is: %s\n", pPath);
		sprintf(folder_name, "%s/%lld", pPath, current_timestamp());
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
	sprintf(posterior_logfile_name, "%s/posterior.csv", folder_name);
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
	sprintf(gas_information_map_logfile_name, "%s/gas_information_map.csv", folder_name);
	sprintf(gas_hit_variance_map_logfile_name, "%s/gas_hit_variance_map.csv", folder_name);
	sprintf(experiment_time_logfile_name, "%s/experiment_time.csv", folder_name);
	sprintf(visited_list_logfile_name, "%s/visited_list.csv", folder_name);
	// // to save some debugging console input
	sprintf(console_logfile_name, "%s/console.txt", folder_name);

      // Running STE offline : run STE from loaded data of previous simulation
	// log name of the experiment and scan
      sprintf(logfile_folder_name, "%s/Results/rel_con_MOX_cdf/rel_con_MOX_source_boundary_cdf/10/1712062974290/101", workspacePath);
	sprintf(experiment_measurements_logfile_name, "%s/measurement_history.csv", logfile_folder_name);
	sprintf(experiment_position_logfile_name, "%s/position_history.csv", logfile_folder_name);
	sprintf(experiment_goalposition_logfile_name, "%s/goal_position.csv", logfile_folder_name);

}
// Function to add noise to a sensor reading
void addNoise(double* sensorReading, double sigma0 = 69.33, double k = 10.80) {
    // Compute the standard deviation of the noise
    double stdDev = std::sqrt(std::pow(sigma0, 2) + std::pow(k, 2) * std::abs(*sensorReading));

    // Set up a random number generator for normal distribution
    std::random_device rd; // Seed for randomness
    std::mt19937 gen(rd()); // Mersenne Twister generator
    std::normal_distribution<> dist(0.0, stdDev); // Mean 0, standard deviation stdDev

    // Generate noise and add it to the sensor reading
    double noise = dist(gen);
    *sensorReading += noise;
}