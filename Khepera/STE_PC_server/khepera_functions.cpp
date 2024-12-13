#include "khepera_functions.h"
#include "Constant.h"

double odor_baseline_value = 0;
// sMsg STmsg;
sUdpMsg udp_msgR;
static int broadcastR = 1;

void init(){
	if(VERBOSE) printf("initialization\n");
	
	// seed the RNG
	srand(time(NULL));
	
	FSM.n_cycles = 0;

    // load the simulation parameters
	string config_file = string(workspacePath) + "/Khepera/Data/config.txt";
    simulation_parameters.load_simulation_parameters_from_config(config_file.c_str());
    simulation_parameters.rewrite_parameters_from_file();

	// set the log file names
	set_logfile_names();

    // write the simulation parameters
	FILE *sim_param_file;
	sprintf(config_logfile_name, "%s/param_log.csv", folder_name);
	sim_param_file = fopen(config_logfile_name, "w");
	fprintf(sim_param_file, "nbins %d %d %d %d %d\n", Q_nbins, x_nbins, y_nbins, a_y_nbins, b_y_nbins);
	fprintf(sim_param_file, "min_maxs %f %f %f %f %f %f %f %f %f %f\n", Q_min, Q_max, source_x_min, source_x_max, source_y_min, source_y_max, a_y_min, a_y_max, b_y_min, b_y_max);
	fprintf(sim_param_file, "max_iter_number %d\n", MAX_ITER);
	fclose(sim_param_file);

	simulation_parameters.log_simulation_parameters(config_logfile_name);

	switch (simulation_parameters.workingMode){
		case EXPERIMENT:
			//set the state to wait for stationary plume
			FSM.state = STATE_LISTEN;
			FSM.hook  = state_listen;
			break;
        	case VIDEO:
			//set the state to wait for stationary plume
			FSM.state = STATE_LISTEN;
			FSM.hook  = state_listen;
			break;

	  	case OFFLINE:
			//read position and measurement from log files
			FSM.state = STATE_READ_ALL;
			FSM.hook  = state_read_goal_measurement;
			break;
		default:
			break;
	}
}

double get_exp_time(){
	struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    double milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds // should be long long instead of double
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

void send_msg(Message msg){
	if(msg.receiver_ID != 0){ //if msg is not meant for supervisor
        //printf("sending a msg for %d\n", msg.receiver_ID);
  		int len = sendto(udp_msgR.snd.sock, msg.buffer, msg.buffer_size, 0, (struct sockaddr*) &udp_msgR.snd.address, sizeof (udp_msgR.snd.address));
  		//if (len < 0)
    		//fail_with_error("During broadcast\n");
    }
}

int receive_msg(Message * msg){
    int result;
    struct sockaddr_in sender;
    socklen_t sender_len;
    sender_len = sizeof (sender);
    memset((char *) (*msg).buffer, '\0', sizeof(double)*12);
    result = recvfrom(udp_msgR.rcv.sock, (*msg).buffer, sizeof(double)*12, 0, (struct sockaddr*) &sender, &sender_len);
    // If reception ok
    if (result > 0 ){ //&& (*msg).buffer[1] != ID
        //printf("RECEIVED msg %f from %f for %f - (%f %f %f) %f (%f %f %f) %f %f \n", (*msg).buffer[0], (*msg).buffer[1], (*msg).buffer[2], (*msg).buffer[3], (*msg).buffer[4], (*msg).buffer[5], (*msg).buffer[6], (*msg).buffer[7], (*msg).buffer[8], (*msg).buffer[9], (*msg).buffer[10], (*msg).buffer[11], (*msg).buffer[12]);
    	(*msg).buffer_to_params();
        //printf("RECEIVED msg %d from %d for %d \n", (*msg).msg_ID, (*msg).sender_ID, (*msg).receiver_ID);
        return 1;
    }else{
    	memset((char *) (*msg).buffer, '\0', sizeof(double)*12);
    	return 0;
    }
}


void cleaning_up(){
	FSM.state = STATE_FINISHED;
    //move out of the way of the others
    printf("[%d] DONE! I'm going back home\n", ID);
    send_goal_to_robot(-1, -1);
}

void next_iteration_time(){
	usleep(SLEEP_PERIOD * 2000);
}

void set_logfile_names(){
	//log files names
	sprintf(folder_name, "./log_STE_multi"); //create a new folder for log files
	mkdir(folder_name, 0755);
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
    sprintf(gas_information_map_logfile_name, "%s/gas_information_map.csv", folder_name);
	sprintf(gas_hit_variance_map_logfile_name, "%s/gas_hit_variance_map.csv", folder_name);
	sprintf(experiment_time_logfile_name, "%s/experiment_time.csv", folder_name);
	sprintf(relative_measurements_logfile_name, "%s/relative_measurements.csv", folder_name);

    // Running STE offline : run STE from loaded data of physical experiment or wind tunnel scan
	// log name of the experiment and scan
    strcpy(logfile_folder_name, simulation_parameters.experimental_logname);
	sprintf(experiment_measurements_logfile_name, "%s/pc/measurement_history.csv", logfile_folder_name);
	sprintf(experiment_position_logfile_name, "%s/pc/position_history.csv", logfile_folder_name);
	sprintf(experiment_goalposition_logfile_name, "%s/k4/goal_position.csv", logfile_folder_name);
    sprintf(visited_list_logfile_name, "%s/visited_list.csv", folder_name);
	sprintf(console_logfile_name, "%s/console.txt", folder_name);

    sprintf(scan_folder_name, "%s/Khepera/Results/log_K4_scan_map5_11_2", workspacePath);
	sprintf(scan_measurements_logfile_name, "%s/measurements.csv", scan_folder_name);
	sprintf(scan_position_logfile_name, "%s/goal_position.csv", scan_folder_name);

	// experiment results folder
	experiment_measurements_logfile = fopen(experiment_measurements_logfile_name,"r");
	experiment_position_logfile = fopen(experiment_position_logfile_name, "r");
	experiment_goalposition_logfile = fopen(experiment_goalposition_logfile_name, "r");
   	// scan results folder
	scan_measurements_logfile = fopen(scan_measurements_logfile_name, "r");
	scan_position_logfile = fopen(scan_position_logfile_name,"r");

}