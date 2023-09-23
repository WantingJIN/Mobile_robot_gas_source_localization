#include "khepera_functions.h"

// struct sKhepera4 khepera4;
// struct sOdorboard odorboard;
// struct sWindSensorBoard windsensorboard;
double odor_baseline_value = 0;
// sMsg STmsg;
sUdpMsg udp_msgR;
static int broadcastR = 1;

void init(){
	if(VERBOSE) printf("initialization\n");
	
	// seed the RNG
	srand(time(NULL));

	//set the state to wait for stationary plume
	switch (WORKING_MODE){
		case EXPERIMENT:
		    FSM.state = STATE_LISTEN;
	        FSM.hook  = state_listen;
			break;
		case OFFLINE:
			FSM.state = STATE_READ;
	        FSM.hook  = state_read;
			break;
	}
	
	FSM.n_cycles = 0;

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
    // log name of the experiment and scan
    sprintf(logfile_folder_name, "/home/durancea/plume_model_learning/Results/Experimental_Results/Results_0228/2023-02-28_19-47/k4");
	sprintf(experiment_measurements_logfile_name, "%s/measurement_history.csv", logfile_folder_name);
	sprintf(experiment_position_logfile_name, "%s/position_history.csv", logfile_folder_name);

    sprintf(scan_folder_name, "/home/durancea/plume_model_learning/Results/Experimental_Results/Results_0228/2023-02-28_12-57/k4");
	sprintf(scan_measurements_logfile_name, "%s/measurement_history.csv", scan_folder_name);
	sprintf(scan_position_logfile_name, "%s/position_history.csv", scan_folder_name);

	//create / reinitialize log files (might be unnecessary)
	measurements_logfile = fopen(measurements_logfile_name, "w");
    fclose(measurements_logfile);
    goalposition_logfile = fopen(goalposition_logfile_name, "w");
    fclose(goalposition_logfile);
    expected_a_posteriori_logfile = fopen(expected_a_posteriori_logfile_name, "w");
    fclose(expected_a_posteriori_logfile);
    max_a_posteriori_logfile = fopen(max_a_posteriori_logfile_name, "w");
    fclose(max_a_posteriori_logfile);
	// experiment results folder
    experiment_measurements_logfile = fopen(experiment_measurements_logfile_name,"r");
    experiment_position_logfile = fopen(experiment_position_logfile_name, "r");
    // scan results folder
	scan_measurements_logfile = fopen(scan_measurements_logfile_name, "r");
	scan_position_logfile = fopen(scan_position_logfile_name,"r");

    // get the configuration
    //default values for the simulation parameters
    simulation_parameters = (struct simulation_parameters) {375, 0.5, 100, 1, 1, 0, 0};

	//read simulation parameters
	FILE* sim_param_file;
	int ok = 0;
	if(access("/home/durancea/plume_model_learning/Khepera/Data/params/likelihood_error.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params/likelihood_error.txt", "r");
		ok = fscanf(sim_param_file, "%f", &simulation_parameters.likelihood_error);
		if(ok) printf("likelihood_error : %f\n", simulation_parameters.likelihood_error);
		fclose(sim_param_file);
	}
    //printf("likelihood_error %f\n", simulation_parameters.likelihood_error);
	if(access("/home/durancea/plume_model_learning/Khepera/Data/params/gloabl_local_search_ratio.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params", "r");
		ok = fscanf(sim_param_file, "%f", &simulation_parameters.gloabl_local_search_ratio);
		if(ok) printf("gloabl_local_search_ratio : %f\n", simulation_parameters.gloabl_local_search_ratio);
		fclose(sim_param_file);
	}
	if(access("/home/durancea/plume_model_learning/Khepera/Data/params/MCMC_iteration_num.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params/MCMC_iteration_num.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.MCMC_iteration_num);
		if(ok) printf("MCMC_iteration_num : %d\n", simulation_parameters.MCMC_iteration_num);
		fclose(sim_param_file);
	}
	if(access("/home/durancea/plume_model_learning/Khepera/Data/params/MCMC_or_exhaustive_eval.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params/MCMC_or_exhaustive_eval.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.MCMC_or_exhaustive_eval);
		if(ok) printf("MCMC_or_exhaustive_eval : %d\n", simulation_parameters.MCMC_or_exhaustive_eval);
		fclose(sim_param_file);
	}
	if(access("/home/durancea/plume_model_learning/Khepera/Data/params/number_of_robots.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params/number_of_robots.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.number_of_robots);
		if(ok) printf("number_of_robots : %d\n", simulation_parameters.number_of_robots);
		fclose(sim_param_file);
	}
	if(access("/home/durancea/plume_model_learning/Khepera/Data/params/scenario.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params/scenario.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.scenario);
		if(ok) printf("scenario : %d\n", simulation_parameters.scenario);
		fclose(sim_param_file);
	}
    if(access("/home/durancea/plume_model_learning/Khepera/Data/params/map_nbr.txt", F_OK) != -1 ){
       sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/params/map_nbr.txt", "r");
       ok = fscanf(sim_param_file, "%d", &simulation_parameters.map_nbr);
       if(ok) printf("map number : %d\n", simulation_parameters.map_nbr);
       fclose(sim_param_file);
   }
   if(access("/home/durancea/plume_model_learning/Khepera/Data/source_position.txt", F_OK) != -1 ){
       sim_param_file = fopen("/home/durancea/plume_model_learning/Khepera/Data/source_position.txt", "r");
       ok = fscanf(sim_param_file, "%lf %lf", &simulation_parameters.source_position[0], &simulation_parameters.source_position[1]);
       if(ok) printf("source position : %lf %lf\n", simulation_parameters.source_position[0], simulation_parameters.source_position[1]);
       fclose(sim_param_file);
   }

	printf("Number of robots : %d - scenario : %d \n", simulation_parameters.number_of_robots, simulation_parameters.scenario);

	//write the simulation parameters
	sprintf(config_logfile_name, "%s/param_log.csv", folder_name);
	sim_param_file = fopen(config_logfile_name, "w");
	fprintf(sim_param_file, "likelihood_error %f\n", simulation_parameters.likelihood_error);
	fprintf(sim_param_file, "gloabl_local_search_ratio %f\n", simulation_parameters.gloabl_local_search_ratio);
	fprintf(sim_param_file, "MCMC_iteration_num %d\n", simulation_parameters.MCMC_iteration_num);
	fprintf(sim_param_file, "MCMC_or_exhaustive_eval %d\n", simulation_parameters.MCMC_or_exhaustive_eval);
	fprintf(sim_param_file, "number_of_robots %d\n", simulation_parameters.number_of_robots);
	fprintf(sim_param_file, "scenario %d\n", simulation_parameters.scenario);
    fprintf(sim_param_file, "map_nbr %d\n", simulation_parameters.map_nbr);
	fprintf(sim_param_file, "nbins %d %d %d %d %d\n", Q_nbins, x_nbins, y_nbins, a_y_nbins, b_y_nbins);
	fprintf(sim_param_file, "min_maxs %f %f %f %f %f %f %f %f %f %f\n", Q_min, Q_max, source_x_min, source_x_max, source_y_min, source_y_max, a_y_min, a_y_max, b_y_min, b_y_max);
    fprintf(sim_param_file, "source_position %f %f %f\n", simulation_parameters.source_position[0], simulation_parameters.source_position[1], 0.1);
	fclose(sim_param_file);
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