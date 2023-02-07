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
	
	//robot
	// khepera4_init();
	
	//get ID from file
	// if(access("ID.txt", F_OK) != -1 ){
	// 	FILE * ID_file = fopen("ID.txt", "r");
	// 	int ok = fscanf(ID_file, "%d", &ID);
	// 	if(ok) printf("My ID is %d\n", ID);
	// 	fclose(ID_file);
	// }
 	//printf("My ID is %d\n", ID);

    //msg_counter = ID*100000;

	// Initialise motors
    // khepera4_drive_start();

	// Initialise odor board
    // odorboard_init();
    // odorboard.led.mode = cOdorBoardLEDMode_Off;
    // odorboard_led_mode_set();
    // odorboard_pump_speed_set(0, PUMP_SPEED); // Pump should preferably be running before...
    
    // Initialise wind board
    // windsensorboard_init();
    // windsensorboard.led.mode = cWindSensorBoardLEDMode_Tracking;
    // windsensorboard_led_mode_set();
    // windsensorboard_windsensor_start();

    // Initialise radio
    // udp_msg_init(3000, ID);
    //initCommSwisTrack();
    // initCortexComm();
    // initCommRobot();

    // Initialise infrared
    // khepera4_infrared_init();

    //receive position from Swistrack
    //receive_messages();
    //updateSwisTrack(current_position);
    // updateCortexPos(current_position);

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
	// measurements.n_samples = 0;
	// measurements.instant_concentration = 0;
	// measurements.odor_sample_memory = odor_filter_make(ODOR_FILTER_LENGTH);
 //    measurements.wind_sample_memory = odor_filter_make(ODOR_FILTER_LENGTH);
	//measurements.wind_filter = wind_filter_make(0);

	//initialize the wind vector
	// wind_vect = vector3d_zeros();

	//get the initial position
    // odometry_track_start_pos(&ot,0,0);

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
    sprintf(logfile_folder_name, "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Results/2/2022-09-11_21-55");
	sprintf(experiment_measurements_logfile_name, "%s/k4/measurement_history.csv", logfile_folder_name);
	sprintf(experiment_position_logfile_name, "%s/k4/position_history.csv", logfile_folder_name);

    sprintf(scan_folder_name, "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Results/log_K4_scan_map5_11_2");
	sprintf(scan_measurements_logfile_name, "%s/measurements.csv", scan_folder_name);
	sprintf(scan_position_logfile_name, "%s/goal_position.csv", scan_folder_name);

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
	if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/likelihood_error.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/likelihood_error.txt", "r");
		ok = fscanf(sim_param_file, "%f", &simulation_parameters.likelihood_error);
		if(ok) printf("likelihood_error : %f\n", simulation_parameters.likelihood_error);
		fclose(sim_param_file);
	}
    //printf("likelihood_error %f\n", simulation_parameters.likelihood_error);
	if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/gloabl_local_search_ratio.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params", "r");
		ok = fscanf(sim_param_file, "%f", &simulation_parameters.gloabl_local_search_ratio);
		if(ok) printf("gloabl_local_search_ratio : %f\n", simulation_parameters.gloabl_local_search_ratio);
		fclose(sim_param_file);
	}
	if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/MCMC_iteration_num.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/MCMC_iteration_num.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.MCMC_iteration_num);
		if(ok) printf("MCMC_iteration_num : %d\n", simulation_parameters.MCMC_iteration_num);
		fclose(sim_param_file);
	}
	if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/MCMC_or_exhaustive_eval.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/MCMC_or_exhaustive_eval.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.MCMC_or_exhaustive_eval);
		if(ok) printf("MCMC_or_exhaustive_eval : %d\n", simulation_parameters.MCMC_or_exhaustive_eval);
		fclose(sim_param_file);
	}
	if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/number_of_robots.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/number_of_robots.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.number_of_robots);
		if(ok) printf("number_of_robots : %d\n", simulation_parameters.number_of_robots);
		fclose(sim_param_file);
	}
	if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/scenario.txt", F_OK) != -1 ){
		sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/scenario.txt", "r");
		ok = fscanf(sim_param_file, "%d", &simulation_parameters.scenario);
		if(ok) printf("scenario : %d\n", simulation_parameters.scenario);
		fclose(sim_param_file);
	}
    if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/map_nbr.txt", F_OK) != -1 ){
       sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/params/map_nbr.txt", "r");
       ok = fscanf(sim_param_file, "%d", &simulation_parameters.map_nbr);
       if(ok) printf("map number : %d\n", simulation_parameters.map_nbr);
       fclose(sim_param_file);
   }
   if(access("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/source_position.txt", F_OK) != -1 ){
       sim_param_file = fopen("/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Data/source_position.txt", "r");
       ok = fscanf(sim_param_file, "%f %f", &simulation_parameters.source_position[0], &simulation_parameters.source_position[1]);
       if(ok) printf("source position : %f %f\n", &simulation_parameters.source_position[0], &simulation_parameters.source_position[1]);
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
	fprintf(sim_param_file, "min_maxs %d %d %f %f %d %d %f %f %f %f\n", Q_min, Q_max, source_x_min, source_x_max, source_y_min, source_y_max, a_y_min, a_y_max, b_y_min, b_y_max);
    fprintf(sim_param_file, "source_position %f %f %f\n", simulation_parameters.source_position[0], simulation_parameters.source_position[1], 0.1);
	fclose(sim_param_file);
}
/*
int initCortexComm(){
    int retval = RC_Okay;
    unsigned char SDK_Version[4];
    sBodyDefs* pBodyDefs = NULL;

    Cortex_GetSdkVersion(SDK_Version);
    //printf("Using SDK Version %d.%d.%d\n", SDK_Version[1], SDK_Version[2], SDK_Version[3]);

    Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
    //Cortex_SetDataHandlerFunc(MyDataHandler);

    printf("****** Cortex_Initialize ******\n");
    char K4_IP[16]; 
    sprintf(K4_IP, "192.168.1.%d", ID);
    retval = Cortex_Initialize(K4_IP, "192.168.1.40");

    if (retval != RC_Okay) {
        printf("Error: Unable to initialize ethernet communication\n");
        retval = Cortex_Exit();
        return 1;
    }

    printf("****** Cortex_GetBodyDefs ******\n");
    pBodyDefs = Cortex_GetBodyDefs();

    if (pBodyDefs == NULL) {
        printf("Failed to get body defs\n");
    } else {
        printf("Got body defs\n");
        Cortex_FreeBodyDefs(pBodyDefs);
        pBodyDefs = NULL;
    }

    void *pResponse;
    int nBytes;
    retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
    if (retval != RC_Okay)
        printf("ERROR, GetContextFrameRate\n");

    float *contextFrameRate = (float*) pResponse;

    printf("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);

    printf("*** Starting live mode ***\n");
    retval = Cortex_Request("LiveMode", &pResponse, &nBytes);
    if (retval != RC_Okay)
        printf("ERROR, live mode\n");

    return 0;
}

void initCommRobot(){
  	udp_msgR.myID = ID;
  	udp_msgR.len = sizeof(double)*12;

  	//initialize receiver
  	memset(&udp_msgR.rcv.address, 0, sizeof (udp_msgR.rcv.address));
  	udp_msgR.rcv.address.sin_family = AF_INET;
  	udp_msgR.rcv.address.sin_port = htons(3010);
  	udp_msgR.rcv.address.sin_addr.s_addr = htonl(INADDR_ANY);

  	//create and bind to socket
  	udp_msgR.rcv.sock = socket(AF_INET, SOCK_DGRAM, 0);
	//if (setsockopt(udp_msgR.rcv.sock, SOL_SOCKET, SO_REUSEADDR, (void*) &broadcastR, sizeof (broadcastR)) <0)
   		//fail_with_error("no reuse \n");
  	if (udp_msgR.rcv.sock < 0)
    	fail_with_error("Unable to create receiver socket");
  	fcntl(udp_msgR.rcv.sock, F_SETFL, O_NONBLOCK);
  	if (bind(udp_msgR.rcv.sock, (struct sockaddr*) &udp_msgR.rcv.address, sizeof (udp_msgR.rcv.address)))
    	fail_with_error("Bind failed robot");

  	//initialize sender
  	memset(&udp_msgR.snd.address, 0, sizeof (udp_msgR.snd.address));
  	udp_msgR.snd.address.sin_family = AF_INET;
  	udp_msgR.snd.address.sin_port = htons(3010);
  	udp_msgR.snd.address.sin_addr.s_addr = inet_addr("192.168.1.255");
  	udp_msgR.snd.sock = socket(AF_INET, SOCK_DGRAM, 0);

  	if (setsockopt(udp_msgR.snd.sock, SOL_SOCKET, SO_BROADCAST, (void*) &broadcastR, sizeof (broadcastR)) < 0)
    	fail_with_error("Unable to set broadcast option");
}



double odor_read(){
	double current_odor = 0;
    while(current_odor < 0.1 ){
        odorboard_voc_get();
        current_odor = (double) odorboard.voc.last.sensor[0];
        usleep(100 * 1000);
    }
    //printf("current_odor : %f \n", current_odor);
    return current_odor;
}

double wind_read(){
	double current_wind = 0, heading=0;
    windsensorboard_windsensor_get();
    current_wind = windsensorboard.windsensor.last.angle;
    //calculation
    heading = current_position[2]*180/PI;
    current_wind = fmod(-(current_wind*180/PI)+3*180,2*180);//-((current_wind*180/PI)-180);
    //printf("heading %f wind %f\n", heading, current_wind);
    return (heading - current_wind);
}

void updateCortexPos(double position[3]){
    sFrameOfData* FrameOfData;
    char prop_name[5];// = "K102";
    sprintf(prop_name, "K%d", ID);
    //printf("requesting a new frame \n");
    FrameOfData = NULL;
    while(FrameOfData == NULL)
        FrameOfData = Cortex_GetCurrentFrame();
    
    //decode the received frame of data
    for(int i=0;i<FrameOfData->nBodies;i++){
        if(!strcmp(FrameOfData->BodyData[i].szName, prop_name)){
            position[0]=FrameOfData->BodyData[i].Segments[0][0]/1000;
            position[1]=FrameOfData->BodyData[i].Segments[0][1]/1000;
            position[2]=(-1)*FrameOfData->BodyData[i].Segments[0][5]/180*PI;
            //while (position[2] > 2*PI) current_position[2] -= 2.0*PI;
            //while (position[2] < 0   ) current_position[2] += 2.0*PI;
            //printf("current_position - x: %f - y: %f - o3: %f\n", current_position[0], current_position[1], current_position[2]);
        }
    }
}

void update_position() {
    //get_gps_position(current_position);
	//updateSwisTrack(current_position);
    updateCortexPos(current_position);
    while (current_position[2] > 2*PI) current_position[2] -= 2.0*PI;
    while (current_position[2] < 0   ) current_position[2] += 2.0*PI;
    //printf("my current position is (%f %f %f)\n", current_position[0], current_position[1], current_position[2]);
}

int obstacle_or_not(){
	double sensors_value[SENSOR_NUMBER];

	khepera4_infrared_proximity();

	for (int i = 1; i < SENSOR_NUMBER-2; i++){
		sensors_value[i] = khepera4.infrared_proximity.sensor[i];
		//if one of the sensors detect something, then obstacle
		if(sensors_value[i] > 30){
			//printf("obstacle detected..!!!!!\n");
			return 1;
		}
	}
	//otherwise no obstacle
	return 0;
}

void receive_from_friends(){
	//printf("I am receiving\n");
	Message msg;
	//get all the messages in the queue
	while(receive_msg(&msg)){
		//got a message
        //printf("I am receiving\n");
        // if it's my own broadcasted msg, forget about it
        if(msg.sender_ID == ID) continue;
        // if it's addressed to another robot, forget about it
        if(!(msg.receiver_ID == 1 || msg.receiver_ID == ID)) continue;
        // if I already received this msg, then ignore it
        if (std::find(received_msgs.begin(), received_msgs.end(), msg.msg_ID) != received_msgs.end()) continue;
        //if the message is an ack, we remove it from the list and done

        if(msg.ack_flag == 2){
            //printf("[%lld] ack received for %d by \n", current_timestamp(), (int)(msg.msg_ID), msg.sender_ID);
            //msg.print_message();
            ack_msg(msg);
            continue;
        }
        //if it's a msg that requires an ack, send ack
        if(msg.ack_flag == 1){
            //printf("[%lld] sending ack for %d\n", current_timestamp(),(int)msg.msg_ID);
            Message ack(msg.msg_ID, ID, msg.sender_ID, msg.position, msg.msg_ID, msg.goal, 15000, 2);
            //msg.print_message();
            send_msg(ack);
            // add the msg to received_msgs list so I don't receive it again
            received_msgs.push_back(msg.msg_ID);
        }else{
            //printf("0000000000000000000000000000000000000000000000000 received a NOACK msg from\n", current_timestamp(),(int)msg.sender_ID);
        }
		//if the message contains a measurement value -> put it in history
		if(msg.measurement0_goal1 == 0){
			//std::cout << "COMMUNICATION++++++++++++++++++" << std::endl;
			//put the relelvant values in history
			history.measurement[history.size] = msg.concentration;
			memcpy(history.position[history.size], msg.position, sizeof(double)*3);
			printf("[%d] received concentration %f at (%f,%f,%f) from robot %d - ack flag %d\n", ID, history.measurement[history.size], history.position[history.size][0], history.position[history.size][1], history.position[history.size][2], msg.sender_ID, msg.ack_flag);
			history.size++;
			printf("my history size is now %d \n", history.size);
		}else{ //if the message contains the current position and goal of another robot -> update the friends list
			//std::cout << "JOINT PLANNING****************" << std::endl;
			//if the message is sent to my ID specifically, then it's a new goal for me!
			if(msg.receiver_ID == ID){
				//std::cout << "JOINT PLANNING****************" << std::endl;
				//std::cout << "Someone swapped with me" << std::endl;
				memcpy(goal_position, msg.goal, sizeof(double)*3);
				printf("[%d] my goal has changed to %f %f %f by %d\n", ID, goal_position[0], goal_position[1], goal_position[2], msg.sender_ID);
			}else {
				for(int i=0; i<simulation_parameters.number_of_robots; i++){
					//std::cout << "received news from friends" << std::endl;
					if(friends[i].ID == 0) friends[i].ID = msg.sender_ID;
					if(friends[i].ID == msg.sender_ID){
						//printf("message goal %f %f %f\n", msg.goal[0], msg.goal[1], msg.goal[2]);
						memcpy(friends[i].position, msg.position, sizeof(double)*3);
						memcpy(friends[i].goal, msg.goal, sizeof(double)*3);
						//printf("[%d] : robot %d is at %f %f %f and goes to %f %f %f\n", ID, friends[i].ID, friends[i].position[0],friends[i].position[1],friends[i].position[2],friends[i].goal[0],friends[i].goal[1],friends[i].goal[2]);
						break;
					}
					//printf("[%d] : robot %d is at %f %f %f and goes to %f %f %f\n", ID, friends[i].ID, friends[i].position[0],friends[i].position[1],friends[i].position[2],friends[i].goal[0],friends[i].goal[1],friends[i].goal[2]);
				}
			}
		}
	}
}
*/
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