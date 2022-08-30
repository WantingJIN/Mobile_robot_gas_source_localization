/*
* File:          controller_STE_clean.cpp
* Date:          Jan 2019
* Description:   The goal is to estimate the source terms including the position
* Author:        Faezeh Rahbar
* Modifications: 26/01/2019
*/

//custom libraries
#include <fstream>
#include "controller_STE_clean.h"
#include "khepera_functions.h"

#include "Position.h"

#define PORT 3011
#define SA struct sockaddr 

//using namespace std; //can't use it because of isnan !

//Global Variables
struct sOdometryTrack ot;

int sock = 0; 

// log files
char folder_name[256];
FILE* goalposition_logfile;
FILE* measurements_logfile;
FILE* posterior_logfile;
FILE* posteriorpdf_logfile;
FILE* expected_a_posteriori_logfile;
FILE* likelihood_logfile;
FILE* measurement_history_logfile;
FILE* position_history_logfile;
FILE* max_a_posteriori_logfile;
FILE* posterior_pdf_3D;
FILE* entropy_logfile;
FILE* tmp_file;
FILE* config_logfile;
FILE* trajectory_logfile;
char goalposition_logfile_name[256];
char measurements_logfile_name[256];
char posterior_logfile_name[256];
char posteriorpdf_logfile_name[256];
char expected_a_posteriori_logfile_name[256];
char max_a_posteriori_logfile_name[256];
char likelihood_logfile_name[256];
char measurement_history_logfile_name[256];
char position_history_logfile_name[256];
char posterior_pdf_3D_name[256];
char entropy_logfile_name[256];
char config_logfile_name[256];
char trajectory_logfile_name[256];

// finite state machine
struct FSM FSM;
// robot's measurements
SampleBuffer sampleBuffer(ODOR_FILTER_LENGTH);

Position goal_position;
Position current_position;
double speed[2] = {0};
// robot's position and measurement history
struct history history;
//wind vector
sVector3d wind_vect;
// current posterior distribution 
// Posterior posterior;

// predicted posterior distribution
// Posterior future_posterior;

// posterior set to zero
// Posterior zero(0);

//robot's ID
int ID;

//message structure
struct message message;

//list of other robots
struct friends friends[MAX_NB_ROBOTS];

int n_time_step = 0;

struct simulation_parameters simulation_parameters;

int msg_counter = 0;

std::vector<Message> ack_msg_vec;
std::vector<int> received_msgs;

// obstacles definitions
int nbr_obs = 4;
float margin = 0.2;
float ** obstacles = NULL;
// float obstacles[][4] = {{5-margin, static_cast<float>(5.5+margin), static_cast<float>(2.5-margin), 4+margin}, 
// 							{5-margin, 8+margin, static_cast<float>(2.5-margin), 3+margin}, 
// 							{9-margin, 12+margin, 1-margin, static_cast<float>(1.5+margin)}, 
// 							{static_cast<float>(11.5-margin), 12+margin, 0-margin, static_cast<float>(1.5+margin)}};
std::list<Position> path;
std::list<Position>::iterator i;

std::string environment_file("/home/root/wanting/test_map.environment");
VisiLibity::Environment my_environment(environment_file);
double epsilon = 0.000000001;
VisiLibity::Visibility_Graph vg(my_environment, epsilon);

int finished = 0;

bool set_odometry_pos_from_MCS = false;

//############################################### to fix
int nb_robots = 1;
int robots_ID_list[] = {101};


// =========================== FUNCTIONS ================================ //
//random value sampled from a uniform distribution
double randu(){
	//random variable between 0 and 1
    return (double)rand() / (double)RAND_MAX ;
}

long long current_timestamp(){
	struct timeval te; 
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
	// printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;
}

void do_odometry() {
    odometry_track_step(&ot);  
    current_position.x = ot.result.x;// + init_pos_x;
    current_position.y = -ot.result.y;// + init_pos_y;
    current_position.heading = -ot.result.theta;// + init_pos_theta;
}

double dist_to_goal(Position goal_position){
    //Euclidean distance between current position to the goal
    double dx, dy;
    dx = goal_position.x-current_position.x;
    dy = goal_position.y-current_position.y;
    return sqrt(dx*dx+dy*dy);
}

//show the measurement history on screen
void history_print(struct history history){
	printf("_____ Sample History _____ \n");
	for(int i=0; i<history.size; i++){
		printf("%d : %f (+/- %f) @ (%f, %f)\n", i, history.measurement_avg[i], history.measurement_std[i], history.position[i].x, history.position[i].y);
	}
	printf("__________________________ \n");
}

void read_obstacle_list(){
	// open file
	std::ifstream infile("/home/root/wanting/obstacle_list.txt");
	// first line is the number of obstacles
	infile >> nbr_obs;
	// allocate memory for obstacle list array
	obstacles = new float*[nbr_obs];
	// read all obstacles now
	for(int i = 0; i < nbr_obs; i++){
		obstacles[i] = new float[4];
		infile >> obstacles[i][0] >> obstacles[i][1] >> obstacles[i][2] >> obstacles[i][3];
		obstacles[i][0] -= margin;
		obstacles[i][1] -= margin;
		obstacles[i][2] += margin;
		obstacles[i][3] += margin;
	}

	for(int i = 0; i < nbr_obs; i++){
		printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
	}
}

// add a message to ack required msgs or remove one from the list
void ack_msg(Message msg){
    //check if it's an ack or a new message
    if(msg.ack_flag == 2){
        //i'ts an ack => find the corresponding msg in the vector and remove it
        //for(std::vector<Message>::iterator it = ack_msg_vec.begin(); it != ack_msg_vec.end(); it++){
        //    if(it->msg_ID == msg.msg_ID){
        //        //remove the acked message
        //        printf("%d acked and removed\n", (int)(it->concentration));
        //        ack_msg_vec.erase(it);
        //   }
        //} // --> makes seg fault because of erase
        std::vector<Message>::iterator it = ack_msg_vec.begin();
        while(it != ack_msg_vec.end()){
    		if(it->msg_ID == msg.msg_ID && it->receiver_ID == msg.sender_ID){
      			//remove the acked message
                //printf("[%lld] %d acked by %d and removed\n", current_timestamp(), (int)(it->msg_ID), msg.sender_ID);
                ack_msg_vec.erase(it);
    		}else
      			it++;
  		}
    }else{
        //it has to be added to the table with all IP addresses
        if(msg.receiver_ID == 1){
            for(int i=0; i<nb_robots; i++){
                if(robots_ID_list[i] == ID) continue;
                Message tmp(msg.msg_ID, ID, robots_ID_list[i], msg.position, msg.concentration, msg.goal, msg.measurement0_goal1, msg.ack_flag);
                //msg.receiver_ID = robots_ID_list[i];
                //printf("[%lld] %d waiting for ack by %d\n", current_timestamp(),(int)(msg.msg_ID), tmp.receiver_ID);
                ack_msg_vec.push_back(tmp);
            }
        }else{
        	ack_msg_vec.push_back(msg);
        }
    }
    //send all the messages in the table
    printf("----> waiting for ack: ");
    for(std::vector<Message>::iterator it = ack_msg_vec.begin(); it != ack_msg_vec.end(); it++){
        printf("%d(%d) ", (int)(it->msg_ID), (int)(it->receiver_ID));

    }
    printf("\n");
}

int init_TCP_socket(){
	struct sockaddr_in address; 
	struct sockaddr_in serv_addr; 

	// Creating socket file descriptor 
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) { 
		printf("\n Socket creation error \n"); 
		return -1; 
	} 

	memset(&serv_addr, '0', sizeof(serv_addr)); 

	serv_addr.sin_family = AF_INET; 
	serv_addr.sin_port = htons(PORT); 
	//serv_addr.sin_addr.s_addr = INADDR_ANY; 
	
	// Convert IPv4 and IPv6 addresses from text to binary form 
	if(inet_pton(AF_INET, "192.168.141.11", &serv_addr.sin_addr)<=0) { //Server's IP
		printf("\nInvalid address/ Address not supported \n"); 
		return -1; 
	} 

	// connect
	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) { 
		printf("\nConnection Failed \n"); 
		return -1; 
	}
}

// ===================== LOG FILES FUNCTIONS ========================== //

void log_position(){
	double time = get_exp_time();
	goalposition_logfile = fopen(goalposition_logfile_name, "a");
	fprintf(goalposition_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, current_position.x, current_position.y, current_position.heading, time);
	fclose(goalposition_logfile);
}

void log_measurements(){
	//int i = 0;
	double time = get_exp_time();
	measurements_logfile = fopen(measurements_logfile_name, "a");
	fprintf(measurements_logfile,"%d, %f, %f, %f\n", FSM.n_cycles, sampleBuffer.average_wind_intensity, sampleBuffer.average_concentration, time);
	//for (i = 0; i < ODOR_FILTER_LENGTH; i++)
	//	fprintf(measurements_logfile,"%f, ", measurements.odor_sample_memory.buffer[i]);
	//fprintf(measurements_logfile,"\n");
	fclose(measurements_logfile);
}

void log_history(){
	// position
	//FILE* position_history_logfile;
	position_history_logfile = fopen(position_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++){
		fprintf(position_history_logfile,"%d, %f, %f, %f\n", i, history.position[i].x,history.position[i].y,history.position[i].heading);
	}
    fclose(position_history_logfile);
    // measurement
    //FILE * measurement_history_logfile;
    measurement_history_logfile = fopen(measurement_history_logfile_name, "w");
	for (int i = 0; i < history.size; i++){
		fprintf(measurement_history_logfile,"%d, %f %f\n", i, history.measurement_avg[i], history.measurement_std[i]);
	}
    fclose(measurement_history_logfile);
   
	//posterior 3D
	//posterior.log_pdf(posterior_pdf_3D_name, 'w');
}

void log_trajectory(Position p){
 	double time = get_exp_time();
 	trajectory_logfile = fopen(trajectory_logfile_name, "a");
	fprintf(trajectory_logfile,"%d, %f, %f, %f, %f\n", FSM.n_cycles, p.x, p.y, p.heading, time);
 	fclose(trajectory_logfile);
}

// ============================== STATES ================================= //
void state_waiting(){
	// initial waiting for the plume to get established
	static int counter = 0;
	counter++;

	if(counter > 300){
		FSM.state = STATE_MEASURING;
		FSM.hook  = state_measuring;
	}
}

void state_measuring(){
	// printf("measuring sample number %d\n", sampleBuffer.n_samples);
	if(sampleBuffer.n_samples < ODOR_FILTER_LENGTH){
		//continue measuring odor concentration
        sampleBuffer.instant_concentration = odor_read();
		//measuring wind
		//wind_read(&sampleBuffer.instant_wind_intensity, &sampleBuffer.instant_wind_angle);
		//add samples to buffers
		sampleBuffer.add_all_to_buffer();
		//printf("n %d wind intensity: %f, angle %f \n", sampleBuffer.n_samples, sampleBuffer.instant_wind_intensity, sampleBuffer.instant_wind_angle);
	}else{
		//average all samples
		sampleBuffer.average_all();

		//log and save
		if(VERBOSE) printf("[%d] ----------------------->>  average concentration: %f - wind intensity: %f angle: %f\n", ID, sampleBuffer.average_concentration, sampleBuffer.average_wind_intensity, sampleBuffer.average_wind_angle);
		log_position();
		log_measurements();
		history.position[history.size].x = current_position.x;
		history.position[history.size].y = current_position.y;
		history.position[history.size].heading = 0;
		history.measurement_avg[history.size] = (sampleBuffer.average_concentration - odor_baseline_value < 0) ? 0 : sampleBuffer.average_concentration - odor_baseline_value;
		// if(history.measurement_avg[history.size] < 0) history.measurement_avg[history.size] = 0;
		history.measurement_std[history.size] = sampleBuffer.stdDev_concentration;
		history.size++;
		history_print(history);

		// send it to friends
		// if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
		// 	Message msg(ID, 1, history.position[(history.size)-1], sampleBuffer.average_concentration, goal_position, 0);
  //   		send_msg(msg);
  //   	}

    	double m_buffer[4] = {0};
		int timeout = 10;
		int success = -1;

    	//send the message
		m_buffer[0] = history.size;
		m_buffer[1] = current_position.x;
		m_buffer[2] = current_position.y;
		m_buffer[3] = (sampleBuffer.average_concentration - odor_baseline_value < 0) ? 0 : sampleBuffer.average_concentration - odor_baseline_value;
		//variable = (condition) ? expressionTrue : expressionFalse;
		do{
			success = send(sock, m_buffer, sizeof(m_buffer), 0);
			--timeout;
		} while(success == -1 && timeout > 0);
		if(success > 0)
			printf("successfully sent measurement %f: (%f, %f) %f\n", m_buffer[0], m_buffer[1], m_buffer[2], m_buffer[3]);
		else
			printf("sending failed with timeout!\n");

		// send the measurement to the server and wait for new orders
		FSM.state = STATE_LISTEN;
		FSM.hook  = state_listen;
	}
}

void state_listen(){
	int result;
	double g_buffer[2] = {0}; 

	//wait for new orders
	result = read(sock, g_buffer, sizeof(g_buffer));
	if(result > 0){
		printf("received a goal position : (%f, %f)\n", g_buffer[0], g_buffer[1]);
		// check if it was a termination message
		if(g_buffer[0] == -1 && g_buffer[1] == -1){
			goal_position.x = 3;
    		goal_position.y = float(ID-100)/2.0;
    		finished = 1;
		}else{
			//set the goal position
			goal_position.x = g_buffer[0];
			goal_position.y = g_buffer[1];
		}
		//find the path using visibility graph
	    VisiLibity::Point start(current_position.x, current_position.y);
	    VisiLibity::Point finish(goal_position.x, goal_position.y);
	    VisiLibity::Polyline Polyline_path = my_environment.shortest_path(start, finish, vg, epsilon);
	    // copy the path in another variable
	    for(unsigned k=0; k<Polyline_path.size(); k++){
	    	Position p(Polyline_path[k].x(), Polyline_path[k].y(),0);
	        path.push_back(p);
	        log_trajectory(p);
	    }
	    i = path.begin();

		//move to the goal position
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
	}
}

void state_moving(){
	//printf("moving toward the goal\n");
	//verify if arrived to the goal if applicable
	if(dist_to_goal(goal_position) > 0.05){
		//move
		if(dist_to_goal(*i) <= 0.05 && i != path.end())
			i++;
		go_to_goal(*i);
		//obstacle avoidance if necessary
		if(obstacle_or_not() == 1){
			FSM.state = STATE_OBSTACLE_AVOIDANCE;
			FSM.hook  = state_obstacle_avoidance;
			return;
		}
		//wb_differential_wheels_set_speed(speed[0],speed[1]);
		set_wheels_speed(speed[0],speed[1]);
		//log the position
		//log_position();
	}else{
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		sampleBuffer.reset_all();
		path.clear();
		if(finished){
			FSM.state = STATE_FINISHED;
		}else{
			//back to measuring
			FSM.state = STATE_MEASURING;
			FSM.hook  = state_measuring;
		}
	}

}

void state_obstacle_avoidance(){
	//if(VERBOSE) printf("[%d] avoiding obstacles .......\n", ID);
	
	//if no obstacle,
	if(obstacle_or_not() == 0 || dist_to_goal(goal_position) <= 0.05){
		//printf("no obstacle anymore..XXXXXXX\n");
		//wb_differential_wheels_set_speed(0,0);
		set_wheels_speed(0,0);
		//back to moving
		FSM.state = STATE_MOVING;
		FSM.hook = state_moving;
		return;
	}

	obstacle_avoidance(speed);
	
	//wb_differential_wheels_set_speed(speed[0],speed[1]);
	set_wheels_speed(speed[0],speed[1]);
	usleep(100 * 1000);
}


// =========================== MAIN ============================== //


int main(int argc, char **argv){
	
	init();
	init_TCP_socket();
	establish_baseline();
	read_obstacle_list();
	//odor_baseline_value = 30000;

	// visibility graph (visilibity library)
	std::cout << "Validating environment model . . . ";
  	if(  my_environment.is_valid( epsilon )  )
    	std::cout << "OK" << std::endl;
  	else{
    	std::cout << std::endl << "Warning:  Environment model "
	    << "is invalid." << std::endl
	    << "A valid environment model must have" << std::endl
	    << "   1) outer boundary and holes pairwise "
	    << "epsilon -disjoint simple polygons" << std::endl
	    << "   (no two features should come "
	    << "within epsilon of each other)," << std::endl 
	    << "   2) outer boundary is oriented ccw, and" 
	    << std::endl
	    << "   3) holes are oriented cw."
	    << std::endl;
    	exit(1);
  	}
  	VisiLibity::Visibility_Graph vg(my_environment, epsilon);
  	
	// main loop
	while (FSM.state != STATE_FINISHED){
		update_position();
		
		//send info to the others
		// if(simulation_parameters.scenario == COM_COOR){
		// 	// if a robot is already sampling (i.e. STATE_MEASURING), its goal should not be swapped with anyone. Therefore we send 0 as goal and current position
		// 	if(FSM.state == STATE_MEASURING && n_time_step%100 == 0){
		// 		Position tmp;
		// 		Message msg(msg_counter, ID, 1, tmp, sampleBuffer.average_concentration, tmp, 1, 0);
		// 		send_msg(msg);
		// 	}else{
		// 		if(n_time_step%100 == 0){
		// 			//wb_emitter_set_channel(emitter, 10);
		// 			Message msg(++msg_counter, ID, 1, current_position, sampleBuffer.average_concentration, goal_position, 1, 0);
		// 			//printf("[%d] I am sendig my position %f %f %f and my goal %f %f %f\n", ID,current_position[0],current_position[1], current_position[2], goal_position[0],goal_position[1],goal_position[2]);
  //   				//wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
  //   				send_msg(msg);
  //   			}
  //   		}
  //   	}
		// //receive info from colleagues
		// if(simulation_parameters.scenario == COM_NO_COOR || simulation_parameters.scenario == COM_COOR){
		// 	//printf("Robot %d has %d unread messages! \n", ID, wb_receiver_get_queue_length(receiver));
		// 	receive_from_friends();
		// 	//printf("[%d] my history size is %d\n", ID, history.size);
		// }
		// //resend all the un-acked messages
		// // int tmp_counter = 0;
		// for(std::vector<Message>::iterator it = ack_msg_vec.begin(); it != ack_msg_vec.end(); it++){
		// 	//printf("[%lld] (%d) sending un-acked message %d to %d\n", current_timestamp(), tmp_counter++, it->msg_ID, it->receiver_ID);
		// 	//(*it).print_message();
  //           send_msg(*it);
		// }
	   	//change state
    	FSM.hook();
    	//next
    	next_iteration_time();
    	n_time_step++;
	}
	// final logs
	log_history();
	cleaning_up();
	//free the allocated memory to obstacle array
    for(int i = 0; i < nbr_obs; i++)
    	delete[] obstacles[i];
	delete[] obstacles;
	exit(0);
}

