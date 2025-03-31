/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        supervisor_target.cpp
 * description: The supervisor gets the goal position from a robot and places 
                a target on that point for visualization purposes. It also 
                exits the simulation when all the robots are done.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h> //time
#include <unistd.h> //access
#include <sys/time.h>
#include <fstream>
#include <map>

#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>
#include <webots/robot.h>
#include "../controller_STE_multi-robot/Message.h"

#define MAX_NB_ROBOTS   3
#define STEP_SIZE       64
#define YMIN            0.0
#define YMAX            4.0
#define XMIN            2.75
#define XMAX            12.75
#define SAFETY_MARGIN   0.2
#define VERBOSE         1
//#define TIMEOUT         600000 // miliseconds

WbDeviceTag receiver[MAX_NB_ROBOTS];         // Supervisor receivers
WbDeviceTag emitter;

// message structure
struct message{
    int sender_ID;
    double position[3];
    double concentration;
    int termination_flag;
}message;

char ID_name[50];

// obstacles definitions
int nbr_obs = 4;
float margin = 0.2;
float ** obstacles = NULL;

char* strcat_robot_ID(char * str, int ID){
    sprintf(ID_name, "%s%d",  str, ID);
    return ID_name;
}

long long current_timestamp(){
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

void read_obstacle_list(){
    // open file
    std::ifstream infile("../../data/obstacle_list.txt");
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

bool is_in_obstacle(float x, float y, float obs_xmin, float obs_xmax, float obs_ymin, float obs_ymax){
    if(x > obs_xmin && x < obs_xmax && y > obs_ymin && y < obs_ymax)
        return true;
    else
        return false;
}

int is_in_obstacle_list(float x, float y, float ** obs_list, int nbr_obs){
    // gets an obstacle list and number of obstacles as input
    // returns the index of the obstacle that the position is in 
    // if the position is not in any obstacles, then -1 is returned
    for(int i = 0; i < nbr_obs; i++){
        if(is_in_obstacle(x, y, obs_list[i][0], obs_list[i][2], obs_list[i][1], obs_list[i][3]))
            return i;
    }
    return -1;
}

void init_receiver_s()
{
    receiver[0] = wb_robot_get_device("receiver_radio");
    wb_receiver_enable(receiver[0], STEP_SIZE);
    emitter = wb_robot_get_device("emitter_sup");
}

void send_msg_sc(int number_of_robots)
{   
    // set channel
    wb_emitter_set_channel(emitter, 600);
    // send a message to the controller
    Message centralized_goal;


    // CONSTANT VALUE
    // centralized_goal.receiver_ID = 103;
    // centralized_goal.position.x = 12.0;
    // centralized_goal.position.y = 4.0;
    //printf("Message sent to the robot %d with goal: %f %f\n", sup_to_con.receiver_ID, sup_to_con.position.x, sup_to_con.position.y);


    // RANDOM VALUE
    // for(int i=0; i<number_of_robots; i++){
    //     do{
    //         centralized_goal.receiver_ID = i+1;

    //         centralized_goal.position.x = (((double)rand() / (double)RAND_MAX) * (5 - XMIN - SAFETY_MARGIN*2)) + SAFETY_MARGIN + XMIN;
    //         centralized_goal.position.y = (((double)rand() / (double)RAND_MAX) * (YMAX - YMIN - SAFETY_MARGIN*2)) + SAFETY_MARGIN + YMIN;
    //         centralized_goal.position.heading = 0;
    //     }while(is_in_obstacle_list(centralized_goal.position.x, centralized_goal.position.y, obstacles, nbr_obs) != -1);
    //     wb_emitter_send(emitter, &centralized_goal, sizeof(Message));
    // }
    //printf("Message sent to the robot %d with RANDOM goal: %f %f\n", centralized_goal.receiver_ID, centralized_goal.position.x, centralized_goal.position.y);


    // BELIEF MERGING

}

void merge_beliefs() {
    Message centralized_goal;
    // static std::map<int, Position> received_positions; // Almacena las posiciones de los robots
    // static const int required_ids[] = {101, 102, 103}; // IDs esperados

    // // Verifica si el sender_ID es 101, 102 o 103
    // if (msg.sender_ID == 101 || msg.sender_ID == 102 || msg.sender_ID == 103) {
    //     received_positions[msg.sender_ID] = msg.position; // Almacena la posición
    // }

    // // Si hemos recibido los tres mensajes
    // if (received_positions.size() == 3) {
    //     Position centralized_goal;
    //     centralized_goal.x = 0.0;
    //     centralized_goal.y = 0.0;

    //     // Calcular el promedio de las posiciones
    //     for (int id : required_ids) {
    //         centralized_goal.x += received_positions[id].x;
    //         centralized_goal.y += received_positions[id].y;
    //     }
    //     centralized_goal.x /= 3.0;
    //     centralized_goal.y /= 3.0;

    //     wb_emitter_send(emitter, &centralized_goal, sizeof(Message));

    //     // Imprimir el resultado para depuración
    //     printf("Centralized Goal: x = %f, y = %f\n", centralized_goal.x, centralized_goal.y);

    //     // Reiniciar la estructura para la siguiente ronda
    //     received_positions.clear();
    // }
    centralized_goal.position.x = 7.0;
    centralized_goal.position.y = 3.0;
    wb_emitter_send(emitter, &centralized_goal, sizeof(Message));
}


// _________main___________
int main(void){
    const void * news;
    FILE* ground_truth_log_file;
    int termination_flag_counter=0;
    int number_of_robots=0;

    //initialization
    wb_robot_init();

    //receiver initialization
    init_receiver_s();

    if(access("../../data/simulation_parameters/number_of_robots.txt", F_OK) != -1 ){
        FILE * sim_param_file = fopen("../../data/simulation_parameters/number_of_robots.txt", "r");
        int ok = fscanf(sim_param_file, "%d", &number_of_robots);
        if(ok) printf("number of robots : %d\n", number_of_robots);
        fclose(sim_param_file);
        if(number_of_robots > MAX_NB_ROBOTS){
            number_of_robots = MAX_NB_ROBOTS;
            printf("Number of robots is now set to %d, it cannot be set higher!\n", MAX_NB_ROBOTS);
        }
    }
    printf("remove successful\n");

    // load obstacles
    read_obstacle_list();

    // randomize the source position on y
    srand(time(NULL));
    double source_position[3] = {0};
    if (access("../../data/source_pos.txt", F_OK) != -1) {
       FILE * sim_param_file = fopen("../../data/source_pos.txt", "r");
        int ok = fscanf(sim_param_file, "%lf %lf", &source_position[0], &source_position[1]);
        source_position[2] = 0.13;
        if(ok) printf("source of gas : %lf %lf %lf\n", source_position[0], source_position[1], source_position[2]);
        fclose(sim_param_file);  
    }
    //make sure the source is not in an obstacle
    if (is_in_obstacle_list(source_position[0], source_position[1], obstacles, nbr_obs) != -1){
        printf("The source is in an obstacle, randomizing the position!!!\n");
        do{
            source_position[0] = 12.7;
            source_position[1] = (((double)rand() / (double)RAND_MAX) * (YMAX - YMIN - SAFETY_MARGIN*2)) + SAFETY_MARGIN + YMIN;
            source_position[2] = 0.13;
        }while(is_in_obstacle_list(source_position[0], source_position[1], obstacles, nbr_obs) != -1);
    }
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("SOURCE_ODOR_0"),"translation"), source_position);
    //wb_emitter_send(emitter, source_position, sizeof(double)*3);
    
    // remove unnecessary robots and their goals
    /*for(int i=number_of_robots+1; i<=MAX_NB_ROBOTS; i++){
        wb_supervisor_node_remove(wb_supervisor_node_get_from_def(strcat_robot_ID("n",i+100)));
        wb_supervisor_node_remove(wb_supervisor_node_get_from_def(strcat_robot_ID("goal_",i+100)));
    }*/

    // randomize the robot's inital position
    for(int i=0; i<number_of_robots; i++){
        double robot_position[3] = {0};
        do{
            robot_position[0] = (((double)rand() / (double)RAND_MAX) * (5 - XMIN - SAFETY_MARGIN*2)) + SAFETY_MARGIN + XMIN;
            robot_position[1] = (((double)rand() / (double)RAND_MAX) * (YMAX - YMIN - SAFETY_MARGIN*2)) + SAFETY_MARGIN + YMIN;
            robot_position[2] = 0;
        }while(is_in_obstacle_list(robot_position[0], robot_position[1], obstacles, nbr_obs) != -1);
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(strcat_robot_ID("n",i+101)),"translation"), robot_position);
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(strcat_robot_ID("goal_",i+101)),"translation"), robot_position);
    
        //log the real position of the source
        char file_name[256];
        sprintf(file_name, "../../../Results/current/%d/param_log.csv", i+101);
        while(access(file_name, F_OK) == -1) { }; // wait for the controller to mkdir the "current" folder and the param_log file
        int fprintf_return = -1;
        do{
            ground_truth_log_file = fopen(file_name, "a");
            fprintf_return = fprintf(ground_truth_log_file, "source_position %f %f %f\n", source_position[0], source_position[1], source_position[2]);
            fclose(ground_truth_log_file);
        }while(fprintf_return <= 0);

        // backup
        sprintf(file_name, "../../../Results/current/%d/source_pos.csv", i+101);
        FILE * source_pos_log_file = fopen(file_name, "a");
        fprintf(source_pos_log_file, "source_position %f %f %f\n", source_position[0], source_position[1], source_position[2]);
        fclose(source_pos_log_file);
    }

    //long long start_time = current_timestamp();
    //wb_supervisor_animation_start_recording("../../../Results/current/animation.html");

    // start the controller
    while (wb_robot_step(STEP_SIZE) != -1 ){ //&& current_timestamp()-start_time < TIMEOUT
        //listen to the radio, if some news arrive, then remove the last marker, and place a new one
        if(wb_receiver_get_queue_length(receiver[0])){
            // receive
            news = wb_receiver_get_data(receiver[0]);
            Message msg(news);

            // if message was addressed to supervisor
            if(msg.receiver_ID == 0){
                // Check if it's a belief map message
                if (msg.measurement0_goal1 == 2) { // Suponiendo que 2 representa un belief map
                    printf("SUPERVISOR -> Received belief map from Robot ID [%d]:\n", msg.sender_ID);
                    printf("SUPERVISOR -> Received belief map with size: %zu\n", msg.belief_map.size());
                    
                    // Verificar que belief_map tiene valores y luego imprimir los primeros 10 valores
                    if (msg.belief_map.size() > 0) {
                        for (size_t i = 0; i < 10 && i < msg.belief_map.size(); i++) {
                            printf("pdf[%zu]: %f\n", i, msg.belief_map[i]);
                        }
                    } else {
                        printf("Error: Belief map is empty or not received correctly.\n");
                        //a
                    }
                }


                //terminate ?
                else if((msg.position.x == -1) && (msg.position.y == -1)){
                    //eliminate the robot that finished and its target indicator
                    double robot_position[3] = {-10,-10,0};
                    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(strcat_robot_ID("n",msg.sender_ID)),"translation"), robot_position);
                    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(strcat_robot_ID("goal_",msg.sender_ID)),"translation"), robot_position);
                    // wb_supervisor_node_remove(wb_supervisor_node_get_from_def(strcat_robot_ID("n",msg.sender_ID)));
                    // wb_supervisor_node_remove(wb_supervisor_node_get_from_def(strcat_robot_ID("goal_",msg.sender_ID)));
                    printf("robot ID %s removed\n", strcat_robot_ID("n",msg.sender_ID));
                    //Wait for all the robots to be finished before reverting the simulation 
                    if(++termination_flag_counter == MAX_NB_ROBOTS) 
                        break;
                }else{
                    // move the target point for each robot
                    if(VERBOSE) printf("received position : %f %f for ID [%d]\n", msg.position.x , msg.position.y, msg.sender_ID);
                    double robot_goal[3] = {msg.position.x, msg.position.y, 0.0};
                    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(strcat_robot_ID("goal_",msg.sender_ID)),"translation"), robot_goal);
                }
            }
            wb_receiver_next_packet(receiver[0]);
        }
        // send
        // send_msg_sc(number_of_robots);
        merge_beliefs();
    }

    //wb_supervisor_animation_stop_recording();
    //move the current folder to one with the current timestamp 
    char folder_name[256];
    sprintf(folder_name, "../../../Results/%lld", current_timestamp());
    rename("../../../Results/current", folder_name);

    //clean up and exit
    //free the allocated memory to obstacle array
    for(int i = 0; i < nbr_obs; i++)
        delete[] obstacles[i];
    delete[] obstacles;
    // wb_supervisor_world_reload(); 
    wb_supervisor_simulation_quit(EXIT_SUCCESS);
    wb_robot_cleanup();
    return 0;
}
