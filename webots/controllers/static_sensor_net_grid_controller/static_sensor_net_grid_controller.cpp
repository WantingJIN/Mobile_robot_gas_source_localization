/*
 * File:         statuc_sensor_net_controller.c
 * Description:  This is a supervisor node to place some static sensors and log their values
 * Author:       Faezeh Rahbar - Jan 2020
 * Note:         
 *               
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#include "SampleBuffer.h"
#include "tinyxml2.h"

#define PI 3.14159265359
#define TIME_STEP           64 //adjusts the speed (ms)
#define SENSOR_NUM_MAX      250
#define ODOR_FILTER_LENGTH  150 //~10sec //number of measurements before averaging
#define ITERATIONS          1
#define RANDOM0_GRID1       1
#define RANDOM_ITERATIONS   0
#define MAX_OBSTACLE_NBR    4

#define XMIN 2.75
#define XMAX 12.7418
#define XSTEP 0.1586
#define XNUM 64

#define YMIN 0.01
#define YMAX 3.9916
#define YSTEP 0.0632
#define YNUM 64

using namespace tinyxml2;

WbFieldRef root_children_field;
WbDeviceTag odor_tag[SENSOR_NUM_MAX];
WbDeviceTag wind_tag[SENSOR_NUM_MAX];
SampleBuffer* sampleBuffer = new SampleBuffer[SENSOR_NUM_MAX];
int sensor_number = 0;
double sensor_position[SENSOR_NUM_MAX][3] = {{0}};
double source_position[3] = {0};
int total_nbr_iteration = 0;
int iteration = 0;

int nbr_obstacle = 0;
float obstacles[MAX_OBSTACLE_NBR][4] = {0};

void change_number_of_sensors();
void set_sensor_position(int i, double * position);
void read_sensor_positions_from_file();
void place_sensor(int sensor_ID, double* position);
void wind_read(int i, double* intensity, double* angle);
bool in_obstacle(double* position);

void init(){
    int i = 0;

    wb_robot_init();
    srand((long)time(NULL));

    // find total number of itrations
    total_nbr_iteration = RANDOM_ITERATIONS + ceil(XNUM*YNUM / (float)SENSOR_NUM_MAX);
    printf("total_nbr_iteration : %d\n", total_nbr_iteration);
    sensor_number = SENSOR_NUM_MAX;

    //Get static sensor network
    WbNodeRef root_node = wb_supervisor_node_get_self();
  
  
    //Define position of new node inside the children field
    root_children_field = wb_supervisor_node_get_field(root_node, "children");
  
    //change_number_of_sensors();
    //read_sensor_positions_from_file();
    //create new sensor nodes
    for (i=0; i<SENSOR_NUM_MAX; i++){
        //find the initial sensor position
        set_sensor_position(i, sensor_position[i]);
        //apply the position to both the odor sensor and wind sensor of ID i
        place_sensor(i, sensor_position[i]);
        //activate the sensors
        //odor
        char tag_name[256];
        sprintf(tag_name,"sensor_odor_%d",i);
        odor_tag[i] = wb_robot_get_device(tag_name);
        wb_receiver_enable(odor_tag[i], TIME_STEP);
        //wind
        sprintf(tag_name,"sensor_wind_%d",i);
        wind_tag[i] = wb_robot_get_device(tag_name);
        wb_receiver_enable(wind_tag[i],TIME_STEP);
        //set buffer length for all sensors
        sampleBuffer[i].set_buffer_length(ODOR_FILTER_LENGTH);
    }

    //read and save the obstacle list
    FILE * obstacle_file = fopen("../../data/obstacle_list.txt", "r");
    fscanf(obstacle_file, "%d\n", &nbr_obstacle);
    for(int i = 0; i < nbr_obstacle; i++){
        fscanf(obstacle_file, "%f %f %f %f\n", &(obstacles[i][0]), &(obstacles[i][1]), &(obstacles[i][2]), &(obstacles[i][3]));
        printf("obstacle %d: %f %f %f %f\n", i, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
    }
    fclose(obstacle_file);

    // set the source position randomly (on upperwind half on x)
    do{
        source_position[0] = ( (rand()/(double)RAND_MAX) * (XMAX-XMIN)/2 ) + XMIN + (XMAX-XMIN)/2;
        source_position[1] = ( (rand()/(double)RAND_MAX) * (YMAX-YMIN) ) + YMIN;
        source_position[2] = 0.1;
    } while(in_obstacle(source_position));
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("SOURCE_ODOR_0"),"translation"), source_position);
}

bool in_obstacle(double* position){
    // returns true if position in inside obstacle
    float x = position[0];
    float y = position[1];
    //obstacles (traning_map1, 4 rectangles)
    // if(x >= 11.5 && x <= 12 && y >= 0 && y <= 1.5)
    //     return true;
    // if(x >= 9 && x <= 12 && y >= 1 && y <= 1.5)
    //     return true;
    // if(x >= 5 && x <= 8 && y >= 2.5 && y <= 3)
    //     return true;
    // if(x >= 5 && x <= 5.5 && y >= 2.5 && y <= 4)
    //     return true;
    for(int i = 0; i < nbr_obstacle; i++){
        if(x >= obstacles[i][0] && y >= obstacles[i][1] && x <= obstacles[i][2] && y <= obstacles[i][3])
            return true;
    }
    // if none of the above, then out of obstacle
    return false;
}

void set_sensor_position(int i, double * position){
    if(total_nbr_iteration - iteration <= RANDOM_ITERATIONS){
        //random
        position[0] = ( (rand()/(double)RAND_MAX) * (XMAX-XMIN) ) + XMIN;
        position[1] = ( (rand()/(double)RAND_MAX) * (YMAX-YMIN) ) + YMIN;
        position[2] =  0.1;
    }else{
        //grid
        int index = i + iteration * SENSOR_NUM_MAX;
        if (index >= XNUM*YNUM){
            if(sensor_number == SENSOR_NUM_MAX)
                sensor_number = i;
            position[0] = 0;
            position[1] = 0;
            position[2] = 0.1;
        }
        else{
            position[0] = XMIN + floor(index/YNUM) * XSTEP;
            position[1] = YMIN + (index%YNUM) * YSTEP;
            position[2] = 0.1;
        }
        //printf("i %d index %d position %f %f \n", i, index, position[0], position[1]);
    }
}

void read_sensor_positions_from_file(){
    int res = 0;
    FILE *pFile = fopen("../../data/sensor_position.csv", "r");
    //read the number of sensors
    res = fscanf(pFile, "%d", &sensor_number);
    if(res == 0)
        printf("/!\\ no sensor number found!!! \n");
    //read the positions
    for(int i=0; i<sensor_number; i++){
        res = fscanf(pFile, "%lf, %lf", &(sensor_position[i][0]), &(sensor_position[i][1]));
        sensor_position[i][2] = 0.1;
    }
    //set other sensors position to 0
    for(int i=sensor_number; i<SENSOR_NUM_MAX; i++){
        sensor_position[i][0] = 0.0;
        sensor_position[i][1] = 0.0;
        sensor_position[i][2] = 0.0;
    }
}

void change_number_of_sensors(){
    //open txt file and make the necessary modifications, then copy paste in the world file
    FILE * wbo_file = fopen("../../protos/sensor_odor_receiver.wbo", "w");
    fprintf(wbo_file, "#VRML_OBJ R2019b utf8\n");
    for (int i = 0; i < SENSOR_NUM_MAX; i++){
        fprintf(wbo_file, "DEF SENSOR_ODOR_%d Receiver {\n\
        translation %f %f 0.1\n\
        rotation 1 0 0 -1.5708\n\
        children [\n\
            Shape {\n\
                appearance Appearance {\n\
                    material Material {\n\
                        diffuseColor 0 1 0\n\
                    }\n\
                }\n\
                geometry DEF CYLINDER_ODOR Cylinder {\n\
                    height 0.02\n\
                    radius 0.01\n\
                }\n\
            }\n\
        ]\n\
        name \"sensor_odor_%d\"\n\
        boundingObject USE CYLINDER_ODOR\n\
        physics Physics {\n\
        }\n\
        channel %d\n\
        }\n", i, 3.0+(rand()/(double)RAND_MAX)*10.0, (rand()/(double)RAND_MAX)*4.0, i, i);
    }
    fclose(wbo_file);

    //open txt file and make the necessary modifications
    FILE * wbo_file_w = fopen("../../protos/sensor_wind_receiver.wbo", "w");
    fprintf(wbo_file_w, "#VRML_OBJ R2019b utf8\n");
    for (int i = 0; i < SENSOR_NUM_MAX; i++){
        fprintf(wbo_file_w, "DEF SENSOR_WIND_%d Receiver {\n\
        translation %f %f 0.2\n\
        rotation 1 0 0 -1.5708\n\
        children [\n\
            Shape {\n\
                appearance Appearance {\n\
                    material Material {\n\
                        diffuseColor 1 0 0\n\
                        emissiveColor 1 0 0\n\
                    }\n\
                }\n\
                geometry DEF CYLINDER_WIND Cylinder {\n\
                    height 0.02\n\
                    radius 0.02\n\
                }\n\
            }\n\
        ]\n\
        name \"sensor_wind_%d\"\n\
        boundingObject USE CYLINDER_WIND\n\
        physics Physics {\n\
        }\n\
        channel %d\n\
        }\n", i, 3.0+(rand()/(double)RAND_MAX)*10.0, (rand()/(double)RAND_MAX)*4.0, i, i+SENSOR_NUM_MAX);
    }
    fclose(wbo_file_w);

    //add the node to the scene tree
    //wb_supervisor_field_import_mf_node(root_children_field, -1, "../../protos/sensor_odor_receiver.wbo");
    
    //exit to start again
    printf("number of sensors changed successfully, now close and run again the simulation.\n");
    exit(0);
}

double odor_read(int i){
    double odor_sample = 0;
    while (wb_receiver_get_queue_length(odor_tag[i]) > 0) {
        assert(wb_receiver_get_data_size(odor_tag[i]) == sizeof(double));
        odor_sample = *(const double *)wb_receiver_get_data(odor_tag[i]);
        wb_receiver_next_packet(odor_tag[i]);
    }
    //printf("odor measurement : %f \n",odor_measurement);
    
    //if sensor is placed in obstacle, we send only 0
    if(in_obstacle(sensor_position[i]))
        return 0;
    else
        return odor_sample;
}

void wind_read(int i, double* intensity, double* angle){
    struct vect3D{
        double x;
        double y;
        double z;
    };
    struct vect3D v = {0};
    while (wb_receiver_get_queue_length(wind_tag[i]) > 0) {
        assert(wb_receiver_get_data_size(wind_tag[i]) == sizeof(struct vect3D));
        v = *(const struct vect3D*)wb_receiver_get_data(wind_tag[i]);
        wb_receiver_next_packet(wind_tag[i]);
    }
    //convert the 3D vector from the sensor to intensity and heading
    *intensity = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    *angle = atan2(v.y, v.x)/PI*180; //degree
    //keep the 3D vector
    //*ux = v.x; 
    //*uy = v.y;
    //*uz = v.z;

    //if sensor is placed in obstacle, we send only 0
    if(in_obstacle(sensor_position[i]))
        *intensity = 0;
        *angle = 0;
}

void place_sensor(int sensor_ID, double* position){
    //wind sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),"translation"), position);
    //odor sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID+SENSOR_NUM_MAX),"translation"), position);
    //correct orientation for wind sensors /!\ determines the origin for wind angle! should match with Khepera IV
    const double rotation_wind_sensors[] = {0, 0.70710528118436, -0.707108281185553, -3.1415923071795864};
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),"rotation"), rotation_wind_sensors);
}

bool still_wait(){
    //place one sensor at the end of the tunnel to measure the waiting time
    double tmp_pos[3] = {0.1,0,0.1};
    wind_read(SENSOR_NUM_MAX-1, &sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_intensity, &sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle);
    //printf("wind angle %f, tan %f \n", sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle, tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle));
    tmp_pos[1] = source_position[1] - tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle) * (source_position[0] - tmp_pos[0]);
    //printf("y %f\n", tmp_pos[1]);
    if(tmp_pos[1] > YMAX+0.5)
        tmp_pos[1] = YMAX+0.5;
    if(tmp_pos[1] < YMIN-0.5)
        tmp_pos[1] = YMIN-0.5;
    //printf("y %f\n", tmp_pos[1]);
    if(tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle) == 0)
        tmp_pos[0] = XMIN;
    else
        tmp_pos[0] = source_position[0] - (fabs(source_position[1] - tmp_pos[1])) / tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle);
    //printf("x %f\n", tmp_pos[0]);
    place_sensor(SENSOR_NUM_MAX-1, tmp_pos);

    // measure odor concentration
    double tmp_c = odor_read(SENSOR_NUM_MAX-1);
    //printf("sensor reading %f\n", tmp_c);
    if(tmp_c > 0)
        return false;
    else
        return true;
}

int main() {
    int i = 0;
    int n_samples = 0;
    int wait = 200;

    init();
    
    //Get the sensor values and write them in a file
    while(iteration < total_nbr_iteration){
        if(wait > 0){
            wait--; //wait for the plume to be established
            if(wait == 1) printf("done waiting!\n");
            // if(still_wait() == false){
            //     wait = 0;    
            //     printf("Plume established!\n");
            //     place_sensor(SENSOR_NUM_MAX-1, sensor_position[SENSOR_NUM_MAX-1]);
            // }
        }
        else{
            //SAMPLE
            if(n_samples < ODOR_FILTER_LENGTH){
                for(i=0; i < sensor_number; i++){
                    //continue measuring odor concentration
                    sampleBuffer[i].instant_concentration = odor_read(i);
                    //printf("odor sensor %d -> %f \n", i, sampleBuffer[i].instant_concentration);
                    //wind 
                    wind_read(i, &sampleBuffer[i].instant_wind_intensity, &sampleBuffer[i].instant_wind_angle);
                    //printf("wind sensor %d -> %f %f\n", i, sampleBuffer[i].instant_wind_intensity, sampleBuffer[i].instant_wind_angle);
                    sampleBuffer[i].add_all_to_buffer();
                }
                n_samples++;
            }else{
            //LOG
                //printf("now we're logging\n");
                //read plugin's info using tinyxml2 library
                float Q = 0;
                XMLDocument doc;
                doc.LoadFile("../../plugins/physics/odor_physics/results/configuration.xml"); //this file is generated by the odor plugin
                const char* Q_line = doc.FirstChildElement("OdorPhysics")->FirstChildElement("FilamentSourceList")->FirstChildElement("FilamentSourceConstant")->FirstChildElement("ReleaseAmount")->GetText();
                sscanf(Q_line, "%f", &Q);
                printf("load odor physics \n");

                //read sensors' data and log
                char logfilename[50];
                sprintf(logfilename, "../../log/spx%f_spy%f.csv", source_position[0], source_position[1]);
                // TODO: creat the file if not exist, check the avaibility before write
                FILE * data_base = fopen(logfilename, "a");
                // if (!data_base){
                   // printf("Error opening files in log\n");
                   // continue;
                // }
                printf("read sensor data and log \n");
                for(i=0; i < sensor_number; i++){
                    //take the average
                    sampleBuffer[i].average_all();
                    //log in the data_base
                    fprintf(data_base, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
                        source_position[0], source_position[1], source_position[2], Q,                  // Sx, Sy, Sz, Q,
                        sampleBuffer[i].average_wind_intensity, sampleBuffer[i].average_wind_angle,     // wind_intensity, wind angle,
                        sensor_position[i][0], sensor_position[i][1], sensor_position[i][2],            // px, py, pz,
                        sampleBuffer[i].average_concentration, sampleBuffer[i].stdDev_concentration);   // C, stdC
                }
                fclose(data_base);
                printf("Iteration %d over\n", iteration+1);
                                
                n_samples = 0;
                iteration++;

                // reshuffle sensors for next iteration
                for (i=0; i<SENSOR_NUM_MAX; i++){
                    //find the initial sensor position
                    set_sensor_position(i, sensor_position[i]);
                    //apply the position to both the odor sensor and wind sensor of ID i
                    place_sensor(i, sensor_position[i]);
                    //reset the buffer of sensor for next iteration
                    sampleBuffer[i].reset_all();
                }
            }
        }
        //NEXT
        wb_robot_step(TIME_STEP);
    }
    printf("FINISHED all %d iterations)\n", total_nbr_iteration);
    wb_supervisor_simulation_quit(EXIT_SUCCESS);
    wb_robot_cleanup();
    return 0;
}
