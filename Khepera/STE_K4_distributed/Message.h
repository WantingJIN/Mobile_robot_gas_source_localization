#ifndef MESSAGE_H
#define MESSAGE_H

#include "Position.h"

class Message{ 
    public: 
    
    int msg_ID;
    int sender_ID; 
    int receiver_ID; //if == 0 -->> msg for supervisor, otherwise it's for robots
    Position position;
    int finish_flag; // -1: not finished, 1: finished        
    float belief_map[MAP_SIZE];
    float buffer[MAP_SIZE + 7]; // 4 additional values for msg_ID, sender_ID, receiver_ID, finish_flag, position (x, y, heading)
    int buffer_size;


    //constructor : everything set to 0
    Message();
    //constructor : set parameters
    Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, int _flag, float *_belief_map);
    Message(const void * m_buffer);

    void params_to_buffer();
    void buffer_to_params();

    void print_message();
    
}; 

#endif