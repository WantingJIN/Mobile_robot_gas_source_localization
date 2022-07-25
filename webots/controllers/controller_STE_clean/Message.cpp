#include "Message.h"
#include <cstring>
#include <stdio.h>
using namespace std;

//constructor : everithing set to 0
Message::Message(){
    sender_ID = 0; 
    receiver_ID = 0; 
    position[0] = 0; position[1] = 0; position[2] = 0;
    concentration = 0;
    goal[0] = 0; goal[1] = 0; goal[2] = 0;
    measurement0_goal1 = 0;
    params_to_buffer();
    buffer_size = 0;
}

Message::Message(int _sender_ID, int _receiver_ID, double _position[], double _concentration, double _goal[], int _flag){
    sender_ID = _sender_ID; 
    receiver_ID = _receiver_ID; 
    memcpy(position,_position,sizeof(double)*3);
    concentration = _concentration;
    memcpy(goal,_goal,sizeof(double)*3);
    measurement0_goal1 = _flag;
    params_to_buffer();
    buffer_size = sizeof(double)*10;
}

Message::Message(const void * m_buffer){
	buffer_size = sizeof(double)*10;
    memcpy(buffer, m_buffer, buffer_size);
    buffer_to_params();
}

void Message::params_to_buffer(){
	buffer[0] = sender_ID;
	buffer[1] = receiver_ID;
	buffer[2] = position[0];
	buffer[3] = position[1];
	buffer[4] = position[2];
	buffer[5] = concentration;
	buffer[6] = goal[0];
	buffer[7] = goal[1];
	buffer[8] = goal[2];
	buffer[9] = measurement0_goal1;
}

void Message::buffer_to_params(){
	sender_ID = buffer[0];
	receiver_ID = buffer[1];
	position[0] = buffer[2];
	position[1] = buffer[3];
	position[2] = buffer[4];
	concentration = buffer[5];
	goal[0] = buffer[6];
	goal[1] = buffer[7];
	goal[2] = buffer[8];
	measurement0_goal1 = buffer[9];
}

void Message::print_message(){
    printf("Message from [%d] to [%d]. Position (%f,%f,%f). Concentration = %f. Goal (%f,%f,%f). Flag = %d.\n", 
    	sender_ID, receiver_ID, position[0], position[1], position[2], concentration, goal[0], goal[1], goal[2], measurement0_goal1);
}