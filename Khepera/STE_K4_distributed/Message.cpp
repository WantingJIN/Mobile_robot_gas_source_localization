#include "Message.h"
#include <cstring>
#include <stdio.h>
using namespace std;

//constructor : everithing set to 0
Message::Message(){
	msg_ID = 0;
    sender_ID = 0; 
    receiver_ID = 0; 
    position.set_to_zero();
    concentration = 0;
    goal.set_to_zero();
    measurement0_goal1 = 0;
    ack_flag = 0;
    params_to_buffer();
    buffer_size = 0;
}

Message::Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag, int _ack){
    msg_ID = _msg_ID;
    sender_ID = _sender_ID; 
    receiver_ID = _receiver_ID; 
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    measurement0_goal1 = _flag;
    ack_flag = _ack;
    params_to_buffer();
    buffer_size = sizeof(double)*12;
}

Message::Message(const void * m_buffer){
	buffer_size = sizeof(double)*12;
    memcpy(buffer, m_buffer, buffer_size);
    buffer_to_params();
}

void Message::params_to_buffer(){
	buffer[0] = msg_ID;
	buffer[1] = sender_ID;
	buffer[2] = receiver_ID;
	buffer[3] = position.x;
	buffer[4] = position.y;
	buffer[5] = position.heading;
	buffer[6] = concentration;
	buffer[7] = goal.x;
	buffer[8] = goal.y;
	buffer[9] = goal.heading;
	buffer[10] = measurement0_goal1;
	buffer[11] = ack_flag;
}

void Message::buffer_to_params(){
	msg_ID = (int)buffer[0];
	sender_ID = (int)buffer[1];
	receiver_ID = (int)buffer[2];
	position.x = buffer[3];
	position.y = buffer[4];
	position.heading = buffer[5];
	concentration = buffer[6];
	goal.x = buffer[7];
	goal.y = buffer[8];
	goal.heading = buffer[9];
	measurement0_goal1 = (int)buffer[10];
	ack_flag = (int)buffer[11];
}

void Message::print_message(){
	printf("____msg %d from %d to %d _____: \n", msg_ID, sender_ID, receiver_ID);
	printf("position: %f %f %f \n", position.x, position.y, position.heading);
	printf("concentration: %f\n", concentration);
	printf("goal: %f %f %f \n", goal.x, goal.y, goal.heading);
	printf("measurement0_goal1: %d\n", measurement0_goal1);
	printf("ack flag: %d\n", ack_flag);
	printf("______________________________\n");
}
