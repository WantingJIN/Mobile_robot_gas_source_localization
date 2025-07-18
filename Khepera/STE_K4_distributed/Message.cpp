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
    finish_flag = 0; // not finished
    memset(belief_map, 0, sizeof(belief_map)); // Inicializar belief map
    params_to_buffer();
    buffer_size = sizeof(float)*(MAP_SIZE + 7); 

}

Message::Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, int _flag, float *_belief_map){
    msg_ID = _msg_ID;
    sender_ID = _sender_ID; 
    receiver_ID = _receiver_ID; 
    position.copy_position(_position);
    finish_flag = _flag;
    memcpy(belief_map, _belief_map, sizeof(belief_map));
    params_to_buffer();
    buffer_size = sizeof(float)*(MAP_SIZE + 7); // belief map + 4 additional values for msg_ID, sender_ID, receiver_ID, position
}

Message::Message(const void * m_buffer){
    buffer_size = sizeof(float)*(MAP_SIZE + 7);
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
	buffer[6] = finish_flag; // finish flag
	memcpy(&buffer[7], belief_map, sizeof(belief_map));
}

void Message::buffer_to_params(){
	msg_ID = (int)buffer[0];
	sender_ID = (int)buffer[1];
	receiver_ID = (int)buffer[2];
	position.x = buffer[3];
	position.y = buffer[4];
	position.heading = buffer[5];
	finish_flag = (int)buffer[6]; // finish flag
	// copy belief map from buffer
	memcpy(belief_map, &buffer[7], sizeof(belief_map));
}

void Message::print_message(){
	printf("____msg %d from %d to %d _____: \n", msg_ID, sender_ID, receiver_ID);
	printf("position: %f %f %f \n", position.x, position.y, position.heading);
	printf("Belief Map: [");
    	for (int i = 0; i < MAP_SIZE; i++) {
		printf("%f ", belief_map[i]);
    	}
    	printf("]\n");
}
