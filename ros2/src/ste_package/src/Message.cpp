#include <cstring>
#include <stdio.h>
#include "ste_package/Message.h"

using namespace std;
extern SourceBounds source_bounds;

// constructor : everithing set to 0
Message::Message()
{
    sender_ID = 0;
    receiver_ID = 0;
    position.set_to_zero();
    concentration = 0;
    goal.set_to_zero();
    measurement0_goal1 = 0;
    //initial belief map to 0
    belief_map = new double[source_bounds.map_size]();
    buffer = new double[source_bounds.map_size + 10](); // buffer size to hold all parameters + belief map
    memset(belief_map, 0, sizeof(belief_map)); 
    params_to_buffer();
    buffer_size = 0;
}

// constructor: standard
Message::Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag)
{
    printf("Creating standard message from %d to %d\n", _sender_ID, _receiver_ID);
    sender_ID = _sender_ID;
    receiver_ID = _receiver_ID;
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    finish_flag = _flag;
    belief_map = new double[source_bounds.map_size]();
    buffer = new double[source_bounds.map_size + 10](); // buffer size to hold all parameters + belief map
    params_to_buffer();
    buffer_size = sizeof(double) * 11;
}


// Cconstructor: max belief value
Message::Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag, double *_belief_map)
{
    sender_ID = _sender_ID;
    receiver_ID = _receiver_ID;
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    finish_flag = _flag;
    belief_map = new double[source_bounds.map_size]();
    buffer = new double[source_bounds.map_size + 10](); // buffer size to hold all parameters + belief map
    belief_map = _belief_map;
    params_to_buffer();
    buffer_size = sizeof(double) * 410;
}

// constructor: buffer
Message::Message(const void *m_buffer)
{
    buffer_size = sizeof(double) * 410;
    buffer = new double[source_bounds.map_size + 10](); // buffer size to hold all parameters + belief map
    memcpy(buffer, m_buffer, buffer_size);
    buffer_to_params();
}

void Message::params_to_buffer()
{
    buffer[0] = sender_ID;
    buffer[1] = receiver_ID;
    buffer[2] = position.x;
    buffer[3] = position.y;
    buffer[4] = position.heading;
    buffer[5] = concentration;
    buffer[6] = goal.x;
    buffer[7] = goal.y;
    buffer[8] = goal.heading;
    buffer[9] = finish_flag;
    memcpy(&buffer[10], belief_map, sizeof(belief_map)); // Agregar belief map al buffer
}

void Message::buffer_to_params()
{
    sender_ID = buffer[0];
    receiver_ID = buffer[1];
    position.x = buffer[2];
    position.y = buffer[3];
    position.heading = buffer[4];
    concentration = buffer[5];
    goal.x = buffer[6];
    goal.y = buffer[7];
    goal.heading = buffer[8];
    finish_flag = buffer[9];
    memcpy(belief_map, &buffer[10], sizeof(belief_map)); // Extraer belief map del buffer
}

void Message::print_message()
{
    printf("Message from [%d] to [%d]. Position (%f,%f,%f). Concentration = %f. Goal (%f,%f,%f). Flag = %d.\n",
           sender_ID, receiver_ID, position.x, position.y, position.heading, concentration, goal.x, goal.y, goal.heading, finish_flag);
    printf("Belief Map: [");
    for (int i = 0; i < source_bounds.map_size; i++) {
        printf("%f ", belief_map[i]);
    }
    printf("]\n");
}
