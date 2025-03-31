#include <cstring>
#include <stdio.h>
#include "Message.h"

using namespace std;

// constructor : everithing set to 0
Message::Message()
{
    sender_ID = 0;
    receiver_ID = 0;
    position.set_to_zero();
    concentration = 0;
    goal.set_to_zero();
    measurement0_goal1 = 0;
    belief_value = 0;
    params_to_buffer();
    buffer_size = 0;
}

// constructor: standard
Message::Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag)
{
    sender_ID = _sender_ID;
    receiver_ID = _receiver_ID;
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    measurement0_goal1 = _flag;
    params_to_buffer();
    buffer_size = sizeof(double) * 11;
}


// Cconstructor: max belief value
Message::Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag, double _belief_value)
{
    sender_ID = _sender_ID;
    receiver_ID = _receiver_ID;
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    measurement0_goal1 = _flag;
    belief_value = _belief_value;
    params_to_buffer();
    buffer_size = sizeof(double) * 11;
}

// constructor: buffer
Message::Message(const void *m_buffer)
{
    buffer_size = sizeof(double) * 11;
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
    buffer[9] = measurement0_goal1;
    buffer[10] = belief_value;
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
    measurement0_goal1 = buffer[9];
    belief_value = buffer[10];
}

void Message::print_message()
{
    printf("Message from [%d] to [%d]. Position (%f,%f,%f). Concentration = %f. Goal (%f,%f,%f). Flag = %d. Belief Value = %f\n",
           sender_ID, receiver_ID, position.x, position.y, position.heading, concentration, goal.x, goal.y, goal.heading, measurement0_goal1, belief_value);
}
