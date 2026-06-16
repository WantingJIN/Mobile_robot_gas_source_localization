#include <cstring>
#include <stdio.h>
#include "Message.h"
using namespace std;

Message::Message()
{
    msg_ID = 0;
    sender_ID = 0;
    receiver_ID = 0;
    position.set_to_zero();
    concentration = 0.0f;
    goal.set_to_zero();
    finish_flag = 0;
    measurement0_goal1 = 0;
    memset(belief_map, 0, sizeof(belief_map));
    buffer_size = sizeof(float) * BUFFER_LEN;
    params_to_buffer();
}

Message::Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, int _flag, const float *_belief_map)
    : Message(_msg_ID, _sender_ID, _receiver_ID, _position, 0.0f, Position(), _flag, _belief_map)
{
}

Message::Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, float _concentration, Position _goal, int _flag)
    : Message(_msg_ID, _sender_ID, _receiver_ID, _position, _concentration, _goal, _flag, nullptr)
{
}

Message::Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, float _concentration, Position _goal, int _flag, const float *_belief_map)
{
    msg_ID = _msg_ID;
    sender_ID = _sender_ID;
    receiver_ID = _receiver_ID;
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    finish_flag = _flag;
    measurement0_goal1 = 0;
    if (_belief_map)
        memcpy(belief_map, _belief_map, sizeof(belief_map));
    else
        memset(belief_map, 0, sizeof(belief_map));
    buffer_size = sizeof(float) * BUFFER_LEN;
    params_to_buffer();
}

Message::Message(const void *m_buffer)
{
    buffer_size = sizeof(float) * BUFFER_LEN;
    memcpy(buffer, m_buffer, buffer_size);
    buffer_to_params();
}

void Message::params_to_buffer()
{
    buffer[0] = static_cast<float>(msg_ID);
    buffer[1] = static_cast<float>(sender_ID);
    buffer[2] = static_cast<float>(receiver_ID);
    buffer[3] = position.x;
    buffer[4] = position.y;
    buffer[5] = position.heading;
    buffer[6] = concentration;
    buffer[7] = goal.x;
    buffer[8] = goal.y;
    buffer[9] = goal.heading;
    buffer[10] = static_cast<float>(finish_flag);
    memcpy(&buffer[MESSAGE_HEADER_LEN], belief_map, sizeof(belief_map));
}

void Message::buffer_to_params()
{
    msg_ID = static_cast<int>(buffer[0]);
    sender_ID = static_cast<int>(buffer[1]);
    receiver_ID = static_cast<int>(buffer[2]);
    position.x = buffer[3];
    position.y = buffer[4];
    position.heading = buffer[5];
    concentration = buffer[6];
    goal.x = buffer[7];
    goal.y = buffer[8];
    goal.heading = buffer[9];
    finish_flag = static_cast<int>(buffer[10]);
    memcpy(belief_map, &buffer[MESSAGE_HEADER_LEN], sizeof(belief_map));
}

void Message::print_message()
{
    printf("____msg %d from %d to %d _____: \n", msg_ID, sender_ID, receiver_ID);
    printf("position: %f %f %f; concentration: %f; goal: %f %f %f; flag: %d\n",
           position.x, position.y, position.heading, concentration, goal.x, goal.y, goal.heading, finish_flag);
    printf("Belief Map: [");
    for (int i = 0; i < MAP_SIZE; i++)
        printf("%f ", belief_map[i]);
    printf("]\n");
}
