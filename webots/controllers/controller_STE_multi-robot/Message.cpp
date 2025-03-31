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
    memset(belief_map, 0, sizeof(belief_map)); // Inicializar belief map
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
Message::Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag, double _belief_map[400])
{
    sender_ID = _sender_ID;
    receiver_ID = _receiver_ID;
    position.copy_position(_position);
    concentration = _concentration;
    goal.copy_position(_goal);
    measurement0_goal1 = _flag;
    memcpy(belief_map, _belief_map, sizeof(belief_map)); // Copiar belief map
    params_to_buffer();
    buffer_size = sizeof(double) * 410;
}

// constructor: buffer
Message::Message(const void *m_buffer)
{
    buffer_size = sizeof(double) * 410;
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
    measurement0_goal1 = buffer[9];
    memcpy(belief_map, &buffer[10], sizeof(belief_map)); // Extraer belief map del buffer
}

void Message::print_message()
{
    printf("Message from [%d] to [%d]. Position (%f,%f,%f). Concentration = %f. Goal (%f,%f,%f). Flag = %d.\n",
           sender_ID, receiver_ID, position.x, position.y, position.heading, concentration, goal.x, goal.y, goal.heading, measurement0_goal1);
    printf("Belief Map: [");
    for (int i = 0; i < 400; i++) {
        printf("%f ", belief_map[i]);
    }
    printf("]\n");
}
