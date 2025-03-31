#ifndef MESSAGE_H
#define MESSAGE_H

#include "Position.h"

class Message
{
public:
    int sender_ID;
    int receiver_ID; // if == 0 -->> msg for supervisor, otherwise it's for robots
    Position position;
    double concentration;
    Position goal;
    int measurement0_goal1; // binary flag that shows what values we are sharing
    double belief_value; // new atribute to store the max belief value
    double buffer[11]; // augmented buffer size for the max belief value
    int buffer_size;

    // constructor : everithing set to 0
    Message();
    
    // constructor : standard
    Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag);
    
    // constructor : max belief value
    Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag, double _belief_value);
    Message(const void *m_buffer);

    void params_to_buffer();
    void buffer_to_params();
    void print_message();
};

#endif
