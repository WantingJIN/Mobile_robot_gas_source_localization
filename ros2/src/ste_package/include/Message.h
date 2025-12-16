#ifndef MESSAGE_H
#define MESSAGE_H

#include "Position.h"
#include <vector>

using namespace std;
using std::vector;
class Message
{
public:
    int sender_ID;
    int receiver_ID; // if == 0 -->> msg for supervisor, otherwise it's for robots
    Position position;
    double concentration;
    Position goal;
    int finish_flag; // binary flag for finished state
    double * belief_map; // new atribute to store the belief map value
    double * buffer; // augmented buffer size for the belief map value
    int buffer_size;
    int measurement0_goal1; // flag to indicate if the message contains a measurement (0) or a goal (1)

    // constructor : everithing set to 0
    Message();
    
    // constructor : standard
    Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag);
    
    // constructor : with belief map value
    Message(int _sender_ID, int _receiver_ID, Position _position, double _concentration, Position _goal, int _flag, double *_belief_map);
    Message(const void *m_buffer);

    void params_to_buffer();
    void buffer_to_params();
    void print_message();
};

#endif
