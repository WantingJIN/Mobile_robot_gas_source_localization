#ifndef MESSAGE_H
#define MESSAGE_H

#include "Position.h"

constexpr int MESSAGE_HEADER_LEN = 11;
constexpr int BUFFER_LEN = MAP_SIZE + MESSAGE_HEADER_LEN;

class Message {
public:
    int msg_ID;       // message type/round id
    int sender_ID;
    int receiver_ID;  // 0 means broadcast/supervisor, otherwise a robot ID
    Position position;
    float concentration;
    Position goal;
    int finish_flag;  // -1 means finished notification
    int measurement0_goal1;
    float belief_map[MAP_SIZE];
    float buffer[BUFFER_LEN];
    int buffer_size;

    Message();
    Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, int _flag, const float *_belief_map);
    Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, float _concentration, Position _goal, int _flag);
    Message(int _msg_ID, int _sender_ID, int _receiver_ID, Position _position, float _concentration, Position _goal, int _flag, const float *_belief_map);
    Message(const void *m_buffer);

    void params_to_buffer();
    void buffer_to_params();
    void print_message();
};

#endif
