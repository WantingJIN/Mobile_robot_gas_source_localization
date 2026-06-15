// BeliefCommManager.h
#pragma once

#include <vector>
#include <string>
#include <map>
#include <set>
#include "Message.h"

#define PORT 3011
#define MAX_PEERS 10

class BeliefCommManager {
public:
    BeliefCommManager();
    void initialize(int self_ID_, const std::vector<int>& all_IDs);
    void send_message_to_all(const Message& msg);
    bool poll_message(Message& msg_out, int& from_peer, int timeout_ms = 0);
    void wait_for_all_maps(std::map<int, Message>& received_maps);

private:
    int self_ID;
    std::vector<int> peer_IDs;
    std::vector<int> server_sockets;
    std::map<int, int> peer_sockets;

    void start_server();
    void start_clients();
    bool receive_from_any(Message& msg_out, int& from_peer, int timeout_ms);
};
