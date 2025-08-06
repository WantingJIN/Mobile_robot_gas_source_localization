// BeliefCommManager.cpp
#include "BeliefCommManager.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

BeliefCommManager::BeliefCommManager() {}

void BeliefCommManager::initialize(int self_ID_, const std::vector<int>& all_IDs) {
    self_ID = self_ID_;
    for (int id : all_IDs) {
        if (id != self_ID) peer_IDs.push_back(id);
    }
    start_server();
    sleep(2);
    start_clients();
}

void BeliefCommManager::send_message_to_all(const Message& msg) {
    for (auto& entry : peer_sockets) {
        int sock = entry.second;
        send(sock, msg.buffer, msg.buffer_size * sizeof(float), 0);
    }
}

void BeliefCommManager::wait_for_all_maps(std::map<int, Message>& received_maps) {
    std::set<int> received_from;
   // while (received_from.size() < peer_IDs.size()) {

        Message msg;
        int from_peer;
        if (receive_from_any(msg, from_peer)) {
           if (int(msg.msg_ID) == 1 && received_from.count(from_peer) == 0) {
                received_from.insert(from_peer);
                received_maps[from_peer] = msg;
                printf("✅ Received belief map from Robot %d \n", from_peer);
           }
        }
        printf("msg.msg_ID = %d, from_peer = %d\n", int(msg.msg_ID), from_peer);
        printf("peer_IDs.size() = %zu, received_from.size() = %zu\n", peer_IDs.size(), received_from.size());
    //}
}

void BeliefCommManager::start_server() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) { perror("socket"); exit(EXIT_FAILURE); }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind"); exit(EXIT_FAILURE);
    }
    listen(server_fd, MAX_PEERS);

    for (int id : peer_IDs) {
        if (id > self_ID) continue;

        int addrlen = sizeof(address);
        int sock = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        if (sock < 0) { perror("accept"); continue; }

        peer_sockets[id] = sock;
        server_sockets.push_back(sock);
        std::cout << "✅ Accepted connection from Robot " << id << std::endl;
    }
}

void BeliefCommManager::start_clients() {
    for (int id : peer_IDs) {
        if (id < self_ID) continue;

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { perror("socket"); continue; }

        struct sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        std::string ip = "192.168.1." + std::to_string(id);
        inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

        int retry = 0;
        while (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            if (++retry > 10) {
                std::cerr << "❌ Could not connect to " << ip << std::endl;
                break;
            }
            sleep(1);
        }
        peer_sockets[id] = sock;
        std::cout << "🔗 Connected to Robot " << id << std::endl;
    }
}

bool BeliefCommManager::receive_from_any(Message& msg_out, int& from_peer) {
    fd_set readfds;
    FD_ZERO(&readfds);
    int max_sd = -1;

    for (auto& entry : peer_sockets) {
        int sock = entry.second;
        FD_SET(sock, &readfds);
        if (sock > max_sd) max_sd = sock;
    }

    timeval timeout{1, 0};
    int activity = select(max_sd + 1, &readfds, nullptr, nullptr, &timeout);
    if (activity <= 0) return false;

    for (auto& entry : peer_sockets) {
        int peer_id = entry.first;
        int sock = entry.second;

        if (FD_ISSET(sock, &readfds)) {
            // float buf[MAP_SIZE + 7];
            // int total = 0;
            // int expected = sizeof(buf);
            // while (total < expected) {
            //     int bytes = recv(sock, ((char*)buf) + total, expected - total, 0);
            //     if (bytes <= 0) {
            //         perror("recv error");
            //         break;
            //     }
            //     total += bytes;
            // }
            // if (total < expected) return false;
            float buf[BUFFER_LEN];
            int success_recv_map = -1;
            success_recv_map = read(sock, buf, sizeof(double)*BUFFER_LEN); 

            if (success_recv_map){
                 printf("Received message from Robot %d\n", peer_id);

                msg_out = Message(buf);
                msg_out.buffer_to_params();
                printf("Received belief map is: \n");
                for (int i = 0; i < MAP_SIZE; ++i) {
                    std::cout << msg_out.belief_map[i] << " ";
                }
                printf("\n");            
                from_peer = peer_id;
                return true;
            } else {
                perror("recv error");
                return false;

            }
        }
    }
    return false;
}
