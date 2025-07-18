#ifndef UDP_MSG
#define UDP_MSG

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <string.h>

// Parameters
#define N 3

//#include "config.h"

typedef struct sUdpMsg {
  int len;
  int myID;

  struct {
    int sock;
    struct sockaddr_in address;
  } rcv;

  struct {
    int sock;
    struct sockaddr_in address;
  } snd;
} sUdpMsg;

typedef struct robot_data {
  int id;
  int state;
  float odor;
  //int rcv_id;
  float X;  
  float Y;  
  float Z;  
  float Angle;
  float x;
  float y;
  float z;
  int IDs[N];

  //int placeholder;
  float range[N][N];
  float bearingH[N][N];
  float bearingV[N][N];

  float windAngle[N];
} sRobotData;

extern sUdpMsg udp_msg;

// Print error message and quits
void fail_with_error(char* text);

// Initializes this module.
void udp_msg_init(int p, int id);

//Check if there is a message, and receive it
int udp_msg_receive(sRobotData *tmp_msg);

//Print the details of the given message
void udp_msg_print(sRobotData *tmp_msg);

// Send message
void udp_msg_send(sRobotData *tmp_msg);

// Clean up module
void udp_msg_clean_up();

#endif
