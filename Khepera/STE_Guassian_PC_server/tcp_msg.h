#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <unistd.h>
#include <netdb.h>

// #define PORT 3010 

typedef struct TCPMsg {
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
} TCPMsg;
