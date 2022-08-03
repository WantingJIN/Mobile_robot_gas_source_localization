#include <stdio.h>
#include <stdlib.h>
#include "udp_msg.h"

sUdpMsg udp_msg;

void fail_with_error(char* text) {
  printf("ERROR : %s \n\t\t%s", strerror(errno), text);
  exit(EXIT_FAILURE);
}

void udp_msg_init(int port, int id) {
  sRobotData tm;
  static int broadcast = 1;
  udp_msg.myID = id;
  udp_msg.len = sizeof (tm);

  //initialize receiver
  memset(&udp_msg.rcv.address, 0, sizeof (udp_msg.rcv.address));
  udp_msg.rcv.address.sin_family = AF_INET;
  udp_msg.rcv.address.sin_port = htons(port);
  udp_msg.rcv.address.sin_addr.s_addr = htonl(INADDR_ANY);

  //create and bind to socket
  udp_msg.rcv.sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_msg.rcv.sock < 0)
    fail_with_error("Unable to create receiver socket");
  fcntl(udp_msg.rcv.sock, F_SETFL, O_NONBLOCK);
  if (bind(udp_msg.rcv.sock, (struct sockaddr*) &udp_msg.rcv.address, sizeof (udp_msg.rcv.address)))
    fail_with_error("Bind failed");

  //initialize sender
  memset(&udp_msg.snd.address, 0, sizeof (udp_msg.snd.address));
  udp_msg.snd.address.sin_family = AF_INET;
  udp_msg.snd.address.sin_port = htons(port);
  udp_msg.snd.address.sin_addr.s_addr = inet_addr("192.168.141.255");
  udp_msg.snd.sock = socket(AF_INET, SOCK_DGRAM, 0);

  if (setsockopt(udp_msg.snd.sock, SOL_SOCKET, SO_BROADCAST, (void*) &broadcast, sizeof (broadcast)) < 0)
    fail_with_error("Unable to set broadcast option");
}

void udp_msg_send(sRobotData *tmp) {
  int len;

  len = sendto(udp_msg.snd.sock, (char *) (tmp), udp_msg.len, 0, (struct sockaddr*) &udp_msg.snd.address, sizeof (udp_msg.snd.address));
  if (len < 0)
    fail_with_error("During broadcast\n");
}

int udp_msg_receive(sRobotData *tmp_msg) {
  char pt[4]; // This will hold robot id
  int result;
  int id;
  struct sockaddr_in sender;
  socklen_t sender_len;
  sender_len = sizeof (sender);
  memset((char *) (tmp_msg), '\0', udp_msg.len);
  result = recvfrom(udp_msg.rcv.sock, (char *) (tmp_msg), udp_msg.len, 0, (struct sockaddr*) &sender, &sender_len);

  // If reception ok
  if (result > 0) {
    // Get ID from address
    strncpy(pt, inet_ntoa(sender.sin_addr) + 12, 3);
    pt[3] = '\n';
    id = atoi(pt);
    tmp_msg->id = id;
  }

  return result;
}

void udp_msg_print(sRobotData *tmp_msg) {
  printf("MESSAGE: sender_id: %d, state: %d, odor: %f, rcv_id: %d, \nX: %f, Y: %f, Z: %f, Angle: %f, \nx: %f, y: %f, z: %f, IDs[0] %d IDs[1] %d\n", 
    tmp_msg->id, tmp_msg->state, tmp_msg->odor, tmp_msg->rcv_id, tmp_msg->X, tmp_msg->Y, tmp_msg->Z, tmp_msg->Angle, tmp_msg->x, tmp_msg->y, tmp_msg->z, tmp_msg->IDs[0], tmp_msg->IDs[1]);
}

void udp_msg_clean_up() {
  close(udp_msg.rcv.sock);
  close(udp_msg.snd.sock);
}
