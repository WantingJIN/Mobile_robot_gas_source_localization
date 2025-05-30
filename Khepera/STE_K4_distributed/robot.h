
#ifndef ROBOT_H
#define	ROBOT_H

#include "timestamp.h"

typedef struct sRobot {
  int id; // robot ID
  int state; // robot state
  double x; // distance in x
  double y; // distance in y
  double z; // distance in z
  double other_range; // radio received range
  double range; // total distance
  double bearingH; // relative angle
  double bearingV; // relative angle
  tTimestamp timestamp; // last relpos received
  double odor; // odor measurement  
} sRobot;

// Updates the state with new relative position for robot id
void update_relpos(int id, double range, double bearingH,double bearingV);

// Updates the state with new odor measurement for robot id
void update_comm(int id, int state, double odor, double range[]);

// Finds the robot structure for the given id
sRobot* find_robot(int id);

// Finds the robot index for the given id
int find_robot_index(int id);

#endif	/* ROBOT_H */

