
#ifndef ROBOT_H
#define	ROBOT_H

#include "timestamp.h"

typedef struct sRobot {
  int id; // robot ID
  int state; // robot state
  float x; // distance in x
  float y; // distance in y
  float z; // distance in z
  float other_range; // radio received range
  float range; // total distance
  float bearingH; // relative angle
  float bearingV; // relative angle
  tTimestamp timestamp; // last relpos received
  float odor; // odor measurement  
} sRobot;

// Updates the state with new relative position for robot id
void update_relpos(int id, float range, float bearingH,float bearingV);

// Updates the state with new odor measurement for robot id
void update_comm(int id, int state, float odor, float range[]);

// Finds the robot structure for the given id
sRobot* find_robot(int id);

// Finds the robot index for the given id
int find_robot_index(int id);

#endif	/* ROBOT_H */

