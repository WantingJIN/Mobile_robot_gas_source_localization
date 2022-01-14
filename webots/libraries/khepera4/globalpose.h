/************************************************************
 *            globalpose.h
 *
 *  2009 Jul 21 17:46
 *  Copyright 2009  Inaki Navarro
 *  Contact <inaki.navarro@upm.es> for comments & bug reports
 *************************************************************/

/** This file contains the functions to initialize the gps of webots
and get the pose (position and heading) of the robot.
*/

#ifndef GLOBALPOSE_H
#define GLOBALPOSE_H

struct sGlobalPose{
  double theta;
  double x;
  double y;
};

struct sGoblaPoseConfig{
    int noise;
    double position_noise;
    double angular_noise;
};


int globalpose_init(int control_step);
struct sGlobalPose globalpose_update(void);
void globalpose_stop(void);

#endif
