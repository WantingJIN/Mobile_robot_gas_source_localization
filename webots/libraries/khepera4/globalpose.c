/************************************************************
 *            gps.c
 *
 *  2009 Jul 21 17:43
 *  Copyright 2009  Inaki Navarro
 *  Contact <inaki.navarro@upm.es> for comments & bug reports
 *************************************************************/

/** This file contains the functions to initialize the gps of webots
and get the pose (position and heading) of the robot.
*/

#include <math.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include "globalpose.h"

#include <gsl/gsl_rng.h> // GSL random number generators
#include <gsl/gsl_randist.h> // GSL random distributions
#include <time.h>
#define PI 3.14159265358979



WbDeviceTag gps;
WbDeviceTag compass;

struct sGoblaPoseConfig gpc;

gsl_rng *rng_ptr; // pointer to random number generator (rng)

/** It initialises the gps*/
int globalpose_init(int control_step){

  gpc.noise = 0;
  gpc.position_noise = 0.1;
  gpc.angular_noise = 0.1;

  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  wb_gps_enable(gps, control_step);
  wb_compass_enable(compass, control_step);

  rng_ptr = gsl_rng_alloc (gsl_rng_taus); // allocate the rng
  gsl_rng_set (rng_ptr, (int) time(NULL)); // seed the rng with  time, ADD MORE to differentiate robots!!

  return 1;
}

/** It returns the pose (position and heading) of the robot.
*/
struct sGlobalPose globalpose_update(void){
  struct sGlobalPose gp;
  const double * position;
  const double * north;
  double x_error;
  double y_error;
  double range;
  double angle;
  double angular_error;

  if (gpc.noise){
    range = gsl_ran_gaussian (rng_ptr, gpc.position_noise);
    angle = gsl_ran_flat (rng_ptr, -PI, PI);
    x_error = range*cos(angle);
    y_error = range*sin(angle);
    angular_error = gsl_ran_gaussian (rng_ptr, gpc.angular_noise);
  } else {
    x_error = 0;
    y_error = 0;
    angular_error = 0;
  }

  position = wb_gps_get_values(gps);
  gp.x = position[0] + x_error;
  gp.y = position[2] + y_error;

  north = wb_compass_get_values(compass);
  gp.theta =  atan2(north[0], north[2]) +  angular_error;
  while (gp.theta > PI) gp.theta -= 2.0*PI;
  while (gp.theta < -PI) gp.theta += 2.0*PI;

  return gp;
}


void globalpose_stop(void){
  wb_compass_disable(compass); 
  wb_gps_disable(gps); 
}
