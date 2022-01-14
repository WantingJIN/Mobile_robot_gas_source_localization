/* KHEPERA4 INITIALIZATION HEADER */

#ifndef KHEPERA4_H
#define KHEPERA4_H

#include <webots/robot.h>
#include <webots/motor.h>
//#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/light_sensor.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "khepera4_infrared.h"
#include "khepera4_ultrasound.h"
#include "khepera4_drive.h"
#include "khepera4_motor.h"
#include "relativepositionboard.h"
#include "odometry_track.h"

#include <string.h>

#define CONTROL_STEP 32

//enable Relative position board
#define RELPOSBOARD

//! Khepera4 Structure
struct sKhepera4 {
	struct sKhepera4Motor motor_left;
	struct sKhepera4Motor motor_right;
	struct sKhepera4SensorsUltrasound ultrasound;
   	struct sKhepera4SensorsInfrared infrared_ambient;
	struct sKhepera4SensorsInfrared infrared_proximity;
	struct sOdometryTrack odometry_track;
#ifdef RELPOSBOARD
	struct sRelativePositionBoard relative_position_board;
#endif
};

extern struct sKhepera4 khepera4;

//! Initializes robot by initialization functions with a default CONTROL_STEP of 32 ms
void khepera4_init();
//! Running condition for Khepera 4. Can be used directly as while(khepera4_running()).
int khepera4_running();
//! Khepera 4 cleanup function. Call to wb_robot_cleanup().
void khepera4_cleanup();


#ifdef __cplusplus
}
#endif

#endif //KHEPERA4_H
