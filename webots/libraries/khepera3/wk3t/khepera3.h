/* KHEPERA3 INITIALIZATION HEADER */

#ifndef KHEPERA3_H
#define KHEPERA3_H

#include "khepera3_infrared.h"
#include "khepera3_ultrasound.h"
#include "khepera3_drive.h"
#include "khepera3_motor.h"
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <string.h>

#define CONTROL_STEP 32

//! Khepera3 Structure
struct sKhepera3 {
	struct sKhepera3Motor motor_left;
	struct sKhepera3Motor motor_right;
	struct sKhepera3SensorsUltrasound ultrasound;
	struct sKhepera3SensorsInfrared infrared_proximity;
};

struct sKhepera3 khepera3;

//! Initializes robot by initialization functions with a default CONTROL_STEP of 32 ms
void khepera3_init();

#endif
