/* KHEPERA3 DRIVE TRANSLATION */

#include "khepera4_drive.h"
#include "khepera4.h"


  WbDeviceTag left_motor, right_motor, left_encoder, right_encoder;
void khepera4_drive_init() {

/*	wb_differential_wheels_enable_encoders(CONTROL_STEP);*/ // Khep 4 model is not differential drive!

	// get the motors and set target position to infinity (speed control)
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

	// gets the encoders & enables them
	left_encoder = wb_robot_get_device("left wheel sensor");
	right_encoder = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(left_encoder,CONTROL_STEP);
	wb_position_sensor_enable(right_encoder, CONTROL_STEP);
}

void khepera4_drive_stop() {

	/*wb_differential_wheels_set_speed(0, 0);*/		// Khep 4 model is not differential drive!
	wb_motor_set_velocity(left_motor, 0);
	wb_motor_set_velocity(right_motor, 0);

}

void khepera4_drive_start() {}

void khepera4_drive_set_speed(int speed_left, int speed_right) {
	/*wb_differential_wheels_set_speed(speed_left, speed_right);*/ // Khep 4 model is not differential drive!
	
	// verify that max speeds are never exceeded. does NOT correct keeping differential direction.
	if(speed_left > wb_motor_get_max_velocity(right_motor)){
		speed_left = wb_motor_get_max_velocity(left_motor);
	}
	if(speed_right > wb_motor_get_max_velocity(right_motor)){
		speed_right = wb_motor_get_max_velocity(right_motor);
	}
	
	wb_motor_set_velocity(left_motor, speed_left);
	wb_motor_set_velocity(right_motor, speed_right);
	khepera4.motor_left.current_speed = speed_left;
	khepera4.motor_right.current_speed = speed_right;

}

void khepera4_drive_set_speed_differential(float speed, float forward_coefficient, float differential_coefficient) {
	float speed_left;
	float speed_right;

	speed_left = speed * (forward_coefficient + differential_coefficient);
	speed_right = speed * (forward_coefficient - differential_coefficient);
	khepera4_drive_set_speed((int)(speed_left + 0.5), (int)(speed_right + 0.5));

}

void khepera4_drive_set_speed_differential_bounded(float speed, float forward_coefficient, float forward_coefficient_max, float differential_coefficient, float differential_coefficient_max) {

	if (forward_coefficient > forward_coefficient_max) {
		forward_coefficient = forward_coefficient_max;
	}
	if (forward_coefficient < -forward_coefficient_max) {
		forward_coefficient = -forward_coefficient_max;
	}

	if (differential_coefficient > differential_coefficient_max) {
		differential_coefficient = differential_coefficient_max;
	}
	if (differential_coefficient < -differential_coefficient_max) {
		differential_coefficient = -differential_coefficient_max;
	}

	khepera4_drive_set_speed_differential(speed, forward_coefficient, differential_coefficient);

}

void khepera4_drive_set_current_position(int position_left, int position_right) {
// Currently not implemented as encoders can't be set to another position... 

/*
	wb_differential_wheels_set_encoders(position_left, position_right);
	khepera4.motor_left.current_position = position_left;
	khepera4.motor_right.current_position = position_right;
*/
}

void khepera4_drive_get_current_speed() {}

void khepera4_drive_get_current_position() {
/*
	khepera4.motor_left.current_position = wb_differential_wheels_get_left_encoder();
	khepera4.motor_right.current_position = wb_differential_wheels_get_right_encoder();
*/
	khepera4.motor_left.current_position = wb_position_sensor_get_value(left_encoder);
	khepera4.motor_right.current_position = wb_position_sensor_get_value(right_encoder);

}
