/* KHEPERA3 MOTOR HEADER */

#ifndef KHEPERA3_MOTOR_H
#define KHEPERA3_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

//! One motor
struct sKhepera3Motor {
	int current_speed;					//!< Current speed (updated by khepera3_motor_get_current_speed)
	int current_position;					//!< Current position (updated by khepera3_motor_get_current_position)
	//int current_torque;					//!< Current torque (updated by khepera3_motor_get_current_torque)
};

#ifdef __cplusplus
}
#endif

#endif
