/* KHEPERA4 INITIALIZATION */

#include "khepera4.h"

// global instance of Khepera4 robot.
struct sKhepera4 khepera4;

/*** Module Initialization ***/

int khepera4_running(){
	return (wb_robot_step(CONTROL_STEP) != -1);
}

void khepera4_init() {
	wb_robot_init();

	khepera4_infrared_init();
	khepera4_ultrasound_init();
	khepera4_drive_init();
#ifdef RELPOSBOARD
	relativepositionboard_init();
#endif
	//Not yet implemented;
	//khepera4_motor_init();
	//khepera4_battery_init();
}

void khepera4_cleanup(){
	wb_robot_cleanup();
}