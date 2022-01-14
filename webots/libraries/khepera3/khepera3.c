/* KHEPERA3 INITIALIZATION */

#include "khepera3.h"


struct sKhepera3 khepera3;


/*** Module Initialization ***/


void khepera3_init() {
	wb_robot_init();


	khepera3_infrared_init();
	khepera3_ultrasound_init();
	khepera3_drive_init();

	//Not yet implemented;
	//khepera3_motor_init();
	//khepera3_battery_init();


}

