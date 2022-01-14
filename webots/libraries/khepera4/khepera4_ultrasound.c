/* KHEPERA3 ULTRASOUND */

#include "khepera4_ultrasound.h"
#include "khepera4.h"

#include <stdio.h>

//! Khepera4 Ultrasound Sensor names
const char *cKhepera4SensorsUltrasound[cKhepera4SensorsUltrasound_Count] = {
  "left ultrasonic sensor",
  "front left ultrasonic sensor",
  "front ultrasonic sensor",
  "front right ultrasonic sensor",
  "right ultrasonic sensor"
};

WbDeviceTag ultrasound[cKhepera4SensorsUltrasound_Count]; 		//!< Remember DeviceTag equals to const char


void khepera4_ultrasound_init() {
	int i;
	// Ultrasound assigment
	for (i = cKhepera4SensorsUltrasound_Begin; i < cKhepera4SensorsUltrasound_Count; i++) {
		ultrasound[i] = wb_robot_get_device(cKhepera4SensorsUltrasound[i]);
		wb_distance_sensor_enable(ultrasound[i], CONTROL_STEP);

	}

	khepera4.ultrasound.distance_per_increment = 0.01;

	ultrasound_structure_init();
}

void ultrasound_structure_init() {
	int i;
	
	for (i = cKhepera4SensorsUltrasound_Begin; i < cKhepera4SensorsUltrasound_End; i++) {
	
		khepera4.ultrasound.sensor[i] = 0;
		khepera4.ultrasound.timestamp = 0;

	}
}

int khepera4_ultrasound(){
	int i;
	for(i=cKhepera4SensorsUltrasound_Begin;i<cKhepera4SensorsUltrasound_Count;i++){
		khepera4_one_ultrasound(i);
	}
	return -1;
}
int khepera4_one_ultrasound(enum eKhepera4SensorsUltrasound sensor){
  khepera4.ultrasound.sensor[sensor] = (int)(wb_distance_sensor_get_value(ultrasound[sensor])*100);
  khepera4.ultrasound.timestamp      = wb_robot_get_time();
  return 0;
}

enum eKhepera4SensorsUltrasound khepera4_ultrasound_getsensorbyname(const char *name, enum eKhepera4SensorsUltrasound defaultvalue) {

	int i;

	if (name == 0) {
		return defaultvalue;
	}
	
	// By name
	for (i = 0; i < 5; i++) {
		if (strcmp(name, cKhepera4SensorsUltrasound[i]) == 0) {
			return i;
		}
	}

	// Nothing
	return defaultvalue;
}

enum eKhepera4SensorsUltrasoundBit khepera4_ultrasound_getsensorbitbysensor(enum eKhepera4SensorsUltrasound sensor) {
	return (1 << sensor);
}

int khepera4_ultrasound_enable(enum eKhepera4SensorsUltrasoundBit bitmask) {
    return 0;
}

int khepera4_ultrasound_set_max_echo_number(int max_echo_number) {
    return 0;
}
