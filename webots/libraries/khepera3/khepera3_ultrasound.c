/* KHEPERA3 ULTRASOUND */

#include "khepera3_ultrasound.h"
#include "khepera3.h"

#include <stdio.h>

const char *cKhepera3SensorsUltrasound[5];


char ultrasound_name[5] = "us0";
WbDeviceTag ultrasound[5]; 		//!< Remember DeviceTag equals to const char
const char *cKhepera3SensorsUltrasound[5] = {"left", "frontleft", "front", "frontright", "right"};

void khepera3_ultrasound_init() {
	int i;
	// Ultrasound assigment
	for (i = cKhepera3SensorsUltrasound_Begin; i < cKhepera3SensorsUltrasound_Count; i++) {
		ultrasound[i] = wb_robot_get_device(ultrasound_name);
		wb_distance_sensor_enable(ultrasound[i], CONTROL_STEP);
                ultrasound_name[2]++;

	}

	khepera3.ultrasound.distance_per_increment = 0.01;

	ultrasound_structure_init();

}

void ultrasound_structure_init() {
	int i, j;
	
	for (i = cKhepera3SensorsUltrasound_Begin; i < cKhepera3SensorsUltrasound_End; i++) {
	
		for (j = 0; j < 10; j++) {

		khepera3.ultrasound.sensor[i].distance[j] = 0;
		khepera3.ultrasound.sensor[i].amplitude[j] = 0;
		khepera3.ultrasound.sensor[i].distance[j] = 0;
		khepera3.ultrasound.sensor[i].timestamp[j] = 0;

		}
	
	khepera3.ultrasound.sensor[i].amplitude[0] = AMPLITUDE;

	}
		
}

int khepera3_ultrasound(enum eKhepera3SensorsUltrasound sensor){
  khepera3.ultrasound.sensor[sensor].distance[0] = (int)(wb_distance_sensor_get_value(ultrasound[sensor]));
  khepera3.ultrasound.sensor[sensor].timestamp[0] = wb_robot_get_time();
  return 0;
}

enum eKhepera3SensorsUltrasound khepera3_ultrasound_getsensorbyname(const char *name, enum eKhepera3SensorsUltrasound defaultvalue) {

	int i;

	if (name == 0) {
		return defaultvalue;
	}
	
	// By name
	for (i = 0; i < 5; i++) {
		if (strcmp(name, cKhepera3SensorsUltrasound[i]) == 0) {
			return i;
		}
	}

	// Nothing
	return defaultvalue;
}

enum eKhepera3SensorsUltrasoundBit khepera3_ultrasound_getsensorbitbysensor(enum eKhepera3SensorsUltrasound sensor) {
	return (1 << sensor);
}

int khepera3_ultrasound_enable(enum eKhepera3SensorsUltrasoundBit bitmask) {
    return 0;
}

int khepera3_ultrasound_set_max_echo_number(int max_echo_number) {
    return 0;
}
