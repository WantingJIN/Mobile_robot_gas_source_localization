/* KHEPERA3 INFRARED */

#include "khepera4_infrared.h"
#include "khepera4.h"

//! Khepera4 Infrared Sensor names
const char *infrared_sensors_names[cKhepera4SensorsInfrared_Count] = {
  // turret sensors
  "rear left infrared sensor",
  "left infrared sensor",
  "front left infrared sensor",
  "front infrared sensor",
  "front right infrared sensor",
  "right infrared sensor",
  "rear right infrared sensor",
  "rear infrared sensor",
  // ground sensors
  "ground left infrared sensor",
  "ground front left infrared sensor",
  "ground front right infrared sensor",
  "ground right infrared sensor"
};

//! Khepera4 Light Sensor names
const char *light_sensors_names[12] = {
  // turret sensors
  "rear left light sensor",
  "left light sensor",
  "front left light sensor",
  "front light sensor",
  "front right light sensor",
  "right light sensor",
  "rear right light sensor",
  "rear light sensor",
  // ground sensors
  "ground left light sensor",
  "ground front left light sensor",
  "ground front right light sensor",
  "ground right light sensor"
};

WbDeviceTag infrared[cKhepera4SensorsInfrared_Count];		//!< Remember DeviceTag equals to const char

WbDeviceTag lightsensor[cKhepera4SensorsInfrared_Count];		//!< Remember DeviceTag equals to const char

void khepera4_infrared_init() {
	int i;
	khepera4.infrared_proximity.timestamp = wb_robot_get_time();
	// Infrared assigment
	for (i = cKhepera4SensorsInfrared_Begin; i < cKhepera4SensorsInfrared_Count; i++) {
		infrared[i] = wb_robot_get_device(infrared_sensors_names[i]);
		wb_distance_sensor_enable(infrared[i], CONTROL_STEP);
	}
	for (i = cKhepera4SensorsInfrared_Begin; i < cKhepera4SensorsInfrared_Count; i++) {
		lightsensor[i] = wb_robot_get_device(light_sensors_names[i]);
		wb_light_sensor_enable(lightsensor[i], CONTROL_STEP);
	}
}

void khepera4_infrared_ambient(){
	int i;
	for (i = cKhepera4SensorsInfrared_Begin; i < cKhepera4SensorsInfrared_Count; i++) {
              	khepera4.infrared_ambient.sensor[i] = (int) wb_light_sensor_get_value(lightsensor[i]);
	}
	khepera4.infrared_ambient.timestamp = wb_robot_get_time();
}


void khepera4_infrared_proximity() {
	int i;
	for (i = cKhepera4SensorsInfrared_Begin; i < cKhepera4SensorsInfrared_Count; i++) {
              	khepera4.infrared_proximity.sensor[i] = (int)wb_distance_sensor_get_value(infrared[i]); 
	}
	khepera4.infrared_proximity.timestamp = wb_robot_get_time();
}



