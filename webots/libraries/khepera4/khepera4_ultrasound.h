/* KHEPERA4 ULTRASOUND HEADER */

#ifndef KHEPERA4_ULTRASOUND_H
#define KHEPERA4_ULTRASOUND_H

#define AMPLITUDE 200


//! Khepera4 Ultrasound Sensors
enum eKhepera4SensorsUltrasound {
	cKhepera4SensorsUltrasound_Left = 0,
	cKhepera4SensorsUltrasound_FrontLeft = 1,
	cKhepera4SensorsUltrasound_Front = 2,
	cKhepera4SensorsUltrasound_FrontRight = 3,
	cKhepera4SensorsUltrasound_Right = 4,

	cKhepera4SensorsUltrasound_Begin = 0,
	cKhepera4SensorsUltrasound_End = 4,
	cKhepera4SensorsUltrasound_Count = 5
};


//! Ultrasound sensor bit masks
enum eKhepera4SensorsUltrasoundBit {
	cKhepera4SensorsUltrasoundBit_Left = 1,
	cKhepera4SensorsUltrasoundBit_FrontLeft = 2,
	cKhepera4SensorsUltrasoundBit_Front = 4,
	cKhepera4SensorsUltrasoundBit_FrontRight = 8,
	cKhepera4SensorsUltrasoundBit_Right = 16,

	cKhepera4SensorsUltrasoundBit_None = 0,
	cKhepera4SensorsUltrasoundBit_All = 31
};

//! One measurement of one ultrasound sensor
struct sKhepera4SensorsUltrasoundSensor {
	int echos_count;		//!< Number of received echos, not currently implemented in webots
	int distance[10];		//!< Received distance in centimeters
	int amplitude[10];		//!< Received amplitude, not currently implemented in webots
	int timestamp[10];		//!< Timestamp at which the echo was received
};

struct sKhepera4SensorsUltrasound {
    int sensor[cKhepera4SensorsUltrasound_Count]; 	//!< The ultrasound sensors
    float distance_per_increment; 					//!< Conversion factor to convert from distance values to meters
    int timestamp;
};

//! Initializes webots ultrasound sensors
void khepera4_ultrasound_init();
//! Reads the last values of an ultrasound sensor and fills them into the khepera4 structure. The return value indicates success (-1) or failure (0).
int khepera4_ultrasound();

// Should not be used in order to preserve compatibility with real robots!!
//! Assigns webots ultrasound sensors previously initialized to the khepera4 ultrasound structure
int khepera4_one_ultrasound(enum eKhepera4SensorsUltrasound sensor);
//! (Privat) Initializes sKhepera4SensorsUltrasoundSensor structure for each ultrasound sensor
void ultrasound_structure_init();
//! (Privat) Gets sensor number by name
enum eKhepera4SensorsUltrasound khepera4_ultrasound_getsensorbyname(const char *name, enum eKhepera4SensorsUltrasound defaultvalue);
//! (Privat) Returns a sensor bitmask by sensor ID.
enum eKhepera4SensorsUltrasoundBit khepera4_ultrasound_getsensorbitbysensor(enum eKhepera4SensorsUltrasound sensor);

/*  Not present anymore on real robot interface !!
 * 
// Just for compatibility with real robots!!
//! Enables or disables ultrasound sensors. The return value indicates success (-1) or failure (0). Transfer on I2C bus: 8 bytes.
int khepera4_ultrasound_enable(enum eKhepera4SensorsUltrasoundBit bitmask);
//! Sets the maximum number of echos received by the ultrasound sensors. The return value indicates success (-1) or failure (0). Transfer on I2C bus: 8 bytes.
int khepera4_ultrasound_set_max_echo_number(int max_echo_number);
*/

#endif
