/* KHEPERA4 INFRARED HEADER */

#ifndef KHEPERA4_INFRARED_H
#define KHEPERA4_INFRARED_H



//! Khepera4 Infrared Sensors
enum eKhepera4SensorsInfrared {
    cKhepera4SensorsInfrared_BackLeft = 0,
    cKhepera4SensorsInfrared_Left = 1,
    cKhepera4SensorsInfrared_FrontLeft = 2,
    cKhepera4SensorsInfrared_Front = 3,
    cKhepera4SensorsInfrared_FrontRight = 4,
    cKhepera4SensorsInfrared_Right = 5,
    cKhepera4SensorsInfrared_BackRight = 6,
    cKhepera4SensorsInfrared_Back = 7,
    cKhepera4SensorsInfrared_FloorLeft = 8,
    cKhepera4SensorsInfrared_FloorFrontLeft = 9,
    cKhepera4SensorsInfrared_FloorFrontRight = 10,
    cKhepera4SensorsInfrared_FloorRight = 11,

    cKhepera4SensorsInfrared_Begin = 0,
    cKhepera4SensorsInfrared_End = 11,
    cKhepera4SensorsInfrared_Count = 12,
    cKhepera4SensorsInfrared_RingBegin = 0,
    cKhepera4SensorsInfrared_RingEnd = 7,
    cKhepera4SensorsInfrared_RingCount = 8,
    cKhepera4SensorsInfrared_FloorBegin = 8,
    cKhepera4SensorsInfrared_FloorEnd = 11,
    cKhepera4SensorsInfrared_FloorCount = 4
};


//! Khepera4 Infrared Structure
struct sKhepera4SensorsInfrared {
	int sensor[cKhepera4SensorsInfrared_Count];			//!< Sensor values
	int timestamp;										//!< Timestamp at which the values were read
};

//! Initializes webots infrared sensors
void khepera4_infrared_init();

//! Assigns webots infrared sensors previously initialized to the khepera4 infrared structure
void khepera4_infrared_proximity();

//! Reads the last ambient infrared values and fills them into the khepera4 structure.
void khepera4_infrared_ambient();

#endif

