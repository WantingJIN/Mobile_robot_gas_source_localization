#ifndef CONSTANT_H
#define CONSTANT_H

//Constant variables
#define BOUND(x, a, b) (((x)<(a))?(a):((x)>(b))?(b):(x))
#define PI 					3.14159265358979
#define TIME_STEP 			64 //adjusts the speed (ms)
#define SAFETY_MARGIN 		.2 // how far in meters are you ready to approach the walls
#define WHEEL_RADIUS	        0.021  	//K4: 0.021 		//K3: 0.0205	  // Wheel radius (meters)
#define AXLE_LENGTH             0.1054 	//K4: 0.0527×2 	//K3: 0.08841
#define ODOR_FILTER_LENGTH 	150 //~10sec //number of measurements before averaging
//#define ODOR_FILTER_LENGTH 	1 //~10sec //number of measurements before averaging
#define MAX_SAMPLE_NUMBER 	30
#define N_CONCENTRATION_LEVEL 4
#define MAX_SPEED 			47.61 //K3: 35748.4
#define SENSOR_NUMBER  		8
#define MAX_NB_ROBOTS          	3
//wind tunnel boundaries
#define XMIN 3.0
#define YMIN 0.16
//#define ZMIN 0.1
#define XMAX 12.5
#define YMAX 3.8

//#define ZMAX 1.9
// min and max for each parameter
#define Q_min  1.0
#define Q_max  20.0
#define source_x_min  3.0
#define source_y_min  0.0
//#define source_z_min  0
#define source_x_max  12.75
#define source_y_max  4.0
//#define source_z_max  2
#define a_y_min  0.01
#define a_y_max  0.05
#define b_y_min  0.01
#define b_y_max  0.05
//#define a_z_min  0.01
//#define a_z_max  0.05
//#define b_z_min  0.1
//#define b_z_max  0.5
// number of bins for posterior PDF
#define Q_nbins 	20 //(Q_max - Q_min) /20
#define x_nbins 	20 //(source_x_max - source_x_min)/0.1 //0.1 m is the granuality
#define y_nbins 	20 //(source_y_max - source_y_min)/0.1 //0.1 m is the granuality
//#define z_nbins 	10 //(source_z_max - source_z_min)/0.1 //0.1 m is the granuality
#define a_y_nbins 	10 //(a_y_max - a_y_min) / 0.0025
#define b_y_nbins 	10 //(b_y_max - b_y_min) / 0.025
//#define a_z_nbins 	10 //(a_z_max - a_z_min) / 0.0025
//#define b_z_nbins 	10 //(b_z_max - b_z_min) / 0.025
//resolution of the estimated terms
#define Q_resol    (static_cast<double>(Q_max-Q_min)/Q_nbins)
#define x_resol    (static_cast<double>(source_x_max-source_x_min)/x_nbins)
#define y_resol    (static_cast<double>(source_y_max-source_y_min)/y_nbins)
#define a_y_resol  (static_cast<double>(a_y_max-a_y_min)/a_y_nbins)
#define b_y_resol  (static_cast<double>(b_y_max-b_y_min)/b_y_nbins)
#define MAX_DISTANCE 10.77 //from (2.75,0) to (12.75,4)

#define VERBOSE 		1

#define NO_COM_NO_COOR	0
#define COM_NO_COOR		1
#define COM_COOR 		2
#define SCENARIO 		NO_COM_NO_COOR

#define TIMEOUT         1000*1000 // miliseconds


#endif


