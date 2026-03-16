#ifndef CONSTANT_H
#define CONSTANT_H
#include <math.h>
//Constant variables
#define BOUND(x, a, b) (((x)<(a))?(a):((x)>(b))?(b):(x))
#define PI 					3.14159265358979
#define TIME_STEP 			64 //adjusts the speed (ms)
#define SAFETY_MARGIN 		.2 // how far in meters are you ready to approach the walls
#define WHEEL_RADIUS	        0.021  	//K4: 0.021 		//K3: 0.0205	  // Wheel radius (meters)
#define AXLE_LENGTH             0.1054 	//K4: 0.0527×2 	//K3: 0.08841
#define ODOR_FILTER_LENGTH 	150 //~10sec //number of measurements before averaging
//#define ODOR_FILTER_LENGTH 	1 //~10sec //number of measurements before averaging
// #define MCMC_iteration_num 	30 
#define N_CONCENTRATION_LEVEL 4
#define MAX_SPEED 			47.61 //K3: 35748.4
#define SENSOR_NUMBER  		8
#define MAX_NB_ROBOTS          	3
//wind tunnel boundaries

inline constexpr double XMIN = 3.0;
inline constexpr double YMIN = 0.16;
inline constexpr double XMAX = 12.75;
inline constexpr double YMAX = 3.8;
// inline constexpr double ZMIN = 0.1;
// inline constexpr double ZMAX = 1.9;

// min and max for each parameter
#define Q_min  1.0
#define Q_max  101.0
// Source estimation boundaries
inline constexpr float source_x_min = XMIN + SAFETY_MARGIN;
inline constexpr float source_y_min = YMIN + SAFETY_MARGIN;
inline constexpr float source_x_max = XMAX - SAFETY_MARGIN;
inline constexpr float source_y_max = YMAX - SAFETY_MARGIN;
inline constexpr int source_x_nbins = 20;
inline constexpr int source_y_nbins = 20;
inline constexpr float source_x_resol = (source_x_max - source_x_min) / source_x_nbins;
inline constexpr float source_y_resol = (source_y_max - source_y_min) / source_y_nbins;
inline constexpr int MAP_SIZE = source_x_nbins * source_y_nbins;
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
// #define x_nbins 	20 //(source_x_max - source_x_min)/0.1 //0.1 m is the granuality
// #define y_nbins 	20 //(source_y_max - source_y_min)/0.1 //0.1 m is the granuality
//#define z_nbins 	10 //(source_z_max - source_z_min)/0.1 //0.1 m is the granuality
#define a_y_nbins 	10 //(a_y_max - a_y_min) / 0.0025
#define b_y_nbins 	10 //(b_y_max - b_y_min) / 0.025
//#define a_z_nbins 	10 //(a_z_max - a_z_min) / 0.0025
//#define b_z_nbins 	10 //(b_z_max - b_z_min) / 0.025
//resolution of the estimated terms
#define Q_resol    (static_cast<double>(Q_max-Q_min)/Q_nbins)
// #define x_resol    (static_cast<double>(source_x_max-source_x_min)/x_nbins)
// #define y_resol    (static_cast<double>(source_y_max-source_y_min)/y_nbins)
#define a_y_resol  (static_cast<double>(a_y_max-a_y_min)/a_y_nbins)
#define b_y_resol  (static_cast<double>(b_y_max-b_y_min)/b_y_nbins)

#define VERBOSE 		1

#define TIMEOUT         1000*1000 // miliseconds

extern const char* workspacePath; 


// random value sampled from a uniform distribution
inline double randu()
{
	// random variable between 0 and 1
	return (double)rand() / (double)RAND_MAX;
}

// inline functions for position to index and vice versa
inline void position_to_indices_2D(float x, float y, int *idx_x, int *idx_y){
      *idx_x = (int)floor((x - source_x_min) / source_x_resol);
      *idx_y = (int)floor((y - source_y_min) / source_y_resol);
      // put idx_x and idx_y in the range of [0, x_nbins-1] and [0, y_nbins-1]      
      if (*idx_x < 0)
            *idx_x = 0; 
      if (*idx_x >= source_x_nbins)  
            *idx_x = source_x_nbins - 1;
      if (*idx_y < 0)
            *idx_y = 0;
      if (*idx_y >= source_y_nbins)
            *idx_y = source_y_nbins - 1;

      // if (*idx_y < 0 || *idx_y > y_nbins)
      //       *idx_y = -1; // invalid
      // if (*idx_x < 0 || *idx_x > x_nbins)
      //       *idx_x = -1; // invalid
      
}
inline void indices_to_position_2D(int idx_x, int idx_y, float *x, float *y){
      *x = source_x_min + idx_x * source_x_resol + source_x_resol / 2.0;
      *y = source_y_min + idx_y * source_y_resol + source_y_resol / 2.0;
}
inline int position_to_index_2D(float x, float y){
      int idx_x = (int)floor((x - source_x_min) / source_x_resol);
      int idx_y = (int)floor((y - source_y_min) / source_y_resol);
      int idx = idx_x + idx_y * source_x_nbins;
      // put idx in the range of [0, MAP_SIZE-1]
      if (idx < 0)
            idx = 0;
      if (idx >= MAP_SIZE)    
            idx = MAP_SIZE - 1;

      // if (idx < 0 || idx > MAP_SIZE)
      //       idx = -1; // invalid
      return idx;
}
inline void index_to_position_2D(int idx, float *x, float *y){
      int idx_x = idx % source_x_nbins;
      int idx_y = idx / source_x_nbins;
      *x = source_x_min + idx_x * source_x_resol + source_x_resol / 2.0;
      *y = source_y_min + idx_y * source_y_resol + source_y_resol / 2.0;
}
inline void index_to_indices_2D(int idx, int *idx_x, int *idx_y){
      *idx_x = idx % source_x_nbins;
      *idx_y = idx / source_x_nbins;
      // put idx_x and idx_y in the range of [0, x_nbins-1] and [0, y_nbins-1]
      if (*idx_x < 0)
            *idx_x = 0; 
      if (*idx_x >= source_x_nbins)  
            *idx_x = source_x_nbins - 1;
}
inline int indices_to_index_2D(int idx_x, int idx_y){
      int idx = idx_x + idx_y * source_x_nbins;
      // put idx in the range of [0, MAP_SIZE-1]
      if (idx < 0)
            idx = 0;
      if (idx >= MAP_SIZE)    
            idx = MAP_SIZE - 1;
      return idx;
}

#endif


