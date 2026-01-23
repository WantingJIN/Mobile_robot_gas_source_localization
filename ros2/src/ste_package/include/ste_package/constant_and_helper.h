#ifndef CONSTANT_H
#define CONSTANT_H
#include <math.h>
#include <string>
// Constant variables
template <typename T>
constexpr T BOUND(const T& x, const T& a, const T& b) noexcept {
    return (x < a) ? a : (x > b ? b : x);
    // Alternatively, if using C++17 or newer, you can just use:
    // return std::clamp(x, a, b);
}

// ------------------------------------------------------------
// 🔢 General constants
// ------------------------------------------------------------
inline constexpr double PI = 3.14159265358979;
inline constexpr int TIME_STEP = 64;           // milliseconds
inline constexpr double SAFETY_MARGIN = 0.2;   // meters
inline constexpr double WHEEL_RADIUS = 0.021;  // meters (K4)
inline constexpr double AXLE_LENGTH = 0.1054;  // meters (K4)
inline constexpr int ODOR_FILTER_LENGTH = 150; // ~10 sec
inline constexpr int MAX_ITER = 30;
inline constexpr int MAX_RELOCATIONS = 5;
inline constexpr double MAX_SPEED = 47.61;     // K4 speed constant
inline constexpr int SENSOR_NUMBER = 8;
inline constexpr int MAX_NB_ROBOTS = 5;

// ------------------------------------------------------------
// 🌬️ Wind tunnel boundaries
// ------------------------------------------------------------
inline constexpr double XMIN = -9;
inline constexpr double YMIN = -2.2;
inline constexpr double XMAX = 2.5;
inline constexpr double YMAX = 2.5;
// inline constexpr double ZMIN = 0.1;
// inline constexpr double ZMAX = 1.9;

// ------------------------------------------------------------
// 📈 Parameter ranges
// ------------------------------------------------------------
inline constexpr double Q_min = 80.0;
inline constexpr double Q_max = 100.0;

inline constexpr double a_y_min = 0.01;
inline constexpr double a_y_max = 0.05;
inline constexpr double b_y_min = 0.05;
inline constexpr double b_y_max = 0.1;

// ------------------------------------------------------------
// 📊 Number of bins
// ------------------------------------------------------------
inline constexpr int Q_nbins = 20;
inline constexpr int a_y_nbins = 10;
inline constexpr int b_y_nbins = 10;

// ------------------------------------------------------------
// 🧮 Derived quantities (computed at compile time)
// ------------------------------------------------------------
inline constexpr double Q_resol = (Q_max - Q_min) / Q_nbins;
inline constexpr double a_y_resol = (a_y_max - a_y_min) / a_y_nbins;
inline constexpr double b_y_resol = (b_y_max - b_y_min) / b_y_nbins;

// ------------------------------------------------------------
// 📏 Geometry and map
// ------------------------------------------------------------
inline constexpr double MAX_DISTANCE = 10.77; // from (2.75,0) to (12.75,4)
// inline constexpr int MAP_SIZE = x_nbins * y_nbins;
inline constexpr float OBS_MARGIN = 0.3;

// ------------------------------------------------------------
// 💡 Miscellaneous
// ------------------------------------------------------------
inline constexpr int CENTRAL_MASTER_ID = 101;
inline constexpr bool VERBOSE = true;
inline constexpr int TIMEOUT = 1000 * 1000; // microseconds

// declare the global parameter - the root path of the project
extern const char *workspacePath;

// declare the gloabl paramter - the estimation bound of the gas source
typedef struct {
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    int x_nbins;
    int y_nbins;
    int map_size; // derived = x_nbins * y_nbins

    float x_resol;  // derived = (x_max - x_min) / x_nbins
    float y_resol;  // derived = (y_max - y_min) / y_nbins
} SourceBounds;


// Function to initialize SourceBounds and compute resolutions
void set_source_bounds(SourceBounds *bounds,
                        double x_min, double y_min,
                        double x_max, double y_max,
                        int x_nbins, int y_nbins);

extern SourceBounds source_bounds;

// random value sampled from a uniform distribution
inline double randu()
{
	// random variable between 0 and 1
	return (double)rand() / (double)RAND_MAX;
}

// inline functions for position to index and vice versa
inline void position_to_indices_2D(float x, float y, int *idx_x, int *idx_y){
      *idx_x = (int)floor((x - source_bounds.x_min) / source_bounds.x_resol);
      *idx_y = (int)floor((y - source_bounds.y_min) / source_bounds.y_resol);
      // put idx_x and idx_y in the range of [0, x_nbins-1] and [0, y_nbins-1]      
      if (*idx_x < 0)
            *idx_x = 0; 
      if (*idx_x >= source_bounds.x_nbins)  
            *idx_x = source_bounds.x_nbins - 1;
      if (*idx_y < 0)
            *idx_y = 0;
      if (*idx_y >= source_bounds.y_nbins)
            *idx_y = source_bounds.y_nbins - 1;

      // if (*idx_y < 0 || *idx_y > y_nbins)
      //       *idx_y = -1; // invalid
      // if (*idx_x < 0 || *idx_x > x_nbins)
      //       *idx_x = -1; // invalid
      
}
inline void indices_to_position_2D(int idx_x, int idx_y, float *x, float *y){
      *x = source_bounds.x_min + idx_x * source_bounds.x_resol + source_bounds.x_resol / 2.0;
      *y = source_bounds.y_min + idx_y * source_bounds.y_resol + source_bounds.y_resol / 2.0;
}
inline int position_to_index_2D(float x, float y){
      int idx_x = (int)floor((x - source_bounds.x_min) / source_bounds.x_resol);
      int idx_y = (int)floor((y - source_bounds.y_min) / source_bounds.y_resol);
      int idx = idx_x + idx_y * source_bounds.x_nbins;
      // put idx in the range of [0, MAP_SIZE-1]
      if (idx < 0)
            idx = 0;
      if (idx >= source_bounds.map_size)    
            idx = source_bounds.map_size - 1;

      // if (idx < 0 || idx > MAP_SIZE)
      //       idx = -1; // invalid
      return idx;
}
inline void index_to_position_2D(int idx, float *x, float *y){
      int idx_x = idx % source_bounds.x_nbins;
      int idx_y = idx / source_bounds.x_nbins;
      *x = source_bounds.x_min + idx_x * source_bounds.x_resol + source_bounds.x_resol / 2.0;
      *y = source_bounds.y_min + idx_y * source_bounds.y_resol + source_bounds.y_resol / 2.0;
}
inline void index_to_indices_2D(int idx, int *idx_x, int *idx_y){
      *idx_x = idx % source_bounds.x_nbins;
      *idx_y = idx / source_bounds.x_nbins;
      // put idx_x and idx_y in the range of [0, x_nbins-1] and [0, y_nbins-1]
      if (*idx_x < 0)
            *idx_x = 0; 
      if (*idx_x >= source_bounds.x_nbins)  
            *idx_x = source_bounds.x_nbins - 1;
}
inline int indices_to_index_2D(int idx_x, int idx_y){
      int idx = idx_x + idx_y * source_bounds.x_nbins;
      // put idx in the range of [0, MAP_SIZE-1]
      if (idx < 0)
            idx = 0;
      if (idx >= source_bounds.map_size)    
            idx = source_bounds.map_size - 1;
      return idx;
}


#endif
