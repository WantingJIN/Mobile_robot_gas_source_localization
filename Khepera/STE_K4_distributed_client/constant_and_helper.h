#ifndef CONSTANT_H
#define CONSTANT_H

#include <cmath>  // C++ standard math header

// --- Mathematical constants ---
constexpr float PI = 3.1415926f;

// --- Time and control parameters ---
constexpr int TIME_STEP = 64;            // milliseconds
constexpr float SAFETY_MARGIN = 0.2f;    // meters
constexpr int ODOR_FILTER_LENGTH = 50;   // samples
constexpr int MAX_ITER = 30;

// --- Robot hardware parameters ---
constexpr float WHEEL_RADIUS = 0.0205f;  // meters
constexpr float AXLE_LENGTH = 0.08841f;  // meters

// --- Robot sensing limits ---
constexpr int SENSOR_NUMBER = 8;
constexpr int MAX_NB_ROBOTS = 5;

// --- Arena boundaries ---
constexpr float XMIN = 3.0f;
constexpr float YMIN = 0.16f;
constexpr float XMAX = 12.5f;
constexpr float YMAX = 3.8f;

// --- Source location and dispersion model parameters ---
constexpr float Q_min = 80.0f;
constexpr float Q_max = 100.0f;

constexpr float source_x_min = 3.0f;
constexpr float source_x_max = 12.75f;
constexpr float source_y_min = 0.0f;
constexpr float source_y_max = 4.0f;

constexpr float a_y_min = 0.01f;
constexpr float a_y_max = 0.05f;
constexpr float b_y_min = 0.05f;
constexpr float b_y_max = 0.1f;

// --- Discretization bins ---
constexpr int Q_nbins     = 20;
constexpr int x_nbins     = 20;
constexpr int y_nbins     = 20;
constexpr int a_y_nbins   = 10;
constexpr int b_y_nbins   = 10;

// --- Resolution of parameters ---
constexpr float Q_resol   = (Q_max - Q_min) / static_cast<float>(Q_nbins);
constexpr float x_resol   = (source_x_max - source_x_min) / static_cast<float>(x_nbins);
constexpr float y_resol   = (source_y_max - source_y_min) / static_cast<float>(y_nbins);
constexpr float a_y_resol = (a_y_max - a_y_min) / static_cast<float>(a_y_nbins);
constexpr float b_y_resol = (b_y_max - b_y_min) / static_cast<float>(b_y_nbins);

// --- Other system constants ---
constexpr float MAX_DISTANCE = 10.77f;  // meters
constexpr int MAP_SIZE       = x_nbins * y_nbins;
constexpr int CENTRAL_MASTER_ID = 101;
constexpr int TIMEOUT        = 1000 * 1000; // microseconds
constexpr bool VERBOSE        = true;

// -------- Utility Functions --------
template <typename T>
inline T bound(T x, T a, T b) {
    return (x < a) ? a : (x > b) ? b : x;
}

// -------- Grid Helper Functions (2D) --------
inline void position_to_indices_2D(float x, float y, int *idx_x, int *idx_y){
      *idx_x = (int)floor((x - source_x_min) / x_resol);
      *idx_y = (int)floor((y - source_y_min) / y_resol);
}
inline void indices_to_position_2D(int idx_x, int idx_y, float *x, float *y){
      *x = source_x_min + idx_x * x_resol + x_resol / 2.0;
      *y = source_y_min + idx_y * y_resol + y_resol / 2.0;
}
inline int position_to_index_2D(float x, float y){
      int idx_x = (int)floor((x - source_x_min) / x_resol);
      int idx_y = (int)floor((y - source_y_min) / y_resol);
      int idx = idx_x + idx_y * x_nbins;
      return idx;
}
inline void index_to_position_2D(int idx, float *x, float *y){
      int idx_x = idx % x_nbins;
      int idx_y = idx / x_nbins;
      *x = source_x_min + idx_x * x_resol + x_resol / 2.0;
      *y = source_y_min + idx_y * y_resol + y_resol / 2.0;
}
inline void index_to_indices_2D(int idx, int *idx_x, int *idx_y){
      *idx_x = idx % x_nbins;
      *idx_y = idx / x_nbins;
}
inline int indices_to_index_2D(int idx_x, int idx_y){
      int idx = idx_x + idx_y * x_nbins;
      return idx;
}

#endif
