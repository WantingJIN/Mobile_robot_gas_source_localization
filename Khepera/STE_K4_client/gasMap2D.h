#ifndef KERNEL3D_H_
#define KERNEL3D_H_

#include <eigen3/Eigen/Geometry>
using namespace Eigen;

//#include "Utils.h"
//#include "GasMap.h"
#include "Position.h"
#include <vector>
// #include <torch/script.h> // One-stop header.
// #include <torch/torch.h>

using namespace std;

//gas map boundaries
#define map_x_min 2.75
#define map_x_max 12.7418
#define map_y_min 0.01
#define map_y_max 3.9916
#define map_x_nbins 64
#define map_y_nbins 64


struct KernelParameters {
  float sigma;
  float gamma;
  float scaling_param;
  float wind_speed;
  float rco;
  float wmin;
  float rmin;
  float rmax;
};

class GasMap2D{
  public:
    Eigen::Matrix<double,map_x_nbins,map_y_nbins> gasWeightConMap;
    Eigen::Matrix<double,map_x_nbins,map_y_nbins> gasAverConMap;
    Eigen::Matrix<double,map_x_nbins,map_y_nbins> gasWeightMap;
    Eigen::Matrix<double,map_x_nbins,map_y_nbins> gasConfidenceMap;

    double x_min;
    double y_min;
    double x_max;
    double y_max;
    double x_resol;
    double y_resol;
    double x_update_range;
    double y_update_range;

    
    GasMap2D(double sigma, double gamma, double scaling_param, double wind_speed, double rco);
    void configureKernelAlgorithm(double sigma, double gamma, double scaling_param, double wind_speed, double rco);
    void runKernelAlgorithm(Position current_position, float current_odor);
    KernelParameters kp;
    void initialize_gas_map_features();
    void retrieve_readings_from_gasmap(const vector<Position> history_position, vector<double> &concentrations_from_map);

    
  private:
    Eigen::Matrix2f _sigma_matrix;
    float _gaussian_divisor;
    float avg_WRt;
    float counter_WRt;

    void setBinBorders(int n_bins);
    void setSigmaMatrix();
    void setGaussianDivisor();
    float compute_gaussian(const double x_dist, const double y_dist);

};

#endif
