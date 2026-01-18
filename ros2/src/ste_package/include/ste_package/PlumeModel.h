#ifndef PLUME_MODEL_H
#define PLUME_MODEL_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "Position.h"
#include <Python.h>

#include <torch/script.h> // One-stop header.
#include <torch/torch.h>

// #include <numpy/ndarrayobject.h>
#include <fstream>
#include <string>

#include <iostream>
#include <memory>
// #include <opencv2/opencv.hpp>
#include <matio.h>
#include <cmath>
#include "CellMeasurements.h"
#include "GasDispersionMap.h"
#include <stdexcept>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
//#include <Eigen/PardisoSupport>
#include <Eigen/SuperLUSupport>
// #include <eigen3/Eigen/Dense>
#include "SimulationParameters.h"
#include "WindEstimator.h"


// #include "pyhelper.hpp"
using namespace std;
using std::vector;

constexpr double kPi = 3.14159265358979323846;

// parameter structure defines properties of a parameter. A model has an array of parameters.
typedef struct
{
	char name[24];
	float lower_limit;
	float upper_limit;
	float resolution;
	int nbins;
	double expected_val;
	double max_a_post_val;
} parameter;

extern FILE *console_logfile;
extern SimulationParameters simulation_parameters;
extern float ** obstacles;
extern int nbr_obs;

/********************************************************/
//						PlumeModel 						//
/********************************************************/

// The base class of plume models. It is an abstract class that defines the base of any plume model
class PlumeModel
{
public:
	// number of parameters to estimate
	int nbr_param;
	// list of parameters
	//(!!!!!!! PROTOCOL : source position x, y, (z) should be the first ones in the param_list)
	parameter *param_list;
	// expected values of the parameters
	double *expected_value;
	// max a posteriori values of the parameters
	double *max_a_post_value;

	// MCMC proposal distribution's covariance sigma_i,i
	double *covar_sig_ii;

	vector<GasDispersionMap> gas_con_map_list;
	vector<CellMeasurements> cell_lists;

	// virtual PlumeModel() = 0;
	// //copy constructor
	// PlumeModel(const PlumeModel & another_model);

	// destructor
	virtual ~PlumeModel() = default;


	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);

	// precompute maps
	void log_gas_concentration_map(char *filename, char writing_mode);
	void calculate_gas_map_and_information_matrix();
	void calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_con_mean_map(double *gas_con_variance_map, double *pdf);

	void calculate_gas_concentration_map_list();
	void calculate_cell_measurement_list();

	// function that returns concentration using parameters
	virtual double concentration(Position robot_position, double wind_intensity, double param_value[]) = 0;
	virtual void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_value[], vector<float> &c_avg, vector<float> &c_std) = 0;
	virtual double approx_concentration(Position robot_position, double wind_intensity) = 0;
	// virtual void concentration(Position robot_position[], int nbr, double wind_intensity, vector<float> &c_avg) = 0;

	// read from precomputed maps
	virtual void retrieve_hist_measurements(int idx[], int idy[], int nbr, int soure_pos_idx, vector<double> &c_avg);


	virtual void reset_estimation_ranges(SourceBounds source_bounds) = 0;

	virtual void update_wind_field_and_gas_concentration_map(vector<float> meas_x, vector<float> meas_y, vector<double> wind_meas_u, vector<double> wind_meas_v) = 0;
	virtual void log_wind_field(char *filename_u, char *filename_v, char *filename_w,char writing_mode) = 0;
	virtual double get_wind_estimation_confidence(int soure_pos_idx) = 0;

	void random_params(double *random_params);

	void get_expected_value(double *container);
	void get_max_a_post_value(double *container);
	void set_expected_value(double *container);
	void set_max_a_post_value(double *container);

	virtual void calculate_gas_concentration_map(GasDispersionMap &gas_map);

};

/********************************************************/
//					GaussianPlumeModel2D 				//
/********************************************************/

// 2D pseudo Gaussian plume model that was initially used in STE (ICRA2019)
class GaussianPlumeModel2D : public PlumeModel
{
public:
	// constructor : default values
	GaussianPlumeModel2D();

	// destructor
	~GaussianPlumeModel2D();

	double concentration(Position robot_position, double wind_intensity, double param_value[]) override;
	void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_value[], vector<float> &c_avg, vector<float> &c_std) override;
	double approx_concentration(Position robot_position, double wind_intensity) override;
    void update_wind_field_and_gas_concentration_map(vector<float> meas_x, vector<float> meas_y, vector<double> wind_meas_u, vector<double> wind_meas_v) override;

    // calculate the gas concentration map and information map

	void reset_estimation_ranges(SourceBounds source_bounds) override;
	void log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode) override;
	double get_wind_estimation_confidence(int soure_pos_idx) override;
};

/********************************************************/
//					DeepCFDPlumeModel2D 				//
/********************************************************/

// 2D plume model generated by DeepCFD CNN
class DeepCFDPlumeModel2D : public PlumeModel
{
public:
	// constructor : default values
	DeepCFDPlumeModel2D();

	DeepCFDPlumeModel2D(const DeepCFDPlumeModel2D &another_model);

	// destructor
	~DeepCFDPlumeModel2D();
	
    // override virtual functions from PlumeModel
	double concentration(Position robot_position, double wind_intensity, double param_value[]) override;
	void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_value[], vector<float> &c_avg, vector<float> &c_std) override;

	double approx_concentration(Position robot_position, double wind_intensity) override;
    
	// read from precomputed maps
	void reset_estimation_ranges(SourceBounds source_bounds) override;
	void update_wind_field_and_gas_concentration_map(vector<float> meas_x, vector<float> meas_y, vector<double> wind_meas_u, vector<double> wind_meas_v) override;
	void log_wind_field(char *filename_u, char *filename_v, char *filename_w,char writing_mode) override;
	double get_wind_estimation_confidence(int soure_pos_idx) override;

private:
	// feature
	torch::Tensor x;
	torch::Tensor r_torch;
	torch::Tensor c_torch;
	// model
	torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
	torch::jit::script::Module DeepCFD;

	// load pytorch model
	void initialize_model_features();
    void calculate_gas_concentration_map(GasDispersionMap &gas_map) override;
};


/*************************************************************************************************/
//	ADEPlumeModel2D (2D Steady state of Advection-Diffusion Model sloved by FDM) 				//
/************************************************************************************************/

class AdeFDMPlumeModel2D : public PlumeModel
{
public:
	// constructor : default values
	AdeFDMPlumeModel2D();

	AdeFDMPlumeModel2D(const AdeFDMPlumeModel2D &another_model);
	// destructor

	~AdeFDMPlumeModel2D();

	using SpMat  = Eigen::SparseMatrix<double>;
    using Triplet = Eigen::Triplet<double>;
    void reset_estimation_ranges(SourceBounds source_bounds) override;
	void retrieve_hist_measurements(int idx[], int idy[], int nbr, int soure_pos_idx, vector<double> &c_avg) override;
	void update_wind_field_and_gas_concentration_map(vector<float> meas_x, vector<float> meas_y, vector<double> wind_meas_u, vector<double> wind_meas_v) override;
    void log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode) override;
	double get_wind_estimation_confidence(int soure_pos_idx) override;
	double concentration(Position robot_position, double wind_intensity, double param_value[]) override;
	void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_value[], vector<float> &c_avg, vector<float> &c_std) override;
	double approx_concentration(Position robot_position, double wind_intensity) override;
private:
	void calculate_gas_concentration_map(GasDispersionMap &gas_map) override;
	// Here, the grid used to solve PDE is different than the grid for the source position estimation
	int Nx_, Ny_, N_;
	double dx_, dy_;
    double D_; // diffusion coefficient
	double Q_; // source quantitity
	double u_x; // uniform wind speed
	double source_strength_; // source strength
    WindEstimator* wind_estimator_ = nullptr;

	SpMat A_;
	// Eigen::VectorXd b = Eigen::VectorXd::Zero(N_);
	// Eigen::VectorXd x = Eigen::VectorXd::Zero(N_); // solution vector
	// u_x per cell (x-advection only, like your original)
    std::vector<double> u_;
	std::vector<double> v_;
	std::vector<double> w_;
	// Eigen::VectorXd b_;
    //mutable Eigen::SparseLU<SpMat> solver_;
	mutable Eigen::SuperLU<SpMat> solver_;
    bool fact_ok_{false};
	void set_u_field_uniform(double u);
	void assemble_and_factorize();
	void set_parameters(double Nx, double Ny, double D, double Q);
	std::vector<double> solve_point_source(double sx, double sy) const;
	static void load_csv(const std::string& filename,
                         std::vector<double>& xs,
                         std::vector<double>& ys,
                         std::vector<double>& us,
                         std::vector<double>& vs);

	void set_u_field_from_csv(const std::string& filename, int Nx_src, int Ny_src);
	// Bilinear interpolation helper
	static double bilinear(double x, double y,
                           const std::vector<double>& X,
                           const std::vector<double>& Y,
                           const std::vector<double>& F,
                           int Nx, int Ny);

    // helper functions; convert between world position and (gas estimation) grid indices
    int grid_indices_to_idx(int i, int j) const { return i + Nx_ * j; }
	void world_pos_to_grid_indices (double x, double y, int &i, int &j) const {
		i = clamp_to_grid_x(world_to_i(x));
		j = clamp_to_grid_y(world_to_j(y));
	}
	int clamp_to_grid_x(int i) const { return std::max(0, std::min(i, Nx_ - 1)); }
	int clamp_to_grid_y(int j) const { return std::max(0, std::min(j, Ny_ - 1)); }
	int grid_idx(int i, int j) const { return (i < 0 || i >= Nx_ || j < 0 || j >= Ny_) ? -1 : grid_indices_to_idx(i, j); }
	int world_to_i(double x) const { return (int)floor((x - source_bounds.x_min) / dx_); }
	int world_to_j(double y) const { return (int)floor((y - source_bounds.y_min) / dy_); }
		
};

#endif