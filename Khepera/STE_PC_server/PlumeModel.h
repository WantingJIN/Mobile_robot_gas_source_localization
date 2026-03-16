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
// #include <eigen3/Eigen/Dense>
#include "SimulationParameters.h"


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



#endif
