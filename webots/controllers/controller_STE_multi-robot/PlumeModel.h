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
// #include <eigen3/Eigen/Dense>

// #include "pyhelper.hpp"
using namespace std;

#define PI 3.14159265358979

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

	// virtual PlumeModel() = 0;
	// //copy constructor
	// PlumeModel(const PlumeModel & another_model);

	// destructor
	// virtual ~PlumeModel() = 0;

	// function that returns concentration using parameters
	virtual double concentration(Position robot_position, double wind_intensity, double param_list[]) = 0;
	virtual void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_list[], vector<float> &c_avg, vector<float> &c_std) = 0;
	// virtual void concentration(Position robot_position[], int nbr, double wind_intensity, vector<float> &c_avg) = 0;

	// read from precomputed maps
	virtual void retrieve_hist_measurements(int idx[], int idy[], int nbr, double wind_intensity, int soure_pos_idx, vector<double> &c_avg) = 0;

	virtual double approx_concentration(Position robot_position, double wind_intensity) = 0;

	// print expected and max a posteriori values on screen
	virtual void print_expected_val() = 0;
	virtual void print_max_a_post_val() = 0;

	virtual int get_source_position_params_nbr() = 0;
	virtual void get_source_position_params(parameter *source_position_param_list) = 0;
	// virtual void calculate_gas_con_map_for_each_source_location(GasDispersionMap &gas_map) = 0;
	virtual void calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf) = 0;
	virtual void calculate_weighted_gas_con_mean_map(double *gas_con_variance_map, double *pdf) = 0;
	virtual void calculate_weighted_gas_hit_variance_map(double *gas_hit_variance_map, double *pdf) = 0;
	virtual void log_gas_concentration_map(char *filename, char writing_mode) = 0;
	virtual void initialize_information_matrix() = 0;

	void random_params(double *random_params);

	void get_expected_value(double *container);
	void get_max_a_post_value(double *container);
	void set_expected_value(double *container);
	void set_max_a_post_value(double *container);
};

/********************************************************/
//					GaussianPlumeModel2D 				//
/********************************************************/

// 2D pseudo Gaussian plume model that was initially used in STE (ICRA2019)
class GaussianPlumeModel2D : public virtual PlumeModel
{
public:
	// constructor : default values
	GaussianPlumeModel2D();

	// destructor
	~GaussianPlumeModel2D();

	// returns concentration for a set of parameters and a position
	double concentration(Position robot_position, double wind_intensity, double param_list[]);
	void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_list[], vector<float> &c_avg);
	// void concentration(Position robot_position[], int nbr, double wind_intensity, vector<float> &c_avg);

	double approx_concentration(Position robot_position, double wind_intensity);

	// print expected and max a posteriori values on screen
	void print_expected_val();
	void print_max_a_post_val();

	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);

	void initialize_information_matrix();
	void calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_con_mean_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_hit_variance_map(double *gas_hit_variance_map, double *pdf);
	void log_gas_concentration_map(char *filename, char writing_mode);
};

/********************************************************/
//					NNGaussianPlumeModel2D 				//
/********************************************************/

// 2D pseudo Gaussian plume model generated using data from webots and NN training in Matlab
class NNGaussianPlumeModel2D : public virtual PlumeModel
{
public:
	// constructor : default values
	NNGaussianPlumeModel2D();

	// destructor
	~NNGaussianPlumeModel2D();

	double myNeuralNetworkFunction(const double x1[3]);

	// returns concentration for a set of parameters and a position
	double concentration(Position robot_position, double wind_intensity, double param_list[]);

	double approx_concentration(Position robot_position, double wind_intensity);

	// print expected and max a posteriori values on screen
	void print_expected_val();
	void print_max_a_post_val();

	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);

	void initialize_information_matrix();
	void calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_con_mean_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_hit_variance_map(double *gas_hit_variance_map, double *pdf);
	void log_gas_concentration_map(char *filename, char writing_mode);
};

/********************************************************/
//					DeepCFDPlumeModel2D 				//
/********************************************************/

// 2D plume model generated by DeepCFD CNN
class DeepCFDPlumeModel2D : public virtual PlumeModel
{
public:
	// constructor : default values
	DeepCFDPlumeModel2D();

	DeepCFDPlumeModel2D(const DeepCFDPlumeModel2D &another_model);

	// destructor
	~DeepCFDPlumeModel2D();

	// model
	torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
	;
	torch::jit::script::Module DeepCFD;

	// load pytorch model
	void initialize_model_features();

	// feature
	torch::Tensor x;
	torch::Tensor r_torch;
	torch::Tensor c_torch;
	vector<GasDispersionMap> gas_con_map_list;
	vector<CellMeasurements> cell_lists;

	// resturns gas concentration map for a set of parameters
	void initialize_information_matrix();
	void initialize_gas_concentration_map_list();
	void initialize_cell_measurement_list();
	void calculate_gas_con_map_for_each_source_location(GasDispersionMap &gas_con_map);
	void calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_con_mean_map(double *gas_con_variance_map, double *pdf);
	void calculate_weighted_gas_hit_variance_map(double *gas_hit_variance_map, double *pdf);
	void calculate_gas_concentration_map(GasDispersionMap &gas_map);

	double concentration(Position robot_position, double wind_intensity, double param_list[]);
	void concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_list[], vector<float> &c_avg, vector<float> &c_std);
	// void concentration(Position robot_position[], int nbr, double wind_intensity, vector<float> &c_avg);

	// read from precomputed maps
	void retrieve_hist_measurements(int idx[], int idy[], int nbr, double wind_intensity, int soure_pos_idx, vector<double> &c_avg);

	double approx_concentration(Position robot_position, double wind_intensity);

	// print expected and max a posteriori values on screen
	void print_expected_val();
	void print_max_a_post_val();

	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);
	// void random_params(double * random_params);
	void log_gas_concentration_map(char *filename, char writing_mode);
};

#endif