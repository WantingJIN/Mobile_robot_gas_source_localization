#ifndef PLUME_MODEL_H
#define PLUME_MODEL_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "Position.h"

// #include <numpy/ndarrayobject.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <memory>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include "CellMeasurements.h"
#include "GasDispersionMap.h"
#include "SimulationParameters.h"
#include "WindEstimator.h"
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
	float expected_val;
	float max_a_post_val;
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
	float *expected_value;
	// max a posteriori values of the parameters
	float *max_a_post_value;

	// MCMC proposal distribution's covariance sigma_i,i
	float *covar_sig_ii;

	virtual ~PlumeModel() = default;

	// function that returns concentration using parameters
	virtual float concentration(Position robot_position, float wind_intensity, float param_list[]) = 0;
	virtual void concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list[], vector<float> &c_avg, vector<float> &c_std) = 0;
	// virtual void concentration(Position robot_position[], int nbr, float wind_intensity, vector<float> &c_avg) = 0;

	// read from precomputed maps
	virtual void retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int soure_pos_idx, vector<float> &c_avg) = 0;

	virtual float approx_concentration(Position robot_position, float wind_intensity) = 0;

	// print expected and max a posteriori values on screen
	virtual void print_expected_val() = 0;
	virtual void print_max_a_post_val() = 0;

	virtual int get_source_position_params_nbr() = 0;
	virtual void get_source_position_params(parameter *source_position_param_list) = 0;
	// virtual void calculate_gas_con_map_for_each_source_location(GasDispersionMap &gas_map) = 0;
	virtual void calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf) = 0;
	virtual void calculate_weighted_gas_con_mean_map(float *gas_con_variance_map, float *pdf) = 0;
	virtual void calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf) = 0;
	virtual void log_gas_concentration_map(char *filename, char writing_mode) = 0;
	virtual void initialize_potential_gas_maps() = 0;
	virtual void update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
	                                                         const vector<float> &meas_y,
	                                                         const vector<float> &wind_meas_u,
	                                                         const vector<float> &wind_meas_v) = 0;
	virtual void log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode) = 0;
	virtual float get_wind_estimation_confidence(int source_pos_idx) = 0;

	void random_params(float *random_params);

	void get_expected_value(float *container);
	void get_max_a_post_value(float *container);
	void set_expected_value(float *container);
	void set_max_a_post_value(float *container);
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
	float concentration(Position robot_position, float wind_intensity, float param_list[]);
	void concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list[], vector<float> &c_avg, vector<float> &c_std);
	// void concentration(Position robot_position[], int nbr, float wind_intensity, vector<float> &c_avg);

	float approx_concentration(Position robot_position, float wind_intensity);

	// print expected and max a posteriori values on screen
	void print_expected_val();
	void print_max_a_post_val();

	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);

	void initialize_potential_gas_maps();
	void calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf);
	void calculate_weighted_gas_con_mean_map(float *gas_con_variance_map, float *pdf);
	void calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf);
	void log_gas_concentration_map(char *filename, char writing_mode);
	void retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int source_pos_idx, vector<float> &c_avg);
	void update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
	                                                 const vector<float> &meas_y,
	                                                 const vector<float> &wind_meas_u,
	                                                 const vector<float> &wind_meas_v);
	void log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode);
	float get_wind_estimation_confidence(int source_pos_idx);
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

	vector<GasDispersionMap> gas_con_map_list;
	vector<CellMeasurements> cell_lists;

	// resturns gas concentration map for a set of parameters
	void initialize_potential_gas_maps();
	void initialize_cell_measurement_list();
	void load_gas_concentration_map_list_from_file(const char *filename);
	void calculate_gas_con_map_for_each_source_location(GasDispersionMap &gas_con_map);
	void calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf);
	void calculate_weighted_gas_con_mean_map(float *gas_con_variance_map, float *pdf);
	void calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf);
	void calculate_gas_concentration_map(GasDispersionMap &gas_map);

	float concentration(Position robot_position, float wind_intensity, float param_list[]);
	void concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list[], vector<float> &c_avg, vector<float> &c_std);
	// void concentration(Position robot_position[], int nbr, float wind_intensity, vector<float> &c_avg);

	// read from precomputed maps
	void retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int soure_pos_idx, vector<float> &c_avg);

	float approx_concentration(Position robot_position, float wind_intensity);

	// print expected and max a posteriori values on screen
	void print_expected_val();
	void print_max_a_post_val();

	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);
	// void random_params(float * random_params);
	void log_gas_concentration_map(char *filename, char writing_mode);
	void update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
	                                                 const vector<float> &meas_y,
	                                                 const vector<float> &wind_meas_u,
	                                                 const vector<float> &wind_meas_v);
	void log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode);
	float get_wind_estimation_confidence(int source_pos_idx);
};

class AdeFDMPlumeModel2D : public virtual PlumeModel
{
public:
	AdeFDMPlumeModel2D();
	AdeFDMPlumeModel2D(const AdeFDMPlumeModel2D &another_model);
	~AdeFDMPlumeModel2D();

	vector<GasDispersionMap> gas_con_map_list;
	vector<CellMeasurements> cell_lists;

	void initialize_potential_gas_maps();
	void initialize_cell_measurement_list();
	void calculate_gas_concentration_map(GasDispersionMap &gas_map);
	void calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf);
	void calculate_weighted_gas_con_mean_map(float *gas_con_mean_map, float *pdf);
	void calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf);
	void log_gas_concentration_map(char *filename, char writing_mode);

	float concentration(Position robot_position, float wind_intensity, float param_list[]);
	void concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list[], vector<float> &c_avg, vector<float> &c_std);
	void retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int source_pos_idx, vector<float> &c_avg);
	float approx_concentration(Position robot_position, float wind_intensity);

	void print_expected_val();
	void print_max_a_post_val();
	int get_source_position_params_nbr();
	void get_source_position_params(parameter *source_position_param_list);

	void update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
	                                                 const vector<float> &meas_y,
	                                                 const vector<float> &wind_meas_u,
	                                                 const vector<float> &wind_meas_v);
	void log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode);
	float get_wind_estimation_confidence(int source_pos_idx);

private:
	typedef Eigen::SparseMatrix<float> SpMat;
	typedef Eigen::Triplet<float> Triplet;

	int Nx_;
	int Ny_;
	int N_;
	float dx_;
	float dy_;
	float D_;
	float Q_;
	float u_x_;
	float source_strength_;
	WindEstimator *wind_estimator_;
	SpMat A_;
	mutable Eigen::SparseLU<SpMat> solver_;
	bool fact_ok_;
	std::vector<float> u_;
	std::vector<float> v_;
	std::vector<float> w_;

	void set_parameters(int Nx, int Ny, float D, float Q);
	void set_u_field_uniform(float u);
	bool set_u_field_from_csv(const std::string &filename, int Nx_src, int Ny_src);
	static bool load_csv(const std::string &filename,
	                     std::vector<float> &xs,
	                     std::vector<float> &ys,
	                     std::vector<float> &us,
	                     std::vector<float> &vs);
	static float bilinear(float x, float y,
	                      const std::vector<float> &X,
	                      const std::vector<float> &Y,
	                      const std::vector<float> &F,
	                      int Nx, int Ny);
	void assemble_and_factorize();
	std::vector<float> solve_point_source(float sx, float sy) const;
	void rebuild_cell_measurements();
	int grid_indices_to_idx(int i, int j) const { return i + Nx_ * j; }
	void world_pos_to_grid_indices(float x, float y, int &i, int &j) const;
	int clamp_to_grid_x(int i) const;
	int clamp_to_grid_y(int j) const;
	int grid_idx(int i, int j) const;
	int world_to_i(float x) const;
	int world_to_j(float y) const;
};

#endif
