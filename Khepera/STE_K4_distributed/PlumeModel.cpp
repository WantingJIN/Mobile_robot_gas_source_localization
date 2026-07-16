#include <math.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <climits>
#include <algorithm>
// #include "nvwa/debug_new.h"
#include "PlumeModel.h"
#include "CellMeasurements.h"
#include "constant_and_helper.h"
#include "GasDispersionMap.h"
// #define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// #include <numpy/arrayobject.h>
#include <stdexcept>

extern SimulationParameters simulation_parameters;
extern float **obstacles;
extern int nbr_obs;

/********************************************************/
//						PlumeModel 						//
/********************************************************/

// PlumeModel::PlumeModel(const PlumeModel & another_model){
// 	nbr_param = another_model.nbr_param;

// 	param_list = new parameter[nbr_param];
// 	expected_value = new float[nbr_param];
// 	max_a_post_value = new float[nbr_param];
// 	covar_sig_ii = new float[nbr_param];

// 	for(int i=0; i<nbr_param; i++){
// 		param_list[i] = another_model.param_list[i];
// 		expected_value[i] = another_model.expected_value[i];
// 		max_a_post_value[i] = another_model.max_a_post_value[i];
// 		covar_sig_ii[i] = another_model.covar_sig_ii[i];
// 	}
// }

// destructor
//  PlumeModel::~PlumeModel(){
//  	delete[] param_list;
//  	delete[] expected_value;
//  	delete[] max_a_post_value;
//  	delete[] covar_sig_ii;
//  }

void PlumeModel::random_params(float *rnd_params)
{
	for (int i = 0; i < nbr_param; i++)
		rnd_params[i] = ((float)rand() / (float)RAND_MAX) * (param_list[i].upper_limit - param_list[i].lower_limit) + param_list[i].lower_limit;
}

void PlumeModel::get_expected_value(float *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		container[i] = expected_value[i];
	}
}

void PlumeModel::get_max_a_post_value(float *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		container[i] = max_a_post_value[i];
	}
}

void PlumeModel::set_expected_value(float *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		expected_value[i] = container[i];
	}
}

void PlumeModel::set_max_a_post_value(float *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		max_a_post_value[i] = container[i];
	}
}

/********************************************************/
//					GaussianPlumeModel2D 				//
/********************************************************/

// constructor : default values
GaussianPlumeModel2D::GaussianPlumeModel2D()
{
	// number of parameters to estimate
	nbr_param = 5;

	// allocate memory for the parameters
	param_list = new parameter[nbr_param];

	// hard-coded parameters {name, lower_limit, upper_limit, resolution, nbins, estimated, max_a_post}
	// source_x
	param_list[0] = parameter{"source_x", source_x_min, source_x_max, x_resol, x_nbins, 0, 0};
	// source_y
	param_list[1] = parameter{"source_y", source_y_min, source_y_max, y_resol, y_nbins, 0, 0};
	// Q
	param_list[2] = parameter{"Q", Q_min, Q_max, Q_resol, Q_nbins, 0, 0};
	// sigma_a
	param_list[3] = parameter{"sigma_y_a", a_y_min, a_y_max, a_y_resol, a_y_nbins, 0, 0};
	// sigma_b
	param_list[4] = parameter{"sigma_y_b", b_y_min, b_y_max, b_y_resol, a_y_nbins, 0, 0};

	// initialize expected and max a posteriori values to zero
	expected_value = new float[nbr_param];
	max_a_post_value = new float[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	// MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new float[nbr_param];
	covar_sig_ii[0] = 4;	   // x
	covar_sig_ii[1] = 0.1;	   // y
	covar_sig_ii[2] = 4;	   // Q
	covar_sig_ii[3] = 0.00001; // sigma_y_a
	covar_sig_ii[4] = 0.0005;  // sigma_y_n
}

// destructor
GaussianPlumeModel2D::~GaussianPlumeModel2D()
{
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

int GaussianPlumeModel2D::get_source_position_params_nbr()
{
	return 2;
}

void GaussianPlumeModel2D::get_source_position_params(parameter *source_position_param_list)
{
	source_position_param_list[0] = param_list[0]; // x
	source_position_param_list[1] = param_list[1]; // y
}

// returns concentration for a set of parameters and a position
float GaussianPlumeModel2D::concentration(Position robot_position, float wind_intensity, float param_list[])
{
	// ASSUMPTION : X is towards the wind speed
	// notation simplification (axis rotation, hence the signs)
	float x = param_list[0] - robot_position.x;
	float y = param_list[1] - robot_position.y;
	float z = 0; // robot_position[2] - parameters.source_position[2];

	// verify if the point is not before the source
	if (x < 0)
		return 0.0;
	else
	{
		// std on y and z
		float sigma_y = param_list[3] * x + param_list[4];
		float sigma_z = 1; // parameters.a_z * x + parameters.b_z;
		// float sigma_y = 0.000803  * x*x - 0.004406 * x + 0.1726;
		// float sigma_z = 0.0006798 * x*x - 0.00244  * x + 0.1562;
		// float sigma_y = 0.01 * x + 0.1;
		// float sigma_z = 0.01 * x + 0.1;

		// model : C(x,y,z) = (Q/2pi.u.sigmaY.sigmaZ) * exp (-y^2/2sigmaY^2 - z^2/2sigmaZ^2)
		float model_value = (param_list[2] / (2 * PI * wind_intensity * sigma_y * sigma_z)) * exp(-((y * y) / (2 * sigma_y * sigma_y)) - ((z * z) / (2 * sigma_z * sigma_z)));

		return model_value;
	}
}

// returns a list of concentrations for ONE set of parameters and A LIST of positions
void GaussianPlumeModel2D::concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list[], vector<float> &c_avg, vector<float> &c_std)
{
	// ASSUMPTION : enough memory is already is already allocated for c_avg
	for (int i = 0; i < nbr; i++)
	{
		c_avg[i] = concentration(robot_position[i], wind_intensity, param_list);
	}
}

// returns a list of concentrations for A LIST of parameters sets and A LIST of positions
//  void GaussianPlumeModel2D::concentration(Position robot_position[], int nbr, float wind_intensity, vector<float> &c_avg){
//  	//for all source positions

// }

// void GaussianPlumeModel2D::concentration(Position robot_position[], int nbr, float wind_intensity, float param_list[], float * c_avg){
// 	//ASSUMPTION : enough memory is already is already allocated for c_avg
// 	for(int i = 0; i < nbr; i++){
// 		c_avg[i] = concentration(robot_position[i], wind_intensity, param_list);
// 	}
// }

float GaussianPlumeModel2D::approx_concentration(Position robot_position, float wind_intensity)
{
	return concentration(robot_position, wind_intensity, expected_value);
}

void GaussianPlumeModel2D::print_expected_val()
{
	// printf("Expected value: Q=%f @(%f %f) sigma y: %f %f  \n",
	// 	 expected_value[2], expected_value[0], expected_value[1], expected_value[3], expected_value[4]);
}

void GaussianPlumeModel2D::print_max_a_post_val()
{
	// printf("MaxApost value: Q=%f @(%f %f) sigma y: %f %f  \n",
	// 	 max_a_post_value[2], max_a_post_value[0], max_a_post_value[1], max_a_post_value[3], max_a_post_value[4]);
}

void GaussianPlumeModel2D::calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf) {}
void GaussianPlumeModel2D::calculate_weighted_gas_con_mean_map(float *gas_con_variance_map, float *pdf) {}
void GaussianPlumeModel2D::calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf) {}
void GaussianPlumeModel2D::log_gas_concentration_map(char *filename, char writing_mode) {}
void GaussianPlumeModel2D::initialize_potential_gas_maps() {}
void GaussianPlumeModel2D::retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int source_pos_idx, vector<float> &c_avg)
{
	float sx = 0.0f;
	float sy = 0.0f;
	index_to_position_2D(source_pos_idx, &sx, &sy);
	float q = (expected_value[2] > 0.0f) ? expected_value[2] : (Q_min + Q_max) * 0.5f;
	float sigma_a = (expected_value[3] > 0.0f) ? expected_value[3] : (a_y_min + a_y_max) * 0.5f;
	float sigma_b = (expected_value[4] > 0.0f) ? expected_value[4] : (b_y_min + b_y_max) * 0.5f;
	float params[5] = {sx, sy, q, sigma_a, sigma_b};
	float wind = (wind_intensity > 1e-6f) ? wind_intensity : 1e-6f;
	for (int i = 0; i < nbr; ++i)
	{
		float x = 0.0f;
		float y = 0.0f;
		indices_to_position_2D(idx[i], idy[i], &x, &y);
		c_avg[i] = concentration(Position(x, y, 0), wind, params);
	}
}
void GaussianPlumeModel2D::update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
																		const vector<float> &meas_y,
																		const vector<float> &wind_meas_u,
																		const vector<float> &wind_meas_v) {}
void GaussianPlumeModel2D::log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode) {}
float GaussianPlumeModel2D::get_wind_estimation_confidence(int source_pos_idx) { return static_cast<float>(INT_MAX); }
/********************************************************/
//					DeepCFDPlumeModel2D 				//
/********************************************************/

// constructor : default values
DeepCFDPlumeModel2D::DeepCFDPlumeModel2D()
{
	// number of parameters to estimate
	nbr_param = 2;

	// allocate memory for the parameters
	param_list = new parameter[nbr_param];

	// hard-coded parameters {name, lower_limit, upper_limit, resolution, nbins, estimated, max_a_post}
	// source_x
	param_list[0] = parameter{"source_x", source_x_min, source_x_max, x_resol, x_nbins, 0, 0};
	// source_y
	param_list[1] = parameter{"source_y", source_y_min, source_y_max, y_resol, y_nbins, 0, 0};

	// initialize expected and max a posteriori values to zero
	expected_value = new float[nbr_param];
	max_a_post_value = new float[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	// MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new float[nbr_param];
	covar_sig_ii[0] = 2;   // x
	covar_sig_ii[1] = 0.1; // y

	initialize_potential_gas_maps();
}

DeepCFDPlumeModel2D::DeepCFDPlumeModel2D(const DeepCFDPlumeModel2D &another_model)
{
	nbr_param = another_model.nbr_param;

	param_list = new parameter[nbr_param];
	expected_value = new float[nbr_param];
	max_a_post_value = new float[nbr_param];
	covar_sig_ii = new float[nbr_param];

	for (int i = 0; i < nbr_param; i++)
	{
		param_list[i] = another_model.param_list[i];
		expected_value[i] = another_model.expected_value[i];
		max_a_post_value[i] = another_model.max_a_post_value[i];
		covar_sig_ii[i] = another_model.covar_sig_ii[i];
	}
}

// destructor
DeepCFDPlumeModel2D::~DeepCFDPlumeModel2D()
{
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

void DeepCFDPlumeModel2D::initialize_potential_gas_maps()
{
	// initialize the dispersion map list
	load_gas_concentration_map_list_from_file("gas_concentration_maps.csv");

	// initialize the cell measurements list
	initialize_cell_measurement_list();
}

int DeepCFDPlumeModel2D::get_source_position_params_nbr()
{
	return 2;
}

void DeepCFDPlumeModel2D::get_source_position_params(parameter *source_position_param_list)
{
	source_position_param_list[0] = param_list[0]; // x
	source_position_param_list[1] = param_list[1]; // y
}

// returns concentration for ONE set of parameters and ONE position
float DeepCFDPlumeModel2D::concentration(Position robot_position, float wind_intensity, float param_list[])
{
	float sx = param_list[0];
	float sy = param_list[1];
	int r_idx_x[1];
	int r_idx_y[1];
	r_idx_x[0] = robot_position.x_idx;
	r_idx_y[0] = robot_position.y_idx;
	int source_pos_idx = indices_to_index_2D(sx, sy);
	vector<float> c_avg(1);
    retrieve_hist_measurements(r_idx_x, r_idx_y, 1, wind_intensity , source_pos_idx, c_avg);
	return c_avg[0];
}

// returns a list of concentrations for ONE set of parameters and A LIST of positions
void DeepCFDPlumeModel2D::concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list[], vector<float> &c_avg, vector<float> &c_std)
{
	// nbr here stands for the number of gathered measurements instead of the number of robots

	float sx = param_list[0];
	float sy = param_list[1];
	int r_idx_x[nbr];
	int r_idx_y[nbr];
	int source_pos_idx = indices_to_index_2D(sx, sy);

	for (int i = 0; i < nbr; i++)
	{
		r_idx_x[i] = robot_position[i].x_idx;
		r_idx_y[i] = robot_position[i].y_idx;
	}

	retrieve_hist_measurements(r_idx_x, r_idx_y, nbr, wind_intensity, source_pos_idx, c_avg);
}

void DeepCFDPlumeModel2D::retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int soure_pos_idx, vector<float> &c_avg)
{
	if (soure_pos_idx < 0 || soure_pos_idx >= static_cast<int>(gas_con_map_list.size()))
	{
		std::fill(c_avg.begin(), c_avg.end(), 0.0f);
		return;
	}

	for (int i = 0; i < nbr; i++)
	{
		int ix = std::max(0, std::min(idx[i], x_nbins - 1));
		int iy = std::max(0, std::min(idy[i], y_nbins - 1));
		c_avg[i] = gas_con_map_list[soure_pos_idx].gasConcentrationMap[ix][iy];
	}
}


//load the gas concentration map from the file
void DeepCFDPlumeModel2D::load_gas_concentration_map_list_from_file(const char *filename)
{
    gas_con_map_list.clear();
    FILE *fp = fopen(filename, "r");
    if (!fp)
    {
        throw std::runtime_error(std::string("Failed to open DDPM gas concentration map file: ") + filename);
    }

    char line[10000]; // adjust if needed
    while (fgets(line, sizeof(line), fp))
    {
        std::stringstream ss(line);
        std::string val;
        float sx, sy;

        // First two values are the source x and y
        std::getline(ss, val, ',');
        sx = std::stof(val);
        std::getline(ss, val, ',');
        sy = std::stof(val);

        GasDispersionMap gas_map(sx, sy); // make sure constructor supports this

        for (int i = 0; i < x_nbins; i++)
        {
            for (int j = 0; j < y_nbins; j++)
            {
                if (!std::getline(ss, val, ',')) break;
                float con = std::stof(val);
                gas_map.gasConcentrationMap[i][j] = con;
            }
        }

        gas_con_map_list.push_back(gas_map);
    }

    fclose(fp);
    if (gas_con_map_list.empty())
        throw std::runtime_error(std::string("DDPM gas concentration map file is empty or invalid: ") + filename);
}


// TODO - possible improvement: only store the variance of the gas concentration map
void DeepCFDPlumeModel2D::initialize_cell_measurement_list()
{
	// initialize cell meausurements list
	for (int j = 0; j < y_nbins; j++)
	{
		for (int i = 0; i < x_nbins; i++)
		{
			CellMeasurements cell_measurements(i, j);
			cell_lists.push_back(cell_measurements);
		}
	}
	for (auto &cell : cell_lists)
	{
		for (auto &gas_map : gas_con_map_list)
		{
			float con = gas_map.get_concentration(cell.idx_x, cell.idx_y);
			cell.add_measurement(con);
			cell.add_gas_hit(con);
		}
	}
}

void DeepCFDPlumeModel2D::calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf)
{
	for (size_t i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_con_variance_map[i] = cell_lists[i].meas_std;
	}
}

void DeepCFDPlumeModel2D::calculate_weighted_gas_con_mean_map(float *gas_con_mean_map, float *pdf)
{
	for (size_t i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_con_mean_map[i] = cell_lists[i].meas_mean;
	}
}

void DeepCFDPlumeModel2D::calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf)
{
	for (size_t i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_hit_variance_map[i] = cell_lists[i].hit_std;
	}
}


float DeepCFDPlumeModel2D::approx_concentration(Position robot_position, float wind_intensity)
{
	return concentration(robot_position, wind_intensity, expected_value);
}

void DeepCFDPlumeModel2D::print_expected_val()
{
	// printf("Expected value: @(%f %f)\n",
	// 	 expected_value[0], expected_value[1]);
}

void DeepCFDPlumeModel2D::print_max_a_post_val()
{
	// printf("MaxApost value: @(%f %f)\n",
	// 	 max_a_post_value[0], max_a_post_value[1]);
}

void DeepCFDPlumeModel2D::log_gas_concentration_map(char *filename, char writing_mode)
{
	for (auto gas_map : gas_con_map_list)
	{
		gas_map.log_map(filename, writing_mode);
	}
}

void DeepCFDPlumeModel2D::update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
																	   const vector<float> &meas_y,
																	   const vector<float> &wind_meas_u,
																	   const vector<float> &wind_meas_v) {}

void DeepCFDPlumeModel2D::log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode) {}

float DeepCFDPlumeModel2D::get_wind_estimation_confidence(int source_pos_idx)
{
	return static_cast<float>(INT_MAX);
}

/*************************************************************************************************/
//	ADEPlumeModel2D (2D steady-state advection-diffusion model solved by FDM)
/*************************************************************************************************/

AdeFDMPlumeModel2D::AdeFDMPlumeModel2D()
{
	nbr_param = 2;
	param_list = new parameter[nbr_param];
	param_list[0] = parameter{"source_x", source_x_min, source_x_max, x_resol, x_nbins, 0, 0};
	param_list[1] = parameter{"source_y", source_y_min, source_y_max, y_resol, y_nbins, 0, 0};

	expected_value = new float[nbr_param];
	max_a_post_value = new float[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	covar_sig_ii = new float[nbr_param];
	covar_sig_ii[0] = 2.0f;
	covar_sig_ii[1] = 0.1f;

	wind_estimator_ = nullptr;
	fact_ok_ = false;
	u_x_ = -0.75f;

	set_parameters(50, 20, simulation_parameters.diff_coefficient, 1500.0f);
	set_u_field_uniform(u_x_);

	if (simulation_parameters.windInfo == WIND_OFFLINE)
	{
		if (!set_u_field_from_csv("/home/root/wanting/wind_map_2D.csv", 64, 64))
			set_u_field_from_csv("wind_map_2D.csv", 64, 64);
	}

	if (simulation_parameters.windInfo == WIND_ONLINE)
	{
		if (simulation_parameters.windEstimationMethod == BASEFLOW_CORRECTION)
		{
			wind_estimator_ = new BaseflowCorrectionWindEstimator(source_x_min, source_x_max,
																 source_y_min, source_y_max,
																 Nx_, Ny_, obstacles, nbr_obs, 1.0f, 100.0f);
		}
		else
		{
			wind_estimator_ = new GaussianWindEstimator(source_x_min, source_x_max,
													   source_y_min, source_y_max,
													   Nx_, Ny_, obstacles, nbr_obs, 0.30f, 1.0f, 0.5f);
		}
	}
	assemble_and_factorize();
	initialize_potential_gas_maps();
}

AdeFDMPlumeModel2D::AdeFDMPlumeModel2D(const AdeFDMPlumeModel2D &another_model)
{
	nbr_param = another_model.nbr_param;
	param_list = new parameter[nbr_param];
	expected_value = new float[nbr_param];
	max_a_post_value = new float[nbr_param];
	covar_sig_ii = new float[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		param_list[i] = another_model.param_list[i];
		expected_value[i] = another_model.expected_value[i];
		max_a_post_value[i] = another_model.max_a_post_value[i];
		covar_sig_ii[i] = another_model.covar_sig_ii[i];
	}
	Nx_ = another_model.Nx_;
	Ny_ = another_model.Ny_;
	N_ = another_model.N_;
	dx_ = another_model.dx_;
	dy_ = another_model.dy_;
	D_ = another_model.D_;
	Q_ = another_model.Q_;
	u_x_ = another_model.u_x_;
	source_strength_ = another_model.source_strength_;
	u_ = another_model.u_;
	v_ = another_model.v_;
	w_ = another_model.w_;
	gas_con_map_list = another_model.gas_con_map_list;
	cell_lists = another_model.cell_lists;
	wind_estimator_ = nullptr;
	fact_ok_ = false;
	assemble_and_factorize();
}

AdeFDMPlumeModel2D::~AdeFDMPlumeModel2D()
{
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
	delete wind_estimator_;
}

void AdeFDMPlumeModel2D::set_parameters(int Nx, int Ny, float D, float Q)
{
	Nx_ = Nx;
	Ny_ = Ny;
	N_ = Nx_ * Ny_;
	D_ = D;
	Q_ = Q;
	dx_ = (param_list[0].upper_limit - param_list[0].lower_limit) / static_cast<float>(Nx_ - 1);
	dy_ = (param_list[1].upper_limit - param_list[1].lower_limit) / static_cast<float>(Ny_ - 1);
	source_strength_ = Q_ / (dx_ * dy_);
	u_.assign(N_, 0.0f);
	v_.assign(N_, 0.0f);
	w_.assign(N_, 0.0f);
}

void AdeFDMPlumeModel2D::initialize_potential_gas_maps()
{
	gas_con_map_list.clear();
	gas_con_map_list.reserve(MAP_SIZE);
	for (int j = 0; j < param_list[1].nbins; j++)
	{
		for (int i = 0; i < param_list[0].nbins; i++)
		{
			GasDispersionMap gas_map(i, j);
			gas_con_map_list.push_back(gas_map);
		}
	}
	calculate_gas_concentration_maps_batched();
	rebuild_cell_measurements();
}

void AdeFDMPlumeModel2D::rebuild_cell_measurements()
{
	cell_lists.clear();
	initialize_cell_measurement_list();
}

void AdeFDMPlumeModel2D::initialize_cell_measurement_list()
{
	for (int j = 0; j < y_nbins; j++)
	{
		for (int i = 0; i < x_nbins; i++)
		{
			CellMeasurements cell_measurements(i, j);
			cell_lists.push_back(cell_measurements);
		}
	}
	for (auto &cell : cell_lists)
	{
		for (auto &gas_map : gas_con_map_list)
		{
			float con = gas_map.get_concentration(cell.idx_x, cell.idx_y);
			cell.add_measurement(con);
			cell.add_gas_hit(con);
		}
	}
}

void AdeFDMPlumeModel2D::retrieve_hist_measurements(int idx[], int idy[], int nbr, float wind_intensity, int source_pos_idx, vector<float> &c_avg)
{
	if (source_pos_idx < 0 || source_pos_idx >= static_cast<int>(gas_con_map_list.size()))
	{
		std::fill(c_avg.begin(), c_avg.end(), 0.0f);
		return;
	}

	for (int i = 0; i < nbr; i++)
	{
		int ix = std::max(0, std::min(idx[i], x_nbins - 1));
		int iy = std::max(0, std::min(idy[i], y_nbins - 1));
		c_avg[i] = gas_con_map_list[source_pos_idx].gasConcentrationMap[ix][iy];
	}
}

void AdeFDMPlumeModel2D::set_u_field_uniform(float u)
{
	for (int j = 0; j < Ny_; ++j)
	{
		for (int i = 0; i < Nx_; ++i)
		{
			int index = grid_idx(i, j);
			u_[index] = u;
			v_[index] = 0.0f;
			w_[index] = 1.0f;
		}
	}
}

bool AdeFDMPlumeModel2D::load_csv(const std::string &filename,
								  std::vector<float> &xs,
								  std::vector<float> &ys,
								  std::vector<float> &us,
								  std::vector<float> &vs)
{
	std::ifstream file(filename.c_str());
	if (!file.is_open())
		return false;

	std::string line;
	bool first = true;
	while (std::getline(file, line))
	{
		if (first)
		{
			first = false;
			continue;
		}
		std::stringstream ss(line);
		std::string cell;
		std::vector<std::string> tokens;
		while (std::getline(ss, cell, ','))
			tokens.push_back(cell);
		if (tokens.size() < 4)
			continue;

		xs.push_back(static_cast<float>(std::atof(tokens[0].c_str())));
		ys.push_back(static_cast<float>(std::atof(tokens[1].c_str())));
		us.push_back(static_cast<float>(std::atof(tokens[2].c_str())));
		vs.push_back(static_cast<float>(std::atof(tokens[3].c_str())));
	}
	return !xs.empty();
}

float AdeFDMPlumeModel2D::bilinear(float x, float y,
								   const std::vector<float> &X,
								   const std::vector<float> &Y,
								   const std::vector<float> &F,
								   int Nx, int Ny)
{
	std::vector<float>::const_iterator itx = std::lower_bound(X.begin(), X.end(), x);
	std::vector<float>::const_iterator ity = std::lower_bound(Y.begin(), Y.end(), y);

	int i1 = std::max(0, static_cast<int>(itx - X.begin()) - 1);
	int j1 = std::max(0, static_cast<int>(ity - Y.begin()) - 1);
	int i2 = std::min(Nx - 1, i1 + 1);
	int j2 = std::min(Ny - 1, j1 + 1);

	float x1 = X[i1];
	float x2 = X[i2];
	float y1 = Y[j1];
	float y2 = Y[j2];
	float q11 = F[j1 + i1 * Ny];
	float q21 = F[j1 + i2 * Ny];
	float q12 = F[j2 + i1 * Ny];
	float q22 = F[j2 + i2 * Ny];
	float tx = (x - x1) / (x2 - x1 + 1e-12f);
	float ty = (y - y1) / (y2 - y1 + 1e-12f);

	return (1.0f - tx) * (1.0f - ty) * q11 +
		   tx * (1.0f - ty) * q21 +
		   (1.0f - tx) * ty * q12 +
		   tx * ty * q22;
}

bool AdeFDMPlumeModel2D::set_u_field_from_csv(const std::string &filename, int Nx_src, int Ny_src)
{
	std::vector<float> xs, ys, us, vs;
	if (!load_csv(filename, xs, ys, us, vs))
		return false;
	if (static_cast<int>(xs.size()) < Nx_src * Ny_src ||
	    static_cast<int>(ys.size()) < Nx_src * Ny_src ||
	    static_cast<int>(us.size()) < Nx_src * Ny_src ||
	    static_cast<int>(vs.size()) < Nx_src * Ny_src)
		return false;

	std::vector<float> Xs;
	std::vector<float> Ys;
	Xs.reserve(Nx_src);
	Ys.reserve(Ny_src);
	for (int i = 0; i < Nx_src; ++i)
		Xs.push_back(xs[i * Ny_src]);
	for (int j = 0; j < Ny_src; ++j)
		Ys.push_back(ys[j]);

	for (int j = 0; j < Ny_; ++j)
	{
		for (int i = 0; i < Nx_; ++i)
		{
			float x = param_list[0].lower_limit + dx_ * i;
			float y = param_list[1].lower_limit + dy_ * j;
			int index = grid_idx(i, j);
			u_[index] = bilinear(x, y, Xs, Ys, us, Nx_src, Ny_src);
			v_[index] = bilinear(x, y, Xs, Ys, vs, Nx_src, Ny_src);
			w_[index] = 1.0f;
		}
	}
	return true;
}

void AdeFDMPlumeModel2D::assemble_and_factorize()
{
	std::vector<Triplet> triplets;
	triplets.reserve(5 * N_);

	const float dx2 = dx_ * dx_;
	const float dy2 = dy_ * dy_;
	const float a_s_base = D_ / dy2;
	const float a_n_base = D_ / dy2;
	const float a_w_base = D_ / dx2;
	const float a_e_base = D_ / dx2;

	for (int j = 0; j < Ny_; ++j)
	{
		for (int i = 0; i < Nx_; ++i)
		{
			int k = grid_idx(i, j);
			if (i == Nx_ - 1 || i == 0 || j == 0 || j == Ny_ - 1)
			{
				triplets.push_back(Triplet(k, k, 1.0f));
				continue;
			}

			const float u = u_[k];
			const float v = v_[k];
			float a_s = a_s_base;
			float a_n = a_n_base;
			float a_w = a_w_base;
			float a_e = a_e_base;

			if (u > 0.0f)
				a_w += u / dx_;
			else
				a_e += -u / dx_;

			if (v > 0.0f)
				a_s += v / dy_;
			else
				a_n += -v / dy_;

			const float a_p = -(a_w + a_e + a_s + a_n);
			triplets.push_back(Triplet(k, grid_idx(i - 1, j), a_w));
			triplets.push_back(Triplet(k, grid_idx(i + 1, j), a_e));
			triplets.push_back(Triplet(k, grid_idx(i, j - 1), a_s));
			triplets.push_back(Triplet(k, grid_idx(i, j + 1), a_n));
			triplets.push_back(Triplet(k, k, a_p));
		}
	}

	A_.resize(N_, N_);
	A_.setZero();
	A_.setFromTriplets(triplets.begin(), triplets.end());
	A_.makeCompressed();
	solver_.analyzePattern(A_);
	solver_.factorize(A_);
	if (solver_.info() != Eigen::Success)
	{
		fact_ok_ = false;
		throw std::runtime_error("ADE FDM SuperLU factorization failed");
	}
	fact_ok_ = true;
}

std::vector<float> AdeFDMPlumeModel2D::solve_point_source(float sx, float sy) const
{
	if (!fact_ok_)
		throw std::runtime_error("ADE FDM matrix must be factorized before solving");

	int si = 0;
	int sj = 0;
	world_pos_to_grid_indices(sx, sy, si, sj);
	int k_src = grid_indices_to_idx(si, sj);
	Eigen::VectorXf b = Eigen::VectorXf::Zero(N_);
	b(k_src) = -source_strength_;
	if (si == 0 || si == Nx_ - 1 || sj == 0 || sj == Ny_ - 1)
		b(k_src) = 0.0f;

	Eigen::VectorXf x = solver_.solve(b);
	if (solver_.info() != Eigen::Success)
		throw std::runtime_error("ADE FDM linear solve failed");

	std::vector<float> C(N_);
	for (int k = 0; k < N_; ++k)
		C[k] = std::max(0.0f, x(k));
	return C;
}

void AdeFDMPlumeModel2D::calculate_gas_concentration_map(GasDispersionMap &gas_map)
{
	std::vector<float> C = solve_point_source(gas_map.sx, gas_map.sy);
	for (int map_i = 0; map_i < x_nbins; map_i++)
	{
		for (int map_j = 0; map_j < y_nbins; map_j++)
		{
			float px = 0.0f;
			float py = 0.0f;
			indices_to_position_2D(map_i, map_j, &px, &py);
			int grid_i = 0;
			int grid_j = 0;
			world_pos_to_grid_indices(px, py, grid_i, grid_j);
			gas_map.gasConcentrationMap[map_i][map_j] = C[grid_indices_to_idx(grid_i, grid_j)];
		}
	}
}

void AdeFDMPlumeModel2D::calculate_gas_concentration_maps_batched()
{
	if (!fact_ok_)
		throw std::runtime_error("ADE FDM matrix must be factorized before solving");

	std::vector<int> map_cell_to_grid(MAP_SIZE, 0);
	for (int map_j = 0; map_j < y_nbins; map_j++)
	{
		for (int map_i = 0; map_i < x_nbins; map_i++)
		{
			float px = 0.0f;
			float py = 0.0f;
			indices_to_position_2D(map_i, map_j, &px, &py);
			int grid_i = 0;
			int grid_j = 0;
			world_pos_to_grid_indices(px, py, grid_i, grid_j);
			map_cell_to_grid[map_i + map_j * x_nbins] = grid_indices_to_idx(grid_i, grid_j);
		}
	}

	const int batch_size = 32;
	const int map_count = static_cast<int>(gas_con_map_list.size());
	for (int batch_begin = 0; batch_begin < map_count; batch_begin += batch_size)
	{
		int batch_cols = std::min(batch_size, map_count - batch_begin);
		Eigen::MatrixXf B = Eigen::MatrixXf::Zero(N_, batch_cols);

		for (int col = 0; col < batch_cols; col++)
		{
			const GasDispersionMap &gas_map = gas_con_map_list[batch_begin + col];
			int si = 0;
			int sj = 0;
			world_pos_to_grid_indices(gas_map.sx, gas_map.sy, si, sj);
			if (si == 0 || si == Nx_ - 1 || sj == 0 || sj == Ny_ - 1)
				continue;
			B(grid_indices_to_idx(si, sj), col) = -source_strength_;
		}

		Eigen::MatrixXf X = solver_.solve(B);
		if (solver_.info() != Eigen::Success)
			throw std::runtime_error("ADE FDM batched linear solve failed");

		for (int col = 0; col < batch_cols; col++)
		{
			GasDispersionMap &gas_map = gas_con_map_list[batch_begin + col];
			for (int map_j = 0; map_j < y_nbins; map_j++)
			{
				for (int map_i = 0; map_i < x_nbins; map_i++)
				{
					int map_idx = map_i + map_j * x_nbins;
					gas_map.gasConcentrationMap[map_i][map_j] =
						std::max(0.0f, X(map_cell_to_grid[map_idx], col));
				}
			}
		}
	}
}

void AdeFDMPlumeModel2D::calculate_weighted_gas_con_variance_map(float *gas_con_variance_map, float *pdf)
{
	for (size_t i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_con_variance_map[i] = cell_lists[i].meas_std;
	}
}

void AdeFDMPlumeModel2D::calculate_weighted_gas_con_mean_map(float *gas_con_mean_map, float *pdf)
{
	for (size_t i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_con_mean_map[i] = cell_lists[i].meas_mean;
	}
}

void AdeFDMPlumeModel2D::calculate_weighted_gas_hit_variance_map(float *gas_hit_variance_map, float *pdf)
{
	for (size_t i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_hit_variance_map[i] = cell_lists[i].hit_std;
	}
}

float AdeFDMPlumeModel2D::concentration(Position robot_position, float wind_intensity, float param_list_value[])
{
	int r_idx_x[1];
	int r_idx_y[1];
	position_to_indices_2D(robot_position.x, robot_position.y, &r_idx_x[0], &r_idx_y[0]);
	int source_pos_idx = position_to_index_2D(param_list_value[0], param_list_value[1]);
	vector<float> c_avg(1, 0.0f);
	retrieve_hist_measurements(r_idx_x, r_idx_y, 1, wind_intensity, source_pos_idx, c_avg);
	return c_avg[0];
}

void AdeFDMPlumeModel2D::concentration(vector<Position> robot_position, int nbr, float wind_intensity, float param_list_value[], vector<float> &c_avg, vector<float> &c_std)
{
	int source_pos_idx = position_to_index_2D(param_list_value[0], param_list_value[1]);
	std::vector<int> r_idx_x(nbr);
	std::vector<int> r_idx_y(nbr);
	for (int i = 0; i < nbr; i++)
		position_to_indices_2D(robot_position[i].x, robot_position[i].y, &r_idx_x[i], &r_idx_y[i]);
	retrieve_hist_measurements(r_idx_x.data(), r_idx_y.data(), nbr, wind_intensity, source_pos_idx, c_avg);
	std::fill(c_std.begin(), c_std.end(), 0.0f);
}

float AdeFDMPlumeModel2D::approx_concentration(Position robot_position, float wind_intensity)
{
	return concentration(robot_position, wind_intensity, expected_value);
}

void AdeFDMPlumeModel2D::print_expected_val()
{
	printf("Expected value: @(%f %f)\n", expected_value[0], expected_value[1]);
}

void AdeFDMPlumeModel2D::print_max_a_post_val()
{
	printf("MaxApost value: @(%f %f)\n", max_a_post_value[0], max_a_post_value[1]);
}

int AdeFDMPlumeModel2D::get_source_position_params_nbr()
{
	return 2;
}

void AdeFDMPlumeModel2D::get_source_position_params(parameter *source_position_param_list)
{
	source_position_param_list[0] = param_list[0];
	source_position_param_list[1] = param_list[1];
}

void AdeFDMPlumeModel2D::log_gas_concentration_map(char *filename, char writing_mode)
{
	for (auto gas_map : gas_con_map_list)
		gas_map.log_map(filename, writing_mode);
}

void AdeFDMPlumeModel2D::update_wind_field_and_gas_concentration_map(const vector<float> &meas_x,
																	  const vector<float> &meas_y,
																	  const vector<float> &wind_meas_u,
																	  const vector<float> &wind_meas_v)
{
	if (!wind_estimator_ || meas_x.empty())
		return;

	auto start = std::chrono::high_resolution_clock::now();
	wind_estimator_->estimateField(meas_x, meas_y, wind_meas_u, wind_meas_v);
	const std::vector<std::vector<float>> &U_field = wind_estimator_->getU();
	const std::vector<std::vector<float>> &V_field = wind_estimator_->getV();
	const std::vector<std::vector<float>> &W_field = wind_estimator_->getW();
	for (int i = 0; i < Nx_; ++i)
	{
		for (int j = 0; j < Ny_; ++j)
		{
			int index = grid_idx(i, j);
			u_[index] = U_field[j][i];
			v_[index] = V_field[j][i];
			w_[index] = W_field[j][i];
		}
	}
	auto end = std::chrono::high_resolution_clock::now();
	printf("Wind field updated in %ld ms.\n",
	       static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()));

	auto start_assemble = std::chrono::high_resolution_clock::now();
	assemble_and_factorize();
	auto end_assemble = std::chrono::high_resolution_clock::now();
	printf("ADE FDM matrix re-assembled and factorized in %ld ms.\n",
	       static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(end_assemble - start_assemble).count()));

	auto start_gas_map = std::chrono::high_resolution_clock::now();
	initialize_potential_gas_maps();
	auto end_gas_map = std::chrono::high_resolution_clock::now();
	printf("Gas concentration map list recalculated in %ld ms.\n",
	       static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(end_gas_map - start_gas_map).count()));
}

void AdeFDMPlumeModel2D::log_wind_field(char *filename_u, char *filename_v, char *filename_w, char writing_mode)
{
	const char mode[2] = {writing_mode, '\0'};
	FILE *file_u = fopen(filename_u, mode);
	if (file_u)
	{
		for (int i = 0; i < N_ - 1; i++)
			fprintf(file_u, "%f,", u_[i]);
		fprintf(file_u, "%f\n", u_[N_ - 1]);
		fclose(file_u);
	}

	FILE *file_v = fopen(filename_v, mode);
	if (file_v)
	{
		for (int i = 0; i < N_ - 1; i++)
			fprintf(file_v, "%f,", v_[i]);
		fprintf(file_v, "%f\n", v_[N_ - 1]);
		fclose(file_v);
	}

	FILE *file_w = fopen(filename_w, mode);
	if (file_w)
	{
		for (int i = 0; i < N_ - 1; i++)
			fprintf(file_w, "%f,", w_[i]);
		fprintf(file_w, "%f\n", w_[N_ - 1]);
		fclose(file_w);
	}
}

float AdeFDMPlumeModel2D::get_wind_estimation_confidence(int source_pos_idx)
{
	float source_x = 0.0f;
	float source_y = 0.0f;
	index_to_position_2D(source_pos_idx, &source_x, &source_y);
	int grid_i = 0;
	int grid_j = 0;
	world_pos_to_grid_indices(source_x, source_y, grid_i, grid_j);
	return w_[grid_indices_to_idx(grid_i, grid_j)];
}

void AdeFDMPlumeModel2D::world_pos_to_grid_indices(float x, float y, int &i, int &j) const
{
	i = clamp_to_grid_x(world_to_i(x));
	j = clamp_to_grid_y(world_to_j(y));
}

int AdeFDMPlumeModel2D::clamp_to_grid_x(int i) const
{
	return std::max(0, std::min(i, Nx_ - 1));
}

int AdeFDMPlumeModel2D::clamp_to_grid_y(int j) const
{
	return std::max(0, std::min(j, Ny_ - 1));
}

int AdeFDMPlumeModel2D::grid_idx(int i, int j) const
{
	if (i < 0 || i >= Nx_ || j < 0 || j >= Ny_)
		return -1;
	return grid_indices_to_idx(i, j);
}

int AdeFDMPlumeModel2D::world_to_i(float x) const
{
	return static_cast<int>(std::floor((x - source_x_min) / dx_));
}

int AdeFDMPlumeModel2D::world_to_j(float y) const
{
	return static_cast<int>(std::floor((y - source_y_min) / dy_));
}
