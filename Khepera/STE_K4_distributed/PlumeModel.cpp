#include <math.h>
#include <cmath>
#include <chrono>
#include <iostream>
// #include "nvwa/debug_new.h"
#include "PlumeModel.h"
#include "CellMeasurements.h"
#include "constant_and_helper.h"
#include "GasDispersionMap.h"
// #define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// #include <numpy/arrayobject.h>
#include <stdexcept>

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
	return c_avg[1];
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
	for (int i = 0; i < nbr; i++)
	{
		c_avg[i] = gas_con_map_list[soure_pos_idx].gasConcentrationMap[idx[i]][idy[i]];
	}
}


//load the gas concentration map from the file
void DeepCFDPlumeModel2D::load_gas_concentration_map_list_from_file(const char *filename)
{
    gas_con_map_list.clear();
    FILE *fp = fopen(filename, "r");
    if (!fp)
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
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