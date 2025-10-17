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
// 	expected_value = new double[nbr_param];
// 	max_a_post_value = new double[nbr_param];
// 	covar_sig_ii = new double[nbr_param];

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

void PlumeModel::random_params(double *rnd_params)
{
	for (int i = 0; i < nbr_param; i++)
		rnd_params[i] = ((double)rand() / (double)RAND_MAX) * (param_list[i].upper_limit - param_list[i].lower_limit) + param_list[i].lower_limit;
}

void PlumeModel::get_expected_value(double *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		container[i] = expected_value[i];
	}
}

void PlumeModel::get_max_a_post_value(double *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		container[i] = max_a_post_value[i];
	}
}

void PlumeModel::set_expected_value(double *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		expected_value[i] = container[i];
	}
}

void PlumeModel::set_max_a_post_value(double *container)
{
	int i = 0;
	for (i = 0; i < nbr_param; i++)
	{
		max_a_post_value[i] = container[i];
	}
}

int PlumeModel::get_source_position_params_nbr()
{
	return 2;
}

void PlumeModel::get_source_position_params(parameter *source_position_param_list)
{
	source_position_param_list[0] = param_list[0]; // x
	source_position_param_list[1] = param_list[1]; // y
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
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	// MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new double[nbr_param];
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

// returns concentration for a set of parameters and a position
double GaussianPlumeModel2D::concentration(Position robot_position, double wind_intensity, double param_list[])
{
	// ASSUMPTION : X is towards the wind speed
	// notation simplification (axis rotation, hence the signs)
	double x = param_list[0] - robot_position.x;
	double y = param_list[1] - robot_position.y;
	double z = 0; // robot_position[2] - parameters.source_position[2];

	// verify if the point is not before the source
	if (x < 0)
		return 0.0;
	else
	{
		// std on y and z
		double sigma_y = param_list[3] * x + param_list[4];
		double sigma_z = 1; // parameters.a_z * x + parameters.b_z;

		// model : C(x,y,z) = (Q/2pi.u.sigmaY.sigmaZ) * exp (-y^2/2sigmaY^2 - z^2/2sigmaZ^2)
		double model_value = (param_list[2] / (2 * PI * wind_intensity * sigma_y * sigma_z)) * exp(-((y * y) / (2 * sigma_y * sigma_y)) - ((z * z) / (2 * sigma_z * sigma_z)));

		return model_value;
	}
}

// returns a list of concentrations for ONE set of parameters and A LIST of positions
void GaussianPlumeModel2D::concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_list[], vector<float> &c_avg, vector<float> &c_std)
{
	// ASSUMPTION : enough memory is already is already allocated for c_avg
	for (int i = 0; i < nbr; i++)
	{
		c_avg[i] = concentration(robot_position[i], wind_intensity, param_list);
	}
}


double GaussianPlumeModel2D::approx_concentration(Position robot_position, double wind_intensity)
{
	return concentration(robot_position, wind_intensity, expected_value);
}

void GaussianPlumeModel2D::calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf) {}
void GaussianPlumeModel2D::calculate_weighted_gas_con_mean_map(double *gas_con_variance_map, double *pdf) {}
void GaussianPlumeModel2D::log_gas_concentration_map(char *filename, char writing_mode) {}
void GaussianPlumeModel2D::initialize_gas_map_and_information_matrix() {}
void GaussianPlumeModel2D::retrieve_hist_measurements(int idx[], int idy[], int nbr, double wind_intensity, int soure_pos_idx, vector<double> &c_avg) {}
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
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	// MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new double[nbr_param];
	covar_sig_ii[0] = 2;   // x
	covar_sig_ii[1] = 0.1; // y

	// initialize the trained neural network
	initialize_model_features();
}

DeepCFDPlumeModel2D::DeepCFDPlumeModel2D(const DeepCFDPlumeModel2D &another_model)
{
	nbr_param = another_model.nbr_param;

	param_list = new parameter[nbr_param];
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	covar_sig_ii = new double[nbr_param];

	for (int i = 0; i < nbr_param; i++)
	{
		param_list[i] = another_model.param_list[i];
		expected_value[i] = another_model.expected_value[i];
		max_a_post_value[i] = another_model.max_a_post_value[i];
		covar_sig_ii[i] = another_model.covar_sig_ii[i];
	}
	DeepCFD = another_model.DeepCFD.clone();
}

// destructor
DeepCFDPlumeModel2D::~DeepCFDPlumeModel2D()
{
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

void DeepCFDPlumeModel2D::initialize_gas_map_and_information_matrix()
{
	// initialize the dispersion map list
	initialize_gas_concentration_map_list();

	// initialize the cell measurements list
	initialize_cell_measurement_list();
}

void DeepCFDPlumeModel2D::initialize_model_features()
{
	// Load the number of maps
	int m;
	std::ifstream f("../../data/map_nbr.txt");
	f >> m;
	f.close();

	// Load the model
	// std::cout << "Loading model..." << std::endl;
	DeepCFD = torch::jit::load("DeepCFD_libtorch_model.pt", device);
	std::string mat_address = "../../../Results/test_maps/map" + std::to_string(m) + "/test_map" + std::to_string(m) + "_flowdef_sdf_sourcepos.mat";
	// convert string to char*
	// std::cout << mat_address << std::endl;
	char *mat_adr = new char[mat_address.length() + 1];
	strcpy(mat_adr, mat_address.c_str());
	mat_t *mat;
	matvar_t *r = NULL, *c = NULL, *flow_def = NULL, *sdf = NULL, *reverse_sd = NULL;
	mat = Mat_Open(mat_adr, MAT_ACC_RDONLY);
	if (mat == NULL)
	{
		std::cout << "Error opening MAT file" << std::endl;
		// return 1;
	}
	// std::cout << mat-> << std::endl;
	// std::cout << mat<< std::endl;
	flow_def = Mat_VarRead(mat, "flow_def");
	sdf = Mat_VarRead(mat, "sdf");
	reverse_sd = Mat_VarRead(mat, "source_pos");
	r = Mat_VarRead(mat, "X");
	c = Mat_VarRead(mat, "Y");
	if (flow_def == NULL || sdf == NULL || reverse_sd == NULL || r == NULL || c == NULL)
	{
		std::cout << "Error reading MAT file" << std::endl;
		// return 1;
	}
	// declare the tensor of dimension 1x3x64x64
	x = torch::zeros({1, 3, 64, 64}, torch::TensorOptions().dtype(torch::kDouble));

	// convert to torch tensor

	torch::Tensor flow_def_torch = torch::from_blob(flow_def->data, {(long long)flow_def->dims[0], (long long)flow_def->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
	torch::Tensor sdf_torch = torch::from_blob(sdf->data, {(long long)sdf->dims[0], (long long)sdf->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
	torch::Tensor reverse_sd_torch = torch::from_blob(reverse_sd->data, {(long long)reverse_sd->dims[0], (long long)reverse_sd->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
	x[0][0].slice(/*dim=*/0, /*start=*/0, /*end=*/flow_def_torch.size(0)).copy_(flow_def_torch);
	x[0][1].slice(/*dim=*/0, /*start=*/0, /*end=*/sdf_torch.size(0)).copy_(sdf_torch);
	x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/reverse_sd_torch.size(0)).copy_(reverse_sd_torch);
	r_torch = torch::from_blob(r->data, {(long long)r->dims[0], (long long)r->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
	c_torch = torch::from_blob(c->data, {(long long)c->dims[0], (long long)c->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
}

// void DeepCFDPlumeModel2D::random_params(double * rnd_params){
// 	for(int i = 0; i < nbr_param; i++)
// 		rnd_params[i] = ((double)rand() / (double)RAND_MAX) * (param_list[i].upper_limit - param_list[i].lower_limit) + param_list[i].lower_limit;
// }

// returns concentration for ONE set of parameters and ONE position
double DeepCFDPlumeModel2D::concentration(Position robot_position, double wind_intensity, double param_list[])
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	float sx = param_list[0];
	float sy = param_list[1];
	float px = robot_position.x;
	float py = robot_position.y;
	double c_avg = 0;

	// Calculate the revert distance to the estimated gas source
	x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/r_torch.size(0)) = 1 / torch::sqrt((sx - r_torch).pow(2) + (sy - c_torch).pow(2));
	std::vector<torch::jit::IValue> inputs;
	// transpose the tensor
	torch::Tensor x_trans = x.permute({0, 1, 3, 2});
	// NOTE: The input tensor must be a float tensor
	inputs.push_back(x_trans.to(torch::kFloat).to(device));
	// std::cout << x_trans[0][1] << std::endl;
	//  Execute the model and turn its output into a tensor.
	at::Tensor output = DeepCFD(inputs).toTensor();
	// std::cout << output << std::endl;

	int idx_x = round((px - min(r_torch[0]).item<float>()) / (r_torch[1][0] - r_torch[0][0]).item<float>());
	int idx_y = round((py - min(c_torch[0]).item<float>()) / (c_torch[0][1] - c_torch[0][0]).item<float>());
	// std::cout<< "idx_x is: "<<idx_x;
	// std::cout<< " idx_y is: "<<idx_y << std::endl;

	c_avg = output[0][0][idx_y][idx_x].item<float>();
	// std::cout << "The estimated concentration at (" << sx <<  ", " << sy  << "; " <<px << ", " << py << ") is " << output[0][0][idx_y][idx_x].item<float>() << std::endl;

	return c_avg;
}

// returns a list of concentrations for ONE set of parameters and A LIST of positions
void DeepCFDPlumeModel2D::concentration(vector<Position> robot_position, int nbr, double wind_intensity, double param_list[], vector<float> &c_avg, vector<float> &c_std)
{
	// nbr here stands for the number of gathered measurements instead of the number of robots

	float sx = param_list[0];
	float sy = param_list[1];
	float px[nbr];
	float py[nbr];

	for (int i = 0; i < nbr; i++)
	{
		px[i] = robot_position[i].x;
		py[i] = robot_position[i].y;
	}

	x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/r_torch.size(0)) = 1 / torch::sqrt((sx - r_torch).pow(2) + (sy - c_torch).pow(2));
	std::vector<torch::jit::IValue> inputs;
	// transpose the tensor
	torch::Tensor x_trans = x.permute({0, 1, 3, 2});
	// NOTE: The input tensor must be a float tensor
	inputs.push_back(x_trans.to(torch::kFloat).to(device));
	// std::cout << x_trans[0][1] << std::endl;
	//  Execute the model and turn its output into a tensor.
	at::Tensor output = DeepCFD(inputs).toTensor();
	// std::cout << output << std::endl;

	for (int i = 0; i < nbr; i++)
	{
		int idx_x = round((px[i] - min(r_torch[0]).item<float>()) / (r_torch[1][0] - r_torch[0][0]).item<float>());
		int idx_y = round((py[i] - min(c_torch[0]).item<float>()) / (c_torch[0][1] - c_torch[0][0]).item<float>());
		// std::cout << "idx_x is: "<<idx_x;
		// std::cout << " idx_y is: "<<idx_y << std::endl;

		c_avg[i] = output[0][0][idx_y][idx_x].item<float>();
		// printf("Expected concentration for source %d: %f\n", i, model_c_avg[i]);
		// printf("Precomputed Expected concentration for source %d: %f\n", i, model.gas_con_map_list[]);
		// std::cout << "The estimated concentration at (" << sx <<  ", " << sy  << "; " <<px[i] << ", " << py[i] << ") is " << output[0][0][idx_y][idx_x].item<float>() << std::endl;
	}
}

void DeepCFDPlumeModel2D::retrieve_hist_measurements(int idx[], int idy[], int nbr, double wind_intensity, int soure_pos_idx, vector<double> &c_avg)
{
	for (int i = 0; i < nbr; i++)
	{
		c_avg[i] = gas_con_map_list[soure_pos_idx].gasConcentrationMap[idx[i]][idy[i]];
	}
}

// 20x20 sized map
void DeepCFDPlumeModel2D::initialize_gas_concentration_map_list()
{
	// initialize gas dispersion list
	// calculate the gas concentration map for each possible source location
	// first scan y axis then x axis; the same order as the pdf
	for (int j = 0; j < y_nbins; j++)
	{
		for (int i = 0; i < x_nbins; i++)
		{
			GasDispersionMap gas_map(i, j);
			calculate_gas_concentration_map(gas_map);
			gas_con_map_list.push_back(gas_map);
		}
	}
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

void DeepCFDPlumeModel2D::calculate_weighted_gas_con_variance_map(double *gas_con_variance_map, double *pdf)
{
	for (int i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_con_variance_map[i] = cell_lists[i].meas_std;
	}
}

void DeepCFDPlumeModel2D::calculate_weighted_gas_con_mean_map(double *gas_con_mean_map, double *pdf)
{
	for (int i = 0; i < cell_lists.size(); i++)
	{
		cell_lists[i].calculate_con_weighted_mean_variance(pdf);
		gas_con_mean_map[i] = cell_lists[i].meas_mean;
	}
}

void DeepCFDPlumeModel2D::calculate_gas_concentration_map(GasDispersionMap &gas_map)
{
	float sx = gas_map.sx;
	float sy = gas_map.sy;
	x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/r_torch.size(0)) = 1 / torch::sqrt((sx - r_torch).pow(2) + (sy - c_torch).pow(2));
	std::vector<torch::jit::IValue> inputs;
	// transpose the tensor
	torch::Tensor x_trans = x.permute({0, 1, 3, 2});
	// NOTE: The input tensor must be a float tensor
	inputs.push_back(x_trans.to(torch::kFloat).to(device));
	// std::cout << x_trans[0][1] << std::endl;
	//  Execute the model and turn its output into a tensor.

	// time(&start);
	at::Tensor output = DeepCFD(inputs).toTensor();
	// calculate the gas concentration for each cell
	for (int i = 0; i < x_nbins; i++)
	{
		for (int j = 0; j < y_nbins; j++)
		{
			float px, py;
			indices_to_position_2D(i, j, &px, &py);
			int idx = round((px - min(r_torch[0]).item<float>()) / (r_torch[1][0] - r_torch[0][0]).item<float>());
			int idy = round((py - min(c_torch[0]).item<float>()) / (c_torch[0][1] - c_torch[0][0]).item<float>());
			float con = output[0][0][idy][idx].item<float>();
			gas_map.gasConcentrationMap[i][j] = con;
		}
	}
}
double DeepCFDPlumeModel2D::approx_concentration(Position robot_position, double wind_intensity)
{
	return concentration(robot_position, wind_intensity, expected_value);
}

void DeepCFDPlumeModel2D::log_gas_concentration_map(char *filename, char writing_mode)
{
	for (auto gas_map : gas_con_map_list)
	{
		gas_map.log_map(filename, writing_mode);
	}
}

/*************************************************************************************************/
//	ADEPlumeModel2D (2D Steady state of Advection-Diffusion Model sloved by FDM) 				//
/************************************************************************************************/

AdeFDMPlumeModel2D::AdeFDMPlumeModel2D()
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
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	for (int i = 0; i < nbr_param; i++)
	{
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	// MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new double[nbr_param];
	covar_sig_ii[0] = 2;   // x
	covar_sig_ii[1] = 0.1; // y
	// FIXME - hard-coded wind speed
	u_x = -0.75;
	// double Nx, double Ny, double D, double Q
	set_parameters(100, 40, 0.008, 1500.0);
	//set_u_field_uniform(u_x);
	set_u_field_from_csv("../../data/wind_map_2D.csv", 64, 64);
	assemble_and_factorize();
}

AdeFDMPlumeModel2D::AdeFDMPlumeModel2D(const AdeFDMPlumeModel2D &another_model)
{
	nbr_param = another_model.nbr_param;

	param_list = new parameter[nbr_param];
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	covar_sig_ii = new double[nbr_param];

	for (int i = 0; i < nbr_param; i++)
	{
		param_list[i] = another_model.param_list[i];
		expected_value[i] = another_model.expected_value[i];
		max_a_post_value[i] = another_model.max_a_post_value[i];
		covar_sig_ii[i] = another_model.covar_sig_ii[i];
	}
}
// destructor
AdeFDMPlumeModel2D::~AdeFDMPlumeModel2D()
{
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

void AdeFDMPlumeModel2D::set_parameters(double Nx, double Ny, double D, double Q)
{
	Nx_ = Nx;
	Ny_ = Ny;
	N_ = Nx_ * Ny_;
	D_ = D;
	dx_ = (source_x_max - source_x_min) / (Nx_ - 1);
	dy_ = (source_y_max - source_y_min) / (Ny_ - 1);
    
	Q_ = Q;
	source_strength_ = Q_ / (dx_ * dy_); // point source strength (mass per area per time)
	u_.assign(N_, 0.0);
	v_.assign(N_, 0.0);
}


void AdeFDMPlumeModel2D::set_u_field_uniform(double u)
{
	 for (int j = 0; j < Ny_; ++j) {
            for (int i = 0; i < Nx_; ++i) {
                u_[grid_idx(i,j)] = u;
            }
        }
}

void AdeFDMPlumeModel2D::load_csv(const std::string& filename,
                         std::vector<double>& xs,
                         std::vector<double>& ys,
                         std::vector<double>& us,
                         std::vector<double>& vs) {
	std::ifstream file(filename);
	if (!file.is_open()) throw std::runtime_error("Cannot open file " + filename);

	std::string line;
	bool first = true;
	while (std::getline(file, line)) {
		if (first) { first = false; continue; } // skip header
		std::stringstream ss(line);
		std::string cell;
		std::vector<std::string> tokens;
		while (std::getline(ss, cell, ',')) tokens.push_back(cell);
		if (tokens.size() < 4) continue;

		xs.push_back(std::stod(tokens[0]));
		ys.push_back(std::stod(tokens[1]));
		us.push_back(std::stod(tokens[2]));
		vs.push_back(std::stod(tokens[3]));
	}
	file.close();
}

    // Bilinear interpolation helper
double AdeFDMPlumeModel2D::bilinear(double x, double y,
                           const std::vector<double>& X,
                           const std::vector<double>& Y,
                           const std::vector<double>& F,
                           int Nx, int Ny) {
        // find cell indices
        auto itx = std::lower_bound(X.begin(), X.end(), x);
        auto ity = std::lower_bound(Y.begin(), Y.end(), y);

        int i1 = std::max(0, int(itx - X.begin() - 1));
        int j1 = std::max(0, int(ity - Y.begin() - 1));
        int i2 = std::min(Nx - 1, i1 + 1);
        int j2 = std::min(Ny - 1, j1 + 1);

        double x1 = X[i1], x2 = X[i2];
        double y1 = Y[j1], y2 = Y[j2];

        double Q11 = F[j1 + i1 * Ny];
        double Q21 = F[j1 + i2 * Ny];
        double Q12 = F[j2 + i1 * Ny];
        double Q22 = F[j2 + i2 * Ny];

        double tx = (x - x1) / (x2 - x1 + 1e-12);
        double ty = (y - y1) / (y2 - y1 + 1e-12);
	
        return (1 - tx) * (1 - ty) * Q11 +
               tx * (1 - ty) * Q21 +
               (1 - tx) * ty * Q12 +
               tx * ty * Q22;
}

    // Set wind field from CSV, resampled to (Nx_, Ny_)
void AdeFDMPlumeModel2D::set_u_field_from_csv(const std::string& filename,
							int Nx_src, int Ny_src) {
	std::vector<double> xs, ys, us, vs;
	load_csv(filename, xs, ys, us, vs);

	// build sorted unique X and Y from source
	std::vector<double> Xs, Ys;
	Xs.reserve(Nx_src); Ys.reserve(Ny_src);
	for (int i = 0; i < Nx_src; ++i){
		Xs.push_back(xs[i*Ny_src]);  // first row 
	}   
	for (int j = 0; j < Ny_src; ++j){
		Ys.push_back(ys[j]); // first col
	} 

	for (int j = 0; j < Ny_; ++j) {
		for (int i = 0; i < Nx_; ++i) {
			double xt = source_x_min + dx_ * i;
			double yt = source_y_min + dy_ * j;
			u_[grid_idx(i,j)] = bilinear(xt, yt, Xs, Ys, us, Nx_src, Ny_src);
			v_[grid_idx(i,j)] = bilinear(xt, yt, Xs, Ys, vs, Nx_src, Ny_src);
			// fprintf(console_logfile, "u[%d,%d] = %f, v[%d,%d] = %f\n", i, j, u_[grid_idx(i,j)], i, j, v_[grid_idx(i,j)]);
		}
	}
}



void AdeFDMPlumeModel2D::assemble_and_factorize() {
	std::vector<Triplet> T;
	T.reserve(5 * N_);

	const double dx2 = dx_ * dx_;
	const double dy2 = dy_ * dy_;
	const double a_s_base = D_ / dy2;
	const double a_n_base = D_ / dy2;
	const double a_w_base = D_ / dx2;
	const double a_e_base = D_ / dx2;
    // NOTE: order:first scan x axis then y axis
	for (int j = 0; j < Ny_; ++j) {
		for (int i = 0; i < Nx_; ++i) {
			const int k = grid_idx(i,j);

			// Boundaries
			if (i == Nx_ - 1) {
				// inflow (Dirichlet)
				T.emplace_back(k, k, 1.0);
				continue;
			}
			if (i == 0) {
				// outflow: C(i) - C(i+1) = 0
				T.emplace_back(k, k, 1.0);
				T.emplace_back(k, grid_idx(i+1,j), -1.0);
				continue;
			}
			if (j == 0) {
				// bottom: C(i,j) - C(i,j+1) = 0
				T.emplace_back(k, k, 1.0);
				T.emplace_back(k, grid_idx(i, j+1), -1.0);
				continue;
			}
			if (j == Ny_ - 1) {
				// top: C(i,j) - C(i,j-1) = 0
				T.emplace_back(k, k, 1.0);
				T.emplace_back(k, grid_idx(i, j-1), -1.0);
				continue;
			}

			// Interior cell
			const double u = u_[k];
			const double v = v_[k];

			// advection + diffusion
			double a_s = a_s_base;
			double a_n = a_n_base;
			double a_w = a_w_base;
			double a_e = a_e_base;
	
			if (u > 0.0) {
				// flow left -> right
				a_w +=  u / dx_ ;
			} else {
				// flow right -> left
				a_e += -u / dx_;
			}
			if (v > 0.0) {
				// flow bottom -> top
				a_s +=  v / dy_ ;
			} else {
				// flow top -> bottom
				a_n += -v / dy_;
			}

			const double a_p = -(a_w + a_e + a_s + a_n);

			// Fill row k
			T.emplace_back(k, grid_idx(i-1, j), a_w);
			T.emplace_back(k, grid_idx(i+1, j), a_e);
			T.emplace_back(k, grid_idx(i, j-1), a_s);
			T.emplace_back(k, grid_idx(i, j+1), a_n);
			T.emplace_back(k, k, a_p);
		}
	}

	A_.resize(N_, N_);
	A_.setFromTriplets(T.begin(), T.end());
	A_.makeCompressed();

	// Factorize once
	solver_.analyzePattern(A_);
	solver_.factorize(A_);
	if (solver_.info() != Eigen::Success) {
		throw std::runtime_error("SparseLU factorization failed");
	}
	fact_ok_ = true;
}

std::vector<double> AdeFDMPlumeModel2D::solve_point_source(double sx, double sy) const {
	if (!fact_ok_) throw std::runtime_error("Call assemble_and_factorize() first");

	int si, sj;
	world_pos_to_grid_indices(sx, sy, si, sj);
	const int k_src = grid_indices_to_idx(si, sj);


	Eigen::VectorXd b = Eigen::VectorXd::Zero(N_);
	b(k_src) = -source_strength_; // match your sign convention

	Eigen::VectorXd x = solver_.solve(b);
	if (solver_.info() != Eigen::Success) {
		throw std::runtime_error("Linear solve failed");
	}

	std::vector<double> C(N_);
	for (int k = 0; k < N_; ++k) C[k] = x(k);
	return C;
}

void AdeFDMPlumeModel2D::calculate_gas_concentration_map(GasDispersionMap &gas_map){
	float sx = gas_map.sx;
	float sy = gas_map.sy;

	// calculate the gas concentration for each cell in the map
	// NOTE: The resolution of the gasConcentrationMap is different than the FDM grid resolution

	std::vector<double> C = solve_point_source(sx, sy);

	for (int map_i = 0; map_i < x_nbins; map_i++)
	{
		for (int map_j = 0; map_j < y_nbins; map_j++)
		{
			// Convert map indices to map index
			float px, py;
			indices_to_position_2D(map_i, map_j, &px, &py);
			int grid_i, grid_j;
			world_pos_to_grid_indices(px, py, grid_i, grid_j);
			int idx = grid_indices_to_idx(grid_i, grid_j);
			float con = C[idx];
			gas_map.gasConcentrationMap[map_i][map_j] = con;
		}
	}
}

