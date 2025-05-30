#include "likelihood_estimator.h"

extern int nbr_obs;
extern float margin;
extern float **obstacles;
extern SimulationParameters simulation_parameters;
// ====================== ESTIMATION functions ========================= //

// Likelihood with concentration values from precomputed gas maps (initialized and stored during initialization)
double likelihood_precomp(struct history &history, double *parameters, int source_pos_idx, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		position_to_indices_2D(px[i], py[i], &idx[i], &idy[i]);
		// idx should be between 0 and x_nbins-1
		if (idx[i] < 0) idx[i] = 0;
		if (idx[i] >= x_nbins) idx[i] = x_nbins - 1;
		// idy should be between 0 and y_nbins-1
		if (idy[i] < 0) idy[i] = 0;
		if (idy[i] >= y_nbins) idy[i] = y_nbins - 1;
	}

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, source_pos_idx, model_c_avg);

	// absolute concentration error
	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.measurement[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
		// if (source_pos_idx == 0)
		// 	fprintf(console_logfile, "x: %f, y: %f, idx: %d, idy: %d, hist: %f, model: %f\n", px[i], py[i], idx[i], idy[i], history.measurement[i], model_c_avg[i]);
	}

	return (exp(-0.5 * sum_tmp));
}


// Likelihood for relative concentration with precomputed gas maps
double likelihood_rel_con(struct history &history, double *parameters, int soure_pos_idx, PlumeModel *model)
{

	// float sigma = 0.22;// works well for Gaussian CDF
	float sigma = 0.35; // works well for Empirical CDF
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		position_to_indices_2D(px[i], py[i], &idx[i], &idy[i]);
	}

	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	vector<double> rel_model_list;
	vector<double> pdf_model_list;
	vector<double> slope_model_list;
	double rel_model_val, rel_hist_val;
	if (simulation_parameters.relativeMethod == NORMALIZATION)
	{
		calculate_normalized_relative_measurements(rel_model_list, model_c_avg);
		sigma = 0.49; // works well for Normalization
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			sum_tmp += pow(rel_model_val - rel_hist_val, 2) / (sigma * sigma);
		}
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF)
	{
		calculate_measurements_list_empirical_CDF(rel_model_list, slope_model_list, model_c_avg);
		// sigma = 0.45;// works well for Empirical CDF
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			sum_tmp += pow(rel_model_val - rel_hist_val, 2) / (simulation_parameters.likelihoodSigma * simulation_parameters.likelihoodSigma);
		}
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF_AND_SLOPE)
	{
		calculate_measurements_list_empirical_CDF(rel_model_list, slope_model_list, model_c_avg);
		// sigma = 0.45;// works well for Empirical CDF
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			sum_tmp += pow(rel_model_val - rel_hist_val, 2) / (simulation_parameters.likelihoodSigma * simulation_parameters.likelihoodSigma);
		}
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF_NUM)
	{
		calculate_measurements_list_empirical_CDF(rel_model_list, slope_model_list, model_c_avg);
		// sigma = 0.48;// works well for Empirical CDF NUM
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			// NOTE: take into account the measurement number information, to avoid the algorithm converge to fast to local minima
			sum_tmp += history.size * pow(rel_model_val - rel_hist_val, 2) / (history.size + 1);
		}
		sum_tmp /= simulation_parameters.likelihoodSigma * simulation_parameters.likelihoodSigma;
	}
	return (exp(-0.5 * sum_tmp));
}

double likelihood(struct history &history, double *parameters, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

	// getting all model concentrations for all sample positions and then calculate the likelihood
	vector<float> model_c_avg(history.size);
	vector<float> model_c_std(history.size);
	model->concentration(history.position, history.size, sampleBuffer.average_wind_intensity, parameters, model_c_avg, model_c_std);

	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.measurement[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
		// std::cout<<"size: "<< history.size <<"model_c_avg[i] "<<model_c_avg[i]<<std::endl;
	}

	return (exp(-0.5 * sum_tmp));
}

void calculate_measurements_list_empirical_CDF(vector<double> &rel_meas_list, vector<double> &slope_list, const vector<double> data_list)
{
	rel_meas_list.resize(data_list.size());
	slope_list.resize(data_list.size());
	if (data_list.size() == 1)
	{
		rel_meas_list[0] = 0.0;
		slope_list[0] = 0.0;
		return;
	}
	vector<double> sortedData = data_list;
	sort(sortedData.begin(), sortedData.end());

	// Calculate ECDF
	for (size_t i = 0; i < data_list.size(); ++i)
	{
		// Count the number of observations less than or equal to the current value
		size_t count = count_if(sortedData.begin(), sortedData.end(),
						[&](double x)
						{ return data_list[i] - x > 0.1; });
		// Calculate proportion
		rel_meas_list[i] = static_cast<double>(count) / data_list.size();
	}
	// Calculate Normalized Slope of ECDF
	double data_range = sortedData.back() - sortedData.front();
	for (size_t i = 1; i < sortedData.size(); ++i)
	{
		double delta_x = (sortedData[i] - sortedData[i - 1]) / data_range;
		slope_list[i] = (delta_x > 0) ? (delta_x / data_list.size()) : 0.0;
	}
	// Handle the first slope value
	slope_list[0] = slope_list[1];
}

void calculate_normalized_relative_measurements(vector<double> &rel_meas_list, const vector<double> data_list)
{
      rel_meas_list.resize(data_list.size());
	if (data_list.size() == 1)
	{
		rel_meas_list[0] = 0.0;
		return;
	}
      // normalize the measurements with max and min values
	double max_val = *max_element(data_list.begin(), data_list.end());
	double min_val = *min_element(data_list.begin(), data_list.end());
	// printf("max: %f, min: %f\n", max_val, min_val);

	for (int i = 0; i < data_list.size(); i++)
	{
		if (abs(max_val - min_val) < 0.1)
		{
			rel_meas_list[i] = 0.0;
		}
		else
		{
			rel_meas_list[i] = (data_list[i] - min_val) / (max_val - min_val);
		}
		// printf("data: %f, rel: %f\n", data_list[i], relative_history_meas[i]);
	}
}

double likelihood_gas_hit(struct history &history, double *parameters, int soure_pos_idx, PlumeModel *model)
{
	double sum_tmp = 0;
	if (sampleBuffer.average_wind_intensity == 0)
		printf("NO WIND, BYE BYE!\n");

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		position_to_indices_2D(px[i], py[i], &idx[i], &idy[i]);
	}

	// extract predicted concentration values by model
	model->retrieve_hist_measurements(idx, idy, history.size, sampleBuffer.average_wind_intensity, soure_pos_idx, model_c_avg);

	int history_hit;
	int model_hit;

	double likelihood;

	//-----model and measurement fixed threshold, penalty on gas hit event difference-------
	if (simulation_parameters.gasHitMethod == FIXED_THRES)
	{
		for (int i = 0; i < history.size; i++)
		{
			if (history.measurement[i] < simulation_parameters.gas_hit_threshold)
				history_hit = 0;
			else
				history_hit = 1;
			history.gas_hit_event[i] = history_hit;
			if (model_c_avg[i] < simulation_parameters.gas_hit_threshold)
				model_hit = 0;
			else
				model_hit = 1;
			if (history_hit != model_hit)
				sum_tmp += 1;
		}
		sum_tmp *= 0.8; // best result with exp 0.5
		likelihood = (exp(-sum_tmp));
	}
	if (simulation_parameters.gasHitMethod == FIXED_THRES_NUM)
	{
		for (int i = 0; i < history.size; i++)
		{
			if (history.measurement[i] < simulation_parameters.gas_hit_threshold)
				history_hit = 0;
			else
				history_hit = 1;
			history.gas_hit_event[i] = history_hit;
			if (model_c_avg[i] < simulation_parameters.gas_hit_threshold)
				model_hit = 0;
			else
				model_hit = 1;
			if (history_hit != model_hit)
				sum_tmp += 1;
		}
		sum_tmp *= 0.8 * double(history.size) / (double(history.size) + 1); // best result with exp 0.5
		likelihood = (exp(-sum_tmp));
	}
	//-------------------------min max avg threshold-------------------------------

	if (simulation_parameters.gasHitMethod == MIN_MAX_THRES)
	{
		double his_thrs = 0;
		double mod_thrs = 0;
		static double max_con = 0.0;
		static double min_con = 10000.0;
		static int max_idx = 0;
		static int min_idx = 0;

		// Dynamic threshold
		if (history.size <= 1)
		{
			max_con = history.measurement[0];
			min_con = history.measurement[0];
			max_idx = 0;
			min_idx = 0;
		}
		else
		{
			if (history.measurement[history.size - 1] > max_con)
			{
				max_con = history.measurement[history.size - 1];
				max_idx = history.size - 1;
			}
			else if (history.measurement[history.size - 1] < min_con)
			{
				min_con = history.measurement[history.size - 1];
				min_idx = history.size - 1;
			}
			his_thrs = min_con + (max_con - min_con) * 0.5;
			mod_thrs = model_c_avg[min_idx] + (model_c_avg[max_idx] - model_c_avg[min_idx]) * 0.5;
		}
		for (int i = 0; i < history.size; i++)
		{
			// decide whether the gas hit event happens
			history_hit = (history.measurement[i] > his_thrs) ? 1 : 0;
			history.gas_hit_event[i] = history_hit;
			model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
			// update the likelihood based on the similarity of the hit event
			if (history_hit != model_hit)
			{
				sum_tmp += 1;
			}
		}
		sum_tmp *= 0.8;
		likelihood = (exp(-sum_tmp));
	}

	//-------------adaptive threshold- based on recent measurements------------------------------
	if (simulation_parameters.gasHitMethod == SLIDING_WINDOW_THRES)
	{
		// double lamda = 0.8; //0.5;
		double his_thrs = 0;
		double mod_thrs = 0;
		double prev_mod_thrs = 0.0;
		double prev_his_thrs = 0.0;

		// Dynamic thresholds computation
		for (int i = 0; i < history.size; i++)
		{
			if (i == 0)
			{
				prev_his_thrs = history.measurement[i];
				his_thrs = history.measurement[i];
				prev_mod_thrs = model_c_avg[i];
				mod_thrs = model_c_avg[i];
			}
			his_thrs = simulation_parameters.lambda * prev_his_thrs + (1 - simulation_parameters.lambda) * history.measurement[i];
			mod_thrs = simulation_parameters.lambda * prev_mod_thrs + (1 - simulation_parameters.lambda) * model_c_avg[i];
			// store history threshold for next iteration
			prev_his_thrs = his_thrs;
			prev_mod_thrs = mod_thrs;

			// decide whether the gas hit event happens
			history_hit = (history.measurement[i] > his_thrs) ? 1 : 0;
			history.gas_hit_event[i] = history_hit;
			model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
			// update the likelihood based on the similarity of the hit event
			if (history_hit != model_hit)
			{
				sum_tmp += 1;
			}
		}
		sum_tmp *= 0.8;
		likelihood = (exp(-sum_tmp));
	}
	if (simulation_parameters.gasHitMethod == SLIDING_WINDOW_THRES_NUM)
	{
		// double lamda = 0.8; //0.5;
		double his_thrs = 0;
		double mod_thrs = 0;
		double prev_mod_thrs = 0.0;
		double prev_his_thrs = 0.0;

		// Dynamic thresholds computation
		for (int i = 0; i < history.size; i++)
		{
			if (i == 0)
			{
				prev_his_thrs = history.measurement[i];
				his_thrs = history.measurement[i];
				prev_mod_thrs = model_c_avg[i];
				mod_thrs = model_c_avg[i];
			}
			his_thrs = simulation_parameters.lambda * prev_his_thrs + (1 - simulation_parameters.lambda) * history.measurement[i];
			mod_thrs = simulation_parameters.lambda * prev_mod_thrs + (1 - simulation_parameters.lambda) * model_c_avg[i];
			// store history threshold for next iteration
			prev_his_thrs = his_thrs;
			prev_mod_thrs = mod_thrs;

			// decide whether the gas hit event happens
			history_hit = (history.measurement[i] > his_thrs) ? 1 : 0;
			history.gas_hit_event[i] = history_hit;
			model_hit = (model_c_avg[i] > mod_thrs) ? 1 : 0;
			// update the likelihood based on the similarity of the hit event
			if (history_hit != model_hit)
			{
				sum_tmp += 1;
			}
		}
		sum_tmp *= 0.8 * double(history.size) / (double(history.size) + 1);
		likelihood = (exp(-sum_tmp));
	}

	return likelihood;
}

bool prior(double *parameters, int nbr_dimension, PlumeModel *model)
{
	int i = 0;
	bool output = 1;

	// verify if all the parameters are between the right intervals
	for (i = 0; i < nbr_dimension; i++)
		output = output && (parameters[i] < model->param_list[i].upper_limit && parameters[i] >= model->param_list[i].lower_limit);

	// verify the x and y elements of the parameters are not inside obstacles
	Position tmp_p(parameters[0], parameters[1], 0);
	if (tmp_p.is_in_obstacle_list(obstacles, nbr_obs) != -1)
	{
		// printf("point %f %f is in obstacle\n", parameters[0], parameters[1]);
		output = 0;
	}

	// if(output == 0)
	//	printf("parameters: %f %f %f %f %f\n", parameters[0], parameters[1], parameters[2], parameters[3], parameters[4]);

	return (output);
}