#include "ste_package/likelihood_estimator.h"

extern int nbr_obs;
extern float margin;
extern float **obstacles;
extern SimulationParameters simulation_parameters;
// ====================== ESTIMATION functions ========================= //

// Likelihood with concentration values from precomputed gas maps (initialized and stored during initialization)
double likelihood_precomp(struct History &history, double *parameters, int source_pos_idx, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		int idx_, idy_;
		position_to_indices_2D(px[i], py[i], &idx_, &idy_);
		idx[i] = idx_;
		idy[i] = idy_;

	}

	model->retrieve_hist_measurements(idx, idy, history.size, source_pos_idx, model_c_avg);

	// absolute concentration error
	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.gas_meas[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
	}


	return (exp(-0.5 * sum_tmp));
}


// Likelihood for relative concentration with precomputed gas maps
double likelihood_rel_con(struct History &history, double *parameters, int source_pos_idx, PlumeModel *model)
{

	float sigma;
	sigma = simulation_parameters.likelihoodSigma;
    if (simulation_parameters.modelUncertainty == FIXED_UNCERTAINTY){
		sigma = simulation_parameters.likelihoodSigma;
	}
	else if (simulation_parameters.modelUncertainty == DYNAMIC_UNCERTAINTY){
		// dynamic uncertainty based on wind estimation confidence
		double wind_confidence = model->get_wind_estimation_confidence(source_pos_idx);
		sigma = simulation_parameters.likelihoodSigma * (1 + simulation_parameters.wind_belief_weight * exp(-wind_confidence*2));
		// if (wind_confidence < 1e-3){
		// 	// very high uncertainty when wind confidence is too low
		// 	sigma = simulation_parameters.likelihoodSigma * 2.0;
		// } else {
		// 	sigma = simulation_parameters.likelihoodSigma;
		// }
	}

	double sum_tmp = 0; 

	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		int idx_, idy_;
		position_to_indices_2D(px[i], py[i], &idx_, &idy_);
		// if (idx_ == -1 || idy_ == -1)
		// {
		// 	printf("point out of the estimation region, does not take into account!\n");
		// 	// remove it from history list
		// 	history.size--;
		// 	history.position.erase(history.position.begin() + i);
		// 	history.gas_meas.erase(history.gas_meas.begin() + i);
		// 	history.FSM_iter.erase(history.FSM_iter.begin() + i);
		// } else {
		idx[i] = idx_;
		idy[i] = idy_;
		// }
	}

	model->retrieve_hist_measurements(idx, idy, history.size, source_pos_idx, model_c_avg);

	vector<double> rel_model_list;
	vector<double> pdf_model_list;
	vector<double> slope_model_list;
	double rel_model_val, rel_hist_val;
	if (simulation_parameters.relativeMethod == NORMALIZATION)
	{
		calculate_normalized_relative_measurements(rel_model_list, model_c_avg);
		// if (source_pos_idx == 82 || source_pos_idx == 385){
		// 	if (source_pos_idx == 82) fprintf(console_logfile, "True source\n");
        //     if (source_pos_idx == 385) fprintf(console_logfile, "Estimated source\n");
		
		// 	fprintf(console_logfile, "Model values: \n");
		// 	for (int i = 0; i < model_c_avg.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", model_c_avg[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");

		// 	fprintf(console_logfile, "Model normalized values: \n");
		// 	for (int i = 0; i < rel_model_list.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", rel_model_list[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");
		// }

		
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			sum_tmp += history.size * pow(rel_model_val - rel_hist_val, 2) / (history.size + 1);
		}
		sum_tmp /= sigma * sigma;
	}

	if (simulation_parameters.relativeMethod == Z_SCORE)
	{
		double cosine_distance = 0;
		calculate_measurements_zscore(rel_model_list, model_c_avg);
		// if (source_pos_idx == 82 || source_pos_idx == 0){
		// 	if (source_pos_idx == 82) fprintf(console_logfile, "True source\n");
        //     if (source_pos_idx == 0) fprintf(console_logfile, "Estimated source\n");
		
		// 	fprintf(console_logfile, "Model values: \n");
		// 	for (int i = 0; i < model_c_avg.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", model_c_avg[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");

		// 	fprintf(console_logfile, "Model normalized values: \n");
		// 	for (int i = 0; i < rel_model_list.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", rel_model_list[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");
		// }

		// cosine_distance = calculate_cosine_distance(rel_model_list, history.relative_meas);
		// sum_tmp = cosine_distance/(sigma*sigma);
		// printf("Cosine distance: %f\n", cosine_distance);
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			sum_tmp += pow(rel_model_val - rel_hist_val, 2) / (sigma * sigma);
		}
	}

	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF)
	{   // print the time it required to calculate the empirical CDF
		auto start = std::chrono::high_resolution_clock::now();
		calculate_measurements_list_empirical_CDF(rel_model_list, slope_model_list, model_c_avg);
		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		std::cout << "Sort measurement takes: " << duration.count() << " ms" << std::endl;
		// sigma = 0.45;// works well for Empirical CDF
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			sum_tmp += pow(rel_model_val - rel_hist_val, 2) / (sigma * sigma);
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
			sum_tmp += pow(rel_model_val - rel_hist_val, 2) / (sigma * sigma);
		}
	}
	if (simulation_parameters.relativeMethod == EMPIRICAL_CDF_NUM)
	{
		// calculate_normalized_relative_measurements(rel_model_list, model_c_avg);
		// if (source_pos_idx == 82 || source_pos_idx == 385){
		// 	if (source_pos_idx == 82) fprintf(console_logfile, "True source\n");
        //     if (source_pos_idx == 354) fprintf(console_logfile, "Estimated source\n");
		
		// 	fprintf(console_logfile, "Model values: \n");
		// 	for (int i = 0; i < model_c_avg.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", model_c_avg[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");

		// 	fprintf(console_logfile, "Model normalized values: \n");
		// 	for (int i = 0; i < rel_model_list.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", rel_model_list[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");
		// }

		
		calculate_measurements_list_empirical_CDF(rel_model_list, slope_model_list, model_c_avg);
		
		// if (source_pos_idx == 82 || source_pos_idx == 354){
		// 	fprintf(console_logfile, "Model ranking values: \n");
		// 	for (int i = 0; i < rel_model_list.size(); i++)
		// 	{
		// 		fprintf(console_logfile, "%f, ", rel_model_list[i]);
		// 	}
		// 	fprintf(console_logfile, "\n");
		// }
		
		// sigma = 0.48;// works well for Empirical CDF NUM
		for (int i = 0; i < history.size; i++)
		{
			rel_hist_val = history.relative_meas[i];
			rel_model_val = rel_model_list[i];
			// NOTE: take into account the measurement number information, to avoid the algorithm converge to fast to local minima
			sum_tmp += history.size * pow(rel_model_val - rel_hist_val, 2) / (history.size + 1);
		}
		sum_tmp /= sigma * sigma;
	}


	// if (simulation_parameters.modelUncertainty == DYNAMIC_UNCERTAINTY){
	// 	// dynamic uncertainty based on wind estimation confidence
	// 	double wind_confidence = model->get_wind_estimation_confidence(source_pos_idx);
	// 	if (wind_confidence < 1e-3){
	// 		// sigmaE = simulation_parameters.likelihood_error * (5.0 + (1.0 - wind_confidence)); // increase uncertainty when wind confidence is low
	// 		sum_tmp = 1; // very high likelihood
	// 	}
	// }

	return (exp(-0.5 * sum_tmp));
}

double likelihood(struct History &history, double *parameters, PlumeModel *model)
{
	float sigmaE = simulation_parameters.likelihood_error;
	float sigmaD = 0; // 125;
	double sum_tmp = 0;

	// getting all model concentrations for all sample positions and then calculate the likelihood
	vector<float> model_c_avg(history.size);
	vector<float> model_c_std(history.size);
	// calculate the average wind intensity from the history measurements
	double average_wind_u = 0, average_wind_v = 0, average_wind_intensity = 0;
	for (int i = 0; i < history.size; i++)
	{
		average_wind_u += history.wind_meas_u[i];
		average_wind_v += history.wind_meas_v[i];
	}
	average_wind_u /= history.size;
	average_wind_v /= history.size;
	average_wind_intensity = sqrt(average_wind_u * average_wind_u + average_wind_v * average_wind_v);

	model->concentration(history.position, history.size, average_wind_intensity, parameters, model_c_avg, model_c_std);

	for (int i = 0; i < history.size; i++)
	{
		sum_tmp += pow(history.gas_meas[i] - model_c_avg[i], 2) / (sigmaE * sigmaE + sigmaD * sigmaD);
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

void calculate_measurements_zscore(vector<double> &rel_meas_list, const vector<double> data_list)
{
	rel_meas_list.resize(data_list.size());
	if (data_list.size() == 1)
	{
		rel_meas_list[0] = 0.0;
		return;
	}
	// calculate the mean and standard deviation
	double sum = 0.0;
	for (const auto& val : data_list) {
		sum += val;
	}
	double mean = sum / data_list.size();

	double sq_sum = 0.0;
	for (const auto& val : data_list) {
		sq_sum += (val - mean) * (val - mean);
	}
	double stddev = sqrt(sq_sum / data_list.size());

	// calculate z-score for each measurement
	for (int i = 0; i < data_list.size(); i++) {
		if (stddev < 1e-6) {
			rel_meas_list[i] = 0.0; // avoid division by zero
		} else {
			rel_meas_list[i] = (data_list[i] - mean) / stddev;
		}
	}
}

double likelihood_gas_hit(struct History &history, double *parameters, int source_pos_idx, PlumeModel *model)
{
	double sum_tmp = 0;
	vector<double> model_c_avg(history.size);

	int nbr = history.size;
	float px[nbr], py[nbr];
	int idx[nbr], idy[nbr];

	// get indices in the map
	for (int i = 0; i < nbr; i++)
	{
		px[i] = history.position[i].x;
		py[i] = history.position[i].y;
		int idx_, idy_;
		position_to_indices_2D(px[i], py[i], &idx_, &idy_);
		// if (idx_ == -1 || idy_ == -1)
		// {
		// 	printf("point out of the estimation region, does not take into account!\n");
		// 	// remove it from history list
		// 	history.size--;
		// 	history.position.erase(history.position.begin() + i);
		// 	history.gas_meas.erase(history.gas_meas.begin() + i);
		// 	history.FSM_iter.erase(history.FSM_iter.begin() + i);
		// } else {
		idx[i] = idx_;
		idy[i] = idy_;
		// }
	}

	// extract predicted concentration values by model
	model->retrieve_hist_measurements(idx, idy, history.size, source_pos_idx, model_c_avg);
    
	int history_hit;
	int model_hit;

	double likelihood;

	//-----model and measurement fixed threshold, penalty on gas hit event difference-------
	if (simulation_parameters.gasHitMethod == FIXED_THRES)
	{
		for (int i = 0; i < history.size; i++)
		{
			if (history.gas_meas[i] < simulation_parameters.gas_hit_threshold)
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
			if (history.gas_meas[i] < simulation_parameters.gas_hit_threshold)
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
			max_con = history.gas_meas[0];
			min_con = history.gas_meas[0];
			max_idx = 0;
			min_idx = 0;
		}
		else
		{
			if (history.gas_meas[history.size - 1] > max_con)
			{
				max_con = history.gas_meas[history.size - 1];
				max_idx = history.size - 1;
			}
			else if (history.gas_meas[history.size - 1] < min_con)
			{
				min_con = history.gas_meas[history.size - 1];
				min_idx = history.size - 1;
			}
			his_thrs = min_con + (max_con - min_con) * 0.5;
			mod_thrs = model_c_avg[min_idx] + (model_c_avg[max_idx] - model_c_avg[min_idx]) * 0.5;
		}
		for (int i = 0; i < history.size; i++)
		{
			// decide whether the gas hit event happens
			history_hit = (history.gas_meas[i] > his_thrs) ? 1 : 0;
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
				prev_his_thrs = history.gas_meas[i];
				his_thrs = history.gas_meas[i];
				prev_mod_thrs = model_c_avg[i];
				mod_thrs = model_c_avg[i];
			}
			his_thrs = simulation_parameters.lambda * prev_his_thrs + (1 - simulation_parameters.lambda) * history.gas_meas[i];
			mod_thrs = simulation_parameters.lambda * prev_mod_thrs + (1 - simulation_parameters.lambda) * model_c_avg[i];
			// store history threshold for next iteration
			prev_his_thrs = his_thrs;
			prev_mod_thrs = mod_thrs;

			// decide whether the gas hit event happens
			history_hit = (history.gas_meas[i] > his_thrs) ? 1 : 0;
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
				prev_his_thrs = history.gas_meas[i];
				his_thrs = history.gas_meas[i];
				prev_mod_thrs = model_c_avg[i];
				mod_thrs = model_c_avg[i];
			}
			his_thrs = simulation_parameters.lambda * prev_his_thrs + (1 - simulation_parameters.lambda) * history.gas_meas[i];
			mod_thrs = simulation_parameters.lambda * prev_mod_thrs + (1 - simulation_parameters.lambda) * model_c_avg[i];
			// store history threshold for next iteration
			prev_his_thrs = his_thrs;
			prev_mod_thrs = mod_thrs;

			// decide whether the gas hit event happens
			history_hit = (history.gas_meas[i] > his_thrs) ? 1 : 0;
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
		output = output && (parameters[i] < model->param_list[i].upper_limit && parameters[i] > model->param_list[i].lower_limit);

	// source should not be at the border of the area
	int idx, idy;
	position_to_indices_2D(parameters[0], parameters[1], &idx, &idy);
    if (idx <= 0 || idx >= model->param_list[0].nbins - 1 || idy <= 0 || idy >= model->param_list[1].nbins - 1)
	{
		output = 0;
	}	

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

// 2. Function to calculate Cosine Distance (1 - Cosine Similarity)
double calculate_cosine_distance(std::vector<double> v1, std::vector<double> v2) {
	double dot_product = 0.0;
    double mag1 = 0.0;
    double mag2 = 0.0;

    for (size_t i = 0; i < v1.size(); ++i) {
        dot_product += v1[i] * v2[i];
        mag1 += v1[i] * v1[i];
        mag2 += v2[i] * v2[i];
    }
    
	
    // Final logic: (dot_product / (sqrt(mag1) * sqrt(mag2)))
	// avoid division by zero
	if (mag1 < 1e-10 || mag2 < 1e-10) {
		return 0.0; // define cosine similarity as 0 when one vector has zero magnitude
	}
	
    double similarity = 1 - dot_product / (std::sqrt(mag1) * std::sqrt(mag2));
    return similarity;
}