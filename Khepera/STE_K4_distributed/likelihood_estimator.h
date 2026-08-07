#ifndef LIKELIHOOD_ESTIMATOR_H
#define LIKELIHOOD_ESTIMATOR_H

#include "PlumeModel.h"
#include "Posterior.h"


extern SampleBuffer sampleBuffer;
// likelihood function
float likelihood(struct history &history, float *parameters, PlumeModel *model);
float likelihood_precomp(struct history &history, float *parameters, int source_pos_idx, PlumeModel *model);
float likelihood_absolute_value(struct history &history, float *parameters, int source_pos_idx, PlumeModel *model);
float likelihood_rel_con(struct history &history, float *parameters, int soure_pos_idx, PlumeModel *model);
float likelihood_gas_hit(struct history &history, float *parameters, int soure_pos_idx, PlumeModel *model);
float log_likelihood(struct history &history, float *parameters, PlumeModel *model);
float log_likelihood_precomp(struct history &history, float *parameters, int source_pos_idx, PlumeModel *model);
float log_likelihood_absolute_value(struct history &history, float *parameters, int source_pos_idx, PlumeModel *model);
float log_likelihood_rel_con(struct history &history, float *parameters, int soure_pos_idx, PlumeModel *model, vector<float> *sorted_model_measurements = nullptr);
float log_likelihood_gas_hit(struct history &history, float *parameters, int soure_pos_idx, PlumeModel *model);

// prior function
bool prior(float *parameters, int nbr_dimension, PlumeModel *model);
float log_prior(float *parameters, int nbr_dimension, PlumeModel *model);

void calculate_measurements_list_empirical_CDF(vector<float> &rel_meas_list, vector<float> &slope_list, const vector<float> &data_list);
void calculate_measurements_list_empirical_CDF(vector<float> &rel_meas_list, vector<float> &slope_list, const vector<float> &data_list, vector<float> &sorted_data);
void calculate_normalized_relative_measurements(vector<float> &rel_meas_list, const vector<float> &data_list);
void calculate_measurements_zscore(vector<float> &rel_meas_list, const vector<float> &data_list);

#endif
