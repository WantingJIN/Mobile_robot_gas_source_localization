#include "PlumeModel.h"
#include "Posterior.h"


extern SampleBuffer sampleBuffer;
// likelihood function
float likelihood(struct history &history, float *parameters, PlumeModel *model);
float likelihood_precomp(struct history &history, float *parameters, int source_pos_idx, PlumeModel *model);
float likelihood_rel_con(struct history &history, float *parameters, int soure_pos_idx, PlumeModel *model);
float likelihood_gas_hit(struct history &history, float *parameters, int soure_pos_idx, PlumeModel *model);

// prior function
bool prior(float *parameters, int nbr_dimension, PlumeModel *model);

void calculate_measurements_list_empirical_CDF(vector<float> &rel_meas_list, vector<float> &slope_list, const vector<float> data_list);
void calculate_normalized_relative_measurements(vector<float> &rel_meas_list, const vector<float> data_list);    



