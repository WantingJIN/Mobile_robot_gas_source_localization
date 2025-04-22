#include "PlumeModel.h"
#include "Posterior.h"


extern SampleBuffer sampleBuffer;
// likelihood function
double likelihood(struct history &history, double *parameters, PlumeModel *model);
double likelihood_precomp(struct history &history, double *parameters, int source_pos_idx, PlumeModel *model);
double likelihood_rel_con(struct history &history, vector<double> rel_history_list, double *parameters, int soure_pos_idx, PlumeModel *model);
double likelihood_gas_hit(struct history &history, double *parameters, int soure_pos_idx, PlumeModel *model);

// prior function
bool prior(double *parameters, int nbr_dimension, PlumeModel *model);

void calculate_measurements_list_empirical_CDF(vector<double> &rel_meas_list, vector<double> &slope_list, vector<double> data_list);
void calculate_normalized_relative_measurements(vector<double> &rel_meas_list, vector<double> data_list);    



