#include "PlumeModel.h"
#include "Posterior.h"

// likelihood function
double likelihood(struct History &history, double *parameters, PlumeModel *model);
double likelihood_precomp(struct History &history, double *parameters, int source_pos_idx, PlumeModel *model);
double likelihood_rel_con(struct History &history, double *parameters, int source_pos_idx, PlumeModel *model);
double likelihood_gas_hit(struct History &history, double *parameters, int source_pos_idx, PlumeModel *model);

// prior function
bool prior(double *parameters, int nbr_dimension, PlumeModel *model);

void calculate_measurements_list_empirical_CDF(vector<double> &rel_meas_list, vector<double> &slope_list, const vector<double> data_list);
void calculate_normalized_relative_measurements(vector<double> &rel_meas_list, const vector<double> data_list);    
void calculate_measurements_zscore(vector<double> &rel_meas_list, const vector<double> data_list);
double calculate_cosine_distance(std::vector<double> v1, std::vector<double> v2);

