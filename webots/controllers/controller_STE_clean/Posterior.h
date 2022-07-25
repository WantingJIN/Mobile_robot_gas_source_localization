#ifndef POSTERIOR_H
#define POSTERIOR_H

#include "controller_STE_clean.h"

class Posterior{ 
    public: 
    
    double sum; 
    double max;
    double entropy;
    int dirac_flag;
    double pdf[x_nbins][y_nbins][Q_nbins][a_y_nbins][b_y_nbins]; 
    struct parameters_set max_a_posteriori;
    struct parameters_set expected_value;

    //constructor : uniform distribution
    Posterior();
    //constructor : empty distribution (all elements 0)
    Posterior(int zero);

    //update the posterior pdf with the value given by sample paramter set
    void update(double value, struct parameters_set sample);
    //normalization + entropy + expected value + max a posterior (value and parameter set)
    void process();

    //efficient sampling through MCMC
    void MCMC_evaluation(struct history history);
    //exhaustive evaluation of all the values of the posterior pdf
    void exhaustive_evaluation(struct history history);

    //relative entropy calculation between one posterior pdf and another Q
    double Kullback_Leibler_divergence(Posterior Q);

    //
    void XYmarginal_distance_combined(double current_position[], double ratio, double * best_point);

    //log the posterior pdf
    void log_pdf(char * file_name, char writing_mode);
}; 

#endif
