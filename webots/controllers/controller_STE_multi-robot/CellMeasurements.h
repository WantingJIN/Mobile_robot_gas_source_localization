#ifndef CELLMEASUREMENTS_H
#define CELLMEASUREMENTS_H

#include <vector>
#include <condition_variable>
using std::vector;

class CellMeasurements{ 
    public: 
        // idx of the cell   
        int idx_x;
        int idx_y;
        // position of the cell center
        double px;
        double py;
        vector<float> measurements;
        float meas_sum;
        float meas_std;
        float meas_mean;
        int meas_num;


        //constructor : everything set to 0
        CellMeasurements();
        CellMeasurements(int idx_x, int idx_y);

    
        void add_measurement(float meas);

        void calculate_mean();

        void calculate_variance();
        // calculate the variance by taking in the pdf of the source location
        void calculate_weighted_variance(double *pdf);
}; 

#endif