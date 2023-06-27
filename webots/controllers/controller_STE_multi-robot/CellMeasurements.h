#ifndef CELLMEASUREMENTS_H
#define CELLMEASUREMENTS_H

#include <vector>
using std::vector;

class CellMeasurements{ 
    public: 
    
        int idx_x;
        int idx_y;
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
}; 

#endif