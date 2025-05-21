#ifndef CELLMEASUREMENTS_H
#define CELLMEASUREMENTS_H

#include <vector>
#include <condition_variable>
using std::vector;


typedef struct CellPoint_
{
    float x, y;  // X, Y, position
    int cluster_id;  // clustered ID
}CellPoint;
class CellMeasurements{ 
    public: 
        // idx of the cell   
        int idx_x;
        int idx_y;
        // position of the cell center
        float px;
        float py;
        vector<float> measurements;
        vector<int> gas_hits;
        // gas concentration features
        float meas_std;
        float meas_mean;
        int meas_num;
        
        // gas hit features
        int gas_hit_thres;
        float hit_sum;
        float hit_std;
        float hit_mean;

        //constructor : everything set to 0
        CellMeasurements();
        CellMeasurements(int idx_x, int idx_y);

        ~CellMeasurements(){}

        void add_measurement(float meas);

        void add_gas_hit(float meas);

        // calculate the variance by taking in the pdf of the source location
        void calculate_con_weighted_mean_variance(double *pdf);
        void calculate_hit_weighted_mean_variance(double *pdf);
}; 

#endif