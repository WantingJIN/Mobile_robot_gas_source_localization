#ifndef GAS_DISPERSION_MAP_H
#define GAS_DISPERSION_MAP_H

#include <vector>
#include <condition_variable>
#include "Constant.h"
//#include "PlumeModel.h"
using std::vector;

class GasDispersionMap{ 
    public: 
        // source position idx
        int idx_sx;
        int idx_sy;
        // source position
        double sx;
        double sy;
        double gasConcentrationMap[x_nbins][y_nbins];
    
        
        //constructor
        GasDispersionMap(int index_sx, int index_sy);
    
}; 

#endif