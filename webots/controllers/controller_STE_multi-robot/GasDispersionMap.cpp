#include <math.h>
//#include "nvwa/debug_new.h"
#include "GasDispersionMap.h"
#include <fstream>

GasDispersionMap::GasDispersionMap(int index_sx, int index_sy){
	idx_sx = index_sx;
	idx_sy = index_sy;
	sx = source_x_min + idx_sx * x_resol;
  	sy = source_y_min + idx_sy * y_resol;
	for (int i = 0; i < x_nbins; i++){
    	for (int j = 0; j < y_nbins; j++) {
      		gasConcentrationMap[i][j] = 0;
   		}
	}
}



