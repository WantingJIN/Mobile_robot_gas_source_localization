#ifndef GAS_DISPERSION_MAP_H
#define GAS_DISPERSION_MAP_H

#include <vector>
#include <condition_variable>
#include "constant_and_helper.h"
// #include "PlumeModel.h"
using std::vector;

class GasDispersionMap
{
public:
    // source position idx
    int idx_sx;
    int idx_sy;
    // source position
    float sx;
    float sy;
    double gasConcentrationMap[x_nbins][y_nbins];
    // constructor
    GasDispersionMap(int index_sx, int index_sym);
    void log_map(const char *, char writing_mode);
    double get_concentration(int idx_x, int idx_y);
};

#endif