#ifndef DBSCAN_H
#define DBSCAN_H

#include<vector>
#include<cmath>
#include "CellMeasurements.h"

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2


using namespace std;

class DBSCAN {
public:
    vector<CellMeasurements> cell_list;

public:
    DBSCAN(float eps, unsigned int minPts, double rat);

    ~DBSCAN(){}
    vector<CellMeasurements> findNeighbors(const CellMeasurements& cell, const vector<CellMeasurements>& cells);
    void expandCluster(CellMeasurements& cell, vector<CellMeasurements>& neighbors, int clusterId, vector<CellMeasurements>& cells);
    void run(vector<CellMeasurements> cells);
    void printResults();

private:
    unsigned int pointSize;
    unsigned int minPoints;
    double ratio;
    float epsilon;
    inline double calculateDistance(CellMeasurements cell1, CellMeasurements cell2);
};

#endif
