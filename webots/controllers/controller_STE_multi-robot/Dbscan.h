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

typedef struct CellPoint_
{
    float x, y;  // X, Y, position
    int cluster_id;  // clustered ID
}CellPoint;

class DBSCAN {
public:
    vector<CellPoint> cell_list;

public:
    DBSCAN(float eps, unsigned int minPts, double rat);

    ~DBSCAN(){}
    vector<int> findNeighbors(const CellPoint cell);
    void expandCluster(CellPoint& cell, vector<int>& neighbors, int clusterId);
    void run(vector<CellMeasurements> cells);
    void printResults(int clusterNum);

private:
    unsigned int pointSize;
    unsigned int minPoints;
    double ratio;
    float epsilon;
    inline double calculateDistance(CellPoint cell1, CellPoint cell2);
    pair<CellPoint, CellPoint> calculateBoundingBox(const std::vector<CellPoint>& points, const int clusterId);
    vector<pair<CellPoint, CellPoint>> calculateBoundingBoxes(int cluster_num);
};

#endif
