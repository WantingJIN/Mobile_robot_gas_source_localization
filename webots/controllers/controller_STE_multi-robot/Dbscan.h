#pragma once
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include "CellMeasurements.h"


#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2

using namespace std;

class DBSCAN
{
public:
    vector<CellPoint> cell_list;
    int cluster_num;
    vector<pair<CellPoint, CellPoint>> cluster_bounding_boxes;

public:
    DBSCAN(float x_eps, float y_eps, unsigned int minPts, double rat);

    ~DBSCAN() {}
    vector<int> findNeighbors(const CellPoint cell);
    void expandCluster(CellPoint &cell, vector<int> &neighbors, int clusterId);
    void run(double cells[]);
    void printResults();

private:
    unsigned int pointSize;
    unsigned int minPoints;
    double ratio;
    float x_epsilon;
    float y_epsilon;

    inline bool isWithinDistance(CellPoint cell1, CellPoint cell2);
    pair<CellPoint, CellPoint> calculateBoundingBox(const std::vector<CellPoint> &points, const int clusterId);
    void calculateBoundingBoxes();
    void logBoundingBoxes();
};

#endif
