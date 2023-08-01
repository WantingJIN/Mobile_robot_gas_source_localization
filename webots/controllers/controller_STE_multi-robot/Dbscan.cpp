#include "Dbscan.h"
#include <stack>

using namespace std;
DBSCAN::DBSCAN(float eps, unsigned int minPts, double rat){
	epsilon = eps;
	minPoints = minPts;
	ratio = rat;
}

vector<int> DBSCAN::findNeighbors(const CellPoint cell){
	int index = 0;
    vector<CellPoint>::iterator iter;
    vector<int> clusterIndex;
    for( iter = cell_list.begin(); iter != cell_list.end(); ++iter)
    {
        if ( calculateDistance(cell, *iter) <= epsilon )
        {
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

void DBSCAN::expandCluster(CellPoint& cell, vector<int>& neighbors, int clusterId){
	 int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        for(iterSeeds = neighbors.begin(); iterSeeds != neighbors.end(); ++iterSeeds)
        {
            cell_list.at(*iterSeeds).cluster_id = clusterId;
            if (cell_list.at(*iterSeeds).x == cell.x && cell_list.at(*iterSeeds).y == cell.y)
            {
                indexCorePoint = index;
				break;
            }
            ++index;
        }
        neighbors.erase(neighbors.begin()+indexCorePoint);

        for( vector<int>::size_type i = 0, n = neighbors.size(); i < n; ++i )
        {
            vector<int> clusterNeighors = findNeighbors(cell_list.at(neighbors[i]));

            if ( clusterNeighors.size() >= minPoints)
            {
                vector<int>::iterator iterNeighors;
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                {
                    if ( cell_list.at(*iterNeighors).cluster_id == UNCLASSIFIED || cell_list.at(*iterNeighors).cluster_id == NOISE)
                    {
                        if (cell_list.at(*iterNeighors).cluster_id == UNCLASSIFIED )
                        {
                            neighbors.push_back(*iterNeighors);
                            n = neighbors.size();
                        }
                        cell_list.at(*iterNeighors).cluster_id = clusterId;
                    }
                }
            }
        }
	printf("Exit expandCluster \n");
	// std::stack<CellMeasurements> stack;
    // stack.push(cell);
    // cell.cluster_id = clusterId;

    // while (!stack.empty()) {
    //     CellMeasurements currentPoint = stack.top();
    //     stack.pop();

    //     // Find neighbors of the current point
    //     std::vector<CellMeasurements> currentNeighbors = findNeighbors(currentPoint, cell_list);

    //     // Check if the current point is a core point
    //     if (currentNeighbors.size() >= minPoints) {
    //         for (auto & neighbor : currentNeighbors) {
    //             if (neighbor.cluster_id == -1) {
    //                 neighbor.cluster_id = clusterId;
    //                 stack.push(neighbor);
    //             }
    //         }
    //     }
    // }
}

void DBSCAN::run(vector<CellMeasurements> cells){
	double max_std = 0;
	double min_std = 100000;
	for (vector<CellMeasurements>::iterator iter = cells.begin(); iter != cells.end(); iter++){
		if (iter->meas_std > max_std){
			max_std = iter->meas_std;
		}
		if (iter->meas_std < min_std){
			min_std = iter->meas_std;
		}
	}
	for (vector<CellMeasurements>::iterator iter = cells.begin(); iter != cells.end(); iter++){
		if (iter->meas_std > 0.5 * (max_std - min_std) + min_std){
			CellPoint cell;
			cell.x = iter->px;
			cell.y = iter->py;
			cell.cluster_id = UNCLASSIFIED;
			cell_list.push_back(cell);
		}
	}
	pointSize = cell_list.size();
	int clusterId = 0;
	for (vector<CellPoint>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++){
		if (iter->cluster_id == UNCLASSIFIED){
			vector<int> neighbors = findNeighbors(*iter);
			if (neighbors.size() < minPoints){
				iter->cluster_id = NOISE;
			}
			else{
				expandCluster(*iter, neighbors, clusterId);
				clusterId++;
			}
		}
	}
	vector<pair<CellPoint, CellPoint>> cluster_bounding_boxes;
	cluster_bounding_boxes = calculateBoundingBoxes(clusterId);
	// print cluster bounding boxes
	printResults(clusterId);
	for (vector<pair<CellPoint, CellPoint>>::iterator iter = cluster_bounding_boxes.begin(); iter != cluster_bounding_boxes.end(); iter++){
		printf("Cluster %d: (%f, %f) - (%f, %f) \n", iter->first.cluster_id, iter->first.x, iter->first.y, iter->second.x, iter->second.y);
	}
}

inline double DBSCAN::calculateDistance(CellPoint cellCore, CellPoint cellTarget){
	return sqrt(pow(cellCore.x - cellTarget.x, 2) + pow(cellCore.y - cellTarget.y, 2));
}

void DBSCAN::printResults(int cluster_num){
	printf("Number of clusters found: %d \n", cluster_num);
	printf("Clustered cells: \n");
	for (vector<CellPoint>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++){
		if (iter->cluster_id != NOISE){
			printf("Cell (%f, %f) @ cluster %d \n", iter->x, iter->y ,iter->cluster_id+1);
		}
	}
}

pair<CellPoint, CellPoint> DBSCAN::calculateBoundingBox(const std::vector<CellPoint>& points, const int clusterId) {
    float minX = numeric_limits<float>::max();
    float minY = numeric_limits<float>::max();
    float maxX = numeric_limits<float>::min();
    float maxY = numeric_limits<float>::min();

    for (auto& point : points){
        minX = min(minX, point.x);
        minY = min(minY, point.y);
        maxX = max(maxX, point.x);
        maxY = max(maxY, point.y);
    }

    CellPoint topLeft = {minX, minY, clusterId};
    CellPoint bottomRight = {maxX, maxY, clusterId};
    return std::make_pair(topLeft, bottomRight);
}

vector<pair<CellPoint, CellPoint>> DBSCAN::calculateBoundingBoxes(int cluster_num){
	vector<pair<CellPoint, CellPoint>> boundingBoxes;
	for (int i = 0; i < cluster_num; i++){
		vector<CellPoint> points;
		for (vector<CellPoint>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++){
			if (iter->cluster_id == i){
				points.push_back(*iter);
			}
		}
		boundingBoxes.push_back(calculateBoundingBox(points, i));
	}
	return boundingBoxes;
}