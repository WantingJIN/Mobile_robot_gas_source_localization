#include "Dbscan.h"


DBSCAN::DBSCAN(float eps, unsigned int minPts, double rat){
	epsilon = eps;
	minPoints = minPts;
	ratio = rat;
}


void DBSCAN::intialize_points(vector<CellMeasurements> cells){
	// only cluster the cells that have the variance larger than 50%
	double max_std = 0;
	double min_std = 100000;
	for (vector<CellMeasurements>::iterator iter = cells.begin(); iter != cells.end(); iter++){
		iter->cluster_id = UNCLASSIFIED;
		if (iter->meas_std > max_std){
			max_std = iter->meas_std;
		}
		if (iter->meas_std < min_std){
			min_std = iter->meas_std;
		}
	}
	for (vector<CellMeasurements>::iterator iter = cells.begin(); iter != cells.end(); iter++){
		if (iter->meas_std > 0.5 * (max_std - min_std) + min_std){
			cell_list.push_back(*iter);
		}
	}
}


vector<CellMeasurements> DBSCAN::findNeighbors(const CellMeasurements& cell, const vector<CellMeasurements>& cells){
	vector<CellMeasurements> neighbors;
	for (vector<CellMeasurements>::const_iterator iter = cells.begin(); iter != cells.end(); iter++){
		if (calculateDistance(cell, *iter) <= epsilon){
			neighbors.push_back(*iter);
		}
	}
	return neighbors;
}

void DBSCAN::expandCluster(CellMeasurements& cell, vector<CellMeasurements>& neighbors, int clusterId, vector<CellMeasurements>& cells){
	cell.cluster_id = clusterId;
	for (vector<CellMeasurements>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++){
		if (iter->cluster_id == UNCLASSIFIED){
			iter->cluster_id = clusterId;
			vector<CellMeasurements> subNeighbors = findNeighbors(*iter, cells);
			if (subNeighbors.size() >= minPoints){
				neighbors.insert(neighbors.end(), subNeighbors.begin(), subNeighbors.end());
			}
		}
		if (iter->cluster_id == NOISE){
			iter->cluster_id = clusterId;
		}
	}
}

void DBSCAN::run(){
	pointSize = cell_list.size();
	int clusterId = 1;
	for (vector<CellMeasurements>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++){
		if (iter->cluster_id == UNCLASSIFIED){
			vector<CellMeasurements> neighbors = findNeighbors(*iter, cell_list);
			if (neighbors.size() < minPoints){
				iter->cluster_id = NOISE;
			}
			else{
				expandCluster(*iter, neighbors, clusterId, cell_list);
				clusterId++;
			}
		}
	}
}

inline double DBSCAN::calculateDistance(CellMeasurements cell1, CellMeasurements cell2){
	return sqrt(pow(cell1.px - cell2.px, 2) + pow(cell1.py - cell2.py, 2));
}

void DBSCAN::printResults(){
	printf("Number of clusters found: %d ", pointSize);
	printf("Clustered cells: \n");
	for (vector<CellMeasurements>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++){
		if (iter->cluster_id != NOISE){
			printf("Cell (%f, %f) @ cluster %d \n", iter->idx_x, iter->idx_y ,iter->cluster_id);
		}
	}
}