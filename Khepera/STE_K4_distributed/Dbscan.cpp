#include "Dbscan.h"
#include <stack>
#include "constant_and_helper.h"
#include <cstring>

using namespace std;
extern char clustering_bounding_boxes_logfile_name[512];
FILE *clustering_bounding_boxes_logfile;
DBSCAN::DBSCAN(float x_eps, float y_eps, unsigned int minPts, double rat)
{
	x_epsilon = x_eps;
	y_epsilon = y_eps;
	minPoints = minPts;
	ratio = rat > 0 ? rat : 0;
}

vector<int> DBSCAN::findNeighbors(const CellPoint cell)
{
	int index = 0;
	vector<CellPoint>::iterator iter;
	vector<int> clusterIndex;
	for (iter = cell_list.begin(); iter != cell_list.end(); ++iter)
	{
		if (isWithinDistance(cell, *iter) == 1)
		{
			clusterIndex.push_back(index);
		}
		index++;
	}
	return clusterIndex;
}

void DBSCAN::expandCluster(CellPoint &cell, vector<int> &neighbors, int clusterId)
{
	int index = 0, indexCorePoint = 0;
	vector<int>::iterator iterSeeds;
	for (iterSeeds = neighbors.begin(); iterSeeds != neighbors.end(); ++iterSeeds)
	{
		cell_list.at(*iterSeeds).cluster_id = clusterId;
		if (cell_list.at(*iterSeeds).x == cell.x && cell_list.at(*iterSeeds).y == cell.y)
		{
			indexCorePoint = index;
			break;
		}
		++index;
	}
	neighbors.erase(neighbors.begin() + indexCorePoint);

	for (vector<int>::size_type i = 0, n = neighbors.size(); i < n; ++i)
	{
		vector<int> clusterNeighors = findNeighbors(cell_list.at(neighbors[i]));

		if (clusterNeighors.size() >= minPoints)
		{
			vector<int>::iterator iterNeighors;
			for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
			{
				if (cell_list.at(*iterNeighors).cluster_id == UNCLASSIFIED || cell_list.at(*iterNeighors).cluster_id == NOISE)
				{
					if (cell_list.at(*iterNeighors).cluster_id == UNCLASSIFIED)
					{
						neighbors.push_back(*iterNeighors);
						n = neighbors.size();
					}
					cell_list.at(*iterNeighors).cluster_id = clusterId;
				}
			}
		}
	}
	//printf("Exit expandCluster \n");
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

void DBSCAN::run(double information[])
{
	double max_info = 0;
	double min_info = 100000;
	for (int i = 0; i < MAP_SIZE; i++)
	{
		if (information[i] > max_info)
		{
			max_info = information[i];
		}
		if (information[i] < min_info)
		{
			min_info = information[i];
		}
	}
	// first fill y then fill x
	for (int j = 0; j < y_nbins; j++)
	{
		for (int i = 0; i < x_nbins; i++)
		{
			if (information[j * y_nbins + i] > ratio * (max_info - min_info) + min_info)
			{
				CellPoint cell;
				indices_to_position_2D(i, j, &cell.x, &cell.y);
				cell.cluster_id = UNCLASSIFIED;
				cell_list.push_back(cell);
			}
		}
	}
	pointSize = cell_list.size();
	int clusterId = 0;
	for (vector<CellPoint>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++)
	{
		if (iter->cluster_id == UNCLASSIFIED)
		{
			vector<int> neighbors = findNeighbors(*iter);
			if (neighbors.size() < minPoints)
			{
				iter->cluster_id = NOISE;
			}
			else
			{
				expandCluster(*iter, neighbors, clusterId);
				clusterId++;
			}
		}
	}
	cluster_num = clusterId;
	// print clustering results
	printResults();
	// calcalate cluster bounding boxes and log it
	calculateBoundingBoxes();
	logBoundingBoxes();
}

inline bool DBSCAN::isWithinDistance(CellPoint cellCore, CellPoint cellTarget)
{
	return abs(cellCore.x - cellTarget.x) <= x_epsilon && abs(cellCore.y - cellTarget.y) <= y_epsilon;
}

void DBSCAN::printResults()
{
	printf("Number of clusters found: %d \n", cluster_num);
	printf("Clustered cells: \n");
	for (vector<CellPoint>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++)
	{
		if (iter->cluster_id != NOISE)
		{
			printf("Cell (%f, %f) @ cluster %d \n", iter->x, iter->y, iter->cluster_id + 1);
		}
	}
}

pair<CellPoint, CellPoint> DBSCAN::calculateBoundingBox(const std::vector<CellPoint> &points, const int clusterId)
{
	float minX = numeric_limits<float>::max();
	float minY = numeric_limits<float>::max();
	float maxX = numeric_limits<float>::min();
	float maxY = numeric_limits<float>::min();

	for (auto &point : points)
	{
		minX = min(minX, point.x - float(x_resol / 2));
		minY = min(minY, point.y - float(y_resol / 2));
		maxX = max(maxX, point.x + float(x_resol / 2));
		maxY = max(maxY, point.y + float(y_resol / 2));
	}

	CellPoint topLeft = {minX, minY, clusterId};
	CellPoint bottomRight = {maxX, maxY, clusterId};
	return std::make_pair(topLeft, bottomRight);
}

void DBSCAN::calculateBoundingBoxes()
{
	for (int i = 0; i < cluster_num; i++)
	{
		vector<CellPoint> points;
		for (vector<CellPoint>::iterator iter = cell_list.begin(); iter != cell_list.end(); iter++)
		{
			if (iter->cluster_id == i)
			{
				points.push_back(*iter);
			}
		}
		cluster_bounding_boxes.push_back(calculateBoundingBox(points, i));
	}
}

void DBSCAN::logBoundingBoxes()
{
	clustering_bounding_boxes_logfile = fopen(clustering_bounding_boxes_logfile_name, "a");

	for (vector<pair<CellPoint, CellPoint>>::iterator iter = cluster_bounding_boxes.begin(); iter != cluster_bounding_boxes.end(); iter++)
	{
		fprintf(clustering_bounding_boxes_logfile, "%f, %f, %f, %f, ", iter->first.x, iter->first.y, iter->second.x, iter->second.y);
		printf("Cluster %d: (%f, %f) - (%f, %f) \n", iter->first.cluster_id, iter->first.x, iter->first.y, iter->second.x, iter->second.y);
	}
	fprintf(clustering_bounding_boxes_logfile, "\n");
	fclose(clustering_bounding_boxes_logfile);
}