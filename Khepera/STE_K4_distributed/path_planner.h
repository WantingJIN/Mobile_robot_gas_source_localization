#pragma once

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include "controller_STE_distributed.h"
#include <unordered_map>
#include "Posterior.h"
#include "PlumeModel.h"
#include <unordered_set>


struct ComparePosition {
      bool operator()(const std::pair<float, Position>& a, const std::pair<float, Position>& b) {
          return a.first > b.first; // Min-heap: prioritize lower cost
      }
};
struct ComparePosition2 {
      bool operator()(const std::pair<float, Position>& a, const std::pair<float, Position>& b) {
          return a.first < b.first; // Min-heap: prioritize higher cost
      }
};


class PathPlanner {
private:
    Posterior *posterior;
    PlumeModel *plume_model;
    struct history *history;

    float gas_information_map[MAP_SIZE] = {0};
    float scores_map[MAP_SIZE] = {0};
    float distance_map[MAP_SIZE] = {0};
    float acc_info_map[MAP_SIZE] = {0}; 

    // helper functions
    void reset_metrics();
    float index_distance(int x_idx_diff, int y_idx_diff);

    //log functions
    void log_scores_map();
    void log_distance_map();
    void log_acc_info_map();
    void log_gas_information_map();
    // functions for Dijkstra
    std::pair<std::vector<float>, std::unordered_map<int, Position>> computeDijkstra(Position start, std::vector<std::tuple<int, int, float>> directions);
    std::pair<std::vector<float>, std::unordered_map<int, Position>> computeMultiDijkstra(Position start, std::vector<std::tuple<int, int, float>> directions);
    std::list<Position> reconstructPath(Position goal, const std::unordered_map<int, Position>& cameFrom);
    void markNeighboringCellsAsVisited(int x, int y, int x_nbins_, int y_nbins_, int radius_neighbor, int *cell_visited_list);
    std::vector<Position> findOptimalPath(Position start, float target[2]);
    //
public:
    
    //////////////////////////////


    PathPlanner(PlumeModel *input_model, Posterior *input_posterior, struct history *input_history);
    ~PathPlanner();

    void maximal_belief_planning();
    void random_navigation();
    bool position_outside_arena(Position p);
    void KLD_informative_goal_selection();
    void random_planning();
    void lawnmower_planning();
    void calculate_information_gain();
    void maximal_information_planning();
    void informative_clustering_planning();
    void belief_clustering_planning();
    void accumulative_Dijkstra();
    std::list<Position> multi_objective_Dijkstra();
    void maximal_merged_belief_planning();

    void calculate_mean_value_within_cluster(const pair<CellPoint, CellPoint> &cluster, float &mean_value, float &min_value, float &max_value);
    float evaluate_cluster(const pair<CellPoint, CellPoint> &cluster);
    void move_to_cluster_centroid(const pair<CellPoint, CellPoint> &cluster, float information_map[]);
    void weighted_belief_clustering();
    void balance_expected_maximal_pdf();
    void joint_planning();
    void KLD_distance_balance();
    void goal_outside_obstacles(Position *goal);
    vector<pair<CellPoint, CellPoint>> clustering_information_blocks(float information_map[]);

};

#endif

