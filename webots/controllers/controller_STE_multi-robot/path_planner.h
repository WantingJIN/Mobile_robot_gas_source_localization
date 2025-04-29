#pragma once

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include "controller_STE_multi-robot.h"
#include "Posterior.h"
#include "PlumeModel.h"

extern Position goal_position;
extern Position current_position;
extern SimulationParameters simulation_parameters;
extern int cell_visited_list[x_nbins * y_nbins];


extern char gas_information_map_logfile_name[512];
extern char scores_map_logfile_name[512];
extern char distance_map_logfile_name[512];
extern char acc_info_map_logfile_name[512];

FILE *gas_information_map_logfile;
FILE *scores_map_logfile;
FILE *distance_map_logfile;
FILE *acc_info_map_logfile;


// text file for storing some prints for debugging
extern FILE *console_logfile;



struct ComparePosition {
      bool operator()(const std::pair<double, Position>& a, const std::pair<double, Position>& b) {
          return a.first > b.first; // Min-heap: prioritize lower cost
      }
};
struct ComparePosition2 {
      bool operator()(const std::pair<double, Position>& a, const std::pair<double, Position>& b) {
          return a.first < b.first; // Min-heap: prioritize higher cost
      }
};


class PathPlanner {
private:
    Posterior *posterior;
    PlumeModel *plume_model;
    struct history *history;

    double gas_information_map[x_nbins * y_nbins] = {0};
    double scores_map[x_nbins * y_nbins] = {0};
    double distance_map[x_nbins * y_nbins] = {0};
    double acc_info_map[x_nbins * y_nbins] = {0}; 

    // helper functions
    void reset_metrics();
    double index_distance(int x_idx_diff, int y_idx_diff);

    //log functions
    void log_scores_map();
    void log_distance_map();
    void log_acc_info_map();
    void log_gas_information_map();
    // functions for Dijkstra
    std::pair<std::vector<double>, std::unordered_map<int, Position>> computeDijkstra(Position start, std::vector<std::tuple<int, int, double>> directions);
    std::pair<std::vector<double>, std::unordered_map<int, Position>> computeMultiDijkstra(Position start, std::vector<std::tuple<int, int, double>> directions);
    std::list<Position> reconstructPath(Position goal, const std::unordered_map<int, Position>& cameFrom);
    void markNeighboringCellsAsVisited(int x, int y, int x_nbins_, int y_nbins_, int radius_neighbor, int *cell_visited_list);
    std::vector<Position> findOptimalPath(Position start, double target[2]);
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
    void calculate_mean_value_within_cluster(const pair<CellPoint, CellPoint> &cluster, double &mean_value, double &min_value, double &max_value);
    double evaluate_cluster(const pair<CellPoint, CellPoint> &cluster);
    void move_to_cluster_centroid(const pair<CellPoint, CellPoint> &cluster, double information_map[]);
    void weighted_belief_clustering();
    void balance_expected_maximal_pdf();
    void joint_planning();
    void KLD_distance_balance();
    void goal_outside_obstacles(Position *goal);
    vector<pair<CellPoint, CellPoint>> clustering_information_blocks(double information_map[]);

};

#endif

