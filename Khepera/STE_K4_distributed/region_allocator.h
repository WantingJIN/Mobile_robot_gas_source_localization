#pragma once

#ifndef REGION_ALLOCATOR_H
#define REGION_ALLOCATOR_H

#include "Posterior.h"
#include "constant_and_helper.h"

#include <utility>
#include <vector>

class RegionAllocator {
private:
    typedef std::pair<int, int> GridPoint;

    Posterior *posterior;
    float (*received_current_pos)[2];
    int belief_clusters[MAP_SIZE] = {0};

    void log_cluster_sizes_geometric(const std::vector<int>& cluster_sizes);
    void log_cluster_sizes_informative(const std::vector<double>& cluster_sizes_info);
    void log_posterior_pdf_3d_cluster();

    bool is_inside_grid(int x, int y) const;
    bool is_active_cell(int idx, bool avoid_obstacles) const;
    int count_active_cells(bool avoid_obstacles) const;
    std::vector<GridPoint> moore_directions() const;
    std::vector<GridPoint> von_neumann_directions() const;
    std::vector<std::pair<double, int>> sorted_active_cells(bool avoid_obstacles) const;
    std::vector<GridPoint> seed_points_from_indices(const std::vector<int>& selected_indices, int count) const;
    std::vector<std::vector<double>> compute_obstacle_aware_distance_maps(const std::vector<GridPoint>& seeds, const std::vector<GridPoint>& directions) const;
    std::vector<int> assign_cells_to_nearest_seed(const std::vector<std::vector<double>>& dist_map, int cluster_count, bool balance_ties, std::vector<int> *cluster_sizes) const;
    std::vector<std::vector<int>> build_pdf_masks(int mask_rows, int cluster_count, int n_cells) const;
    std::vector<double> compute_informative_cluster_sizes(const std::vector<std::vector<int>>& pdf_masks, int cluster_count, int n_cells) const;
    void log_informative_sizes_and_apply_masks(int mask_rows, int cluster_count, int n_cells);
    void apply_pdf_mask_for_current_robot(const std::vector<std::vector<int>>& pdf_masks, int cluster_count, int n_cells);
    double normalize_posterior(int n_cells);
    std::vector<int> best_seed_assignment_for_robots(const std::vector<GridPoint>& seeds) const;

    void clusterize_belief_map_obstacle_dist_VORONOI(const std::vector<int>& selected_indices);
    void clusterize_belief_map_obstacle_dist_LLOYD(const std::vector<int>& selected_indices);
    void clusterize_belief_map_obstacle_dist_REGION_GROWING_D();
    void clusterize_belief_map_obstacle_dist_REGION_GROWING_P();

public:
    RegionAllocator(Posterior *input_posterior, float (*input_received_current_pos)[2]);
    ~RegionAllocator();

    void region_allocation_algorithm();
};

#endif
