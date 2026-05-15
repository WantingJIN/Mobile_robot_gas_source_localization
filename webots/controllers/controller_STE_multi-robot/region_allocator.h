#pragma once

#ifndef REGION_ALLOCATOR_H
#define REGION_ALLOCATOR_H

#include "Posterior.h"
#include "constant_and_helper.h"

#include <vector>

class RegionAllocator {
private:
    Posterior *posterior;
    float (*received_current_pos)[2];
    int belief_clusters[MAP_SIZE] = {0};

    void log_cluster_sizes_geometric(const std::vector<int>& cluster_sizes);
    void log_cluster_sizes_informative(const std::vector<double>& cluster_sizes_info);
    void log_posterior_pdf_3d_cluster();

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
