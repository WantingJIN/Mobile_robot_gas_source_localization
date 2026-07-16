#include "region_allocator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>

extern SimulationParameters simulation_parameters;
extern Position current_position;
extern int cell_visited_list[MAP_SIZE];
extern int ID;
extern int self_index;
extern FILE *clustering_debugging_logfile;
extern char region_allocation_logfile_name[512];
extern std::vector<int> obstacle_map;

RegionAllocator::RegionAllocator(Posterior *input_posterior, float (*input_received_current_pos)[2])
{
    posterior = input_posterior;
    received_current_pos = input_received_current_pos;
}

RegionAllocator::~RegionAllocator()
{
}

void RegionAllocator::log_region_allocation_result()
{
    FILE *f = fopen(region_allocation_logfile_name, "a");
    if (!f) return;

    fprintf(f, "%f,%f,", current_position.x, current_position.y);
    for (int y = 0; y < y_nbins; ++y) {
        for (int x = 0; x < x_nbins; ++x) {
            int idx = y * x_nbins + x;
            fprintf(f, "%d", belief_clusters[idx]);
            if (x != x_nbins - 1 || y != y_nbins - 1)
                fprintf(f, ",");
        }
    }
    fprintf(f, "\n");
    fclose(f);
}

bool RegionAllocator::is_inside_grid(int x, int y) const
{
    return x >= 0 && x < x_nbins && y >= 0 && y < y_nbins;
}

bool RegionAllocator::is_active_cell(int idx, bool avoid_obstacles) const
{
    if (idx < 0 || idx >= MAP_SIZE)
        return false;
    if (posterior->pdf[idx] <= simulation_parameters.epsilon_clusterize)
        return false;
    return !avoid_obstacles || obstacle_map.empty() || obstacle_map[idx] == 0;
}

int RegionAllocator::count_active_cells(bool avoid_obstacles) const
{
    int count = 0;
    for (int i = 0; i < MAP_SIZE; ++i)
        if (is_active_cell(i, avoid_obstacles))
            count++;
    return count;
}

std::vector<RegionAllocator::GridPoint> RegionAllocator::moore_directions() const
{
    std::vector<GridPoint> directions;
    directions.push_back(GridPoint(1, 0));
    directions.push_back(GridPoint(-1, 0));
    directions.push_back(GridPoint(0, 1));
    directions.push_back(GridPoint(0, -1));
    directions.push_back(GridPoint(1, 1));
    directions.push_back(GridPoint(1, -1));
    directions.push_back(GridPoint(-1, 1));
    directions.push_back(GridPoint(-1, -1));
    return directions;
}

std::vector<RegionAllocator::GridPoint> RegionAllocator::von_neumann_directions() const
{
    std::vector<GridPoint> directions;
    directions.push_back(GridPoint(1, 0));
    directions.push_back(GridPoint(-1, 0));
    directions.push_back(GridPoint(0, 1));
    directions.push_back(GridPoint(0, -1));
    return directions;
}

std::vector<std::pair<double, int>> RegionAllocator::sorted_active_cells(bool avoid_obstacles) const
{
    std::vector<std::pair<double, int>> cells;
    for (int i = 0; i < MAP_SIZE; ++i)
        if (is_active_cell(i, avoid_obstacles))
            cells.push_back(std::make_pair(static_cast<double>(posterior->pdf[i]), i));
    std::sort(cells.begin(), cells.end(), std::greater<std::pair<double, int>>());
    return cells;
}

std::vector<RegionAllocator::GridPoint> RegionAllocator::seed_points_from_indices(const std::vector<int>& selected_indices, int count) const
{
    std::vector<GridPoint> seeds(count, GridPoint(-1, -1));
    for (int i = 0; i < count && i < static_cast<int>(selected_indices.size()); ++i) {
        if (selected_indices[i] >= 0)
            index_to_indices_2D(selected_indices[i], &seeds[i].first, &seeds[i].second);
    }
    return seeds;
}

std::vector<std::vector<double>> RegionAllocator::compute_obstacle_aware_distance_maps(const std::vector<GridPoint>& seeds, const std::vector<GridPoint>& directions) const
{
    const int K = seeds.size();
    std::vector<std::vector<double>> dist_map(K, std::vector<double>(MAP_SIZE, std::numeric_limits<double>::max()));

    for (int k = 0; k < K; ++k) {
        std::queue<GridPoint> q;
        int sx = seeds[k].first;
        int sy = seeds[k].second;
        if (!is_inside_grid(sx, sy))
            continue;

        int seed_idx = sy * x_nbins + sx;
        dist_map[k][seed_idx] = 0.0;
        q.push(GridPoint(sx, sy));

        while (!q.empty()) {
            GridPoint p = q.front();
            q.pop();
            int idx = p.second * x_nbins + p.first;
            double d = dist_map[k][idx];

            for (size_t i = 0; i < directions.size(); ++i) {
                int nx = p.first + directions[i].first;
                int ny = p.second + directions[i].second;
                if (!is_inside_grid(nx, ny))
                    continue;
                int nidx = ny * x_nbins + nx;
                if (!obstacle_map.empty() && obstacle_map[nidx] == 1)
                    continue;
                double cost = std::sqrt(std::pow(directions[i].first * x_resol, 2) + std::pow(directions[i].second * y_resol, 2));
                if (dist_map[k][nidx] > d + cost) {
                    dist_map[k][nidx] = d + cost;
                    q.push(GridPoint(nx, ny));
                }
            }
        }
    }
    return dist_map;
}

std::vector<int> RegionAllocator::assign_cells_to_nearest_seed(const std::vector<std::vector<double>>& dist_map, int cluster_count, bool balance_ties, std::vector<int> *cluster_sizes) const
{
    std::vector<int> assignments(MAP_SIZE, -1);
    std::vector<int> sizes(cluster_count, 0);

    for (int i = 0; i < MAP_SIZE; ++i) {
        if (!is_active_cell(i, false))
            continue;

        double min_dist = dist_map[0][i];
        int best_k = 0;
        for (int k = 1; k < cluster_count; ++k) {
            double dist = dist_map[k][i];
            if (dist < min_dist || (balance_ties && dist == min_dist && sizes[k] < sizes[best_k])) {
                min_dist = dist;
                best_k = k;
            }
        }
        assignments[i] = best_k;
        sizes[best_k]++;
    }

    if (cluster_sizes)
        *cluster_sizes = sizes;
    return assignments;
}

std::vector<std::vector<int>> RegionAllocator::build_pdf_masks(int mask_rows, int cluster_count, int n_cells) const
{
    std::vector<std::vector<int>> pdf_masks(mask_rows, std::vector<int>(n_cells, 0));
    for (int i = 0; i < n_cells; ++i) {
        if (!is_active_cell(i, false))
            continue;
        for (int k = 0; k < cluster_count && k < mask_rows; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }
    return pdf_masks;
}

std::vector<double> RegionAllocator::compute_informative_cluster_sizes(const std::vector<std::vector<int>>& pdf_masks, int cluster_count, int n_cells) const
{
    std::vector<double> cluster_sizes_info(cluster_count, 0.0);
    for (int k = 0; k < cluster_count && k < static_cast<int>(pdf_masks.size()); ++k)
        for (int i = 0; i < n_cells; ++i)
            cluster_sizes_info[k] += posterior->pdf[i] * pdf_masks[k][i];
    return cluster_sizes_info;
}

void RegionAllocator::apply_pdf_masks(int mask_rows, int cluster_count, int n_cells)
{
    std::vector<std::vector<int>> pdf_masks = build_pdf_masks(mask_rows, cluster_count, n_cells);
    apply_pdf_mask_for_current_robot(pdf_masks, cluster_count, n_cells);
}

void RegionAllocator::apply_pdf_mask_for_current_robot(const std::vector<std::vector<int>>& pdf_masks, int cluster_count, int n_cells)
{
    if (self_index < 0 || self_index >= cluster_count || self_index >= static_cast<int>(pdf_masks.size()))
        return;
    for (int i = 0; i < n_cells; ++i)
        posterior->pdf[i] *= pdf_masks[self_index][i];
}

double RegionAllocator::normalize_posterior(int n_cells)
{
    double sum = std::accumulate(posterior->pdf, posterior->pdf + n_cells, 0.0);
    if (sum > 0.0)
        for (int i = 0; i < n_cells; ++i)
            posterior->pdf[i] /= sum;
    return sum;
}

std::vector<int> RegionAllocator::best_seed_assignment_for_robots(const std::vector<GridPoint>& seeds) const
{
    const int K = seeds.size();
    std::vector<int> perm(K);
    std::iota(perm.begin(), perm.end(), 0);
    std::vector<int> best_perm = perm;
    double best_cost = std::numeric_limits<double>::max();

    do {
        double cost = 0.0;
        for (int k = 0; k < K; ++k) {
            const GridPoint& seed = seeds[perm[k]];
            float cx = 0.0f, cy = 0.0f;
            indices_to_position_2D(seed.first, seed.second, &cx, &cy);
            double dx = received_current_pos[k][0] - cx;
            double dy = received_current_pos[k][1] - cy;
            cost += std::sqrt(dx * dx + dy * dy);
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_perm = perm;
        }
    } while (std::next_permutation(perm.begin(), perm.end()));

    return best_perm;
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_VORONOI(const std::vector<int>& selected_indices)
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    std::vector<GridPoint> seeds = seed_points_from_indices(selected_indices, K);
    std::vector<std::vector<double>> dist_map = compute_obstacle_aware_distance_maps(seeds, moore_directions());
    std::vector<int> cluster_sizes;
    std::vector<int> assignments = assign_cells_to_nearest_seed(dist_map, K, false, &cluster_sizes);

    std::fill(belief_clusters, belief_clusters + N, 0);
    for (int i = 0; i < N; ++i)
        if (assignments[i] >= 0)
            belief_clusters[i] = assignments[i] + 1;

    log_region_allocation_result();
    apply_pdf_masks(K, K, N);
    normalize_posterior(N);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_LLOYD(const std::vector<int>&)
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    const int max_iterations = 10;
    std::vector<std::pair<double, int>> top_cells = sorted_active_cells(false);
    if (top_cells.empty())
        return;

    std::vector<GridPoint> seeds(K, GridPoint(-1, -1));
    for (int i = 0; i < K && i < static_cast<int>(top_cells.size()); ++i)
        index_to_indices_2D(top_cells[i].second, &seeds[i].first, &seeds[i].second);

    std::vector<int> assignments(N, -1);
    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<std::vector<double>> dist_map = compute_obstacle_aware_distance_maps(seeds, moore_directions());
        assignments = assign_cells_to_nearest_seed(dist_map, K, true, nullptr);

        std::vector<double> weighted_sum_x(K, 0.0), weighted_sum_y(K, 0.0), total_weight(K, 0.0);
        for (int i = 0; i < N; ++i) {
            int cluster = assignments[i];
            if (cluster < 0)
                continue;
            double weight = (simulation_parameters.mapCoverageStrategy == LLOYD_D) ? 1.0 : posterior->pdf[i];
            if (weight <= simulation_parameters.epsilon_clusterize)
                continue;
            int x, y;
            index_to_indices_2D(i, &x, &y);
            weighted_sum_x[cluster] += x * weight;
            weighted_sum_y[cluster] += y * weight;
            total_weight[cluster] += weight;
        }

        for (int k = 0; k < K; ++k) {
            if (total_weight[k] > simulation_parameters.epsilon_clusterize) {
                seeds[k].first = static_cast<int>(weighted_sum_x[k] / total_weight[k]);
                seeds[k].second = static_cast<int>(weighted_sum_y[k] / total_weight[k]);
            }
        }
    }

    std::vector<int> best_perm = best_seed_assignment_for_robots(seeds);
    std::vector<int> inv_perm(K);
    for (int k = 0; k < K; ++k)
        inv_perm[best_perm[k]] = k;

    std::vector<int> cluster_sizes(K, 0);
    std::fill(belief_clusters, belief_clusters + N, 0);
    for (int i = 0; i < N; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize)
            continue;
        int cluster = assignments[i];
        if (cluster < 0 || cluster >= K)
            continue;
        int robot_k = inv_perm[cluster];
        belief_clusters[i] = robot_k + 1;
        cluster_sizes[robot_k]++;
    }
    log_region_allocation_result();
    apply_pdf_masks(K, K, N);
    normalize_posterior(N);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_REGION_GROWING_D()
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    std::vector<std::pair<double, int>> top_cells = sorted_active_cells(true);
    if (static_cast<int>(top_cells.size()) < K)
        return;

    std::vector<GridPoint> seeds(K);
    for (int k = 0; k < K; ++k)
        index_to_indices_2D(top_cells[k].second, &seeds[k].first, &seeds[k].second);

    std::vector<int> best_perm = best_seed_assignment_for_robots(seeds);
    std::vector<GridPoint> ordered_seeds(K);
    for (int k = 0; k < K; ++k)
        ordered_seeds[k] = seeds[best_perm[k]];
    seeds = ordered_seeds;

    std::fill(belief_clusters, belief_clusters + N, 0);
    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::vector<std::queue<GridPoint>> queues(K);
    std::vector<int> cluster_sizes(K, 0);
    int target_size = std::max(1, count_active_cells(true) / K);
    int max_size = static_cast<int>(target_size * 1.05);
    std::vector<GridPoint> directions = moore_directions();

    for (int k = 0; k < K; ++k) {
        int sx = seeds[k].first;
        int sy = seeds[k].second;
        int idx = sy * x_nbins + sx;
        if (!is_inside_grid(sx, sy) || !is_active_cell(idx, true))
            continue;
        queues[k].push(GridPoint(sx, sy));
        visited[sy][sx] = true;
        belief_clusters[idx] = k + 1;
        cluster_sizes[k]++;
    }

    bool any_growth = true;
    while (any_growth) {
        any_growth = false;
        std::vector<int> grow_order(K);
        std::iota(grow_order.begin(), grow_order.end(), 0);
        std::sort(grow_order.begin(), grow_order.end(), [&](int a, int b) { return cluster_sizes[a] < cluster_sizes[b]; });

        for (int oi = 0; oi < K; ++oi) {
            int k = grow_order[oi];
            if (cluster_sizes[k] >= max_size || queues[k].empty())
                continue;

            int frontier_size = queues[k].size();
            for (int qi = 0; qi < frontier_size && cluster_sizes[k] < max_size; ++qi) {
                GridPoint p = queues[k].front();
                queues[k].pop();
                for (size_t d = 0; d < directions.size() && cluster_sizes[k] < max_size; ++d) {
                    int nx = p.first + directions[d].first;
                    int ny = p.second + directions[d].second;
                    if (!is_inside_grid(nx, ny) || visited[ny][nx])
                        continue;
                    int nidx = ny * x_nbins + nx;
                    if (!is_active_cell(nidx, true))
                        continue;
                    visited[ny][nx] = true;
                    belief_clusters[nidx] = k + 1;
                    queues[k].push(GridPoint(nx, ny));
                    cluster_sizes[k]++;
                    any_growth = true;
                }
            }
        }
    }

    log_region_allocation_result();
    apply_pdf_masks(K, K, N);
    normalize_posterior(N);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_REGION_GROWING_P()
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    std::vector<std::pair<double, int>> top_cells = sorted_active_cells(true);
    if (static_cast<int>(top_cells.size()) < K)
        return;

    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::vector<std::queue<GridPoint>> queues(K);
    std::vector<int> cluster_sizes(K, 0);
    std::fill(belief_clusters, belief_clusters + N, 0);

    for (int k = 0; k < K; ++k) {
        int sx, sy;
        index_to_indices_2D(top_cells[k].second, &sx, &sy);
        queues[k].push(GridPoint(sx, sy));
        visited[sy][sx] = true;
        belief_clusters[sy * x_nbins + sx] = k + 1;
        cluster_sizes[k]++;
    }

    int target_size = std::max(1, count_active_cells(true) / K);
    int max_size = static_cast<int>(target_size * 1.2);
    std::vector<GridPoint> directions = von_neumann_directions();
    bool growing = true;

    while (growing) {
        growing = false;
        for (int k = 0; k < K; ++k) {
            if (cluster_sizes[k] >= max_size || queues[k].empty())
                continue;

            GridPoint p = queues[k].front();
            queues[k].pop();
            int best_nx = -1, best_ny = -1;
            double best_pdf = -1.0;
            for (size_t d = 0; d < directions.size(); ++d) {
                int nx = p.first + directions[d].first;
                int ny = p.second + directions[d].second;
                if (!is_inside_grid(nx, ny) || visited[ny][nx])
                    continue;
                int idx = ny * x_nbins + nx;
                if (!is_active_cell(idx, true))
                    continue;
                if (posterior->pdf[idx] > best_pdf) {
                    best_pdf = posterior->pdf[idx];
                    best_nx = nx;
                    best_ny = ny;
                }
            }
            if (best_nx != -1) {
                int idx = best_ny * x_nbins + best_nx;
                visited[best_ny][best_nx] = true;
                belief_clusters[idx] = k + 1;
                queues[k].push(GridPoint(best_nx, best_ny));
                cluster_sizes[k]++;
                growing = true;
            }
        }
    }

    log_region_allocation_result();
    apply_pdf_masks(K, K, N);
    normalize_posterior(N);
}

void RegionAllocator::region_allocation_algorithm()
{
    if (simulation_parameters.mapCoverageStrategy == NONE)
        return;

    const int K = simulation_parameters.number_of_robots;
    std::vector<int> selected_indices;

    if (simulation_parameters.mapCoverageSeed == ROBOT_CELL) {
        selected_indices.resize(K);
        for (int i = 0; i < K; ++i)
            selected_indices[i] = position_to_index_2D(received_current_pos[i][0], received_current_pos[i][1]);
    }
    else {
        std::vector<std::pair<double, int>> top_cells = sorted_active_cells(false);
        selected_indices.assign(K, -1);
        for (int i = 0; i < K && i < static_cast<int>(top_cells.size()); ++i)
            selected_indices[i] = top_cells[i].second;
    }

    for (size_t i = 0; i < selected_indices.size(); ++i)
        if (selected_indices[i] >= 0)
            cell_visited_list[selected_indices[i]] = 1;

    if (simulation_parameters.mapCoverageStrategy == VORONOI)
        clusterize_belief_map_obstacle_dist_VORONOI(selected_indices);
    else if (simulation_parameters.mapCoverageStrategy == LLOYD_D || simulation_parameters.mapCoverageStrategy == LLOYD_P)
        clusterize_belief_map_obstacle_dist_LLOYD(selected_indices);
    else if (simulation_parameters.mapCoverageStrategy == REGION_GROWING_P)
        clusterize_belief_map_obstacle_dist_REGION_GROWING_P();
    else if (simulation_parameters.mapCoverageStrategy == REGION_GROWING_D)
        clusterize_belief_map_obstacle_dist_REGION_GROWING_D();

    if (clustering_debugging_logfile) {
        fprintf(clustering_debugging_logfile, "Robot ID [%d]: Region allocation complete, self index %d\n", ID, self_index);
        fflush(clustering_debugging_logfile);
    }
}
