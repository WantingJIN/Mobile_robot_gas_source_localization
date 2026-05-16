#include "region_allocator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <utility>

extern SimulationParameters simulation_parameters;
extern Position current_position;
extern int cell_visited_list[MAP_SIZE];
extern int ID;
extern FILE *clustering_debugging_logfile;
extern char posterior_pdf_3D_cluster_name[512];
extern char cluster_sizes_geometric_name[512];
extern char cluster_sizes_informative_name[512];
extern std::vector<int> obstacle_map;

RegionAllocator::RegionAllocator(Posterior *input_posterior, float (*input_received_current_pos)[2])
{
    posterior = input_posterior;
    received_current_pos = input_received_current_pos;
}

RegionAllocator::~RegionAllocator()
{
}

void RegionAllocator::log_cluster_sizes_geometric(const std::vector<int>& cluster_sizes)
{
    FILE *cluster_sizes_geometric = fopen(cluster_sizes_geometric_name, "a");
    if (!cluster_sizes_geometric) return;

    for (size_t i = 0; i < cluster_sizes.size(); ++i) {
        fprintf(cluster_sizes_geometric, "%d", cluster_sizes[i]);
        if (i != cluster_sizes.size() - 1)
            fprintf(cluster_sizes_geometric, ",");
    }

    fprintf(cluster_sizes_geometric, "\n");
    fclose(cluster_sizes_geometric);
}

void RegionAllocator::log_cluster_sizes_informative(const std::vector<double>& cluster_sizes_info)
{
    FILE *cluster_sizes_informative = fopen(cluster_sizes_informative_name, "a");
    if (!cluster_sizes_informative) return;

    for (size_t i = 0; i < cluster_sizes_info.size(); ++i) {
        fprintf(cluster_sizes_informative, "%f", cluster_sizes_info[i]);
        if (i != cluster_sizes_info.size() - 1)
            fprintf(cluster_sizes_informative, ",");
    }

    fprintf(cluster_sizes_informative, "\n");
    fclose(cluster_sizes_informative);
}

void RegionAllocator::log_posterior_pdf_3d_cluster()
{
    FILE *posterior_pdf_3D_cluster = fopen(posterior_pdf_3D_cluster_name, "a");
    if (!posterior_pdf_3D_cluster) return;

    fprintf(posterior_pdf_3D_cluster, "%f,%f,", current_position.x, current_position.y);

    for (int y = 0; y < source_y_nbins; ++y) {
        for (int x = 0; x < source_x_nbins; ++x) {
            int idx = y * source_x_nbins + x;
            fprintf(posterior_pdf_3D_cluster, "%d", belief_clusters[idx]);

            if (x != source_x_nbins - 1 || y != source_y_nbins - 1)
                fprintf(posterior_pdf_3D_cluster, ",");
        }
    }

    fprintf(posterior_pdf_3D_cluster, "\n");
    fclose(posterior_pdf_3D_cluster);
}

bool RegionAllocator::is_inside_grid(int x, int y) const
{
    return x >= 0 && x < source_x_nbins && y >= 0 && y < source_y_nbins;
}

bool RegionAllocator::is_active_cell(int idx, bool avoid_obstacles) const
{
    if (idx < 0 || idx >= MAP_SIZE)
        return false;
    if (posterior->pdf[idx] <= simulation_parameters.epsilon_clusterize)
        return false;
    return !avoid_obstacles || obstacle_map[idx] == 0;
}

int RegionAllocator::count_active_cells(bool avoid_obstacles) const
{
    int count = 0;
    for (int i = 0; i < MAP_SIZE; ++i)
        if (is_active_cell(i, avoid_obstacles))
            count++;
    return count;
}

const std::vector<RegionAllocator::GridPoint>& RegionAllocator::moore_directions() const
{
    static const std::vector<GridPoint> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    return directions;
}

const std::vector<RegionAllocator::GridPoint>& RegionAllocator::von_neumann_directions() const
{
    static const std::vector<GridPoint> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };
    return directions;
}

std::vector<std::pair<double, int>> RegionAllocator::sorted_active_cells(bool avoid_obstacles) const
{
    std::vector<std::pair<double, int>> top_cells;
    for (int i = 0; i < MAP_SIZE; ++i)
        if (is_active_cell(i, avoid_obstacles))
            top_cells.emplace_back(posterior->pdf[i], i);

    std::sort(top_cells.begin(), top_cells.end(), std::greater<>());
    return top_cells;
}

std::vector<RegionAllocator::GridPoint> RegionAllocator::seed_points_from_indices(const std::vector<int>& selected_indices, int count) const
{
    std::vector<GridPoint> seeds(count, {-1, -1});

    for (int i = 0; i < count && i < (int)selected_indices.size(); ++i) {
        if (selected_indices[i] >= 0)
            index_to_indices_2D(selected_indices[i], &seeds[i].first, &seeds[i].second);
    }

    return seeds;
}

std::vector<std::vector<double>> RegionAllocator::compute_obstacle_aware_distance_maps(const std::vector<GridPoint>& seeds, const std::vector<GridPoint>& directions) const
{
    const int K = seeds.size();
    const double cell_size_x = (XMAX - XMIN) / static_cast<double>(source_x_nbins);
    const double cell_size_y = (YMAX - YMIN) / static_cast<double>(source_y_nbins);
    std::vector<std::vector<double>> dist_map(K, std::vector<double>(MAP_SIZE, std::numeric_limits<double>::max()));

    for (int k = 0; k < K; ++k) {
        std::queue<GridPoint> q;
        int sx = seeds[k].first;
        int sy = seeds[k].second;

        if (!is_inside_grid(sx, sy))
            continue;

        int seed_idx = sy * source_x_nbins + sx;
        dist_map[k][seed_idx] = 0.0;
        q.push({sx, sy});

        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();

            int idx = y * source_x_nbins + x;
            double d = dist_map[k][idx];

            for (const auto& [dx, dy] : directions) {
                int nx = x + dx;
                int ny = y + dy;

                if (!is_inside_grid(nx, ny))
                    continue;

                int nidx = ny * source_x_nbins + nx;
                if (obstacle_map[nidx] == 1)
                    continue;

                double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));
                if (dist_map[k][nidx] > d + cost) {
                    dist_map[k][nidx] = d + cost;
                    q.push({nx, ny});
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

    for (int k = 0; k < cluster_count && k < (int)pdf_masks.size(); ++k)
        for (int i = 0; i < n_cells; ++i)
            cluster_sizes_info[k] += posterior->pdf[i] * pdf_masks[k][i];

    return cluster_sizes_info;
}

void RegionAllocator::log_informative_sizes_and_apply_masks(int mask_rows, int cluster_count, int n_cells)
{
    std::vector<std::vector<int>> pdf_masks = build_pdf_masks(mask_rows, cluster_count, n_cells);
    log_cluster_sizes_informative(compute_informative_cluster_sizes(pdf_masks, mask_rows, n_cells));
    apply_pdf_mask_for_current_robot(pdf_masks, cluster_count, n_cells);
}

void RegionAllocator::apply_pdf_mask_for_current_robot(const std::vector<std::vector<int>>& pdf_masks, int cluster_count, int n_cells)
{
    for (int k = 0; k < cluster_count && k < (int)pdf_masks.size(); ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < n_cells; ++i)
                posterior->pdf[i] *= pdf_masks[k][i];
        }
    }
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
            double cx = XMIN + (seed.first + 0.5) * (XMAX - XMIN) / source_x_nbins;
            double cy = YMIN + (seed.second + 0.5) * (YMAX - YMIN) / source_y_nbins;
            double ddx = received_current_pos[k][0] - cx;
            double ddy = received_current_pos[k][1] - cy;
            cost += std::sqrt(ddx * ddx + ddy * ddy);
        }

        if (cost < best_cost) {
            best_cost = cost;
            best_perm = perm;
        }
    } while (std::next_permutation(perm.begin(), perm.end()));

    return best_perm;
}

void RegionAllocator::log_cluster_map_debug_for_current_robot(int cluster_count)
{
    for (int k = 0; k < cluster_count; ++k) {
        if (ID == 101 + k) {
            fprintf(clustering_debugging_logfile, "\n Clusterized Belief Map (balanceado):\n");
            for (int y = 0; y < source_y_nbins; ++y) {
                for (int x = 0; x < source_x_nbins; ++x) {
                    int idx = y * source_x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%d ", belief_clusters[idx]);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }
        }
    }
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_VORONOI(const std::vector<int>& selected_indices)
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    std::vector<GridPoint> seeds = seed_points_from_indices(selected_indices, K);
    std::vector<std::vector<double>> dist_map = compute_obstacle_aware_distance_maps(seeds, moore_directions());
    std::vector<int> cluster_sizes;
    std::vector<int> cluster_assignments = assign_cells_to_nearest_seed(dist_map, K, false, &cluster_sizes);

    std::fill(belief_clusters, belief_clusters + N, 0);
    for (int i = 0; i < N; ++i)
        if (cluster_assignments[i] >= 0)
            belief_clusters[i] = cluster_assignments[i] + 1;

    log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster();
    log_informative_sizes_and_apply_masks(K, K, N);

    fprintf(clustering_debugging_logfile, "Robot ID [%d]: NO Normalized Individual pdf after mask:\n", ID);
    for (int y = 0; y < source_y_nbins; ++y) {
        for (int x = 0; x < source_x_nbins; ++x) {
            int idx = y * source_x_nbins + x;
            fprintf(clustering_debugging_logfile, "%.9f ", posterior->pdf[idx]);
        }
        fprintf(clustering_debugging_logfile, "\n");
    }

    normalize_posterior(N);
    double sum_def = std::accumulate(posterior->pdf, posterior->pdf + N, 0.0);
    printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

    fprintf(clustering_debugging_logfile, "Robot ID [%d]: Normalized Individual pdf after mask:\n", ID);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_LLOYD(const std::vector<int>&)
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    const int max_iterations = 10;

    std::vector<std::pair<double, int>> top_cells_lloyd = sorted_active_cells(false);
    std::vector<GridPoint> seeds(K);
    for (int i = 0; i < K && i < (int)top_cells_lloyd.size(); ++i)
        index_to_indices_2D(top_cells_lloyd[i].second, &seeds[i].first, &seeds[i].second);

    std::vector<std::vector<double>> dist_map = compute_obstacle_aware_distance_maps(seeds, moore_directions());

    std::vector<int> cluster_assignments(N, -1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        cluster_assignments = assign_cells_to_nearest_seed(dist_map, K, true, nullptr);

        std::vector<double> weighted_sum_x(K, 0.0), weighted_sum_y(K, 0.0), total_weight(K, 0.0);
        for (int i = 0; i < N; ++i) {
            int cluster = cluster_assignments[i];
            if (cluster < 0) continue;

            double weight = 0.0;

            if (simulation_parameters.mapCoverageStrategy == LLOYD_D) {
                weight = 1;
            }
            else if (simulation_parameters.mapCoverageStrategy == LLOYD_P) {
                weight = posterior->pdf[i];
            }

            if (weight < simulation_parameters.epsilon_clusterize) continue;

            int x, y;
            index_to_indices_2D(i, &x, &y);
            weighted_sum_x[cluster] += x * weight;
            weighted_sum_y[cluster] += y * weight;
            total_weight[cluster] += weight;
        }

        for (int k = 0; k < K; ++k) {
            if (total_weight[k] > simulation_parameters.epsilon_clusterize) {
                seeds[k].first  = static_cast<int>(weighted_sum_x[k] / total_weight[k]);
                seeds[k].second = static_cast<int>(weighted_sum_y[k] / total_weight[k]);
            }
        }

        dist_map = compute_obstacle_aware_distance_maps(seeds, moore_directions());

        for (int i = 0; i < N; ++i)
            belief_clusters[i] = (posterior->pdf[i] > simulation_parameters.epsilon_clusterize) ? cluster_assignments[i] + 1 : 0;

        log_posterior_pdf_3d_cluster();
    }

    std::vector<int> best_perm = best_seed_assignment_for_robots(seeds);

    std::vector<int> inv_perm(K);
    for (int k = 0; k < K; ++k) inv_perm[best_perm[k]] = k;

    std::vector<int> cluster_sizes(K, 0);
    std::fill(belief_clusters, belief_clusters + N, 0);

    for (int i = 0; i < N; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        int cluster = cluster_assignments[i];
        int robot_k = inv_perm[cluster];
        belief_clusters[i] = robot_k + 1;
        cluster_sizes[robot_k]++;
    }

    log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster();
    log_informative_sizes_and_apply_masks(K, K, N);

    double sum = normalize_posterior(N);
    printf("Robot ID [%d]: Sum of Lloyd-obstacle-aware pdf is %f \n", ID, sum);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_REGION_GROWING_D()
{
    const int K = simulation_parameters.number_of_robots;
    const int N = MAP_SIZE;
    std::vector<std::pair<double, int>> top_cells = sorted_active_cells(true);

    if ((int)top_cells.size() < K) {
        printf("Robot ID [%d]: Not enough valid seed cells for balanced region growing.\n", ID);
        return;
    }

    std::vector<GridPoint> seeds(K);

    for (int k = 0; k < K; ++k)
        index_to_indices_2D(top_cells[k].second, &seeds[k].first, &seeds[k].second);

    std::vector<int> best_perm = best_seed_assignment_for_robots(seeds);
    std::vector<GridPoint> ordered_seeds(K);
    for (int k = 0; k < K; ++k) {
        ordered_seeds[k] = seeds[best_perm[k]];
    }
    seeds = ordered_seeds;

    std::fill(belief_clusters, belief_clusters + N, 0);

    const std::vector<GridPoint> all_directions = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1},
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    std::vector<std::vector<bool>> visited(source_y_nbins, std::vector<bool>(source_x_nbins, false));
    std::vector<std::queue<std::pair<int, int>>> queues(K);
    std::vector<int> cluster_sizes(K, 0);

    int total_active_cells = count_active_cells(true);

    int target_size = std::max(1, total_active_cells / K);
    int max_size_margin = static_cast<int>(target_size * 1.05);

    for (int k = 0; k < K; ++k) {
        int sx = seeds[k].first;
        int sy = seeds[k].second;
        int idx = sy * source_x_nbins + sx;

        if (!is_inside_grid(sx, sy))
            continue;

        if (posterior->pdf[idx] <= simulation_parameters.epsilon_clusterize)
            continue;

        if (obstacle_map[idx] == 1)
            continue;

        queues[k].push({sx, sy});
        visited[sy][sx] = true;
        belief_clusters[idx] = k + 1;
        cluster_sizes[k]++;
    }

    bool any_growth = true;

    while (any_growth) {
        any_growth = false;

        std::vector<int> grow_order(K);
        std::iota(grow_order.begin(), grow_order.end(), 0);

        std::sort(grow_order.begin(), grow_order.end(),
            [&](int a, int b) {
                return cluster_sizes[a] < cluster_sizes[b];
            }
        );

        for (int kk = 0; kk < K; ++kk) {
            int k = grow_order[kk];

            if (cluster_sizes[k] >= max_size_margin)
                continue;

            if (queues[k].empty())
                continue;

            int current_frontier_size = queues[k].size();

            for (int q_i = 0; q_i < current_frontier_size; ++q_i) {
                if (cluster_sizes[k] >= max_size_margin)
                    break;

                auto [x, y] = queues[k].front();
                queues[k].pop();

                for (const auto& [dx, dy] : all_directions) {
                    if (cluster_sizes[k] >= max_size_margin)
                        break;

                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= source_x_nbins || ny < 0 || ny >= source_y_nbins)
                        continue;

                    if (visited[ny][nx])
                        continue;

                    int nidx = ny * source_x_nbins + nx;

                    if (obstacle_map[nidx] == 1)
                        continue;

                    if (posterior->pdf[nidx] <= simulation_parameters.epsilon_clusterize)
                        continue;

                    visited[ny][nx] = true;
                    belief_clusters[nidx] = k + 1;
                    queues[k].push({nx, ny});
                    cluster_sizes[k]++;

                    any_growth = true;
                }
            }
        }
    }

    log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster();
    log_informative_sizes_and_apply_masks(K, K, N);

    normalize_posterior(N);
    double sum_def = std::accumulate(posterior->pdf, posterior->pdf + N, 0.0);

    printf("Robot ID [%d]: Sum of balanced 2D region-growing clusterized pdf is %f \n",
           ID, sum_def);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_REGION_GROWING_P()
{
    constexpr int K = 3;
    const int N = MAP_SIZE;
    std::vector<std::pair<double, int>> top_cells = sorted_active_cells(false);

    std::vector<int> seeds_x(K), seeds_y(K);
    for (int i = 0; i < K && i < (int)top_cells.size(); ++i)
        index_to_indices_2D(top_cells[i].second, &seeds_x[i], &seeds_y[i]);

    std::fill(belief_clusters, belief_clusters + N, 0);

    std::vector<std::vector<bool>> visited(source_y_nbins, std::vector<bool>(source_x_nbins, false));
    std::queue<GridPoint> queues[K];
    int cluster_sizes[K] = {0, 0, 0};

    int target_size = count_active_cells(true) / K;
    int max_size_margin = target_size * 1.2;

    for (int k = 0; k < K; ++k) {
        queues[k].push({seeds_x[k], seeds_y[k]});
        visited[seeds_y[k]][seeds_x[k]] = true;
        belief_clusters[seeds_y[k] * source_x_nbins + seeds_x[k]] = k + 1;
        cluster_sizes[k]++;
    }

    bool growing = true;
    while (growing) {
        growing = false;

        for (int k = 0; k < K; ++k) {
            if (cluster_sizes[k] >= max_size_margin) continue;

            if (queues[k].empty()) goto stop_region_growing;

            auto [x, y] = queues[k].front(); queues[k].pop();

            int best_nx = -1, best_ny = -1;
            double best_pdf = -1.0;

            for (const auto& [dx, dy] : von_neumann_directions()) {
                int nx = x + dx, ny = y + dy;
                if (!is_inside_grid(nx, ny)) continue;
                if (obstacle_map[ny * source_x_nbins + nx] == 1) continue;
                if (visited[ny][nx]) continue;

                int idx = ny * source_x_nbins + nx;
                if (posterior->pdf[idx] <= simulation_parameters.epsilon_clusterize) continue;

                if (posterior->pdf[idx] > best_pdf) {
                    best_pdf = posterior->pdf[idx];
                    best_nx = nx;
                    best_ny = ny;
                }
            }

            if (best_nx != -1 && best_ny != -1) {
                int idx = best_ny * source_x_nbins + best_nx;
                visited[best_ny][best_nx] = true;
                belief_clusters[idx] = k + 1;
                queues[k].push({best_nx, best_ny});
                cluster_sizes[k]++;
                growing = true;
            }
        }
    }
stop_region_growing:

    log_cluster_sizes_geometric(std::vector<int>(cluster_sizes, cluster_sizes + K));
    log_posterior_pdf_3d_cluster();
    log_informative_sizes_and_apply_masks(simulation_parameters.number_of_robots, K, N);

    normalize_posterior(N);
    double sum_def = std::accumulate(posterior->pdf, posterior->pdf + N, 0.0);
    printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

    log_cluster_map_debug_for_current_robot(K);
}

void RegionAllocator::region_allocation_algorithm()
{
    std::vector<int> selected_indices;

    if (simulation_parameters.mapCoverageSeed == ROBOT_CELL) {
        selected_indices.resize(simulation_parameters.number_of_robots);

        for (int i = 0; i < simulation_parameters.number_of_robots; ++i) {
            float x = received_current_pos[i][0];
            float y = received_current_pos[i][1];
            selected_indices[i] = position_to_index_2D(x, y);
        }
    }
    else if (simulation_parameters.mapCoverageSeed == MAX_CELL) {
        constexpr int N_SEEDS = 3;
        std::vector<std::pair<double,int>> top_cells = sorted_active_cells(false);

        selected_indices.resize(N_SEEDS, -1);

        for (int i = 0; i < N_SEEDS && i < (int)top_cells.size(); ++i)
            selected_indices[i] = top_cells[i].second;
    }

    for (int idx : selected_indices)
        if (idx >= 0) cell_visited_list[idx] = 1;

    if (simulation_parameters.mapCoverageStrategy == VORONOI) {
        clusterize_belief_map_obstacle_dist_VORONOI(selected_indices);
    }

    if (simulation_parameters.mapCoverageStrategy == LLOYD_D || simulation_parameters.mapCoverageStrategy == LLOYD_P) {
        clusterize_belief_map_obstacle_dist_LLOYD(selected_indices);
    }

    if (simulation_parameters.mapCoverageStrategy == REGION_GROWING_P)
        clusterize_belief_map_obstacle_dist_REGION_GROWING_P();
    if (simulation_parameters.mapCoverageStrategy == REGION_GROWING_D)
        clusterize_belief_map_obstacle_dist_REGION_GROWING_D();
}
