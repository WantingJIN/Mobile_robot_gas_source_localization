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

    for (int y = 0; y < y_nbins; ++y) {
        for (int x = 0; x < x_nbins; ++x) {
            int idx = y * x_nbins + x;
            fprintf(posterior_pdf_3D_cluster, "%d", belief_clusters[idx]);

            if (x != x_nbins - 1 || y != y_nbins - 1)
                fprintf(posterior_pdf_3D_cluster, ",");
        }
    }

    fprintf(posterior_pdf_3D_cluster, "\n");
    fclose(posterior_pdf_3D_cluster);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_VORONOI(const std::vector<int>& selected_indices)
{
    std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots, std::vector<int>(x_nbins * y_nbins, 0));
    std::vector<int> seeds_x(simulation_parameters.number_of_robots), seeds_y(simulation_parameters.number_of_robots);
    std::vector<std::vector<double>> dist_map(simulation_parameters.number_of_robots, std::vector<double>(x_nbins * y_nbins, std::numeric_limits<double>::max()));

    for (int i = 0; i < simulation_parameters.number_of_robots; ++i)
        index_to_indices_2D(selected_indices[i], &seeds_x[i], &seeds_y[i]);

    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));
    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    double cell_size_x = (XMAX - XMIN) / static_cast<double>(x_nbins);
    double cell_size_y = (YMAX - YMIN) / static_cast<double>(y_nbins);

    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        std::vector<std::vector<double>> local_dist(y_nbins, std::vector<double>(x_nbins, std::numeric_limits<double>::max()));
        std::queue<std::pair<int, int>> q;

        int sx = seeds_x[k], sy = seeds_y[k];

        local_dist[sy][sx] = 0.0;
        q.push({sx, sy});

        while (!q.empty()) {
            auto [x, y] = q.front(); q.pop();
            double d = local_dist[y][x];

            for (const auto& [dx, dy] : directions) {
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                if (obstacle_map_2d[ny][nx] == 1) continue;

                double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));

                if (local_dist[ny][nx] > d + cost) {
                    local_dist[ny][nx] = d + cost;
                    q.push({nx, ny});
                }
            }
        }

        for (int y = 0; y < y_nbins; ++y)
            for (int x = 0; x < x_nbins; ++x)
                dist_map[k][y * x_nbins + x] = local_dist[y][x];
    }

    std::vector<int> cluster_sizes(simulation_parameters.number_of_robots, 0);
    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize) continue;

        double min_dist = dist_map[0][i];
        int min_k = 0;
        for (int k = 1; k < simulation_parameters.number_of_robots; ++k) {
            if (dist_map[k][i] < min_dist) {
                min_dist = dist_map[k][i];
                min_k = k;
            }
        }

        belief_clusters[i] = min_k + 1;
        cluster_sizes[min_k]++;
    }

    log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster();

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        for (int k = 0; k < simulation_parameters.number_of_robots; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

    std::vector<double> cluster_sizes_info(simulation_parameters.number_of_robots, 0.0);

    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        double sum_prob = 0.0;

        for (int i = 0; i < x_nbins * y_nbins; ++i) {
            double masked_value = posterior->pdf[i] * pdf_masks[k][i];
            sum_prob += masked_value;
        }

        cluster_sizes_info[k] = sum_prob;
    }
    log_cluster_sizes_informative(cluster_sizes_info);

    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior->pdf[i] *= pdf_masks[k][i];
        }
    }

    fprintf(clustering_debugging_logfile, "Robot ID [%d]: NO Normalized Individual pdf after mask:\n", ID);
    for (int y = 0; y < y_nbins; ++y) {
        for (int x = 0; x < x_nbins; ++x) {
            int idx = y * x_nbins + x;
            fprintf(clustering_debugging_logfile, "%.9f ", posterior->pdf[idx]);
        }
        fprintf(clustering_debugging_logfile, "\n");
    }

    double sum = std::accumulate(posterior->pdf, posterior->pdf + x_nbins * y_nbins, 0.0);
    if (sum > 0)
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            posterior->pdf[i] /= sum;

    double sum_def = std::accumulate(posterior->pdf, posterior->pdf + x_nbins * y_nbins, 0.0);
    printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

    fprintf(clustering_debugging_logfile, "Robot ID [%d]: Normalized Individual pdf after mask:\n", ID);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_LLOYD(const std::vector<int>& selected_indices)
{
    const int K = simulation_parameters.number_of_robots;
    const int max_iterations = 10;

    std::vector<std::pair<double, int>> top_cells_lloyd;
    for (int i = 0; i < x_nbins * y_nbins; ++i)
        if (posterior->pdf[i] > simulation_parameters.epsilon_clusterize)
            top_cells_lloyd.emplace_back(posterior->pdf[i], i);
    std::sort(top_cells_lloyd.begin(), top_cells_lloyd.end(), std::greater<>());

    std::vector<std::pair<int, int>> seeds(K);
    for (int i = 0; i < K && i < (int)top_cells_lloyd.size(); ++i)
        index_to_indices_2D(top_cells_lloyd[i].second, &seeds[i].first, &seeds[i].second);

    std::vector<std::vector<double>> dist_map(K, std::vector<double>(x_nbins * y_nbins, std::numeric_limits<double>::max()));

    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));
    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    double cell_size_x = (XMAX - XMIN) / static_cast<double>(x_nbins);
    double cell_size_y = (YMAX - YMIN) / static_cast<double>(y_nbins);

    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    for (int k = 0; k < K; ++k) {
        std::vector<std::vector<double>> local_dist(y_nbins, std::vector<double>(x_nbins, std::numeric_limits<double>::max()));
        std::queue<std::pair<int, int>> q;

        int sx = seeds[k].first, sy = seeds[k].second;
        local_dist[sy][sx] = 0.0;
        q.push({sx, sy});

        while (!q.empty()) {
            auto [x, y] = q.front(); q.pop();
            double d = local_dist[y][x];

            for (const auto& [dx, dy] : directions) {
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                if (obstacle_map_2d[ny][nx] == 1) continue;

                double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));
                if (local_dist[ny][nx] > d + cost) {
                    local_dist[ny][nx] = d + cost;
                    q.push({nx, ny});
                }
            }
        }

        for (int y = 0; y < y_nbins; ++y)
            for (int x = 0; x < x_nbins; ++x)
                dist_map[k][y * x_nbins + x] = local_dist[y][x];
    }

    std::vector<int> cluster_assignments(x_nbins * y_nbins, -1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<int> temp_cluster_sizes(K, 0);

        for (int i = 0; i < x_nbins * y_nbins; ++i) {
            if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize) continue;

            double min_dist = dist_map[0][i];
            int best_k = 0;

            for (int k = 1; k < K; ++k) {
                double dist = dist_map[k][i];
                if (dist < min_dist || (dist == min_dist && temp_cluster_sizes[k] < temp_cluster_sizes[best_k])) {
                    min_dist = dist;
                    best_k = k;
                }
            }

            cluster_assignments[i] = best_k;
            temp_cluster_sizes[best_k]++;
        }

        std::vector<double> weighted_sum_x(K, 0.0), weighted_sum_y(K, 0.0), total_weight(K, 0.0);
        for (int i = 0; i < x_nbins * y_nbins; ++i) {
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

        for (int k = 0; k < K; ++k) {
            std::vector<std::vector<double>> local_dist(y_nbins, std::vector<double>(x_nbins, std::numeric_limits<double>::max()));
            std::queue<std::pair<int, int>> q;

            int sx = seeds[k].first, sy = seeds[k].second;
            local_dist[sy][sx] = 0.0;
            q.push({sx, sy});

            while (!q.empty()) {
                auto [x, y] = q.front(); q.pop();
                double d = local_dist[y][x];

                for (const auto& [dx, dy] : directions) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                    if (obstacle_map_2d[ny][nx] == 1) continue;

                    double cost = std::sqrt(std::pow(dx * cell_size_x, 2) + std::pow(dy * cell_size_y, 2));
                    if (local_dist[ny][nx] > d + cost) {
                        local_dist[ny][nx] = d + cost;
                        q.push({nx, ny});
                    }
                }
            }

            for (int y = 0; y < y_nbins; ++y)
                for (int x = 0; x < x_nbins; ++x)
                    dist_map[k][y * x_nbins + x] = local_dist[y][x];
        }

        for (int i = 0; i < x_nbins * y_nbins; ++i)
            belief_clusters[i] = (posterior->pdf[i] > simulation_parameters.epsilon_clusterize) ? cluster_assignments[i] + 1 : 0;

        log_posterior_pdf_3d_cluster();
    }

    std::vector<int> perm(K);
    std::iota(perm.begin(), perm.end(), 0);
    std::vector<int> best_perm = perm;
    double best_cost = std::numeric_limits<double>::max();
    do {
        double cost = 0.0;
        for (int k = 0; k < K; ++k) {
            double cx = XMIN + (seeds[perm[k]].first  + 0.5) * (XMAX - XMIN) / x_nbins;
            double cy = YMIN + (seeds[perm[k]].second + 0.5) * (YMAX - YMIN) / y_nbins;
            double ddx = received_current_pos[k][0] - cx;
            double ddy = received_current_pos[k][1] - cy;
            cost += std::sqrt(ddx * ddx + ddy * ddy);
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_perm = perm;
        }
    } while (std::next_permutation(perm.begin(), perm.end()));

    std::vector<int> inv_perm(K);
    for (int k = 0; k < K; ++k) inv_perm[best_perm[k]] = k;

    std::vector<int> cluster_sizes(K, 0);
    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);
    std::vector<std::vector<int>> pdf_masks(K, std::vector<int>(x_nbins * y_nbins, 0));

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        int cluster = cluster_assignments[i];
        int robot_k = inv_perm[cluster];
        belief_clusters[i] = robot_k + 1;
        pdf_masks[robot_k][i] = 1;
        cluster_sizes[robot_k]++;
    }

    log_cluster_sizes_geometric(cluster_sizes);
    log_posterior_pdf_3d_cluster();

    std::vector<double> cluster_sizes_info(K, 0.0);
    for (int k = 0; k < K; ++k)
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            cluster_sizes_info[k] += posterior->pdf[i] * pdf_masks[k][i];
    log_cluster_sizes_informative(cluster_sizes_info);

    for (int k = 0; k < K; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior->pdf[i] *= pdf_masks[k][i];

            double sum = std::accumulate(posterior->pdf, posterior->pdf + x_nbins * y_nbins, 0.0);
            if (sum > 0.0)
                for (int i = 0; i < x_nbins * y_nbins; ++i)
                    posterior->pdf[i] /= sum;

            printf("Robot ID [%d]: Sum of Lloyd-obstacle-aware pdf is %f \n", ID, sum);
        }
    }
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_REGION_GROWING_D()
{
    const int K = simulation_parameters.number_of_robots;
    const int N = x_nbins * y_nbins;

    std::vector<std::vector<int>> pdf_masks(K, std::vector<int>(N, 0));
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));

    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    std::vector<std::pair<double, int>> top_cells;

    for (int i = 0; i < N; ++i) {
        if (posterior->pdf[i] > simulation_parameters.epsilon_clusterize &&
            obstacle_map[i] == 0) {
            top_cells.push_back({posterior->pdf[i], i});
        }
    }

    std::sort(top_cells.begin(), top_cells.end(), std::greater<>());

    if ((int)top_cells.size() < K) {
        printf("Robot ID [%d]: Not enough valid seed cells for balanced region growing.\n", ID);
        return;
    }

    std::vector<int> seeds_x(K), seeds_y(K);

    for (int k = 0; k < K; ++k)
        index_to_indices_2D(top_cells[k].second, &seeds_x[k], &seeds_y[k]);

    std::vector<int> perm(K);
    std::iota(perm.begin(), perm.end(), 0);

    std::vector<int> best_perm = perm;
    double best_cost = std::numeric_limits<double>::max();

    do {
        double cost = 0.0;

        for (int k = 0; k < K; ++k) {
            double cx = XMIN + (seeds_x[perm[k]] + 0.5) * (XMAX - XMIN) / x_nbins;
            double cy = YMIN + (seeds_y[perm[k]] + 0.5) * (YMAX - YMIN) / y_nbins;

            double ddx = received_current_pos[k][0] - cx;
            double ddy = received_current_pos[k][1] - cy;

            cost += std::sqrt(ddx * ddx + ddy * ddy);
        }

        if (cost < best_cost) {
            best_cost = cost;
            best_perm = perm;
        }

    } while (std::next_permutation(perm.begin(), perm.end()));

    std::vector<int> tmp_x(K), tmp_y(K);

    for (int k = 0; k < K; ++k) {
        tmp_x[k] = seeds_x[best_perm[k]];
        tmp_y[k] = seeds_y[best_perm[k]];
    }

    seeds_x = tmp_x;
    seeds_y = tmp_y;

    std::fill(belief_clusters, belief_clusters + N, 0);

    const std::vector<std::pair<int, int>> all_directions = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1},
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::vector<std::queue<std::pair<int, int>>> queues(K);
    std::vector<int> cluster_sizes(K, 0);

    int total_active_cells = 0;

    for (int i = 0; i < N; ++i) {
        if (posterior->pdf[i] > simulation_parameters.epsilon_clusterize &&
            obstacle_map[i] == 0) {
            total_active_cells++;
        }
    }

    int target_size = std::max(1, total_active_cells / K);
    int max_size_margin = static_cast<int>(target_size * 1.05);

    for (int k = 0; k < K; ++k) {
        int sx = seeds_x[k];
        int sy = seeds_y[k];
        int idx = sy * x_nbins + sx;

        if (sx < 0 || sx >= x_nbins || sy < 0 || sy >= y_nbins)
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

                    if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins)
                        continue;

                    if (visited[ny][nx])
                        continue;

                    int nidx = ny * x_nbins + nx;

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

    for (int i = 0; i < N; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize)
            continue;

        for (int k = 0; k < K; ++k) {
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
        }
    }

    std::vector<double> cluster_sizes_info(K, 0.0);

    for (int k = 0; k < K; ++k) {
        double sum_prob = 0.0;

        for (int i = 0; i < N; ++i)
            sum_prob += posterior->pdf[i] * pdf_masks[k][i];

        cluster_sizes_info[k] = sum_prob;
    }

    log_cluster_sizes_informative(cluster_sizes_info);

    for (int k = 0; k < K; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < N; ++i)
                posterior->pdf[i] *= pdf_masks[k][i];
        }
    }

    double sum = std::accumulate(posterior->pdf, posterior->pdf + N, 0.0);

    if (sum > 0) {
        for (int i = 0; i < N; ++i)
            posterior->pdf[i] /= sum;
    }

    double sum_def = std::accumulate(posterior->pdf, posterior->pdf + N, 0.0);

    printf("Robot ID [%d]: Sum of balanced 2D region-growing clusterized pdf is %f \n",
           ID, sum_def);
}

void RegionAllocator::clusterize_belief_map_obstacle_dist_REGION_GROWING_P()
{
    std::vector<std::vector<int>> pdf_masks(simulation_parameters.number_of_robots, std::vector<int>(x_nbins * y_nbins, 0));
    std::vector<std::vector<int>> obstacle_map_2d(y_nbins, std::vector<int>(x_nbins));

    for (int y = 0; y < y_nbins; ++y)
        for (int x = 0; x < x_nbins; ++x)
            obstacle_map_2d[y][x] = obstacle_map[y * x_nbins + x];

    std::vector<std::pair<double, int>> top_cells;
    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior->pdf[i] > simulation_parameters.epsilon_clusterize)
            top_cells.push_back({posterior->pdf[i], i});
    }
    std::sort(top_cells.begin(), top_cells.end(), std::greater<>());

    std::vector<int> seeds_x(3), seeds_y(3);
    for (int i = 0; i < 3 && i < (int)top_cells.size(); ++i)
        index_to_indices_2D(top_cells[i].second, &seeds_x[i], &seeds_y[i]);

    std::fill(belief_clusters, belief_clusters + x_nbins * y_nbins, 0);

    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    std::vector<std::vector<bool>> visited(y_nbins, std::vector<bool>(x_nbins, false));
    std::queue<std::pair<int, int>> queues[3];
    int cluster_sizes[3] = {0, 0, 0};

    int total_active_cells = 0;
    for (int i = 0; i < x_nbins * y_nbins; ++i)
        if (posterior->pdf[i] > simulation_parameters.epsilon_clusterize && obstacle_map[i] == 0)
            total_active_cells++;

    int target_size = total_active_cells / 3;
    int max_size_margin = target_size * 1.2;

    for (int k = 0; k < 3; ++k) {
        queues[k].push({seeds_x[k], seeds_y[k]});
        visited[seeds_y[k]][seeds_x[k]] = true;
        belief_clusters[seeds_y[k] * x_nbins + seeds_x[k]] = k + 1;
        cluster_sizes[k]++;
    }

    bool growing = true;
    while (growing) {
        growing = false;

        for (int k = 0; k < 3; ++k) {
            if (cluster_sizes[k] >= max_size_margin) continue;

            if (queues[k].empty()) goto stop_region_growing;

            auto [x, y] = queues[k].front(); queues[k].pop();

            int best_nx = -1, best_ny = -1;
            double best_pdf = -1.0;

            for (const auto& [dx, dy] : directions) {
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= x_nbins || ny < 0 || ny >= y_nbins) continue;
                if (obstacle_map_2d[ny][nx] == 1) continue;
                if (visited[ny][nx]) continue;

                int idx = ny * x_nbins + nx;
                if (posterior->pdf[idx] <= simulation_parameters.epsilon_clusterize) continue;

                if (posterior->pdf[idx] > best_pdf) {
                    best_pdf = posterior->pdf[idx];
                    best_nx = nx;
                    best_ny = ny;
                }
            }

            if (best_nx != -1 && best_ny != -1) {
                int idx = best_ny * x_nbins + best_nx;
                visited[best_ny][best_nx] = true;
                belief_clusters[idx] = k + 1;
                queues[k].push({best_nx, best_ny});
                cluster_sizes[k]++;
                growing = true;
            }
        }
    }
stop_region_growing:

    log_cluster_sizes_geometric(std::vector<int>(cluster_sizes, cluster_sizes + 3));
    log_posterior_pdf_3d_cluster();

    for (int i = 0; i < x_nbins * y_nbins; ++i) {
        if (posterior->pdf[i] <= simulation_parameters.epsilon_clusterize) continue;
        for (int k = 0; k < 3; ++k)
            pdf_masks[k][i] = (belief_clusters[i] == k + 1) ? 1 : 0;
    }

    std::vector<double> cluster_sizes_info(simulation_parameters.number_of_robots, 0.0);

    for (int k = 0; k < simulation_parameters.number_of_robots; ++k) {
        double sum_prob = 0.0;

        for (int i = 0; i < x_nbins * y_nbins; ++i) {
            double masked_value = posterior->pdf[i] * pdf_masks[k][i];
            sum_prob += masked_value;
        }

        cluster_sizes_info[k] = sum_prob;
    }
    log_cluster_sizes_informative(cluster_sizes_info);

    for (int k = 0; k < 3; ++k) {
        if (ID == 101 + k) {
            for (int i = 0; i < x_nbins * y_nbins; ++i)
                posterior->pdf[i] *= pdf_masks[k][i];
        }
    }

    double sum = std::accumulate(posterior->pdf, posterior->pdf + x_nbins * y_nbins, 0.0);
    if (sum > 0)
        for (int i = 0; i < x_nbins * y_nbins; ++i)
            posterior->pdf[i] /= sum;

    double sum_def = std::accumulate(posterior->pdf, posterior->pdf + x_nbins * y_nbins, 0.0);
    printf("Robot ID [%d]: Sum of clusterized pdf is %f \n", ID, sum_def);

    for (int k = 0; k < 3; ++k) {
        if (ID == 101 + k) {
            fprintf(clustering_debugging_logfile, "\n Clusterized Belief Map (balanceado):\n");
            for (int y = 0; y < y_nbins; ++y) {
                for (int x = 0; x < x_nbins; ++x) {
                    int idx = y * x_nbins + x;
                    fprintf(clustering_debugging_logfile, "%d ", belief_clusters[idx]);
                }
                fprintf(clustering_debugging_logfile, "\n");
            }
        }
    }
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
        std::vector<std::pair<double,int>> top_cells;

        for (int idx = 0; idx < x_nbins * y_nbins; ++idx) {
            if (posterior->pdf[idx] > simulation_parameters.epsilon_clusterize)
                top_cells.emplace_back(posterior->pdf[idx], idx);
        }

        std::partial_sort(top_cells.begin(),
                          top_cells.begin() + std::min(N_SEEDS, (int)top_cells.size()),
                          top_cells.end(),
                          std::greater<>());

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
