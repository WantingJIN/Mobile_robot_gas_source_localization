#include "BeliefMapFusion.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
	constexpr double LOG_ZERO = -std::numeric_limits<double>::infinity();
}

namespace BeliefMapFusion
{
	bool is_boundary_cell(int index)
	{
		int x = 0;
		int y = 0;
		index_to_indices_2D(index, &x, &y);
		return x <= 0 || x >= source_x_nbins - 1 ||
		       y <= 0 || y >= source_y_nbins - 1;
	}

	bool is_invalid_source_cell(int index, const std::vector<int> &obstacle_map)
	{
		return is_boundary_cell(index) ||
		       (index >= 0 && index < static_cast<int>(obstacle_map.size()) &&
		        obstacle_map[index] == 1);
	}

	void pdf_to_log_map(const double pdf[MAP_SIZE], double log_map[MAP_SIZE],
					const std::vector<int> &obstacle_map)
	{
		for (int i = 0; i < MAP_SIZE; ++i)
		{
			if (is_invalid_source_cell(i, obstacle_map) || pdf[i] <= 0.0 ||
			    !std::isfinite(pdf[i]))
			{
				log_map[i] = LOG_ZERO;
			}
			else
			{
				log_map[i] = std::log(pdf[i]);
			}
		}
	}

	void sanitize_log_map(const double log_map[MAP_SIZE], double sanitized_log_map[MAP_SIZE],
				 const std::vector<int> &obstacle_map)
	{
		for (int i = 0; i < MAP_SIZE; ++i)
		{
			if (is_invalid_source_cell(i, obstacle_map) || !std::isfinite(log_map[i]))
				sanitized_log_map[i] = LOG_ZERO;
			else
				sanitized_log_map[i] = log_map[i];
		}
	}

	void log_map_to_pdf(const double log_map[MAP_SIZE], double pdf[MAP_SIZE])
	{
		for (int i = 0; i < MAP_SIZE; ++i)
			pdf[i] = std::isfinite(log_map[i]) ? std::exp(log_map[i]) : 0.0;
	}

	bool normalize_pdf(double pdf[MAP_SIZE])
	{
		double sum = 0.0;
		for (int i = 0; i < MAP_SIZE; ++i)
			sum += pdf[i];

		if (sum <= 0.0 || !std::isfinite(sum))
			return false;

		for (int i = 0; i < MAP_SIZE; ++i)
			pdf[i] /= sum;

		return true;
	}

	bool normalize_log_map(double log_map[MAP_SIZE])
	{
		double max_log = LOG_ZERO;
		for (int i = 0; i < MAP_SIZE; ++i)
			if (log_map[i] > max_log)
				max_log = log_map[i];

		if (!std::isfinite(max_log))
			return false;

		double shifted_sum = 0.0;
		for (int i = 0; i < MAP_SIZE; ++i)
			if (std::isfinite(log_map[i]))
				shifted_sum += std::exp(log_map[i] - max_log);

		if (shifted_sum <= 0.0 || !std::isfinite(shifted_sum))
			return false;

		const double log_sum = max_log + std::log(shifted_sum);
		for (int i = 0; i < MAP_SIZE; ++i)
			if (std::isfinite(log_map[i]))
				log_map[i] -= log_sum;

		return true;
	}

	bool fuse_normal_maps(const double belief_maps[MAX_NB_ROBOTS][MAP_SIZE],
				 int robot_count, double merged_pdf[MAP_SIZE])
	{
		for (int i = 0; i < MAP_SIZE; ++i)
			merged_pdf[i] = 1.0;

		for (int r = 0; r < robot_count; ++r)
			for (int i = 0; i < MAP_SIZE; ++i)
				merged_pdf[i] *= belief_maps[r][i];

		return normalize_pdf(merged_pdf);
	}

	bool fuse_log_maps(const double belief_maps[MAX_NB_ROBOTS][MAP_SIZE],
			      int robot_count, double merged_log_map[MAP_SIZE])
	{
		for (int i = 0; i < MAP_SIZE; ++i)
			merged_log_map[i] = 0.0;

		for (int r = 0; r < robot_count; ++r)
		{
			for (int i = 0; i < MAP_SIZE; ++i)
			{
				if (!std::isfinite(merged_log_map[i]) || !std::isfinite(belief_maps[r][i]))
					merged_log_map[i] = LOG_ZERO;
				else
					merged_log_map[i] += belief_maps[r][i];
			}
		}

		return normalize_log_map(merged_log_map);
	}
}
