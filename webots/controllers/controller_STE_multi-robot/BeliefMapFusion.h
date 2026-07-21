#ifndef BELIEF_MAP_FUSION_H
#define BELIEF_MAP_FUSION_H

#include <vector>

#include "constant_and_helper.h"

namespace BeliefMapFusion
{
	bool is_boundary_cell(int index);
	bool is_invalid_source_cell(int index, const std::vector<int> &obstacle_map);
	void pdf_to_log_map(const double pdf[MAP_SIZE], double log_map[MAP_SIZE],
					const std::vector<int> &obstacle_map);
	void sanitize_log_map(const double log_map[MAP_SIZE], double sanitized_log_map[MAP_SIZE],
				 const std::vector<int> &obstacle_map);
	void log_map_to_pdf(const double log_map[MAP_SIZE], double pdf[MAP_SIZE]);
	bool normalize_pdf(double pdf[MAP_SIZE]);
	bool normalize_log_map(double log_map[MAP_SIZE]);
	bool fuse_normal_maps(const double belief_maps[MAX_NB_ROBOTS][MAP_SIZE],
				 int robot_count, double merged_pdf[MAP_SIZE]);
	bool fuse_log_maps(const double belief_maps[MAX_NB_ROBOTS][MAP_SIZE],
			      int robot_count, double merged_log_map[MAP_SIZE]);
}

#endif
