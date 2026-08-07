#ifndef BELIEF_MAP_FUSION_H
#define BELIEF_MAP_FUSION_H

#include <vector>

#include "constant_and_helper.h"

namespace BeliefMapFusion
{
	bool is_boundary_cell(int index);
	bool is_invalid_source_cell(int index, const std::vector<int> &obstacle_map);
	void pdf_to_log_map(const float pdf[MAP_SIZE], float log_map[MAP_SIZE],
					const std::vector<int> &obstacle_map);
	void sanitize_log_map(const float log_map[MAP_SIZE], float sanitized_log_map[MAP_SIZE],
				 const std::vector<int> &obstacle_map);
	void log_map_to_pdf(const float log_map[MAP_SIZE], float pdf[MAP_SIZE]);
	bool normalize_pdf(float pdf[MAP_SIZE]);
	bool normalize_log_map(float log_map[MAP_SIZE]);
	bool fuse_normal_maps(const float belief_maps[MAX_NB_ROBOTS][MAP_SIZE],
				 int robot_count, float merged_pdf[MAP_SIZE]);
	bool fuse_log_maps(const float belief_maps[MAX_NB_ROBOTS][MAP_SIZE],
			      int robot_count, float merged_log_map[MAP_SIZE]);
}

#endif
