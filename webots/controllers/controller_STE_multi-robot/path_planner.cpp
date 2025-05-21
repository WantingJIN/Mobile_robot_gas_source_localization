#include "path_planner.h"

extern SimulationParameters simulation_parameters;
FILE *gas_information_map_logfile;
FILE *scores_map_logfile;
FILE *distance_map_logfile;
FILE *acc_info_map_logfile;
extern Position goal_position;
extern Position goal_position_team[3];
extern Position current_position;
extern SimulationParameters simulation_parameters;
extern int cell_visited_list[MAP_SIZE];

extern char gas_information_map_logfile_name[512];
extern char scores_map_logfile_name[512];
extern char distance_map_logfile_name[512];
extern char acc_info_map_logfile_name[512];
extern int ID;
extern int nbr_obs;
extern float margin;
extern float **obstacles;


// text file for storing some prints for debugging
extern FILE *console_logfile;
extern FILE *clustering_debugging_logfile_101;
extern FILE *clustering_debugging_logfile_102;
extern FILE *clustering_debugging_logfile_103;

PathPlanner::PathPlanner(PlumeModel *input_model, Posterior *input_posterior, struct history *input_history)
{
      plume_model = input_model;
      posterior = input_posterior;
	history = input_history;
}
PathPlanner:: ~PathPlanner()
{
}
// TODO - maybe merge this to greedy_information_planning
void PathPlanner::maximal_belief_planning()
{
	printf("Enter maximal belief planning \n");
	fprintf(console_logfile, "Enter maximal belief planning \n");
	double best_point[2] = {0};
	// find the maximal variance in gas variance map
	double max_pdf = 0;
	int max_pdf_index = 0;
	int sum = 0;
	for (int i = 0; i < MAP_SIZE; i++)
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			if (posterior->pdf[i] > max_pdf && cell_visited_list[i] == 0)
			{
				max_pdf = posterior->pdf[i];
				max_pdf_index = i;
			}
		}
		else
		{
			if (posterior->pdf[i] > max_pdf)
			{
				max_pdf = posterior->pdf[i];
				max_pdf_index = i;
			}
		}
	}
	fprintf(console_logfile, "max_pdf : %f , max_pdf_iter: %d\n", max_pdf, max_pdf_index);

	double epsilon = 1e-9;
	if (fabs(max_pdf - 0.0) < epsilon) // all the possible points have been visited
	{
		printf("all the probable points have been visited \n");
		fprintf(console_logfile, "all the probable points have been visited, use maximal information planning \n");
		maximal_information_planning();
		printf("goal position : %f %f \n", goal_position.x, goal_position.y);
		return;
	}
	posterior->index_to_param_values(max_pdf_index, best_point);

	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;
}

void PathPlanner::random_navigation()
{
	goal_position.x = (randu() * (XMAX - SAFETY_MARGIN * 2)) + SAFETY_MARGIN; // position in m
	goal_position.y = (randu() * (YMAX - SAFETY_MARGIN * 2)) + SAFETY_MARGIN;
	goal_position.heading = 0; //(randu() * (ZMAX - SAFETY_MARGIN*2)) + SAFETY_MARGIN;
}

bool PathPlanner::position_outside_arena(Position p)
{
	bool outside = 0;
	if (p.x < float(XMIN) || p.x > float(XMAX))
		outside = 1;
	if (p.y < float(YMIN) || p.y > float(YMAX))
		outside = 1;
	// if(position[2] < ZMIN || position[2] > ZMAX) outside = 1;
	return outside;
}
void PathPlanner::KLD_informative_goal_selection()
{
	float p0 = 0.5; // 50 cm
	double KLD = 0, max_KLD = -100, max_meas = -100;
	// double position[1][3] = {{0}};
	Position tmp_position;
	struct history fake_history;
	fake_history.position = history->position;
	fake_history.measurement = history->measurement;
	fake_history.size = history->size;

	// add fake prediction for friends goal position, so we don't choose the same ones
	// for (int i = 0; i < simulation_parameters.number_of_robots; i++)
	// {
	// 	if (friends[i].ID == 0 || friends[i].ID == ID)
	// 		continue;
	// 	fake_history.position[fake_history.size].copy_position(friends[i].goal);
	// 	fake_history.measurement[fake_history.size] = plume_model->approx_concentration(friends[i].goal, sampleBuffer.average_wind_intensity);
	// 	(fake_history.size)++;
	// }
	// 8 candidate positions
	Position candidates[8] = {Position(current_position.x + p0, current_position.y, 0),
					  Position(current_position.x - p0, current_position.y, 0),
					  Position(current_position.x, current_position.y + p0, 0),
					  Position(current_position.x, current_position.y - p0, 0),
					  Position(current_position.x + p0, current_position.y + p0, 0),
					  Position(current_position.x - p0, current_position.y + p0, 0),
					  Position(current_position.x + p0, current_position.y - p0, 0),
					  Position(current_position.x - p0, current_position.y - p0, 0)};
	for (auto &candidate_pos : candidates)
	{
		// for each possible action
		tmp_position.x = candidate_pos.x;
		tmp_position.y = candidate_pos.y;
		tmp_position.heading = 0;
		if (position_outside_arena(tmp_position))
			continue;
		// calculate the expected concentration on position
		// Note: the average_wind_intensity is not used in DDPM for predicting the concentration, so we set it to 0 here
		fake_history.measurement.push_back(plume_model->approx_concentration(tmp_position, 0.0));
		std::cout << "fake_history.measurement = " << fake_history.measurement[fake_history.size] << std::endl;
		fake_history.position.push_back(tmp_position);
		(fake_history.size)++;
		DeepCFDPlumeModel2D plume_model_fake;
		Posterior future_posterior(&plume_model_fake, 0);
		// future_posterior.copy_posterior(zero);
		future_posterior.MCMC_evaluation(fake_history);
		// calculate the expected information gain
		KLD = posterior->Kullback_Leibler_divergence(future_posterior);
		// printf("KLD: %f _______________________\n", KLD);
		if (KLD > max_KLD)
		{
			max_KLD = KLD;
			goal_position.x = tmp_position.x; // + (randu()-0.5)/5.0
			goal_position.y = tmp_position.y;
			goal_position.heading = 0;
		}
		fake_history.measurement.pop_back();
		fake_history.position.pop_back();
		fake_history.size--;
	}
}

void PathPlanner::random_planning()
{
	printf("Enter random planning \n");
	fprintf(console_logfile, "Enter random planning \n");
	do
	{
		goal_position.heading = 0;
		goal_position.x = (((double)rand() / (double)RAND_MAX) * (XMAX - XMIN - SAFETY_MARGIN * 2)) + SAFETY_MARGIN + XMIN;
		goal_position.y = (((double)rand() / (double)RAND_MAX) * (YMAX - YMIN - SAFETY_MARGIN * 2)) + SAFETY_MARGIN + YMIN;
	} while (goal_position.is_in_obstacle_list(obstacles, nbr_obs) != -1);
}

//TODO - Now can only be used in obstacle free environment
void PathPlanner::lawnmower_planning()
{
	printf("Enter lawnmower planning \n");
	int static direction = 1;
	double x_step = 1.0;
	double y_step = 4;
	goal_position.y = current_position.y + direction * y_step;
	goal_position.x = current_position.x;
	goal_position.heading = 0;
	if (current_position.y > YMAX - SAFETY_MARGIN - 0.05 && direction == 1)
	{
		goal_position.x = current_position.x + x_step;
		direction = -1;
	}
	if (current_position.y < YMIN + SAFETY_MARGIN + 0.1 && direction == -1)
	{
		goal_position.x = current_position.x + x_step;
		direction = 1;
	}
	if (goal_position.x > XMAX - SAFETY_MARGIN - 0.05)
	{
		FSM.state = STATE_FINISHED;
		printf("FINISHED \n");
	}
}

void PathPlanner::calculate_information_gain()
{
	if (simulation_parameters.infoFeature == VARIANCE)
	{
		plume_model->calculate_weighted_gas_con_variance_map(gas_information_map, posterior->pdf);
	}
	if (simulation_parameters.infoFeature == MEAN)
	{
		plume_model->calculate_weighted_gas_con_mean_map(gas_information_map, posterior->pdf);
	}
      if (simulation_parameters.infoFeature == PDF){
		for (int i = 0; i < MAP_SIZE; i++)
		{
			gas_information_map[i] = posterior->pdf[i];
		}
	}

}


double PathPlanner::index_distance(int x_idx_diff, int y_idx_diff){
	return sqrt(pow(x_idx_diff * x_resol, 2) + pow(y_idx_diff * y_resol, 2));
}

// FIXME - If the information is PDF, then the logic is a bit wrong
void PathPlanner::maximal_information_planning()
{
	printf("Enter maximal information planning \n");
	fprintf(console_logfile, "Enter maximal information planning \n");
	calculate_information_gain();
	double best_point[2] = {0};
	// find the maximal variance in gas variance map
	double max_info = 0;
	int max_info_index = 0;
	int sum = 0;
	for (int i = 0; i < MAP_SIZE; i++)
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			if (gas_information_map[i] > max_info && cell_visited_list[i] == 0)
			{
				max_info = gas_information_map[i];
				max_info_index = i;
			}
		}
		else
		{
			if (gas_information_map[i] > max_info)
			{
				max_info = gas_information_map[i];
				max_info_index = i;
			}
		}
	}
	fprintf(console_logfile, "max_info : %f , max_info_iter: %d\n", max_info, max_info_index);

	double epsilon = 1e-9;
	if (fabs(max_info - 0.0) < epsilon) // all the informative points have been visited
	{
		printf("all the informative points have been visited \n");
		fprintf(console_logfile, "all the informative points have been visited \n");
		random_planning();
		printf("goal position : %f %f \n", goal_position.x, goal_position.y);
		return;
	}

	posterior->index_to_param_values(max_info_index, best_point);

	goal_position.x = best_point[0];
	goal_position.y = best_point[1];
	goal_position.heading = 0;
}

vector<pair<CellPoint, CellPoint>> PathPlanner::clustering_information_blocks(double information_map[])
{
	// dbscan clustering
	// param: x_eps, y_eps, minPts, ratio
	DBSCAN dbscan(x_resol + 0.1, y_resol + 0.1, 4, (1.0 - posterior->entropy / simulation_parameters.cluster_entropy_ratio));
	dbscan.run(information_map);
	return dbscan.cluster_bounding_boxes;
}

void PathPlanner::belief_clustering_planning()
{
	printf("Enter belief clustering planning \n");

	vector<pair<CellPoint, CellPoint>> clusters_bounding_box;
	clusters_bounding_box = clustering_information_blocks(posterior->pdf);
	if (clusters_bounding_box.size() == 0)
	{
		printf("no cluster found \n");
		fprintf(console_logfile, "no cluster found \n");
		maximal_belief_planning();
		// maximal_information_planning();
		return;
	}
	else
	{
		// fprintf(console_logfile, "clusters found \n");
		// Find furthest vertex of each cluster
		double max_dist_to_robot;
		int max_dist_to_robot_index;
		vector<Position> candidate_goal;
		for (int i = 0; i < clusters_bounding_box.size(); i++)
		{
			max_dist_to_robot = 0;
			max_dist_to_robot_index = 0;
			// calculate the distance of each vertex to robot
			Position cluster_vertex[4] = {Position(clusters_bounding_box[i].first.x, clusters_bounding_box[i].first.y, 0),
								Position(clusters_bounding_box[i].first.x, clusters_bounding_box[i].second.y, 0),
								Position(clusters_bounding_box[i].second.x, clusters_bounding_box[i].first.y, 0),
								Position(clusters_bounding_box[i].second.x, clusters_bounding_box[i].second.y, 0)};
			for (int i = 0; i < 4; i++)
			{
				double dist = dist_to_goal(cluster_vertex[i]);
				if (dist > max_dist_to_robot)
				{
					max_dist_to_robot = dist;
					max_dist_to_robot_index = i;
				}
			}
			candidate_goal.push_back(cluster_vertex[max_dist_to_robot_index]);
		}
		// find the closest unvisited point among candidate goals
		double min_dist_to_robot = 1000;
		int min_dist_to_robot_index = 0;
		bool find_goal = false;
		for (int j = 0; j < candidate_goal.size(); j++)
		{
			double dist = dist_to_goal(candidate_goal[j]);
			if (dist < min_dist_to_robot)
			{
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int pos_coord[2];
                              position_to_indices_2D(candidate_goal[j].x, candidate_goal[j].y, &pos_coord[0], &pos_coord[1]);
					int idx = posterior->indices_to_index(pos_coord);
					if (cell_visited_list[idx] == 0)
					{
						min_dist_to_robot = dist;
						min_dist_to_robot_index = j;
						find_goal = true;
					}
				}
				else
				{
					min_dist_to_robot = dist;
					min_dist_to_robot_index = j;
					find_goal = true;
				}
			}
		}
		if (!find_goal)
		{
			fprintf(console_logfile, "all the vertex of clusters are visited, use maximal belief planning \n");
			// maximal_information_planning();
			maximal_belief_planning();
			return;
		}
		else
		{
			goal_position.x = candidate_goal[min_dist_to_robot_index].x;
			goal_position.y = candidate_goal[min_dist_to_robot_index].y;
			goal_position.heading = 0;
		}

		// fprintf(console_logfile, "clustering goal position : %f %f \n", goal_position.x, goal_position.y);
		// fprintf(console_logfile, "current position : %f %f \n", current_position.x, current_position.y);
	}
}

void PathPlanner::informative_clustering_planning()
{
	printf("Enter informative clustering planning \n");

	vector<pair<CellPoint, CellPoint>> clusters_bounding_box;
	clusters_bounding_box = clustering_information_blocks(gas_information_map);
	if (clusters_bounding_box.size() == 0)
	{
		printf("no cluster found \n");
		fprintf(console_logfile, "no cluster found \n");
		maximal_information_planning();
		return;
	}
	else
	{
		// fprintf(console_logfile, "clusters found \n");
		// Find furthest vertex of each cluster
		double max_dist_to_robot;
		int max_dist_to_robot_index;
		vector<Position> candidate_goal;
		for (int i = 0; i < clusters_bounding_box.size(); i++)
		{
			max_dist_to_robot = 0;
			max_dist_to_robot_index = 0;
			// calculate the distance of each vertex to robot
			Position cluster_vertex[4] = {Position(clusters_bounding_box[i].first.x, clusters_bounding_box[i].first.y, 0),
								Position(clusters_bounding_box[i].first.x, clusters_bounding_box[i].second.y, 0),
								Position(clusters_bounding_box[i].second.x, clusters_bounding_box[i].first.y, 0),
								Position(clusters_bounding_box[i].second.x, clusters_bounding_box[i].second.y, 0)};
			for (int i = 0; i < 4; i++)
			{
				double dist = dist_to_goal(cluster_vertex[i]);
				if (dist > max_dist_to_robot)
				{
					max_dist_to_robot = dist;
					max_dist_to_robot_index = i;
				}
			}
			candidate_goal.push_back(cluster_vertex[max_dist_to_robot_index]);
		}
		// find the closest unvisited point among candidate goals
		double min_dist_to_robot = 1000;
		int min_dist_to_robot_index = 0;
		bool find_goal = false;
		for (int j = 0; j < candidate_goal.size(); j++)
		{
			double dist = dist_to_goal(candidate_goal[j]);
			if (dist < min_dist_to_robot)
			{
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int pos_coord[2];
					position_to_indices_2D(candidate_goal[j].x, candidate_goal[j].y, &pos_coord[0], &pos_coord[1]);
					int idx = posterior->indices_to_index(pos_coord);
					if (cell_visited_list[idx] == 0)
					{
						min_dist_to_robot = dist;
						min_dist_to_robot_index = j;
						find_goal = true;
					}
				}
				else
				{
					min_dist_to_robot = dist;
					min_dist_to_robot_index = j;
					find_goal = true;
				}
			}
		}
		if (!find_goal)
		{
			fprintf(console_logfile, "all the vertex of clusters are visited, use maximal belief planning \n");
			maximal_information_planning();
			return;
		}
		else
		{
			goal_position.x = candidate_goal[min_dist_to_robot_index].x;
			goal_position.y = candidate_goal[min_dist_to_robot_index].y;
			goal_position.heading = 0;
		}

		// fprintf(console_logfile, "clustering goal position : %f %f \n", goal_position.x, goal_position.y);
		// fprintf(console_logfile, "current position : %f %f \n", current_position.x, current_position.y);
	}
}

// **Dijkstra’s Algorithm with Path Tracking**


std::pair<std::vector<double>, std::unordered_map<int, Position>> 
PathPlanner::computeDijkstra(Position start, std::vector<std::tuple<int, int, double>> directions) {
    
    std::unordered_map<int, Position> cameFrom; // Stores the parent of each cell
    std::priority_queue<std::pair<double, Position>, 
                        std::vector<std::pair<double, Position>>, 
                        ComparePosition> pq;

    std::vector<double> dist_map(MAP_SIZE, 1e3);
    dist_map[start.index] = 0;
    pq.push({0, start});
    while (!pq.empty()) {
        auto [currentDist, pos] = pq.top();
        pq.pop();
	
        int x = static_cast<int>(pos.x_idx);
        int y = static_cast<int>(pos.y_idx);
        
        // Expand neighbors
        for (auto [dx, dy, rel_dist] : directions) {
            int nx = x + dx, ny = y + dy;
		Position neg(nx, ny); 
            if (nx >= 0 && ny >= 0 && nx < x_nbins && ny < y_nbins && neg.is_in_obstacle_list(obstacles, nbr_obs)==-1) {
                double newDist = currentDist + rel_dist;
		    if (newDist < dist_map[neg.index]) {
                    dist_map[neg.index] = newDist;
                    pq.push({newDist, neg});
                    cameFrom[neg.index] = {x, y};  // Store parent
                }
            }
        }
    }
    return {dist_map, cameFrom};
}

std::pair<std::vector<double>, std::unordered_map<int, Position>> 
PathPlanner::computeMultiDijkstra(Position start, std::vector<std::tuple<int, int, double>> directions) {
    
    std::unordered_map<int, Position> cameFrom; // Stores the parent of each cell
    std::priority_queue<std::pair<double, Position>, 
                        std::vector<std::pair<double, Position>>, 
                        ComparePosition2> pq;

    double start_dist = 0.0;
    double start_info = gas_information_map[start.index];
    double start_obj = simulation_parameters.scoreSigma * start_info - (1 - simulation_parameters.scoreSigma) * start_dist;
    std::vector<double> obj_map(MAP_SIZE, -1e3);
    obj_map[start.index] = start_obj;
    pq.push({start_obj, start});
    int step_counter = 0;
    while (!pq.empty() && step_counter < simulation_parameters.planning_step) {
    //while (!pq.empty()) {
	  step_counter++;
        auto [currentObj, pos] = pq.top();
        pq.pop();
	   
        int x = static_cast<int>(pos.x_idx);
        int y = static_cast<int>(pos.y_idx);
      //   printf("Current cell is : %d, %d, obj value is: %f \n", x, y, currentObj);
	//   printf("Current cell's parents is : %d, %d \n", cameFrom[pos.index].x_idx, cameFrom[pos.index].y_idx);
        // Expand neighbors
        for (auto [dx, dy, rel_dist] : directions) {
            int nx = x + dx, ny = y + dy;
		Position neg(nx, ny); 
		// when the neg is already in the path, skip it
            Position current = pos;
		bool in_path = false;
		std::unordered_set<int> visited_positions;
		while (cameFrom.find(current.index) != cameFrom.end()) {
                  if (visited_positions.find(current.index) != visited_positions.end()) {
				in_path = true;
				break;
			}
    			visited_positions.insert(current.index);  // Mark node as visited

         		current = cameFrom.at(current.index);
			if (current.x_idx == nx && current.y_idx == ny){
				in_path = true;
				break;
			}	
            }

		if (in_path) continue;
		if (nx >= 0 && ny >= 0 && nx < x_nbins && ny < y_nbins && neg.is_in_obstacle_list(obstacles, nbr_obs)==-1) {
                // normalize the relative distance
		    double normalized_rel_dist = (1 - simulation_parameters.scoreSigma) * rel_dist / (index_distance(1, 1) * simulation_parameters.planning_step);
                // the gas information map is already normalized (at least for PDF)
		    double normalized_info_gain = simulation_parameters.scoreSigma * gas_information_map[neg.index];
		    double relativeObj = normalized_info_gain - normalized_rel_dist;
		    //printf("normalized rel dist: %f, normalized info gain: %f, relative obj: %f \n", normalized_rel_dist, normalized_info_gain, relativeObj);
		    double newObj = currentObj + relativeObj;
		    if (newObj > obj_map[neg.index]) {
                    obj_map[neg.index] = newObj;
                    pq.push({newObj, neg});
                    cameFrom[neg.index] = Position(x, y);  // Store parent
			  //printf("neighbor cell is: %d, %d, parent is: %d, %d, obj value is: %f \n", nx, ny, x, y, newObj);
                }
            }
        }
    }
    return {obj_map, cameFrom};
}

// **Reconstruct Path from Dijkstra Results**
std::list<Position> PathPlanner::reconstructPath(Position goal, const std::unordered_map<int, Position>& cameFrom) {
    std::list<Position> dij_path;
    Position current = goal;
    while (cameFrom.find(current.index) != cameFrom.end()) {
        dij_path.push_back(current);
        current = cameFrom.at(current.index);
    }
    dij_path.push_back(current);  // Add the starting position
    std::reverse(dij_path.begin(), dij_path.end()); // Reverse to get the correct order
    return dij_path;
}

void PathPlanner::reset_metrics(){
    for (int i = 0; i < MAP_SIZE; i++)
    {
	  acc_info_map[i] = 0.0;
	  distance_map[i] = 0.0;
	  scores_map[i] = 0.0;
    }
}

void PathPlanner::markNeighboringCellsAsVisited(int x, int y, int x_nbins_, int y_nbins_, int radius_neighbor, int *cell_visited_list)
{
	// Parcours des cellules dans le voisinage
	for (int dx = -radius_neighbor; dx <= radius_neighbor; ++dx)
	{
		for (int dy = -radius_neighbor; dy <= radius_neighbor; ++dy)
		{
			int neighbor_x = x + dx;
			int neighbor_y = y + dy;

			// Vérifier si la cellule voisine est dans les limites de la grille
			if (neighbor_x >= 0 && neighbor_x < x_nbins_ && neighbor_y >= 0 && neighbor_y < y_nbins_)
			{
				// Calculer l'index dans le tableau
				int index = neighbor_y * x_nbins_ + neighbor_x;
				// Marquer la cellule voisine comme visitée
				cell_visited_list[index] = 1;
			}
		}
	}
}


// **Main Function: Compute Accumulative Information Gain Using Dijkstra**
void PathPlanner::accumulative_Dijkstra() {
     calculate_information_gain();
     reset_metrics();
    // Directions for a 8-connected grid
    std::vector<std::tuple<int, int, double>> directions = {{-1, 0, index_distance(-1,0)}, {1, 0, index_distance(1,0)},
     {0, -1, index_distance(0,-1)}, {0, 1, index_distance(0,1)}, {-1, -1, index_distance(-1,-1)},
     {-1, 1, index_distance(-1,1)}, {1, -1, index_distance(1,-1)}, {1, 1, index_distance(1,1)}};
    // Compute shortest distances and paths from current position to all cells
    auto [distanceMap, cameFrom] = computeDijkstra(current_position, directions);

    double bestScore = -1000; // Initialize the best score
    Position bestTarget;
    std::list<Position> bestPath;

    double min_acc_info = 1e9, max_acc_info = -1e9;
    double min_dist = 1e9, max_dist = -1e9;

    int radius_neighbor = 2;

    for (int y_idx = 0; y_idx < y_nbins; y_idx++) {
        for (int x_idx = 0; x_idx < x_nbins; x_idx++) {
		Position cell(x_idx,y_idx);
            if (cell.is_in_obstacle_list(obstacles, nbr_obs)!=-1) continue; // Skip obstacles

            double pathLength = distanceMap[cell.index];

            if (pathLength == 1e3) continue; // Unreachable cell
 
            // **Retrieve the path to this cell**
            std::list<Position> dij_path = reconstructPath(cell, cameFrom);
            if (dij_path.empty()){
			std::cout<<"no valid path exists";
			continue; // Skip if no path exists	
		} 

            // **Accumulate total information gain along the path**
            double totalInfo = 0.0;
		if (simulation_parameters.infoSum == LINEAR_SUM){
			for (const auto& pos : dij_path) {		    
				totalInfo += gas_information_map[pos.index]; // Sum information along the path
			}
		}
		if (simulation_parameters.infoSum == EXP_SUM){
			for (const auto& pos : dij_path) {		    
				totalInfo += exp(3*gas_information_map[pos.index])-1; // Sum information along the path
			}
		}
		
		acc_info_map[cell.index] = totalInfo;
		distance_map[cell.index] = pathLength;

		// Update min/max values
		min_acc_info = std::min(min_acc_info, totalInfo);
		max_acc_info = std::max(max_acc_info, totalInfo);
		min_dist = std::min(min_dist, pathLength);
		max_dist = std::max(max_dist, pathLength);
        }
    }

    for (int y_idx = 0; y_idx < y_nbins; y_idx++) {
        for (int x_idx = 0; x_idx < x_nbins; x_idx++) {
		Position cell(x_idx,y_idx);
            if (cell.is_in_obstacle_list(obstacles, nbr_obs)!=-1) continue; // Skip obstacles

            if (max_acc_info != min_acc_info) {
            	acc_info_map[cell.index] = (acc_info_map[cell.index] - min_acc_info) / (max_acc_info - min_acc_info);
        	} else {
            	acc_info_map[cell.index] = 0; // Avoid division by zero
       	}
		double totalInfo = acc_info_map[cell.index];

        	if (max_dist != min_dist) {
            	distance_map[cell.index] = (distance_map[cell.index] - min_dist) / (max_dist - min_dist);
       	} else {
            	distance_map[cell.index] = 0; // Avoid division by zero
        	}
		double pathLength = distance_map[cell.index];
            
            if (totalInfo < 1e-2) continue; // Skip no information gain cells
		// skip visited cells
		if (simulation_parameters.visitedList == visitedList_ON){
			if (cell_visited_list[cell.index] == 1)
				continue;
		}

		// **Compute the score based on information-to-distance trade-off**
            double score;
            if (simulation_parameters.scoreFun == RATIO)
		{
			score = (simulation_parameters.scoreSigma * totalInfo) / std::max(0.0, pathLength);
		}
		if (simulation_parameters.scoreFun == LOG)
		{
			score = (simulation_parameters.scoreSigma * totalInfo) / std::log(std::max(0.0, pathLength) + 1);
		}
		if (simulation_parameters.scoreFun == SIGMOID)
		{
			score = 1 / (1 + exp(-simulation_parameters.scoreSigma * totalInfo)) - 1 / (1 + exp(-0.1 * std::max(0.0, pathLength)));
		}
		if (simulation_parameters.scoreFun == SUM)
		{
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_FIX)
			{
				score = simulation_parameters.scoreSigma * totalInfo - (1.0 - simulation_parameters.scoreSigma) * pathLength;
			}
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_ADP){
				// in the beginning, when the uncertainty is high, the dynamic weight is smaller
				double dynamic_weight = 1.0 - posterior->entropy / simulation_parameters.weighting_entropy_ratio;
				if (dynamic_weight < 0.0)
					dynamic_weight = 0.0;
				score = dynamic_weight * totalInfo - (1.0 - dynamic_weight) * pathLength;
			}
		}
		if (simulation_parameters.scoreFun == EXP)
		{
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_FIX)
			{
				score = simulation_parameters.scoreSigma * (exp(totalInfo) - 1.0) - (1.0 - simulation_parameters.scoreSigma) * pathLength;
			}
			if (simulation_parameters.planningStrategy == SUM_DIJKSTRA_ADP){
				// in the beginning, when the uncertainty is high, the dynamic weight is smaller
				double dynamic_weight = 1.0 - posterior->entropy / simulation_parameters.weighting_entropy_ratio;
				if (dynamic_weight < 0.0)
					dynamic_weight = 0.0;
				score = dynamic_weight * (exp(totalInfo) - 1.0) - (1.0 - dynamic_weight) * pathLength;
			}
		}

            scores_map[cell.index] = score;

            if (score > bestScore) {
		    std::list<Position> dij_path = reconstructPath(cell, cameFrom);
		    if (dij_path.empty()){
				std::cout<<"no valid path exists";
				continue; // Skip if no path exists	
		    } 
                bestScore = score;
                bestTarget.copy_position(cell);
                bestPath = dij_path;
            }
	  }
    }

    log_scores_map();
    log_distance_map();
    log_acc_info_map();

    if (bestScore > -1e3) {
        std::cout << "Best cell: (" << bestTarget.x_idx << ", " << bestTarget.y_idx << ") with score: " << bestScore << std::endl;
        std::cout << "Optimal Path: ";
        for (const auto& pos : bestPath) {
            std::cout << "(" << pos.x << ", " << pos.y << ") -> ";
        }
        std::cout << "Goal\n";
	  goal_position.x = bestTarget.x;
	  goal_position.y = bestTarget.y;
	  cell_visited_list[bestTarget.index] = 1;
	  markNeighboringCellsAsVisited(bestTarget.x_idx, bestTarget.y_idx, x_nbins, y_nbins, radius_neighbor, cell_visited_list);
    } else {
        std::cout << "No valid path found.\n";
    }
}


std::list<Position> PathPlanner::multi_objective_Dijkstra() {
	printf("Enter multi_objective_Dijkstra \n");
     double bestScore = -1e2; // Initialize the best score
     Position bestTarget;
     std::list<Position> bestPath;
     calculate_information_gain();
     
     std::vector<std::tuple<int, int, double>> directions = {{-1, 0, index_distance(-1,0)}, {1, 0, index_distance(1,0)},
     {0, -1, index_distance(0,-1)}, {0, 1, index_distance(0,1)}, {-1, -1, index_distance(-1,-1)},
     {-1, 1, index_distance(-1,1)}, {1, -1, index_distance(1,-1)}, {1, 1, index_distance(1,1)}};
     
     // Compute shortest distances and paths from current position to all cells
     auto [costMap, cameFrom] = computeMultiDijkstra(current_position, directions);
     
	for (int y_idx = 0; y_idx < y_nbins; y_idx++) {
		for (int x_idx = 0; x_idx < x_nbins; x_idx++) {
			Position cell(x_idx,y_idx);
			if (cell.is_in_obstacle_list(obstacles, nbr_obs)!=-1) continue; // Skip obstacles
			double cost = costMap[cell.index];
			scores_map[cell.index] = cost;
			if (cost == -1e3) continue; // Unreachable cell
			if (cost > bestScore) {
				bestScore = cost;
				bestTarget.copy_position(cell);
				//printf("current best Target is: %d, %d with score %f \n", bestTarget.x_idx, bestTarget.y_idx, cost);
				std::list<Position> dij_path = reconstructPath(cell, cameFrom);
				if (dij_path.empty()){
					std::cout<<"no valid path exists";
					continue; // Skip if no path exists	
				}
				bestPath = dij_path;			
			}
		}
	}

	log_scores_map();
	log_distance_map();
	log_acc_info_map();

 	if (bestScore > -1e3) {
		std::cout << "Best cell: (" << bestTarget.x_idx << ", " << bestTarget.y_idx << ") with score: " << bestScore << std::endl;
		std::cout << "Optimal Path: ";
		for (const auto& pos : bestPath) {
			std::cout << "(" << pos.x << ", " << pos.y << ") -> ";
		}
		std::cout << "Goal\n";
		goal_position.x = bestTarget.x;
		goal_position.y = bestTarget.y;
	} else {
		std::cout << "No valid path found.\n";
		goal_position.x = current_position.x;
		goal_position.y = current_position.y;
	}
      return bestPath;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PathPlanner::calculate_mean_value_within_cluster(const pair<CellPoint, CellPoint> &cluster, double &mean_value, double &min_value, double &max_value)
{

	double total_value = 0.0;
	int count = 0;
	int bin_values[2];
	int index_mean;
	double value;

	// Initialiser min et max avec des valeurs appropriées
	min_value = std::numeric_limits<double>::max();	   // Valeur initiale très élevée
	max_value = std::numeric_limits<double>::lowest(); // Valeur initiale très basse

	// Iterate on the cluster zone
	for (int x = cluster.first.x; x <= cluster.second.x; ++x)
	{
		for (int y = cluster.first.y; y <= cluster.second.y; ++y)
		{
			// Récupérer l'index pour ces valeurs de paramètres
			std::cout << "x : " << x << std::endl;
			std::cout << "y : " << y << std::endl;
			bin_values[0] = x; // coordonnée x
			bin_values[1] = y; // coordonnée y

			index_mean = indices_to_index_2D(x,y);
			value = gas_information_map[index_mean];
			total_value += value;
			count++;

			// Min and Max values
			if (value < min_value)
			{
				min_value = value;
			}
			if (value > max_value)
			{
				max_value = value;
			}
		}
	}
	// Calculate the mean
	mean_value = (count > 0) ? (total_value / count) : 0.0;
}

double PathPlanner::evaluate_cluster(const pair<CellPoint, CellPoint> &cluster)
{
	// Assuming you have a way to calculate the mean value inside the cluster
	double mean_value, mean_value_min, mean_value_max;
	calculate_mean_value_within_cluster(cluster, mean_value, mean_value_min, mean_value_max);
	double mean_value_normed = (mean_value - mean_value_min) / (mean_value_max - mean_value_min);
	// Calculate the centroid of the cluster for distance measurement
	double centroid_idx_x = (cluster.first.x + cluster.second.x) / 2.0;
	double centroid_idx_y = (cluster.first.y + cluster.second.y) / 2.0;

	Position centroid(centroid_idx_x, centroid_idx_y, 0);

	// Calculate the traveling distance to the centroid
	double TD = dist_to_goal(centroid); // en m
      double TD_min = 0;
      double TD_max = sqrt((source_y_max - source_y_min) * (source_y_max - source_y_min) + (source_x_max - source_x_min) * (source_x_max - source_x_min));
	double TD_normed = 1 - (TD - TD_min) / (TD_max - TD_min); // en m
	// Combine the mean value and traveling distance into a score
	// weights to be adjusted
	double score = mean_value_normed / TD_normed;

	return score;
}

void PathPlanner::move_to_cluster_centroid(const pair<CellPoint, CellPoint> &cluster, double information_map[])
{
	double total_x = 0.0;
	double total_y = 0.0;
	double total_value = 0.0; // To compute the weighted average based on gas values
	int count = 0;

	int cluster_first_x_index = static_cast<int>((cluster.first.x - source_x_min) / x_resol);
	int cluster_first_y_index = static_cast<int>((cluster.first.y - source_y_min) / y_resol); // conversion from meters to cell indexes
	int cluster_second_x_index = static_cast<int>((cluster.second.x - source_x_min) / x_resol);
	int cluster_second_y_index = static_cast<int>((cluster.second.y - source_y_min) / y_resol);

	// Iterate over the area defined by the cluster             x and y are in cell indexes
	for (int x = cluster_first_x_index; x <= cluster_second_x_index; ++x)
	{
		for (int y = cluster_first_y_index; y <= cluster_second_y_index; ++y)
		{

			int index = indices_to_index_2D(x, y);
			double value = information_map[index];

			// Accumulate total coordinates weighted by value
			total_x += x * value;
			total_y += y * value;
			total_value += value;
			count++;
		}
	}
	// Calculate the weighted centroid if total_value is not zero
	if (total_value > 0)
	{
		goal_position.x_idx = total_x / total_value; // Weighted average x
		goal_position.y_idx = total_y / total_value; // Weighted average y
		indices_to_position_2D(goal_position.x_idx, goal_position.y_idx, &goal_position.x, &goal_position.y);
	}
	else
	{
		// Handle the case where there are no valid points
		goal_position.x_idx = (cluster.first.x + cluster.second.x) / 2.0; // Fallback to centroid of the bounding box
		goal_position.y_idx = (cluster.first.y + cluster.second.y) / 2.0;
		indices_to_position_2D(goal_position.x_idx, goal_position.y_idx, &goal_position.x, &goal_position.y);
	}
	goal_position.heading = 0; // Set heading as needed
	printf("Cluster Center: (%f, %f)\n", (cluster.first.x + cluster.second.x) / 2.0, (cluster.first.y + cluster.second.y) / 2.0);
	printf("Moving to centroid: (%f, %f)\n", goal_position.x, goal_position.y);
}

void PathPlanner::weighted_belief_clustering()
{

	for (int i = 0; i < MAP_SIZE; i++)
	{
		gas_information_map[i] = posterior->pdf[i];
	}
	log_gas_information_map();

	vector<pair<CellPoint, CellPoint>> clusters_bounding_box;
	clusters_bounding_box = clustering_information_blocks(gas_information_map);
	if (clusters_bounding_box.size() == 0)
	{
		printf("no cluster found \n");
		maximal_information_planning();
		// informative_point_planning();
		return;
	}
	else
	{

		double max_score = -1; // Assuming scores are non-negative
		int best_cluster_index = -1;
		// Evaluate each cluster using the scoring function
		for (int i = 0; i < clusters_bounding_box.size(); i++)
		{
			double score = evaluate_cluster(clusters_bounding_box[i]); // Scoring function
			if (score > max_score)
			{
				max_score = score;
				best_cluster_index = i;
			}
		}
		pair<CellPoint, CellPoint> chosen_cluster = clusters_bounding_box[best_cluster_index];
		std::cout << "Bords en x du cluster choisi :" << chosen_cluster.first.x << ", " << chosen_cluster.second.x << std::endl;
		std::cout << "Bords en y du cluster choisi :" << chosen_cluster.first.y << ", " << chosen_cluster.second.y << std::endl;
		move_to_cluster_centroid(chosen_cluster, gas_information_map);
	}
}

void PathPlanner::balance_expected_maximal_pdf()
{
	double maximal_pdf_point[2] = {0};
	double expected_pdf_point[2] = {0};
	maximal_pdf_point[0] = plume_model->max_a_post_value[0];
	maximal_pdf_point[1] = plume_model->max_a_post_value[1];
	expected_pdf_point[0] = plume_model->expected_value[0];
	expected_pdf_point[1] = plume_model->expected_value[1];
	goal_position.x = (maximal_pdf_point[0] + expected_pdf_point[0]) / 2;
	goal_position.y = (maximal_pdf_point[1] + expected_pdf_point[1]) / 2;
	goal_position.heading = 0;
}

// void joint_planning()
// {
// 	// std::cout<<"joint_planning"<<std::endl;
// 	//  2 robots, 2 possibilities
// 	if (simulation_parameters.number_of_robots == 2)
// 	{
// 		if (pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, goal_position) < pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[0].goal))
// 		{
// 			// printf("let's swap ************************************************\n");
// 			// swap the goal positions
// 			// inform the other guy
// 			// wb_emitter_set_channel(emitter, 10);
// 			Message msg(ID, friends[0].ID, current_position, -1, goal_position, 1);
// 			// wb_emitter_send(emitter, msg.buffer, msg.buffer_size);
// 			send_msg(msg);
// 			// change mine as well
// 			goal_position.copy_position(friends[0].goal);
// 			// printf("my new goal %f %f %f , my friend's new goal \n", goal_position[0], goal_position[1], goal_position[2]);
// 		}
// 	}
// 	// 3 robots, 6 possibilites
// 	if (simulation_parameters.number_of_robots == 3)
// 	{
// 		if (std::isnan(friends[0].goal.x) || std::isnan(friends[0].goal.y) || std::isnan(friends[1].goal.x) || std::isnan(friends[1].goal.y))
// 			return;
// 		int new_dist = 0, min_dist = 0;
// 		Position my_new_goal;
// 		Position friend_1_new_goal;
// 		Position friend_2_new_goal;
// 		my_new_goal.copy_position(goal_position);
// 		friend_1_new_goal.copy_position(friends[0].goal);
// 		friend_2_new_goal.copy_position(friends[1].goal);
// 		// printf("our old goals : mine %f %f %f, friend 1: %f %f %f, friend 2: %f %f %f \n", goal_position[0], goal_position[1], goal_position[2], friends[0].goal[0], friends[0].goal[1], friends[0].goal[2], friends[1].goal[0],friends[1].goal[1],friends[1].goal[2]);

// 		// 0 : no swap
// 		min_dist = pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[0].goal) + pt_distance(friends[1].position, friends[1].goal);
// 		// 1 : friends swap
// 		new_dist = pt_distance(current_position, goal_position) + pt_distance(friends[0].position, friends[1].goal) + pt_distance(friends[1].position, friends[0].goal);
// 		if (new_dist < min_dist)
// 		{
// 			// printf("we are in 1\n");
// 			min_dist = new_dist;
// 			my_new_goal.copy_position(goal_position);
// 			friend_1_new_goal.copy_position(friends[1].goal);
// 			friend_2_new_goal.copy_position(friends[0].goal);
// 		}
// 		// 2 : I swap with friend1, friend1 swaps with friend2
// 		new_dist = pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, friends[1].goal) + pt_distance(friends[1].position, goal_position);
// 		if (new_dist < min_dist)
// 		{
// 			// printf("we are in 2\n");
// 			min_dist = new_dist;
// 			my_new_goal.copy_position(friends[0].goal);
// 			friend_1_new_goal.copy_position(friends[1].goal);
// 			friend_2_new_goal.copy_position(goal_position);
// 		}
// 		// 3 : I swap with friend1
// 		new_dist = pt_distance(current_position, friends[0].goal) + pt_distance(friends[0].position, goal_position) + pt_distance(friends[1].position, friends[1].goal);
// 		if (new_dist < min_dist)
// 		{
// 			// printf("we are in 3\n");
// 			min_dist = new_dist;
// 			my_new_goal.copy_position(friends[0].goal);
// 			friend_1_new_goal.copy_position(goal_position);
// 			friend_2_new_goal.copy_position(friends[1].goal);
// 		}
// 		// 4 : I swap with friend2, friend2 swaps with friend1
// 		new_dist = pt_distance(current_position, friends[1].goal) + pt_distance(friends[0].position, goal_position) + pt_distance(friends[1].position, friends[0].goal);
// 		if (new_dist < min_dist)
// 		{
// 			// printf("we are in 4\n");
// 			min_dist = new_dist;
// 			my_new_goal.copy_position(friends[1].goal);
// 			friend_1_new_goal.copy_position(goal_position);
// 			friend_2_new_goal.copy_position(friends[0].goal);
// 		}
// 		// 5 : I swap with friend2
// 		new_dist = pt_distance(current_position, friends[1].goal) + pt_distance(friends[0].position, friends[0].goal) + pt_distance(friends[1].position, goal_position);
// 		if (new_dist < min_dist)
// 		{
// 			// printf("we are in 5\n");
// 			min_dist = new_dist;
// 			my_new_goal.copy_position(friends[1].goal);
// 			friend_1_new_goal.copy_position(friends[0].goal);
// 			friend_2_new_goal.copy_position(goal_position);
// 		}

// 		// printf("Our new goals : mine %f %f %f, friend 1: %f %f %f, friend 2: %f %f %f \n", goal_position[0], goal_position[1], goal_position[2], friends[0].goal[0], friends[0].goal[1], friends[0].goal[2], friends[1].goal[0],friends[1].goal[1],friends[1].goal[2]);

// 		// inform the other guys if any changes
// 		if (friend_1_new_goal.x != friends[0].goal.x || friend_1_new_goal.y != friends[0].goal.y)
// 		{
// 			Message msg1(ID, friends[0].ID, current_position, -1, friend_1_new_goal, 1);
// 			send_msg(msg1);
// 		} // else printf("no change for friend %d\n", friends[0].ID);
// 		if (friend_2_new_goal.x != friends[1].goal.x || friend_2_new_goal.y != friends[1].goal.y)
// 		{
// 			Message msg2(ID, friends[1].ID, current_position, -1, friend_2_new_goal, 1);
// 			send_msg(msg2);
// 		} // else printf("no change for friend %d\n", friends[1].ID);

// 		// change mine as well
// 		goal_position.copy_position(my_new_goal);
// 	}
// }

void PathPlanner::KLD_distance_balance()
{
	double best_point[2] = {0};
	posterior->XYmarginal_distance_combined(current_position, 1.0, best_point);
	printf("greedy goal %f %f\n", goal_position.x, goal_position.y);
	printf("best point %f %f\n", best_point[0], best_point[1]);

	float ratio = simulation_parameters.global_local_search_ratio; //ratio of exploration //entropy_posterior/10;//
	float vec1[2] = {0}, vec2[2] = {0};
	// printf("current_position : %f %f %f, goal position : %f %f %f, estimated source : %f %f %f\n", current_position[0],current_position[1],current_position[2],
	//	goal_position[0],goal_position[1],goal_position[2], estimated_source_parameters.source_position[0],estimated_source_parameters.source_position[1],estimated_source_parameters.source_position[2]);
	vec1[0] = goal_position.x - current_position.x; // exploration
	vec1[1] = goal_position.y - current_position.y;
	// vec1[2] = goal_position[2] - current_position[2];
	vec2[0] = best_point[0] - current_position.x; // exploitation (source) //estimated_source_parameters //max_a_post_source_parameters
	vec2[1] = best_point[1] - current_position.y;
	// vec2[2] = estimated_source_parameters.source_position[2] - current_position[2];
	goal_position.x = current_position.x + ratio * vec1[0] + (1 - ratio) * vec2[0];
	goal_position.y = current_position.y + ratio * vec1[1] + (1 - ratio) * vec2[1];
	goal_position.heading = 0; // current_position[2] + ratio*vec1[2] + (1-ratio)*vec2[2];
}

void PathPlanner::goal_outside_obstacles(Position *goal)
{

	double min_dist_to_current = 0.05 + simulation_parameters.sampling_dist_thresh;
	// is the goal position in an obstacle or out of arena or too close to current position?
	int idx = goal->is_in_obstacle_list(obstacles, nbr_obs);
	if (idx == -1 && !position_outside_arena(*goal) && goal->distance_from(current_position) > min_dist_to_current)
	{
		printf(">>> the goal is not in any obstacles or outside of the region; and far from current position\n");
		return;
	}

	// goal inside obstacle
	if (idx != -1)
	{
		// mark the cell as visited so that never plans to go there again
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			int idx = position_to_index_2D(goal->x, goal->y);
			cell_visited_list[idx] = 1;
		}
		printf(">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);
		fprintf(console_logfile, ">>> ! position %f %f is in an obstacle\n", goal->x, goal->y);
		// otherwise, lets try 4 different candidates
		float min_dist = 10000.0;
		float min_dist_unvisited = 10000.0;
		Position nearest_position(*goal);
		Position nearest_unvisited_position(*goal); // for testing priority to unvisited cells
		Position candidates[4] = {Position(obstacles[idx][0], goal->y, 0),
						  Position(obstacles[idx][2], goal->y, 0),
						  Position(goal->x, obstacles[idx][1], 0),
						  Position(goal->x, obstacles[idx][3], 0)};

		int unvisited = 0;

		// for each candidate position
		for (int i = 0; i < 4; i++)
		{
			// check if the distance to candidate is smaller than others
			//  printf(">>> distance from candidate %f %f = %f\n", candidates[i].x, candidates[i].y, goal->distance_from(candidates[i]));
			if (goal->distance_from(candidates[i]) < min_dist)
			{
				if (candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1)
				{
					// not in obstacle
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
				}
				else
				{
					if (simulation_parameters.visitedList == visitedList_ON)
					{
						// mark the cell as visited so that never plans to go there again
						int idx = position_to_index_2D(candidates[i].x, candidates[i].y);
						cell_visited_list[idx] = 1;
					}
				}
			}
		}
		// change the goal position using the pointer
		printf(">>> Obstacle Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		// fprintf(console_logfile, ">>> Obstacle Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);

		goal->copy_position(nearest_position);
	}

	// oustide arena
	if (position_outside_arena(*goal))
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			// mark the cell as visited so that never plans to go there again
			int idx = position_to_index_2D(goal->x, goal->y);
			cell_visited_list[idx] = 1;
		}
		printf(">>> ! position %f %f is outside the region\n", goal->x, goal->y);
		fprintf(console_logfile, ">>> ! position %f %f is outside the region\n", goal->x, goal->y);
		// otherwise, lets try 8 different candidates
		float min_dist = 10000.0;
		float min_dist_unvisited = 10000.0;
		Position nearest_position(*goal);
		Position nearest_unvisited_position(*goal); // to give priority to unvisited cells
		Position candidates[8] = {Position(float(XMIN + SAFETY_MARGIN), goal->y, 0),
						  Position(float(XMAX - SAFETY_MARGIN), goal->y, 0),
						  Position(goal->x, float(YMIN + SAFETY_MARGIN), 0),
						  Position(goal->x, float(YMAX - SAFETY_MARGIN), 0),
						  Position(float(XMIN + SAFETY_MARGIN), float(YMIN + SAFETY_MARGIN), 0),
						  Position(float(XMIN + SAFETY_MARGIN), float(YMAX - SAFETY_MARGIN), 0),
						  Position(float(XMAX - SAFETY_MARGIN), float(YMIN + SAFETY_MARGIN), 0),
						  Position(float(XMAX - SAFETY_MARGIN), float(YMAX - SAFETY_MARGIN), 0)};
		// for each candidate position
		for (int i = 0; i < 8; i++)
		{

			// give priority to unvisited cells but was much slower
			/*
			if (goal->distance_from(candidates[i]) <= min_dist_unvisited && simulation_parameters.visitedList == visitedList_ON){
                        int idx = position_to_index_2D(candidates[i].x, candidates[i].y);
				if (cell_visited_list[idx] == 0){
					//if so, then this is the nearest position
					nearest_unvisited_position.copy_position(candidates[i]);
					min_dist_unvisited = goal->distance_from(candidates[i]);
				}
			}
			*/
			// check if the distance to candidate is smaller than others
			if (goal->distance_from(candidates[i]) < min_dist)
			{
				if (candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i]))
				{
					nearest_position.copy_position(candidates[i]);
					min_dist = goal->distance_from(candidates[i]);
				}
				else
				{
					if (simulation_parameters.visitedList == visitedList_ON)
					{
						int idx = position_to_index_2D(candidates[i].x, candidates[i].y);
						cell_visited_list[idx] = 1;
					}
				}
			}
		}
		// fprintf(console_logfile, ">>> Outside Arena Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}
	// make sure that the chosen goal is not too close from the robot current position so that it will not instantaneously be reached
	double dist_candidate_goal = goal->distance_from(current_position);
	if (goal->distance_from(current_position) < min_dist_to_current)
	{
		if (simulation_parameters.visitedList == visitedList_ON)
		{
			// mark the cell as visited so that never plans to go there again
			int idx = position_to_index_2D(goal->x, goal->y);
			cell_visited_list[idx] = 1;
		}
		printf(">>> ! position %f %f is too close to current position\n", goal->x, goal->y);
		fprintf(console_logfile, ">>> ! position %f %f is too close to current position\n", goal->x, goal->y);
		//  otherwise, lets try 4 different candidates
		Position nearest_position(*goal);
		Position candidates[8] = {Position(goal->x + min_dist_to_current, goal->y + min_dist_to_current, 0),
						  Position(goal->x + min_dist_to_current, goal->y - min_dist_to_current, 0),
						  Position(goal->x - min_dist_to_current, goal->y - min_dist_to_current, 0),
						  Position(goal->x - min_dist_to_current, goal->y + min_dist_to_current, 0),
						  Position(goal->x + min_dist_to_current, goal->y, 0),
						  Position(goal->x - min_dist_to_current, goal->y, 0),
						  Position(goal->x, goal->y + min_dist_to_current, 0),
						  Position(goal->x, goal->y - min_dist_to_current, 0)};
		// for each candidate position
		for (int i = 0; i < 8; i++)
		{
			if (candidates[i].is_in_obstacle_list(obstacles, nbr_obs) == -1 && !position_outside_arena(candidates[i]) && current_position.distance_from(candidates[i]) > min_dist_to_current)
			{
				// Take the first candidate that satisfies the condition
				nearest_position.copy_position(candidates[i]);
				int idx = position_to_index_2D(candidates[i].x, candidates[i].y);
				if (cell_visited_list[idx] == 0)
				{
					break; // give priority to unvisited cell
				}
				// printf(">>> chose it as new position !!! \n");
			}
			else
			{
				if (simulation_parameters.visitedList == visitedList_ON)
				{
					int idx = position_to_index_2D(candidates[i].x, candidates[i].y);
					cell_visited_list[idx] = 1;
				}
			}
		}
		fprintf(console_logfile, ">>> Too close Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
		goal->copy_position(nearest_position);
	}

	// change the goal position using the pointer
	// printf(">>> Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
	// fprintf(console_logfile, ">>> Outside arena Goal position (%f, %f) was changed to (%f, %f)\n", goal->x, goal->y, nearest_position.x, nearest_position.y);
}


// log functions

void PathPlanner::log_scores_map()
{
	scores_map_logfile = fopen(scores_map_logfile_name, "a");

	for (int i = 0; i < (MAP_SIZE) - 1; i++) // Jusqu'à l'avant-dernière valeur
		fprintf(scores_map_logfile, "%.9f, ", scores_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(scores_map_logfile, "%.9f\n", scores_map[(MAP_SIZE) - 1]);

	fclose(scores_map_logfile);
}

void PathPlanner::log_distance_map()
{
	distance_map_logfile = fopen(distance_map_logfile_name, "a");

	for (int i = 0; i < (MAP_SIZE) - 1; i++) // Jusqu'à l'avant-dernière valeur
		fprintf(distance_map_logfile, "%.9f, ", distance_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(distance_map_logfile, "%.9f\n", distance_map[(MAP_SIZE) - 1]);

	fclose(distance_map_logfile);
}

void PathPlanner::log_acc_info_map()
{
	acc_info_map_logfile = fopen(acc_info_map_logfile_name, "a");

	for (int i = 0; i < (MAP_SIZE) - 1; i++) // Jusqu'à l'avant-dernière valeur
		fprintf(acc_info_map_logfile, "%.9f, ", acc_info_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(acc_info_map_logfile, "%.9f\n", acc_info_map[(MAP_SIZE) - 1]);

	fclose(acc_info_map_logfile);
}

void PathPlanner::log_gas_information_map()
{
	gas_information_map_logfile = fopen(gas_information_map_logfile_name, "a");

	// Parcourez jusqu'à l'avant-dernière valeur
	for (int i = 0; i < (MAP_SIZE - 1); i++)
		fprintf(gas_information_map_logfile, "%.9f, ", gas_information_map[i]);

	// Ajoutez la dernière valeur sans virgule
	fprintf(gas_information_map_logfile, "%.9f\n", gas_information_map[MAP_SIZE - 1]);

	fclose(gas_information_map_logfile);
}

void PathPlanner:: maximal_merged_belief_planning()
{
	// compute maximal merged belief map values
	std::vector<int> top1_indices, top2_indices, top3_indices;
	double top1 = -1, top2 = -1, top3 = -1;
	double epsilon = 1e-6;

	for (int i = 0; i < MAP_SIZE; i++) {

		if (simulation_parameters.visitedList == visitedList_ON && cell_visited_list[i] != 0)
			continue;

		double value = posterior->pdf[i];

		if (value > top1 + epsilon) {
			// shift down
			top3 = top2; top3_indices = top2_indices;
			top2 = top1; top2_indices = top1_indices;
			top1 = value; top1_indices = {i};
		}
		else if (fabs(value - top1) < epsilon) {
			top1_indices.push_back(i);
		}
		else if (value > top2 + epsilon) {
			top3 = top2; top3_indices = top2_indices;
			top2 = value; top2_indices = {i};
		}
		else if (fabs(value - top2) < epsilon) {
			top2_indices.push_back(i);
		}
		else if (value > top3 + epsilon) {
			top3 = value; top3_indices = {i};
		}
		else if (fabs(value - top3) < epsilon) {
			top3_indices.push_back(i);
		}
	}

	int top_indices[3] = {-1, -1, -1};
	double top_values[3] = {top1, top2, top3};

	if (!top1_indices.empty())
		top_indices[0] = top1_indices[rand() % top1_indices.size()];
	if (!top2_indices.empty())
		top_indices[1] = top2_indices[rand() % top2_indices.size()];
	if (!top3_indices.empty())
		top_indices[2] = top3_indices[rand() % top3_indices.size()];

	printf("Robot ID [%d]: Top 3 maximal merged belief map values:\n1) Value: %f at index %d\n2) Value: %f at index %d\n3) Value: %f at index %d\n", ID,
	top_values[0], top_indices[0],
	top_values[1], top_indices[1],
	top_values[2], top_indices[2]);

	cell_visited_list[top_indices[0]] = 1;
	cell_visited_list[top_indices[1]] = 1;
	cell_visited_list[top_indices[2]] = 1;

	// assign goals to the maximal merged belief map values
	double best_points[3][2] = {0};
	int max_indices[3] = {top_indices[0], top_indices[1], top_indices[2]};

	for (int i = 0; i < 3; i++) {
		posterior->index_to_param_values(max_indices[i], best_points[i]);
		goal_position_team[i].x = best_points[i][0];
		goal_position_team[i].y = best_points[i][1];
		goal_position_team[i].heading = 0;
	}

	printf("Robot [101]: Assigned possitions to each robot are:\n 1) Robot [101]: %f, %f \n 2) Robot [102]: %f, %f \n 3) Robot [103]: %f, %f\n",
	goal_position_team[0].x, goal_position_team[0].y,
	goal_position_team[1].x, goal_position_team[1].y,
	goal_position_team[2].x, goal_position_team[2].y);
}
