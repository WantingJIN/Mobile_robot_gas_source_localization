# Multi-Robot GSL Task


## Controller MRS
The main part of the code is in the file `controller_STE_multi-robot.cpp`.

Simulation parameters are set in `webots_functions.cpp `in the simulation_parameters structure. The definitions for some of these parameter values are in `controller_STE_multi-robot.h`. IN  `constant.h`, the definitions of useful values, such as the maximum number of iterations, are given.

In order to be able to run the code with different parameters without having to recompile everything, some of the parameter values are taken from text files for which paths are specified in `webots_functions.cpp`. These text files are in  `your-path-of-project/webots/data/simulation_parameters`.

There are several parameters to tune for the simulation on ``config.txt``:

- mapCoverageStrategy: to choose region allocator, which can be {"NONE", "VORONOI", "LLOYD_D", "LLOYD_P", "REGION_D" and "REGION_P"}.
- mapCoverageSeed: to choose the seed for the region allocator, which can be {"ROBOT_CELL", "MAX_CELL"}
- distObstacleAware: to consider obstacles or not, which can be {"DIJKSTRA", "EUCLIDEAN"}
- increaseFreqBM: in case KLD information gain-based improvement wants to be tested, which can be {"YES", "NO"}
- update_time: to specify constant period of time considered to compare KLD
- KLD_thresh: to specify threshold for the KLD comparison


### - How to run the code
1. navigate to the webots folder in the project `cd your-path-of-project/webots`

2. open bash script code `code test_maps_evaluation.sh`

    2.1. introduce desired number of iterations in line 16, concretelly the variable `nbr_source_pos`

    2.2. introduce desired map to be simulated in line 47, concretelly `for m in map_number`, where map_number is the variable to be modified

3. execute following command in terminal `./test_maps_evaluation.sh`

### - How to run large number of simulations
These codes are useful to run a large number of simulations on a single map. In MRS, it is typical to automatically test several combinations of region allocators and path planners, where manual launching is not worth the effort. 

As a suggestion, if it is desired to simulate on several different maps, this file will be modified once per each map, in order to store the results on different folders, for instance, /map0, /map5 and /map6.

1. navigate to the webots folder in the project `cd your-path-of-project/webots`

2. open bash script code `code run_all_simulations.sh`

    2.1. introduce the map directory where to store the results automatically, concretelly the variable ``RESULTS_BASE="../Results/your_directory``. 

3. open bash script code `test_maps_evaluation.sh`

    3.1 introduce desired number of iterations for each combination in line 16, concretelly the variable `nbr_source_pos`

    3.2. introduce desired map to be simulated in line 47, concretelly `for m in map_number`, where map_number is the variable to be modified

4. execute following command in terminal `./run_all_simulations.sh`

### - Residual simulations
More combinations are specified on `run_all_simulations_extra.sh`. 

The modifications to be done are exactly the same as before.

### - Reallocate iterations
In case some iterations for a predefined configuration of region allocation and path planner are not in the desired folder, the code `move_numeric_dirs.sh` performs the mapping between the assigned Configuration ID (done for the Master thesis results section) with the actual value of the ``config.txt``.  

The modifications to be done are exactly the same as before.

1. introduce the desired folder where to be reallocated ``COMBINATIONS_DIR="$RESULTS_DIR/desired_folder``



## - Generate plots for visualization
Important: MATLAB files on that section need to be modified the directory specified on variable `base_path`, which has to be modified according to where the simulation folder -which contains the logs for the simulation- is located.

In order to visualize the simulation in a dynamic plot:

- if real belief probabilistic values want to be observed as well as the trajectories, then `your-path-to-project/matlab/MT_Multirobot_Simulation_BeliefAndTrajectories.m` should be executed. 

- if clustered environment want to be observed as well as the trajectories, then `your-path-to-project/matlab/MT_Multirobot_Simulation_ClustersAndTrajectories.m` should be executed. 

Other useful MATLAB files are 

- `your-path-to-project/matlab/MT_relative_measurements.m`: This shows the evolution of the relative measurements for each of the robots of the MRS
- `your-path-to-project/matlab/MT_lloyd_visualization.m`: This shows in a dynamic plot how the clusters look along the iterations carried out by Lloyd algorithm on region allocation
- `your-path-to-project/matlab/MT_cluster_belief_map_cool_BMupdate.m`: Used to observe the evaluation of KLD value along the simulation, i.e., different clusters configurations
- `your-path-to-project/matlab/MT_compare_sim_results_multirobot.m`: Used to obtain the results (boxplot) from a simulated configuration (tipically 30 iterations).
- `your-path-to-project/matlab/MT_compare_allsim_results_multirobot.m`: Used to obtain the results (boxplot) from a group of simulated configurations. Useful to compare between different configurations. `MT_compare_sim_results_multirobot` should be executed always before this one.
- `your-path-to-project/matlab/MT_compare_allsim_results_multirobot_report.m`: Used to obtain the results (boxplot) from a group of simulated configurations, cool plot for the report. You should specify on manual_selection = {'config1','config2', 'config3'}; or simply filter by means of `filter_X = []` (for region allocation), `filter_Y = [];` (for the seed), `filter_Z = [];` (for path planner) and `filter_K = [1];` (for gas feature). For instance, if filter_X = [1], all configurations containing Voronoi region allocator will be displayed, independently of the seed, path planner, gas feature...
- `your-path-to-project/matlab/MT_compare_allsim_results_by_groupmap_numericlabels.m`: Used Used to obtain the results (boxplot) from a group of simulated configurations accross different scenarios. Useful to test the complexities of maps.
