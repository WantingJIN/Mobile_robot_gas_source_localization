# Plume model learning for STE


## Overview of the project

This project aims at localizing a gas source in an cluttered arena with a steady wind, using the Source Term Estimation (STE) framework and gas sensor measurements collected by Khepera IV mobile robots. A Data-Driven Plume Model (DDPM) predicts gas concentration measurements. Details on this project can be found in [^1] and [^2].

[^1]: Rahbar, F., Marjovi, A., & Martinoli, A. (2019, May). An algorithm for odor source localization based on source term estimation. In 2019 International Conference on Robotics and Automation (ICRA) (pp. 973-979). IEEE.

[^2]: Jin, W., Rahbar, F., Ercolani, C., & Martinoli, A. (2023, June). Towards Efficient Gas Leak Detection in Built Environments: Data-Driven Plume Modeling for Gas Sensing Robots. In Proceedings of the 2023 IEEE International Conference on Robotics and Automation (ICRA).

Please refer to this ReadME file for details on the required software versions and instructions on how to build the project.

## Openfoam 
openfoam v9: https://openfoam.org/download/9-ubuntu/ 

Openfoam is an open-source tool used to simulate wind data inside the arena given the obstacle positions and the constant  wind speed at the inlet. The speed at the outlet is given by the zero-gradient condition to simulate free exit.

## Webots 

webots 2021b

Webots is a high fidelity physics simulator that is used to simulate the gas inside the arena thanks to the Webots gas dispersion plugin and the wind data produced with OpenFoam. The DDPM is trained in simulation with Webots data and the known obstacle positions.

### - How to compile the controller?

##### Requirement: 

- gcc/g++ version: 9.4.0 (check your gcc version with gcc -v)
- cuda version:  12.0 (check your cuda version with nvcc --version) download from https://developer.nvidia.com/cuda-downloads
- libtorch: libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117 download from https://pytorch.org/get-started/locally/ by selecting stable; Linux; LibTorch; C++; CUDA 11.7

##### Modify the CMakeLists:
1. Modify the path of the webot: `set(WEBOTS_HOME "your-path-of-webots")`
2. Modify the path of the libtorch: `set(CMAKE_PREFIX_PATH "your-path-of-pytorch")`
3. Modify the path of the FindMATIO.cmake path: `set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/your-path-of-project/webots/libraries")` 
   

#### compile
1. navigate to the folder of the controller `cd path-to-controller`
2. creat the build folder `mkdir build`
3. navigate to the build folder `cd build`
4. cmake `cmake ..`
5. make `make`

### - How to run the code
1. navigate to the webots folder in the project `cd your-path-of-project/webots`

There are several controllers available as well as example sh files in this folder that run the code with various parameters.

#### Controller STE multi robot

The current version of the STE multi robot controller contains the continuous STE implementation with the DDPM in simulation, as well as the binary gas hit and relative concentration strategies proposed to improve robustness to different release rates.

The main part of the code is in the file `controller_STE_multi-robot.cpp`.

Simulation parameters are set in `webots_functions.cpp `in the simulation_parameters structure. The definitions for some of these parameter values are in `controller_STE_multi-robot.h`. IN  `constant.h`, the definitions of useful values, such as the maximum number of iterations, are given.

In order to be able to run the code with different parameters without having to recompile everything, some of the parameter values are taken from text files for which paths are specified in `webots_functions.cpp`. These text files are in  `your-path-of-project/webots/data/simulation_parameters`.

There are three available types of likelihood to choose from in the simulation_parameters:

- CONCENTRATION: for the original likelihood computation with the absolute concentration error
- RELATIVE_CONCENTRATION: for the likelihood based on the error between measured and modeled relative measurements computed as cumulative density functions
- GASHIT: for the approach with gas detection events. Different gas hit strategies have been proposed and are commented in the likelihood_gas_hit function in `controller_STE_multi-robot.cpp` for future modifications


To run the code in different environment maps in simulation, different sh files are provided with examples of how to vary the tested simulation parameters.

The most straightforward example is `your-path-to-project/webots/test_maps_evaluation.sh`, which runs experiments in different test maps. The gas source release rate can also be modified here. It is defined in the file `your-path-to-project/webots/data/plugin_parameters/releaseAmount.txt`. In the file, remove the  '--no-rendering' option to visualize the 3D simulation in Webots. Each configuration is tested for a number of source locations equally spread along the y direction defined by the user as nbr_source_pos.

Other example sh files following a similar structure are proposed to vary other specific parameters, such as `test_different_vel_factor.sh` to vary the robot constant velocity when the constant velocity controller is used, or `test_different_sampling_dist_thresh.sh`.

Experiment results are stored in `your-path-to-project/Results`.



### - Generate plots for visualization

A general example of code for visualizing performance is `your-path-to-project/matlab/test_maps_results_comparison.m` which separates experiments according to the map in which they were conducted, and then evaluates algorithm performance for each map and compares the results.  Other example files with a similar structure but comparing other parameters are `sensor_type_comp.m`, `release_amount_comp.m`, `vel_factor_results_comparison.m` etc. 

Other useful MATLAB files are 
- `your-path-to-project/matlab/all_display.m`: This plots the robot trajectory with collected measurements, evolution of entropy and of concentration over time for a whole set of experiments.
- `your-path-to-project/matlab/show_posterior_anim.m`: This plots and stores the posterior PDF of the source location map for every iteration of one experiment.
- `your-path-to-project/matlab/show_info_map.m`: For one experiment, plots and stores the robot trajectory, PDF map update and the map used for belief clustering navigation planning with the resulting cluster. 
- `your-path-to-project/matlab/show_info_map.m`: For one experiment, plots and stores the robot trajectory, PDF map update and the map used for belief clustering navigation planning with the resulting cluster. 
- `your-path-to-project/matlab/show_visited_list.m`: For one experiment, plots the robot trajectory and cells of the map marked as visited during the experiment.














