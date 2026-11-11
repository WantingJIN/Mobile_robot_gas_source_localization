# Physics-Aware Inference and Informative Exploration for Robust Gas Source Localization with Mobile Robots


## Overview of the project

This project aims at localizing a gas source in an cluttered environments with a multi-robot system, using a probablistic Gas Source Localization (GSL) framework, Source Term Estimation (STE), which contains several modules, namely Gas dispersion prediction, Gas and wind sampling module, Gas feature extraction, Baysian estimation and Informative path planning. Publications related to this project can be found in [^1] [^2] [^3] [^4] [^5] [^6].

[^1]: W. Jin, F. Rahbar, C. Ercolani and A. Martinoli, "Towards Efficient Gas Leak Detection in Built Environments: Data-Driven Plume Modeling for Gas Sensing Robots," 2023 IEEE International Conference on Robotics and Automation (ICRA), London, United Kingdom, 2023, pp. 7749-7755, [doi:10.1109/ICRA48891.2023.10160816](https://doi.org/10.1109/ICRA48891.2023.10160816).
[^2]: W. Jin and A. Martinoli, "Sense in Motion with Belief Clustering: Efficient Gas Source Localization with Mobile Robots," IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 14909-14916, [doi:10.1109/ICRA57147.2024.10611440](https://doi.org/10.1109/ICRA57147.2024.10611440).
[^3]: W. Jin, H. Leroy, N. Bösel-Schmid and A. Martinoli, "Cumulative Informative Path Planning for Efficient Gas Source Localization with Mobile Robots," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Hangzhou, China, 2025, pp. 18088-18094, [doi:10.1109/IROS60139.2025.11246224](https://doi.org/10.1109/IROS60139.2025.11246224).
[^4]: W. Jin, A. Duranceau, İ. K. Erünsal, and A. Martinoli, "Calibration-Free Gas Source Localization with Mobile Robots: Source Term Estimation Based on Concentration Measurement Ranking," 2026 IEEE International Conference on Robotics and Automation (ICRA), Vienna, Austria.
[^5]: W. Jin, M. Mitjà and A. Martinoli, "Probabilistic Multi-Robot Gas Source Localization with Uncalibrated Sensors: A Distributed Estimation Approach", International Symposium on Distributed Autonomous Robotic Systems (DARS), Tokyo, Japan, 2026.
[^6]: W. Jin, T. Wiedemann, H. Fan, A. Lilenthal, A. Martinoli, "Mapping the Wind to Find the Source: Online Physics-Informed Gas Source Localization with Mobile Robots", 2026, Submitted to IEEE Transaction on Robotics (TRO).

Please refer to this README for the tested software versions, dependencies, and instructions for building and configuring the project.

## Repository structure

The main folders in the repository are:

```text
Multi-robot_GSL/
├── Khepera/   # Software and launch scripts for physical Khepera IV robots
├── matlab/   # Dataset processing, analysis, plotting, and visualization scripts
├── openfoam/ # OpenFOAM environments and scripts for generating wind fields
├── protos/   # Shared Webots robot, sensor, and supervisor PROTO/WBO models
├── python/   # DDPM training/inference and Python analysis/visualization tools
├── ros2/     # ROS 2 interfaces, robot coordination, drivers, and STE nodes
├── webots/   # Webots worlds, controllers, gas plugin, libraries, and runtime data
└── Results/  # Example/reference results and default simulation output location
```
- **`webots/`** is the main simulation workspace. Its `controllers/` directory contains the STE controllers, `worlds/` contains the simulation scenes, `plugins/` contains the gas-dispersion physics plugin, `libraries/` contains supporting libraries, and `data/config.yaml` controls runtime method selection.
- **`Khepera/`** contains the code used for experiments with physical Khepera IV robots. It includes the PC-side STE servers, robot-side clients, distributed multi-robot code, launch scripts, an analysis tools.
-  **`ros2/`** contains the ROS 2 workspace, including custom interfaces, the STE package, robot coordination, path planning, and RoboClaw motor-driver integration. See [`ros2/ReadMe.md`](ros2/ReadMe.md) for its build and run notes.
- **`matlab/`** contains scripts for preparing DDPM datasets and plotting.
- **`openfoam/`** contains the training and test environments used to simulate wind fields.
- **`python/`** contains the DeepCFD-based DDPM implementation and utilities for model training, conversion, inference, result analysis, and visualization.
- **`Results/`** is the default location for simulation outputs and also contains selected reference results. Large generated campaigns should normally remain untracked.

For the full DDPM data-generation and deployment pipeline, see [`STE_with_DDPM_user_manual.md`](STE_with_DDPM_user_manual.md).

### Configuration and implemented methods

Runtime parameters are centralized in [`webots/data/config.yaml`](webots/data/config.yaml). The controller reads this YAML file at startup, so the plume model, path planner, sensor model, gas feature and robot number can be changed without recompiling. Parameters that belong to a particular field are grouped below that method in the YAML file.

For example:

```yaml
sensing:
  gasSensor: MOX

gas_feature:
  gasFeature: RELATIVE_VALUE
  RELATIVE_VALUE:
    relativeMethod: Z_SCORE

plume_model:
  plumeModel: DDPM

planning:
  planningStrategy: INFORMATIVE_KLD

robot:
  number_of_robots: 3
  vel_factor: 1.0
```

The principal configurable method families corrsponding to each publication are:

- **Plume model** (`plume_model.plumeModel`): `GAUSSIAN_PLUME`, `DDPM`, or `FDM_PDE`.
  - `DDPM`: **W. Jin, F. Rahbar, C. Ercolani and A. Martinoli, "Towards Efficient Gas Leak Detection in Built Environments: Data-Driven Plume Modeling for Gas Sensing Robots," (2023 ICRA).**
  - `FDM_PDE`: **W. Jin, T. Wiedemann, H. Fan, A. Lilenthal, A. Martinoli, "Mapping the Wind to Find the Source: Online Physics-Informed Gas Source Localization with Mobile Robots", 2026, (Submitted to TRO).**
- **Path planning** (`planning.planningStrategy`): `GREEDY`, `BALANCE_MAXIMAL_EXPECTED`, `RANDOM`, `LAWNMOWER`, `INFORMATIVE_KLD`, `BELIEF_CLUSTERING`, `WEIGHTED_CLUSTERING`, `INFO_CLUSTERING`, `INFO_CENTER`, `SUM_DIJKSTRA_ADP`.
  - `BELIEF_CLUSTERING`: **W. Jin and A. Martinoli, "Sense in Motion with Belief Clustering: Efficient Gas Source Localization with Mobile Robots," (2024 ICRA).**
  - `SUM_DIJKSTRA_ADP`: **W. Jin, H. Leroy, N. Bösel-Schmid and A. Martinoli, "Cumulative Informative Path Planning for Efficient Gas Source Localization with Mobile Robots," (2025 IROS)**.
- **Gas feature** (`gas_feature.gasFeature`): `ABSOLUTE_VALUE`, `RELATIVE_VALUE`, or `GASHIT`.
  - `RELATIVE_VALUE` **W. Jin, A. Duranceau, İ. K. Erünsal, and A. Martinoli, "Calibration-Free Gas Source Localization with Mobile Robots: Source Term Estimation Based on Concentration Measurement Ranking," (2026 ICRA)**.
- **Number of robot**(`robot.number_of_robots`)
  - `3` **W. Jin, M. Mitjà and A. Martinoli, "Probabilistic Multi-Robot Gas Source Localization with Uncalibrated Sensors: A Distributed Estimation Approach", (2026 DARS).**
- **Sensor model** (`sensing.gasSensor`): `PID`, `MOX`, `NOISY_MOX`, `SLOW_MOX`. Sensor calibration and temporal/spatial processing are selected separately with `calibrationMode` and `processingMode`.


Additional YAML sections select the sampling behavior, Bayesian evaluation method, wind information and estimation method, model uncertainty, multi-robot team strategy, map-coverage strategy, and termination condition. See the inline comments in `config.yaml` for the accepted values and method-specific parameters.



## Dependencies
### Openfoam
openfoam v9: https://openfoam.org/download/9-ubuntu/ 

Openfoam is an open-source tool used to simulate wind data inside the arena given the obstacle positions and the constant  wind speed at the inlet. The speed at the outlet is given by the zero-gradient condition to simulate free exit.

### Webots

webots 2021b

Webots is a high fidelity physics simulator that is used to simulate the gas inside the arena thanks to the Webots gas dispersion plugin and the wind data produced with OpenFoam. The DDPM is trained in simulation with Webots data and the known obstacle positions.

### - How to compile the controller?

##### Requirements

The current controller is built as C++17 code and has been tested with the following toolchain:

- GCC/G++ 11.5.0 (check with `gcc --version` and `g++ --version`)
- CMake 3.12 or newer
- CUDA 11.7 (check with `nvcc --version`)
- LibTorch `libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117`
- Python 3.8
- Webots R2021b
- OpenFOAM 9

A CPU-only LibTorch distribution may also be used if CUDA inference is not required, but it has not been tested with the configuration documented here.

The controller also depends on:

- [Eigen3](https://eigen.tuxfamily.org/)
- [GNU Scientific Library (GSL)](https://www.gnu.org/software/gsl/)
- [MatIO](https://github.com/tbeu/matio)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [SuperLU](https://portal.nersc.gov/project/sparse/superlu/)
- the Khepera Webots support library included under `webots/libraries/khepera4`

On Ubuntu, the system dependencies can be installed with:

```bash
sudo apt install build-essential cmake python3.8-dev libeigen3-dev \
  libgsl-dev libmatio-dev libyaml-cpp-dev libsuperlu-dev
```

Download the matching LibTorch C++ distribution from the [PyTorch installation page](https://pytorch.org/get-started/locally/). This project was tested with LibTorch 1.13.1 using the C++11 ABI and CUDA 11.7.

##### Configure the paths

1. Export `WEBOTS_HOME` as the directory containing the Webots executable and its `include` and `lib` directories:

   ```bash
   export WEBOTS_HOME=/path/to/webots
   ```

2. Set `CMAKE_PREFIX_PATH` in `webots/controllers/controller_STE_multi-robot/CMakeLists.txt` to the extracted LibTorch directory.
3. Set `CMAKE_MODULE_PATH` to this repository's `webots/libraries` directory so that CMake can find `FindMatIO.cmake`.

The current CMake file contains developer-specific absolute paths. Replace these paths before building from a new clone.
   

#### Compile

```bash
cd webots/controllers/controller_STE_multi-robot
mkdir -p build
cd build
cmake ..
make -j
```

### - How to run the code
1. navigate to the webots folder in the project `cd your-path-of-project/webots`

There are several controllers available as well as example sh files in this folder that run the code with various parameters.

#### Controller STE multi robot


The main part of the code is in the file `controller_STE_multi-robot.cpp`.

To test the performance of the controller under different environment and setip:

`your-path-to-project/webots/test_maps_evaluation.sh`,

The gas source release rate can also be modified here. It is defined in the file `your-path-to-project/webots/data/plugin_parameters/releaseAmount.txt`. In the file, remove the  '--no-rendering' option to visualize the 3D simulation in Webots. Each configuration is tested for a number of source locations equally spread along the y direction defined by the user as nbr_source_pos.


Experiment results are stored in `your-path-to-project/Results`.



### - Generate plots for visualization

A general example of code for visualizing performance is `your-path-to-project/matlab/test_maps_results_comparison.m` which separates experiments according to the map in which they were conducted, and then evaluates algorithm performance for each map and compares the results.  Other example files with a similar structure but comparing other parameters are `sensor_type_comp.m`, `release_amount_comp.m`, `vel_factor_results_comparison.m` etc. 

Other useful MATLAB files are 
- `your-path-to-project/matlab/all_display.m`: This plots the robot trajectory with collected measurements, evolution of entropy and of concentration over time for a whole set of experiments.
- `your-path-to-project/matlab/show_posterior_anim.m`: This plots and stores the posterior PDF of the source location map for every iteration of one experiment.
- `your-path-to-project/matlab/show_info_map.m`: For one experiment, plots and stores the robot trajectory, PDF map update and the map used for belief clustering navigation planning with the resulting cluster. 
- `your-path-to-project/matlab/show_info_map.m`: For one experiment, plots and stores the robot trajectory, PDF map update and the map used for belief clustering navigation planning with the resulting cluster. 
- `your-path-to-project/matlab/show_visited_list.m`: For one experiment, plots the robot trajectory and cells of the map marked as visited during the experiment.














