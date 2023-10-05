# Plume model learning for STE

## Overview of the project

This project builds upon the traditional STE for GSL framework, adapting it for complex environments. We've created a data-driven plume model to serve as a substitute for the original plume dispersion model in the STE framework. This project involves multiple steps and uses various platforms and programming languages.

Please refer to this ReadME file for details on the required software versions and instructions on how to build the project.

## Openfoam

openfoam v9: https://openfoam.org/download/9-ubuntu/ 

## Webots 

webots 2021b

### - How to compile the Webots controller?

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