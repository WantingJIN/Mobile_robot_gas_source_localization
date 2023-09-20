# Plume model learning for STE

## Overview of the project
## Openfoam 

## Webots 

### - How to compile the controller?

##### Requirement: 

- gcc/g++ version: 9.0
- libtorch: libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117

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