cmake_minimum_required(VERSION 3.5)
project(ste_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)   
find_package(ste_interfaces REQUIRED)
find_package(std_srvs REQUIRED)


# libtorch
set(CMAKE_PREFIX_PATH /home/wanting/Softwares/libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117/libtorch)
find_package(Torch REQUIRED NO_DEFAULT_PATH PATHS ${CMAKE_PREFIX_PATH})

# MatIO
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/home/wanting/Workspace/kheperaodor/plume_model_learning/webots/libraries")
find_package(MatIO REQUIRED)
# MatIO gives: MATIO_INCLUDE_DIRS, MATIO_LIBRARIES

# Eigen, SuperLU, Python3, etc.
include_directories(/usr/include/eigen3)

find_path(SUPERLU_INCLUDE_DIR slu_Cnames.h
  PATHS /usr/include /usr/include/superlu /usr/local/include /opt/homebrew/include
  PATH_SUFFIXES superlu)

find_library(SUPERLU_LIBRARY superlu
  PATHS /usr/lib /usr/local/lib /opt/homebrew/lib $ENV{CONDA_PREFIX}/lib)

include_directories(${SUPERLU_INCLUDE_DIR})

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include_directories(${Python3_INCLUDE_DIRS})
link_directories(${Python3_LIBRARY_DIRS})

link_directories(
  "/usr/lib/python3.8/"
  "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/")

include_directories(
  ${MATIO_INCLUDE_DIRS}
)
set (LIBRARIES m -lgsl -lgslcblas -lm)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Sources & headers ------------------------------------------------------

# All your .cpp files (FSM, ROS bridge, whatever)
# Better to list them explicitly, but globbing is okay to start
file(GLOB CPP_SOURCES "src/*.cpp")

# --- Build the ROS 2 node executable ----------------------------------------

add_executable(controller_node
  ${CPP_SOURCES}
)

# Public include dirs for this target (your headers are in include/my_controller_pkg)
target_include_directories(controller_node PUBLIC
  include                        # <-- your package's include/
  /usr/include/eigen3
  ${SUPERLU_INCLUDE_DIR}
  ${MATIO_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# Link ROS 2 dependencies
ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  ste_interfaces
  std_srvs
  ament_index_cpp 
)

# Link your external libs
target_link_libraries(controller_node
  ${LIBRARIES}
  ${Python3_LIBRARIES}
  -g -lgsl -lgslcblas
  "${TORCH_LIBRARIES}"
  ${MATIO_LIBRARIES}
  ${SUPERLU_LIBRARY}
)

set_property(TARGET controller_node PROPERTY CXX_STANDARD 17)

# --- Installation so ros2 run can find it -----------------------------------

install(
  TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

# --- ament package ----------------------------------------------------------
ament_package()
