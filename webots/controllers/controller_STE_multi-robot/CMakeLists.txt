# Description: Webots Controller build using CMake.
# Example: Copy CMakeLists.txt in $WEBOTS_HOME/projects/robots/gctronic/e-puck/controllers/e-puck/ and execute the following command:
#     rm -fr build e-puck; mkdir build; cd build; cmake ..; make; cd ..

cmake_minimum_required(VERSION 3.0)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)  # controller directory name

project(${PROJECT})

set(WEBOTS_HOME "/home/wjin/Softwares/webots-R2021b/webots")

include_directories(../../libraries/khepera4)
include_directories(/usr/include/eigen3)
include_directories(/usr/include/python3.8)

find_package(Python3 REQUIRED)
set(PYTHON_EXECUTABLE /usr/bin/python3)
include_directories(${PYTHON3_INCLUDE_DIRS})
link_directories(${Python3_LIBRARY_DIRS})

# find_library(NVWA_LIBRARY NAMES nvwa)

link_directories($ENV{WEBOTS_HOME}/lib/controller)


link_directories("../../libraries/khepera4" "/usr/lib/python3.8/" "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/")


file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} "nvwa/debug_new.cpp")

set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)



#set(CMAKE_EXE_LINKER_FLAGS "-lController")
add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT} ${LIBRARIES})
target_link_libraries(${PROJECT} ${PYTHON3_LIBRARIES})
#target_link_libraries(${PROJECT} -g -L"/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/webots/libraries/khepera4")
target_link_libraries(${PROJECT} -g -lk4_webots -lgsl -lgslcblas)
# Copy the controller to the project directory
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
