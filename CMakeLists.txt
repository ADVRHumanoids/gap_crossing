cmake_minimum_required(VERSION 3.0)
project(gap_crossing)

add_compile_options(-std=c++11)

find_package(ompl 1.5.0 REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(SE2_discrete_control src/SE2_discrete_control.cpp)
target_link_libraries(SE2_discrete_control ${OMPL_LIBRARIES})