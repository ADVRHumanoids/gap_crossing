cmake_minimum_required(VERSION 3.0)
project(gap_crossing)

add_compile_options(-std=c++14)

find_package(ompl 1.4.0 REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(Quadruped_Unicycle src/Quadruped_Unicycle.cpp)
target_link_libraries(Quadruped_Unicycle ${OMPL_LIBRARIES})
