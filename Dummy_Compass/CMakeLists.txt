cmake_minimum_required(VERSION 3.0)
project(gap_crossing)

add_compile_options(-std=c++11)

find_package(ompl 1.4.0 REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(biped_ompl src/biped_ompl.cpp)
target_link_libraries(biped_ompl ${OMPL_LIBRARIES})
