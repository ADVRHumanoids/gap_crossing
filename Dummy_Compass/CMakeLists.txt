cmake_minimum_required(VERSION 3.0)
project(gap_crossing)

add_compile_options(-std=c++11)

find_package(ompl 1.4.0 REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(compass_ompl src/compass_ompl.cpp)
target_link_libraries(compass_ompl ${OMPL_LIBRARIES})
