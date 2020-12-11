cmake_minimum_required(VERSION 3.0)
project(gap_crossing)

add_compile_options(-std=c++11)

find_package(ompl 1.4.0 REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(Compass src/Compass.cpp)
target_link_libraries(Compass ${OMPL_LIBRARIES})
