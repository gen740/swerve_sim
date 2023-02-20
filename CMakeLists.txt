cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(test SHARED test.cc)
target_link_libraries(test ${GAZEBO_LIBRARIES})
