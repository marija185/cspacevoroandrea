cmake_minimum_required(VERSION 2.4.6)
project(cspacevoroandrea)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# COMPILER FLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cspacevoroandrea
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(cspacevoroandrea src/dynamicvoronoi.cpp src/cspacevoronoilayer.cpp src/cspacevoronoi.cpp src/OrientationIntervals.cpp src/AStar.cpp src/OINodeConnector.cpp src/OISearchNode.cpp src/SearchNode.cpp src/DStarSearchNode.cpp src/OIDStarSearchNode.cpp src/OIDStar.cpp)

add_executable(example src/example.cpp)
target_link_libraries(example cspacevoroandrea ${catkin_LIBRARIES})


