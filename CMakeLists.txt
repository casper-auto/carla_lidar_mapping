cmake_minimum_required(VERSION 3.0.2)
project(lidar_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_mapping
  src/lidar_mapping_node.cpp
  src/lidar_mapping.cpp
)

target_link_libraries(lidar_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(lidar_mapping
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
