cmake_minimum_required(VERSION 2.8.3)
project(gridmapper)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters
  tf
  tf2
  tf2_ros
  image_transport
  std_msgs
  message_generation
)

find_package(octomap REQUIRED)

add_service_files(DIRECTORY srv
  FILES
  classify.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(
  gridmapper_node
  src/gridmapper_node.cpp
)

add_executable(
  tf_broadcaster
  src/tf_broadcaster.cpp
)

add_executable(
  img_pub
  src/img_pub.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  gridmapper_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  tf_broadcaster
  ${catkin_LIBRARIES}
)

target_link_libraries(
  img_pub
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS scripts/classifier_oll.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)