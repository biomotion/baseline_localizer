cmake_minimum_required(VERSION 3.0.2)
project(baseline_localizer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_msgs
  tf_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baseline_localizer
#  CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs tf2 tf2_msgs tf_conversions
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(localizer src/localizer_node.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES})

add_executable(pub_map src/pub_map_node.cpp)
target_link_libraries(pub_map ${catkin_LIBRARIES})

