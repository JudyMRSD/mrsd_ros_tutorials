cmake_minimum_required(VERSION 2.8.3)
project(mrsd_ros_tutorials)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
  tf
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(
  config/box_tf_offsets.cfg
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
################
#### Build #####
################

add_executable(in_source_node src/in_source_node.cpp)
target_link_libraries(in_source_node ${catkin_LIBRARIES})


add_executable(linear_node src/linear_node.cpp)
target_link_libraries(linear_node ${catkin_LIBRARIES})


add_executable(abstracted_class_node 
	src/abstracted_class_node.cpp
	src/image_painter.cpp)
target_link_libraries(abstracted_class_node ${catkin_LIBRARIES})

add_executable(display_box_marker src/display_box_marker.cpp)
target_link_libraries(display_box_marker ${catkin_LIBRARIES})
add_dependencies(display_box_marker ${PROJECT_NAME}_gencfg)

add_executable(pointcloud_transform_node src/pointcloud_transform_node.cpp)
target_link_libraries(pointcloud_transform_node ${catkin_LIBRARIES})

