cmake_minimum_required(VERSION 3.0)

project(road_boundary_detection)


#set(CMAKE_CXX_FLAGS "-std=c++14 -march=native" )
set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")
set(LIMBO_INCLUDE_DIRS ./third/limbo/src)

find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Boost)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

#CATKIN_DEPENDS pcl_ros roscpp sensor_msgs

add_definitions(${PCL_DEFINITIONS})

#add_subdirectory(./lib)

include_directories(
    include
    /home/fmaaf/catkin_ws/src/road_boundary_detection/third/limbo/src
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}  
#   ${Qt5Charts_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS} 
    ${LIMBO_INCLUDE_DIRS}
    )

link_directories(${PCL_LIBRARY_DIRS} /home/fmaaf/catkin_ws/src/road_boundary_detection/third/limbo/build/src)

set(HEADER_FILES 
    src/types.h
    src/boundary_points.h
    src/cloud_mapper.h
    src/curbfit.h
    src/feature_points.h
    src/gaussian_process.h
    src/grid_map.h
    src/ground_segment.h
    src/myfilter.h
    src/math_utils.h
    src/read_frametransform.h
    src/read_pose.h
    src/rectify.h
    src/road_segmentation.h
    src/tracking.h
    src/ray_filter.h
)

set(SOURCE_FILES 
    src/road_boundary_detection_node.cpp 
    src/boundary_points.cpp
    src/cloud_mapper.cpp
    src/curbfit.cpp
    src/feature_points.cpp
    src/gaussian_process.cpp
    src/grid_map.cpp
    src/ground_segment.cpp
    src/myfilter.cpp
    src/read_frametransform.cpp
    src/read_pose.cpp
    src/rectify.cpp
    src/road_segmentation.cpp
    src/tracking.cpp
    src/ray_filter.cpp
)

add_executable(
    ${PROJECT_NAME}_node
    ${HEADER_FILES} 
    ${SOURCE_FILES}
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )