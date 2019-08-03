cmake_minimum_required(VERSION 2.8.3)
project(rgb_map_to_lidar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  #cloud_msgs
)

#find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
#find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
 # CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	#${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	#${GTSAM_LIBRARY_DIRS}
)


add_executable(bag2img src/bag2img.cpp src/fisheyeimg.cpp)
target_link_libraries(bag2img ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(bag2other src/bag2other.cpp)
target_link_libraries(bag2other ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection src/projection_node.cpp src/fisheyeimg.cpp)
target_link_libraries(projection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

