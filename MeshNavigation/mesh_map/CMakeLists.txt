cmake_minimum_required(VERSION 3.0.2)
project(mesh_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
IF (CMAKE_BUILD_TYPE STREQUAL Debug)      
    ADD_DEFINITIONS(-DDEBUG)                        
ENDIF()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  octomap_ros
  octomap_msgs
)

find_package(octomap REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_mesh_map
#  CATKIN_DEPENDS nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories( ${OCTOMAP_INCLUDE_DIRS})
aux_source_directory(src DIR_SRCS)

###### build ########
add_executable(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
