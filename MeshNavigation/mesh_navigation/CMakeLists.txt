cmake_minimum_required(VERSION 3.0.2)
project(mesh_navigation)

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
  shape_msgs
  ompl
  message_generation
)

find_package(octomap REQUIRED)

find_package(fcl REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  MeshArray.msg
  FrameRecon.msg
)

add_service_files(
  # DIRECTORY ../../LidarPointCloudReconstruction/hash_fusion/srv
  DIRECTORY srv
  FILES
  multi_recon.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  shape_msgs
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES husky_mesh_map
CATKIN_DEPENDS 
message_runtime
# nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

aux_source_directory(src/Boundary Boundary_DIR_SRCS)
add_library(Boundary STATIC ${Boundary_DIR_SRCS})
# add_library(src/MeshMap.cpp)
###### build ########
add_executable(${PROJECT_NAME}_Navigation src/MeshNavigation.cpp src/MeshMap.cpp)
add_dependencies(${PROJECT_NAME}_Navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_Navigation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} Boundary)

# add_executable(${PROJECT_NAME}_PathPlaner_RRT src/PathPlanning/RRT.cpp)
# add_dependencies(${PROJECT_NAME}_PathPlaner_RRT ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_PathPlaner_RRT ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

# add_executable(${PROJECT_NAME}_PathPlaner_AStar src/PathPlanning/A-Star.cpp)
# add_dependencies(${PROJECT_NAME}_PathPlaner_AStar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_PathPlaner_AStar ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})