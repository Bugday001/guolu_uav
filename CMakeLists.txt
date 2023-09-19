cmake_minimum_required(VERSION 3.0.2)
project(px4_offboard)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED
    pcl_ros
    pcl_conversions
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES px4_offboard
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(tf_cloud_fast_planner src/tf_cloud_fast_planner.cpp)

target_link_libraries(tf_cloud_fast_planner
  ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(position_to_mavros_node src/position_to_mavros.cpp)
target_link_libraries(position_to_mavros_node
  ${catkin_LIBRARIES}
)

add_executable(odom2pose_node src/odom2pose.cpp)
target_link_libraries(odom2pose_node
  ${catkin_LIBRARIES}
)

add_executable(avoidance_node src/avoidance.cpp)

target_link_libraries(avoidance_node
  ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(transform_node src/transformCPandIMU.cpp)

target_link_libraries(transform_node
  ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
)