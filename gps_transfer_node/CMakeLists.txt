cmake_minimum_required(VERSION 2.8.3)
project(gps_transfer_node) ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    eigen_conversions
  )

catkin_package(
   CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)
add_executable(gps_transfer_node gps_transfer_node.cpp)
target_link_libraries(gps_transfer_node ${catkin_LIBRARIES})
