cmake_minimum_required(VERSION 3.0.2)
project(self_balancing_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  xacro
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  gazebo_ros
  gazebo_plugins
 
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    xacro
    urdf
    controller_manager
    joint_state_controller
    robot_state_publisher
    gazebo_ros
    gazebo_plugins
  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

