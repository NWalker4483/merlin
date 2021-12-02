cmake_minimum_required(VERSION 2.8.3)
project(merlin_hardware_interface)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
)
SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()


catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
  LIBRARIES 
    merlin_hardware_interface
)

include_directories(
  include/
  "$ENV{EIGEN3_INCLUDE_DIR}"
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(merlin_hardware_interface src/merlin_hardware_interface.cpp)
target_link_libraries(merlin_hardware_interface
	${catkin_LIBRARIES})

add_executable(merlin_hardware_interface_node src/merlin_hardware_interface_node.cpp)
add_dependencies(merlin_hardware_interface_node merlin_hardware_interface)

target_link_libraries(merlin_hardware_interface_node
  merlin_hardware_interface
	${catkin_LIBRARIES})
