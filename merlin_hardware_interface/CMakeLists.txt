cmake_minimum_required(VERSION 3.16.3)
project(merlin_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

# Define targets
add_library(
   ${PROJECT_NAME}
   SHARED
   src/target.cpp
)

# Add include directories for target
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# Link against dependencies
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager_msgs
  controller_interface
  pluginlib
  rclcpp
  realtime_tools
)

pluginlib_export_plugin_description_file(hardware_interface merlin_hardware_interface.xml)

# Export for downstream usage, see https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html
ament_export_targets(
  ${PROJECT_NAME}_export HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  hardware_interface
  controller_manager_msgs
  controller_interface
  pluginlib
  rclcpp
  realtime_tools
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_package()
