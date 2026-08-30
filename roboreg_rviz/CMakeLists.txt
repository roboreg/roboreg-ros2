cmake_minimum_required(VERSION 3.22)
project(roboreg_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(roboreg_idl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON) # for Qt MOC

set(roboreg_rviz_dependencies
  rclcpp::rclcpp
  rviz_common::rviz_common
  roboreg_idl::roboreg_idl
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  pluginlib::pluginlib
)

add_library(
  roboreg_rviz
  SHARED
  src/collect_data_widget.cpp
  src/display.cpp
  src/formatting.cpp
  src/io_widget.cpp
  src/register_widget.cpp
  include/roboreg_rviz/display.hpp # for Qt MOC
)

target_include_directories(
  roboreg_rviz
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  roboreg_rviz
  ${roboreg_rviz_dependencies}
)

pluginlib_export_plugin_description_file(rviz_common roboreg_rviz.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${roboreg_rviz_dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES icon/RoboregClient.svg
  DESTINATION share/${PROJECT_NAME}/icons/classes
)

install(
  TARGETS roboreg_rviz
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
