cmake_minimum_required(VERSION 3.8)
project(ros2_roboreg_rviz)

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
find_package(ros2_roboreg_idl REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)

set(ros2_roboreg_rviz_dependencies
  rclcpp
  rviz_common
  ros2_roboreg_idl
  std_srvs
  pluginlib
)

add_library(
  ros2_roboreg_rviz
  SHARED
  src/collect_data_widget.cpp
  src/display.cpp
  src/register_widget.cpp
  src/save_data_widget.cpp
)

target_include_directories(
  ros2_roboreg_rviz
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ros2_roboreg_rviz
  ${ros2_roboreg_rviz_dependencies}
)


pluginlib_export_plugin_description_file(rviz_common ros2_roboreg_rviz.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ros2_roboreg_rviz_dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES icon/Roboreg.svg
  DESTINATION share/${PROJECT_NAME}/icons/classes
)

install(
  TARGETS ros2_roboreg_rviz
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
