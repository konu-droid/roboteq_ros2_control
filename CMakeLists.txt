cmake_minimum_required(VERSION 3.16)
project(roboteq_hardware_interface LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  roboteq_hardware_interface
  SHARED
  src/diffbot_system.cpp
)
target_compile_features(roboteq_hardware_interface PUBLIC cxx_std_17)
target_include_directories(roboteq_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/roboteq_hardware_interface>
)
ament_target_dependencies(
  roboteq_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(roboteq_hardware_interface PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROBOTEQ_HARDWARE_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface roboteq_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/roboteq_hardware_interface
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/roboteq_hardware_interface
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/roboteq_hardware_interface
)
install(TARGETS roboteq_hardware_interface
  EXPORT export_roboteq_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_roboteq_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
