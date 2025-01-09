cmake_minimum_required(VERSION 3.8)
project(odrive_v3_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(odrive_communication REQUIRED)

add_library(
  odrive_v3_hardware
  SHARED
  src/odrive_v3_hardware.cpp
)
target_include_directories(
  odrive_v3_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  odrive_v3_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  odrive_communication
)
# prevent pluginlib from using boost
target_compile_definitions(odrive_v3_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface odrive_v3_hardware.xml)

install(
  TARGETS
  odrive_v3_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  odrive_v3_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
