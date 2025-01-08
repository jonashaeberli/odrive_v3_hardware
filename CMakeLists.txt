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

if(BUILD_TESTING)

  ament_add_gmock(test_odrive_v3_hardware test/test_odrive_v3_hardware.cpp)
  target_include_directories(test_odrive_v3_hardware PRIVATE include)
  ament_target_dependencies(
    test_odrive_v3_hardware
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

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
