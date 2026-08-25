cmake_minimum_required(VERSION 3.10)
project(g1_control VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(g1_model REQUIRED)

add_library(g1_control SHARED src/joint_pd_controller.cpp)
target_include_directories(g1_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(g1_control
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools
  sensor_msgs g1_model)
pluginlib_export_plugin_description_file(controller_interface g1_control_plugins.xml)

install(TARGETS g1_control EXPORT export_g1_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_g1_control HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools sensor_msgs g1_model)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pd_law test/test_pd_law.cpp)
  target_include_directories(test_pd_law PRIVATE include)
  ament_target_dependencies(test_pd_law g1_model)
  target_compile_definitions(test_pd_law PRIVATE
    G1_URDF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../description/urdf")
endif()

ament_package()
