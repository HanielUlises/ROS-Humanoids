cmake_minimum_required(VERSION 3.10)
project(g1_model VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)

add_library(g1_model src/g1_model.cpp)
target_include_directories(g1_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(g1_model PUBLIC urdfdom::urdfdom_model orocos-kdl)

add_executable(g1_info examples/g1_info.cpp)
target_link_libraries(g1_info g1_model)
add_executable(g1_skeleton_dump examples/g1_skeleton_dump.cpp)
target_link_libraries(g1_skeleton_dump g1_model)
install(TARGETS g1_info g1_skeleton_dump DESTINATION lib/${PROJECT_NAME})

install(TARGETS g1_model EXPORT export_g1_model
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_g1_model HAS_LIBRARY_TARGET)
ament_export_dependencies(urdfdom urdfdom_headers orocos_kdl)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_g1_model test/test_g1_model.cpp)
  target_link_libraries(test_g1_model g1_model)
  # Tests read the URDFs straight from the description package sources, so they
  # exercise the files that are actually shipped.
  target_compile_definitions(test_g1_model PRIVATE
    G1_URDF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../description/urdf")
endif()

ament_package()
