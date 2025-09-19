cmake_minimum_required(VERSION 3.10)
project(urdf_to_screw_list)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
  include
)

add_executable(urdf_to_screw_list
  src/urdf_to_screw_list.cpp
)

ament_target_dependencies(urdf_to_screw_list
  rclcpp
  kdl_parser
  orocos_kdl
  urdf
  urdfdom
  urdfdom_headers
)

target_link_libraries(urdf_to_screw_list
  Eigen3::Eigen
)

install(TARGETS urdf_to_screw_list
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
