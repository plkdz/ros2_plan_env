cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env
  src/grid_map.cpp
  src/voxel_grid_geometry.cpp
  src/binary_occupancy_layer.cpp
  src/binary_inflation_layer.cpp
)

ament_target_dependencies(plan_env
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  Eigen3
)
target_link_libraries(plan_env ${PCL_LIBRARIES})

add_executable(grid_map_standalone_node src/grid_map_standalone_node.cpp)
ament_target_dependencies(grid_map_standalone_node rclcpp)
target_link_libraries(grid_map_standalone_node plan_env)

install(
  TARGETS plan_env grid_map_standalone_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  Eigen3
)

ament_package()
