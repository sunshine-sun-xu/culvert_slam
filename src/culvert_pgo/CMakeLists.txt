cmake_minimum_required(VERSION 3.8)
project(culvert_pgo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(culvert_mapping_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(culvert_pgo_node
  src/culvert_pgo_node.cpp
  src/scan_context_manager.cpp
)

ament_target_dependencies(culvert_pgo_node
  GTSAM
  geometry_msgs
  culvert_mapping_interfaces
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

target_link_libraries(culvert_pgo_node
  ${PCL_LIBRARIES}
  gtsam
  yaml-cpp
)

install(TARGETS
  culvert_pgo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
