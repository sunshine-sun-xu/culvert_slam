cmake_minimum_required(VERSION 3.8)
project(culvert_submap_manager)

find_package(ament_cmake REQUIRED)
find_package(culvert_mapping_interfaces REQUIRED)
find_package(fastdem REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(submap_rebuild
  src/submap_rebuild.cpp
)

target_link_libraries(submap_rebuild
  fastdem::fastdem
  yaml-cpp
)

install(PROGRAMS
  scripts/submap_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  submap_rebuild
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
