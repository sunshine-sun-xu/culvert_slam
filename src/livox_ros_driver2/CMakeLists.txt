# judge which cmake codes to use
if(ROS_EDITION STREQUAL "ROS1")

  cmake_minimum_required(VERSION 3.0)

  include(cmake/version.cmake)
  project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
  message(STATUS "livox_ros_driver2 version: ${LIVOX_ROS_DRIVER2_VERSION}")

  add_definitions(-DBUILDING_ROS1)

  find_package(Boost 1.54 REQUIRED COMPONENTS
    system
    thread
    chrono
  )

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    rosbag
    pcl_ros
  )

  find_package(PCL REQUIRED)

  add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
  )

  generate_messages(DEPENDENCIES
    std_msgs
  )

  find_package(PkgConfig)
  pkg_check_modules(APR apr-1)
  if(APR_FOUND)
    message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES})
  endif()

  catkin_package(CATKIN_DEPENDS
    roscpp rospy std_msgs message_runtime
    pcl_ros
  )

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
  endif()

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_static.a /usr/local/lib)

  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_executable(${PROJECT_NAME}_node
    ""
  )

  target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
  )

  add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

  target_sources(${PROJECT_NAME}_node
    PRIVATE
    src/driver_node.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/lddc.cpp
    src/livox_ros_driver2.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp
    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp
    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )

  target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    3rdparty
    src
  )

  target_link_libraries(${PROJECT_NAME}_node
    ${LIVOX_LIDAR_SDK_LIBRARY}
    ${Boost_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch_ROS1/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch_ROS1
  )

else()

  cmake_minimum_required(VERSION 3.14)
  project(livox_ros_driver2)

  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
  endif()

  include(cmake/version.cmake)
  project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
  message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

  add_definitions(-DBUILDING_ROS2)

  if(NOT DEFINED DISTRO_ROS OR DISTRO_ROS STREQUAL "")
    set(DISTRO_ROS $ENV{ROS_DISTRO})
  endif()

  message(STATUS "DISTRO_ROS: ${DISTRO_ROS}")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  find_package(PCL REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  find_package(PkgConfig)
  pkg_check_modules(APR apr-1)
  if(APR_FOUND)
    message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES})
  endif()

  set(LIVOX_INTERFACES livox_interfaces2)
  rosidl_generate_interfaces(${LIVOX_INTERFACES}
    "msg/CustomPoint.msg"
    "msg/CustomMsg.msg"
    DEPENDENCIES builtin_interfaces std_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )

  find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)

  find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
    NAMES "livox_lidar_api.h" "livox_lidar_def.h"
    REQUIRED
  )

  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/livox_ros_driver2.cpp
    src/lddc.cpp
    src/driver_node.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp
    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp
    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )

  target_include_directories(${PROJECT_NAME} PRIVATE ${livox_sdk_INCLUDE_DIRS})

  if(DISTRO_ROS STREQUAL "humble" OR DISTRO_ROS STREQUAL "jazzy")
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${LIVOX_INTERFACES}
      "rosidl_typesupport_cpp"
    )

    set(LIVOX_INTERFACE_TARGET ${cpp_typesupport_target})

    set(LIVOX_INTERFACES_INCLUDE_DIRECTORIES
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    )

    target_link_libraries(${PROJECT_NAME}
      ${LIVOX_INTERFACE_TARGET}
    )
  else()
    set(LIVOX_INTERFACE_TARGET "${LIVOX_INTERFACES}__rosidl_typesupport_cpp")
    add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})
    get_target_property(LIVOX_INTERFACES_INCLUDE_DIRECTORIES
      ${LIVOX_INTERFACE_TARGET}
      INTERFACE_INCLUDE_DIRECTORIES
    )

    if(NOT LIVOX_INTERFACES_INCLUDE_DIRECTORIES)
      set(LIVOX_INTERFACES_INCLUDE_DIRECTORIES
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
      )
    endif()
  endif()

  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
    ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}
    3rdparty
    src
  )

  target_link_libraries(${PROJECT_NAME}
    ${LIVOX_LIDAR_SDK_LIBRARY}
    ${PPT_LIBRARY}
    ${Boost_LIBRARY}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "livox_ros::DriverNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_auto_package(INSTALL_TO_SHARE
    config
    launch_ROS2
  )

endif()