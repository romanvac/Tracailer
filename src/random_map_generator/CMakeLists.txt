cmake_minimum_required(VERSION 3.12)
project(random_map_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------------
# random_map node
# -----------------------------------------------------------------------------
# random_map_node is ported to the ROS 2 Jazzy rclcpp API. Build can be disabled
# from the colcon command line with -DBUILD_RANDOM_MAP_NODE=OFF if only the
# package resources (meshes, params, rviz) are required.
option(BUILD_RANDOM_MAP_NODE "Build the random_map executable" ON)

if(BUILD_RANDOM_MAP_NODE)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED)

  add_executable(random_map src/random_map_node.cpp)
  target_include_directories(random_map PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(random_map
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    pcl_conversions
    ament_index_cpp
  )
  target_link_libraries(random_map ${PCL_LIBRARIES})

  install(TARGETS random_map DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch params meshes rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
