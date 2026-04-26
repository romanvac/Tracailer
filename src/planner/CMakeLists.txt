cmake_minimum_required(VERSION 3.12)
project(planner)

# -----------------------------------------------------------------------------
# Compile-time configuration
# -----------------------------------------------------------------------------
# Number of trailers baked into the planner (inherited from ROS 1 build).
# This propagates into the planner library and all nodes consuming it.
# Changing this requires a clean rebuild of every node linking against planner.
if(NOT DEFINED TRAILER_NUM)
  set(TRAILER_NUM 3 CACHE STRING "Number of trailers")
endif()
add_compile_definitions(TRAILER_NUM=${TRAILER_NUM})

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# -----------------------------------------------------------------------------
# Package dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface-level deps
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# -----------------------------------------------------------------------------
# Message generation (rosidl)
# -----------------------------------------------------------------------------
set(msg_files
  "msg/TrailerState.msg"
  "msg/PolyTraj.msg"
  "msg/ArcTrailerTraj.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    nav_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------------------------------------------------------
# C++ nodes
# -----------------------------------------------------------------------------
# The planner library and its three nodes (planner_node, simulator_node,
# mpc_node) have been ported to the ROS 2 Jazzy rclcpp API. Build can be
# disabled from the colcon command line with -DBUILD_PLANNER_NODES=OFF if
# only the message package is desired.
option(BUILD_PLANNER_NODES "Build planner library and nodes" ON)

if(BUILD_PLANNER_NODES)
  find_package(rclcpp REQUIRED)
  find_package(ackermann_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED)
  find_package(ompl REQUIRED)
  find_package(CASADI REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # --- planner library ---
  add_library(${PROJECT_NAME}_core
    src/grid_map.cpp
    src/hybrid_astar.cpp
    src/arc_opt.cpp
    src/planner.cpp
  )
  target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(${PROJECT_NAME}_core
    rclcpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    pcl_conversions
  )
  target_link_libraries(${PROJECT_NAME}_core ${OMPL_LIBRARIES} ${PCL_LIBRARIES})
  # Link generated interfaces into the same-package library.
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(${PROJECT_NAME}_core ${cpp_typesupport_target})

  # --- planner_node ---
  add_executable(planner_node src/planner_node.cpp)
  target_link_libraries(planner_node ${PROJECT_NAME}_core)
  ament_target_dependencies(planner_node rclcpp)

  # --- simulator_node ---
  add_executable(simulator_node src/simulator.cpp)
  target_include_directories(simulator_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
  )
  ament_target_dependencies(simulator_node
    rclcpp std_msgs nav_msgs geometry_msgs ackermann_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs
  )
  target_link_libraries(simulator_node ${cpp_typesupport_target})

  # --- mpc_node ---
  add_executable(mpc_node src/mpc.cpp)
  target_include_directories(mpc_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${CASADI_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
  )
  ament_target_dependencies(mpc_node
    rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs ackermann_msgs visualization_msgs pcl_conversions
  )
  target_link_libraries(mpc_node ${CASADI_LIBRARIES} ${PCL_LIBRARIES} ${cpp_typesupport_target})

  install(TARGETS ${PROJECT_NAME}_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS planner_node simulator_node mpc_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/ DESTINATION include)
endif()

# -----------------------------------------------------------------------------
# Shared resources
# -----------------------------------------------------------------------------
install(DIRECTORY launch params meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# -----------------------------------------------------------------------------
# Testing / linting
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
