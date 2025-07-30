cmake_minimum_required(VERSION 3.8)
project(moveit_min_jerk_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# MoveIt2 dependencies
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(moveit_configs_utils REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Additional dependencies
find_package(ompl REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the main library
add_library(${PROJECT_NAME} SHARED
  src/min_jerk_trajectory_generator.cpp
  src/moveit_min_jerk_planner.cpp
  src/collision_aware_planner.cpp
  src/path_planning_interface.cpp
)

# Set target dependencies
set(MOVEIT_DEPS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_kinematics
  moveit_planners_ompl
  moveit_visual_tools
)

set(ROS_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
  action_msgs
  moveit_msgs
  shape_msgs
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}
  ${MOVEIT_DEPS}
  ${ROS_DEPS}
  geometric_shapes
  ompl
  Eigen3
)

# Create the main executable
add_executable(moveit_min_jerk_planner_node
  src/moveit_min_jerk_planner_node.cpp
)

target_link_libraries(moveit_min_jerk_planner_node
  ${PROJECT_NAME}
)

ament_target_dependencies(moveit_min_jerk_planner_node
  ${MOVEIT_DEPS}
  ${ROS_DEPS}
  Eigen3
)

# Create service executable for Python interface
add_executable(moveit_planner_service_node
  src/moveit_planner_service_node.cpp
)

target_link_libraries(moveit_planner_service_node
  ${PROJECT_NAME}
)

ament_target_dependencies(moveit_planner_service_node
  ${MOVEIT_DEPS}
  ${ROS_DEPS}
  Eigen3
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  moveit_min_jerk_planner_node
  moveit_planner_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/moveit_min_jerk_python_interface.py
  scripts/collision_aware_demo.py
  scripts/path_planning_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install service definitions
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# Generate service interfaces
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanMinJerkTrajectory.srv"
  "srv/PlanCollisionFreeTrajectory.srv"
  "srv/PlanCartesianPath.srv"
  DEPENDENCIES
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_min_jerk_generator
    test/test_min_jerk_generator.cpp
  )
  
  target_link_libraries(test_min_jerk_generator
    ${PROJECT_NAME}
  )
  
  ament_target_dependencies(test_min_jerk_generator
    ${MOVEIT_DEPS}
    ${ROS_DEPS}
  )
endif()

# Export dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${MOVEIT_DEPS}
  ${ROS_DEPS}
  geometric_shapes
  ompl
  Eigen3
)

ament_package()