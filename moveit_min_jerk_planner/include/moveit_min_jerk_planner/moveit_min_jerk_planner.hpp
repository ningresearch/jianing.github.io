#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "moveit_min_jerk_planner/min_jerk_trajectory_generator.hpp"
#include "moveit_min_jerk_planner/collision_aware_planner.hpp"

namespace moveit_min_jerk_planner
{

/**
 * @brief High-precision trajectory planner combining MoveIt2 path planning with min-jerk execution
 * 
 * This class provides a comprehensive solution for robotic applications requiring:
 * - Sub-millimeter accuracy (<0.5mm deviation)
 * - Collision avoidance
 * - Path planning through complex environments
 * - Smooth min-jerk trajectory execution
 */
class MoveItMinJerkPlanner
{
public:
  /**
   * @brief Constructor
   * @param node ROS 2 node handle
   * @param group_name Planning group name (e.g., "manipulator", "arm")
   */
  explicit MoveItMinJerkPlanner(
    const rclcpp::Node::SharedPtr& node,
    const std::string& group_name);

  /**
   * @brief Destructor
   */
  ~MoveItMinJerkPlanner();

  /**
   * @brief Initialize the planner with robot model and planning scene
   * @return True if initialization successful
   */
  bool initialize();

  /**
   * @brief Plan collision-free trajectory to pose goal with min-jerk smoothing
   * @param target_pose Target end-effector pose
   * @param trajectory Output trajectory
   * @param enable_collision_avoidance Enable collision checking
   * @param apply_min_jerk_smoothing Apply min-jerk post-processing
   * @return True if planning successful
   */
  bool planToPose(
    const geometry_msgs::msg::PoseStamped& target_pose,
    trajectory_msgs::msg::JointTrajectory& trajectory,
    bool enable_collision_avoidance = true,
    bool apply_min_jerk_smoothing = true);

  /**
   * @brief Plan collision-free trajectory through multiple waypoints
   * @param waypoints Vector of Cartesian waypoints
   * @param trajectory Output trajectory
   * @param enable_collision_avoidance Enable collision checking
   * @param apply_min_jerk_smoothing Apply min-jerk post-processing
   * @return True if planning successful
   */
  bool planThroughWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
    trajectory_msgs::msg::JointTrajectory& trajectory,
    bool enable_collision_avoidance = true,
    bool apply_min_jerk_smoothing = true);

  /**
   * @brief Plan Cartesian path with collision avoidance
   * @param waypoints Cartesian waypoints as poses
   * @param trajectory Output trajectory
   * @param max_step Maximum step size between waypoints (m)
   * @param jump_threshold Jump threshold for joint space
   * @param avoid_collisions Enable collision avoidance
   * @return Fraction of path successfully planned (0.0 to 1.0)
   */
  double planCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    trajectory_msgs::msg::JointTrajectory& trajectory,
    double max_step = 0.01,
    double jump_threshold = 0.0,
    bool avoid_collisions = true);

  /**
   * @brief Add collision object to planning scene
   * @param object_id Unique identifier for the object
   * @param pose Object pose
   * @param shape_type Shape type ("box", "sphere", "cylinder")
   * @param dimensions Shape dimensions
   * @return True if object added successfully
   */
  bool addCollisionObject(
    const std::string& object_id,
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& shape_type,
    const std::vector<double>& dimensions);

  /**
   * @brief Remove collision object from planning scene
   * @param object_id Object identifier to remove
   * @return True if object removed successfully
   */
  bool removeCollisionObject(const std::string& object_id);

  /**
   * @brief Apply min-jerk smoothing to existing trajectory
   * @param input_trajectory Input trajectory to smooth
   * @param output_trajectory Smoothed output trajectory
   * @param trajectory_duration Desired duration (auto-calculate if 0)
   * @param enable_jerk_limiting Enable jerk limiting
   * @return True if smoothing successful
   */
  bool applyMinJerkSmoothing(
    const trajectory_msgs::msg::JointTrajectory& input_trajectory,
    trajectory_msgs::msg::JointTrajectory& output_trajectory,
    double trajectory_duration = 0.0,
    bool enable_jerk_limiting = true);

  /**
   * @brief Validate trajectory for collision and joint limits
   * @param trajectory Trajectory to validate
   * @param collision_free Output: true if collision-free
   * @param within_limits Output: true if within joint limits
   * @return True if validation completed
   */
  bool validateTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    bool& collision_free,
    bool& within_limits);

  /**
   * @brief Set planning parameters
   * @param planner_id OMPL planner to use
   * @param planning_time Maximum planning time (s)
   * @param planning_attempts Maximum planning attempts
   * @param velocity_scaling Velocity scaling factor (0-1)
   * @param acceleration_scaling Acceleration scaling factor (0-1)
   */
  void setPlanningParameters(
    const std::string& planner_id = "RRTConnect",
    double planning_time = 5.0,
    int planning_attempts = 5,
    double velocity_scaling = 0.5,
    double acceleration_scaling = 0.5);

  /**
   * @brief Set min-jerk parameters for high precision
   * @param control_frequency Control frequency (Hz)
   * @param jerk_scaling Jerk scaling factor (0-1)
   * @param enable_jerk_limiting Enable jerk limiting
   */
  void setMinJerkParameters(
    double control_frequency = 1000.0,
    double jerk_scaling = 0.5,
    bool enable_jerk_limiting = true);

  /**
   * @brief Get current robot state
   * @return Current joint state
   */
  sensor_msgs::msg::JointState getCurrentJointState();

  /**
   * @brief Get planning group name
   * @return Planning group name
   */
  const std::string& getGroupName() const { return group_name_; }

  /**
   * @brief Check if planner is initialized
   * @return True if initialized
   */
  bool isInitialized() const { return initialized_; }

private:
  // ROS 2 node
  rclcpp::Node::SharedPtr node_;
  
  // Planning group name
  std::string group_name_;
  
  // Initialization flag
  bool initialized_;

  // MoveIt2 interfaces
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  
  // Robot model and state
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  
  // Min-jerk trajectory generator
  std::unique_ptr<MinJerkTrajectoryGenerator> min_jerk_generator_;
  
  // Collision-aware planner
  std::unique_ptr<CollisionAwarePlanner> collision_planner_;
  
  // Visualization tools
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  
  // Planning parameters
  std::string planner_id_;
  double planning_time_;
  int planning_attempts_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double jerk_scaling_;
  bool enable_jerk_limiting_;
  double control_frequency_;

  // Internal helper methods
  bool setupMoveGroup();
  bool setupPlanningScene();
  bool setupRobotModel();
  bool setupVisualization();
  
  trajectory_msgs::msg::JointTrajectory convertRobotTrajectoryToJointTrajectory(
    const robot_trajectory::RobotTrajectory& robot_trajectory);
    
  bool applyTimeParameterization(
    robot_trajectory::RobotTrajectory& trajectory);
};

} // namespace moveit_min_jerk_planner