#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace moveit_min_jerk_planner
{

/**
 * @brief High-precision minimum jerk trajectory generator
 * 
 * Generates smooth trajectories with continuous position, velocity, 
 * acceleration, and jerk profiles for robotic applications requiring
 * high accuracy (<0.5mm deviation).
 */
class MinJerkTrajectoryGenerator
{
public:
  /**
   * @brief Constructor
   * @param control_frequency Control frequency in Hz (default: 1000 Hz)
   */
  explicit MinJerkTrajectoryGenerator(double control_frequency = 1000.0);

  /**
   * @brief Destructor
   */
  ~MinJerkTrajectoryGenerator() = default;

  /**
   * @brief Generate minimum jerk trajectory between two joint configurations
   * @param start_positions Starting joint positions [rad]
   * @param end_positions Target joint positions [rad]  
   * @param duration Trajectory duration [s]
   * @param start_velocities Starting velocities [rad/s] (optional)
   * @param end_velocities Target velocities [rad/s] (optional)
   * @param start_accelerations Starting accelerations [rad/s²] (optional)
   * @param end_accelerations Target accelerations [rad/s²] (optional)
   * @return Generated joint trajectory
   */
  trajectory_msgs::msg::JointTrajectory generateMinJerkTrajectory(
    const std::vector<double>& start_positions,
    const std::vector<double>& end_positions,
    double duration,
    const std::vector<double>& start_velocities = {},
    const std::vector<double>& end_velocities = {},
    const std::vector<double>& start_accelerations = {},
    const std::vector<double>& end_accelerations = {});

  /**
   * @brief Generate trajectory through multiple waypoints
   * @param waypoints Vector of joint configurations [rad]
   * @param segment_durations Duration for each segment [s] (auto-calculated if empty)
   * @param joint_names Names of the joints
   * @param velocity_scaling Global velocity scaling factor (0-1)
   * @param acceleration_scaling Global acceleration scaling factor (0-1)
   * @return Generated joint trajectory
   */
  trajectory_msgs::msg::JointTrajectory generateMultiWaypointTrajectory(
    const std::vector<std::vector<double>>& waypoints,
    const std::vector<double>& segment_durations,
    const std::vector<std::string>& joint_names,
    double velocity_scaling = 1.0,
    double acceleration_scaling = 1.0);

  /**
   * @brief Apply min-jerk smoothing to existing trajectory
   * @param input_trajectory Input trajectory to smooth
   * @param trajectory_duration Desired duration (auto-calculate if 0)
   * @param enable_jerk_limiting Enable jerk limiting
   * @return Smoothed trajectory
   */
  trajectory_msgs::msg::JointTrajectory applyMinJerkSmoothing(
    const trajectory_msgs::msg::JointTrajectory& input_trajectory,
    double trajectory_duration = 0.0,
    bool enable_jerk_limiting = true);

  /**
   * @brief Set joint limits for trajectory validation
   * @param joint_names Names of the joints
   * @param position_limits Position limits [min, max] for each joint [rad]
   * @param velocity_limits Velocity limits for each joint [rad/s]
   * @param acceleration_limits Acceleration limits for each joint [rad/s²]
   * @param jerk_limits Jerk limits for each joint [rad/s³]
   */
  void setJointLimits(
    const std::vector<std::string>& joint_names,
    const std::vector<std::pair<double, double>>& position_limits,
    const std::vector<double>& velocity_limits,
    const std::vector<double>& acceleration_limits,
    const std::vector<double>& jerk_limits);

  /**
   * @brief Validate trajectory against joint limits
   * @param trajectory Trajectory to validate
   * @param violations Output vector of limit violations
   * @return True if trajectory is valid
   */
  bool validateTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::vector<std::string>& violations);

  /**
   * @brief Analyze trajectory quality metrics
   * @param trajectory Trajectory to analyze
   * @param max_velocities Output: maximum velocities for each joint
   * @param max_accelerations Output: maximum accelerations for each joint
   * @param max_jerks Output: maximum jerks for each joint
   * @param smoothness_score Output: overall smoothness score (0-1)
   * @return True if analysis completed
   */
  bool analyzeTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::vector<double>& max_velocities,
    std::vector<double>& max_accelerations,
    std::vector<double>& max_jerks,
    double& smoothness_score);

  /**
   * @brief Set control frequency
   * @param frequency Control frequency in Hz
   */
  void setControlFrequency(double frequency);

  /**
   * @brief Get control frequency
   * @return Control frequency in Hz
   */
  double getControlFrequency() const { return control_frequency_; }

private:
  // Control parameters
  double control_frequency_;
  double dt_;  // Time step

  // Joint limits
  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> position_limits_;
  std::vector<double> velocity_limits_;
  std::vector<double> acceleration_limits_;
  std::vector<double> jerk_limits_;

  /**
   * @brief Generate single-joint min-jerk trajectory using quintic polynomial
   * @param q0 Start position [rad]
   * @param q1 End position [rad]
   * @param v0 Start velocity [rad/s]
   * @param v1 End velocity [rad/s]
   * @param a0 Start acceleration [rad/s²]
   * @param a1 End acceleration [rad/s²]
   * @param T Duration [s]
   * @param positions Output: position trajectory
   * @param velocities Output: velocity trajectory
   * @param accelerations Output: acceleration trajectory
   * @param jerks Output: jerk trajectory
   */
  void generateSingleJointMinJerk(
    double q0, double q1,
    double v0, double v1,
    double a0, double a1,
    double T,
    std::vector<double>& positions,
    std::vector<double>& velocities,
    std::vector<double>& accelerations,
    std::vector<double>& jerks);

  /**
   * @brief Calculate optimal segment durations based on joint limits
   * @param waypoints Vector of joint configurations
   * @param velocity_scaling Velocity scaling factor
   * @param acceleration_scaling Acceleration scaling factor
   * @return Vector of segment durations
   */
  std::vector<double> calculateSegmentDurations(
    const std::vector<std::vector<double>>& waypoints,
    double velocity_scaling,
    double acceleration_scaling);

  /**
   * @brief Calculate via-point velocity for smooth trajectory continuity
   * @param prev_waypoint Previous waypoint
   * @param current_waypoint Current waypoint
   * @param next_waypoint Next waypoint
   * @param prev_duration Previous segment duration
   * @param next_duration Next segment duration
   * @return Via-point velocity
   */
  std::vector<double> calculateViaVelocity(
    const std::vector<double>& prev_waypoint,
    const std::vector<double>& current_waypoint,
    const std::vector<double>& next_waypoint,
    double prev_duration,
    double next_duration);

  /**
   * @brief Check trajectory continuity (no jumps in derivatives)
   * @param trajectory Trajectory to check
   * @param tolerance Continuity tolerance
   * @return True if trajectory is continuous
   */
  bool checkContinuity(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    double tolerance = 1e-6);

  /**
   * @brief Convert time to trajectory point index
   * @param time Time value [s]
   * @return Trajectory point index
   */
  size_t timeToIndex(double time) const;

  /**
   * @brief Convert trajectory point index to time
   * @param index Trajectory point index
   * @return Time value [s]
   */
  double indexToTime(size_t index) const;
};

} // namespace moveit_min_jerk_planner