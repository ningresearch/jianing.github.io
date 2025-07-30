#!/usr/bin/env python3
"""
Min-jerk trajectory planner ROS 2 node.

Provides trajectory planning services for high-precision robotic control
with sub-millimeter accuracy requirements.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from typing import List, Dict, Any
import yaml
import os

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from min_jerk_planner.min_jerk_trajectory import MinJerkTrajectoryGenerator


class MinJerkPlannerNode(Node):
    """
    ROS 2 node for high-precision min-jerk trajectory planning.
    
    Provides services for:
    - Point-to-point trajectory planning
    - Multi-waypoint trajectory planning  
    - Cartesian path planning (with IK solver)
    - Real-time trajectory modification
    """
    
    def __init__(self):
        super().__init__('min_jerk_planner_node')
        
        # Initialize callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('control_frequency', 1000.0)  # 1kHz for high precision
        self.declare_parameter('joint_limits_file', '')
        self.declare_parameter('default_velocity_scaling', 0.5)
        self.declare_parameter('default_acceleration_scaling', 0.5)
        self.declare_parameter('trajectory_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')
        
        # Get parameters
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.joint_limits_file = self.get_parameter('joint_limits_file').get_parameter_value().string_value
        self.default_velocity_scaling = self.get_parameter('default_velocity_scaling').get_parameter_value().double_value
        self.default_acceleration_scaling = self.get_parameter('default_acceleration_scaling').get_parameter_value().double_value
        trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        
        # Initialize trajectory generator
        dt = 1.0 / self.control_frequency
        self.trajectory_generator = MinJerkTrajectoryGenerator(dt=dt)
        
        # Load joint limits
        self.load_joint_limits()
        
        # Initialize current joint state
        self.current_joint_state = None
        self.joint_state_lock = False
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribers  
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action client for trajectory execution
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Service servers (to be implemented)
        # self.plan_point_to_point_srv = self.create_service(...)
        # self.plan_multi_waypoint_srv = self.create_service(...)
        
        self.get_logger().info(f'Min-jerk planner node initialized')
        self.get_logger().info(f'Joint names: {self.joint_names}')
        self.get_logger().info(f'Control frequency: {self.control_frequency} Hz')
        
    def load_joint_limits(self):
        """Load joint limits from YAML file."""
        if not self.joint_limits_file or not os.path.exists(self.joint_limits_file):
            self.get_logger().warn(f'Joint limits file not found: {self.joint_limits_file}')
            self.get_logger().warn('Using default limits')
            
            # Set conservative default limits
            default_limits = {
                'position_min': -3.14,
                'position_max': 3.14,
                'velocity_max': 1.0,
                'acceleration_max': 2.0,
                'jerk_max': 10.0
            }
            
            joint_limits = {}
            for joint_name in self.joint_names:
                joint_limits[joint_name] = default_limits.copy()
                
            self.trajectory_generator.set_joint_limits(joint_limits)
            return
        
        try:
            with open(self.joint_limits_file, 'r') as f:
                limits_data = yaml.safe_load(f)
            
            joint_limits = {}
            
            # Parse joint limits from YAML
            for joint_name in self.joint_names:
                if joint_name in limits_data:
                    joint_data = limits_data[joint_name]
                    joint_limits[joint_name] = {}
                    
                    # Extract limits
                    if 'min_position' in joint_data:
                        joint_limits[joint_name]['position_min'] = joint_data['min_position']
                    if 'max_position' in joint_data:
                        joint_limits[joint_name]['position_max'] = joint_data['max_position']
                    if 'max_velocity' in joint_data:
                        joint_limits[joint_name]['velocity_max'] = joint_data['max_velocity']
                    if 'max_acceleration' in joint_data:
                        joint_limits[joint_name]['acceleration_max'] = joint_data['max_acceleration']
                    if 'max_jerk' in joint_data:
                        joint_limits[joint_name]['jerk_max'] = joint_data['max_jerk']
                else:
                    self.get_logger().warn(f'No limits found for joint: {joint_name}')
            
            self.trajectory_generator.set_joint_limits(joint_limits)
            self.get_logger().info(f'Loaded joint limits for {len(joint_limits)} joints')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load joint limits: {str(e)}')
            
    def joint_state_callback(self, msg: JointState):
        """Update current joint state."""
        if not self.joint_state_lock:
            self.current_joint_state = msg
    
    def get_current_joint_positions(self) -> np.ndarray:
        """Get current joint positions in the order specified by joint_names."""
        if self.current_joint_state is None:
            self.get_logger().warn('No joint state received yet')
            return np.zeros(len(self.joint_names))
        
        positions = np.zeros(len(self.joint_names))
        
        for i, joint_name in enumerate(self.joint_names):
            try:
                joint_idx = self.current_joint_state.name.index(joint_name)
                positions[i] = self.current_joint_state.position[joint_idx]
            except ValueError:
                self.get_logger().warn(f'Joint {joint_name} not found in joint state')
                
        return positions
    
    def plan_point_to_point_trajectory(
        self,
        target_positions: np.ndarray,
        duration: float = None,
        velocity_scaling: float = None,
        acceleration_scaling: float = None
    ) -> JointTrajectory:
        """
        Plan point-to-point min-jerk trajectory.
        
        Args:
            target_positions: Target joint positions [rad]
            duration: Trajectory duration [s] (auto-calculated if None)
            velocity_scaling: Velocity scaling factor (0-1)
            acceleration_scaling: Acceleration scaling factor (0-1)
            
        Returns:
            JointTrajectory message
        """
        if velocity_scaling is None:
            velocity_scaling = self.default_velocity_scaling
        if acceleration_scaling is None:
            acceleration_scaling = self.default_acceleration_scaling
            
        # Get current joint positions
        start_positions = self.get_current_joint_positions()
        
        # Auto-calculate duration if not provided
        if duration is None:
            max_displacement = np.max(np.abs(target_positions - start_positions))
            # Heuristic: T = 2 * sqrt(displacement / max_acceleration)
            duration = max(2.0 * np.sqrt(max_displacement / (acceleration_scaling * 2.0)), 0.5)
        
        self.get_logger().info(f'Planning trajectory with duration: {duration:.3f}s')
        
        # Generate min-jerk trajectory
        time_array, positions, velocities, accelerations, jerks = \
            self.trajectory_generator.generate_min_jerk_trajectory(
                start_positions,
                target_positions,
                duration
            )
        
        # Validate trajectory
        is_valid, violations = self.trajectory_generator.validate_trajectory(
            positions, velocities, accelerations, jerks, self.joint_names
        )
        
        if not is_valid:
            self.get_logger().warn(f'Trajectory validation failed: {violations}')
        
        # Convert to JointTrajectory message
        trajectory = self.create_joint_trajectory_msg(
            time_array, positions, velocities, accelerations
        )
        
        return trajectory
    
    def plan_multi_waypoint_trajectory(
        self,
        waypoints: List[np.ndarray],
        segment_durations: List[float] = None,
        velocity_scaling: float = None,
        acceleration_scaling: float = None
    ) -> JointTrajectory:
        """
        Plan multi-waypoint min-jerk trajectory.
        
        Args:
            waypoints: List of joint configurations [rad]
            segment_durations: Duration for each segment [s]
            velocity_scaling: Velocity scaling factor (0-1)
            acceleration_scaling: Acceleration scaling factor (0-1)
            
        Returns:
            JointTrajectory message
        """
        if velocity_scaling is None:
            velocity_scaling = self.default_velocity_scaling
        if acceleration_scaling is None:
            acceleration_scaling = self.default_acceleration_scaling
        
        # Add current position as first waypoint if not already included
        current_pos = self.get_current_joint_positions()
        if len(waypoints) == 0 or not np.allclose(waypoints[0], current_pos, atol=0.01):
            waypoints.insert(0, current_pos)
        
        self.get_logger().info(f'Planning trajectory through {len(waypoints)} waypoints')
        
        # Generate multi-waypoint trajectory
        time_array, positions, velocities, accelerations, jerks = \
            self.trajectory_generator.generate_multi_waypoint_trajectory(
                waypoints,
                segment_durations,
                velocity_scaling,
                acceleration_scaling
            )
        
        # Validate trajectory
        is_valid, violations = self.trajectory_generator.validate_trajectory(
            positions, velocities, accelerations, jerks, self.joint_names
        )
        
        if not is_valid:
            self.get_logger().warn(f'Trajectory validation failed: {violations}')
        
        # Convert to JointTrajectory message
        trajectory = self.create_joint_trajectory_msg(
            time_array, positions, velocities, accelerations
        )
        
        return trajectory
    
    def create_joint_trajectory_msg(
        self,
        time_array: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray
    ) -> JointTrajectory:
        """Convert trajectory arrays to JointTrajectory message."""
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = 'base_link'
        trajectory.joint_names = self.joint_names
        
        for i in range(len(time_array)):
            point = JointTrajectoryPoint()
            point.positions = positions[i, :].tolist()
            point.velocities = velocities[i, :].tolist()
            point.accelerations = accelerations[i, :].tolist()
            point.time_from_start.sec = int(time_array[i])
            point.time_from_start.nanosec = int((time_array[i] - int(time_array[i])) * 1e9)
            
            trajectory.points.append(point)
        
        return trajectory
    
    def publish_trajectory(self, trajectory: JointTrajectory):
        """Publish trajectory to joint trajectory controller."""
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Published trajectory with {len(trajectory.points)} points')
    
    def execute_trajectory_async(self, trajectory: JointTrajectory):
        """Execute trajectory using action client."""
        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Trajectory action server not available')
            return None
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future = self.trajectory_action_client.send_goal_async(goal_msg)
        return future


def main(args=None):
    rclpy.init(args=args)
    
    node = MinJerkPlannerNode()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()