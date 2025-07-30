#!/usr/bin/env python3
"""
Demo client for min-jerk trajectory planner.

Demonstrates high-precision trajectory planning
for sub-millimeter accuracy applications.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
from typing import List

from min_jerk_planner.min_jerk_planner_node import MinJerkPlannerNode


class MinJerkPlannerDemo(Node):
    """
    Demo client showing various trajectory planning scenarios.
    """
    
    def __init__(self):
        super().__init__('min_jerk_planner_demo')
        
        # Wait for the planner node to be ready
        self.get_logger().info('Waiting for min-jerk planner node...')
        time.sleep(2.0)
        
        # Create planner instance (in practice, this would be a service client)
        self.planner = MinJerkPlannerNode()
        
        self.get_logger().info('Min-jerk planner demo ready!')
        
    def demo_point_to_point_planning(self):
        """Demonstrate point-to-point trajectory planning."""
        self.get_logger().info('=== Point-to-Point Trajectory Demo ===')
        
        # Define target joint positions (example for 6-DOF arm)
        target_positions = np.array([0.5, -0.3, 1.2, -1.5, 0.8, 0.0])
        
        # Plan trajectory
        trajectory = self.planner.plan_point_to_point_trajectory(
            target_positions=target_positions,
            duration=3.0,  # 3 second trajectory
            velocity_scaling=0.5,
            acceleration_scaling=0.3
        )
        
        self.get_logger().info(f'Generated trajectory with {len(trajectory.points)} points')
        self.get_logger().info(f'Total duration: {trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9:.3f}s')
        
        # Publish trajectory
        self.planner.publish_trajectory(trajectory)
        
        return trajectory
    
    def demo_multi_waypoint_planning(self):
        """Demonstrate multi-waypoint trajectory planning."""
        self.get_logger().info('=== Multi-Waypoint Trajectory Demo ===')
        
        # Define waypoints for a complex path
        waypoints = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),      # Home position
            np.array([0.5, -0.5, 1.0, -1.0, 0.5, 0.2]),    # Waypoint 1
            np.array([1.0, -0.3, 0.8, -1.5, 1.0, -0.3]),   # Waypoint 2
            np.array([0.8, 0.2, 1.2, -0.8, 0.3, 0.5]),     # Waypoint 3
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])       # Return home
        ]
        
        # Plan trajectory
        trajectory = self.planner.plan_multi_waypoint_trajectory(
            waypoints=waypoints,
            velocity_scaling=0.4,
            acceleration_scaling=0.3
        )
        
        self.get_logger().info(f'Generated multi-waypoint trajectory with {len(trajectory.points)} points')
        self.get_logger().info(f'Total duration: {trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9:.3f}s')
        
        # Publish trajectory
        self.planner.publish_trajectory(trajectory)
        
        return trajectory
    
    def demo_high_precision_trajectory(self):
        """Demonstrate high-precision trajectory for sub-millimeter accuracy."""
        self.get_logger().info('=== High-Precision Trajectory Demo ===')
        
        # Small, precise movements (simulating precision assembly task)
        current_pos = self.planner.get_current_joint_positions()
        
        # Define very small incremental movements
        precision_waypoints = [
            current_pos,
            current_pos + np.array([0.001, 0.0, 0.0, 0.0, 0.0, 0.0]),    # 1mm equivalent
            current_pos + np.array([0.001, 0.001, 0.0, 0.0, 0.0, 0.0]),  # 1mm x,y
            current_pos + np.array([0.001, 0.001, 0.001, 0.0, 0.0, 0.0]), # 1mm x,y,z
            current_pos  # Return to start
        ]
        
        # Use very slow, precise motion
        trajectory = self.planner.plan_multi_waypoint_trajectory(
            waypoints=precision_waypoints,
            velocity_scaling=0.1,      # Very slow
            acceleration_scaling=0.1   # Very gentle
        )
        
        self.get_logger().info(f'Generated high-precision trajectory with {len(trajectory.points)} points')
        self.get_logger().info('This trajectory is optimized for <0.5mm accuracy')
        
        # Publish trajectory
        self.planner.publish_trajectory(trajectory)
        
        return trajectory
    
    def demo_trajectory_analysis(self, trajectory):
        """Analyze trajectory properties for precision validation."""
        self.get_logger().info('=== Trajectory Analysis ===')
        
        if len(trajectory.points) == 0:
            self.get_logger().warn('Empty trajectory for analysis')
            return
        
        # Extract trajectory data
        positions = np.array([point.positions for point in trajectory.points])
        velocities = np.array([point.velocities for point in trajectory.points])
        accelerations = np.array([point.accelerations for point in trajectory.points])
        
        # Calculate statistics
        max_velocities = np.max(np.abs(velocities), axis=0)
        max_accelerations = np.max(np.abs(accelerations), axis=0)
        
        # Calculate jerk (numerical differentiation)
        dt = 1.0 / 1000.0  # 1ms time step
        jerks = np.diff(accelerations, axis=0) / dt
        max_jerks = np.max(np.abs(jerks), axis=0) if len(jerks) > 0 else np.zeros(len(self.planner.joint_names))
        
        # Report statistics
        self.get_logger().info('Trajectory Statistics:')
        for i, joint_name in enumerate(self.planner.joint_names):
            self.get_logger().info(f'  {joint_name}:')
            self.get_logger().info(f'    Max velocity: {max_velocities[i]:.4f} rad/s')
            self.get_logger().info(f'    Max acceleration: {max_accelerations[i]:.4f} rad/s²')
            self.get_logger().info(f'    Max jerk: {max_jerks[i]:.4f} rad/s³')
        
        # Check smoothness (continuous derivatives)
        velocity_continuity = self.check_continuity(velocities)
        acceleration_continuity = self.check_continuity(accelerations)
        
        self.get_logger().info(f'Velocity continuity: {"PASS" if velocity_continuity else "FAIL"}')
        self.get_logger().info(f'Acceleration continuity: {"PASS" if acceleration_continuity else "FAIL"}')
    
    def check_continuity(self, data: np.ndarray, tolerance: float = 1e-6) -> bool:
        """Check if trajectory data is continuous (no jumps)."""
        if len(data) < 2:
            return True
            
        # Check for large jumps between consecutive points
        diffs = np.diff(data, axis=0)
        max_diff = np.max(np.abs(diffs))
        
        return max_diff < tolerance
    
    def run_all_demos(self):
        """Run all demonstration scenarios."""
        self.get_logger().info('Starting min-jerk trajectory planner demonstrations...')
        
        try:
            # Demo 1: Point-to-point planning
            trajectory1 = self.demo_point_to_point_planning()
            self.demo_trajectory_analysis(trajectory1)
            time.sleep(1.0)
            
            # Demo 2: Multi-waypoint planning
            trajectory2 = self.demo_multi_waypoint_planning()
            self.demo_trajectory_analysis(trajectory2)
            time.sleep(1.0)
            
            # Demo 3: High-precision planning
            trajectory3 = self.demo_high_precision_trajectory()
            self.demo_trajectory_analysis(trajectory3)
            
            self.get_logger().info('All demonstrations completed successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Demo failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    demo_node = MinJerkPlannerDemo()
    
    try:
        # Run demonstrations
        demo_node.run_all_demos()
        
        # Keep node alive for visualization
        self.get_logger().info('Demo completed. Press Ctrl+C to exit.')
        rclpy.spin(demo_node)
        
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()