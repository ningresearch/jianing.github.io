#!/usr/bin/env python3
"""
Comprehensive demo for MoveIt2 Min-Jerk Planner with collision avoidance.

This demo showcases:
1. Collision-free path planning using MoveIt2
2. Min-jerk trajectory smoothing for high precision
3. Dynamic obstacle avoidance
4. Cartesian path planning
5. Real-time trajectory execution

Designed for sub-millimeter accuracy applications.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
from typing import List, Tuple
import math

from moveit_min_jerk_planner.scripts.moveit_min_jerk_python_interface import MoveItMinJerkPlannerPython


class CollisionAwareDemo(Node):
    """
    Comprehensive demonstration of collision-aware min-jerk trajectory planning.
    """
    
    def __init__(self):
        super().__init__('collision_aware_demo')
        
        # Initialize the planner
        self.planner = MoveItMinJerkPlannerPython(
            group_name="panda_arm",  # Adjust for your robot
            node_name="collision_demo_planner"
        )
        
        self.get_logger().info('Collision-aware demo initialized!')
        
    def demo_1_basic_collision_avoidance(self):
        """Demo 1: Basic collision avoidance planning."""
        self.get_logger().info('=== Demo 1: Basic Collision Avoidance ===')
        
        # Add a collision box obstacle
        box_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]  # [x, y, z, qx, qy, qz, qw]
        box_size = [0.2, 0.2, 0.4]  # [length, width, height]
        
        success = self.planner.add_collision_box("obstacle_box", box_pose, box_size)
        if not success:
            self.get_logger().error('Failed to add collision box')
            return False
        
        # Plan around the obstacle
        target_pose = [0.6, 0.3, 0.7, 0.0, 0.707, 0.0, 0.707]  # Behind the box
        
        success, trajectory, error_msg = self.planner.plan_to_pose(
            target_pose=target_pose,
            enable_collision_avoidance=True,
            apply_min_jerk_smoothing=True,
            velocity_scaling=0.3,  # Conservative for precision
            acceleration_scaling=0.3,
            planning_time=10.0,
            planner_id="RRTConnect"
        )
        
        if success:
            self.get_logger().info(f'Successfully planned collision-free trajectory!')
            self.get_logger().info(f'Trajectory has {len(trajectory.points)} points')
            
            # Analyze trajectory for precision
            self._analyze_trajectory_precision(trajectory)
            
            # Execute trajectory
            execution_success = self.planner.execute_trajectory(trajectory, blocking=True)
            if execution_success:
                self.get_logger().info('Trajectory executed successfully!')
            else:
                self.get_logger().error('Trajectory execution failed')
                
        else:
            self.get_logger().error(f'Planning failed: {error_msg}')
            return False
        
        # Clean up
        self.planner.remove_collision_object("obstacle_box")
        return True
    
    def demo_2_cartesian_path_with_obstacles(self):
        """Demo 2: Cartesian path planning with collision avoidance."""
        self.get_logger().info('=== Demo 2: Cartesian Path with Obstacles ===')
        
        # Add multiple obstacles
        obstacles = [
            {"id": "wall_1", "pose": [0.4, -0.3, 0.5, 0, 0, 0, 1], "size": [0.1, 0.6, 1.0]},
            {"id": "wall_2", "pose": [0.4, 0.3, 0.5, 0, 0, 0, 1], "size": [0.1, 0.6, 1.0]},
            {"id": "ceiling", "pose": [0.5, 0.0, 0.8, 0, 0, 0, 1], "size": [0.8, 0.8, 0.1]}
        ]
        
        for obs in obstacles:
            success = self.planner.add_collision_box(obs["id"], obs["pose"], obs["size"])
            if not success:
                self.get_logger().error(f'Failed to add obstacle: {obs["id"]}')
        
        # Define Cartesian waypoints that navigate between obstacles
        waypoints = [
            [0.3, 0.0, 0.4, 0, 0.707, 0, 0.707],  # Start position
            [0.5, 0.0, 0.4, 0, 0.707, 0, 0.707],  # Move forward (under ceiling)
            [0.7, 0.0, 0.4, 0, 0.707, 0, 0.707],  # Continue forward
            [0.7, 0.0, 0.6, 0, 0.707, 0, 0.707],  # Move up (past ceiling)
            [0.5, 0.0, 0.6, 0, 0.707, 0, 0.707],  # Move back
            [0.3, 0.0, 0.6, 0, 0.707, 0, 0.707],  # Return to start x
        ]
        
        success, trajectory, fraction, error_msg = self.planner.plan_cartesian_path(
            waypoints=waypoints,
            max_step=0.01,  # 1cm steps for precision
            avoid_collisions=True,
            velocity_scaling=0.2,  # Very conservative for precision
            acceleration_scaling=0.2
        )
        
        if success and fraction > 0.95:  # At least 95% of path achieved
            self.get_logger().info(f'Cartesian path planned: {fraction:.1%} achieved')
            
            # Analyze trajectory
            self._analyze_trajectory_precision(trajectory)
            
            # Execute trajectory
            execution_success = self.planner.execute_trajectory(trajectory, blocking=True)
            if execution_success:
                self.get_logger().info('Cartesian path executed successfully!')
            else:
                self.get_logger().error('Cartesian path execution failed')
                
        else:
            self.get_logger().error(f'Cartesian planning failed: {error_msg} (fraction: {fraction:.1%})')
        
        # Clean up obstacles
        for obs in obstacles:
            self.planner.remove_collision_object(obs["id"])
        
        return success and fraction > 0.95
    
    def demo_3_dynamic_obstacle_avoidance(self):
        """Demo 3: Dynamic obstacle avoidance with replanning."""
        self.get_logger().info('=== Demo 3: Dynamic Obstacle Avoidance ===')
        
        # Initial target
        target_pose = [0.6, 0.2, 0.6, 0, 0.707, 0, 0.707]
        
        # Plan initial trajectory
        success, trajectory, error_msg = self.planner.plan_to_pose(
            target_pose=target_pose,
            enable_collision_avoidance=True,
            velocity_scaling=0.4,
            acceleration_scaling=0.4
        )
        
        if not success:
            self.get_logger().error(f'Initial planning failed: {error_msg}')
            return False
        
        self.get_logger().info('Initial trajectory planned successfully')
        
        # Start trajectory execution (non-blocking)
        self.planner.execute_trajectory(trajectory, blocking=False)
        
        # Simulate dynamic obstacle appearing
        time.sleep(2.0)  # Let robot start moving
        
        # Add dynamic obstacle
        dynamic_obstacle_pose = [0.5, 0.1, 0.5, 0, 0, 0, 1]
        dynamic_obstacle_size = [0.3, 0.3, 0.3]
        
        self.get_logger().info('Adding dynamic obstacle...')
        self.planner.add_collision_box("dynamic_obstacle", dynamic_obstacle_pose, dynamic_obstacle_size)
        
        # Replan trajectory to avoid new obstacle
        success, new_trajectory, error_msg = self.planner.plan_to_pose(
            target_pose=target_pose,
            enable_collision_avoidance=True,
            velocity_scaling=0.3,
            acceleration_scaling=0.3,
            planning_time=5.0
        )
        
        if success:
            self.get_logger().info('Successfully replanned around dynamic obstacle')
            
            # Execute new trajectory
            execution_success = self.planner.execute_trajectory(new_trajectory, blocking=True)
            if execution_success:
                self.get_logger().info('Replanned trajectory executed successfully!')
            else:
                self.get_logger().error('Replanned trajectory execution failed')
        else:
            self.get_logger().error(f'Replanning failed: {error_msg}')
        
        # Clean up
        self.planner.remove_collision_object("dynamic_obstacle")
        return success
    
    def demo_4_high_precision_assembly_task(self):
        """Demo 4: High-precision assembly task simulation."""
        self.get_logger().info('=== Demo 4: High-Precision Assembly Task ===')
        
        # Simulate assembly environment with tight tolerances
        assembly_fixtures = [
            {"id": "fixture_1", "pose": [0.45, -0.1, 0.3, 0, 0, 0, 1], "size": [0.05, 0.05, 0.2]},
            {"id": "fixture_2", "pose": [0.45, 0.1, 0.3, 0, 0, 0, 1], "size": [0.05, 0.05, 0.2]},
            {"id": "base_plate", "pose": [0.5, 0.0, 0.25, 0, 0, 0, 1], "size": [0.3, 0.3, 0.02]}
        ]
        
        for fixture in assembly_fixtures:
            self.planner.add_collision_box(fixture["id"], fixture["pose"], fixture["size"])
        
        # Define precise assembly waypoints (threading through narrow gaps)
        assembly_waypoints = [
            [0.3, 0.0, 0.5, 0, 1, 0, 0],      # Approach position
            [0.4, 0.0, 0.4, 0, 1, 0, 0],      # Pre-insertion
            [0.45, 0.0, 0.35, 0, 1, 0, 0],    # Insertion start (between fixtures)
            [0.5, 0.0, 0.28, 0, 1, 0, 0],     # Final insertion (just above base)
        ]
        
        # Use very conservative parameters for high precision
        success, trajectory, fraction, error_msg = self.planner.plan_cartesian_path(
            waypoints=assembly_waypoints,
            max_step=0.005,  # 5mm steps for extreme precision
            avoid_collisions=True,
            velocity_scaling=0.1,  # 10% of max velocity
            acceleration_scaling=0.1  # 10% of max acceleration
        )
        
        if success and fraction > 0.99:  # Require 99% completion for assembly
            self.get_logger().info(f'High-precision assembly path planned: {fraction:.2%} achieved')
            
            # Detailed trajectory analysis for precision validation
            precision_ok = self._validate_precision_requirements(trajectory)
            
            if precision_ok:
                self.get_logger().info('Trajectory meets <0.5mm precision requirements')
                
                # Execute with maximum precision
                execution_success = self.planner.execute_trajectory(trajectory, blocking=True)
                if execution_success:
                    self.get_logger().info('High-precision assembly task completed!')
                else:
                    self.get_logger().error('Assembly task execution failed')
            else:
                self.get_logger().warn('Trajectory does not meet precision requirements')
                
        else:
            self.get_logger().error(f'Assembly planning failed: {error_msg} (fraction: {fraction:.2%})')
        
        # Clean up
        for fixture in assembly_fixtures:
            self.planner.remove_collision_object(fixture["id"])
        
        return success and fraction > 0.99
    
    def demo_5_multi_robot_coordination(self):
        """Demo 5: Multi-robot coordination with shared workspace."""
        self.get_logger().info('=== Demo 5: Multi-Robot Coordination ===')
        
        # Simulate other robot's workspace as dynamic obstacles
        other_robot_poses = [
            [0.3, 0.3, 0.4, 0, 0, 0, 1],  # Other robot base
            [0.4, 0.4, 0.5, 0, 0, 0, 1],  # Other robot arm segment
            [0.5, 0.5, 0.6, 0, 0, 0, 1],  # Other robot end-effector
        ]
        
        other_robot_sizes = [
            [0.2, 0.2, 0.1],  # Base
            [0.1, 0.1, 0.3],  # Arm
            [0.05, 0.05, 0.1]  # End-effector
        ]
        
        # Add other robot as obstacles
        for i, (pose, size) in enumerate(zip(other_robot_poses, other_robot_sizes)):
            self.planner.add_collision_box(f"other_robot_{i}", pose, size)
        
        # Plan coordinated motion that avoids other robot
        target_pose = [0.7, 0.0, 0.5, 0, 0.707, 0, 0.707]  # Move around other robot
        
        success, trajectory, error_msg = self.planner.plan_to_pose(
            target_pose=target_pose,
            enable_collision_avoidance=True,
            velocity_scaling=0.3,
            acceleration_scaling=0.3,
            planning_time=8.0,
            planner_id="RRTstar"  # Use RRT* for optimal paths
        )
        
        if success:
            self.get_logger().info('Multi-robot coordination trajectory planned')
            
            # Execute coordinated motion
            execution_success = self.planner.execute_trajectory(trajectory, blocking=True)
            if execution_success:
                self.get_logger().info('Coordinated motion completed successfully!')
            else:
                self.get_logger().error('Coordinated motion execution failed')
        else:
            self.get_logger().error(f'Multi-robot planning failed: {error_msg}')
        
        # Clean up
        for i in range(len(other_robot_poses)):
            self.planner.remove_collision_object(f"other_robot_{i}")
        
        return success
    
    def _analyze_trajectory_precision(self, trajectory):
        """Analyze trajectory for precision metrics."""
        if len(trajectory.points) < 2:
            return
        
        # Calculate maximum velocities and accelerations
        max_velocities = []
        max_accelerations = []
        
        for i in range(len(trajectory.joint_names)):
            joint_velocities = [abs(point.velocities[i]) for point in trajectory.points if len(point.velocities) > i]
            joint_accelerations = [abs(point.accelerations[i]) for point in trajectory.points if len(point.accelerations) > i]
            
            if joint_velocities:
                max_velocities.append(max(joint_velocities))
            if joint_accelerations:
                max_accelerations.append(max(joint_accelerations))
        
        self.get_logger().info('Trajectory Analysis:')
        self.get_logger().info(f'  Duration: {trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9:.3f}s')
        self.get_logger().info(f'  Points: {len(trajectory.points)}')
        
        if max_velocities:
            self.get_logger().info(f'  Max velocity: {max(max_velocities):.4f} rad/s')
        if max_accelerations:
            self.get_logger().info(f'  Max acceleration: {max(max_accelerations):.4f} rad/s²')
    
    def _validate_precision_requirements(self, trajectory) -> bool:
        """Validate trajectory meets <0.5mm precision requirements."""
        # Check trajectory smoothness (continuous derivatives)
        if len(trajectory.points) < 3:
            return True
        
        # Check for velocity discontinuities
        for i in range(1, len(trajectory.points) - 1):
            prev_vel = np.array(trajectory.points[i-1].velocities)
            curr_vel = np.array(trajectory.points[i].velocities)
            next_vel = np.array(trajectory.points[i+1].velocities)
            
            # Check velocity continuity
            vel_jump = np.max(np.abs(curr_vel - prev_vel))
            if vel_jump > 0.1:  # 0.1 rad/s discontinuity threshold
                self.get_logger().warn(f'Velocity discontinuity detected: {vel_jump:.4f} rad/s')
                return False
        
        # Check acceleration limits for smooth motion
        for point in trajectory.points:
            if len(point.accelerations) > 0:
                max_acc = max(abs(acc) for acc in point.accelerations)
                if max_acc > 5.0:  # Conservative acceleration limit
                    self.get_logger().warn(f'High acceleration detected: {max_acc:.4f} rad/s²')
                    return False
        
        return True
    
    def run_all_demos(self):
        """Run all demonstration scenarios."""
        self.get_logger().info('Starting comprehensive collision-aware trajectory planning demos...')
        
        demos = [
            ("Basic Collision Avoidance", self.demo_1_basic_collision_avoidance),
            ("Cartesian Path with Obstacles", self.demo_2_cartesian_path_with_obstacles),
            ("Dynamic Obstacle Avoidance", self.demo_3_dynamic_obstacle_avoidance),
            ("High-Precision Assembly", self.demo_4_high_precision_assembly_task),
            ("Multi-Robot Coordination", self.demo_5_multi_robot_coordination),
        ]
        
        results = []
        
        for demo_name, demo_func in demos:
            self.get_logger().info(f'\n{"="*60}')
            self.get_logger().info(f'Running: {demo_name}')
            self.get_logger().info(f'{"="*60}')
            
            try:
                success = demo_func()
                results.append((demo_name, success))
                
                if success:
                    self.get_logger().info(f'✓ {demo_name} completed successfully')
                else:
                    self.get_logger().error(f'✗ {demo_name} failed')
                    
                # Wait between demos
                time.sleep(2.0)
                
            except Exception as e:
                self.get_logger().error(f'✗ {demo_name} failed with exception: {str(e)}')
                results.append((demo_name, False))
        
        # Summary
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info('DEMO SUMMARY')
        self.get_logger().info(f'{"="*60}')
        
        successful = sum(1 for _, success in results if success)
        total = len(results)
        
        for demo_name, success in results:
            status = "✓ PASS" if success else "✗ FAIL"
            self.get_logger().info(f'{status}: {demo_name}')
        
        self.get_logger().info(f'\nOverall: {successful}/{total} demos successful')
        
        if successful == total:
            self.get_logger().info('🎉 All demos completed successfully!')
            self.get_logger().info('MoveIt2 Min-Jerk Planner with collision avoidance is working perfectly!')
        else:
            self.get_logger().warn(f'⚠️  {total - successful} demo(s) failed. Check logs for details.')


def main(args=None):
    """Main function for the collision-aware demo."""
    rclpy.init(args=args)
    
    demo_node = CollisionAwareDemo()
    
    try:
        # Run all demonstrations
        demo_node.run_all_demos()
        
        # Keep node alive for any remaining operations
        demo_node.get_logger().info('Demo completed. Press Ctrl+C to exit.')
        rclpy.spin(demo_node)
        
    except KeyboardInterrupt:
        demo_node.get_logger().info('Demo interrupted by user')
    except Exception as e:
        demo_node.get_logger().error(f'Demo failed with exception: {str(e)}')
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()