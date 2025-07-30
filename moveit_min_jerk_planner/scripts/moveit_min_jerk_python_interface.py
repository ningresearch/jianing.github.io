#!/usr/bin/env python3
"""
Python interface for MoveIt2 Min-Jerk Planner.

This module provides a Python wrapper around the C++ MoveIt2 min-jerk planner,
enabling high-precision trajectory planning with collision avoidance in Python
despite the lack of native MoveIt2 Python API in Humble.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from typing import List, Tuple, Optional, Dict, Union
import time

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory

# Import custom service messages
from moveit_min_jerk_planner.srv import (
    PlanMinJerkTrajectory,
    PlanCollisionFreeTrajectory, 
    PlanCartesianPath
)


class MoveItMinJerkPlannerPython(Node):
    """
    Python interface for high-precision trajectory planning with MoveIt2.
    
    This class provides a Python API for:
    - Collision-free path planning using MoveIt2
    - Min-jerk trajectory generation for high precision
    - Cartesian path planning with obstacle avoidance
    - Real-time trajectory execution
    """
    
    def __init__(self, group_name: str = "manipulator", node_name: str = "moveit_min_jerk_python"):
        """
        Initialize the Python interface.
        
        Args:
            group_name: MoveIt2 planning group name
            node_name: ROS 2 node name
        """
        super().__init__(node_name)
        
        self.group_name = group_name
        self.callback_group = ReentrantCallbackGroup()
        
        # Service clients for C++ backend
        self.min_jerk_client = self.create_client(
            PlanMinJerkTrajectory,
            '/moveit_min_jerk_planner/plan_min_jerk_trajectory',
            callback_group=self.callback_group
        )
        
        self.collision_free_client = self.create_client(
            PlanCollisionFreeTrajectory,
            '/moveit_min_jerk_planner/plan_collision_free_trajectory',
            callback_group=self.callback_group
        )
        
        self.cartesian_path_client = self.create_client(
            PlanCartesianPath,
            '/moveit_min_jerk_planner/plan_cartesian_path',
            callback_group=self.callback_group
        )
        
        # Action client for trajectory execution
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Publishers for planning scene updates
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10,
            callback_group=self.callback_group
        )
        
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Current state
        self.current_joint_state = None
        
        # Wait for services
        self.get_logger().info('Waiting for MoveIt2 min-jerk planner services...')
        self._wait_for_services()
        
        self.get_logger().info(f'MoveIt2 Min-Jerk Planner Python interface ready for group: {group_name}')
    
    def _wait_for_services(self, timeout: float = 10.0):
        """Wait for all required services to become available."""
        services = [
            self.min_jerk_client,
            self.collision_free_client,
            self.cartesian_path_client
        ]
        
        start_time = time.time()
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                if time.time() - start_time > timeout:
                    raise RuntimeError(f'Service {service.srv_name} not available after {timeout}s')
                self.get_logger().info(f'Waiting for service {service.srv_name}...')
    
    def joint_state_callback(self, msg: JointState):
        """Update current joint state."""
        self.current_joint_state = msg
    
    def get_current_joint_state(self) -> Optional[JointState]:
        """Get current joint state."""
        return self.current_joint_state
    
    def plan_to_pose(
        self,
        target_pose: Union[PoseStamped, Pose, List[float]],
        enable_collision_avoidance: bool = True,
        apply_min_jerk_smoothing: bool = True,
        velocity_scaling: float = 0.5,
        acceleration_scaling: float = 0.5,
        planning_time: float = 5.0,
        planner_id: str = "RRTConnect"
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """
        Plan collision-free trajectory to target pose with min-jerk smoothing.
        
        Args:
            target_pose: Target end-effector pose
            enable_collision_avoidance: Enable collision checking
            apply_min_jerk_smoothing: Apply min-jerk post-processing
            velocity_scaling: Velocity scaling factor (0-1)
            acceleration_scaling: Acceleration scaling factor (0-1)
            planning_time: Maximum planning time (s)
            planner_id: OMPL planner to use
            
        Returns:
            Tuple of (success, trajectory, error_message)
        """
        # Convert input to PoseStamped
        pose_stamped = self._convert_to_pose_stamped(target_pose)
        
        # Create service request
        request = PlanMinJerkTrajectory.Request()
        request.group_name = self.group_name
        request.pose_goal = pose_stamped
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.planning_time = planning_time
        request.enable_collision_checking = enable_collision_avoidance
        request.planner_id = planner_id
        
        # Call service
        try:
            future = self.min_jerk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=planning_time + 2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Successfully planned trajectory in {response.planning_time_taken:.3f}s')
                    return True, response.trajectory, ""
                else:
                    return False, None, response.error_message
            else:
                return False, None, "Service call failed"
                
        except Exception as e:
            return False, None, f"Exception during planning: {str(e)}"
    
    def plan_collision_free_trajectory(
        self,
        start_pose: Union[PoseStamped, Pose],
        goal_pose: Union[PoseStamped, Pose],
        collision_objects: List[Dict] = None,
        velocity_scaling: float = 0.5,
        acceleration_scaling: float = 0.5,
        planning_time: float = 5.0,
        planner_id: str = "RRTConnect"
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """
        Plan collision-free trajectory between two poses.
        
        Args:
            start_pose: Starting end-effector pose
            goal_pose: Target end-effector pose
            collision_objects: List of collision objects to add
            velocity_scaling: Velocity scaling factor (0-1)
            acceleration_scaling: Acceleration scaling factor (0-1)
            planning_time: Maximum planning time (s)
            planner_id: OMPL planner to use
            
        Returns:
            Tuple of (success, trajectory, error_message)
        """
        # Convert inputs
        start_pose_stamped = self._convert_to_pose_stamped(start_pose)
        goal_pose_stamped = self._convert_to_pose_stamped(goal_pose)
        
        # Create service request
        request = PlanCollisionFreeTrajectory.Request()
        request.group_name = self.group_name
        request.start_pose = start_pose_stamped
        request.goal_pose = goal_pose_stamped
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.planning_time = planning_time
        request.planner_id = planner_id
        
        # Add collision objects if provided
        if collision_objects:
            request.collision_objects = self._create_collision_objects(collision_objects)
        
        # Call service
        try:
            future = self.collision_free_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=planning_time + 2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Successfully planned collision-free trajectory')
                    self.get_logger().info(f'Planning time: {response.planning_time_taken:.3f}s')
                    self.get_logger().info(f'Trajectory length: {response.trajectory_length:.3f}')
                    return True, response.trajectory, ""
                else:
                    return False, None, response.error_message
            else:
                return False, None, "Service call failed"
                
        except Exception as e:
            return False, None, f"Exception during planning: {str(e)}"
    
    def plan_cartesian_path(
        self,
        waypoints: List[Union[Pose, List[float]]],
        max_step: float = 0.01,
        avoid_collisions: bool = True,
        velocity_scaling: float = 0.5,
        acceleration_scaling: float = 0.5,
        link_name: str = ""
    ) -> Tuple[bool, Optional[JointTrajectory], float, str]:
        """
        Plan Cartesian path through waypoints with collision avoidance.
        
        Args:
            waypoints: List of Cartesian waypoints
            max_step: Maximum step size between waypoints (m)
            avoid_collisions: Enable collision avoidance
            velocity_scaling: Velocity scaling factor (0-1)
            acceleration_scaling: Acceleration scaling factor (0-1)
            link_name: Link to move (empty = end-effector)
            
        Returns:
            Tuple of (success, trajectory, fraction_achieved, error_message)
        """
        # Convert waypoints to Pose messages
        pose_waypoints = []
        for waypoint in waypoints:
            if isinstance(waypoint, list):
                pose = Pose()
                pose.position.x = waypoint[0]
                pose.position.y = waypoint[1]
                pose.position.z = waypoint[2]
                if len(waypoint) >= 7:
                    pose.orientation.x = waypoint[3]
                    pose.orientation.y = waypoint[4]
                    pose.orientation.z = waypoint[5]
                    pose.orientation.w = waypoint[6]
                else:
                    pose.orientation.w = 1.0
                pose_waypoints.append(pose)
            else:
                pose_waypoints.append(waypoint)
        
        # Create service request
        request = PlanCartesianPath.Request()
        request.group_name = self.group_name
        request.link_name = link_name
        request.waypoints = pose_waypoints
        request.max_step = max_step
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        
        # Call service
        try:
            future = self.cartesian_path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Cartesian path planned: {response.fraction_achieved:.1%} achieved')
                    return True, response.trajectory, response.fraction_achieved, ""
                else:
                    return False, None, 0.0, response.error_message
            else:
                return False, None, 0.0, "Service call failed"
                
        except Exception as e:
            return False, None, 0.0, f"Exception during planning: {str(e)}"
    
    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        blocking: bool = True
    ) -> bool:
        """
        Execute trajectory using action interface.
        
        Args:
            trajectory: Joint trajectory to execute
            blocking: Wait for completion if True
            
        Returns:
            True if execution started successfully
        """
        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Trajectory action server not available')
            return False
        
        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # Send goal
        future = self.trajectory_action_client.send_goal_async(goal_msg)
        
        if blocking:
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('Trajectory execution goal rejected')
                return False
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result()
            if result.result.error_code == 0:
                self.get_logger().info('Trajectory executed successfully')
                return True
            else:
                self.get_logger().error(f'Trajectory execution failed: {result.result.error_string}')
                return False
        
        return True
    
    def add_collision_box(
        self,
        box_id: str,
        pose: Union[PoseStamped, Pose, List[float]],
        size: List[float]
    ) -> bool:
        """
        Add collision box to planning scene.
        
        Args:
            box_id: Unique identifier for the box
            pose: Box pose [x, y, z, qx, qy, qz, qw] or Pose message
            size: Box dimensions [length, width, height]
            
        Returns:
            True if box added successfully
        """
        return self.add_collision_object(box_id, pose, "box", size)
    
    def add_collision_sphere(
        self,
        sphere_id: str,
        pose: Union[PoseStamped, Pose, List[float]],
        radius: float
    ) -> bool:
        """
        Add collision sphere to planning scene.
        
        Args:
            sphere_id: Unique identifier for the sphere
            pose: Sphere pose [x, y, z, qx, qy, qz, qw] or Pose message
            radius: Sphere radius
            
        Returns:
            True if sphere added successfully
        """
        return self.add_collision_object(sphere_id, pose, "sphere", [radius])
    
    def add_collision_object(
        self,
        object_id: str,
        pose: Union[PoseStamped, Pose, List[float]],
        shape_type: str,
        dimensions: List[float]
    ) -> bool:
        """
        Add collision object to planning scene.
        
        Args:
            object_id: Unique identifier for the object
            pose: Object pose
            shape_type: Shape type ("box", "sphere", "cylinder")
            dimensions: Shape dimensions
            
        Returns:
            True if object added successfully
        """
        try:
            # Create collision object
            collision_object = CollisionObject()
            collision_object.header.frame_id = "base_link"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = object_id
            
            # Create primitive shape
            primitive = SolidPrimitive()
            if shape_type.lower() == "box":
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = dimensions  # [x, y, z]
            elif shape_type.lower() == "sphere":
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [dimensions[0]]  # [radius]
            elif shape_type.lower() == "cylinder":
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = dimensions  # [height, radius]
            else:
                self.get_logger().error(f'Unsupported shape type: {shape_type}')
                return False
            
            collision_object.primitives.append(primitive)
            
            # Set pose
            pose_msg = self._convert_to_pose(pose)
            collision_object.primitive_poses.append(pose_msg)
            
            collision_object.operation = CollisionObject.ADD
            
            # Create planning scene message
            planning_scene = PlanningScene()
            planning_scene.world.collision_objects.append(collision_object)
            planning_scene.is_diff = True
            
            # Publish
            self.planning_scene_pub.publish(planning_scene)
            
            self.get_logger().info(f'Added collision {shape_type}: {object_id}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to add collision object: {str(e)}')
            return False
    
    def remove_collision_object(self, object_id: str) -> bool:
        """
        Remove collision object from planning scene.
        
        Args:
            object_id: Object identifier to remove
            
        Returns:
            True if object removed successfully
        """
        try:
            collision_object = CollisionObject()
            collision_object.id = object_id
            collision_object.operation = CollisionObject.REMOVE
            
            planning_scene = PlanningScene()
            planning_scene.world.collision_objects.append(collision_object)
            planning_scene.is_diff = True
            
            self.planning_scene_pub.publish(planning_scene)
            
            self.get_logger().info(f'Removed collision object: {object_id}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to remove collision object: {str(e)}')
            return False
    
    def _convert_to_pose_stamped(self, pose_input: Union[PoseStamped, Pose, List[float]]) -> PoseStamped:
        """Convert various pose inputs to PoseStamped."""
        if isinstance(pose_input, PoseStamped):
            return pose_input
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        if isinstance(pose_input, Pose):
            pose_stamped.pose = pose_input
        elif isinstance(pose_input, list):
            pose_stamped.pose.position.x = pose_input[0]
            pose_stamped.pose.position.y = pose_input[1]
            pose_stamped.pose.position.z = pose_input[2]
            if len(pose_input) >= 7:
                pose_stamped.pose.orientation.x = pose_input[3]
                pose_stamped.pose.orientation.y = pose_input[4]
                pose_stamped.pose.orientation.z = pose_input[5]
                pose_stamped.pose.orientation.w = pose_input[6]
            else:
                pose_stamped.pose.orientation.w = 1.0
        
        return pose_stamped
    
    def _convert_to_pose(self, pose_input: Union[PoseStamped, Pose, List[float]]) -> Pose:
        """Convert various pose inputs to Pose."""
        if isinstance(pose_input, Pose):
            return pose_input
        elif isinstance(pose_input, PoseStamped):
            return pose_input.pose
        elif isinstance(pose_input, list):
            pose = Pose()
            pose.position.x = pose_input[0]
            pose.position.y = pose_input[1]
            pose.position.z = pose_input[2]
            if len(pose_input) >= 7:
                pose.orientation.x = pose_input[3]
                pose.orientation.y = pose_input[4]
                pose.orientation.z = pose_input[5]
                pose.orientation.w = pose_input[6]
            else:
                pose.orientation.w = 1.0
            return pose
    
    def _create_collision_objects(self, objects: List[Dict]) -> List[CollisionObject]:
        """Create CollisionObject messages from dictionary descriptions."""
        collision_objects = []
        
        for obj in objects:
            collision_obj = CollisionObject()
            collision_obj.header.frame_id = obj.get('frame_id', 'base_link')
            collision_obj.id = obj['id']
            
            # Create primitive
            primitive = SolidPrimitive()
            shape_type = obj['shape_type'].lower()
            
            if shape_type == 'box':
                primitive.type = SolidPrimitive.BOX
            elif shape_type == 'sphere':
                primitive.type = SolidPrimitive.SPHERE
            elif shape_type == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
            
            primitive.dimensions = obj['dimensions']
            collision_obj.primitives.append(primitive)
            
            # Set pose
            pose = self._convert_to_pose(obj['pose'])
            collision_obj.primitive_poses.append(pose)
            
            collision_obj.operation = CollisionObject.ADD
            collision_objects.append(collision_obj)
        
        return collision_objects


def main(args=None):
    """Main function for testing the Python interface."""
    rclpy.init(args=args)
    
    planner = MoveItMinJerkPlannerPython()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()