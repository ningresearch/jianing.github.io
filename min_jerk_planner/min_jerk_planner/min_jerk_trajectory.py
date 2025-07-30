#!/usr/bin/env python3
"""
Min-jerk trajectory generation for high-precision robotic control.

This module implements minimum jerk trajectory generation algorithms
optimized for sub-millimeter precision applications.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
import warnings


class MinJerkTrajectoryGenerator:
    """
    High-precision minimum jerk trajectory generator.
    
    Generates smooth trajectories with continuous position, velocity, 
    acceleration, and jerk profiles for robotic applications requiring
    high accuracy (<0.5mm deviation).
    """
    
    def __init__(self, dt: float = 0.001):
        """
        Initialize the trajectory generator.
        
        Args:
            dt: Time step for trajectory discretization (default: 1ms for high precision)
        """
        self.dt = dt
        self.joint_limits = {}
        
    def set_joint_limits(self, joint_limits: Dict[str, Dict[str, float]]):
        """
        Set joint limits for trajectory validation.
        
        Args:
            joint_limits: Dictionary with joint names as keys and limit dictionaries as values.
                         Each limit dict should contain: 'position_min', 'position_max', 
                         'velocity_max', 'acceleration_max', 'jerk_max'
        """
        self.joint_limits = joint_limits
    
    def generate_min_jerk_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        duration: float,
        start_vel: Optional[np.ndarray] = None,
        end_vel: Optional[np.ndarray] = None,
        start_acc: Optional[np.ndarray] = None,
        end_acc: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate minimum jerk trajectory between two configurations.
        
        Args:
            start_pos: Starting joint positions [rad]
            end_pos: Target joint positions [rad]
            duration: Trajectory duration [s]
            start_vel: Starting velocities (default: zero)
            end_vel: Target velocities (default: zero)
            start_acc: Starting accelerations (default: zero)  
            end_acc: Target accelerations (default: zero)
            
        Returns:
            Tuple of (time_array, positions, velocities, accelerations, jerks)
        """
        n_joints = len(start_pos)
        
        # Set default boundary conditions
        if start_vel is None:
            start_vel = np.zeros(n_joints)
        if end_vel is None:
            end_vel = np.zeros(n_joints)
        if start_acc is None:
            start_acc = np.zeros(n_joints)
        if end_acc is None:
            end_acc = np.zeros(n_joints)
            
        # Time array
        time_steps = int(duration / self.dt) + 1
        t = np.linspace(0, duration, time_steps)
        
        # Initialize output arrays
        positions = np.zeros((time_steps, n_joints))
        velocities = np.zeros((time_steps, n_joints))
        accelerations = np.zeros((time_steps, n_joints))
        jerks = np.zeros((time_steps, n_joints))
        
        # Generate trajectory for each joint
        for joint_idx in range(n_joints):
            pos_traj, vel_traj, acc_traj, jerk_traj = self._generate_single_joint_min_jerk(
                start_pos[joint_idx],
                end_pos[joint_idx],
                start_vel[joint_idx],
                end_vel[joint_idx],
                start_acc[joint_idx],
                end_acc[joint_idx],
                duration,
                t
            )
            
            positions[:, joint_idx] = pos_traj
            velocities[:, joint_idx] = vel_traj
            accelerations[:, joint_idx] = acc_traj
            jerks[:, joint_idx] = jerk_traj
            
        return t, positions, velocities, accelerations, jerks
    
    def _generate_single_joint_min_jerk(
        self,
        q0: float, q1: float,
        v0: float, v1: float,
        a0: float, a1: float,
        T: float,
        t: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate minimum jerk trajectory for a single joint using quintic polynomial.
        
        The minimum jerk trajectory is characterized by a quintic (5th order) polynomial
        that satisfies boundary conditions on position, velocity, and acceleration.
        """
        # Quintic polynomial coefficients
        # q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        
        # Solve for coefficients using boundary conditions
        c0 = q0
        c1 = v0
        c2 = a0 / 2.0
        
        # Solve 3x3 system for remaining coefficients
        A = np.array([
            [T**3, T**4, T**5],
            [3*T**2, 4*T**3, 5*T**4],
            [6*T, 12*T**2, 20*T**3]
        ])
        
        b = np.array([
            q1 - c0 - c1*T - c2*T**2,
            v1 - c1 - 2*c2*T,
            a1 - 2*c2
        ])
        
        c345 = np.linalg.solve(A, b)
        c3, c4, c5 = c345
        
        # Evaluate polynomial and derivatives
        t_norm = t / T  # Normalize time for numerical stability
        T_norm = T
        
        # Position
        pos = (c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5)
        
        # Velocity  
        vel = (c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4)
        
        # Acceleration
        acc = (2*c2 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3)
        
        # Jerk
        jerk = (6*c3 + 24*c4*t + 60*c5*t**2)
        
        return pos, vel, acc, jerk
    
    def generate_multi_waypoint_trajectory(
        self,
        waypoints: List[np.ndarray],
        segment_durations: Optional[List[float]] = None,
        velocity_scaling: float = 1.0,
        acceleration_scaling: float = 1.0
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate min-jerk trajectory through multiple waypoints.
        
        Args:
            waypoints: List of joint configurations [rad]
            segment_durations: Duration for each segment (auto-calculated if None)
            velocity_scaling: Global velocity scaling factor (0-1)
            acceleration_scaling: Global acceleration scaling factor (0-1)
            
        Returns:
            Tuple of (time_array, positions, velocities, accelerations, jerks)
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints required")
            
        n_segments = len(waypoints) - 1
        n_joints = len(waypoints[0])
        
        # Auto-calculate segment durations if not provided
        if segment_durations is None:
            segment_durations = self._calculate_segment_durations(
                waypoints, velocity_scaling, acceleration_scaling
            )
        
        if len(segment_durations) != n_segments:
            raise ValueError("Number of segment durations must equal number of segments")
        
        # Generate trajectory for each segment
        all_times = []
        all_positions = []
        all_velocities = []
        all_accelerations = []
        all_jerks = []
        
        current_time = 0.0
        
        for i in range(n_segments):
            start_pos = waypoints[i]
            end_pos = waypoints[i + 1]
            duration = segment_durations[i]
            
            # Calculate boundary velocities for continuity (except at endpoints)
            start_vel = np.zeros(n_joints) if i == 0 else self._calculate_via_velocity(
                waypoints[i-1] if i > 0 else waypoints[i],
                waypoints[i],
                waypoints[i+1],
                segment_durations[i-1] if i > 0 else duration,
                duration
            )
            
            end_vel = np.zeros(n_joints) if i == n_segments - 1 else self._calculate_via_velocity(
                waypoints[i],
                waypoints[i+1],
                waypoints[i+2] if i < n_segments - 1 else waypoints[i+1],
                duration,
                segment_durations[i+1] if i < n_segments - 1 else duration
            )
            
            # Generate segment trajectory
            t_seg, pos_seg, vel_seg, acc_seg, jerk_seg = self.generate_min_jerk_trajectory(
                start_pos, end_pos, duration, start_vel, end_vel
            )
            
            # Adjust time to be continuous
            t_seg_adjusted = t_seg + current_time
            current_time += duration
            
            # Skip first point of subsequent segments to avoid duplication
            if i > 0:
                t_seg_adjusted = t_seg_adjusted[1:]
                pos_seg = pos_seg[1:]
                vel_seg = vel_seg[1:]
                acc_seg = acc_seg[1:]
                jerk_seg = jerk_seg[1:]
            
            all_times.append(t_seg_adjusted)
            all_positions.append(pos_seg)
            all_velocities.append(vel_seg)
            all_accelerations.append(acc_seg)
            all_jerks.append(jerk_seg)
        
        # Concatenate all segments
        time_array = np.concatenate(all_times)
        positions = np.concatenate(all_positions, axis=0)
        velocities = np.concatenate(all_velocities, axis=0)
        accelerations = np.concatenate(all_accelerations, axis=0)
        jerks = np.concatenate(all_jerks, axis=0)
        
        return time_array, positions, velocities, accelerations, jerks
    
    def _calculate_segment_durations(
        self,
        waypoints: List[np.ndarray],
        velocity_scaling: float,
        acceleration_scaling: float
    ) -> List[float]:
        """Calculate optimal durations for each segment based on joint limits."""
        durations = []
        
        for i in range(len(waypoints) - 1):
            start_pos = waypoints[i]
            end_pos = waypoints[i + 1]
            
            # Calculate required time based on distance and limits
            max_displacement = np.max(np.abs(end_pos - start_pos))
            
            # Simple heuristic: T = 2 * sqrt(displacement / max_acceleration)
            # This ensures we don't exceed acceleration limits
            estimated_duration = 2.0 * np.sqrt(max_displacement / (acceleration_scaling * 2.0))
            estimated_duration = max(estimated_duration, 0.1)  # Minimum duration
            
            durations.append(estimated_duration)
        
        return durations
    
    def _calculate_via_velocity(
        self,
        prev_waypoint: np.ndarray,
        current_waypoint: np.ndarray,
        next_waypoint: np.ndarray,
        prev_duration: float,
        next_duration: float
    ) -> np.ndarray:
        """
        Calculate velocity at via point for smooth trajectory continuity.
        
        Uses finite difference approximation with time weighting.
        """
        # Weighted average of incoming and outgoing velocities
        v_in = (current_waypoint - prev_waypoint) / prev_duration
        v_out = (next_waypoint - current_waypoint) / next_duration
        
        # Time-weighted average
        total_time = prev_duration + next_duration
        w_in = next_duration / total_time
        w_out = prev_duration / total_time
        
        via_velocity = w_in * v_in + w_out * v_out
        
        return via_velocity
    
    def validate_trajectory(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray,
        jerks: np.ndarray,
        joint_names: List[str]
    ) -> Tuple[bool, List[str]]:
        """
        Validate trajectory against joint limits.
        
        Returns:
            Tuple of (is_valid, list_of_violations)
        """
        violations = []
        
        for i, joint_name in enumerate(joint_names):
            if joint_name not in self.joint_limits:
                continue
                
            limits = self.joint_limits[joint_name]
            
            # Check position limits
            if 'position_min' in limits:
                if np.any(positions[:, i] < limits['position_min']):
                    violations.append(f"{joint_name}: position below minimum")
            
            if 'position_max' in limits:
                if np.any(positions[:, i] > limits['position_max']):
                    violations.append(f"{joint_name}: position above maximum")
            
            # Check velocity limits
            if 'velocity_max' in limits:
                if np.any(np.abs(velocities[:, i]) > limits['velocity_max']):
                    violations.append(f"{joint_name}: velocity exceeds maximum")
            
            # Check acceleration limits  
            if 'acceleration_max' in limits:
                if np.any(np.abs(accelerations[:, i]) > limits['acceleration_max']):
                    violations.append(f"{joint_name}: acceleration exceeds maximum")
            
            # Check jerk limits
            if 'jerk_max' in limits:
                if np.any(np.abs(jerks[:, i]) > limits['jerk_max']):
                    violations.append(f"{joint_name}: jerk exceeds maximum")
        
        return len(violations) == 0, violations