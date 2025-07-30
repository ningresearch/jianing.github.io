# Min-Jerk Trajectory Planner for ROS 2 Humble

A high-precision minimum jerk trajectory planner designed for robotic applications requiring sub-millimeter accuracy (<0.5mm deviation). This package provides a Python-based solution for MoveIt2 Humble environments where Python API is not available.

## Features

- **High-Precision Control**: Designed for <0.5mm accuracy requirements
- **Min-Jerk Trajectories**: Smooth quintic polynomial trajectories with continuous jerk
- **Real-time Performance**: 1kHz control frequency support
- **Joint Space Planning**: Direct joint trajectory control for maximum precision
- **Multi-waypoint Support**: Complex path planning through multiple waypoints
- **Trajectory Validation**: Built-in joint limit checking and trajectory analysis
- **ROS 2 Integration**: Native ROS 2 Humble support with Joint Trajectory Controller

## Installation

### Prerequisites

- ROS 2 Humble
- Python 3.8+
- NumPy
- PyYAML

### Build Instructions

1. Create or navigate to your ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone or copy the `min_jerk_planner` package to your workspace

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:
```bash
colcon build --packages-select min_jerk_planner
source install/setup.bash
```

## Quick Start

### Basic Usage

1. **Launch the min-jerk planner node:**
```bash
ros2 launch min_jerk_planner min_jerk_demo.launch.py
```

2. **Run the demo client to see examples:**
```bash
ros2 run min_jerk_planner demo_client
```

### Integration with Your Robot

1. **Configure joint limits** in `config/joint_limits.yaml`:
```yaml
joint1:
  min_position: -2.8973  # rad
  max_position: 2.8973   # rad
  max_velocity: 2.1750   # rad/s
  max_acceleration: 15.0 # rad/s²
  max_jerk: 50.0         # rad/s³
```

2. **Update joint names** in the launch file or as parameters:
```bash
ros2 launch min_jerk_planner min_jerk_demo.launch.py \
  joint_names:="['shoulder_pan_joint', 'shoulder_lift_joint', ...]"
```

3. **Connect to your Joint Trajectory Controller:**
```bash
ros2 launch min_jerk_planner min_jerk_demo.launch.py \
  trajectory_topic:="/your_robot/joint_trajectory_controller/joint_trajectory"
```

## API Usage

### Point-to-Point Planning

```python
import numpy as np
from min_jerk_planner.min_jerk_planner_node import MinJerkPlannerNode

# Create planner
planner = MinJerkPlannerNode()

# Define target positions
target_positions = np.array([0.5, -0.3, 1.2, -1.5, 0.8, 0.0])

# Plan trajectory
trajectory = planner.plan_point_to_point_trajectory(
    target_positions=target_positions,
    duration=3.0,
    velocity_scaling=0.5,
    acceleration_scaling=0.3
)

# Execute trajectory
planner.publish_trajectory(trajectory)
```

### Multi-Waypoint Planning

```python
# Define waypoints
waypoints = [
    np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),      # Start
    np.array([0.5, -0.5, 1.0, -1.0, 0.5, 0.2]),    # Waypoint 1
    np.array([1.0, -0.3, 0.8, -1.5, 1.0, -0.3]),   # Waypoint 2
    np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])       # End
]

# Plan trajectory
trajectory = planner.plan_multi_waypoint_trajectory(
    waypoints=waypoints,
    velocity_scaling=0.4,
    acceleration_scaling=0.3
)
```

## High-Precision Configuration

For applications requiring <0.5mm accuracy:

### 1. Control Frequency
Set high control frequency (1kHz recommended):
```yaml
control_frequency: 1000.0
```

### 2. Conservative Scaling
Use conservative velocity/acceleration scaling:
```python
velocity_scaling=0.1      # 10% of max velocity
acceleration_scaling=0.1  # 10% of max acceleration
```

### 3. Joint Limits Tuning
Set appropriate jerk limits for smooth motion:
```yaml
max_jerk: 50.0  # rad/s³ (adjust based on your robot)
```

### 4. Trajectory Analysis
Use built-in analysis tools:
```python
# Analyze trajectory smoothness
is_valid, violations = planner.trajectory_generator.validate_trajectory(
    positions, velocities, accelerations, jerks, joint_names
)
```

## Architecture

### Core Components

1. **MinJerkTrajectoryGenerator**: Core algorithm implementation
   - Quintic polynomial trajectory generation
   - Multi-waypoint trajectory planning
   - Trajectory validation and analysis

2. **MinJerkPlannerNode**: ROS 2 node wrapper
   - Parameter management
   - Joint state monitoring
   - Trajectory publishing
   - Action client interface

3. **Demo Client**: Example usage and testing
   - Point-to-point examples
   - Multi-waypoint examples
   - High-precision demonstrations

### Algorithm Details

The planner uses **quintic polynomials** (5th order) to generate minimum jerk trajectories:

- **Position**: q(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴ + c₅t⁵
- **Velocity**: v(t) = c₁ + 2c₂t + 3c₃t² + 4c₄t³ + 5c₅t⁴
- **Acceleration**: a(t) = 2c₂ + 6c₃t + 12c₄t² + 20c₅t³
- **Jerk**: j(t) = 6c₃ + 24c₄t + 60c₅t²

This ensures **continuous derivatives** up to jerk, providing smooth motion profiles.

## Integration with Joint Trajectory Controller

The planner is designed to work seamlessly with ROS 2 Joint Trajectory Controller:

1. **Trajectory Format**: Generates `trajectory_msgs/JointTrajectory` messages
2. **High-frequency Output**: Supports 1kHz+ trajectory points
3. **Action Interface**: Compatible with `FollowJointTrajectory` actions
4. **Real-time Capable**: Optimized for real-time control loops

## Comparison with MoveIt2

| Feature | Min-Jerk Planner | MoveIt2 |
|---------|------------------|---------|
| Python API | ✅ Full support | ❌ Limited in Humble |
| Min-jerk Trajectories | ✅ Native | ⚠️ Post-processing |
| High-precision Control | ✅ Optimized for <0.5mm | ⚠️ General purpose |
| Real-time Performance | ✅ 1kHz+ capable | ⚠️ Variable |
| Joint Space Focus | ✅ Direct control | ⚠️ Mixed approaches |
| Collision Avoidance | ❌ Not included | ✅ Full support |
| Path Planning | ❌ Not included | ✅ Full support |

## Troubleshooting

### Common Issues

1. **"No joint state received"**
   - Ensure `/joint_states` topic is published
   - Check joint names match your robot configuration

2. **"Trajectory validation failed"**
   - Review joint limits in `config/joint_limits.yaml`
   - Reduce velocity/acceleration scaling factors

3. **Jerky motion despite min-jerk planning**
   - Increase control frequency
   - Check trajectory execution rate
   - Verify Joint Trajectory Controller configuration

### Performance Optimization

1. **For sub-millimeter accuracy:**
   - Use 1kHz+ control frequency
   - Set conservative scaling factors (0.1-0.3)
   - Enable jerk limiting
   - Use high-resolution encoders

2. **For real-time performance:**
   - Minimize trajectory point density when possible
   - Use appropriate time discretization (1ms default)
   - Monitor CPU usage during execution

## Contributing

Contributions are welcome! Please:

1. Follow PEP 8 style guidelines
2. Add unit tests for new features
3. Update documentation
4. Test with real hardware when possible

## License

MIT License - see LICENSE file for details.

## Citation

If you use this work in research, please cite:

```
@software{min_jerk_planner,
  title={High-Precision Min-Jerk Trajectory Planner for ROS 2},
  author={Your Name},
  year={2024},
  url={https://github.com/your-repo/min_jerk_planner}
}
```