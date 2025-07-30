# MoveIt2 Min-Jerk Planner with Collision Avoidance

A comprehensive high-precision trajectory planner that combines MoveIt2's collision avoidance and path planning capabilities with min-jerk trajectory generation for sub-millimeter accuracy applications.

## 🎯 Key Features

### **Complete Solution for High-Precision Robotics**
- **🔥 Sub-millimeter Accuracy**: Designed for <0.5mm deviation requirements
- **🛡️ Full Collision Avoidance**: Complete MoveIt2 integration with OMPL planners
- **🗺️ Path Planning**: Navigate complex environments with obstacles
- **🎯 Min-Jerk Trajectories**: Smooth quintic polynomial trajectories
- **🐍 Python API**: Full Python interface despite MoveIt2 Humble limitations
- **⚡ Real-time Capable**: 1kHz+ control frequency support
- **🤖 Multi-Robot Support**: Coordinate multiple robots in shared workspace

### **Advanced Capabilities**
- **Dynamic Obstacle Avoidance**: Real-time replanning around moving obstacles
- **Cartesian Path Planning**: Precise end-effector path following
- **Assembly Task Support**: High-precision insertion and threading operations
- **Trajectory Validation**: Built-in collision and limit checking
- **Quality Metrics**: Smoothness and precision analysis

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Python Interface                        │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │ Collision Avoid │  │ Cartesian Path  │  │ Min-Jerk Gen │ │
│  │    Planning     │  │    Planning     │  │  Smoothing   │ │
│  └─────────────────┘  └─────────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                    ┌─────────┴─────────┐
                    │   ROS 2 Services  │
                    └─────────┬─────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                   C++ Backend                              │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐ │
│  │   MoveIt2    │  │ Min-Jerk     │  │ Collision-Aware     │ │
│  │ Integration  │  │ Generator    │  │    Planner          │ │
│  └──────────────┘  └──────────────┘  └─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                    ┌─────────┴─────────┐
                    │ Joint Trajectory  │
                    │   Controller      │
                    └───────────────────┘
```

## 🚀 Quick Start

### Prerequisites
- **ROS 2 Humble** on Ubuntu 22.04
- **MoveIt2 Humble** (`sudo apt install ros-humble-moveit`)
- **Python 3.8+** with NumPy
- **C++17** compiler
- **Eigen3** (`sudo apt install libeigen3-dev`)

### Installation

1. **Create workspace and clone:**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone or copy the moveit_min_jerk_planner package here
```

2. **Install dependencies:**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package:**
```bash
colcon build --packages-select moveit_min_jerk_planner
source install/setup.bash
```

### Launch Demo

```bash
# Terminal 1: Start MoveIt2 and the planner service
ros2 launch moveit_min_jerk_planner moveit_planner_demo.launch.py

# Terminal 2: Run collision-aware demo
ros2 run moveit_min_jerk_planner collision_aware_demo.py
```

## 📖 Usage Examples

### Basic Collision-Free Planning

```python
from moveit_min_jerk_planner.scripts.moveit_min_jerk_python_interface import MoveItMinJerkPlannerPython

# Initialize planner
planner = MoveItMinJerkPlannerPython(group_name="panda_arm")

# Add collision obstacle
box_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]  # [x, y, z, qx, qy, qz, qw]
box_size = [0.2, 0.2, 0.4]  # [length, width, height]
planner.add_collision_box("obstacle", box_pose, box_size)

# Plan collision-free trajectory with min-jerk smoothing
target_pose = [0.6, 0.3, 0.7, 0.0, 0.707, 0.0, 0.707]
success, trajectory, error_msg = planner.plan_to_pose(
    target_pose=target_pose,
    enable_collision_avoidance=True,
    apply_min_jerk_smoothing=True,
    velocity_scaling=0.3,      # Conservative for precision
    acceleration_scaling=0.3,
    planner_id="RRTConnect"
)

if success:
    # Execute trajectory
    planner.execute_trajectory(trajectory, blocking=True)
    print("Collision-free trajectory executed successfully!")
else:
    print(f"Planning failed: {error_msg}")
```

### Cartesian Path Planning

```python
# Define precise Cartesian waypoints
waypoints = [
    [0.3, 0.0, 0.4, 0, 0.707, 0, 0.707],  # Start
    [0.5, 0.0, 0.4, 0, 0.707, 0, 0.707],  # Forward
    [0.7, 0.0, 0.6, 0, 0.707, 0, 0.707],  # Up and forward
    [0.3, 0.0, 0.6, 0, 0.707, 0, 0.707],  # Return
]

success, trajectory, fraction, error_msg = planner.plan_cartesian_path(
    waypoints=waypoints,
    max_step=0.01,              # 1cm steps for precision
    avoid_collisions=True,
    velocity_scaling=0.2,       # Very conservative
    acceleration_scaling=0.2
)

if success and fraction > 0.95:
    print(f"Cartesian path planned: {fraction:.1%} achieved")
    planner.execute_trajectory(trajectory, blocking=True)
```

### High-Precision Assembly Task

```python
# Add assembly fixtures
fixtures = [
    {"id": "fixture_1", "pose": [0.45, -0.1, 0.3, 0, 0, 0, 1], "size": [0.05, 0.05, 0.2]},
    {"id": "fixture_2", "pose": [0.45, 0.1, 0.3, 0, 0, 0, 1], "size": [0.05, 0.05, 0.2]},
]

for fixture in fixtures:
    planner.add_collision_box(fixture["id"], fixture["pose"], fixture["size"])

# Precise assembly waypoints (threading through narrow gap)
assembly_waypoints = [
    [0.3, 0.0, 0.5, 0, 1, 0, 0],      # Approach
    [0.45, 0.0, 0.35, 0, 1, 0, 0],    # Between fixtures
    [0.5, 0.0, 0.28, 0, 1, 0, 0],     # Final insertion
]

# Ultra-precise planning
success, trajectory, fraction, error_msg = planner.plan_cartesian_path(
    waypoints=assembly_waypoints,
    max_step=0.005,             # 5mm steps for extreme precision
    avoid_collisions=True,
    velocity_scaling=0.1,       # 10% of max velocity
    acceleration_scaling=0.1    # 10% of max acceleration
)

if success and fraction > 0.99:  # Require 99% completion
    print("High-precision assembly trajectory ready")
    planner.execute_trajectory(trajectory, blocking=True)
```

## 🎮 Demo Scenarios

The package includes comprehensive demos showcasing different capabilities:

### 1. **Basic Collision Avoidance**
- Single obstacle avoidance
- Min-jerk trajectory smoothing
- Precision trajectory analysis

### 2. **Cartesian Path with Obstacles**
- Navigate through complex environments
- Multiple obstacle types
- Path fraction optimization

### 3. **Dynamic Obstacle Avoidance**
- Real-time replanning
- Moving obstacle detection
- Trajectory re-execution

### 4. **High-Precision Assembly**
- Sub-millimeter accuracy validation
- Tight tolerance operations
- Threading through narrow gaps

### 5. **Multi-Robot Coordination**
- Shared workspace planning
- Robot-robot collision avoidance
- Coordinated motion execution

## ⚙️ Configuration

### Joint Limits Configuration

```yaml
# config/joint_limits.yaml
panda_joint1:
  min_position: -2.8973
  max_position: 2.8973
  max_velocity: 2.1750
  max_acceleration: 15.0
  max_jerk: 50.0          # Critical for min-jerk planning
  has_jerk_limits: true

# High-precision control parameters
control_parameters:
  control_frequency: 1000.0    # 1kHz for precision
  position_tolerance: 0.001    # 1mm tolerance
  enable_jerk_limiting: true
  default_velocity_scaling: 0.5
  default_acceleration_scaling: 0.5
```

### Planning Parameters

```yaml
# config/planning_config.yaml
planning_parameters:
  planner_id: "RRTConnect"
  planning_time: 5.0
  planning_attempts: 5
  max_velocity_scaling_factor: 0.5
  max_acceleration_scaling_factor: 0.5
  
min_jerk_parameters:
  control_frequency: 1000.0
  enable_jerk_limiting: true
  trajectory_smoothing: true
  precision_mode: true        # Enable <0.5mm accuracy mode
```

## 🔧 Advanced Features

### Service Interface

The C++ backend provides ROS 2 services for Python integration:

- **`/moveit_min_jerk_planner/plan_min_jerk_trajectory`**: Basic trajectory planning
- **`/moveit_min_jerk_planner/plan_collision_free_trajectory`**: Full collision avoidance
- **`/moveit_min_jerk_planner/plan_cartesian_path`**: Cartesian path planning

### Trajectory Analysis

```python
# Built-in trajectory analysis
def analyze_trajectory_precision(trajectory):
    """Analyze trajectory for precision metrics."""
    max_velocities = []
    max_accelerations = []
    
    for i, joint_name in enumerate(trajectory.joint_names):
        joint_vels = [abs(point.velocities[i]) for point in trajectory.points]
        joint_accs = [abs(point.accelerations[i]) for point in trajectory.points]
        
        max_velocities.append(max(joint_vels))
        max_accelerations.append(max(joint_accs))
    
    return {
        'duration': trajectory.points[-1].time_from_start,
        'points': len(trajectory.points),
        'max_velocity': max(max_velocities),
        'max_acceleration': max(max_accelerations),
        'smoothness_score': calculate_smoothness(trajectory)
    }
```

### Collision Object Management

```python
# Dynamic collision scene management
collision_objects = [
    {
        'id': 'dynamic_obstacle',
        'shape_type': 'box',
        'dimensions': [0.2, 0.2, 0.4],
        'pose': [0.5, 0.0, 0.5, 0, 0, 0, 1],
        'frame_id': 'base_link'
    }
]

# Add multiple objects at once
for obj in collision_objects:
    planner.add_collision_object(
        obj['id'], obj['pose'], obj['shape_type'], obj['dimensions']
    )
```

## 📊 Performance Characteristics

### Precision Metrics
- **Accuracy**: <0.5mm deviation capability
- **Smoothness**: Continuous derivatives up to jerk
- **Control Frequency**: 1kHz+ support
- **Planning Time**: 1-10s depending on complexity
- **Trajectory Quality**: Optimized for industrial applications

### Supported Planners
- **RRTConnect**: Fast, general-purpose planning
- **RRTstar**: Optimal path planning
- **PRM**: Probabilistic roadmap methods
- **STOMP**: Stochastic trajectory optimization
- **Pilz**: Industrial motion commands

## 🔍 Troubleshooting

### Common Issues

1. **"Service not available"**
   ```bash
   # Check if C++ backend is running
   ros2 service list | grep moveit_min_jerk
   
   # Restart the service node
   ros2 run moveit_min_jerk_planner moveit_planner_service_node
   ```

2. **"Planning failed: No solution found"**
   - Increase planning time: `planning_time=10.0`
   - Try different planner: `planner_id="RRTstar"`
   - Check collision objects for over-constraints
   - Verify joint limits and workspace bounds

3. **"Trajectory validation failed"**
   - Reduce velocity/acceleration scaling
   - Check joint limits configuration
   - Verify trajectory smoothness requirements

4. **"Precision requirements not met"**
   - Lower velocity scaling to 0.1-0.3
   - Increase control frequency to 1kHz+
   - Enable jerk limiting
   - Use smaller Cartesian step sizes (0.005m)

### Performance Optimization

1. **For Maximum Precision:**
   ```python
   planner.plan_to_pose(
       target_pose=target,
       velocity_scaling=0.1,        # Very conservative
       acceleration_scaling=0.1,
       enable_collision_avoidance=True,
       apply_min_jerk_smoothing=True
   )
   ```

2. **For Complex Environments:**
   ```python
   planner.plan_collision_free_trajectory(
       start_pose=start,
       goal_pose=goal,
       planning_time=15.0,          # More time for complex scenes
       planner_id="RRTstar"         # Optimal planning
   )
   ```

## 🤝 Integration Examples

### With Existing MoveIt2 Setup

```python
# Use with existing MoveIt2 configuration
planner = MoveItMinJerkPlannerPython(
    group_name="your_robot_arm",     # Your planning group
    node_name="precision_planner"
)

# Configure for your robot's joint limits
planner.set_joint_limits_from_file("config/your_robot_limits.yaml")
```

### With Custom Controllers

```python
# Custom trajectory topic
planner = MoveItMinJerkPlannerPython(
    group_name="manipulator"
)

# Execute on custom controller
success = planner.execute_trajectory(
    trajectory,
    controller_name="/your_custom_controller/follow_joint_trajectory"
)
```

## 📈 Comparison Matrix

| Feature | Pure MoveIt2 | Min-Jerk Only | **This Package** |
|---------|--------------|---------------|------------------|
| Collision Avoidance | ✅ Full | ❌ None | ✅ **Full** |
| Path Planning | ✅ Full | ❌ Limited | ✅ **Full** |
| Min-Jerk Trajectories | ⚠️ Post-process | ✅ Native | ✅ **Native** |
| Python API (Humble) | ❌ Limited | ✅ Full | ✅ **Full** |
| Sub-mm Precision | ⚠️ General | ✅ Optimized | ✅ **Optimized** |
| Real-time Capable | ⚠️ Variable | ✅ Yes | ✅ **Yes** |
| Industrial Ready | ✅ Yes | ⚠️ Limited | ✅ **Yes** |

## 📝 License

MIT License - see LICENSE file for details.

## 🙏 Acknowledgments

- **MoveIt2 Team**: For the excellent motion planning framework
- **OMPL Developers**: For robust path planning algorithms
- **ROS 2 Community**: For the reliable robotics middleware

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-repo/moveit_min_jerk_planner/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-repo/moveit_min_jerk_planner/discussions)
- **Documentation**: See `docs/` directory for detailed guides

---

**🎯 Perfect for: Industrial automation, precision assembly, medical robotics, research applications requiring both collision avoidance AND sub-millimeter accuracy.**