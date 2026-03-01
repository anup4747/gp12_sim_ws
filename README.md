
# Yaskawa Motoman GP12 Gazebo Simulation in ROS 2 Humble

ROS 2 Humble workspace for simulating the **Yaskawa Motoman GP12** industrial robot arm in **Gazebo Classic** (Gazebo 11) using `ros2_control` and `gazebo_ros2_control`.

**Current status (March 2026):**

- Robot URDF/xacro parses correctly
- Model successfully spawns in Gazebo Classic
- `robot_state_publisher` works and publishes `/joint_states`
- `gazebo_ros2_control` plugin loads (partially)
- **Issue remaining**: controller manager services (`/controller_manager/*`) are not available → spawners hang forever

The goal is to reach a state where joint_state_broadcaster and arm_controller are active and the robot can be commanded.

## Requirements

- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS 2 Humble** (desktop-full recommended)


### Workspace layout

```text
gp12_sim_ws/
├── src/
│   ├── gp12_gazebo/              # Custom package: launches, controllers, URDF overlay
│       ├── launch/
│       │   └── sim.launch.py
│       ├── config/
│       │   └── gp12_controllers.yaml
│       ├──  urdf/
│       │   └── common_colors.xacro
│       │     └── common_materials.xacro
│       │     └── gp12_macro.xacro
│       │     └── gp12.xacro
│       └── meshes/
│             └── collision
|             └── visual
├── .gitignore
└── README.md
```

## Installation & Dependencies

### 1. Install ROS 2 Humble (if not already done)

Follow official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### 2. Install core simulation packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-effort-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-interfaces
```


### Quick start

```bash
git clone https://github.com/anup4747/gp12-sim-ws.git
cd gp12-sim-ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 launch gp12_gazebo sim.launch.py
```