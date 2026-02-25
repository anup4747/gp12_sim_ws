# Yaskawa Motoman GP12 Gazebo Simulation in ROS 2

ROS 2 Humble workspace for simulating the **Yaskawa Motoman GP12** industrial robot arm in **Gazebo Sim** (Ignition-based).

The primary goal is to support gesture‑based teleoperation (e.g. using a smartphone IMU such as the iQOO Z7s) so that hand/wrist motions are mimicked by the robot's end‑effector.

## Features (Current Status)

- [x] GP12 robot description (URDF/xacro) included locally
- [x] Gazebo simulation with `ros2_control` integration
- [x] Basic joint trajectory control (`arm_controller`)
- [x] Custom package `gp12_gazebo` for launch files, controllers, and plugins
- [ ] Planned: phone IMU streaming → gesture mapping → robot control

## Requirements

- **Ubuntu 22.04** (recommended)
- **ROS 2 Humble** (desktop‑full)
- **Gazebo Sim** (Fortress or Harmonic) with `ros-humble-ros-gz*` packages

### Dependencies

```bash
sudo apt install \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller
```

### Workspace layout

```text
gp12_sim_ws/
├── src/
│   ├── gp12_gazebo/              # Custom package: launches, controllers, URDF overlay
│   │   ├── launch/
│   │   │   └── sim.launch.py
│   │   ├── config/
│   │   │   └── gp12_controllers.yaml
│   │   └── urdf/
│   │       └── gp12.xacro        # Local copy with transmissions
│   ├── motoman_ros2_support_packages/   # (optional reference clone)
│   └── motoros2*                 # (optional for real robot later)
├── .gitignore
└── README.md
```

### Quick start

```bash
git clone https://github.com/anup4747/gp12-sim-ws.git
cd gp12-gazebo-sim

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 launch gp12_gazebo sim.launch.py
```