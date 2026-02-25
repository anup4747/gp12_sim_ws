# Yaskawa Motoman GP12 Gazebo Simulation in ROS 2

ROS 2 Humble workspace for simulating the **Yaskawa Motoman GP12** industrial robot arm in **Gazebo Sim** (Ignition-based).

Goal: Enable gesture-based teleoperation (e.g., using smartphone IMU like iQOO Z7s) for mimicking hand/wrist movements on the robot's end-effector.

## Features (Current Status)
- GP12 robot description (URDF/xacro) included locally
- Gazebo simulation with ros2_control integration
- Basic joint trajectory control (`arm_controller`)
- Custom package: `gp12_gazebo` for launch files, controllers, and plugins
- Planned: Phone IMU streaming → gesture mapping → robot control

## Requirements
- **Ubuntu 22.04** (recommended)
- **ROS 2 Humble** (desktop-full)
- **Gazebo Sim** (Fortress or Harmonic) + `ros-humble-ros-gz*` packages
- Installed dependencies:
  ```bash
  sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller


gp12_sim_ws/
├── src/
│   ├── gp12_gazebo/              # Custom package: launches, controllers, urdf overlay
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


git clone https://github.com/anup4747/gp12-gazebo-sim.git
cd gp12-gazebo-sim

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash

ros2 launch gp12_gazebo sim.launch.py
