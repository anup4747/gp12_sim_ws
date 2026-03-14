# MoveIt Setup for Yaskawa GP12

This guide specifically covers how to set up and configure MoveIt for the Motoman GP12 robot after adding the end effector.

## 1. Environment Setup

First, ensure your workspace is built and the environment is sourced.

```bash
cd ~/Desktop/ArmStrong/gp12_sim_ws
colcon build --packages-select gp12_simulation --symlink-install
source install/setup.bash
```

## 2. Generate the URDF File

MoveIt Setup Assistant works best with a single URDF file rather than XACRO files when dealing with complex integrations. Generate the latest URDF from your XACRO:

```bash
# From the workspace root
source /opt/ros/humble/setup.bash
source install/setup.bash
xacro src/gp12_simulation/urdf/gp12.xacro > src/gp12_simulation/urdf/gp12.urdf
```

## 3. Launch MoveIt Setup Assistant

To avoid potential issues with resource paths, it is recommended to launch the setup assistant while having the workspace environment sourced.

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## 4. Loading the Robot

1.  In the Setup Assistant window, select **Create New MoveIt Configuration Package**.
2.  Click **Browse** and navigate to your generated URDF: `src/gp12_simulation/urdf/gp12.urdf`.
3.  Click **Load Files**.
4.  Verify that the robot and the Robotiq gripper are visible in the visualization window.

## 5. Next Steps in Setup Assistant

Follow the standard MoveIt configuration steps:
- **Self-Collisions**: Generate the collision matrix.
- **Virtual Joints**: Usually a fixed joint between `world` and `base_link`.
- **Planning Groups**: Create a group for the robot arm (e.g., `gp12_arm`) and another for the gripper (e.g., `hand`).
- **Robot Poses**: Define some default poses like `home`.
- **End Effectors**: Define the gripper as an end effector for the arm.
- **ROS 2 Controllers**: Configure the necessary controllers (e.g., `FollowJointTrajectory`).
- **Author Information**: Add your name and email.
- **Configuration Package**: Generate a new package named `gp12_moveit_config`.
