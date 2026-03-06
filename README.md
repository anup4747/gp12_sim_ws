# Yaskawa Motoman GP12 – ROS 2 Humble (RViz2 + MoveIt)

ROS 2 Humble workspace for visualizing and planning motions for the **Yaskawa Motoman GP12** industrial robot arm using **RViz2** and **MoveIt**.

---

## Current Status (March 2026)

- ✅ Robot URDF/Xacro parses correctly  
- ✅ Robot model loads in RViz2  
- ✅ TF tree is correct (`base_link → tool0`)  
- ✅ Ready for MoveIt configuration  
- ❌ No Gazebo simulation  
- ❌ No ros2_control  
- ❌ No hardware controllers  

This workspace focuses purely on:

> Kinematics + Visualization + Motion Planning

---

# Requirements

- Ubuntu 22.04 LTS (Jammy)
- ROS 2 Humble (desktop-full recommended)

---

# Installation

## 1️⃣ Install ROS 2 Humble

Follow the official guide:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

---

### Workspace layout

```text
gp12_sim_ws/
├── src/
│   ├── gp12_simulation/
│       ├── launch/
│       │     └── rviz2.launch.py
│       ├── urdf/
│       │     ├── common_colors.xacro
│       │     ├── common_materials.xacro
│       │     ├── gp12_macro.xacro
│       │     └── gp12.xacro
│       └── meshes/
│             ├── collision/
│             └── visual/
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
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-moveit
```


### Quick start

```bash
git clone https://github.com/anup4747/gp12-sim-ws.git
cd gp12-sim-ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gp12_simulation --symlink-install
source install/setup.bash

# Launch Rviz2
ros2 launch gp12_simulation rviz2.launch.py
```

### moveit setup

```
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
