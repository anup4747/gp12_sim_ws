# MoveIt 2 Servo / MoveIt 2 Installation Guide

This guide sets up a working MoveIt 2 development environment from source so you can build the MoveIt tutorials, work with the MoveIt 2 codebase, and run Servo-related examples.

## 1. Install ROS 2

MoveIt 2 supports several ROS 2 distributions. For the newest stable experience, use ROS 2 humble on Ubuntu 22.04.

Recommended setup:

- ROS 2 Humble (Ubuntu 22.04)
- Alternative supported options: Rolling, Jazzy

After installation, source the ROS environment in every new terminal:

```bash
source /opt/ros/humble/setup.bash
```

Important: if you previously sourced another ROS distribution in your shell, make sure the correct one is active before building. Otherwise, you may get dependency or build errors.

## 2. Install Required System Tools

Install rosdep:

```bash
sudo apt install python3-rosdep
```

Update system packages and initialize rosdep:

```bash
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```

Install the ROS 2 build tools:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

Install `vcstool`:

```bash
sudo apt install python3-vcstool
```

## 3. Create a Colcon Workspace

Create a workspace directory and source folder:

```bash
mkdir -p ~/ws_moveit/src
```

## 4. Download MoveIt Source Code

Go into the workspace and clone the MoveIt tutorials repository:

```bash
cd ~/ws_moveit/src
git clone -b <branch> https://github.com/moveit/moveit2_tutorials
```

Replace `<branch>` with the branch you want, for example:

- `humble` for ROS 2 Humble
- `main` for the latest tutorials

Next, import the MoveIt source tree using `vcs`:

```bash
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

If GitHub prompts for credentials, just press Enter and continue. The import may show authentication warnings, but it usually proceeds.

## 5. Install Dependencies

Before building, remove any previous MoveIt binaries from the system:

```bash
sudo apt remove ros-$ROS_DISTRO-moveit*
```

Install all dependencies from Debian packages and ROS package manifests:

```bash
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

## 6. Build the Workspace

Configure and build the workspace:

```bash
cd ~/ws_moveit
colcon build --mixin release
```

This step can take a long time (typically 20–30 minutes or more), depending on CPU speed, RAM, and build parallelism.

### Build Tips

Some packages may require a significant amount of memory during compilation. If your machine is memory-limited, reduce parallelism:

```bash
colcon build --executor sequential --mixin release
```

Or use a stricter limit:

```bash
MAKEFLAGS="-j1 -l1" colcon build --executor sequential --mixin release
```

If the build succeeds, you should see a summary similar to:

```text
Summary: X packages finished
```

## 7. Source the Workspace

After a successful build, source the workspace setup script:

```bash
source ~/ws_moveit/install/setup.bash
```

Optional: add it to your shell startup file:

```bash
echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc
```

## 8. Notes and Best Practices

- Source the correct ROS distribution before building.
- Avoid mixing multiple ROS workspaces unless you know how to manage them carefully.
- For low-memory systems, prefer sequential builds.
- Re-check the ROS installation if build errors appear.

## Summary

The core setup is:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b <branch> https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/ws_moveit
colcon build --mixin release
source ~/ws_moveit/install/setup.bash
```

This creates a fully built MoveIt 2 environment from source and prepares the workspace for running MoveIt and Servo tutorials.