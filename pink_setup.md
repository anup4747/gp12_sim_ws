# Pink IK Setup Guide for Motoman GP12

This guide documents the specific installation steps and fixes required to get the Pink IK system running on ROS 2 Humble (Ubuntu 22.04).

## 1. Core Python Libraries
Install the main libraries for kinematics and optimization:
```bash
pip install pinocchio pin-pink qpsolvers
```

## 2. Essential Fixes (Common Errors)

### A. NumPy Version Conflict (`ImportError: numpy.core.multiarray failed to import`)
ROS 2 Humble and the pre-built Pinocchio/SciPy binaries on Ubuntu 22.04 require **NumPy 1.x**. If you have NumPy 2.x installed, the system will crash.
**Fix:** Downgrade to the latest 1.x version:
```bash
pip install "numpy<2"
```

### B. Missing OSQP Solver (`SolverNotFound: 'osqp'`)
The Pink library defaults to using the OSQP solver for performance. It must be installed as an extra for `qpsolvers`.
**Fix:** Install the OSQP solver:
```bash
pip install qpsolvers[osqp]
```

## 3. Robot Integration
1. **Generate URDF**: Pink needs a static URDF file rather than a Xacro.
   ```bash
   xacro src/gp12_simulation/urdf/gp12.xacro > src/gp12_simulation/urdf/gp12.urdf
   ```
2. **Launch System**:
   ```bash
   ros2 launch gp12_simulation gp12_pink.launch.py
   ```

## 4. Hyper IMU Setup
- **Endpoint**: UDP
- **Port**: 5555
- **Format**: CSV
- **Sensors**: Accelerometer and Rotation Vector (non-wakeup is fine).
