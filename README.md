# Gluon Hardware Interface

Ros2 control hardware interface for the Gluon robotic arm.

## Prerequisites

- ROS 2 Jazzy installed
- Hardware set up
- Calibration done for the arm

⚠️ Warning: Ensure the robot's workspace is clear and safe before operation.
Before launching any bringup or motion planning node, confirm that the arm has free range of motion, no obstacles or people are within reach of the robot, and an emergency stop is accessible.

## Setup

### 1. Clone the repositories

```bash
cd ~/ros2_ws/src

# Robot description
git clone https://github.com/ariccspstk/gluon_description.git

# Hardware interface
git clone --recurse-submodules https://github.com/ariccspstk/gluon_hardware_interface.git


```
### 2. Install dependencies
From the workspace root:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
colcon build
source install/setup.bash
```

## Usage

### Bringup (Requires real hardware)

```bash
ros2 launch gluon_hardware_interface gluon_bringup.launch.py
```

### Control joints with rqt_joint_trajectory_controller

With the gluon_bringup running, open a new terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

In the rqt window:

1. Select the appropriate **Controller Manager ns** (namespace) for your robot.
2. Select the **Controller** (e.g. `joint_trajectory_controller`) from the dropdown.
3. Enable the controller/joints and use the sliders to move each joint.