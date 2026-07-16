# Gluon Hardware Interface

ROS 2 hardware interface for the Gluon robotic arm.

## Prerequisites

- ROS 2 Jazzy installed
- Hardware set up
- Calibration done for the arm

## Setup

### 1. Clone the repositories

```bash
cd ~/ros2_ws/src

# Robot description
git clone https://github.com/YongQuanz/gluon_description.git

# Hardware interface
git clone https://github.com/ariccspstk/gluon_hardware_interface.git

# Gluon moveit config files
git clone https://github.com/ariccspstk/gluon_moveit_config.git
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

### Bringup only

```bash
ros2 launch gluon_hardware_interface gluon_bringup.launch.py
```

### Bringup with MoveIt (motion planning)

```bash
ros2 launch gluon_hardware_interface gluon_moveit_bringup.launch.py
```