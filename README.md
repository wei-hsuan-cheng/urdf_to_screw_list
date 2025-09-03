# URDF → Screw List (ROS 2)

A tiny ROS 2 utility that converts a URDF/Xacro robot model into a **Screw List** for PoEs-based robot kinematics. It parses the kinematic chain (base → end-effector) and outputs:

- **`screw_list`**: arrays are twists in **[v; w]** order (linear on top, angular below)
- **`M`**: the end-effector home pose $^{\mathbf{base}}_{\mathbf{ee}}T$ at zero joint configuration  
- Frame options: **space screws S** or **body screws B** (via `use_body_frame`)

Inspired by *Modern Robotics* (PoEs method), built on `urdfdom` + `kdl_parser` + `Orocos-KDL`.

---

## Table of Contents

- [URDF → Screw List (ROS 2)](#urdf--screw-list-ros-2)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
    - [Under development](#under-development)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
      - [Dependencies from `CMakeLists.txt`](#dependencies-from-cmakeliststxt)
  - [Run Demo](#run-demo)

---

## Features

- **URDF/Xacro → Screw list** for a chosen chain (`base_link` → `ee_link`)
- **Twist column order**: **$S = [v^T \omega^T]^T\in\mathbb{R}^6$** (linear on top, angular below)
- **Space or body screws**: toggle with `use_body_frame`
- **Plain YAML / TXT / C++ snippet** outputs
- **One-shot node**: runs, writes the file, exits cleanly

---

## Installation

This is a ROS 2 Humble package.

### Prerequisites

- **ROS 2 Humble** installed & sourced
- **C++17** compiler

#### Dependencies from `CMakeLists.txt`

- `rclcpp`
- `kdl_parser`
- `orocos_kdl`
- `urdf`, `urdfdom`, `urdfdom_headers`
- `eigen3_cmake_module` + `Eigen3`

**Build and Install**

```bash
cd ~/ros2_ws/src
cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select urdf_to_screw_list && . install/setup.bash
```

## Run Demo

> The robot_description of openarm_v10 can be downloaded here: https://github.com/enactic/openarm_description.git

```bash
ros2 launch urdf_to_screw_list openarm_v10.launch.py \
arm_type:=v10 bimanual:=false \
base_link:=openarm_link0 ee_link:=openarm_hand_tcp \
use_body_frame:=true output_format:=yaml \
output_path:=/home/you/ros2_ws/src/urdf_to_screw_list/screw_lists/openarm_v10_body_screws.yaml
```


Then, you will get `openarm_v10_body_screws.yaml` like:

```yaml
frame: body
joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
screw_list, S = [v; w] = [-w x q; w]:
  - [0.0045000000, 0.0000010000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
  - [0.0000000000, 0.6161000000, 0.0045000000, -1.0000000000, 0.0000000000, 0.0000000000]
  - [0.0045000000, 0.0000010000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
  - [0.3961000000, 0.0000000000, -0.0000010000, 0.0000000000, 1.0000000000, 0.0000000000]
  - [0.0045000000, 0.0000010000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
  - [0.0000000000, -0.1801000000, -0.0045000000, 1.0000000000, 0.0000000000, 0.0000000000]
  - [-0.1801000000, 0.0000000000, 0.0000010000, 0.0000000000, -1.0000000000, 0.0000000000]
M:
  - [1.0000000000, 0.0000000000, 0.0000000000, 0.0000010000]
  - [0.0000000000, 1.0000000000, 0.0000000000, -0.0045000000]
  - [0.0000000000, 0.0000000000, 1.0000000000, 0.7386000000]
  - [0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
```
