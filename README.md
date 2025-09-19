# URDF → Screw List (ROS 2)

A tiny ROS 2 utility that converts a URDF/Xacro robot model into a **Screw List** for PoEs-based robot kinematics. It parses the kinematic chain (base → end-effector) and outputs:

- **`screw_list`**: twists in **$S = [v^\top\ \omega^\top]^\top \in \mathbb{R}^6$** (linear on top, angular below)
- **`M`**: the end-effector home pose $^{\mathbf{base}}\xi_{\mathbf{ee}}\in SE(3)$ at zero joint configuration  
- **Frame options**: **space or body screws** (via `use_body_frame`)
- **`joint_limits`**: per joint limit `lower/upper/velocity/effort` extracted from URDF
- **`robot_name`**: robot name extracted from URDF
- **`base_link`, `ee_link`, `num_joints` (DoFs)**: metadata saved alongside the screw list


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
    - [OpenArmv10](#openarmv10)
    - [TM5-700](#tm5-700)

---

## Features

- **URDF/Xacro → Screw list** for a chosen chain (`base_link` → `ee_link`)
- **Twist column order**: **$S = [v^T \omega^T]^T\in\mathbb{R}^6$** (linear on top, angular below)
- **Space or body screws**: toggle with `use_body_frame`
- **Home pose representation**: emit as (position + quaternion_wxzy) or transformation matrix with `home_pose_as_pos_quat`
- **Plain YAML file** output
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


### OpenArmv10

> The robot_description of openarm_v10 can be downloaded [here](https://github.com/enactic/openarm_description.git). Make sure to install it first!!!

```bash
ros2 launch urdf_to_screw_list openarm_v10.launch.py \
arm_type:=v10 bimanual:=false \
base_link:=openarm_link0 ee_link:=openarm_hand \
use_body_frame:=true \
home_pose_as_pos_quat:=true \
output_format:=yaml \
output_path:=<your_path>/openarm_v10_body_screws.yaml
```

Then, you will get `openarm_v10_body_screws.yaml`:

```yaml
robot_name: openarm
base_link: openarm_link0
ee_link: openarm_hand
screw_representation: body  # space or body
joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
num_joints: 7
screw_list:
  openarm_joint1: [0.0045000000, 0.0000010000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
  openarm_joint2: [0.0000000000, 0.5361000000, 0.0045000000, -1.0000000000, 0.0000000000, 0.0000000000]
  openarm_joint3: [0.0045000000, 0.0000010000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
  openarm_joint4: [0.3161000000, 0.0000000000, -0.0000010000, 0.0000000000, 1.0000000000, 0.0000000000]
  openarm_joint5: [0.0045000000, 0.0000010000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]
  openarm_joint6: [0.0000000000, -0.1001000000, -0.0045000000, 1.0000000000, 0.0000000000, 0.0000000000]
  openarm_joint7: [-0.1001000000, 0.0000000000, 0.0000010000, 0.0000000000, -1.0000000000, 0.0000000000]
joint_limits:
  openarm_joint1: { lower: -1.3962630000, upper: 3.4906590000, velocity: 16.7546660000, effort: 40.0000000000 }
  openarm_joint2: { lower: -1.7453290000, upper: 1.7453290000, velocity: 16.7546660000, effort: 40.0000000000 }
  openarm_joint3: { lower: -1.5707960000, upper: 1.5707960000, velocity: 5.4454260000, effort: 27.0000000000 }
  openarm_joint4: { lower: 0.0000000000, upper: 2.4434610000, velocity: 5.4454260000, effort: 27.0000000000 }
  openarm_joint5: { lower: -1.5707960000, upper: 1.5707960000, velocity: 20.9439460000, effort: 7.0000000000 }
  openarm_joint6: { lower: -0.7853980000, upper: 0.7853980000, velocity: 20.9439460000, effort: 7.0000000000 }
  openarm_joint7: { lower: -1.5707960000, upper: 1.5707960000, velocity: 20.9439460000, effort: 7.0000000000 }
M_position: [0.0000010000, -0.0045000000, 0.6586000000]
M_quaternion_wxyz: [1.0000000000, 0.0000000000, 0.0000000000, 0.0000000000]
```

### TM5-700

> The robot_description of tm5-700 can be downloaded [here](https://github.com/TechmanRobotInc/tmr_ros2/tree/humble). Make sure to install it first!!!

```bash
ros2 launch urdf_to_screw_list tm5_700.launch.py \
base_link:=base ee_link:=flange \
use_body_frame:=true \
home_pose_as_pos_quat:=true \
output_format:=yaml \
output_path:=<your_path>/tm5_700_body_screws.yaml
```

Then you will get `tm5_700_body_screws.yaml`:

```yaml
robot_name: tm5-700
base_link: base
ee_link: flange
screw_representation: body  # space or body
joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
num_joints: 6
screw_list:
  joint_1: [0.2354500000, 0.0000000000, -0.0000002093, -0.0000000000, 1.0000000000, 0.0000006536]
  joint_2: [0.7465000370, -0.0000002093, -0.0000000000, 0.0000000000, 0.0000006536, -1.0000000000]
  joint_3: [0.4175000370, -0.0000001018, -0.0000000000, 0.0000000000, 0.0000006536, -1.0000000000]
  joint_4: [0.1060000370, 0.0000000000, -0.0000000000, 0.0000000000, 0.0000006536, -1.0000000000]
  joint_5: [0.1131500000, 0.0000000000, -0.0000000000, -0.0000000000, 1.0000000000, 0.0000003268]
  joint_6: [0.0000000000, -0.0000000000, 0.0000000000, -0.0000000000, 0.0000000000, 1.0000000000]
joint_limits:
  joint_1: { lower: -4.7123889804, upper: 4.7123889804, velocity: 3.1415926536, effort: 157.0000000000 }
  joint_2: { lower: -3.1415926536, upper: 3.1415926536, velocity: 3.1415926536, effort: 157.0000000000 }
  joint_3: { lower: -2.7052603406, upper: 2.7052603406, velocity: 3.1415926536, effort: 157.0000000000 }
  joint_4: { lower: -3.1415926536, upper: 3.1415926536, velocity: 3.9269908170, effort: 43.0000000000 }
  joint_5: { lower: -3.1415926536, upper: 3.1415926536, velocity: 3.9269908170, effort: 43.0000000000 }
  joint_6: { lower: -4.7123889804, upper: 4.7123889804, velocity: 3.9269908170, effort: 43.0000000000 }
M_position: [0.0000002863, -0.2354500000, 0.8917000370]
M_quaternion_wxyz: [0.7071070123, 0.7071065501, 0.0000001155, 0.0000001155]

```