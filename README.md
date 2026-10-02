# go2_sim

Unitree **Go2** quadruped simulation for ROS 2 Humble + Gazebo Classic, based on the
[CHAMP](https://github.com/chvmp/champ) framework, extended with a **depth camera (RGB-D)**
and a **Livox Mid-360** 3D lidar.

This repo lives at the `src/` level of a colcon workspace and contains the Go2-specific
packages only. The CHAMP framework is an external dependency (cloned separately).

## Repository layout

| Package | Description |
|---|---|
| [`go2_description`](go2_description/) | Go2 URDF/xacro description, meshes, `ros2_control` config, and the sensor definitions (`xacro/sensors.xacro`: depth camera + Livox Mid-360) |
| [`go2_config`](go2_config/) | Go2 launch files (Gazebo sim, real-robot bringup, SLAM, navigation), gait/joints configs, worlds, maps, and the ground-truth odometry relay |
| [`livox_laser_simulation_RO2`](livox_laser_simulation_RO2/) | `ros2_livox_simulation`: Gazebo plugin that simulates Livox lidars (publishes `CustomMsg` + `PointCloud2`) |

## Prerequisites

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Gazebo Classic 11 (`ros-humble-gazebo-ros-*`, `ros-humble-gazebo-ros2-control`, `ros-humble-gazebo-plugins`)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) (for the Livox messages)

## Dependencies

### 1. CHAMP

Clone the CHAMP framework next to this repo (inside the same workspace `src/`):

```bash
cd <workspace>/src
git clone https://github.com/chvmp/champ.git
```

> `go2_config` and `go2_description` depend on CHAMP packages
> (`champ_bringup`, `champ_gazebo`, `champ_description`, `champ_navigation`, `champ_base`, ...).

A few CHAMP launch files need small patches for the Go2 sim to run cleanly:

- `champ_gazebo/launch/gazebo.launch.py` — add a `publish_foot_contacts` argument that gates the
  `contact_sensor` node (the foot-contact odometry crashes under Gazebo; `go2_config` passes
  `publish_foot_contacts:=false` in simulation).
- `champ_description/launch/description.launch.py` and
  `champ_bringup/launch/bringup.launch.py` — wrap the `xacro` `Command(...)` results in
  `ParameterValue(..., value_type=str)` so the URDF string is not YAML-parsed.

### 2. livox_ros_driver2

The Livox plugin depends on `livox_ros_driver2` (its `CustomMsg` type). Build it in a separate
workspace (or the same one) and source it before building/lanching:

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# build it per its README
source <livox_ws>/install/setup.bash
```

## Build

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
source <livox_ws>/install/setup.bash        # required for ros2_livox_simulation
colcon build --symlink-install
source install/setup.bash
```

> `livox_laser_simulation_RO2` is a C++ package and needs `livox_ros_driver2` findable at
> build time (hence sourcing the livox workspace above).

## Quick start (Gazebo simulation)

```bash
ros2 launch go2_config gazebo.launch.py
```

Options: `rviz:=true`, `world:=<path>`, `gui:=false` (headless).

The launch starts Gazebo + the Go2 with `gazebo_ros2_control`, and replaces the foot-contact
odometry with the Gazebo ground truth (`/odom` + `odom -> base_footprint` TF).

### Control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i` forward, `,` backward, `j`/`l` turn, `Shift`+`J`/`Shift`+`L` strafe, `k` stop.
The robot listens on `/cmd_vel` (`geometry_msgs/msg/Twist`).

## Sensors

| Sensor | Topics |
|---|---|
| Depth camera (front of trunk) | `depth_camera/image_raw`, `depth_camera/depth/image_raw`, `depth_camera/camera_info`, `depth_camera/depth/camera_info`, `depth_camera/points` |
| Livox Mid-360 (top of trunk) | `livox_mid360` (`livox_ros_driver2/msg/CustomMsg`), `livox_mid360_PointCloud2` (`sensor_msgs/msg/PointCloud2`) |

The Mid-360 is a non-repetitive 360° **3D** lidar (no `LaserScan`); use `livox_mid360_PointCloud2`
for visualization or FAST-LIO for 3D SLAM.

## SLAM & Navigation

```bash
# Online SLAM (slam_toolbox) + Nav2, with RViz
ros2 launch go2_config slam.launch.py

# Save the map, then navigate with AMCL
ros2 run nav2_map_server map_saver_cli -f ~/map
ros2 launch go2_config navigate.launch.py map:=/path/to/map.yaml
```

## Real robot

```bash
ros2 launch go2_config bringup.launch.py
```
