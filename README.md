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

## How the simulation works

`gazebo.launch.py` composes three pieces:

```
ros2 launch go2_config gazebo.launch.py
 ├─ champ_bringup/bringup.launch.py   # robot_description + champ_base nodes (controller, state estimator, EKFs)
 ├─ champ_gazebo/gazebo.launch.py     # starts Gazebo, loads the world, spawns the robot
 └─ go2_config ground_truth_odom      # publishes /odom + odom -> base_footprint TF (ground truth)
```

**URDF assembly** — `go2_description/xacro/robot.xacro` includes, in order: `const.xacro`,
`leg.xacro`, `materials.xacro`, `gazebo.xacro`, and `sensors.xacro` (the depth camera + Livox
Mid-360 definitions added for this project).

**Gazebo plugins** (defined in `gazebo.xacro` / `sensors.xacro`):

| Plugin | Purpose | Output |
|---|---|---|
| `libgazebo_ros_p3d.so` | Ground-truth pose from Gazebo | `/odom/ground_truth` |
| `libgazebo_ros2_control.so` | Bridge between champ joint commands and Gazebo joints | — |
| `libgazebo_ros_imu_sensor.so` | IMU | `/imu/data` |
| `libgazebo_ros_camera.so` (depth) | Depth camera | `depth_camera/*` |
| `libros2_livox.so` | Livox Mid-360 | `livox_mid360`, `livox_mid360_PointCloud2` |

**Control loop:**

```
/cmd_vel ──► quadruped_controller_node (champ_base)
                └─► joint_group_effort_controller/joint_trajectory
                        └─► gazebo_ros2_control ──► Gazebo joints
```

`champ_base`'s `quadruped_controller_node` subscribes to `/cmd_vel` and turns the velocity
command into leg joint trajectories (gait defined in `go2_config/config/gait/gait.yaml`).

**Odometry** — in simulation the foot-contact odometry is unreliable, so `gazebo.launch.py`
passes `publish_foot_contacts:=false` and `close_loop_odom:=true`. The `ground_truth_odom` node
subscribes to `/odom/ground_truth` and republishes it as `/odom` plus the
`odom -> base_footprint` transform (replacing the foot-contact EKF).

**TF tree:**

```
map ─► odom ─► base_footprint ─► base_link ─► trunk ─┬─► lf_*/rf_*/lh_*/rh_* legs
                                                     ├─► imu_link
                                                     ├─► depth_camera_link
                                                     └─► livox_mid360
```

`map` only appears once SLAM / AMCL is running (see below); otherwise the tree is rooted at `odom`.

## Sensors

| Sensor | Topics |
|---|---|
| Depth camera (front of trunk) | `depth_camera/image_raw`, `depth_camera/depth/image_raw`, `depth_camera/camera_info`, `depth_camera/depth/camera_info`, `depth_camera/points` |
| Livox Mid-360 (top of trunk) | `livox_mid360` (`livox_ros_driver2/msg/CustomMsg`), `livox_mid360_PointCloud2` (`sensor_msgs/msg/PointCloud2`) |

The Mid-360 is a non-repetitive 360° **3D** lidar (no `LaserScan`); use `livox_mid360_PointCloud2`
for visualization or FAST-LIO for 3D SLAM.

## SLAM & Navigation

### Worlds and maps

The simulation **world** and the navigation **map** are two independent arguments:

```bash
# Choose a world (default: worlds/outdoor.world)
ros2 launch go2_config gazebo.launch.py world:=<path/to/world>

# Built-in worlds: worlds/outdoor.world (default), worlds/default.world, worlds/playground.world
```

```bash
# Choose a prebuilt map for AMCL navigation (default: maps/map.yaml)
ros2 launch go2_config navigate.launch.py map:=<path/to/map.yaml>

# Built-in maps: maps/map.yaml, maps/playground.yaml
```

> The **world** is the Gazebo 3D environment; the **map** is the 2D occupancy grid used by
> Nav2/AMCL. They are matched by hand — e.g. `playground.world` pairs with `playground.yaml`.

### Mapping (SLAM)

```bash
ros2 launch go2_config slam.launch.py             # slam_toolbox (online async) + Nav2 + RViz
ros2 run nav2_map_server map_saver_cli -f ~/map   # save the map once built
```

### Localization + navigation (AMCL)

```bash
ros2 launch go2_config navigate.launch.py map:=/path/to/map.yaml
```

### How SLAM connects to sensor data — and a known gap

`slam_toolbox` (`go2_config/config/autonomy/slam.yaml`) is configured with `scan_topic: /scan`,
i.e. it expects a **2D laser scan** (`sensor_msgs/msg/LaserScan`). The Nav2 costmaps
(`config/autonomy/navigation.yaml`) likewise subscribe to 2D laser scans (`/scan`, `/base/scan`)
and 3D point clouds (`/camera/depth/color/points`, `/zed/point_cloud/cloud_registered`).

The Go2 described here currently ships **no 2D lidar**, so this data does **not** match out of
the box:

| Component | Expects | Actually available on this robot |
|---|---|---|
| `slam_toolbox` | `/scan` (LaserScan) | ✗ none |
| costmap `voxel2d_layer` | `/scan`, `/base/scan` (LaserScan) | ✗ none |
| costmap `voxel3d_layer` | `/camera/depth/color/points`, `/zed/...` (PointCloud2) | ✗ (`depth_camera/points` is the real topic) |
| Livox Mid-360 | (not consumed here) | `livox_mid360_PointCloud2` (3D) |

Consequences, and two ways to close the gap:

- **As-is**: `slam_toolbox` receives nothing on `/scan`, so it builds no map, and Nav2's
  obstacle layers have no sensor input.
- **Path A — 2D SLAM (slam_toolbox + Nav2)**: add a 2D lidar to `sensors.xacro`
  (`libgazebo_ros_ray_sensor.so` publishing `/scan`), then update the `navigation.yaml`
  observation topics to the real topics (e.g. `depth_camera/points` for the 3D layer).
- **Path B — 3D SLAM (FAST-LIO)**: use the Mid-360 `livox_mid360_PointCloud2` directly with
  FAST-LIO. The Mid-360 is a non-repetitive 360° 3D lidar that emits `PointCloud2`, not
  `LaserScan`, so it cannot feed `slam_toolbox`; Nav2's 2D costmaps still need a 2D source for
  obstacle avoidance.

## Real robot

```bash
ros2 launch go2_config bringup.launch.py
```
