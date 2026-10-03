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

Each patch is described in detail, with rationale and line numbers, in
[`champ/docs/go2-sim-patches.md`](champ/docs/go2-sim-patches.md).

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

Options: `rviz:=true`, `world:=<path>`, `gui:=false` / `headless:=true` (no Gazebo GUI).

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

## Gazebo performance (real-time factor)

Out of the box the simulation ran at **RTF ≈ 0.11** — about 9× slower than wall clock. That is
what makes ROS time look like it is stuttering: `/clock` is perfectly regular in *simulation*
time (exactly 5 ms between messages, standard deviation 0.00), but on the wall clock it arrives
with a mean gap of ~47 ms and a **worst-case gap of 1.1 s**. Every node with `use_sim_time:=true`
therefore sees time freeze and then lurch, and anything that mixes real-time timers with sim time
(Nav2, AMCL, `message_filters`, TF timeouts, RViz) misbehaves.

**The dominant cost is the Livox ray casting inside `gzserver`, not the GUI.** Once the lidar is
actually running, headless and GUI differ by only ~1.3× and RTF stays low either way. Meanwhile
the sensor publish rates on the wall clock are simply `RTF × nominal` — measured 0.66 → lidar
6.7 Hz (= 10 × 0.67) and IMU 66.1 Hz (= 100 × 0.66). Nothing is throttling the sensors; the
simulation just cannot keep up, so a low RTF means proportionally sparse lidar and IMU data.

Levers, measured on a 12-core laptop with a GTX 1650 Ti (see the caveat about desktop load below):

| Cost | Lever | Effect |
|---|---|---|
| Livox ray casting (dominant) | `<downsample>` in `mid360.xacro` | RTF 0.18 → 0.52 (downsample 1 → 4) |
| Scene collision geometry | pick a smaller world | RTF 0.60 → 0.85 (outdoor → playground) |
| `gzclient` rendering | run headless | RTF 0.66 → 0.83 (≈ 1.3×) |
| 1 ms physics step | 2 ms / 500 Hz | small (RTF 0.56 → 0.60) |

> **Absolute RTF depends on what else the machine is doing.** With a browser and desktop
> compositor competing for CPU (load average ~8), the *same* configuration measured 0.26 in one
> run and 0.61 in another. Treat the numbers here as ratios measured back-to-back; re-measure on
> a quiet machine before drawing conclusions from an absolute value.

### What was changed

| File | Change | Why |
|---|---|---|
| `livox_laser_simulation_RO2/urdf/mid360.xacro:47` | sensor `<visualize>true</visualize>` → `false` | The Mid-360 ray sensor is 100 × 360 = **36 000 rays** at 10 Hz, and `<visualize>true</visualize>` made `gzclient` draw every one of them |
| `livox_laser_simulation_RO2/urdf/mid360.xacro:48` | **added** `<always_on>true</always_on>` | **Required** once `<visualize>` is `false` — see the trap below. Without it the lidar silently stops publishing |
| `livox_laser_simulation_RO2/urdf/mid360.xacro:80` | `<downsample>` 1 → **4** | The plugin casts `samples / downsample` rays per scan (40 000 / 4 = 10 000). This is the single biggest lever |
| `go2_config/worlds/{outdoor,default,playground}.world` | `max_step_size` 0.001 → **0.002**, `real_time_update_rate` 1000 → **500** | gzserver was pinned at ~160 % CPU trying to sustain 1000 steps/s |
| same worlds | scene `<shadows>` 1 → **0** | shadow rendering is pure `gzclient` cost |
| `go2_config/launch/gazebo.launch.py` | default world `outdoor.world` → **`playground.world`** | outdoor's mesh collision geometry costs a large share of the ray-casting budget; an empty world is *no faster* than playground, so playground is the better default |
| `go2_config/launch/gazebo.launch.py` | new `headless` argument forwarded to `champ_gazebo`; `gui:=false` kept as an alias | `champ_gazebo` decides whether to start `gzclient` from **`headless`** and never looked at `gui`, so the GUI used to come up no matter what |

### Measured results

World choice (downsample 4, 2 ms, headless):

| World | Models | RTF | Lidar | IMU |
|---|---|---|---|---|
| `outdoor.world` (was the default) | 84 | 0.60 | 5.8 Hz | 58.8 Hz |
| `playground.world` (**now the default**) | 42 | 0.83–0.87 | 8.4 Hz | 82.8 Hz |
| `default.world` (empty) | 2 | 0.82 | 8.5 Hz | 86.7 Hz |

Lidar density vs scan rate, `outdoor.world`, headless, 2 ms — total throughput is ~5.5 × 10⁴
points/s in every row, i.e. CPU-bound, so this is a pure tradeoff and not a free win:

| `<downsample>` | Rays/scan | Cloud points | RTF | Lidar | IMU |
|---|---|---|---|---|---|
| 1 (upstream value) | 40 000 | 80 000 | 0.18 | 1.3 Hz | 12.7 Hz |
| 2 | 20 000 | 40 000 | 0.29 | 2.9 Hz | 23.8 Hz |
| **4 (current)** | 10 000 | 20 000 | 0.52 | 5.7 Hz | 55.9 Hz |

The GUI costs about 1.3×: back-to-back on `playground.world` at downsample 4, RTF went 0.83
headless → 0.66 with the GUI. Repeated runs of that same GUI configuration gave 0.61, 0.66 and
0.34 as the desktop load varied — which is exactly why the numbers above are worth reading as
ratios. At every one of these settings `/clock` no longer shows the multi-hundred-millisecond
gaps that caused the visible stutter.

### Physics was not sacrificed

Standing pose with the lidar actually running (`outdoor.world`, headless, 15 samples each):

| Step | RTF | z | roll std | pitch std |
|---|---|---|---|---|
| 2 ms | 0.60 | 0.2923 (std 0.0003) | 0.0009 | 0.0009 |
| 1 ms | 0.56 | 0.2932 (std 0.0002) | 0.0010 | 0.0010 |

The 2 ms step is statistically indistinguishable from 1 ms. Walking was verified too, with the
lidar running: 10 s at `/cmd_vel` 0.3 m/s moved the robot 0.94 m on outdoor and 1.61 m on
playground (the difference is just RTF — the sim-time speed matches the 0.3 m/s command in both),
and it stayed upright (z ≈ 0.29, |roll| ≤ 0.04, |pitch| ≤ 0.06).

### Gotchas

- **Setting `<visualize>false</visualize>` without `<always_on>true</always_on>` silently kills
  the lidar.** This is the trap to know about. A Gazebo `RaySensor` only wakes up if
  `always_on` is set **or** something is subscribed to the sensor's internal scan topic — and
  that topic exists *only when `<visualize>` is `true`* (`RaySensor` advertises it as part of
  enabling visualization). So `visualize=false` alone leaves the sensor with no reason to update:
  `OnNewLaserScans()` never fires, no point cloud and no `CustomMsg` are ever published, and the
  plugin's publishers sit there with `Publisher count: 1` and zero messages. Worse, it makes RTF
  look *great* (0.95 in one measurement) because the most expensive sensor is not running at all.
  `<always_on>true</always_on>` decouples the two: the sensor keeps working, and because no scan
  topic is advertised, `gzclient` still does not draw the 36 000 rays. The IMU and depth camera
  avoid this because `gazebo.xacro` already sets `<always_on>true</always_on>` on the IMU, and
  `libgazebo_ros_camera.so` activates its sensor itself.
- **`gui:=false` vs `headless:=true`** — both work from `go2_config` now. The value forwarded to
  `champ_gazebo` must be the Python literals `True`/`False`, never `true`/`false`: `champ_gazebo`
  evaluates `not <headless>` through `PythonExpression`, and `not false` raises `NameError`. The
  launch then dies *after* `gzserver` has already started, leaving an orphaned gzserver behind
  with no robot spawned in it. See [`champ/docs/go2-sim-patches.md`](champ/docs/go2-sim-patches.md).
- **These worlds belong to `go2_config`, not CHAMP.** `champ_gazebo/worlds/*.world` still carry
  upstream's 1 ms / 1000 Hz values and were deliberately left alone; `go2_config` overrides the
  world through its `world:=` argument, so they only matter if you launch `champ_gazebo` directly.
- **Rebuild after editing `worlds/` or `*.xacro`.** They are installed as *copies* unless you
  build with `--symlink-install`. Without that flag, run
  `colcon build --packages-select go2_config ros2_livox_simulation` or your changes will have no
  effect — the launch loads them out of `install/`, not `src/`.
- **Tuning the step size further.** 0.0015 / 667 is a reasonable middle ground if 2 ms ever proves
  too coarse for gait work; 0.002 is the largest step verified here to keep the Go2 standing and
  walking normally.
- **Only `mid360.xacro` was changed.** `avia / horizon / mid40 / mid70 / tele / HAP.xacro` in the
  same directory still have `<visualize>true</visualize>`, `<always_on>` unset and
  `<downsample>1</downsample>`; the Go2 uses the Mid-360 only. If you switch to another model,
  apply the same three edits together — particularly `always_on`, or you will hit the trap above.
- **The Livox point cloud is duplicated.** The plugin pushes each point into the outgoing cloud
  twice (`livox_points_plugin.cpp:185` and again at `:191`), so a 10 000-ray scan arrives as a
  20 000-point `PointCloud2` with every point repeated. Pre-existing upstream behaviour, not a
  consequence of the tuning here; halve your expectation of the effective density.

## Sensors

| Sensor | Topics |
|---|---|
| Depth camera (front of trunk) | `depth_camera/image_raw`, `depth_camera/depth/image_raw`, `depth_camera/camera_info`, `depth_camera/depth/camera_info`, `depth_camera/points` |
| Livox Mid-360 (top of trunk) | `livox_mid360` (`livox_ros_driver2/msg/CustomMsg`), `livox_mid360_PointCloud2` (`sensor_msgs/msg/PointCloud2`) |

The Mid-360 is a non-repetitive 360° **3D** lidar (no `LaserScan`); use `livox_mid360_PointCloud2`
for visualization or FAST-LIO for 3D SLAM.

Sensor rates are `RTF × nominal` (lidar 10 Hz, IMU 100 Hz, depth camera 30 Hz), so on a machine
that cannot run the simulation at real time the point cloud arrives proportionally slowly — see
the performance section. If the lidar topics exist but never publish, check `<always_on>` first.

## SLAM & Navigation

### Worlds and maps

The simulation **world** and the navigation **map** are two independent arguments:

```bash
# Choose a world (default: worlds/playground.world)
ros2 launch go2_config gazebo.launch.py world:=<path/to/world>

# Built-in worlds: worlds/playground.world (default), worlds/default.world (empty),
# worlds/outdoor.world (84 models, noticeably slower -- see the performance section)
```

```bash
# Choose a prebuilt map for AMCL navigation (default: maps/map.yaml)
ros2 launch go2_config navigate.launch.py map:=<path/to/map.yaml>

# Built-in maps: maps/map.yaml, maps/playground.yaml
```

> The **world** is the Gazebo 3D environment; the **map** is the 2D occupancy grid used by
> Nav2/AMCL. They are matched by hand — e.g. `playground.world` pairs with `playground.yaml`.

> **The default world is now `playground.world`, not `outdoor.world`** (performance, see above).
> AMCL's default map is still `maps/map.yaml`, which was built for `outdoor.world` — if you
> navigate with the new default world, pass the matching map explicitly:
> `ros2 launch go2_config navigate.launch.py map:=$(ros2 pkg prefix go2_config)/share/go2_config/maps/playground.yaml`

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
