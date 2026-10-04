# go2_sim_ws — Go2 四足 + Mid-360 仿真 / LIO 建图

ROS 2 Humble + Gazebo Classic 11 的宇树 Go2 仿真，带 Livox Mid-360 雷达、IMU、GPS，
并串好了 Point-LIO 做激光惯性里程计建图。

本文档记录**实际验证过的运行命令**，以及搭建过程中踩过的所有坑（含根因），避免重复排查。

> 上游原始文档在 `src/GazeboQuadbot/README.md`，本文档只覆盖本工作区的改动与运维。

---

## 0. 编译

**不需要 source 任何外部工作区。** `point_lio` 和 `ign_sim_pointcloud_tool` 都已经在本工作区里
（`src/GazeboQuadbot/{point_lio,ign_sim_pointcloud_tool}`），而它们唯一的编译期外部依赖
`livox_ros_driver2` 已经由 `~/.bashrc` 里的 `source /home/ros/ros_ws/livox_ros/install/setup.sh`
提供：

```bash
cd /home/ros/ros_ws/go2_sim_ws
colcon build --symlink-install
```

只改了 LIO / 点云 / 场景的话，可以只编这三个包：

```bash
colcon build --packages-select point_lio ign_sim_pointcloud_tool robot_scene --symlink-install
```

> 若换了机器 / 改了 `.bashrc`，用 `ros2 pkg prefix livox_ros_driver2` 确认它还能找到；
> 找不到就 source 一个提供它的工作区（`livox_ros` 或 `LIO_Nav2_ROS2` 都行）。

> `point_lio/package.xml` 里写了 `<depend>libunwind-dev</depend>`，但 **CMakeLists 和源码根本
> 没用到 libunwind**（只 `find_package(glog)`）。所以**不用装 libunwind-dev**，不装也能编过。
> 用 `rosdep install` 时它会去要这个包，可以忽略。

编译产物验证：

```bash
source install/setup.bash
ros2 pkg prefix point_lio          # 应该指向 go2_sim_ws/install/point_lio
```

---

## 1. 快速开始

**仿真和 LIO 是两个独立的 launch，各起一个终端。**

### 终端 1 —— 只起 Gazebo

```bash
source /home/ros/ros_ws/go2_sim_ws/install/setup.bash

# 带界面
ros2 launch robot_scene go2_lidar_gps.launch.py

# 或 headless（无界面，更快）
ros2 launch robot_scene go2_lidar_gps.launch.py headless:=True
```

### 终端 2 —— 只起 LIO 建图 + RViz

```bash
source /home/ros/ros_ws/go2_sim_ws/install/setup.bash
ros2 launch robot_scene go2_lio.launch.py
```

RViz 用 Point-LIO 的配置自动打开（Fixed Frame `camera_init`）。

> ⚠️ **不要在两个终端里都起 LIO**。`go2_lio.launch.py` 现在**只**起 LIO，不含仿真；
> 如果你额外再起一个带 LIO 的 launch，会有**两个 Point-LIO 同时发 TF
> `camera_init→aft_mapped`**，RViz 收到冲突的 TF 和重复点云，画面就是乱的（踩过）。

### headless 的两个注意点

用 `headless:=True` 时：

- **必须写大写 `True`** —— champ 用 `PythonExpression([" not ", headless])` 判断，小写 `true` 会 abort（坑 7）；
- **启动后等约 50 秒**雷达才出数据 —— 仿真 launch 会在 50 秒时自动订阅 scan 话题唤醒传感器
  （传感器没有 `always_on`，必须有人订阅才激活，见坑 3）。想调延时用
  `activate_delay:=30.0`，但**别调太小**，过早激活会让 gzserver 段错误。

### 两种模式实测差异

同一 `point_rate`（200000）下只变 GUI/headless（RTF 会随机器负载波动 ±10%）：

| | 带界面 | headless |
|---|---|---|
| RTF（稳态） | ~0.36 | **~0.44** |
| `/livox/lidar` | 2.7 Hz | **4.7 Hz** |
| Gazebo 窗口 | 有 | 无 |

> RTF 的上限受射线数限制（见 §10），**不要为了提速去降 `point_rate`** —— 那会让 LIO 发散。

> ⚠️ **不要给仿真 launch 加 `gui:=false`**。那只是关掉 gzclient，但传感器就没人去激活了，
> 雷达**一条数据都没有**。不要界面请用 `headless:=True`。

### 停止

Ctrl-C 各自的终端即可。要一次全停：

```bash
pkill -x gzserver; pkill -x gzclient; pkill -x rviz2
pkill -f "ros2 launch robot_scene"; pkill -f pointlio_mapping
pkill -f ign_sim_pointcloud_tool_node; pkill -f quadruped_controller_node
```

> `pointlio_mapping` 用默认的 SIGTERM 退出会写 PCD（见 §3 的建图产物）。`kill -9` 不会。

---

## 2. 依赖的工作区（重要）

**本工作区已完全自包含**，编译和运行都只需要它自己：

| 提供者 | 内容 | 何时需要 |
|---|---|---|
| `/home/ros/ros_ws/go2_sim_ws` | `point_lio`、`ign_sim_pointcloud_tool`、`robot_scene`、`champ_*`、`go2_config`、`mid360_simulation` | **编译 + 运行** |
| `/home/ros/ros_ws/livox_ros`（由 `~/.bashrc` 自动 source） | `livox_ros_driver2` | **仅编译期**：`point_lio` 需要它的 `CustomMsg` |
| `/opt/ros/humble` | ROS 2 本体 | 运行 |

也就是说正常流程只有：

```bash
cd /home/ros/ros_ws/go2_sim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_scene go2_lio.launch.py
```

> ⚠️ **同名包冲突**：`point_lio` 在本工作区、`LIO_Nav2_ROS2`、`lio_slam_ws` 各有一份；
> `ign_sim_pointcloud_tool` 在本工作区和 `LIO_Nav2_ROS2` 各有一份。
> launch 用 `get_package_share_directory(...)` 按 `AMENT_PREFIX_PATH` 顺序解析，**后 source 的赢**。
> **如果你手动 source 了 `LIO_Nav2_ROS2` / `lio_slam_ws`，一定要把本工作区放在最后 source**，
> 否则会拿到它们那份（可能版本不一致）。
> 本工作区的两份已确认和来源**逐字节一致**（`diff -rq` 无差异）。用 `ros2 pkg prefix` 确认：

```bash
ros2 pkg prefix point_lio                 # 期望 .../go2_sim_ws/install/point_lio
ros2 pkg prefix ign_sim_pointcloud_tool   # 期望 .../go2_sim_ws/install/ign_sim_pointcloud_tool
```

---

## 3. 数据流

```
Gazebo (gzserver)
  └─ libmid360_plugin.so ──► /livox/lidar    sensor_msgs/PointCloud2   ~4 Hz
                             (frame=livox, 字段 x,y,z,intensity,timestamp)
  └─ libgazebo_ros_imu_sensor.so ──► /livox/imu  sensor_msgs/Imu       ~200 Hz
                             (frame=livox_imu)

ign_sim_pointcloud_tool（转换器：补 ring 和 time 字段）
  └─ /livox/lidar ──► /velodyne_points   sensor_msgs/PointCloud2   ~4 Hz
                       (x,y,z,intensity,ring,time；time ∈ [0, 0.1] s)

point_lio（pointlio_mapping，lidar_type=2 / VELODYNE）
  ├─ /cloud_registered      **当前帧**特征点（世界系 camera_init），跟着机器人跑，不是累积地图
  ├─ /cloud_registered_body 当前帧点云（IMU 体系）
  ├─ /Laser_map             初始化瞬间的快照，**只发布一次**
  └─ TF: camera_init ──► aft_mapped
```

### 话题 / 频率速查（实测）

| 话题 | 类型 | frame | 频率 |
|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `livox` | ~4.1 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_imu` | ~199 Hz |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | `livox` | ~4.0 Hz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` | ~3.5 Hz |
| `/gps/data` | `sensor_msgs/NavSatFix` | — | — |
| `/cmd_vel` | `geometry_msgs/Twist` | — | 遥控输入 |
| `/body_pose` | `geometry_msgs/Pose` | — | champ 机身姿态 |

### 外参（从 TF 实测）

```
trunk   -> livox        t = (0, 0, 0.1)                     q = 单位
livox   -> livox_imu    t = (0.011, 0.02329, -0.04412)      q = 单位
```

Point-LIO 的 `mapping.extrinsic_T` 语义是 `p_imu = R * p_lidar + T`（见
`laserMapping.cpp` 里的 `Lidar_T_wrt_IMU`），所以是**上面 livox->livox_imu 的取反**：
`[-0.011, -0.02329, 0.04412]`。`mid360_sim.yaml` 里就是这个值，**不要改成正号**。

---

## 4. 控制机器人行走

`/cmd_vel` 由 `quadruped_controller_node` 独家订阅（RELIABLE QoS）。
launch 里已把 `/cmd_vel/smooth` remap 到 `/cmd_vel`，所以直接发 `/cmd_vel` 即可。

### 键盘遥控

```bash
source /home/ros/ros_ws/go2_sim_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# i 前进 / , 后退 / j 左转 / l 右转 / k 停止
```

### 命令行直接发

**champ 需要持续指令**，`--once` 只会让狗挪一下（实测单发仅走 0.15 m）：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"                 # 前进
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: -0.4}}"  # 前进+右转
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}"                 # 侧移（狗能横着走）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                                # 停止
```

Ctrl-C 停发后，champ 有指令超时保护会**自动停下**。

### 速度上限（`quadruped_controller_node` 的 gait 参数）

| 方向 | 上限 |
|---|---|
| `linear.x` 前后 | ±1.0 m/s |
| `linear.y` 左右 | ±0.5 m/s |
| `angular.z` 转向 | ±1.2 rad/s |

实测：持续发 6 秒 `linear.x=0.3`，机器人位移约 0.68 m，Point-LIO 位姿同步跟上。

### 边遥控边看建图

RViz 里 Fixed Frame 设为 `camera_init`，加 PointCloud2 display 指向 `/cloud_registered`。

> ⚠️ **但 RViz 里看不到"逐渐长出来的地图"**。这个版本的 Point-LIO **不持续发布累积地图**：
>
> - `/cloud_registered` 是**当前帧**特征点，RViz 每帧替换，看上去是一小片点云跟着机器人跑
> - `/Laser_map` 只在**初始化时发布一次**（`laserMapping.cpp:528` 的 `publish_init_map`）
> - 累积地图 `pcl_wait_pub` 在 `laserMapping.cpp:180` **声明了但从未被 publish**
>
> 真正的累积地图只在 **Ctrl-C 正常退出**时写到
> `src/GazeboQuadbot/point_lio/PCD/scans.pcd`（实测静止几秒即 29 万点）。
> 想看"长出来"的过程，可以每隔一段时间退一次、看 PCD；或者自己给
> `laserMapping.cpp` 加一个周期性 `publish_map`。

---

## 5. launch 参数

### `go2_lidar_gps.launch.py` —— 只起仿真

| 参数 | 默认 | 说明 |
|---|---|---|
| `world` | `robot_scene/worlds/indoor_walls1.world` | 世界文件 |
| `gui` | `true` | 传给 champ 的 gui 开关（实际抑制 gzclient 的是 `headless`） |
| `headless` | `False` | **必须写 `True`/`False`（大写）**。不启 gzclient，且**自动在 `activate_delay` 秒后订阅 scan 话题激活雷达**，见坑 3 / §10 |
| `activate_delay` | `50.0` | `headless:=True` 时延时多少秒后激活传感器。**调太小会让 gzserver 段错误** |
| `publish_foot_contacts` | `false` | champ 的 contact_sensor，已知不稳定，默认关（见坑 1） |
| `rviz` | `false` | 传给 champ bringup 的 rviz（不是 Point-LIO 那个） |

其余：`use_sim_time` / `robot_name` / `lite` / `ros_control_file` / `world_init_x|y|z|heading`

### `go2_lio.launch.py` —— 只起 LIO

| 参数 | 默认 | 说明 |
|---|---|---|
| `rviz` | `true` | 起 RViz（Point-LIO 配置，Fixed Frame `camera_init`） |
| `point_lio_cfg` | `point_lio/share/point_lio/config/mid360_sim.yaml` | Point-LIO 参数文件 |

> 这个 launch **不含仿真**。它只起 `ign_sim_pointcloud_tool` 转换器 + `pointlio_mapping` + RViz，
> 订阅已经在跑的仿真发出来的 `/livox/*`。

---

## 6. 踩坑记录

### 坑 1 — `contact_sensor` 启动即崩，`exit code -6`

**现象**
```
[contact_sensor-8] contact_sensor: pthread_mutex_lock.c:94: ___pthread_mutex_lock:
Assertion `mutex->__data.__owner == 0' failed.
[ERROR] [contact_sensor-8]: process has died [... exit code -6 ...]
```

**根因** 两个问题叠加：

1. `champ_gazebo/src/contact_sensor.cpp` 是个"混血"进程：一边跑 `rclcpp::init()`
   + ROS 2 executor，一边 `gazebo::client::setup()` 起 Gazebo Classic transport 客户端线程。
   两套线程/信号系统挤在一个进程里，触发 glibc 对非递归互斥锁的自死锁断言 → `abort()`。
   上游 champ 自己在 `gazebo.launch.py` 里就留了 TODO 说这节点没修好。
2. **开关是断的**：`go2_lidar_gps.launch.py` 给 champ_bringup 传了
   `publish_foot_contacts: "false"`，但 `champ_gazebo/launch/gazebo.launch.py` 里
   `contact_sensor` 节点是**无条件启动**的，根本没接这个参数 → 上层说不要，底层照样拉起来。

另外这个节点的数据源其实也是空的：`go2_gazebo.xacro` 里 foot link 没有
`<max_contacts>`，gzserver 不会建 ContactManager，`~/physics/contacts` 永远没有消息。

**处理** 给 `champ_gazebo/launch/gazebo.launch.py` 加 `publish_foot_contacts` 参数
（默认 `false`）并给节点加 `condition`；顶层 launch 把这个开关透传下来。
需要时用 `publish_foot_contacts:=true` 强制打开（但崩溃问题仍在）。

---

### 坑 2 — `garage.world` 不可用（世界加载不出来 / 没有可视化）

**现象** gzclient 窗口只有 576×320 且空白；`spawn_entity` 30 秒后报
`Service /spawn_entity unavailable`；`controller_manager` 一直没有控制器。

**根因** `robot_scene/worlds/garage.world`：

- `include` 了 `model://garage`，但本机 `~/.gazebo/models/` 里**没有 garage 模型**
  （只有 `parking_garage` / `willowgarage`），且 `GAZEBO_MODEL_DATABASE_URI` 是空的，
  gzserver 卡在联网下载 → 世界永远加载不完 → `/spawn_entity` 服务注册不出来。
- 更要命的是它**是从无人机世界抄来的**：
  `<gravity>0 0 0</gravity>`（零重力，狗会飘）+ 引用了本机不存在的
  `librotors_gazebo_ros_interface_plugin.so`（PX4 rotors 的插件）。

**处理** 换用自洽的世界。`robot_scene/worlds/` 下这几个是自带 100×100 地面、
`<gravity>0 0 -9.8</gravity>`、零外部模型依赖的：

| world | 说明 |
|---|---|
| `indoor_walls1.world` | 20 m 墙，最轻量 ← **当前默认** |
| `indoor_50x50.world` | 50×50 室内带墙 |
| `indoor_rooms1.world` | 多房间 |

`go2_lidar_gps.launch.py` 的 `default_world_path` 已改为 `indoor_walls1.world`。

---

### 坑 3 — 没有 GUI 就没有雷达数据 ⚠️ 最重要

**现象** `gui:=false` / headless 跑时：

- `/livox/imu` 正常 ~199 Hz
- `/livox/lidar` **一条消息都没有**，且不是"空点云"，是完全没有消息

而只要开 GUI，或者手动 `gz topic -e .../scan` 订阅一下，雷达立刻开始出数据。

**根因** `mid360.xacro` 里 ray sensor **没有 `<always_on>true</always_on>`**
（IMU 那一段有，所以 IMU 一直正常）。

Gazebo Classic 里没有 `always_on` 的传感器**要等有东西订阅它的 scan 话题才会被激活**。
gzclient 因为 xacro 里的 `<visualize>true</visualize>` 会去订阅 scan 来画激光线，
**顺带把传感器激活了** —— 这就是为什么"开 GUI 就有数据"。

**为什么不能简单加 `always_on` 了事**（我试过，更糟）：

加上 `<always_on>true</always_on>` 后 gzserver 直接 **SIGSEGV（exit -11）**：

```
#0-#2  ?? ()                                  libgazebo_ode.so.11
#3     dSpaceCollide2()                       libgazebo_ode.so.11
#4     Mid360OdeMultiRayShape::UpdateRays()   libmid360_plugin.so
#5     MultiRayShape::Update()
#6     Mid360PointsPlugin::OnNewLaserScans()  libmid360_plugin.so
#7     RaySensor::UpdateImpl(bool)
#8-#11 Sensor::Update → SensorManager::SensorContainer::RunLoop（传感器线程）
```

崩在 ODE 的 `dSpaceCollide2` 内部 —— 射线空间的几何数据是坏的。
**在 5 / 20 / 40 m 三种量程下都复现，所以跟量程无关，只跟"激活时机"有关**：
过早激活（模型插入瞬间）会踩到自定义射线形状的初始化竞态。

**处理** 保持"晚激活"模式。有两条路：

1. **开 GUI**（默认）：gzclient 自己会订阅，传感器自然被激活。
2. **`headless:=True`**：launch 会**延时 50 秒后自己订阅** scan 话题来激活传感器。
   实测 RTF ~0.44（GUI 是 ~0.36），雷达 4.7 Hz（GUI 是 2.7 Hz）。见 §10。

`mid360.xacro` 里已写注释固化"不要加 `always_on`"这个约束。

> 更彻底的做法是改 `mid360_points_plugin.cpp` 做**延迟激活**（`Load()` 完成后过几秒再
> `raySensor_->SetActive(true)`），这样就不依赖外部订阅者。目前用 launch 的定时订阅
> 达到同样效果，没动插件。另外单纯在 `OnNewLaserScans()` 里加就绪守卫（已加 `ready_`）
> **不足以**解决段错误。

---

### 坑 4 — `/livox/lidar` 的 `timestamp` 字段是 CSV 行号，不是时间

**现象** 实测单帧内 `timestamp` 跨度达 **18709**，而雷达标称 10 Hz（一帧 0.1 s），物理上不可能。

**根因** `mid360_points_plugin.cpp`：

```cpp
double sim_time_sec = world->SimTime().Double();
double timestamp = sim_time_sec + rotate_info.time;   // rotate_info.time 来自 CSV 第 0 列
```

而 `scan_mode/mid360.csv` 第 0 列表头写着 `Time/s`，**实际值是 1, 2, 3 … 800000 的行号**。
插件直接拿来用，于是 `timestamp = 仿真秒 + 行号`。

好在：单帧内它**单调递增**，跨度恰好等于每帧采样数
（`samplesStep_ = point_rate/update_rate = 200000/10 = 20000`），**正好对应一个扫描周期**。

**处理** 在转换器里**归一化**，不依赖它的单位：

```cpp
new_point.time = (ts[i] - ts_min) / (ts_max - ts_min) * scan_period;   // 默认 scan_period = 0.1 s
```

配套：转换器自动检测输入是否带 `timestamp` 字段，**没有就回退到原来的伪造公式**，
所以对 `LIO_Nav2_ROS2` 原来的工程没有破坏。

实测验证：`/velodyne_points` 的 `time = 0.00000 .. 0.10000 s` 且单调递增 ✅。
配置侧 `mid360_sim.yaml` 的 `timestamp_unit: 0`（秒）不用改。

---

### 坑 5 — 雷达量程只有 5 米

**现象** 点云 `range` 最大只有 4.999 m，建不出图。

**根因** `mid360.xacro`：

```xml
<xacro:property name="laser_max_range" value="5.0"/>
```

真实 Mid-360 是 ~40 m（70 m max）。

**处理** 改为 `20.0`（当前值，够 20 m 房间用）。实测改成 20 后
`range = 1.19 .. 13.66 m`，**34% 的点超过 5 m**。

若跑更大的世界可调大到 `40.0`（注意：量程已确认与崩溃无关，但射线越远越吃性能）。

---

### 坑 6 — xacro 注释里的 `": "` 会让 launch 直接挂

**现象**

```
[ERROR] [launch]: Caught exception in launch: Unable to parse the value of
parameter robot_description as yaml.
```

**根因** `launch_ros` 会把 xacro 生成的**整个 URDF 字符串当 YAML 解析**。
xacro **会保留 XML 注释**，所以我在 `<sensor>` 里写的说明注释中有一句
`load-bearing: it makes ...`，那个**冒号+空格**让 YAML 认为这是 mapping → 解析失败。

**处理** 把注释里的 `": "` 去掉（`gui:=true` 这种冒号后接非空格是安全的）。
**改完任何 xacro 注释后，务必先验一遍**：

```bash
source install/setup.bash
xacro install/robot_scene/share/robot_scene/xacro/go2_robot_VLP.xacro > /tmp/x.urdf
python3 -c "import yaml;yaml.safe_load(open('/tmp/x.urdf').read());print('OK')"
```

---

### 坑 7 — `headless:=true` 会报 `name 'true' is not defined`

**根因** `champ_gazebo/launch/gazebo.launch.py` 里 gzclient 的条件写的是
`PythonExpression([" not ", headless])`，`true` 是 Python 里的 `True`。

**处理** 传 `headless:=True`（大写 T）。不传参数不受影响。
（另外这个 launch 里 `gui` 参数是**声明了但没用到**的，gzclient 只看 `headless`。）

---

### 坑 8 — 订阅 `/velodyne_points` 时 QoS 要对

**现象** 用 `ros2 topic echo` / `ros2 topic hz` 收不到数据，日志里有
`requesting incompatible QoS ... Last incompatible policy: RELIABILITY`。

**根因** 转换器用 `rclcpp::SensorDataQoS()` 发布（**BEST_EFFORT**）。
默认的 `ros2 topic echo/hz` 用 RELIABLE 订阅，配不上。

**处理** 探针要显式用 BEST_EFFORT：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
```

注意：Point-LIO 那边也是 `SensorDataQoS()`（BEST_EFFORT），**和转换器是匹配的**，这点没问题。

---

### 坑 9 — 定位崩溃的姿势（调试技巧）

```bash
# 1) apport 会自动抓崩溃
ls -lat /var/crash/ | head

# 2) .crash 文件里的 CoreDump 是 base64，解出来还是 gzip
awk '/^CoreDump: base64/{f=1;next} f' /var/crash/_usr_bin_gzserver-11.10.2.1000.crash \
  | base64 -d -i > /tmp/gz.core.gz
gunzip -c /tmp/gz.core.gz > /tmp/gz.core      # 会很大（这次 3.1 GB）

# 3) gdb 取回溯
gdb -batch -ex "bt 45" /usr/bin/gzserver /tmp/gz.core
```

`mid360_simulation` 是本地编译的，所以回溯里**插件那几帧有符号**
（`Mid360OdeMultiRayShape::UpdateRays` 等），这就够定位了。

---

## 7. 已知限制 / TODO

| 项 | 状态 |
|---|---|
| 雷达只有 ~4.7 Hz（标称 10 Hz） | **不要靠降 `point_rate` 解决**（会让 LIO 发散，见 §10）。headless 已是当前配置下的最好结果；再高要改插件的射线调度（分批摊到多帧），不是改采样数 |
| ~~必须开 GUI~~ **已支持 headless** | 用 `headless:=True`（launch 会延时订阅唤醒传感器）。更彻底的做法是改插件做延迟激活，见坑 3 |
| `mapping.extrinsic_est_en` | `mid360_sim.yaml` 里注释和实际值不一致（注释说 True，实际 False），目前用 False 是对的 |
| `ros2 control load_controller` 启动时偶发 exit 1 | 竞态，控制器最终是 `active` 的，不影响使用 |
| `garage.world` | 仍不可用；要用需补 garage 模型**并且**把重力改成 `0 0 -9.8` |
| `contact_sensor` | 默认关闭；`publish_foot_contacts:=true` 可打开，但崩溃问题仍在 |
| **RViz 里看不到累积地图** | Point-LIO 这个版本不持续发布地图（`pcl_wait_pub` 从未 publish，`/Laser_map` 只发一次）。只能用退出时的 PCD，或自己加周期性 publish |
| 遥控时机器人走得**明显比指令慢** | 实测发 `x=0.3 m/s` 持续 20 秒只走了约 1.1 m。开 GUI 时 RTF 仅 ~0.5，且 champ 步态在仿真里有打滑。不是 LIO 的问题（LIO 报告的是真实位移） |

---

## 8. 本工作区相对上游的改动

| 文件 | 改动 |
|---|---|
| `champ/champ_gazebo/launch/gazebo.launch.py` | 新增 `publish_foot_contacts` 参数（默认 `false`），给 `contact_sensor` 加 `condition`。**修坑 1** |
| `robot_scene/launch/go2_lidar_gps.launch.py` | **只起仿真**。`publish_foot_contacts` 改为 launch 参数并透传给 champ；默认 world 改为 `indoor_walls1.world`；**新增 `headless` / `activate_delay` 参数**（headless 时自动延时订阅 scan 话题激活传感器，RTF ~0.36 → ~0.44）。**修坑 1 / 2 / 3** |
| `robot_scene/launch/go2_lio.launch.py` | **新增，只起 LIO**：转换器 + Point-LIO + RViz。不含仿真（避免两个 Point-LIO 抢发 TF） |
| `robot_scene/xacro/mid360.xacro` | `laser_max_range` 5.0 → 20.0（`point_rate` 试过降到 50000 但会让 LIO 发散，**已回滚保持 200000**）；注释固化"别加 `always_on`""注释别写冒号""别降 point_rate"三条约束。**修坑 5 / 3 / 6 / §10** |
| `robot_scene/worlds/indoor_walls1.world` | 物理段加注释，说明为什么不能调大 `max_step_size`（实测 0.002 会让四足发散）。**§10** |
| `robot_scene/xacro/go2_gazebo.xacro` | 未改（foot link 无 `max_contacts`，是坑 1 的数据源背景） |
| `GazeboQuadbot/point_lio/` | **新迁入本工作区**（原件在 `LIO_Nav2_ROS2`）。源码逐字节未改，`config/mid360_sim.yaml` 就是验证过的那份 |
| `GazeboQuadbot/ign_sim_pointcloud_tool/` | **新迁入本工作区**（原件在 `LIO_Nav2_ROS2`），**含坑 4 的转换器修复**：改用真实逐点时间（归一化，带回退），新增 `scan_period` 参数 |
| `GazeboQuadbot/mid360_simulation/{src,include}` | `Mid360PointsPlugin` 加 `ready_` 守卫（防御性，**不足以**支持 headless，见坑 3） |
| `GazeboQuadbot/docs/quickstart.md` | **本文件** |

> 本工作区**不是 git 仓库**，改动用文件时间戳和上面的表对照。

---

## 9. 常用排查命令

```bash
# 确认 point_lio 解析到哪一份（避免 lio_slam_ws 抢走）
ros2 pkg prefix point_lio

# 确认 gzserver / gzclient 都在（缺 gzclient 就没雷达数据）
pgrep -ax gzserver; pgrep -ax gzclient

# 雷达 → 转换器链路
ros2 topic hz /livox/lidar
ros2 topic hz /velodyne_points

# LIO 是否在工作
ros2 topic hz /cloud_registered
ros2 run tf2_ros tf2_echo camera_init aft_mapped

# 检查点云字段（确认 ring / time 在）
ros2 topic echo --once /velodyne_points --field fields

# Gazebo transport 上有什么
gz topic -l | grep -i lidar
```

---

## 10. 性能与 RTF 调优（实测）

测试环境：`indoor_walls1` world，12 核，loadavg 4~6。**RTF 会随机器负载波动 ±10%**，
下面都是稳态取值（跳过启动后第一个偏高的采样）。

### 受控对比（只变一个变量）

**`point_rate = 200000`（当前默认，也是 LIO 能稳定工作的唯一配置）**

| 模式 | RTF（稳态） | `/livox/lidar` |
|---|---|---|
| 带 GUI（gzclient） | ~0.36 | 2.7 Hz |
| `headless:=True` | **~0.44** | **4.7 Hz** |

**`point_rate = 50000`（❌ 不要用，LIO 会发散）**

| 模式 | RTF（稳态） | `/livox/lidar` |
|---|---|---|
| `headless:=True` | ~0.85 | 8.5 Hz |

### 结论

**1. 射线数才是主要开销**，不是 GUI。20000 → 5000 射线/帧能把 RTF 从 ~0.44 抬到 ~0.85。
**但这条路不能走** —— 见结论 2。

**2. headless 是小幅但确实的收益**：同一射线数下 RTF ~0.36 → ~0.44（+20%），
雷达 2.7 → 4.7 Hz（+75%）。原因是不用再跑 gzclient（实测占 ~65% CPU）及其与 gzserver 的争用。

**已经做成仿真 launch 的选项**：

```bash
ros2 launch robot_scene go2_lidar_gps.launch.py headless:=True
```

`headless:=True` 时那个 launch 会自动做两件事：
- 不给 gzclient（通过 champ 的 `headless` 开关），
- **延时 `activate_delay`（默认 50 秒）后自己订阅雷达的 scan 话题**，把传感器唤醒。

（LIO 部分照常在另一个终端 `ros2 launch robot_scene go2_lio.launch.py` 起，见 §1。）

- ⚠️ **`headless` 必须写 Python 风格的 `True`/`False`**（大写）。champ 那边用
  `PythonExpression([" not ", headless])` 判断，小写 `true` 会让 launch 直接报
  `name 'true' is not defined`（坑 7）。
- ⚠️ **别把 `activate_delay` 调太小**。在启动瞬间就订阅 = "过早激活"，会让 gzserver
  在 `dSpaceCollide2` 里段错误（坑 3），这台机器上实测 50 秒是安全的。
- 话题名格式是 `/gazebo/<world>/<model>/<link>/<sensor>/scan`，launch 里用
  `gz topic -l | grep -m1 livox/scan` 自动发现，没有硬编码。

**3. 不要为了提 RTF 去降射线数**（这条我试过，是错的，已回滚）。

`point_rate` 200000 → 50000（20000 → 5000 射线/帧）确实把 RTF 抬到 ~0.85、雷达频率到 8.5 Hz，
**但 Point-LIO 每帧特征点从 ~2957 掉到 ~506**，几何约束不够，
位姿会在几分钟内发散（实测漂到 `(1248, 4964, -31498)`，几万米）。

| `point_rate` | 每帧射线 | `/velodyne_points` 点数 | LIO 每帧 `surf` 特征（中位） | 4 分钟内位姿 |
|---|---|---|---|---|
| **200000（当前）** | 20000 | ~11800 | **2957** | **稳定**（漂 0.06 m） |
| 50000 | 5000 | ~4125 | 506 | ❌ 发散到 ~3 万米 |

所以 `mid360.xacro` 保持 `point_rate = 200000`。**提性能请用 headless，不要动射线数。**
（降射线数也不影响时间戳正确性 —— 转换器按帧内 `timestamp` 跨度归一化，坑 4 —— 但 LIO 质量撑不住。）

> 判断 LIO 是否健康的快速方法：看 pointlio 的日志
> `grep -oE "surf=[0-9]+" <log> | sort -n | awk '{a[NR]=$1} END {print a[int(NR/2)]}'`
> —— 中位数应该在 **2000~3000**。掉到几百就是特征不够，位姿离发散不远了。

**4. 不要调大 `max_step_size`。** 实测 `0.001 → 0.002`（配合 `real_time_update_rate 1000 → 500`）：
RTF 几乎没变，而且**四足物理发散**（LIO 位姿飞到 `-11740`）。已回滚，
world 文件里留了注释警告。

**5. gzserver 只用 ~1.1 核**（12 核机器上没打满），说明 RTF 受**内部串行**限制
（射线计算要拿物理引擎互斥锁），不是 CPU 总量不足。
所以关掉机器上其它吃 CPU 的程序（实测 Edge 占 ~60%）帮助有限。

**6. 结论：在 LIO 能正常工作的前提下，RTF 上限就在 ~0.44（headless）。**
要真正提上去，只有改插件**减少射线调度开销**（比如把每帧 20000 条射线分批跨帧摊开），
而不是减采样数 —— 但那会改动 `mid360_points_plugin.cpp` 的扫描逻辑，属于另一件事。
