"""Run the Gazebo go2 sim and a LiDAR-inertial odometry (Point-LIO) mapping pipeline.

Data flow:
    Gazebo mid360 plugin            -> /livox/lidar   (PointCloud2: x,y,z,intensity,timestamp)
    ign_sim_pointcloud_tool         -> /velodyne_points (PointXYZIRT: ...+ring,time)
    point_lio (pointlio_mapping)    -> /cloud_registered, /Laser_map, TF camera_init->aft_mapped

Requires these workspaces to be sourced (the converter and point_lio live there):
    /home/ros/ros_ws/LIO_Nav2_ROS2/install/setup.bash

Note: the /livox/lidar `timestamp` field is NOT a physical time. mid360_points_plugin.cpp fills it
with `SimTime() + mid360.csv column 0`, and that CSV column holds a row index (1..800000) despite
its "Time/s" header. Its intra-scan spread does correspond to exactly one scan period, so the
converter normalises it to [0, scan_period] seconds rather than trusting the unit.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_scene_share = get_package_share_directory("robot_scene")
    point_lio_share = get_package_share_directory("point_lio")

    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="Also start the Gazebo go2 simulation (false = reuse one already running)",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Start the Gazebo GUI (gzclient)"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start RViz to watch the map"
    )
    declare_point_lio_cfg = DeclareLaunchArgument(
        "point_lio_cfg",
        default_value=os.path.join(point_lio_share, "config", "mid360_sim.yaml"),
        description="Point-LIO parameter file (expects lidar_type=2 on velodyne_points)",
    )

    # 1) Gazebo go2 sim: the LiDAR/IMU source.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_scene_share, "launch", "go2_lidar_gps.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("sim")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": "false",
        }.items(),
    )

    # 2) Bridge to Velodyne layout: adds the `ring` and `time` fields Point-LIO's
    #    velodyne_handler requires.
    converter = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="point_cloud_converter",
        output="screen",
        parameters=[
            {
                "pcd_topic": "/livox/lidar",
                "n_scan": 50,
                "horizon_scan": 360,
                "ang_bottom": 7.22,
                "ang_res_y": 1.248,
                "scan_period": 0.1,
            }
        ],
    )

    # 3) Point-LIO. Publishes /cloud_registered, /Laser_map and TF camera_init -> aft_mapped.
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        parameters=[LaunchConfiguration("point_lio_cfg")],
    )

    # 4) RViz, preconfigured for Point-LIO.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
        arguments=[
            "-d",
            os.path.join(point_lio_share, "rviz_cfg", "pointlio_robosense.rviz"),
        ],
    )

    return LaunchDescription(
        [
            declare_sim,
            declare_gui,
            declare_rviz,
            declare_point_lio_cfg,
            sim,
            converter,
            point_lio,
            rviz,
        ]
    )
