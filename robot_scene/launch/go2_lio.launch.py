"""LiDAR-inertial odometry (Point-LIO) mapping stack for the go2 sim.

This launch starts ONLY the mapping side - it does not start Gazebo. Run the simulation
separately, in its own terminal, so the two can be started, stopped and restarted
independently:

    # terminal 1: Gazebo (GUI, or add headless:=True for no GUI)
    ros2 launch robot_scene go2_lidar_gps.launch.py

    # terminal 2: mapping + RViz
    ros2 launch robot_scene go2_lio.launch.py

Data flow (all behind /livox/*, published by the sim):

    /livox/lidar        PointCloud2 (x,y,z,intensity,timestamp)
        -> ign_sim_pointcloud_tool -> /velodyne_points (PointXYZIRT: ... + ring, time)
        -> point_lio               -> /cloud_registered, TF camera_init->aft_mapped

Everything it needs lives in this workspace (point_lio, ign_sim_pointcloud_tool), so:

    source /home/ros/ros_ws/go2_sim_ws/install/setup.bash

RViz opens with the Point-LIO config (Fixed Frame `camera_init`). Note that RViz shows the
*current* scan, not an accumulating map - this Point-LIO never publishes an accumulated map
(`pcl_wait_pub` is never published); the map only lands in
`point_lio/PCD/scans.pcd` when you Ctrl-C this launch. See docs/quickstart.md.

Do NOT also run a launch that starts the mapping stack - two Point-LIO instances publish
conflicting TF on camera_init->aft_mapped and RViz shows garbage.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    point_lio_share = get_package_share_directory("point_lio")

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start RViz to watch the map"
    )
    declare_point_lio_cfg = DeclareLaunchArgument(
        "point_lio_cfg",
        default_value=os.path.join(point_lio_share, "config", "mid360_sim.yaml"),
        description="Point-LIO parameter file (expects lidar_type=2 on velodyne_points)",
    )

    # Bridge to Velodyne layout: adds the `ring` and `time` fields Point-LIO's
    # velodyne_handler requires. It also fixes up the sim's bogus per-point `timestamp`
    # (mid360_points_plugin.cpp fills it with SimTime() + a CSV row index, not a time) by
    # normalising the intra-scan spread to [0, scan_period] seconds.
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

    # Point-LIO. Publishes /cloud_registered, /cloud_registered_body and TF
    # camera_init -> aft_mapped.
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        parameters=[LaunchConfiguration("point_lio_cfg")],
    )

    # RViz, preconfigured for Point-LIO.
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
            declare_rviz,
            declare_point_lio_cfg,
            converter,
            point_lio,
            rviz,
        ]
    )
