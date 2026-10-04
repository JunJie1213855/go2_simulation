import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    # use_sim_time = LaunchConfiguration("use_sim_time")
    # description_path = LaunchConfiguration("description_path")
    # base_frame = "base_link"

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    resource_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="robot_scene"
    ).find("robot_scene")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "/config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(resource_pkg_share, "xacro/go2_robot_VLP.xacro")
    # garage.world is unusable here: it includes model://garage (not installed locally, no
    # GAZEBO_MODEL_DATABASE_URI -> gzserver stalls trying to download it) and it is a drone
    # world copied from rotors_simulator, with <gravity>0 0 0</gravity> and a
    # librotors_gazebo_ros_interface_plugin.so that does not exist. gzserver never finishes
    # loading the world, so /spawn_entity is never advertised and the robot never spawns.
    # indoor_walls1.world is self-contained (inline 100x100 ground, gravity -9.8, no model:// deps).
    default_world_path = os.path.join(resource_pkg_share, "worlds/indoor_walls1.world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts",
        default_value="false",
        description="Publish foot contacts (champ contact_sensor; known unstable, off by default)",
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description=(
            "Do not start gzclient (measurably faster). MUST be Python-style True/False - "
            "champ gates gzclient with PythonExpression 'not headless'. When True this "
            "launch also activates the lidar sensor itself, see activate_delay."
        ),
    )
    declare_activate_delay = DeclareLaunchArgument(
        "activate_delay",
        default_value="50.0",
        description=(
            "Seconds to wait before subscribing to the lidar scan topic, which is what "
            "activates the sensor (only used when headless:=True). Do not lower it much: "
            "activating the sensor before the robot has settled segfaults gzserver."
        ),
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": LaunchConfiguration("publish_foot_contacts"),
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "headless": LaunchConfiguration("headless"),
            "close_loop_odom": "true",
            "publish_foot_contacts": LaunchConfiguration("publish_foot_contacts"),
        }.items(),
    )

    # Headless has no gzclient, and gzclient is what normally activates the mid360 ray sensor:
    # its SDF has no <always_on>, so Gazebo only starts updating it once something subscribes
    # to its scan topic - which gzclient does because <visualize> is true. Without that
    # subscription /livox/lidar stays completely silent (not empty, absent), so subscribe
    # ourselves. Delayed on purpose: activating the sensor while the robot is still spawning
    # and settling segfaults gzserver inside dSpaceCollide2. The robot spawns ~15 s in and the
    # plugin spends ~6 s parsing an 800k-row CSV, so 50 s is comfortably past that.
    # `gz topic -e` is used only for its subscription side effect; its output is enormous and
    # is discarded. The topic is /gazebo/<world>/<model>/<link>/<sensor>/scan, so discover it
    # rather than hardcoding.
    activate_lidar_sensor = TimerAction(
        period=LaunchConfiguration("activate_delay"),
        condition=IfCondition(LaunchConfiguration("headless")),
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "t=$(gz topic -l 2>/dev/null | grep -m1 livox/scan); "
                    "if [ -z \"$t\" ]; then "
                    "  echo '[go2_sim] ERROR: no livox/scan topic; lidar will stay silent'; "
                    "  exit 1; "
                    "fi; "
                    "echo \"[go2_sim] activating lidar sensor via $t\"; "
                    "exec gz topic -e \"$t\" > /dev/null",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_publish_foot_contacts,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_headless,
            declare_activate_delay,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            bringup_ld,
            gazebo_ld,
            activate_lidar_sensor,
        ]
    )
