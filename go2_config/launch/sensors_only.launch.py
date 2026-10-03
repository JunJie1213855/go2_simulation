import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Minimal launch: Gazebo world + Go2 + sensor data, no control/odometry.

    This is the same Gazebo + spawn + robot_state_publisher stack that
    gazebo.launch.py uses, but WITHOUT champ_bringup (quadruped_controller,
    state_estimation, EKFs) and WITHOUT ground_truth_odom. The lidar / RGB-D /
    IMU data comes from the Gazebo plugins inside the URDF, so none of those
    nodes are needed for the sensors to publish.
    """
    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")

    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/playground.world")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_world = DeclareLaunchArgument("world", default_value=default_world_path)
    declare_gui = DeclareLaunchArgument("gui", default_value="true")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")

    # Same as gazebo.launch.py: gui:=false is an alias for headless:=true, and the
    # value forwarded to champ_gazebo must be the Python literal True/False.
    headless = PythonExpression([
        "'True' if '", LaunchConfiguration("headless"),
        "'.lower() == 'true' or '", LaunchConfiguration("gui"),
        "'.lower() == 'false' else 'False'",
    ])

    # robot_state_publisher: publishes /robot_description (used by spawn_entity)
    # and the TF tree, including the fixed sensor frames
    # trunk -> depth_camera_link / livox_mid360 / imu_link.
    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_description"),
                "launch",
                "description.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "description_path": default_model_path,
        }.items(),
    )

    # Gazebo: gzserver + gzclient + spawn_entity + joint controllers.
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
            "robot_name": "go2",
            "world": LaunchConfiguration("world"),
            "headless": headless,
            "publish_foot_contacts": "false",
        }.items(),
    )

    # Hold the neutral pose so the robot stays upright (the walking controller
    # that normally does this is not launched here).
    stand = Node(
        package="go2_config",
        executable="stand.py",
        name="stand",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_gui,
        declare_headless,
        description_ld,
        gazebo_ld,
        stand,
    ])
