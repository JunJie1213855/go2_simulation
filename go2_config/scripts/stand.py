#!/usr/bin/env python3
"""Hold the Go2 in its neutral (standing) pose.

`sensors_only.launch.py` deliberately does not launch the champ walking
controller, so nothing commands the ``joint_group_effort_controller``. Without
a goal the effort controller outputs zero effort and the robot collapses under
gravity. This node periodically publishes the neutral pose so the controller
keeps the robot upright while the sensors stream.
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Joint order must match joint_group_effort_controller's `joints` list in
# go2_description/config/ros_control/ros_control.yaml.
JOINTS = [
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
]


class StandNode(Node):
    def __init__(self):
        super().__init__("stand")
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10
        )
        self.timer = self.create_timer(0.5, self.publish_stand)

    def publish_stand(self):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(JOINTS)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
        msg.points = [point]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
