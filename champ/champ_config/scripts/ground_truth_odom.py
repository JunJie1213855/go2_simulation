#!/usr/bin/env python3
#
# Ground-truth odometry relay for CHAMP simulation.
#
# In Gazebo, the `p3d_base_controller` plugin publishes the exact robot pose as
# /odom/ground_truth (world -> base_link). This node converts that into the
# odom -> base_footprint transform that slam_toolbox / Nav2 expect, replacing
# the foot-contact odometry (which is unreliable in simulation).
#
# It publishes:
#   - TF:            odom -> base_footprint (x, y, yaw only; z/roll/pitch = 0)
#   - Odometry:      /odom (frame_id=odom, child_frame_id=base_footprint)
#
# Launch this INSTEAD of footprint_to_odom_ekf when close_loop_odom is true.

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__("ground_truth_odom")

        self.broadcaster_ = TransformBroadcaster(self)
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)
        self.sub_ = self.create_subscription(
            Odometry, "odom/ground_truth", self.callback_, 10
        )

        self.prev_x_ = None
        self.prev_y_ = None
        self.prev_yaw_ = None
        self.prev_time_ = None

        self.get_logger().info("Ground-truth odometry relay started")

    @staticmethod
    def yaw_from_quaternion_(q):
        # ZYX yaw (rotation about the world Z axis).
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def callback_(self, msg):
        q = msg.pose.pose.orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw_from_quaternion_(q)

        stamp = msg.header.stamp
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        # Publish the odom -> base_footprint transform (2D only).
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster_.sendTransform(tf)

        # Publish the matching /odom message.
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Body-frame twist from the pose difference.
        if self.prev_x_ is not None and self.prev_time_ is not None:
            dt = t_sec - self.prev_time_
            if dt > 0.0:
                dx = x - self.prev_x_
                dy = y - self.prev_y_
                dyaw = yaw - self.prev_yaw_
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))

                odom.twist.twist.linear.x = (dx * math.cos(yaw) + dy * math.sin(yaw)) / dt
                odom.twist.twist.linear.y = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / dt
                odom.twist.twist.angular.z = dyaw / dt

        self.prev_x_ = x
        self.prev_y_ = y
        self.prev_yaw_ = yaw
        self.prev_time_ = t_sec

        self.odom_pub_.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
