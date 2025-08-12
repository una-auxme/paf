#!/usr/bin/env python

"""
This node publishes the position and heading estimated by
an Extended Kalman Filter (EKF) on the topics:
  - ekf_pos
  - ekf_heading

There are currently two EKFs from the robot_localization package running:
  - ekf_local (publishing the transformation odom->hero to the /tf topic)
  - ekf_global (publishing the transformation global->odom to the /tf topic)
These nodes provide this node with its input.
"""

from typing import List
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time, Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R
import tf2_ros
from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace


class EKFStatePublisher(Node):
    def __init__(self):
        super().__init__("ekf_state_publisher")
        self.get_logger().info(f"{type(self).__name__} node initializing...")
        # Parameters
        self.loop_rate = (
            self.declare_parameter("loop_rate", 0.05).get_parameter_value().double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        # Publishes ekf_pos and ekf_heading from hero frame out of tf-graph
        self.position_publisher: Publisher = self.create_publisher(
            PoseStamped, f"/paf/{self.role_name}/ekf_pos", qos_profile=1
        )

        self.heading_publisher: Publisher = self.create_publisher(
            Float32, f"/paf/{self.role_name}/ekf_heading", qos_profile=1
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(buffer=self.tf_buffer, node=self)

        self.create_timer(self.loop_rate, self.publish_heading_handler)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def publish_heading_handler(self, timer_event=None):
        try:
            self.publish_heading(timer_event)
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    def publish_heading(self, timer_event):
        if not self.tf_buffer.can_transform("global", "hero", Time()):
            self.get_logger().warn(
                "Transform not available yet. Waiting for transform.",
                throttle_duration_sec=2,
            )
            return
        transform: TransformStamped = self.tf_buffer.lookup_transform(
            "global",  # Target frame
            "hero",  # Source frame
            Time(),  # Get the latest available transform
            Duration(seconds=self.loop_rate),
        )  # Timeout duration
        position = PoseStamped()
        position.header.frame_id = "global"
        position.pose.orientation = transform.transform.rotation

        translation = transform.transform.translation
        position.pose.position.x = translation.x
        position.pose.position.y = translation.y
        position.pose.position.z = translation.z
        quaternion = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        rot = R.from_quat(quaternion)
        heading = rot.as_euler("xyz", degrees=False)[2]

        self.position_publisher.publish(position)
        self.heading_publisher.publish(Float32(data=heading))


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    rclpy.init(args=args)

    try:
        node = EKFStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
