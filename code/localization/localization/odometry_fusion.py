#!/usr/bin/env python

"""
This node creates an Odometry message for the local and global EKFs.
The message is calculated using the following data:
  - the current velocity (topic: /carla/hero/Speed)
  - the current steering angle (topic: /carla/hero/vehicle_control_command)
"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl
import math
import numpy as np
from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue


# our car: Lincoln MKZ 2020
WHEELBASE_MKZ_2020 = 2.85
MAX_STEERING_ANGLE_MKZ_2020 = np.deg2rad(70)


class OdometryNode(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters
        self.control_loop_rate = (
            self.declare_parameter("loop_rate", 0.05).get_parameter_value().double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        self.pose_covariance = (
            self.declare_parameter(
                "pose_covariance",
                ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
                descriptor=ParameterDescriptor(
                    description="Covariance for Odometry Pose",
                ),
            )
            .get_parameter_value()
            .double_array_value
        )
        self.twist_covariance = (
            self.declare_parameter(
                "twist_covariance",
                ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
                descriptor=ParameterDescriptor(
                    description="Covariance for Odometry Twist",
                ),
            )
            .get_parameter_value()
            .double_array_value
        )

        # Node starts only if at least one steer angle
        # and one speed message was received
        self.steer_ang_init: bool = False
        self.speed_init: bool = False
        self.initialized: bool = False

        # Input buffer
        self.speed: float = 0.0  # Speed in meters/second
        self.steering_angle: float = 0.0  # Steering angle in radians

        # ROS Publishers and Subscribers
        self.odom_pub = self.create_publisher(
            Odometry, "/wheel/odometry", qos_profile=1
        )

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer,
            "/carla/" + self.role_name + "/Speed",
            self.speed_callback,
            qos_profile=1,
        )
        self.steering_sub = self.create_subscription(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            self.steering_callback,
            qos_profile=1,
        )

        self.create_timer(self.control_loop_rate, self.publish_odometry_handler)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def speed_callback(self, msg: CarlaSpeedometer):
        """Saves Carlas reported speed in a buffer.

        Callback to /carla/hero/Speed topic.

        Args:
            msg (CarlaSpeedometer): The Carla Speed message.
        """
        self.speed = msg.speed

        self.speed_init = True
        if self.steer_ang_init:
            self.initialized = True

    def steering_callback(self, msg: CarlaEgoVehicleControl):
        """Saves the steering angle we sent to Carla in a buffer.

        The steering input is in the field .steer of the message.
        It is in the range between [-1.0, 1.0].
        We have to know our vehicle in order to calculate the true angle.

        Args:
            msg (CarlaEgoVehicleControl): The vehicle info message we receive.
        """
        self.steering_angle = MAX_STEERING_ANGLE_MKZ_2020 * msg.steer

        self.steer_ang_init = True
        if self.speed_init:
            self.initialized = True

    def publish_odometry(self):
        """Calculate and publish odometry data from our buffered values.

        This gets called with our loop rate.
        It uses the buffered messages, calculates the odometry and publishes
        this data to our output topic.
        """

        dt = self.control_loop_rate
        velocity = self.speed
        steering_angle = -self.steering_angle

        # Calculate the change in orientation omega
        # based on velocity and turning radius
        if self.steering_angle != 0:
            turning_radius = WHEELBASE_MKZ_2020 / math.tan(steering_angle)
            # neg sign because of ros conversion
        else:
            turning_radius = math.inf
        omega = velocity / turning_radius * dt

        # Calculate hero based velocities
        vx = velocity * math.cos(steering_angle)
        vy = velocity * math.sin(steering_angle)

        # Create Odometry message from calculated data.
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.role_name
        odom.pose.pose.position.z = 0.0

        # The pose message is disabled in ekf_config.yaml
        odom.pose.covariance = self.pose_covariance

        # The velocity (twist) must be set. (Angular x, y are also disabled.)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.z = omega

        odom.twist.covariance = self.twist_covariance

        # Publish Odometry message
        self.odom_pub.publish(odom)

    def publish_odometry_handler(self):
        if not self.initialized:
            return

        try:
            self.publish_odometry()
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    rclpy.init(args=args)

    try:
        node = OdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
