#!/usr/bin/env python

import rospy
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TransformStamped
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl
import math
import threading

# import tf2_ros

from typing import Optional


class OdometryNode(CompatibleNode):
    def __init__(self):
        # Parameters
        self.wheelbase = 2.85  # our car: Lincoln MKZ 2020
        self.turning_cicle = 11.58
        self.width = 1.86
        self.max_steering_angle = math.atan(
            self.wheelbase / ((self.turning_cicle / 2) - self.width)
        )

        self.imu: Optional[Imu] = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = rospy.Time.now()

        self.role_name = self.get_param("role_name", "hero")
        self.loop_rate = self.get_param("control_loop_rate", 0.05)

        self.steer_ang_init = False
        self.speed_init = False
        self.initialized = False

        # ROS Publishers and Subscribers
        self.odom_pub = self.new_publisher(Odometry, "/wheel/odometry", qos_profile=1)

        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.speed_sub = self.new_subscription(
            CarlaSpeedometer,
            "/carla/" + self.role_name + "/Speed",
            self.speed_callback,
            qos_profile=1,
        )
        self.steering_sub = self.new_subscription(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            self.steering_callback,
            qos_profile=1,
        )

        self.imu_sub = self.new_subscription(
            Imu, "carla/hero/IMU", self.imu_callback, qos_profile=1
        )

        # Variables to store inputs
        self.speed = 0.0
        self.steering_angle = 0.0

    def imu_callback(self, msg: Imu):
        self.imu = msg

    def speed_callback(self, msg):
        self.speed = msg.speed

        self.speed_init = True
        if self.steer_ang_init:
            self.initialized = True

    def steering_callback(self, msg):
        self.steering_angle = msg.steer  # [-1.0, 1.0]
        self.steering_angle *= self.max_steering_angle

        self.steer_ang_init = True
        if self.speed_init:
            self.initialized = True

    def publish_odometry(self):
        dt = 1 / 20.0

        # Compute velocities
        v = self.speed
        steering_angle = self.steering_angle
        omega = v * math.tan(steering_angle) / self.wheelbase

        # Update pose
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += omega * dt

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "hero"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.pose.covariance = rospy.get_param("~pose_covariance")

        # Velocity
        # odom.twist.twist.linear.x = v * math.cos(self.yaw)
        # odom.twist.twist.linear.y = v * math.sin(self.yaw)
        # odom.twist.twist.angular.z = omega
        odom.twist.covariance = rospy.get_param("~twist_covariance")

        # Publish odometry message
        self.odom_pub.publish(odom)

    def run(self):
        # wait until Speedometer and steering angle msg are received
        while not self.initialized:
            rospy.sleep(1)
        rospy.sleep(1)

        self.new_timer(1 / 20.0, lambda _: self.publish_odometry())

        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("odometry_fusion", args=args)

    try:
        node = OdometryNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
