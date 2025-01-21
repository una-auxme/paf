#!/usr/bin/env python

import rospy
from ros_compatibility.node import CompatibleNode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl
import tf2_ros
import math


class OdometryNode(CompatibleNode):
    def __init__(self):
        # Parameters
        self.wheelbase = 2.85  # our car: Lincoln MKZ 2020
        self.turning_cicle = 11.58
        self.width = 1.86
        self.max_steering_angle = math.atan(
            self.wheelbase / ((self.turning_cicle / 2) - self.width)
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = rospy.Time.now()

        self.role_name = self.get_param("role_name", "hero")

        # ROS Publishers and Subscribers
        self.odom_pub = self.new_publisher(Odometry, "/wheel/odometry", qos_profile=1)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

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

        # Variables to store inputs
        self.speed = 0.0
        self.steering_angle = 0.0

    def speed_callback(self, msg):
        self.speed = msg.speed

    def steering_callback(self, msg):
        self.steering_angle = msg.steer  # [-1.0, 1.0]
        self.steering_angle *= self.max_steering_angle

    def publish_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

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
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Publish odometry message
        self.odom_pub.publish(odom)

        # Publish TF
        tf = TransformStamped()
        tf.header.stamp = current_time
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    def spin(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("odometry_node")
    node = OdometryNode()
    rospy.loginfo("Odometry node started")
    node.spin()
