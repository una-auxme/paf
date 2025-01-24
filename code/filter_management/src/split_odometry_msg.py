#!/usr/bin/env python

import rospy
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl
import math
import threading

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, UInt32
from coordinate_transformation import quat_to_heading


class SplitOdometryNode(CompatibleNode):
    def __init__(self):
        # Parameters
        self.role_name = self.get_param("role_name", "hero")
        self.loop_rate = self.get_param("control_loop_rate", 0.05)

        self.publish_seq = UInt32(0)
        self.frame_id = "map"

        # ROS Publishers and Subscribers
        self.pos_pub = self.new_publisher(
            PoseStamped, "/paf/" + self.role_name + "/ekf_pos", qos_profile=1
        )
        # Initialize the publisher for the kalman-heading
        self.heading_pub = self.new_publisher(
            Float32, "/paf/" + self.role_name + "/ekf_heading", qos_profile=1
        )

        self.odom_sub = self.new_subscription(
            Odometry,
            "odometry/filtered_map",
            self.odom_callback,
            qos_profile=1,
        )

    def odom_callback(self, msg):
        ekf_pos = PoseStamped()

        # Fill the EKF position
        ekf_pos.header.frame_id = self.frame_id
        ekf_pos.header.stamp = rospy.Time.now()
        ekf_pos.header.seq = self.publish_seq

        self.publish_seq.data += 1

        ekf_pos.pose.position.x = msg.pose.pose.position.x
        ekf_pos.pose.position.y = msg.pose.pose.position.y
        ekf_pos.pose.position.z = msg.pose.pose.position.z

        ekf_pos.pose.orientation.x = 0
        ekf_pos.pose.orientation.y = 0
        ekf_pos.pose.orientation.z = 1
        ekf_pos.pose.orientation.w = 0

        # Publish the EKF position
        self.pos_pub.publish(ekf_pos)

        ekf_heading = Float32()

        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w

        quaternion = [quat_x, quat_y, quat_z, quat_w]

        ekf_heading.data = quat_to_heading(quaternion)

        self.heading_pub.publish(ekf_heading)

    def run(self):
        self.loginfo("SplitOdometry node started its loop!")

        def loop():
            while True:
                rospy.sleep(self.loop_rate)

        threading.Thread(target=loop).start()
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("split_odometry_node", args=args)

    try:
        node = SplitOdometryNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
