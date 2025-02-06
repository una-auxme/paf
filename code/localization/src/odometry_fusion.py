#!/usr/bin/env python

import rospy
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl
import math
import numpy as np

from localization.cfg import OdometryCovarianceConfig
from dynamic_reconfigure.server import Server

# our car: Lincoln MKZ 2020
WHEELBASE_MKZ_2020 = 2.85
MAX_STEERING_ANGLE_MKZ_2020 = np.deg2rad(70)


class OdometryNode(CompatibleNode):
    def __init__(self):

        self.__use_odometry_yaml_covariance: bool
        self.__odometry_pose_covariance_translation: float
        self.__odometry_pose_covariance_rotation: float
        self.__odometry_twist_covariance_linear: float
        self.__odometry_twist_covariance_angular: float
        Server(OdometryCovarianceConfig, self.dynamic_reconfigure_callback)

        self.role_name: str = self.get_param("role_name", "hero")
        self.loop_rate: float = self.get_param("control_loop_rate", 0.05)

        # Node starts only if at least one steer angle
        # and one speed message was received
        self.steer_ang_init: bool = False
        self.speed_init: bool = False
        self.initialized: bool = False

        # Input buffer
        self.speed: float = 0.0
        self.steering_angle: float = 0.0

        # ROS Publishers and Subscribers
        self.odom_pub = self.new_publisher(Odometry, "/wheel/odometry", qos_profile=1)

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

    def dynamic_reconfigure_callback(self, config: "OdometryCovarianceConfig", label):
        self.__use_odometry_yaml_covariance = config["use_yaml_covariance_odometry"]
        self.__odometry_pose_covariance_translation = config[
            "odometry_pose_covariance_translation"
        ]
        self.__odometry_pose_covariance_rotation = config[
            "odometry_pose_covariance_rotation"
        ]
        self.__odometry_twist_covariance_linear = config[
            "odometry_twist_covariance_linear"
        ]
        self.__odometry_twist_covariance_angular = config[
            "odometry_twist_covariance_angular"
        ]
        return config

    def speed_callback(self, msg):
        self.speed = msg.speed

        self.speed_init = True
        if self.steer_ang_init:
            self.initialized = True

    def steering_callback(self, msg):
        # msg.steer: # [-1.0, 1.0]
        self.steering_angle = self.max_steering_angle * msg.steer

        self.steer_ang_init = True
        if self.speed_init:
            self.initialized = True

    def publish_odometry(self):
        dt = self.loop_rate
        v = self.speed

        # Calculate the change in orientation omega
        # based on velocity and turning radius
        if self.steering_angle != 0:
            turning_radius = -self.wheelbase / math.tan(self.steering_angle)
            # neg sign because of ros conversion
        else:
            turning_radius = math.inf
        omega = v / turning_radius * dt

        # Calculate hero based velocities
        vx = v * math.cos(omega)
        vy = v * math.sin(omega)

        # Create Odometry message from calculated data.
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "hero"
        odom.pose.pose.position.z = 0.0

        # These are disabled in ekf_config.yaml
        if self.__use_odometry_yaml_covariance:
            odom.pose.covariance = rospy.get_param("~pose_covariance")
        else:
            pass

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0

        odom.twist.twist.angular.z = omega

        if self.__use_odometry_yaml_covariance:
            odom.twist.covariance = rospy.get_param("~twist_covariance")
        else:
            linear = np.diag(np.full(3, self.__odometry_twist_covariance_linear))
            angular = np.diag(np.full(3, self.__odometry_twist_covariance_angular))
            cov = np.zeros((6, 6), dtype=np.float32)
            cov[:3, :3] = linear
            cov[3:, 3:] = angular
            odom.twist.covariance = list(cov.flatten())

        # Publish odometry message
        self.odom_pub.publish(odom)

    def run(self):
        # wait until Speedometer and steering angle msg are received
        while not self.initialized:
            rospy.sleep(1)
        rospy.sleep(1)

        self.new_timer(self.loop_rate, lambda _: self.publish_odometry())

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
