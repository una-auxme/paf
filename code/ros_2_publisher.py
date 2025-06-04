import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import rclpy.time
from std_msgs.msg import Bool
from carla_msgs.msg import CarlaEgoVehicleControl
from rosgraph_msgs.msg import Clock


class ROS2LeaderboardTestNode(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            f"/carla/hero/vehicle_control_cmd",
            qos_profile=10,
        )
        self.status_pub = self.create_publisher(
            Bool,
            f"/carla/hero/status",
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 1
        )

    def clock_callback(self, data: Clock):
        status = Bool()
        status.data = True
        self.status_pub.publish(status)
        control = CarlaEgoVehicleControl()
        control.throttle = 1.0
        control.header.stamp = rclpy.time.Time(
            seconds=data.clock.sec, nanoseconds=data.clock.nanosec
        ).to_msg()
        self.control_publisher.publish(control)


def main(args=None):
    rclpy.init(args=args)

    main_node = ROS2LeaderboardTestNode("TestNode")

    rclpy.spin(main_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
