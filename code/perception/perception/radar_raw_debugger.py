import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

"This file can be used to output the raw data from the radar sensors."
"To execute, this file must be run in a terminal while the simulation is running."


class RadarDump(Node):
    def __init__(self):
        super().__init__("radar_dump")
        topic = "/carla/hero/RADAR0"  # Radar whose data is to be displayed
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, 10)
        self.get_logger().info(f"Listening on {topic}")

    def cb(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        self.get_logger().info(
        (
            f"Fields: {field_names} | "
            f"width={msg.width} "
            f"height={msg.height} "
            f"point_step={msg.point_step}"
        )
    )

        # first 10 points
        i = 0
        for p in point_cloud2.read_points(msg, field_names=field_names, skip_nans=True):
            self.get_logger().info(f"P{i}: {p}")
            i += 1
            if i >= 10:
                break


def main():
    rclpy.init()
    node = RadarDump()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
