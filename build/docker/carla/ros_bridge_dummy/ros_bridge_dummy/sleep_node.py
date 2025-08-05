import rclpy
from rclpy.node import Node


class SleepNode(Node):

    def __init__(self):
        super().__init__("sleep_node")
        timer_period = 20.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    sleep_node = SleepNode()

    rclpy.spin(sleep_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
