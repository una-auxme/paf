"""Coordinate deterministic startup readiness before unblocking CARLA."""

from __future__ import annotations

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from paf_common.sync import StartupReadinessTracker, normalize_sync_id, startup_topic


DEFAULT_REQUIRED_NODES = [
    # Route-dependent nodes become ready only after the leaderboard publishes
    # the global plan, which happens after /carla/<role>/status is released.
    "data_management",
]


class StartupCoordinator(Node):
    """Wait for the required startup nodes before publishing CARLA status."""

    def __init__(self):
        super().__init__("startup_coordinator")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        self.role_name = self.declare_parameter("role_name", "hero").value
        required_nodes = self.declare_parameter(
            "required_nodes", DEFAULT_REQUIRED_NODES
        ).value
        self.tracker = StartupReadinessTracker(required_nodes)
        self._published_ready_once = False

        self.status_pub = self.create_publisher(
            Bool,
            f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        startup_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for node_id in self.tracker.required_nodes:
            self.create_subscription(
                Bool,
                startup_topic(self.role_name, node_id),
                lambda msg, current_node_id=node_id: self._startup_callback(
                    current_node_id, msg
                ),
                startup_qos,
            )

        system_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME)
        self.create_timer(0.5, self._republish_status_if_ready, clock=system_clock)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _startup_callback(self, node_id: str, msg: Bool) -> None:
        normalized = normalize_sync_id(node_id)
        self.tracker.update(normalized, msg.data)
        if self.tracker.all_ready() and not self._published_ready_once:
            self.get_logger().info(
                "All required startup nodes are ready. Releasing CARLA status."
            )
            self.status_pub.publish(Bool(data=True))
            self._published_ready_once = True

    def _republish_status_if_ready(self) -> None:
        if self.tracker.all_ready():
            self.status_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)

    try:
        node = StartupCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
