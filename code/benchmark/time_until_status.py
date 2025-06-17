#!/usr/bin/env python3

from std_msgs.msg import Bool
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import time


class TimeUntilStatusNode(CompatibleNode):

    def __init__(self):
        super().__init__(type(self).__name__)
        self.start_time = time.time_ns()
        self.loginfo(f"{type(self).__name__} started measuring...")
        self.new_subscription(
            Bool,
            "/carla/hero/status",
            self.status_callback,
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

    def status_callback(self, msg: Bool):
        if not msg.data:
            return

        end_time = time.time_ns()
        delta_time = end_time - self.start_time

        self.loginfo(
            f"Time until first status: {delta_time}ns, {delta_time/1000:.2f}us, "
            f"{delta_time/1000000:.2f}ms, "
            f"{delta_time/1000000000:.2f}s"
        )
        exit(0)


def main():
    roscomp.init(TimeUntilStatusNode.__name__)
    node = TimeUntilStatusNode()
    node.spin()


if __name__ == "__main__":
    main()
