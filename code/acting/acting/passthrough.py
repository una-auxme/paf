#!/usr/bin/env python

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


from dataclasses import dataclass
from typing import Type, Dict

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription


@dataclass
class TopicMapping:
    pub_name: str
    sub_name: str
    topic_type: Type


class Passthrough(Node):
    """This nodes sole purpose is to pass through all messages that control needs.
    The purpose of this is that Control-Package should not have any global dependencies,
    but is only dependent on the acting package.
    """

    role_name = "hero"  # Legacy will change soon

    # Topics for velocity controller.
    target_velocity = TopicMapping(
        pub_name="/paf/acting/target_velocity",
        sub_name=f"/paf/{role_name}/acc_velocity",
        topic_type=Float32,
    )

    # Publish acc_velocity as paf/hero/target_velocity
    acc_to_target_velocity = TopicMapping(
        pub_name="/paf/hero/target_velocity",
        sub_name=f"/paf/{role_name}/acc_velocity",
        topic_type=Float32,
    )

    # Topics for steering controllers
    trajectory = TopicMapping(
        pub_name="/paf/acting/trajectory_local",
        sub_name=f"/paf/{role_name}/trajectory_local",
        topic_type=Path,
    )
    position = TopicMapping(
        pub_name="/paf/acting/current_pos",
        sub_name=f"/paf/{role_name}/current_pos",
        topic_type=PoseStamped,
    )
    heading = TopicMapping(
        pub_name="/paf/acting/current_heading",
        sub_name=f"/paf/{role_name}/current_heading",
        topic_type=Float32,
    )

    mapped_topics = [
        target_velocity,
        acc_to_target_velocity,
        trajectory,
        position,
        heading,
    ]

    def __init__(self):
        super().__init__(type(self).__name__)

        self.pt_publishers: Dict[str, Publisher] = {}
        self.pt_subscribers: Dict[str, Subscription] = {}
        for topic in self.mapped_topics:
            self.pt_publishers[topic.pub_name] = self.create_publisher(
                topic.topic_type, topic.pub_name, qos_profile=1
            )

            self.pt_subscribers[topic.pub_name] = self.create_subscription(
                topic.topic_type,
                topic.sub_name,
                callback=self.pt_publishers[topic.pub_name].publish,
                qos_profile=1,
            )


def main(args=None):
    """Start the node.
    This is the entry point, if called by a launch file.
    """
    rclpy.init(args=args)

    try:
        node = Passthrough()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
