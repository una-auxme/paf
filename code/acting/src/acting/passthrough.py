#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


from dataclasses import dataclass
from typing import Type, Dict


@dataclass
class TopicMapping:
    pub_name: str
    sub_name: str
    topic_type: Type


class Passthrough(CompatibleNode):
    """This nodes sole purpose is to pass through all messages that control needs.
    The purpose of this is that Control-Package should not have any global dependencies,
    but is only dependent on the acting package.
    """

    role_name = "hero"  # Legacy will change soon

    # Topics for velocity controller.
    target_velocity = TopicMapping(
        pub_name="/paf/acting/target_velocity",
        sub_name=f"/paf/{role_name}/target_velocity",
        topic_type=Float32,
    )
    # Topics for steering controllers
    trajectory = TopicMapping(
        pub_name="/paf/acting/trajectory",
        sub_name=f"/paf/{role_name}/trajectory",
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

    mapped_topics = [target_velocity, trajectory, position, heading]

    def __init__(self):
        self.publishers: Dict[str, Publisher] = {}
        self.subscribers: Dict[str, Subscriber] = {}
        for topic in self.mapped_topics:
            self.publishers[topic.pub_name] = self.new_publisher(
                topic.topic_type, topic.pub_name, qos_profile=1
            )

            self.subscribers[topic.pub_name] = self.new_subscription(
                topic.topic_type,
                topic.sub_name,
                callback=self.publishers[topic.pub_name].publish,
                qos_profile=1,
            )


def main(args=None):
    """Start the node.
    This is the entry point, if called by a launch file.
    """
    roscomp.init("passthrough", args=args)

    try:
        node = Passthrough()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
