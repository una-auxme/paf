#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from abc import ABC, abstractmethod

from numpy.typing import NDArray


class TrajectoryModifier(ABC):
    """
    Baseclass for a trajectory modifier.
    Two parameters specifie the topic names of input and output.
    If a trajectory on input gets received a callback gets triggered expecting
    a trajectory in return. This trajectory then gets sent to output trajectory.
    """

    def __init__(self, node: CompatibleNode):
        input_topic_name = node.get_param("input_topic_name", "paf/hero/trajectory")
        output_topic_name = node.get_param("output_topic_name", "paf/acting/trajectory")

        self.subscriber: Subscriber = node.new_subscription(
            msg_type=Path,
            topic=input_topic_name,
            callback=self._input_callback,
            qos_profile=10,
        )
        self.publisher: Publisher = node.new_publisher(
            msg_type=Path, topic=output_topic_name, qos_profile=10
        )

    def _input_callback(self, path: Path) -> None:
        positions = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses]
        )
        positions = self.modify_path(positions)
        path.poses.clear()
        for position in positions:
            pose = PoseStamped()
            pose.header.frame_id = "global"
            pose.pose.position.x, pose.pose.position.y = position
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.publisher.publish(path)

    @abstractmethod
    def modify_path(self, positions: NDArray) -> NDArray:
        roscomp.logerr("There must be a modify_path definition in your class.")
        return positions
