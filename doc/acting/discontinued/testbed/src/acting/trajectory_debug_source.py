#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from trajectory_generator_base import TrajectoryGenerator
from lanechange_source import LanechangeGenerator
from sine_source import SineGenerator

from numpy.typing import NDArray
from typing import Dict
from rospy import Publisher


Z_VISUAL = 0


class TrajectoryDebugSource(CompatibleNode):
    """
    Is a source for various debug trajectories.
    These can be set with a rosparameter.
    The rosparam is called "trajectory_source"
    Type e.g rosparam set trajectory_source sine
    """

    def __init__(self):
        super().__init__("trajectory_debug_source")
        output_topic_name = self.get_param("output_topic_name", "paf/acting/trajectory")
        control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.default_source = self.get_param("default_source_name", "lanechange")
        self.origin_x = self.get_param("origin_x", 984.5)
        self.origin_y = self.get_param("origin_y", -5442.0)

        self.publisher: Publisher = self.new_publisher(
            msg_type=Path, topic=output_topic_name, qos_profile=10
        )

        self.generators: Dict[str, TrajectoryGenerator] = {}
        lanechange = LanechangeGenerator()
        sine = SineGenerator()
        self.generators[lanechange.name] = lanechange
        self.generators[sine.name] = sine

        self.new_timer(
            timer_period_sec=control_loop_rate, callback=self.publish_trajectory
        )

    def publish_trajectory(self, timer_event):
        source: str = self.get_param("trajectory_source", self.default_source)
        if source in self.generators.keys():
            trajectory_array: NDArray = self.generators[source].generate_trajectory(
                origin=(self.origin_x, self.origin_y)
            )
            path_msg: Path = self.__np_array_to_path(trajectory_array)

            self.publisher.publish(path_msg)

    def __np_array_to_path(self, positions: NDArray) -> Path:
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "global"

        for position in positions:
            self.__add_path_element(path_msg, position)

        return path_msg

    def __add_path_element(self, path_msg: Path, position: NDArray) -> None:
        pos = PoseStamped()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = "global"
        pos.pose.position.x = position[0]
        pos.pose.position.y = position[1]
        pos.pose.position.z = Z_VISUAL
        # currently not used therefore zeros
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 0
        path_msg.poses.append(pos)


def main(args=None):
    roscomp.init("traj_debug_source", args=args)

    try:
        node = TrajectoryDebugSource()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
