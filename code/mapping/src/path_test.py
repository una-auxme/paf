#!/usr/bin/env python
import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
from nav_msgs.msg import Path
from mapping.msg import Map as MapMsg
from mapping_common.map import Map
import shapely


class TestPath(CompatibleNode):

    def __init__(self):
        """
        This node handles the translation from the static main frame to the
        moving hero frame. The hero frame always moves and rotates as the
        ego vehicle does. The hero frame is used by sensors like the lidar.
        Rviz also uses the hero frame. The main frame is used for planning.
        """
        super(TestPath, self).__init__("TestPath")
        self.loginfo("TestPath node started")
        self.local_trajectory = Path()

        self.publisher = self.new_publisher(
            msg_type=Path,
            topic="test_trajectory",
            qos_profile=1,
        )

        self.current_pos_subscriber = self.new_subscription(
            Path,
            "/test_trajectory",
            self.callback,
            qos_profile=1,
        )

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self.map_callback,
            qos_profile=1,
        )

    def run(self):
        self.loginfo("TestPath node running")

        def loop(timer_event=None):
            pose_stamps = []
            for i in range(0, 10):
                temp = PoseStamped()
                temp.pose.position.x = i / 2
                temp.pose.position.y = i * 0.2
                pose_stamps.append(temp)

            msg = Path()
            msg.poses = pose_stamps
            msg.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
            self.publisher.publish(msg)

        self.new_timer(5, loop)
        self.spin()

    def callback(self, data: Path):
        self.local_trajectory = data

    def map_callback(self, data: MapMsg):
        map = Map.from_ros_msg(data)
        status = map.check_trajectory(self.local_trajectory)
        self.loginfo("#Status trajectory check " + str(status))


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("main_frame_publisher", args=args)

    try:
        node = TestPath()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
