#!/usr/bin/env python

from math import pi, cos, sin
import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
import tf
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32


class MainFramePublisher(CompatibleNode):

    def __init__(self):
        """
        This node handles the translation from the static main frame to the
        moving hero frame. The hero frame always moves and rotates as the
        ego vehicle does. The hero frame is used by sensors like the lidar.
        Rviz also uses the hero frame. The main frame is used for planning.
        """
        super(MainFramePublisher, self).__init__("main_frame_publisher")
        self.loginfo("MainFramePublisher node started")

        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.current_pos: PoseStamped = PoseStamped()
        self.current_heading: float = 0

        self.current_pos_subscriber = self.new_subscription(
            PoseStamped,
            "/paf/" + self.role_name + "/current_pos",
            self.get_current_pos,
            qos_profile=1,
        )

        self.current_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.get_current_heading,
            qos_profile=1,
        )

    def run(self):
        self.loginfo("MainFramePublisher node running")
        br = tf.TransformBroadcaster()

        def loop(timer_event=None):
            if self.current_pos is None:
                # conversion only works if pos is known
                return
            rot = -self.current_heading
            pos = [0, 0, 0]
            pos[0] = (
                cos(rot) * self.current_pos.pose.position.x
                - sin(rot) * self.current_pos.pose.position.y
            )
            pos[1] = (
                sin(rot) * self.current_pos.pose.position.x
                + cos(rot) * self.current_pos.pose.position.y
            )
            pos[2] = -self.current_pos.pose.position.z
            rot_quat = R.from_euler(
                "xyz", [0, 0, -self.current_heading + pi], degrees=False
            ).as_quat()

            br.sendTransform(
                pos,
                rot_quat,
                rospy.Time.now(),
                "global",
                "hero",
            )

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def get_current_pos(self, data: PoseStamped):
        self.current_pos = data

    def get_current_heading(self, data: Float32):
        self.current_heading = data.data


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("main_frame_publisher", args=args)

    try:
        node = MainFramePublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
