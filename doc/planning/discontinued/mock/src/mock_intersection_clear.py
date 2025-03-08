#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher
from std_msgs.msg import Bool


class MockIntersectionClearPublisher(CompatibleNode):
    """
    This node publishes intersection clear information. It can be used for
    testing.
    """

    def __init__(self):
        super(MockIntersectionClearPublisher, self).__init__("intersectionClearMock")

        self.control_loop_rate = self.get_param("control_loop_rate", 10)
        self.role_name = self.get_param("role_name", "ego_vehicle")
        # self.enabled = self.get_param('enabled', False)

        self.stop_sign_pub: Publisher = self.new_publisher(
            Bool, f"/paf/{self.role_name}/intersection_clear", qos_profile=1
        )
        self.delta = 0.2
        self.distance = 75.0
        self.isClear = False

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        # if not self.enabled:
        #     return
        self.loginfo("Stopsignmock node running")

        def loop(timer_event=None):
            """
            Publishes dummy intersection information
            :param timer_event: Timer event from ROS
            :return:
            """
            msg = Bool()
            msg.data = self.isClear
            self.distance -= self.delta
            if self.distance < 0.0:
                self.isClear = True
            self.stop_sign_pub.publish(msg)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("velocity_publisher_dummy", args=args)

    try:
        node = MockIntersectionClearPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
