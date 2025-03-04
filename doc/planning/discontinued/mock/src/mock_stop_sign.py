#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher

# from std_msgs.msg import Float32
from mock.msg import Stop_sign


class MockStopSignPublisher(CompatibleNode):
    """
    This node publishes stop sign light information. It can be used for
    testing.
    """

    def __init__(self):
        super(MockStopSignPublisher, self).__init__("stopSignMock")

        self.control_loop_rate = self.get_param("control_loop_rate", 10)
        self.role_name = self.get_param("role_name", "ego_vehicle")
        # self.enabled = self.get_param('enabled', False)

        self.stop_sign_pub: Publisher = self.new_publisher(
            Stop_sign, f"/paf/{self.role_name}/stop_sign", qos_profile=1
        )
        self.delta = 0.2
        self.distance = 20.0
        self.isStop = False

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
            Publishes dummy stop signs
            :param timer_event: Timer event from ROS
            :return:
            """
            msg = Stop_sign()
            msg.isStop = self.isStop
            self.distance -= self.delta
            if self.distance < 0.0:
                self.isStop = True
                self.distance = 20.0
            msg.distance = self.distance
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
        node = MockStopSignPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
