#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher

# from std_msgs.msg import Float32
from mock.msg import Traffic_light


class MockTrafficLightPublisher(CompatibleNode):
    """
    This node publishes traffic light information. It can be used for testing.
    """

    def __init__(self):
        super(MockTrafficLightPublisher, self).__init__("trafficLightMock")

        self.control_loop_rate = self.get_param("control_loop_rate", 10)
        self.role_name = self.get_param("role_name", "ego_vehicle")
        # self.enabled = self.get_param('enabled', False)

        self.traffic_light_pub: Publisher = self.new_publisher(
            Traffic_light, f"/paf/{self.role_name}/traffic_light", qos_profile=1
        )
        self.delta = 0.2
        self.distance = 20.0
        self.color = "green"

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        # if not self.enabled:
        #     return
        self.loginfo("TrafficLightmock node running")

        def loop(timer_event=None):
            """
            Publishes dummy velocity
            :param timer_event: Timer event from ROS
            :return:
            """
            msg = Traffic_light()
            msg.color = self.color
            self.distance -= self.delta
            if self.distance < 0.0:
                if self.color == "green":
                    self.color = "yellow"
                elif self.color == "yellow":
                    self.color = "red"
                elif self.color == "red":
                    self.color = "green"
                self.distance = 20.0
            msg.distance = self.distance
            self.traffic_light_pub.publish(msg)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("traffic_light_publisher_dummy", args=args)

    try:
        node = MockTrafficLightPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
