#!/usr/bin/env python
# import BehaviourEnum
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber, Publisher
import rospy
from planning.srv import RequestBehaviourChange, RequestBehaviourChangeResponse
from std_msgs.msg import String, Int8


class RequestBehaviourChangeService(CompatibleNode):
    def __init__(self):
        super(RequestBehaviourChangeService, self).__init__(
            "RequestBehaviourChangeService"
        )
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.__curr_behavior = None

        self.service = rospy.Service(
            "RequestBehaviourChange",
            RequestBehaviourChange,
            self.handle_request_behaviour_change,
        )

        self.behaviour_pub: Publisher = self.new_publisher(
            Int8,
            f"/paf/{self.role_name}/behaviour_request",
            qos_profile=1,
        )

        self.curr_behavior_sub: Subscriber = self.new_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )

        self.behaviour_pub.publish(0)
        rospy.spin()

    def __set_curr_behavior(self, data: String):
        """
        Sets the received current behavior of the vehicle.
        """
        self.__curr_behavior = data.data

    def handle_request_behaviour_change(self, req):
        if (
            self.__curr_behavior == "us_unstuck"
            or self.__curr_behavior == "us_stop"
            or self.__curr_behavior == "us_overtake"
            or self.__curr_behavior == "Cruise"
        ):
            self.behaviour_pub.publish(req.request)
            return RequestBehaviourChangeResponse(True)
        else:
            return RequestBehaviourChangeResponse(False)

    def run(self):
        """
        Control loop

        :return:
        """

        self.spin()


if __name__ == "__main__":
    """
    main function starts the RequestBehaviourChangeService node
    :param args:
    """
    roscomp.init("RequestBehaviourChangeService")
    try:
        node = RequestBehaviourChangeService()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
