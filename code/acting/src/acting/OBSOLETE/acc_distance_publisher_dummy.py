#!/usr/bin/env python
import random

import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32


class AccDistancePublisherDummy(CompatibleNode):
    """
    This node dummy distances. It can be used for testing the acc.
    """

    def __init__(self):
        super(AccDistancePublisherDummy, self). \
            __init__('acc_distance_publisher_dummy')

        self.control_loop_rate = 0.05
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.enabled = self.get_param('enabled', False)

        self.distance_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/acc_distance",
            qos_profile=1)

        self.speed_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_speed,
            qos_profile=1)

        self.velocity_car_ahead = 0  # = 36 km/h
        self.distance = 10
        self.velocity = 0

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        if not self.enabled:
            return
        self.loginfo('AccDistancePublisherDummy node running')

        def loop(timer_event=None):
            """
            Publishes dummy velocity
            :param timer_event: Timer event from ROS
            :return:
            """
            max_delta_v = 1
            self.velocity_car_ahead = \
                self.velocity_car_ahead + \
                random.uniform(-max_delta_v, max_delta_v)
            self.velocity_car_ahead = max(self.velocity_car_ahead, 0)
            self.velocity_car_ahead = min(self.velocity_car_ahead, 50)

            self.distance = \
                self.distance + \
                self.velocity_car_ahead * self.control_loop_rate
            self.distance = \
                self.distance - \
                self.velocity * self.control_loop_rate
            if self.distance > 200:
                self.logdebug("AccDistancePublisherDummy: "
                              "You lost the car in front")
                self.distance_pub.publish(-1)
            else:
                self.distance_pub.publish(self.distance)
                self.loginfo("AccDistancePublisherDummy published: d = " + str(
                    self.distance) + "; v_car_in_front = " + str(
                    self.velocity_car_ahead))

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __set_speed(self, data: CarlaSpeedometer):
        self.velocity = data.speed


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('acc_distance_publisher_dummy', args=args)

    try:
        node = AccDistancePublisherDummy()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
