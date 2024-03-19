#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaRoute

from perception.msg import Waypoint, LaneChange

import math
# import rospy


class GlobalPlanDistance(CompatibleNode):
    """
    Creates a node that publishes the distance to the next waypoint and the
    stop line of an intersection specified by the global waypoint in front of
    it.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """

        super(GlobalPlanDistance, self).__init__('global_plan_distance'
                                                 '_publisher')
        self.loginfo("GlobalPlanDistance node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")
        self.publish_loop_rate = 0.05  # 20Hz rate like the sensors

        self.current_pos = PoseStamped()
        self.global_route = None
        self.road_options = None

        # Subscriber
        self.pos_subscriber = self.new_subscription(
            PoseStamped,
            "/paf/" + self.role_name + "/current_pos",
            self.update_position,
            qos_profile=1)

        self.global_plan_subscriber = self.new_subscription(
            CarlaRoute,
            "/carla/" + self.role_name + "/global_plan",
            self.update_global_route,
            qos_profile=1)

        self.waypoint_publisher = self.new_publisher(
            Waypoint,
            "/paf/" + self.role_name + "/waypoint_distance",
            qos_profile=1)

        self.lane_change_publisher = self.new_publisher(
            LaneChange,
            "/paf/" + self.role_name + "/lane_change_distance",
            qos_profile=1)

    def update_position(self, pos):
        """
        Updates the current position based on the most upto date
        IMU, Speedometer and GNSS sensor data.
        :return:
        """

        def distance(a, b):
            d_x = (a.x - b.x) ** 2
            d_y = (a.y - b.y) ** 2
            return math.sqrt(d_x + d_y)

        self.current_pos = pos.pose

        # check if the global route has been published and that there are still
        # points to navigate to
        if self.global_route is not None and self.global_route:

            current_distance = distance(self.global_route[0].position,
                                        self.current_pos.position)
            next_distance = distance(self.global_route[1].position,
                                     self.current_pos.position)

            # if the road option indicates an intersection, the distance to the
            # next waypoint is also the distance to the stop line
            if self.road_options[0] < 4:
                # print("publish waypoint")

                self.waypoint_publisher.publish(
                    Waypoint(current_distance, True))
                self.lane_change_publisher.publish(
                    LaneChange(current_distance, False, self.road_options[0]))
            else:
                self.waypoint_publisher.publish(
                    Waypoint(current_distance, False))
                if self.road_options[0] == 5 or self.road_options[0] == 6:
                    self.lane_change_publisher.publish(
                        LaneChange(current_distance, True,
                                   self.road_options[0]))
            # if we reached the next waypoint, pop it and the next point will
            # be published
            if current_distance < 2.5 or next_distance < current_distance:
                self.road_options.pop(0)
                self.global_route.pop(0)

                if self.road_options[0] in {5, 6} and \
                   self.road_options[0] == self.road_options[1]:
                    self.road_options[1] = 4

                print(f"next road option = {self.road_options[0]}")

    def update_global_route(self, route):
        """
        Callback if the global route is published, saves the route and road
        options locally
        :param: route, CarlaRoute holding global route
        :return:
        """
        if self.global_route is None:
            self.global_route = list(route.poses)
            self.road_options = list(route.road_options)
            self.road_options.pop(0)
            self.global_route.pop(0)

    def run(self):
        """
        Control loop

        :return:
        """

        self.spin()


def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("position_publisher", args=args)
    try:
        node = GlobalPlanDistance()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
