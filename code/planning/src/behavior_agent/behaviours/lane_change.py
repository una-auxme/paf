import py_trees
import numpy as np
from std_msgs.msg import String
# from nav_msgs.msg import Odometry
# from custom_carla_msgs.srv import UpdateLocalPath

import rospy

from . import behavior_speed as bs

"""
Source: https://github.com/ll7/psaf2
"""


def convert_to_ms(speed):
    return speed / 3.6


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an intersection and behaviours.road_features.intersection_ahead is
    triggered. It than handles the approaching the intersection, slowing the
    vehicle down appropriately.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
         :param name: name of the behaviour
        """
        super(Approach, self).__init__(name)
        rospy.loginfo("Approach started")

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the target speed publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher("/paf/hero/"
                                                 "curr_behavior",
                                                 String, queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This initializes the variables needed to save information about the
        stop line, stop signs and the traffic light.
        """
        rospy.loginfo("Approaching Change")
        self.start_time = rospy.get_time()
        self.change_detected = False
        self.change_distance = np.inf
        self.virtual_change_distance = np.inf
        self.curr_behavior_pub.publish(bs.lc_app_init.name)
        self.blocked = False

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Gets the current traffic light status, stop sign status
        and the stop line distance
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        # Update stopline Info
        _dis = self.blackboard.get("/paf/hero/lane_change_distance")
        if _dis is not None:
            self.change_distance = _dis.distance
            self.change_detected = _dis.isLaneChange
            self.change_option = _dis.roadOption
            rospy.loginfo(f"Change distance: {self.change_distance}")

        # calculate virtual stopline
        if self.change_distance != np.inf and self.change_detected:
            self.virtual_change_distance = self.change_distance

        # slow down before lane change
        # if self.virtual_change_distance < 15.0:
        #     if self.change_option == 5:
        #         distance_lidar = self.blackboard. \
        #             get("/carla/hero/LIDAR_range_rear_left")
        #     elif self.change_option == 6:
        #         distance_lidar = self.blackboard. \
        #             get("/carla/hero/LIDAR_range_rear_right")
        #     else:
        #         distance_lidar = None

            distance_lidar = 20  # Remove and adjust to check for cars behind

            if distance_lidar is not None and distance_lidar > 15.0:
                rospy.loginfo("Change is free not slowing down!")
                self.curr_behavior_pub.publish(bs.lc_app_free.name)
                self.blocked = False
            else:
                rospy.loginfo("Change blocked slowing down")
                self.blocked = True

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("no speedometer connected")
            return py_trees.common.Status.RUNNING
        if self.virtual_change_distance > 5 and self.blocked:
            # too far
            rospy.loginfo("still approaching")
            self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
            return py_trees.common.Status.RUNNING
        elif speed < convert_to_ms(2.0) and \
                self.virtual_change_distance < 5.0 and self.blocked:
            # stopped
            rospy.loginfo("stopped")
            return py_trees.common.Status.SUCCESS
        elif speed > convert_to_ms(5.0) and \
                self.virtual_change_distance < 3.5:
            # running over line
            return py_trees.common.Status.SUCCESS

        if self.virtual_change_distance < 5 and not self.change_detected:
            rospy.loginfo("Leave Change!")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        When is this called?
        Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting
            down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of the stop line at the inter-
    section until there either is no traffic light, the traffic light is
    green or the intersection is clear.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
         :param name: name of the behaviour
        """
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the target speed publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher("/paf/hero/"
                                                 "curr_behavior", String,
                                                 queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This just prints a state status message.
        """
        rospy.loginfo("Wait Change")
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Waits in front of the intersection until there is a green light, the
        intersection is clear or no traffic light at all.
        :return: py_trees.common.Status.RUNNING, while traffic light is yellow
                 or red
                 py_trees.common.Status.SUCCESS, if the traffic light switched
                 to green or no traffic light is detected
        """

        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("no speedometer connected")
            return py_trees.common.Status.RUNNING

        if speed > convert_to_ms(10):
            rospy.loginfo("Forward to enter")
            return py_trees.common.Status.SUCCESS

        # if road_option == 5:
        #     distance_lidar = self.blackboard. \
        #         get("/carla/hero/LIDAR_range_rear_left")
        # elif road_option == 6:
        #     distance_lidar = self.blackboard. \
        #         get("/carla/hero/LIDAR_range_rear_right")
        # else:
        #     distance_lidar = None

        distance_lidar = 20  # Remove to wait

        change_clear = False
        if distance_lidar is not None:
            # if distance smaller than 15m, change is blocked
            if distance_lidar < 15.0:
                change_clear = False
            else:
                change_clear = True
        if not change_clear:
            rospy.loginfo("Change blocked")
            self.curr_behavior_pub.publish(bs.lc_wait.name)
            return py_trees.common.Status.RUNNING
        else:
            rospy.loginfo("Change clear")
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
            Whenever your behaviour switches to a non-running state.
           - SUCCESS || FAILURE : your behaviour's work cycle has finished
           - INVALID : a higher priority branch has interrupted, or shutting
           down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))


class Enter(py_trees.behaviour.Behaviour):
    """
    This behavior handles the driving through an intersection, it initially
    sets a speed and finishes if the ego vehicle is close to the end of the
    intersection.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the target speed publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher("/paf/hero/"
                                                 "curr_behavior", String,
                                                 queue_size=1)
        # rospy.wait_for_service('update_local_path')
        # self.update_local_path = rospy.ServiceProxy("update_local_path",
        # UpdateLocalPath)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This prints a state status message and changes the driving speed for
        the intersection.
        """
        rospy.loginfo("Enter next Lane")
        self.curr_behavior_pub.publish(bs.lc_enter_init.name)

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Continues driving through the intersection until the vehicle gets
        close enough to the next global way point.
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        next_waypoint_msg = self.blackboard.\
            get("/paf/hero/lane_change_distance")

        if next_waypoint_msg is None:
            return py_trees.common.Status.FAILURE
        # if next_waypoint_msg.distance < 5 and
            # not next_waypoint_msg.isStopLine:
        if next_waypoint_msg.distance < 5:
            rospy.loginfo("Drive on the next lane!")
            # self.update_local_path(leave_intersection=True)
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
          - SUCCESS || FAILURE : your behaviour's work cycle has finished
          - INVALID : a higher priority branch has interrupted, or shutting
          down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))


class Leave(py_trees.behaviour.Behaviour):
    """
    This behaviour defines the leaf of this subtree, if this behavior is
    reached, the vehicle left the intersection.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the target speed publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher("/paf/hero/"
                                                 "curr_behavior", String,
                                                 queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This prints a state status message and changes the driving speed to
        the street speed limit.
        """
        rospy.loginfo("Leave Change")
        street_speed_msg = self.blackboard.get("/paf/hero/speed_limit")
        if street_speed_msg is not None:
            # self.curr_behavior_pub.publish(street_speed_msg.data)
            self.curr_behavior_pub.publish(bs.lc_exit.name)
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Abort this subtree
        :return: py_trees.common.Status.FAILURE, to exit this subtree
        """
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
          - SUCCESS || FAILURE : your behaviour's work cycle has finished
          - INVALID : a higher priority branch has interrupted, or shutting
          down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))
