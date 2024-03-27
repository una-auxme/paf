import py_trees
import numpy as np
from std_msgs.msg import String

import rospy

from .import behavior_speed as bs

"""
Source: https://github.com/ll7/psaf2
"""


def convert_to_ms(speed):
    return speed / 3.6


# 1: Green, 2: Red, 4: Yellow, 0: Unknown
def get_color(state):
    if state == 1:
        return "green"
    elif state == 2:
        return "red"
    elif state == 4:
        return "yellow"
    else:
        return ""


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
        rospy.loginfo("Approaching Intersection")
        self.start_time = rospy.get_time()
        self.stop_sign_detected = False
        self.stop_distance = np.inf
        self.intersection_distance = np.inf
        self.traffic_light_detected = False
        self.traffic_light_distance = np.inf
        self.traffic_light_status = ''
        self.virtual_stopline_distance = np.inf
        self.curr_behavior_pub.publish(bs.int_app_init.name)
        self.last_virtual_distance = np.inf

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
        # Update Light Info
        light_status_msg = self.blackboard.get(
            "/paf/hero/Center/traffic_light_state")
        light_distance_y_msg = self.blackboard.get(
            "/paf/hero/Center/traffic_light_y_distance")
        if light_status_msg is not None:
            self.traffic_light_status = get_color(light_status_msg.state)
            self.traffic_light_detected = True
            if light_distance_y_msg is not None:
                self.traffic_light_distance = light_distance_y_msg.data

        # Update stopline Info
        _dis = self.blackboard.get("/paf/hero/waypoint_distance")
        if _dis is not None:
            self.stopline_distance = _dis.distance
            self.stopline_detected = _dis.isStopLine

        # Update stop sign Info
        stop_sign_msg = self.blackboard.get("/paf/hero/stop_sign")
        if stop_sign_msg is not None:
            self.stop_sign_detected = stop_sign_msg.isStop
            self.stop_distance = stop_sign_msg.distance

        # calculate virtual stopline
        if self.stopline_distance != np.inf and self.stopline_detected:
            self.virtual_stopline_distance = self.stopline_distance
        elif self.stop_sign_detected:
            self.virtual_stopline_distance = self.stop_distance
        else:
            self.virtual_stopline_distance = 0.0
        target_distance = 5.0
        # stop when there is no or red/yellow traffic light or a stop sign is
        # detected
        if self.traffic_light_status == '' \
                or self.traffic_light_status == 'red' \
                or self.traffic_light_status == 'yellow'\
                or (self.stop_sign_detected and
                    not self.traffic_light_detected):

            rospy.loginfo("slowing down!")
            self.curr_behavior_pub.publish(bs.int_app_to_stop.name)

        # approach slowly when traffic light is green as traffic lights are
        # higher priority than traffic signs this behavior is desired
        if self.traffic_light_status == 'green':
            self.curr_behavior_pub.publish(bs.int_app_green.name)

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("no speedometer connected")
            return py_trees.common.Status.RUNNING
        if (self.virtual_stopline_distance > target_distance) and \
                (self.traffic_light_distance > 150):
            # too far
            print("still approaching")
            return py_trees.common.Status.RUNNING
        elif speed < convert_to_ms(2.0) and \
                ((self.virtual_stopline_distance < target_distance) or
                 (self.traffic_light_distance < 150)):
            # stopped
            print("stopped")
            return py_trees.common.Status.SUCCESS
        elif speed > convert_to_ms(5.0) and \
                self.virtual_stopline_distance < 6.0 and \
                self.traffic_light_status == "green":

            # drive through intersection even if traffic light turns yellow
            return py_trees.common.Status.SUCCESS
        elif speed > convert_to_ms(5.0) and \
                self.virtual_stopline_distance < 3.5:
            # running over line
            return py_trees.common.Status.SUCCESS
        elif self.last_virtual_distance == self.virtual_stopline_distance \
                and self.virtual_stopline_distance < 10.0:
            # ran over line
            return py_trees.common.Status.SUCCESS

        if self.virtual_stopline_distance < target_distance and \
           not self.stopline_detected:
            rospy.loginfo("Leave intersection!")
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
        self.red_light_flag = False
        self.green_light_counter = 0
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This just prints a state status message.
        :return: True
        """
        rospy.loginfo("Wait Intersection")
        self.red_light_flag = False
        self.green_light_counter = 0
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
        light_status_msg = self.blackboard.get(
            "/paf/hero/Center/traffic_light_state")

        lidar_data = None
        intersection_clear = True
        if lidar_data is not None:
            # if distance smaller than 10m, intersection is blocked
            if lidar_data.data < 10.0:
                intersection_clear = False
            else:
                intersection_clear = True

        if light_status_msg is not None:
            traffic_light_status = get_color(light_status_msg.state)
            if traffic_light_status == "red" or \
                    traffic_light_status == "yellow":
                self.red_light_flag = True
                self.green_light_counter = 0
                rospy.loginfo(f"Light Status: {traffic_light_status}")
                self.curr_behavior_pub.publish(bs.int_wait.name)
                return py_trees.common.Status.RUNNING
            elif self.green_light_counter < 6 and \
                    traffic_light_status == "green":
                self.green_light_counter += 1
                rospy.loginfo(f"Light Counter: {self.green_light_counter}")
                return py_trees.common.Status.RUNNING
            elif self.red_light_flag and traffic_light_status != "green":
                rospy.loginfo(f"Light Status: {traffic_light_status}"
                              "-> prev was red")
                return py_trees.common.Status.RUNNING
            elif self.green_light_counter >= 6 and \
                    traffic_light_status == "green":
                rospy.loginfo(f"Light Status: {traffic_light_status}")
                return py_trees.common.Status.SUCCESS
            else:
                rospy.loginfo(f"Light Status: {traffic_light_status}"
                              "-> No Traffic Light detected")

        # Check clear if no traffic light is detected
        if not intersection_clear:
            rospy.loginfo("Intersection blocked")
            self.curr_behavior_pub.publish(bs.int_wait.name)
            return py_trees.common.Status.RUNNING
        else:
            rospy.loginfo("Intersection clear")
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
        rospy.loginfo("Enter Intersection")

        light_status_msg = self.blackboard.get(
            "/paf/hero/Center/traffic_light_state")
        if light_status_msg is None:
            self.curr_behavior_pub.publish(bs.int_enter.name)
            return True

        traffic_light_status = get_color(light_status_msg.state)

        rospy.loginfo(f"Light Status: {traffic_light_status}")
        self.curr_behavior_pub.publish(bs.int_enter.name)

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
        next_waypoint_msg = self.blackboard.get("/paf/hero/waypoint_distance")

        if next_waypoint_msg is None:
            return py_trees.common.Status.FAILURE
        # if next_waypoint_msg.distance < 5 and
            # not next_waypoint_msg.isStopLine:
        if next_waypoint_msg.distance < 5:
            rospy.loginfo("Drive through intersection!")
            self.curr_behavior_pub.publish(bs.int_enter.name)
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
        :return: True
        """
        rospy.loginfo("Leave Intersection")
        self.curr_behavior_pub.publish(bs.int_exit.name)
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?exit_int
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
