import py_trees
import numpy as np
from std_msgs.msg import String

import rospy

from behaviors import behavior_speed as bs
from local_planner.utils import TARGET_DISTANCE_TO_STOP, convert_to_ms

"""
Source: https://github.com/ll7/psaf2
"""


class Ahead(py_trees.behaviour.Behaviour):
    """
    This behaviour checkes wheather there is an lane change in front of the
    ego vehicle or not and triggers the rest of the decision tree handling the
     lane change.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.
        Other one-time initialisation requirements should be met via the
        setup() method.
         :param name: name of the behaviour
        """
        super(Ahead, self).__init__(name)

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
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This initializes the variables needed to save information about the
        lane change.
        """
        self.dist = 0

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Gets the current distance to the next lane change.
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                    of the lane change
                 py_trees.common.Status.FAILURE, if we are too far away from
                 the lane change
        """
        bb = self.blackboard.get("/paf/hero/lane_change_distance")
        if bb is None:
            return py_trees.common.Status.FAILURE
        else:
            dist = bb.distance
            isIntersection = bb.isLaneChange
        if dist < 30 and isIntersection:
            return py_trees.common.Status.SUCCESS
        else:
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
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an lane change and behaviours.road_features.LaneChangeAhead is
    triggered. It than handles approaching the lane change, slowing the
    vehicle down appropriately.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
         :param name: name of the behaviour
        """
        super(Approach, self).__init__(name)
        rospy.loginfo("Lane Change Approach started")

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the current behavior publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
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
        lane change.
        """
        rospy.loginfo("Approaching Change")
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
        Calculates a virtual stop line and slows down while approaching unless
        lane change is not blocked.
        :return: py_trees.common.Status.RUNNING, if too far from lane change
                 py_trees.common.Status.SUCCESS, if stopped in front of lane
                 change or entered the lane change
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        # Update stopline Info
        _dis = self.blackboard.get("/paf/hero/lane_change_distance")
        if _dis is not None:
            self.change_distance = _dis.distance
            self.change_detected = _dis.isLaneChange
            self.change_option = _dis.roadOption
            # rospy.loginfo(f"Change distance: {self.change_distance}")

        # calculate virtual stopline
        if self.change_distance != np.inf and self.change_detected:
            self.virtual_change_distance = self.change_distance

            # TODO: ADD FEATURE Check for Traffic
            distance_lidar = 20

            if distance_lidar is not None and distance_lidar > 15.0:
                rospy.loginfo("Lane Change is free not slowing down!")
                self.curr_behavior_pub.publish(bs.lc_app_free.name)
                self.blocked = False
            else:
                rospy.loginfo("Lane Change blocked slowing down")
                self.blocked = True

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")

        target_dis = TARGET_DISTANCE_TO_STOP

        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("Lane Change: no speedometer connected")
            return py_trees.common.Status.RUNNING
        if self.virtual_change_distance > target_dis and self.blocked:
            # too far
            rospy.loginfo(
                f"Lane Change: still approaching, "
                f"distance:{self.virtual_change_distance}"
            )
            self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
            return py_trees.common.Status.RUNNING
        elif (
            speed < convert_to_ms(2.0)
            and self.virtual_change_distance < target_dis
            and self.blocked
        ):
            # stopped
            rospy.loginfo("Lane Change: stopped at virtual stop line")
            return py_trees.common.Status.SUCCESS
        elif (
            speed > convert_to_ms(5.0)
            and self.virtual_change_distance < 3.5
            and not self.blocked
        ):
            # running over line
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
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of the lane change.
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
        by the ROS topics and the current behavior publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
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
        rospy.loginfo("Lane Change Wait")
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Waits in front of the lane change until the lane change is free.
        :return: py_trees.common.Status.RUNNING, while traffic light is yellow
                 or red
                 py_trees.common.Status.SUCCESS, if the traffic light switched
                 to green or no traffic light is detected
        """

        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("Lane change wait: no speedometer connected")
            return py_trees.common.Status.RUNNING

        if speed > convert_to_ms(10):
            rospy.loginfo("Lane change wait: Was not blocked, proceed to drive forward")
            return py_trees.common.Status.SUCCESS

        # TODO: ADD FEATURE Check for Traffic
        distance_lidar = 20

        change_clear = False
        if distance_lidar is not None:
            # if distance smaller than 15m, change is blocked
            if distance_lidar < 15.0:
                change_clear = False
            else:
                change_clear = True
        if not change_clear:
            rospy.loginfo("Lane Change Wait: blocked")
            self.curr_behavior_pub.publish(bs.lc_wait.name)
            return py_trees.common.Status.RUNNING
        else:
            rospy.loginfo("Lane Change Wait: Change clear")
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
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Enter(py_trees.behaviour.Behaviour):
    """
    This behavior inititates the lane change and waits until the
    lane change is finished.
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
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
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
        the lane change.
        """
        rospy.loginfo("Lane Change: Enter next Lane")
        self.curr_behavior_pub.publish(bs.lc_enter_init.name)

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Continues driving through the lane change until the vehicle gets
        close enough to the next global way point.
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        next_waypoint_msg = self.blackboard.get("/paf/hero/lane_change_distance")

        if next_waypoint_msg is None:
            return py_trees.common.Status.FAILURE
        if next_waypoint_msg.distance < 5:
            rospy.loginfo("Lane Change Enter: Drive on the next lane!")
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
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Leave(py_trees.behaviour.Behaviour):
    """
    This behaviour defines the leaf of this subtree, if this behavior is
    reached, the vehicle has finished the lane change.
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
        by the ROS topics and the current behavior publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
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
        rospy.loginfo("Lane Change Finished")

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
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )
