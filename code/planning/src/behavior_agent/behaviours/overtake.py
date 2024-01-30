import py_trees
from std_msgs.msg import String

import rospy
import numpy as np

from . import behavior_speed as bs

"""
Source: https://github.com/ll7/psaf2
"""


def convert_to_ms(speed):
    return speed / 3.6


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an object which needs to be overtaken and
    behaviours.road_features.overtake_ahead is triggered.
    It than handles the procedure for overtaking.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
         :param name: name of the behaviour
        """
        super(Approach, self).__init__(name)
        rospy.loginfo("Init -> Overtake Behavior: Approach")

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

        This initializes the overtaking distance to a default value.
        """
        rospy.loginfo("Approaching Overtake")
        self.ot_distance = 30

        self.ot_option = 1  # self.blackboard.get("paf/hero/...")
        if self.ot_option == 0:
            self.clear_distance = 15
            self.side = "LIDAR_range_rear_left"
        elif self.ot_option == 1:
            self.clear_distance = 30
            self.side = "LIDAR_range"

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        Gets the current distance to overtake, the current lane status and the
        distance to collsion object.
        :return: py_trees.common.Status.RUNNING, if too far from overtaking
                 py_trees.common.Status.SUCCESS, if stopped behind the blocking
                 object or entered the process.
                 py_trees.common.Status.FAILURE,
        """
        # Update distance to collision object
        _dis = self.blackboard.get("/paf/hero/collision")
        if _dis is not None:
            self.ot_distance = _dis.data[0]
            rospy.loginfo(f"Overtake distance: {self.ot_distance}")

        # slow down before overtake if blocked
        if self.ot_distance < 15.0:
            distance_lidar = self.blackboard. \
                    get(f"/carla/hero/{self.side}")
            if distance_lidar is not None:
                distance_lidar = distance_lidar.data
            else:
                distance_lidar = None

            if distance_lidar is not None and \
                    distance_lidar > self.clear_distance:
                rospy.loginfo("Overtake is free not slowing down!")
                self.curr_behavior_pub.publish(bs.ot_app_free.name)
                return py_trees.common.Status.SUCCESS
            else:
                rospy.loginfo("Overtake blocked slowing down")
                self.curr_behavior_pub.publish(bs.ot_app_blocked.name)

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("no speedometer connected")
            return py_trees.common.Status.RUNNING

        if self.ot_distance > 15.0:
            # too far
            rospy.loginfo("still approaching")
            return py_trees.common.Status.RUNNING
        elif speed < convert_to_ms(2.0) and \
                self.ot_distance < 5.0:
            # stopped
            rospy.loginfo("stopped")
            return py_trees.common.Status.SUCCESS
        else:
            rospy.logerr("Overtake Approach: Default Case")
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
    This behavior handles the waiting in front of object,
    which is blocking the road.
    The Ego vehicle is waiting to get a clear path for overtaking.
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
        rospy.loginfo("Waiting for Overtake")
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        Waits behind the road object until the lidar distance meets the
        requirement for the clear distance.

        :return: py_trees.common.Status.RUNNING, while clear distance is bigger
                    than lidar_distance
                 py_trees.common.Status.SUCCESS,if no lidar distance is present
                    or the lidar distance is bigger than the clear distance
        """

        # Update distance to collison and distance for clear
        self.ot_option = 1  # self.blackboard.get("paf/hero/...")
        if self.ot_option == 0:
            distance_lidar = self.blackboard. \
                get("/carla/hero/LIDAR_range_rear_left")
            clear_distance = 15
        elif self.ot_option == 1:
            distance_lidar = self.blackboard. \
                get("/carla/hero/LIDAR_range")
            distance_lidar = distance_lidar.data
            clear_distance = 30
        else:
            distance_lidar = None

        obstacle_msg = self.blackboard.get("/paf/hero/collision")
        if obstacle_msg is None:
            return py_trees.common.Status.FAILURE

        if distance_lidar is not None:
            collision_distance = distance_lidar.data
            if collision_distance > clear_distance:
                rospy.loginfo("Overtake is free!")
                self.curr_behavior_pub.publish(bs.ot_wait_free.name)
                return py_trees.common.Status.SUCCESS
            else:
                rospy.loginfo("Overtake still blocked")
                self.curr_behavior_pub.publish(bs.ot_wait_stopped.name)
                return py_trees.commom.Status.RUNNING
        elif obstacle_msg.data[0] == np.inf:
            return py_trees.common.Status.FAILURE
        else:
            rospy.loginfo("No Lidar Distance")
            return py_trees.common.Success

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
    This behavior handles the switching to a new lane in the
    overtaking procedure.
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
        by the ROS topics and the current behavior publisher.
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

        This prints a state status message and publishes the behavior to
        trigger the replanning
        """
        rospy.loginfo("Enter Overtake")
        self.curr_behavior_pub.publish(bs.ot_enter_init.name)

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]


        :return: py_trees.common.Status.RUNNING,
                 py_trees.common.Status.SUCCESS,
                 py_trees.common.Status.FAILURE,
        """
        status = self.blackboard.get("/paf/hero/overtake_success")
        if status is not None:
            if status.data == 1:
                rospy.loginfo("Overtake: Trajectory planned")
                return py_trees.common.Status.SUCCESS
            elif status.data == 0:
                self.curr_behavior_pub.publish(bs.ot_enter_slow.name)
                rospy.loginfo("Overtake: Slowing down")
                return py_trees.common.Status.RUNNING
            else:
                rospy.loginfo("Overtake: Big Failure")
                return py_trees.common.Status.FAILURE
        else:
            rospy.loginfo("Overtake: Bigger Failure")
            return py_trees.common.Status.FAILURE
        # Currently not in use
        # Can be used to check if we can go back to the original lane

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
    reached, the vehicle peformed the overtake.
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
        This prints a state status message and publishes the behavior
        """
        rospy.loginfo("Leave Overtake")
        self.curr_behavior_pub.publish(bs.ot_leave.name)
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
