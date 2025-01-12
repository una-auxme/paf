#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import py_trees
from std_msgs.msg import Float32

"""
Source: https://github.com/ll7/psaf2
"""


class Start(py_trees.behaviour.Behaviour):
    """
    This behavior is the first one being called when the decision tree starts,
    it sets a first target_speed
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Start, self).__init__(name)

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
        self.target_speed_pub = rospy.Publisher(
            "paf/hero/" "max_velocity", Float32, queue_size=1
        )
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        Publishes a first target speed
        """
        rospy.loginfo("Starting startup sequence!")
        self.target_speed_pub.publish(0.0)
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Returns SUCCESS
        :return:  py_trees.common.Status.SUCCESS, as the start up is successful
        """
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


class End(py_trees.behaviour.Behaviour):
    """
    This behavior is called as the last one when the agent finished the path.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(End, self).__init__(name)

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
        self.target_speed_pub = rospy.Publisher(
            "/paf/hero/" "max_velocity", Float32, queue_size=1
        )
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        Publishes a last target speed
        """
        self.target_speed_pub.publish(0.0)

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Returns SUCCESS
        :return:  py_trees.common.Status.RUNNING, if too far from last way
                  point
                  py_trees.common.Status.FAILURE, if last point reached
        """
        speed = self.blackboard.get("/carla/hero/Speed")
        if speed is None:
            return py_trees.common.Status.FAILURE
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
