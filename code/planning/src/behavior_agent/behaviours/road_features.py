#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import numpy as np
from scipy.spatial.transform import Rotation
import rospy

"""
Source: https://github.com/ll7/psaf2
"""


class IntersectionAhead(py_trees.behaviour.Behaviour):
    """
    This behaviour checks whether there is an intersection in front of the
    ego vehicle or not and triggers the rest of the decision tree handling the
     intersection.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(IntersectionAhead, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.
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
        stop line.
        """
        self.dist = 0
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Gets the current distance to the next intersection.
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                    of the intersection
                 py_trees.common.Status.FAILURE, if we are too far away from
                 the intersection
        """

        bb = self.blackboard.get("/paf/hero/waypoint_distance")
        if bb is None:
            return py_trees.common.Status.FAILURE
        else:
            dist = bb.distance
            isIntersection = bb.isStopLine
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


class LaneChangeAhead(py_trees.behaviour.Behaviour):
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
        super(LaneChangeAhead, self).__init__(name)

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


class OvertakeAhead(py_trees.behaviour.Behaviour):
    """
    This behaviour checks whether an object that needs to be overtaken is
    ahead
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(OvertakeAhead, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.
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
        """
        # Counter for detecting overtake situation
        self.counter_overtake = 0
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        Gets the current distance and speed to object in front.
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                    for the overtaking procedure
                 py_trees.common.Status.FAILURE, if we are too far away for
                 the overtaking procedure
        """

        obstacle_msg = self.blackboard.get("/paf/hero/collision")
        current_position = self.blackboard.get("/paf/hero/current_pos")
        current_heading = self.blackboard.get("/paf/hero/current_heading").data

        if obstacle_msg is None or current_position is None or current_heading is None:
            return py_trees.common.Status.FAILURE
        current_position = [
            current_position.pose.position.x,
            current_position.pose.position.y,
            current_position.pose.position.z,
        ]

        obstacle_distance = obstacle_msg.data[0]
        obstacle_speed = obstacle_msg.data[1]

        if obstacle_distance == np.Inf:
            return py_trees.common.Status.FAILURE
        # calculate approx collision position in global coords
        rotation_matrix = Rotation.from_euler("z", current_heading)
        # Apply current heading to absolute distance vector
        # and add to current position
        pos_moved_in_x_direction = current_position + rotation_matrix.apply(
            np.array([obstacle_distance, 0, 0])
        )

        if np.linalg.norm(pos_moved_in_x_direction - current_position) < 1:
            # current collision is not near trajectory lane
            rospy.logerr("Obstacle is not near trajectory lane")
            return py_trees.common.Status.FAILURE

        if obstacle_speed < 2 and obstacle_distance < 30:
            self.counter_overtake += 1
            rospy.loginfo("Overtake counter: " + str(self.counter_overtake))
            if self.counter_overtake > 3:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING
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


class MultiLane(py_trees.behaviour.Behaviour):
    """
    This behavior decides if the road the agent is currently on, has more than
    one lane in the driving direction. This could be used to change lanes to
    the right to perhaps evade an emergency vehicle.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(MultiLane, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.

        :return: True
        """
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        This part checks if the agent is on a multi-lane and corresponding
        overtaking behavior should be triggered. If we are not on a multi-lane
        it checks if overtaking on a single lane is possible. Otherwise, the
        overtaking process will be canceled.
        :return: py_trees.common.Status.SUCCESS, if the agent is on a multi-
                 lane
                 py_trees.common.Status.FAILURE, if the agent is not on a
                 multi-lane
        """
        bb = self.blackboard.get("/paf/hero/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.isMultiLane:
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
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class SingleLineDotted(py_trees.behaviour.Behaviour):
    """
    This behavior checks if it is allowed to switch lanes one a single lane
    street.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(SingleLineDotted, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.Success = False
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.

        :return: True

        """
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Right now, there is no way for us to detect a single dotted line, so it
        is assumed that we are on one.
        :return: py_trees.common.Status.SUCCESS, if the agent is on a single
                 dotted line
                 py_trees.common.Status.FAILURE, if the agent is not on a
                 single dotted line
        """
        if self.Success:
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
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class RightLaneAvailable(py_trees.behaviour.Behaviour):
    """
    This behavior checks if there is a lane to the right of the agent it could
    change to.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(RightLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
           Any initialisation you need before putting your behaviour to work.

        """
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        This part checks if there is a lane to the right of the agent.
        :return: py_trees.common.Status.SUCCESS, if there is a lane to the
                 right of the agent
                 py_trees.common.Status.FAILURE, if there is no lane to the
                 right of the agent
        """
        bb = self.blackboard.get("/paf/hero/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.rightLaneId != -1:
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
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class LeftLaneAvailable(py_trees.behaviour.Behaviour):
    """
    On a multi-lane, this behavior checks if there is a lane to the left of the
    agent it could change to, to overtake.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(LeftLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """

        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.

        :return: True
        """
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        This part checks if there is a lane to the left of the agent and the
        rest of the overtaking can take place.
        :return: py_trees.common.Status.SUCCESS, if there is a lane to the left
                 of the agent
                 py_trees.common.Status.FAILURE, if there is no lane to the
                 left of the agent
        """
        bb = self.blackboard.get("/paf/hero/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.leftLaneId != -1:
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
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s ]"
            % (self.name, self.status, new_status)
        )
