import py_trees
import rospy
from std_msgs.msg import String
import numpy as np

from . import behavior_speed as bs
# from behavior_agent.msg import BehaviorSpeed

"""
Source: https://github.com/ll7/psaf2
"""


class LeaveParkingSpace(py_trees.behaviour.Behaviour):
    """
    This behavior is triggered in the beginning when the vehicle needs
    to leave the parking space.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(SwitchLaneLeft, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and gathers the time to check how much time has passed.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.blackboard = py_trees.blackboard.Blackboard()
        self.curr_behavior_pub = rospy.Publisher("/paf/hero/"
                                                 "curr_behavior",
                                                 String, queue_size=1)
        self.initRosTime = rospy.get_rostime()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
            Any initialisation you need before putting your behaviour to work.
        Get initial position to check how far vehicle has moved during execution
        """
        self.initPosition = self.blackboard.get("/paf/hero/current_pos")

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This behaviour runs until the agent has left the parking space.
        This is checked by calculating the euclidian distance thath the agent has moved since the start

        :return: py_trees.common.Status.RUNNING, while the agent is leaving the parking space
                 py_trees.common.Status.SUCCESS, never to continue with intersection
                 py_trees.common.Status.FAILURE, if not in parking
                 lane
        """
        position = self.blackboard.get("/paf/hero/current_pos")
        # calculate distance between start and current position
        startPos = np.array([position.pose.position.x, position.pose.position.y])
        endPos = np.array([self.initPosition.pose.position.x, self.initPosition.pose.position.y])
        distance = np.linalg.norm(startPos - endPos)
        # Additionally the behavior is only executed for 10 seconds
        if distance < 3 and rospy.get_rostime() - self.initRosTime < rospy.Time(10):	
            self.curr_behavior_pub.publish(bs.parking)
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class SwitchLaneLeft(py_trees.behaviour.Behaviour):
    """
    This behavior triggers the replanning of the path in the local planner to
    switch to the lane to the left. A check if the lane is free might be added
    in the future.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(SwitchLaneLeft, self).__init__(name)

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
        Get the current lane ID
        """
        lane_status = self.blackboard.get("/paf/hero/lane_status")
        self.lanelet_id_before_lane_change = lane_status.currentLaneId

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This behaviour runs until the agent is on a different lane as in the
        start of this behavior

        :return: py_trees.common.Status.RUNNING, while the lane hasn't changed
                 py_trees.common.Status.SUCCESS, if the agent changed the lane
                 py_trees.common.Status.FAILURE, if the agent is on an unknown
                 lane
        """
        lane_status = self.blackboard.get("/paf/hero/lane_status")
        if lane_status.currentLaneId == -1:
            return py_trees.common.Status.FAILURE
        elif self.lanelet_id_before_lane_change != lane_status.currentLaneId:
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class SwitchLaneRight(py_trees.behaviour.Behaviour):
    """
    This behavior triggers the replanning of the path in the local planner to
    switch to the lane to the right. A check if the lane is free might be added
    in the future.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(SwitchLaneRight, self).__init__(name)

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

        Get the current lane ID
        """
        lane_status = self.blackboard.get("/paf/hero/lane_status")
        self.lanelet_id_before_lane_change = lane_status.currentLaneId

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This behaviour runs until the agent is on a different lane as in the
        start of this behavior

        :return: py_trees.common.Status.RUNNING, while the lane hasn't changed
                 py_trees.common.Status.SUCCESS, if the agent changed the lane
                 py_trees.common.Status.FAILURE, if the agent is on an unknown
                 lane
        """
        lane_status = self.blackboard.get("/paf/hero/lane_status")
        if lane_status.currentLaneId == -1:
            return py_trees.common.Status.FAILURE
        elif self.lanelet_id_before_lane_change != lane_status.currentLaneId:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class Overtake(py_trees.behaviour.Behaviour):
    """
    This behavior handles the overtaking process.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(Overtake, self).__init__(name)

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

        This behaviour runs until the agent is on a different lane as in the
        start of this behavior

        :return: py_trees.common.Status.SUCCESS, if this behavior is
                 implemented
                 py_trees.common.Status.FAILURE, otherwise
        """
        if self.Success:
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class Cruise(py_trees.behaviour.Behaviour):
    """
    This behaviour is the lowest priority one and will be executed when no
    other behaviour is triggered. It doesn't do much, as in the normal cruising
    the holding of the lane and speed control is done by different parts of the
    project. It might be possible to put the activation/deactivation of the ACC
    here.

    speed control = acting via speed limits and target_speed
    following the trajectory = acting
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(Cruise, self).__init__(name)
        rospy.loginfo("Cruise started")

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
        :return: True
        """
        rospy.loginfo("Starting Cruise")
        return True

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This behaviour doesn't do anything else than just keep running unless
        there is a higher priority behaviour

        :return: py_trees.common.Status.RUNNING, keeps the decision tree from
        finishing
        """
        self.curr_behavior_pub.publish(bs.cruise.name)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        When is this called?
        Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting
            down

        writes a status message to the console when the behaviour terminates
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
