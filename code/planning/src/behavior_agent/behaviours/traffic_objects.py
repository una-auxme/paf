import py_trees
import rospy

"""
Source: https://github.com/ll7/psaf2
"""


class NotSlowedByCarInFront(py_trees.behaviour.Behaviour):
    """
    This behavior triggers all overtaking behaviors. It checks if there is
    something in front of the agent slowing it down that requires a lane change
    to avoid that obstacle. This could be a slower car or static obstacle.
    More cases could be added later on. This behavior should be triggered by
    the perception.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(NotSlowedByCarInFront, self).__init__(name)

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

        This part checks if there is something in front, that should trigger
        the overtaking process

        :return: py_trees.common.Status.SUCCESS, if there is nothing to react
                 to
                 py_trees.common.Status.FAILURE, if overtaking should be
                 triggered
        """
        slowed = self.blackboard.get("/paf/hero/slowed_by_car_in_front")
        if slowed is None:
            return py_trees.common.Status.SUCCESS
        if slowed.data is True:
            return py_trees.common.Status.FAILURE
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class WaitLeftLaneFree(py_trees.behaviour.Behaviour):
    """
    This behavior checks if it is safe to change to the lane on the left.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(WaitLeftLaneFree, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics. Also the time when this behavior started is recorded
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

        sets the timer for the timeout, when the overtaking is still not
        possible
        """
        self.timer = rospy.get_time()

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This part checks if the left lane is free. It is deemed free, if there
        is nothing or the distance to the obstacle is more than 20 m.

        :return: py_trees.common.Status.SUCCESS, if the lane change can be
                 performed
                 py_trees.common.Status.FAILURE, in case of error
                 py_trees.common.Status.RUNNING, while there will be waited for
                 the left lane to be free.
        """
        car_left = self.blackboard.get("/paf/hero/Obstacle_on_left_lane")
        if car_left is None:
            return py_trees.common.Status.SUCCESS
        elif car_left.data is None or car_left.data > 20:
            return py_trees.common.Status.SUCCESS
        elif (self.timer + 2) < rospy.get_time():
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


class WaitRightLaneFree(py_trees.behaviour.Behaviour):
    """
    This behavior checks if it is safe to change to the lane on the left.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(WaitRightLaneFree, self).__init__(name)

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
        The current time for a timeout is recorded
        """
        self.timer = rospy.get_time()

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This part checks if the right lane is free. It is deemed free, if there
        is nothing or the distance to the obstacle is more than 40 m.

        :return: py_trees.common.Status.SUCCESS, if the lane change can be
                 performed
                 py_trees.common.Status.FAILURE, in case of error
                 py_trees.common.Status.RUNNING, while there will be waited for
                 the right lane to be free.
        """
        car_right = self.blackboard.get("/paf/hero/obstacle_on_right_lane")
        if car_right is None:
            return py_trees.common.Status.SUCCESS
        elif car_right.data is None or car_right.data > 40:
            return py_trees.common.Status.SUCCESS
        elif (self.timer + 2) < rospy.get_time():
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


class NotSlowedByCarInFrontRight(py_trees.behaviour.Behaviour):
    """
    Checks if there is a car on the lane to the right that would slow the
    agent down.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(NotSlowedByCarInFrontRight, self).__init__(name)

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
        self.Success = True
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

        This part checks if there is something in front, that should trigger
        the overtaking process

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


class OvertakingPossible(py_trees.behaviour.Behaviour):
    """
    Checks if the overtaking is possible.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(OvertakingPossible, self).__init__(name)

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

        This part checks if there is something in front, that should trigger
        the overtaking process

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
