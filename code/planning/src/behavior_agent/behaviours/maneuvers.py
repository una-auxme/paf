import py_trees
import rospy
from std_msgs.msg import String, Float32
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
        super(LeaveParkingSpace, self).__init__(name)
        rospy.loginfo("LeaveParkingSpace started")
        self.called = False

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and gathers the time to check how much time has
        passed.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.curr_behavior_pub = rospy.Publisher("/paf/hero/"
                                                 "curr_behavior",
                                                 String, queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.initPosition = None
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
            Any initialisation you need before putting your behaviour to work.
        Get initial position to check how far vehicle has moved during
        execution
        """
        self.initPosition = self.blackboard.get("/paf/hero/current_pos")

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        pose: 
            position: 
                x: 294.43757083094295
                y: -1614.961812061094
                z: 211.1994649671884
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This behaviour runs until the agent has left the parking space.
        This is checked by calculating the euclidian distance thath the agent
        has moved since the start

        :return: py_trees.common.Status.RUNNING, while the agent is leaving
                                            the parking space
                 py_trees.common.Status.SUCCESS, never to continue with
                                            intersection
                 py_trees.common.Status.FAILURE, if not in parking
                 lane
        """
        position = self.blackboard.get("/paf/hero/current_pos")
        speed = self.blackboard.get("/carla/hero/Speed")
        if self.called is False:
            # calculate distance between start and current position
            if position is not None and \
                    self.initPosition is not None and \
                    speed is not None:
                startPos = np.array([position.pose.position.x,
                                     position.pose.position.y])
                endPos = np.array([self.initPosition.pose.position.x,
                                   self.initPosition.pose.position.y])
                distance = np.linalg.norm(startPos - endPos)
                if distance < 1 or speed.speed < 2:
                    self.curr_behavior_pub.publish(bs.parking.name)
                    self.initPosition = position
                    return py_trees.common.Status.RUNNING
                else:
                    self.called = True
                    return py_trees.common.Status.FAILURE
            else:
                self.initPosition = position
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


def get_distance(pos_1, pos_2):
    """Calculate the distance between two positions

    Args:
        pos1 (np.array): Position 1 [#,#]
        pos2 (np.array): Position 2 [#,#]

    Returns:
        float: Distance
    """

    return np.linalg.norm(pos_1 - pos_2)


def pos_to_np_array(pos):
    if pos is not None:
        return np.array([pos.pose.position.x, pos.pose.position.y])
    else:
        return None


TRIGGER_STUCK_COUNT = 30  # default 30 (about 5 s -> behavior tree 5 Hz)
TRIGGER_WAIT_STUCK_COUNT = 100  # default 100 (about 20 s-> behavior tree 5 Hz)
UNSTUCK_DRIVE_DURATION = 1.2  # default 1.2 (s)


class UnstuckRoutine(py_trees.behaviour.Behaviour):

    """
    This behavior is triggered when the vehicle is stuck and needs to be
    unstuck. The behavior will then try to reverse and steer to the left or
    right to get out of the stuck situation.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(UnstuckRoutine, self).__init__(name)
        self.stuck_count = 0
        self.wait_stuck_count = 0
        self.init_pos = None
        self.current_pos = None
        self.current_speed = None
        self.unstuck_overtake_count = 0
        dummy_pos = np.array([0, 0])
        self.last_unstuck_positions = np.array([dummy_pos, dummy_pos])

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
        self.pub_unstuck_distance = rospy.Publisher("/paf/hero/"
                                                    "unstuck_distance",
                                                    Float32, queue_size=1)
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
        self.init_ros_time = rospy.Time.now()

        self.current_speed = self.blackboard.get("/carla/hero/Speed")
        target_speed = self.blackboard.get("/paf/hero/target_velocity")

        # check for None values and initialize if necessary
        if self.current_speed is None or target_speed is None:
            if self.current_speed is None:
                rospy.logdebug("current_speed is None")
            elif target_speed is None:
                rospy.logdebug("target_speed is None")
            return True

        # check if vehicle is stuck, v = 0 when told to v > 0
        if self.current_speed.speed < 1 and target_speed.data >= 1:
            # rospy.loginfo("stuck_count increased by 1, now: %s",
            #               self.stuck_count + 1)
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        # check if vehicle is stuck in wait loop
        # for example when waiting for a traffic light seems to take too long
        # -> something might have went wrong
        if self.current_speed.speed < 1.0:
            # rospy.loginfo("wait_stuck_count increased by 1, now: %s",
            #               self.wait_stuck_count + 1)
            self.wait_stuck_count += 1
        else:
            self.wait_stuck_count = 0

        # inform about which kind of stuck situation is detected
        if self.stuck_count >= TRIGGER_STUCK_COUNT:
            rospy.logfatal("Stuck detected -> starting unstuck routine")
            self.init_pos = pos_to_np_array(
                self.blackboard.get("/paf/hero/current_pos"))
        elif self.wait_stuck_count >= TRIGGER_WAIT_STUCK_COUNT:
            rospy.logfatal("Wait stuck detected -> starting unstuck routine")
            self.init_pos = pos_to_np_array(
                self.blackboard.get("/paf/hero/current_pos"))

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

        self.current_pos = pos_to_np_array(
            self.blackboard.get("/paf/hero/current_pos"))
        self.current_speed = self.blackboard.get("/carla/hero/Speed")

        if self.stuck_count is None:
            # rospy.logwarn("stuck_count is None")
            return py_trees.common.Status.FAILURE
        # if no stuck detected, return failure
        if self.stuck_count < TRIGGER_STUCK_COUNT and \
           self.wait_stuck_count < TRIGGER_WAIT_STUCK_COUNT:
            return py_trees.common.Status.FAILURE

        # stuck detected -> unstuck routine
        ros_time = rospy.Time.now()
        if ros_time - self.init_ros_time < \
           rospy.Duration(UNSTUCK_DRIVE_DURATION):

            self.curr_behavior_pub.publish(bs.us_unstuck.name)
            rospy.logwarn("Unstuck routine running.")
            return py_trees.common.Status.RUNNING
        else:
            # while vehicle is stopping publish us_stop
            if self.current_speed.speed < -0.1:
                self.curr_behavior_pub.publish(bs.us_stop.name)
                return py_trees.common.Status.RUNNING
            # vehicle has stopped:
            # check if vehicle needs to overtake:

            # save current pos to last_unstuck_positions
            self.last_unstuck_positions = np.roll(self.last_unstuck_positions,
                                                  -1, axis=0)
            self.last_unstuck_positions[-1] = self.init_pos

            # if last unstuck was too far away, no overtake
            # we only want to overtake when we tried to unstuck twice
            if np.array_equal(self.last_unstuck_positions[0],
                              np.array([0, 0])):
                self.stuck_count = 0
                self.wait_stuck_count = 0
                self.unstuck_overtake_count = 0
                return py_trees.common.Status.FAILURE
            rospy.logfatal("Distance to last unstuck position: %s",
                           get_distance(self.last_unstuck_positions[0],
                                        self.last_unstuck_positions[-1]))
            if get_distance(self.last_unstuck_positions[0],
                            self.last_unstuck_positions[-1]) > 1.5:
                self.stuck_count = 0
                self.wait_stuck_count = 0
                self.unstuck_overtake_count = 0
                return py_trees.common.Status.FAILURE

            # once we tried the unstuck twice, we try to overtake
            if self.current_speed.speed < 1:
                unstuck_distance = get_distance(self.init_pos,
                                                self.current_pos)

                self.pub_unstuck_distance.publish(unstuck_distance)
                # rospy.logwarn("Unstuck DISTANCE %s.", unstuck_distance)

                # publish the over take behavior 3 times to make sure
                # it is detected
                self.curr_behavior_pub.publish(bs.us_overtake.name)
                if self.unstuck_overtake_count > 3:
                    self.stuck_count = 0
                    self.wait_stuck_count = 0
                    self.unstuck_overtake_count = 0
                    rospy.logwarn("Unstuck routine finished.")
                    return py_trees.common.Status.FAILURE
                else:
                    self.unstuck_overtake_count += 1
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
