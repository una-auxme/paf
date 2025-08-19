import py_trees
import rospy
from std_msgs.msg import String

from . import behavior_names as bs


class Cruise(py_trees.behaviour.Behaviour):
    """
    This behaviour is the lowest priority one and will be executed when no
    other behaviour is triggered. It doesn't do much, as in the normal cruising
    the holding of the lane and speed control is done by different parts of the
    project.

    speed control = acc with speed limits and other limits
    following the trajectory = acting
    """

    def __init__(self, name):
        super(Cruise, self).__init__(name)
        rospy.loginfo("Starting Cruise")

    def setup(self, timeout):
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/curr_behavior", String, queue_size=1
        )
        return True

    def initialise(self):
        self.curr_behavior_pub.publish(bs.cruise.name)

    def update(self):
        """
        This behaviour doesn't do anything else than just keep running unless
        there is a higher priority behaviour

        :return: py_trees.common.Status.RUNNING, keeps the decision tree from
        finishing
        """
        self.curr_behavior_pub.publish(bs.cruise.name)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
