from typing import Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile

import py_trees
import py_trees_ros

from std_msgs.msg import Float32, String
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mapping_interfaces.msg import Map as MapMsg
from mapping_common.map import Map

from perception_interfaces.msg import Waypoint, TrafficLightState
from planning.behavior_agent.blackboard_utils import Blackboard

BLACKBOARD_MAP_ID = "/import/map"


class ImportMapBehavior(py_trees.behaviour.Behaviour):
    """Converts the /paf/hero/mapping/init_data from the Blackboard into
    the Map type and puts it into the blackboard at *BLACKBOARD_MAP_ID*
    """

    def __init__(self, *args, **kwargs):
        super().__init__(type(self).__name__, *args, **kwargs)

        self.blackboard = Blackboard()

    def update(self):
        map_data_topic = "/paf/hero/mapping/init_data"
        map_msg = self.blackboard.try_get(map_data_topic)
        if map_msg is None:
            return py_trees.common.Status.FAILURE

        map = Map.from_ros_msg(map_msg)
        self.blackboard.set(BLACKBOARD_MAP_ID, map)
        return py_trees.common.Status.SUCCESS


"""
Source: https://github.com/ll7/psaf2
"""


def create_node(role_name: str, callback_group: Optional[CallbackGroup] = None):
    """
    This function initializes the topics which will be written to the decision
    tree blackboard and accessible by the decision tree.
    :param role_name: name of the agent
    :return: topics2blackboard the subtree of the topics in the blackboard
    """
    topics = [
        {
            "name": f"/carla/{role_name}/Speed",
            "msg": CarlaSpeedometer,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/current_waypoint",
            "msg": Waypoint,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/Center/traffic_light_state",
            "msg": TrafficLightState,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/acc_velocity",
            "msg": Float32,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/current_pos",
            "msg": PoseStamped,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/current_heading",
            "msg": Float32,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/target_velocity",
            "msg": Float32,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/mapping/init_data",
            "msg": MapMsg,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/trajectory_local",
            "msg": Path,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
        {
            "name": f"/paf/{role_name}/curr_behavior",
            "msg": String,
            "clearing-policy": py_trees.common.ClearingPolicy.NEVER,
        },
    ]

    qos_profile = QoSProfile(depth=1)
    topics2blackboard = py_trees.composites.Parallel(
        "Topics to Blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    for topic in topics:
        new_child = py_trees_ros.subscribers.ToBlackboard(
            name=topic["name"],
            topic_name=topic["name"],
            topic_type=topic["msg"],
            qos_profile=qos_profile,
            blackboard_variables={topic["name"]: None},
            clearing_policy=topic["clearing-policy"],
        )
        new_child.callback_group = callback_group
        topics2blackboard.add_child(new_child)

    topics2blackboard.add_child(ImportMapBehavior())

    return topics2blackboard
