#!/usr/bin/env python

from typing import Optional, Dict
import sys

import py_trees
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.behaviours import Running
import py_trees_ros

import rospy
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from behavior_agent.behaviors import (
    cruise,
    intersection,
    lane_change,
    leave_parking_space,
    overtake,
    topics2blackboard,
    unstuck_routine,
    debug_markers,
    speed_alteration,
)


from planning.cfg import BEHAVIORConfig
from dynamic_reconfigure.server import Server

"""
Source: https://github.com/ll7/psaf2
"""


def grow_a_tree(role_name):

    rules = Parallel(
        "Rules",
        children=[
            Selector(
                "Priorities",
                children=[
                    unstuck_routine.UnstuckRoutine("Unstuck Routine"),
                    Selector(
                        "Road Features",
                        children=[
                            leave_parking_space.LeaveParkingSpace(
                                "Leave Parking Space"
                            ),
                            Sequence(
                                "Intersection",
                                children=[
                                    intersection.Ahead("Intersection Ahead?"),
                                    Sequence(
                                        "Intersection Actions",
                                        children=[
                                            intersection.Approach(
                                                "Approach Intersection"
                                            ),
                                            intersection.Wait("Wait Intersection"),
                                            intersection.Enter("Enter Intersection"),
                                            intersection.Leave("Leave Intersection"),
                                        ],
                                    ),
                                ],
                            ),
                        ],
                    ),
                    Selector(
                        "Laneswitching",
                        children=[
                            Sequence(
                                "Laneswitch",
                                children=[
                                    lane_change.Ahead("Lane Change Ahead?"),
                                    Sequence(
                                        "Lane Change Actions",
                                        children=[
                                            lane_change.Approach("Approach Change"),
                                            lane_change.Wait("Wait Change"),
                                            lane_change.Change("Execute Change"),
                                        ],
                                    ),
                                ],
                            ),
                        ],
                    ),
                    Selector(
                        "Overtaking",
                        children=[
                            Sequence(
                                "Overtake",
                                children=[
                                    overtake.Ahead("Overtake Ahead?"),
                                    Sequence(
                                        "Overtake Actions",
                                        children=[
                                            overtake.Approach("Approach Overtake"),
                                            overtake.Wait("Wait Overtake"),
                                            overtake.Enter("Enter Overtake"),
                                            overtake.Leave("Leave Overtake"),
                                        ],
                                    ),
                                ],
                            ),
                        ],
                    ),
                    cruise.Cruise("Cruise"),
                ],
            )
        ],
    )

    root = Parallel(
        "Root",
        children=[
            debug_markers.DebugMarkerBlackboardSetupBehavior(),
            speed_alteration.SpeedAlterationSetupBehavior(),
            topics2blackboard.create_node(role_name),
            DynReconfigImportBehavior(),
            rules,
            speed_alteration.SpeedAlterationRequestBehavior(),
            debug_markers.DebugMarkerBlackboardPublishBehavior(),
            Running("Idle"),
        ],
    )
    return root


class DynReconfigImportBehavior(py_trees.Behaviour):
    """Imports the config into the blackboard

    Imports variables from the dynamic reconfigure config (config/behavior_config.yaml)
    into the blackboard.

    Parameters are available as "/params/*parameter_name*"
    """

    config: Optional[Dict] = None

    def __init__(self, *args, **kwargs):
        super().__init__(type(self).__name__, *args, **kwargs)
        self.blackboard = py_trees.blackboard.Blackboard()
        Server(BEHAVIORConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config: Dict, level):
        self.config = config
        return config

    def update(self):
        if self.config is None:
            return py_trees.common.Status.FAILURE

        for param in BEHAVIORConfig.config_description["parameters"]:
            param_name = param["name"]
            self.blackboard.set(
                f"/params/{param_name}", self.config[param_name], overwrite=True
            )
        return py_trees.common.Status.SUCCESS


class BehaviorTree(CompatibleNode):

    def __init__(self):
        super().__init__("BehaviorTree")

        role_name = self.get_param("~role_name", "hero")
        root = grow_a_tree(role_name)
        self.behavior_tree = py_trees_ros.trees.BehaviourTree(root)

        if not self.behavior_tree.setup(timeout=15):
            rospy.logerr("Behavior tree Setup failed.")
            sys.exit(1)

        rospy.loginfo("Behavior tree setup done.")

        self.rate = self.get_param("~tick_rate", 5.3)
        self.new_timer(1.0 / self.rate, self.tick_tree_handler)

    def tick_tree_handler(self, timer_event=None):
        try:
            self.tick_tree()
        except Exception as e:
            rospy.logfatal(e)

    def tick_tree(self, timer_event=None):
        self.behavior_tree.tick()

    def shutdown(self):
        self.behavior_tree.interrupt()


def main():
    """
    Entry point for the demo script.
    """
    roscomp.init("BehaviorTree")

    node = None
    try:
        node = BehaviorTree()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
        roscomp.shutdown()


if __name__ == "__main__":
    main()
