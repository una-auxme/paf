from typing import List, Optional
import sys

import rclpy
import rclpy.executors
import rclpy.callback_groups
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    SetParametersResult,
)
from visualization_msgs.msg import MarkerArray, Marker
from mapping_interfaces.srv import UpdateStopMarks
from planning_interfaces.srv import (
    SpeedAlteration,
    StartOvertake,
    EndOvertake,
    OvertakeStatus,
)

import py_trees
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.behaviours import Running
import py_trees_ros

from . import behaviors
from .behaviors import (
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
from .bt_parameters import register_parameters

import mapping_common

"""
Source: https://github.com/ll7/psaf2
"""


def grow_a_tree(
    role_name: str, node: "BehaviorTree", callback_group: Optional[CallbackGroup] = None
):

    rules = Parallel(
        "Rules",
        policy=ParallelPolicy.SuccessOnOne(),
        children=[
            Selector(
                "Priorities",
                memory=False,
                children=[
                    unstuck_routine.UnstuckRoutine("Unstuck Routine"),
                    Selector(
                        "Road Features",
                        memory=False,
                        children=[
                            leave_parking_space.LeaveParkingSpace(
                                "Leave Parking Space"
                            ),
                            Sequence(
                                "Intersection",
                                memory=True,
                                children=[
                                    intersection.Ahead("Intersection Ahead?"),
                                    Sequence(
                                        "Intersection Actions",
                                        memory=True,
                                        children=[
                                            intersection.Approach(
                                                "Approach Intersection"
                                            ),
                                            intersection.Wait("Wait Intersection"),
                                            intersection.Enter("Enter Intersection"),
                                        ],
                                    ),
                                ],
                            ),
                        ],
                    ),
                    Selector(
                        "Laneswitching",
                        memory=False,
                        children=[
                            Sequence(
                                "Laneswitch",
                                memory=True,
                                children=[
                                    lane_change.Ahead("Lane Change Ahead?"),
                                    Sequence(
                                        "Lane Change Actions",
                                        memory=True,
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
                        memory=False,
                        children=[
                            Sequence(
                                "Overtake",
                                memory=True,
                                children=[
                                    overtake.Ahead("Overtake Ahead?"),
                                    Sequence(
                                        "Overtake Actions",
                                        memory=True,
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
                    cruise.Cruise("Cruise", node.curr_behavior_pub),
                ],
            )
        ],
    )

    root = Parallel(
        "Root",
        policy=ParallelPolicy.SuccessOnOne(),
        children=[
            debug_markers.DebugMarkerBlackboardSetupBehavior(),
            speed_alteration.SpeedAlterationSetupBehavior(),
            topics2blackboard.create_node(role_name, callback_group=callback_group),
            rules,
            speed_alteration.SpeedAlterationRequestBehavior(
                node.speed_alteration_client
            ),
            debug_markers.DebugMarkerBlackboardPublishBehavior(
                node.get_clock(), node.marker_publisher, node.info_publisher
            ),
            Running("Idle"),
        ],
    )
    return root


class BehaviorTree(Node):

    def __init__(self):
        super().__init__("BehaviorTree")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        mapping_common.set_logger(self.get_logger())
        behaviors.set_logger(self.get_logger())

        self.blackboard = py_trees.blackboard.Blackboard()
        self.client_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # Parameters
        self.control_loop_rate = (
            self.declare_parameter(
                "control_loop_rate",
                1.0 / 5.3,
            )
            .get_parameter_value()
            .double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        register_parameters(self)

        # Publishers
        self.curr_behavior_pub = self.create_publisher(
            String, "/paf/hero/curr_behavior", 1
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/paf/hero/behavior_tree/debug_markers", 1
        )
        self.info_publisher = self.create_publisher(
            Marker, "/paf/hero/behavior_tree/info_marker", 1
        )

        # Service clients
        self.speed_alteration_client = self.create_client(
            SpeedAlteration,
            "/paf/hero/acc/speed_alteration",
            callback_group=self.client_callback_group,
        )
        self.stop_marks_client = self.create_client(
            UpdateStopMarks,
            "/paf/hero/mapping/update_stop_marks",
            callback_group=self.client_callback_group,
        )
        self.start_overtake_client = self.create_client(
            StartOvertake,
            "/paf/hero/motion_planning/start_overtake",
            callback_group=self.client_callback_group,
        )
        self.end_overtake_client = self.create_client(
            EndOvertake,
            "/paf/hero/motion_planning/end_overtake",
            callback_group=self.client_callback_group,
        )
        self.overtake_status_client = self.create_client(
            OvertakeStatus,
            "/paf/hero/motion_planning/overtake_status",
            callback_group=self.client_callback_group,
        )
        for client in self.clients:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"Waiting for the {client.service_name} service "
                    "to become available..."
                )

        root = grow_a_tree(self.role_name, self, callback_group=None)
        self.behavior_tree = py_trees_ros.trees.BehaviourTree(root)

        if not self.behavior_tree.setup(node=self, timeout=15):
            self.get_logger().fatal("Behavior tree Setup failed.")
            sys.exit(1)

        self.loop_timer = self.create_timer(
            self.control_loop_rate, self.tick_tree_handler
        )
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Callback for parameter updates.

        Updates the node attributes ["role_name", "control_loop_rate"] and
        puts all parameters into the blackboard.

        Parameters are available as "/params/*parameter_name*"
        """
        blackboard_error_reason = None
        node_parameters: List[Parameter] = []
        for param in params:
            if param.name in ["role_name", "control_loop_rate"]:
                node_parameters.append(param)

            # Sets parameter in the blackboard
            blackboard_key = f"/params/{param.name}"
            error_reason = None
            if self.blackboard.exists(blackboard_key):
                new_value = param.value
                orig_value = self.blackboard.get(blackboard_key)
                if (
                    orig_value is not None
                    and not isinstance(new_value, type(orig_value))
                    and not isinstance(orig_value, type(new_value))
                ):
                    error_reason = "type mismatch"
                else:
                    self.blackboard.set(blackboard_key, new_value)
                    self.get_logger().info(
                        f"Updated blackboard entry {blackboard_key} to {new_value}"
                    )
            else:
                error_reason = "parameter not found in blackboard"

            if error_reason is not None:
                blackboard_error_reason = error_reason
                self.get_logger().warn(
                    f"Failed to update parameter {param.name}: "
                    f"{blackboard_error_reason}"
                )

        result = update_attributes(self, node_parameters)
        if blackboard_error_reason is not None:
            result.successful = False
            result.reason = blackboard_error_reason
        return result

    def tick_tree_handler(self):
        try:
            self.tick_tree()
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    def tick_tree(self):
        self.behavior_tree.tick()

    def shutdown(self):
        self.behavior_tree.interrupt()
        self.behavior_tree.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Executor with exactly two threads
    # - One for the behavior tree tick timer
    # - One for internal ros callback
    # Note that this thread split is not enforced, but the two threads
    #   are necessary to not deadlock the node when issuing service calls
    # IMPORTANT: services must only be called
    #   from inside the timer callback -> from inside the behaviours
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)

    try:
        node = BehaviorTree()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
