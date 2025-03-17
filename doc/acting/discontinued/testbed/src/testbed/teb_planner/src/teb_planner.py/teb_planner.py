#!/usr/bin/env python


import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point32
from nav_msgs.msg import Path

from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from teb_planner_pa_msgs.srv import PlanRequest, PlanResponse, Plan


import numpy as np
from sim.msg import (
    Vector2D,
    VisPath,
    MultiPath,
)

from mapping.msg import Map as MapMsg
from mapping_common.entity import Entity
from mapping_common.map import Map

from acting.trajectory_modifier import TrajectoryModifier

from typing import Tuple, List, Optional
from numpy.typing import NDArray


class TebPlanner(CompatibleNode, TrajectoryModifier):
    multi_path: List[List[Tuple[float, float]]] = []

    def __init__(self):
        CompatibleNode.__init__(self, "teb_planner")
        TrajectoryModifier.__init__(self, self)

        self.multi_path_pub = self.new_publisher(
            msg_type=MultiPath, topic="multi_path", qos_profile=10
        )

        self.new_subscription(
            msg_type=PoseStamped,
            topic="paf/hero/current_pos",
            callback=self._position_callback,
            qos_profile=10,
        )

        self.new_subscription(
            msg_type=Float32,
            topic="paf/hero/current_heading",
            callback=self._heading_callback,
            qos_profile=10,
        )

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self._map_callback,
            qos_profile=1,
        )
        self.plan_service = rospy.ServiceProxy(
            name="/teb_planner_node_pa/plan", service_class=Plan
        )

        self.car_position: Tuple[float, float] = (0, 0)
        self.car_theta: float = 0.0

        self.map: Optional[Map] = None

        rate_hz = 20
        self.dt = 1.0 / rate_hz
        self.new_timer(self.dt, lambda _: self._update())

    def _map_callback(self, map_msg: MapMsg):
        self.map = Map.from_ros_msg(map_msg)

    def _update(self):
        self.show_multi_path(self.multi_path)
        self.multi_path = []

    def _position_callback(self, msg: PoseStamped):
        position = msg.pose.position
        self.car_position = (position.x, position.y)

    def _heading_callback(self, msg: Float32):
        self.car_theta = msg.data

    def show_multi_path(self, multi_path: List[List[Tuple[float, float]]]) -> None:
        msg = MultiPath()
        for vp in multi_path:
            vis_path = VisPath()
            for p in vp:
                vis_path.points.append(Vector2D(x=p[0], y=p[1]))

            msg.paths.append(vis_path)
        self.multi_path_pub.publish(msg)

    def _car_to_world(self, x: float, y: float):
        car_x, car_y = self.car_position
        car_phi = self.car_theta
        translation = np.array([car_x, car_y])
        rotation_matrix = np.array(
            [[np.cos(car_phi), -np.sin(car_phi)], [np.sin(car_phi), np.cos(car_phi)]]
        )
        car_position = np.array([x, y])
        world_position = (
            rotation_matrix @ car_position + translation
        )  # Rotate and then translate
        return world_position

    # overrides superclass
    def modify_path(self, positions: NDArray) -> bool:
        if self.map is None:
            return positions

        req = PlanRequest()
        start = Pose()
        start.position.x, start.position.y = self.car_position
        start.orientation.z = 1.0  # ???
        req.request.start = start

        goal = Pose()
        goal.position.x, goal.position.y = positions[-1]
        goal.orientation.z = 1.0  # ???
        req.request.goal = goal

        for position in positions:
            pos = PoseStamped()
            pos.pose.position.x, pos.pose.position.y = position
            pos.pose.orientation.z = 1.0  # ???
            req.request.initial_plan.poses.append(pos)

        obst = ObstacleArrayMsg()
        obstacles = self.map.entities_without_hero()
        for obstacle in obstacles:
            ob = ObstacleMsg()
            x = obstacle.transform.translation().x()
            y = obstacle.transform.translation().y()
            x, y = self._car_to_world(x, y)
            ob.polygon.points.append(Point32(x=x, y=y))
            ob.orientation.w = 1
            obst.obstacles.append(ob)
        req.request.obstacles = obst

        response: PlanResponse = self.plan_service.call(req)

        self.multi_path.append(
            [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in response.respond.path.poses
            ]
        )
        positions = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in response.respond.path.poses
            ]
        )

        return positions


if __name__ == "__main__":
    roscomp.init("pot_field")
    teb = TebPlanner()
    teb.spin()
