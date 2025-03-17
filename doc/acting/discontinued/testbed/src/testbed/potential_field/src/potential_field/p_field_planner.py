#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from sim.msg import (
    Vector2D,
    VisPath,
    MultiPath,
)
from mapping.msg import Map as MapMsg
from mapping_common.entity import Entity
from mapping_common.map import Map

from acting.trajectory_modifier import TrajectoryModifier
import numpy as np

from typing import Tuple, List, Optional
from numpy.typing import NDArray


class PotentialField(CompatibleNode, TrajectoryModifier):
    multi_path: List[List[Tuple[float, float]]] = []

    def __init__(self):
        CompatibleNode.__init__(self, "potential_field")
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

        self.car_position: Tuple[float, float] = (0, 0)
        self.car_theta: float = 0.0

        self.map: Optional[Map] = None

        rate_hz = 20
        self.dt = 1.0 / rate_hz
        self.new_timer(self.dt, lambda _: self._update())

    def _map_callback(self, map_msg: MapMsg):
        self.map = Map.from_ros_msg(map_msg)

    def _update(self):
        force_x, force_y = self.force_vector(0, 0)  # car is at 0,0 for this
        w_f_x, w_f_y = self._car_to_world(force_x, force_y)
        path: List[Tuple[float, float]] = [
            self.car_position,
            (w_f_x, w_f_y),
        ]
        self.multi_path.append(path)
        self.show_multi_path(self.multi_path)
        self.multi_path = []

    def _position_callback(self, msg: PoseStamped):
        position = msg.pose.position
        self.car_position = (position.x, position.y)

    def _heading_callback(self, msg: Float32):
        self.car_theta = msg.data

    def force_vector(self, x: float, y: float) -> Tuple[float, float]:
        cumulative = np.zeros((2))
        if self.map is None:
            return cumulative

        obstacles = self.map.entities_without_hero()
        for obstacle in obstacles:
            distance = self._distance(x, y, obstacle)
            direction = self._direction_towards(x, y, obstacle)
            if distance == 0:
                continue
            cumulative += 2 * direction * 1 / distance
        return cumulative

    def show_multi_path(self, multi_path: List[List[Tuple[float, float]]]) -> None:
        msg = MultiPath()
        for vp in multi_path:
            vis_path = VisPath()
            for p in vp:
                vis_path.points.append(Vector2D(x=p[0], y=p[1]))

            msg.paths.append(vis_path)
        self.multi_path_pub.publish(msg)

    def _distance(self, x: float, y: float, ent: Entity):
        pos1 = np.array([x, y])
        x = ent.transform.translation().x()
        y = ent.transform.translation().y()
        pos2 = self._car_to_world(x, y)
        return np.linalg.norm(pos1 - pos2)

    def _direction_towards(self, x: float, y: float, ent: Entity):
        pos1 = np.array([x, y])
        x = ent.transform.translation().x()
        y = ent.transform.translation().y()
        pos2 = self._car_to_world(x, y)
        d: float = np.linalg.norm(pos2 - pos1)
        if d == 0.0:
            return pos2 - pos1
        return (pos2 - pos1) / d

    def _car_to_world(self, x: float, y: float) -> NDArray:
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
    def modify_path(self, positions: NDArray) -> NDArray:
        for i, position in enumerate(positions):
            x, y = position
            force = self.force_vector(x, y)
            positions[i] -= force

            self.multi_path.append([(x, y), (positions[i][0], positions[i][1])])

        self.multi_path.append([(position[0], position[1]) for position in positions])

        return positions


if __name__ == "__main__":
    roscomp.init("pot_field")

    pf = PotentialField()
    pf.spin()
