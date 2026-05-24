import math
import queue
import threading

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaGnssRoute, CarlaRoute
from carla_msgs.srv import DestroyObject, SpawnObject
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from leaderboard.autoagents.ros2_agent import ROS2Agent
from leaderboard.autoagents.ros_base_agent import ROSLauncher

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


def get_entry_point():
    return "PAFAgent"


class PAFAgent(ROS2Agent):
    def __init__(self, carla_host, carla_port, debug=False):
        if rclpy.ok():
            rclpy.shutdown()

        try:
            AutonomousAgent.__init__(self, carla_host, carla_port, debug)

            self._bridge_process = ROSLauncher(
                "bridge", ros_version=self.ROS_VERSION, debug=debug
            )
            self._bridge_process.run(
                package="carla_ros_bridge",
                launch_file="carla_ros_bridge.launch.py",
                parameters={
                    "host": carla_host,
                    "port": carla_port,
                    "timeout": 60,
                    "synchronous_mode": True,
                    "synchronous_mode_wait_for_vehicle_control_command": True,
                    "passive": True,
                    "register_all_sensors": False,
                    "ego_vehicle_role_name": "\"['hero']\"",
                },
                wait=True,
            )

            self._agent_process = ROSLauncher(
                "agent", ros_version=self.ROS_VERSION, debug=debug
            )
            self._agent_process.run(**self.get_ros_entrypoint(), wait=True)

            self._control_queue = queue.Queue(1)
            self._last_control_timestamp = None

            rclpy.init(args=None)
            self.ros_node = rclpy.create_node("leaderboard_node")

            self._spawn_object_service = self.ros_node.create_client(
                SpawnObject, "/carla/spawn_object"
            )
            self._destroy_object_service = self.ros_node.create_client(
                DestroyObject, "/carla/destroy_object"
            )

            self._path_publisher = self.ros_node.create_publisher(
                CarlaRoute,
                "/carla/hero/global_plan",
                qos_profile=QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )
            self._path_gnss_publisher = self.ros_node.create_publisher(
                CarlaGnssRoute,
                "/carla/hero/global_plan_gnss",
                qos_profile=QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )

            self.ctrl_subscriber = self.ros_node.create_subscription(
                CarlaEgoVehicleControl,
                "/carla/hero/vehicle_control_cmd",
                self._vehicle_control_cmd_callback,
                qos_profile=QoSProfile(depth=1),
            )

            self.spin_thread = threading.Thread(
                target=rclpy.spin, args=(self.ros_node,)
            )
            self.spin_thread.start()
        except Exception:
            self._cleanup_partial_ros2_setup()
            raise

    def setup(self, path_to_conf_file):
        self.track = Track.MAP

    def get_ros_entrypoint(self):
        raise NotImplementedError()

    def sensors(self):
        sensors = [
            {
                "type": "sensor.camera.rgb",
                "id": "Center",
                "x": 0.0,
                "y": 0.0,
                "z": 1.70,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "width": 1280,
                "height": 720,
                "fov": 100,
            },
            {
                "type": "sensor.lidar.ray_cast",
                "id": "LIDAR",
                "x": 0.0,
                "y": 0.0,
                "z": 1.70,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            {
                "type": "sensor.other.radar",
                "id": "RADAR0",
                "x": 2.0,
                "y": -1.5,
                "z": 0.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "horizontal_fov": 130,
                "vertical_fov": 0.1,
            },
            {
                "type": "sensor.other.radar",
                "id": "RADAR1",
                "x": -2.0,
                "y": -1.5,
                "z": 0.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": math.radians(180),
                "horizontal_fov": 130,
                "vertical_fov": 0.1,
            },
            {"type": "sensor.other.gnss", "id": "GPS", "x": 0.0, "y": 0.0, "z": 0.0},
            {
                "type": "sensor.other.imu",
                "id": "IMU",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "noise_accel_stddev_x": 0.0,
                "noise_accel_stddev_y": 0.0,
                "noise_accel_stddev_z": 0.0,
                "noise_gyro_stddev_x": 0.0,
                "noise_gyro_stddev_y": 0.0,
                "noise_gyro_stddev_z": 0.0,
                "noise_gyro_bias_x": 0.0,
                "noise_gyro_bias_y": 0.0,
                "noise_gyro_bias_z": 0.0,
            },
            {"type": "sensor.opendrive_map", "id": "OpenDRIVE", "reading_frequency": 1},
            {"type": "sensor.speedometer", "id": "Speed"},
        ]
        return sensors

    def spawn_object(self, type_, id_, transform, attributes, attach_to=0):
        self._spawn_object_service.wait_for_service()
        return super().spawn_object(type_, id_, transform, attributes, attach_to)

    def destroy_object(self, uid):
        self._destroy_object_service.wait_for_service()
        return super().destroy_object(uid)

    def destroy(self):
        self._cleanup_partial_ros2_setup()

    def _cleanup_partial_ros2_setup(self):
        spin_thread = getattr(self, "spin_thread", None)
        ros_node = getattr(self, "ros_node", None)
        agent_process = getattr(self, "_agent_process", None)
        bridge_process = getattr(self, "_bridge_process", None)

        if ros_node is not None:
            try:
                ros_node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=5.0)

        if agent_process is not None and agent_process.is_alive():
            agent_process.terminate()

        if bridge_process is not None and bridge_process.is_alive():
            bridge_process.terminate()
