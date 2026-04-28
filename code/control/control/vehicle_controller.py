import math
import time
from typing import List, Optional

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer
import rclpy
import rclpy.clock
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
)
from std_msgs.msg import Bool, Float32, String, UInt64
from rosgraph_msgs.msg import Clock
from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace
from paf_common.sync import (
    FrameBarrier,
    frame_complete_topic,
    frame_id_from_time_ns,
    startup_topic,
)


DEFAULT_REQUIRED_SYNC_STAGES = [
    "mapping",
    "motion_planning",
    "acc",
    "pure_pursuit",
    "velocity_controller",
]


class VehicleController(Node):
    """
    This node is responsible for collecting all data needed for the
    vehicle_control_cmd and sending it only after the required stages
    completed the current simulation frame.
    The node uses the pure pursuit controller for steering.
    If the node receives an emergency msg, it will bring the vehicle to a stop
    and send an emergency msg with data = False back, after the velocity of the
    vehicle has reached 0.
    """

    def __init__(self):
        super().__init__("vehicle_controller")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Configuration parameters
        self.control_loop_rate = self.declare_parameter("control_loop_rate", 0.05).value
        self.role_name = self.declare_parameter("role_name", "hero").value
        self.sync_frame_delta_seconds = self.declare_parameter(
            "sync_frame_delta_seconds", 0.05
        ).value
        self.frame_barrier_timeout = self.declare_parameter(
            "frame_barrier_timeout",
            1.0,
            descriptor=ParameterDescriptor(
                description="Max wall-clock time to wait for the current frame's "
                "stage completions before publishing a safe stop command.",
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)
                ],
            ),
        ).value
        self.required_sync_stages = self.declare_parameter(
            "required_sync_stages", DEFAULT_REQUIRED_SYNC_STAGES
        ).value
        # Manual control
        self.manual_override_active = self.declare_parameter(
            "manual_override_active",
            False,
            descriptor=ParameterDescriptor(description="Activate Manual Override"),
        ).value
        self.manual_steer = self.declare_parameter(
            "manual_steer",
            0.0,
            descriptor=ParameterDescriptor(
                description="Steering input sent to carla.",
                floating_point_range=[
                    FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.01)
                ],
            ),
        ).value
        self.manual_throttle = self.declare_parameter(
            "manual_throttle",
            0.0,
            descriptor=ParameterDescriptor(
                description="Throttle input sent to carla.",
                floating_point_range=[
                    FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.01)
                ],
            ),
        ).value

        # State variables
        self.__curr_behavior = None
        self.__reverse = False
        self.__emergency = False
        self.__brake = 0.0
        self.__throttle = 0.0
        self._p_steer = 0.0
        self.frame_barrier = FrameBarrier(self.required_sync_stages)
        self.pending_clock: Optional[Clock] = None
        self.last_published_frame_id: int = -1

        # Initialize publishers
        self.control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=10,
        )
        self.controller_pub = self.create_publisher(
            Float32,
            f"/paf/{self.role_name}/controller",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.startup_ready_pub = self.create_publisher(
            Bool,
            startup_topic(self.role_name, "vehicle_controller"),
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.emergency_pub = self.create_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        # Initialize subscribers
        self.create_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )
        self.create_subscription(
            Bool,
            f"/paf/{self.role_name}/emergency",
            self.__set_emergency,
            qos_profile=1,
        )
        self.create_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_velocity,
            qos_profile=1,
        )
        self.create_subscription(
            Float32,
            f"/paf/{self.role_name}/throttle",
            self.__set_throttle,
            qos_profile=1,
        )
        self.create_subscription(
            Float32, f"/paf/{self.role_name}/brake", self.__set_brake, qos_profile=1
        )
        self.create_subscription(
            Bool, f"/paf/{self.role_name}/reverse", self.__set_reverse, qos_profile=1
        )
        self.create_subscription(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            self.__set_pure_pursuit_steer,
            qos_profile=1,
        )

        self.clock_sub = self.create_subscription(Clock, "/clock", self.loop_handler, 1)
        for stage_id in self.frame_barrier.required_stages:
            self.create_subscription(
                UInt64,
                frame_complete_topic(self.role_name, stage_id),
                lambda msg, current_stage_id=stage_id: self._stage_complete_callback(
                    current_stage_id, msg
                ),
                qos_profile=1,
            )
        system_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME)
        self.create_timer(0.05, self._frame_timeout_handler, clock=system_clock)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.startup_ready_pub.publish(Bool(data=True))
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def build_control_message(self) -> CarlaEgoVehicleControl:
        """Build the control message based on the current state."""
        message = CarlaEgoVehicleControl()

        if self.manual_override_active:
            message.reverse = self.manual_throttle < 0
            message.throttle = abs(self.manual_throttle)
            message.steer = self.manual_steer
            message.brake = 0.0
            message.hand_brake = False
        elif self.__emergency:
            self._apply_emergency_brake(message)
        else:
            steer = (
                self._p_steer
                if self.__curr_behavior == "us_unstuck"
                else -self._p_steer
            )

            message.reverse = self.__reverse
            message.throttle = self.__throttle
            message.brake = self.__brake
            message.steer = steer
            message.hand_brake = False
            message.manual_gear_shift = False

        return message

    # Subscriber callbacks
    def __set_curr_behavior(self, data: String):
        self.__curr_behavior = data.data

    def __set_emergency(self, data: Bool):
        if data.data and not self.__emergency:
            self.get_logger().error("Emergency braking engaged")
            self.__emergency = True

    def __get_velocity(self, data: CarlaSpeedometer):
        self.__velocity = data.speed
        if self.__emergency and data.speed < 0.1:
            self.__emergency = False
            for _ in range(7):
                self.emergency_pub.publish(Bool(data=False))
            self.get_logger().info("Emergency braking disengaged")

    def __set_throttle(self, data: Float32):
        self.__throttle = data.data

    def __set_brake(self, data: Float32):
        self.__brake = data.data

    def __set_reverse(self, data: Bool):
        self.__reverse = data.data

    def __set_pure_pursuit_steer(self, data: Float32):
        self._p_steer = data.data / (math.pi / 2)

    def _apply_emergency_brake(self, message: CarlaEgoVehicleControl) -> None:
        message.throttle = 0.0
        message.steer = 0.0
        message.brake = 1.0
        message.reverse = False
        message.hand_brake = True

    def _build_safe_stop_message(self) -> CarlaEgoVehicleControl:
        message = CarlaEgoVehicleControl()
        message.throttle = 0.0
        message.steer = 0.0
        message.brake = 1.0
        message.reverse = False
        message.hand_brake = True
        message.manual_gear_shift = False
        return message

    def loop(self, clock: Clock):
        """Begin a new pending simulation frame and wait for stage completion."""
        clock_ns = clock.clock.sec * 1_000_000_000 + clock.clock.nanosec
        frame_id = frame_id_from_time_ns(clock_ns, self.sync_frame_delta_seconds)
        if frame_id <= self.last_published_frame_id:
            return

        self.pending_clock = clock
        self.frame_barrier.begin_frame(frame_id, time.monotonic())
        self._try_publish_pending_frame()

    def _stage_complete_callback(self, stage_id: str, data: UInt64) -> None:
        self.frame_barrier.mark_stage_complete(stage_id, int(data.data))
        self._try_publish_pending_frame()

    def _try_publish_pending_frame(self) -> None:
        pending_frame_id = self.frame_barrier.pending_frame_id
        if pending_frame_id is None or self.pending_clock is None:
            return
        if not self.frame_barrier.is_ready(pending_frame_id):
            return

        self._publish_control_message(self.build_control_message(), self.pending_clock)

    def _frame_timeout_handler(self) -> None:
        pending_frame_id = self.frame_barrier.pending_frame_id
        if pending_frame_id is None or self.pending_clock is None:
            return
        if not self.frame_barrier.timed_out(
            time.monotonic(), self.frame_barrier_timeout
        ):
            return

        missing_stages = self.frame_barrier.missing_stages(pending_frame_id)
        self.get_logger().warn(
            "Frame barrier timeout for frame "
            f"{pending_frame_id}: missing stages {missing_stages}. "
            "Publishing safe stop command.",
            throttle_duration_sec=1.0,
        )
        self._publish_control_message(
            self._build_safe_stop_message(),
            self.pending_clock,
        )

    def _publish_control_message(
        self, message: CarlaEgoVehicleControl, clock: Clock
    ) -> None:
        message.header.stamp = rclpy.time.Time(
            seconds=clock.clock.sec, nanoseconds=clock.clock.nanosec
        ).to_msg()
        self.control_publisher.publish(message)
        if self.frame_barrier.pending_frame_id is not None:
            self.last_published_frame_id = self.frame_barrier.pending_frame_id
        self.frame_barrier.clear_pending()
        self.pending_clock = None

    def loop_handler(self, clock: Clock):
        try:
            self.loop(clock)
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VehicleController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
