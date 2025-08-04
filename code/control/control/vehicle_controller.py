import math
import time
from typing import List

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
from std_msgs.msg import Bool, Float32, String
from rosgraph_msgs.msg import Clock
from paf_common.parameters import update_attributes


class VehicleController(Node):
    """
    This node is responsible for collecting all data needed for the
    vehicle_control_cmd and sending it.
    The node uses the pure pursuit controller for steering.
    If the node receives an emergency msg, it will bring the vehicle to a stop
    and send an emergency msg with data = False back, after the velocity of the
    vehicle has reached 0.
    INFO: Currently the loop of the node has a sleep command in it. The control
    command triggers the carla simulator to render the next frame. If the loop
    does not have the time to sleep the simulator will run as fast as the system
    allows it to run. If your system is too slow to run with the 0.2 loop_sleep_time
    you could slow it down by setting the loop_sleep_time to a higher value.
    """

    def __init__(self):
        super(VehicleController, self).__init__("vehicle_controller")
        self.get_logger().info("VehicleController node initializing...")

        # Configuration parameters
        self.control_loop_rate = (
            self.declare_parameter("control_loop_rate", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        self.loop_sleep_time = (
            self.declare_parameter(
                "loop_sleep_time",
                0.2,
                descriptor=ParameterDescriptor(
                    description="This sleep time is used to slow down the vehicle "
                    "controller to a reasonable speed",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.05, to_value=0.4, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        # Manual control
        self.manual_override_active = (
            self.declare_parameter(
                "manual_override_active",
                False,
                descriptor=ParameterDescriptor(description="Activate Manual Override"),
            )
            .get_parameter_value()
            .bool_value
        )
        self.manual_steer = (
            self.declare_parameter(
                "manual_steer",
                0.0,
                descriptor=ParameterDescriptor(
                    description="Steering input sent to carla.",
                    floating_point_range=[
                        FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.manual_throttle = (
            self.declare_parameter(
                "manual_throttle",
                0.0,
                descriptor=ParameterDescriptor(
                    description="Steering input sent to carla.",
                    floating_point_range=[
                        FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        # State variables
        self.__curr_behavior = None
        self.__reverse = False
        self.__emergency = False
        self.__brake = 0.0
        self.__throttle = 0.0
        self._p_steer = 0.0

        # Initialize publishers
        self.control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=10,
        )
        self.status_pub = self.create_publisher(
            Bool,
            f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.controller_pub = self.create_publisher(
            Float32,
            f"/paf/{self.role_name}/controller",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
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

        # Control message
        self.message = CarlaEgoVehicleControl()

        # Periodically send out the status signal,
        # because otherwise, the leaderboard does not start the simulation.
        # This has to use system time, because the leaderboard
        # only sends out clock signals AFTER the simulation has started.
        system_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME)
        self.create_timer(0.5, self.publish_status, clock=system_clock)

        self.clock_sub = self.create_subscription(Clock, "/clock", self.loop_handler, 1)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info("VehicleController node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def update_control_message(self):
        """Update the control message based on the current state."""

        if self.manual_override_active:
            self.message.reverse = self.manual_throttle < 0
            self.message.throttle = self.manual_throttle
            self.message.steer = self.manual_steer
            self.message.brake = 0.0
            self.message.hand_brake = False
        elif self.__emergency:
            self.__emergency_brake(True)
        else:
            steer = (
                self._p_steer * (-1)
                if self.__curr_behavior == "us_unstuck"
                else self._p_steer
            )

            self.message.reverse = self.__reverse
            self.message.throttle = self.__throttle
            self.message.brake = self.__brake
            self.message.steer = steer
            self.message.hand_brake = False
            self.message.manual_gear_shift = False

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
            self.__emergency_brake(False)
            for _ in range(7):
                self.emergency_pub.publish(Bool(False))
            self.get_logger().info("Emergency braking disengaged")

    def __set_throttle(self, data: Float32):
        self.__throttle = data.data

    def __set_brake(self, data: Float32):
        self.__brake = data.data

    def __set_reverse(self, data: Bool):
        self.__reverse = data.data

    def __set_pure_pursuit_steer(self, data: Float32):
        self._p_steer = data.data / (math.pi / 2)

    def __emergency_brake(self, active: bool):
        if active:
            self.message.throttle = 0.0
            self.message.steer = 0.0
            self.message.brake = 1.0
            self.message.reverse = False
            self.message.hand_brake = True
        else:
            self.__emergency = False
            self.message.brake = 0.0
            self.message.hand_brake = False

    def publish_status(self):
        self.status_pub.publish(Bool(data=True))

    def loop(self, clock: Clock):
        """Main control loop"""
        self.update_control_message()
        self.message.header.stamp = rclpy.time.Time(
            seconds=clock.clock.sec, nanoseconds=clock.clock.nanosec
        ).to_msg()
        self.control_publisher.publish(self.message)

        time.sleep(self.loop_sleep_time)

    def loop_handler(self, clock: Clock):
        try:
            self.loop(clock)
        except Exception as e:
            self.get_logger().fatal(e)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VehicleController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
