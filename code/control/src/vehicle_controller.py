#!/usr/bin/env python
import math

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer
from rospy import Publisher, Subscriber
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32, String


class VehicleController(CompatibleNode):
    """
    This node is responsible for collecting all data needed for the
    vehicle_control_cmd and sending it.
    The node also decides weather to use the stanley or pure pursuit
    controller.
    If the node receives an emergency msg, it will bring the vehicle to a stop
    and send an emergency msg with data = False back, after the velocity of the
    vehicle has reached 0.
    """

    def __init__(self):
        super(VehicleController, self).__init__("vehicle_controller")
        self.loginfo("VehicleController node started")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.__curr_behavior = None  # only unstuck behavior is relevant here

        # Publisher for Carla Vehicle Control Commands
        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=10,
        )

        # Publisher for Status TODO: Maybe unneccessary
        self.status_pub: Publisher = self.new_publisher(
            Bool,
            f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        # Publisher for which steering-controller is mainly used
        # 1 = PurePursuit and 2 = Stanley
        self.controller_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/controller",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.emergency_pub: Publisher = self.new_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        # Subscribers
        self.curr_behavior_sub: Subscriber = self.new_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )

        self.emergency_sub: Subscriber = self.new_subscription(
            Bool,
            f"/paf/{self.role_name}/emergency",
            self.__set_emergency,
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_velocity,
            qos_profile=1,
        )

        self.throttle_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/throttle",
            self.__set_throttle,
            qos_profile=1,
        )

        self.brake_sub: Subscriber = self.new_subscription(
            Float32, f"/paf/{self.role_name}/brake", self.__set_brake, qos_profile=1
        )

        self.reverse_sub: Subscriber = self.new_subscription(
            Bool, f"/paf/{self.role_name}/reverse", self.__set_reverse, qos_profile=1
        )

        self.pure_pursuit_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            self.__set_pure_pursuit_steer,
            qos_profile=1,
        )

        self.stanley_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/stanley_steer",
            self.__set_stanley_steer,
            qos_profile=1,
        )

        self.__reverse: bool = False
        self.__emergency: bool = False
        self.__velocity: float = 0.0
        self.__brake: float = 0.0
        self.__throttle: float = 0.0
        self._p_steer: float = 0.0
        self._s_steer: float = 0.0

    def run(self):
        """
        Starts the main loop of the node and send a status msg.
        :return:
        """
        self.status_pub.publish(True)
        self.loginfo("VehicleController node running")

        def loop(timer_event=None) -> None:
            """
            Collects all data received and sends a CarlaEgoVehicleControl msg.
            The Leaderboard expects a msg every 0.05 seconds
            OTHERWISE IT WILL LAG REALLY BADLY
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__emergency:
                # emergency is already handled in  __emergency_brake()
                self.__emergency_brake(True)
                return

            # Velocities over 5 m/s = use Stanley, else use PurePuresuit
            if self.__velocity > 5:
                steer = self._s_steer
            else:
                # while doing the unstuck routine we don't want to steer
                if (
                    self.__curr_behavior == "us_unstuck"
                    or self.__curr_behavior == "us_stop"
                ):
                    steer = 0
                else:
                    steer = self._p_steer

            message = CarlaEgoVehicleControl()
            message.reverse = self.__reverse
            message.hand_brake = False
            message.manual_gear_shift = False
            message.gear = 1
            message.throttle = self.__throttle
            message.brake = self.__brake
            message.steer = steer
            message.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
            self.control_publisher.publish(message)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __set_curr_behavior(self, data: String) -> None:
        """
        Sets the current behavior
        :param data:
        :return:
        """
        self.__curr_behavior = data.data

    def __set_emergency(self, data) -> None:
        """
        In case of an emergency set the emergency flag to True ONCE
        The emergency flag can be ONLY be set to False if velocity is > 0.1
        by __get_velocity
        :param data:
        :return:
        """
        if not data.data:  # not an emergency
            return
        if self.__emergency:  # emergency was already triggered
            return

        self.logerr("Emergency braking engaged")
        self.__emergency = True

    def __emergency_brake(self, active) -> None:
        """
        Executes emergency stop
        :param data:
        :return:
        """
        if not self.__emergency:
            return
        message = CarlaEgoVehicleControl()
        if active:
            message.throttle = 1
            message.steer = 1
            message.brake = 1
            message.reverse = True
            message.hand_brake = True
            message.manual_gear_shift = False
            message.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        else:
            self.__emergency = False
            message.throttle = 0
            message.steer = 0
            message.brake = 1
            message.reverse = False
            message.hand_brake = False
            message.manual_gear_shift = False
            message.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        self.control_publisher.publish(message)

    def __get_velocity(self, data: CarlaSpeedometer) -> None:
        """
        Ends emergency if vehicle velocity reaches 0. Sends emergency msg
        with data = False after stopping.
        :param data:
        :return:
        """
        self.__velocity = data.speed
        if not self.__emergency:  # nothing to do in this case
            return
        if data.speed < 0.1:  # vehicle has come to a stop
            self.__emergency_brake(False)
            self.loginfo(
                "Emergency breaking disengaged "
                "(Emergency breaking has been executed successfully)"
            )
            for _ in range(7):  # publish 7 times just to be safe
                self.emergency_pub.publish(Bool(False))

    def __set_throttle(self, data):
        self.__throttle = data.data

    def __set_brake(self, data):
        self.__brake = data.data

    def __set_reverse(self, data):
        self.__reverse = data.data

    def __set_pure_pursuit_steer(self, data: Float32):
        r = math.pi / 2  # convert from RAD to [-1;1]
        self._p_steer = data.data / r

    def __set_stanley_steer(self, data: Float32):
        r = math.pi / 2  # convert from RAD to [-1;1]
        self._s_steer = data.data / r


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("vehicle_controller", args=args)

    try:
        node = VehicleController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
