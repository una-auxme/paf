#!/usr/bin/env python
import math

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer
from rospy import Publisher, Subscriber
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32
from simple_pid import PID

PURE_PURSUIT_CONTROLLER: int = 1
STANLEY_CONTROLLER: int = 2
STANLEY_CONTROLLER_MIN_V: float = 4.0  # ~14kph

MAX_STEER_ANGLE: float = 0.75


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
        super(VehicleController, self).__init__('vehicle_controller')
        self.loginfo('VehicleController node started')
        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        # Publisher for Carla Vehicle Control Commands
        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{self.role_name}/vehicle_control_cmd',
            qos_profile=10
        )

        # Publisher for Status TODO: Where needed? Why carla?
        self.status_pub: Publisher = self.new_publisher(
            Bool,
            f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # Publisher for which steering-controller is mainly used
        # 1 = PurePursuit and 2 = Stanley
        self.controller_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/controller",
            qos_profile=QoSProfile(depth=10,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # Publisher for emergency message TODO: should VC really trigger this?
        self.emergency_pub: Publisher = self.new_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=QoSProfile(depth=10,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        # Subscriber for emergency TODO: who can trigger this, what happens?
        self.emergency_sub: Subscriber = self.new_subscription(
            Bool,
            f"/paf/{self.role_name}/emergency",
            self.__emergency_break,
            qos_profile=QoSProfile(depth=10,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_velocity,
            qos_profile=1)

        self.throttle_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/throttle",
            self.__set_throttle,
            qos_profile=1)

        self.pure_pursuit_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            self.__set_pure_pursuit_steer,
            qos_profile=1)

        self.stanley_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/stanley_steer",
            self.__set_stanley_steer,
            qos_profile=1)

        # Testing / Debugging -->
        self.brake_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/brake",
            self.__set_brake,
            qos_profile=1)

        self.target_steering_publisher: Publisher = self.new_publisher(
            Float32,
            f'/paf/{self.role_name}/target_steering_debug',
            qos_profile=1)

        self.pidpoint_publisher: Publisher = self.new_publisher(
            Float32,
            f'/paf/{self.role_name}/pid_point_debug',
            qos_profile=1)

        self.controller_selector_sub: Subscriber = self.new_subscription(
            Float32,
            f'/paf/{self.role_name}/controller_selector_debug',
            self.__set_controller,
            qos_profile=1)
        # <-- Testing / Debugging

        self.__emergency: bool = False
        self.__throttle: float = 0.0
        self.__velocity: float = 0.0
        self.__pure_pursuit_steer: float = 0.0
        self.__stanley_steer: float = 0.0
        self.__current_steer: float = 0.0

        self.__brake: float = 0.0

        self.controller_testing: bool = False
        self.controller_selected_debug: int = 1
        # TODO: check emergency behaviour

    def run(self):
        """
        Starts the main loop of the node and send a status msg.
        :return:
        """
        self.status_pub.publish(True)
        self.loginfo('VehicleController node running')
        # currently pid for steering is not used, needs fixing
        # or maybe deletion since it is not that useful
        pid = PID(0.5, 0.001, 0)  # PID(0.5, 0.1, 0.1, setpoint=0)
        # TODO: TUNE AND FIX?
        pid.output_limits = (-MAX_STEER_ANGLE, MAX_STEER_ANGLE)

        def loop(timer_event=None) -> None:
            """
            Collects all data received and sends a CarlaEgoVehicleControl msg.
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__emergency:  # emergency is already handled in
                # __emergency_break()
                return
            if (self.controller_testing):
                if (self.controller_selected_debug == 2):
                    p_stanley = 1
                elif (self.controller_selected_debug == 1):
                    p_stanley = 0
            else:
                p_stanley = self.__choose_controller()

            if p_stanley < 0.5:
                self.logdebug('Using PURE_PURSUIT_CONTROLLER')
                self.controller_pub.publish(float(PURE_PURSUIT_CONTROLLER))
            elif p_stanley >= 0.5:
                self.logdebug('Using STANLEY_CONTROLLER')
                self.controller_pub.publish(float(STANLEY_CONTROLLER))

            f_stanley = p_stanley * self.__stanley_steer
            f_pure_p = (1 - p_stanley) * self.__pure_pursuit_steer
            steer = f_stanley + f_pure_p

            # only use pure_pursuit controller for now, since
            # stanley seems broken with the new heading-bug
            # TODO: swap back if stanley is fixed
            # steer = self.__pure_pursuit_steer

            self.target_steering_publisher.publish(steer)  # debugging

            message = CarlaEgoVehicleControl()
            message.reverse = False
            message.throttle = self.__throttle
            message.brake = self.__brake
            message.hand_brake = False
            message.manual_gear_shift = False
            message.steer = self.__map_steering(steer)
            # Original Code:
            # pid.setpoint = self.__map_steering(steer)
            # message.steer = pid(self.__current_steer)
            message.gear = 1
            message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                         from_sec=True)
            self.control_publisher.publish(message)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __map_steering(self, steering_angle: float) -> float:
        """
        Takes the steering angle calculated by the controller and maps it to
        the available steering angle
        This is from (left, right): [pi/2 , -pi/2] to [-1, 1]
        :param steering_angle: calculated by a controller in [-pi/2 , pi/2]
        :return: float for steering in [-1, 1]
        """
        r = - 1 / (math.pi / 2)  # -1 to invert for carla steering
        steering_float = steering_angle * r  # No Tuning needed * tune_k
        self.pidpoint_publisher.publish(steering_float)  # TODO needed?
        return steering_float

    def __emergency_break(self, data) -> None:
        """
        Executes emergency stop
        :param data:
        :return:
        """
        if not data.data:  # not an emergency
            return
        if self.__emergency:  # emergency was already triggered
            return
        self.logerr("Emergency breaking engaged")
        self.__emergency = True
        message = CarlaEgoVehicleControl()
        message.throttle = 1
        message.steer = 1  # todo: maybe use 30 degree angle
        message.brake = 1
        message.reverse = True
        message.hand_brake = True
        message.manual_gear_shift = False
        message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                     from_sec=True)
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
            self.__emergency = False
            message = CarlaEgoVehicleControl()
            message.throttle = 0
            message.steer = 0
            message.brake = 1
            message.reverse = False
            message.hand_brake = False
            message.manual_gear_shift = False
            message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                         from_sec=True)
            self.control_publisher.publish(message)

            self.loginfo("Emergency breaking disengaged "
                         "(Emergency breaking has been executed successfully)")
            for _ in range(7):  # publish 7 times just to be safe
                self.emergency_pub.publish(
                    Bool(False))

    def __set_throttle(self, data):
        self.__throttle = data.data

    def __set_brake(self, data):
        self.__brake = data.data

    def __set_pure_pursuit_steer(self, data: Float32):
        self.__pure_pursuit_steer = data.data

    def __set_stanley_steer(self, data: Float32):
        self.__stanley_steer = data.data

    def __set_controller(self, data: Float32):
        self.controller_testing = True
        self.controller_selected_debug = data.data

    def sigmoid(self, x: float):
        """
        Evaluates the sigmoid function s(x) = 1 / (1+e^-25x)
        :param x: x
        :return: s(x) = 1 / (1+e^-25x)
        """
        temp_x = min(-25 * x, 25)
        res = 1 / (1 + math.exp(temp_x))
        return res

    def __choose_controller(self) -> float:
        """
        Returns the proportion of stanley to use.
        Publishes the currently used controller
        :return:
        """
        res = self.sigmoid(self.__velocity - STANLEY_CONTROLLER_MIN_V)
        return res


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('vehicle_controller', args=args)

    try:
        node = VehicleController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
