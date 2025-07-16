from typing import Optional
from carla_msgs.msg import CarlaSpeedometer
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from simple_pid import PID
from std_msgs.msg import Float32, Bool


class VelocityController(Node):
    """
    This node controls the velocity of the vehicle.
    For this it uses a PID controller.
    """

    def __init__(self):
        super().__init__("velocity_controller")
        self.get_logger().info("VelocityController node initializing...")

        self.control_loop_rate_param = self.declare_parameter("control_loop_rate", 0.05)
        self.control_loop_rate = (
            self.control_loop_rate_param.get_parameter_value().double_value
        )
        self.role_name_param = self.declare_parameter("role_name", "ego_vehicle")
        self.role_name = self.role_name_param.get_parameter_value().string_value

        self.FIXED_SPEED_param = self.declare_parameter("fixed_speed", 0.0)
        self.FIXED_SPEED_OVERRIDE_param = self.declare_parameter(
            "fixed_speed_active", False
        )

        self.pid_p_param = self.declare_parameter("pid_p", 0.60)
        self.pid_i_param = self.declare_parameter("pid_i", 0.00076)
        self.pid_d_param = self.declare_parameter("pid_d", 0.63)

        self.target_velocity_sub: Subscription = self.create_subscription(
            Float32,
            "/paf/acting/target_velocity",
            self.__get_target_velocity,
            qos_profile=1,
        )

        self.velocity_sub: Subscription = self.create_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1,
        )

        self.throttle_pub: Publisher = self.create_publisher(
            Float32, f"/paf/{self.role_name}/throttle", qos_profile=1
        )

        self.brake_pub: Publisher = self.create_publisher(
            Float32, f"/paf/{self.role_name}/brake", qos_profile=1
        )

        self.reverse_pub: Publisher = self.create_publisher(
            Bool, f"/paf/{self.role_name}/reverse", qos_profile=1
        )

        self.__current_velocity: Optional[float] = None
        self.__target_velocity: Optional[float] = None
        self.pid_t = PID(
            self.pid_p_param.get_parameter_value().double_value,
            self.pid_i_param.get_parameter_value().double_value,
            self.pid_d_param.get_parameter_value().double_value,
        )
        # since we use this for braking aswell, allow -1 to 0.
        self.pid_t.output_limits = (-1.0, 1.0)

        self.loop_timer = self.create_timer(self.control_loop_rate, self.loop)
        self.get_logger().info("VelocityController node initialized.")

    def loop(self):
        """
        Calculates the result of the PID controller and publishes it.
        Never publishes values above speed limit
        (Publish speed_limit = -1 to drive without speeed limit)
        :param timer_event: Timer event from ROS
        :return:
        """
        if self.__target_velocity is None:
            self.get_logger().debug(
                "VelocityController hasn't received target"
                "_velocity yet. target_velocity has been set to"
                "default value 0"
            )
            self.__target_velocity = 0

        if self.__current_velocity is None:
            self.get_logger().debug(
                "VelocityController  hasn't received "
                "current_velocity yet and can therefore not"
                "publish a throttle value"
            )
            return

        self.pid_t.Kp = self.pid_p_param.get_parameter_value().double_value
        self.pid_t.Ki = self.pid_i_param.get_parameter_value().double_value
        self.pid_t.Kd = self.pid_d_param.get_parameter_value().double_value

        FIXED_SPEED = self.FIXED_SPEED_param.get_parameter_value().double_value
        FIXED_SPEED_OVERRIDE = (
            self.FIXED_SPEED_OVERRIDE_param.get_parameter_value().bool_value
        )

        target_velocity = (
            self.__target_velocity if not FIXED_SPEED_OVERRIDE else FIXED_SPEED
        )
        # revert driving
        if target_velocity < 0:
            reverse = True
            v = abs(target_velocity)
            self.pid_t.setpoint = v
            brake = 0
            throttle = self.pid_t(-self.__current_velocity)
            if throttle < 0:
                brake = abs(throttle)
                throttle = 0
        # very low target_velocities -> stand
        elif target_velocity < 0.1:
            reverse = False
            brake = 1
            throttle = 0
        else:
            reverse = False
            v = target_velocity
            self.pid_t.setpoint = v
            throttle = self.pid_t(self.__current_velocity)
            # any throttle < 0 is used as brake signal
            if throttle < 0:
                brake = abs(throttle)
                throttle = 0
            else:
                brake = 0

        self.reverse_pub.publish(reverse)
        self.brake_pub.publish(brake)
        self.throttle_pub.publish(throttle)

    def loop_handler(self):
        try:
            self.loop()
        except Exception as e:
            self.get_logger().fatal(e)

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocity = float(data.speed)

    def __get_target_velocity(self, data: Float32):
        self.__target_velocity = float(data.data)


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    rclpy.init(args=args)

    try:
        node = VelocityController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
