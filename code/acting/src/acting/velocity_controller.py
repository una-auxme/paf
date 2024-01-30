#!/usr/bin/env python
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from simple_pid import PID
from std_msgs.msg import Float32
from nav_msgs.msg import Path

# TODO put back to 36 when controller can handle it
SPEED_LIMIT_DEFAULT: float = 7  # 36.0


class VelocityController(CompatibleNode):
    """
    This node controls the velocity of the vehicle.
    For this it uses a PID controller
    Published speeds will always stay below received speed limit
    Publish speed_limit = -1 to drive without speeed limit
    """

    def __init__(self):
        super(VelocityController, self).__init__('velocity_controller')
        self.loginfo('VelocityController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.target_velocity_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/target_velocity",
            self.__get_target_velocity,
            qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

        self.throttle_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/throttle",
            qos_profile=1)

        self.brake_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/brake",
            qos_profile=1)

        # needed to prevent the car from driving before a path to follow is
        # available. Might be needed later to slow down in curves
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        self.__current_velocity: float = None
        self.__target_velocity: float = None
        self.__trajectory: Path = None

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('VelocityController node running')
        # PID for throttle
        pid_t = PID(0.60, 0.00076, 0.63)
        pid_t.output_limits = (-1.0, 1.0)
        # new PID for braking, much weaker than throttle controller!
        # pid_b = PID(-0.1, -0, -0)  # TODO tune? BUT current P can be good
        # Kp just says "brake fully(1) until you are only Kp*speedError faster"
        # so with Kp = -1.35 -> the actual braking range is hardly used
        # pid_b.output_limits = (.0, 1.0)

        def loop(timer_event=None):
            """
            Calculates the result of the PID controller and publishes it.
            Never publishes values above speed limit
            (Publish speed_limit = -1 to drive without speeed limit)
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__target_velocity is None:
                self.logdebug("VelocityController hasn't received target"
                              "_velocity yet. target_velocity has been set to"
                              f"default value {SPEED_LIMIT_DEFAULT}")
                self.__target_velocity = SPEED_LIMIT_DEFAULT

            if self.__current_velocity is None:
                self.logdebug("VelocityController  hasn't received "
                              "current_velocity yet and can therefore not"
                              "publish a throttle value")
                return

            """if self.__trajectory is None:
                self.logdebug("VelocityController  hasn't received "
                              "trajectory yet and can therefore not"
                              "publish a throttle value (to prevent stupid)")
                return
            """
            if self.__target_velocity < 0:
                self.logerr("VelocityController doesn't support backward "
                            "driving yet.")
                return

            v = self.__target_velocity

            pid_t.setpoint = v
            throttle = pid_t(self.__current_velocity)

            # pid_b.setpoint = v
            # brake = pid_b(self.__current_velocity)

            # use negative throttles as brake inputs, works OK
            if throttle < 0:
                brake = abs(throttle)
                throttle = 0
            else:
                brake = 0

            self.brake_pub.publish(brake)
            self.throttle_pub.publish(throttle)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocity = float(data.speed)

    def __get_target_velocity(self, data: Float32):
        self.__target_velocity = float(data.data)

    def __set_trajectory(self, data: Path):
        self.__trajectory = data


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('velocity_controller', args=args)

    try:
        node = VelocityController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
