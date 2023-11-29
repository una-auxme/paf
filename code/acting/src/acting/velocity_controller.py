#!/usr/bin/env python
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from simple_pid import PID
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Path

# TODO put back to 36 when controller can handle it
SPEED_LIMIT_DEFAULT: float = 10  # 36.0


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

        self.max_velocity_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/max_velocity",
            self.__get_max_velocity,
            qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

        self.speed_limit_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/speed_limit",
            qos_profile=1)

        self.max_tree_v_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/max_tree_velocity",
            self.__set_max_tree_v,
            qos_profile=1)

        self.throttle_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/throttle",
            qos_profile=1)

        # rqt_plot can't read the speed data provided by the rosbridge
        # Therefore, the speed is published again as a float value
        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/velocity_as_float",
            qos_profile=1)

        self.veldiff_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/vel_diff",
            qos_profile=1)

        # needed to prevent the car from driving before a path to follow is
        # available. Might be needed later to slow down in curves
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        # TODO: does this replace paf/hero/speed_limit?
        self.speed_limit_OD_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/speed_limits_OpenDrive",
            self.__set_speed_limits_opendrive,
            qos_profile=1)

        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

        self.__current_velocity: float = None
        self.__max_velocity: float = None
        self.__max_tree_v: float = None
        self.__speed_limit: float = None
        self.__trajectory: Path = None
        self.__speed_limits_OD: [float] = []
        self.__current_wp_index: int = 0

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('VehicleController node running')
        pid = PID(0.47, 0, 0)  # PID(0.25, 0, 0.1) values from paf21-2 TUNE!

        def loop(timer_event=None):
            """
            Calculates the result of the PID controller and publishes it.
            Never publishes values above speed limit
            (Publish speed_limit = -1 to drive without speeed limit)
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__max_velocity is None:
                self.logdebug("VehicleController hasn't received max_velocity"
                              " yet. max_velocity has been set to"
                              f"default value {SPEED_LIMIT_DEFAULT}")
                # return
                self.__max_velocity = SPEED_LIMIT_DEFAULT

            if self.__current_velocity is None:
                self.logdebug("VehicleController hasn't received "
                              "current_velocity yet and can therefore not"
                              "publish a throttle value")
                return
            """
            if self.__trajectory is None:
                self.logdebug("VehicleController hasn't received "
                              "trajectory yet and can therefore not"
                              "publish a throttle value (to prevent stupid)")
                return

            if self.__speed_limit is None or self.__speed_limit < 0:
                self.logdebug("VelocityController hasn't received a acceptable"
                              " speed_limit yet. speed_limit has been set to"
                              f"default value {SPEED_LIMIT_DEFAULT}")
                self.__speed_limit = SPEED_LIMIT_DEFAULT

            if self.__max_tree_v is None or self.__max_tree_v < 0:
                self.logdebug("VelocityController hasn't received a acceptable"
                              " max_tree_v yet. speed_limit has been set to"
                              f"default value {SPEED_LIMIT_DEFAULT}")
                self.__max_tree_v = SPEED_LIMIT_DEFAULT

            if self.__max_velocity < 0:
                self.logerr("Velocity controller doesn't support backward "
                            "driving yet.")
                return
            """
            # v = min(self.__max_velocity, self.__max_tree_v)
            # v = min(v, self.__speed_limit)
            v = self.__max_velocity

            pid.setpoint = v
            throttle = pid(self.__current_velocity)
            throttle = max(throttle, 0)  # ensures that throttle >= 0
            throttle = min(throttle, 1.0)  # ensures that throttle <= 1
            self.throttle_pub.publish(throttle)
            vel_diff = self.__current_velocity - v
            self.veldiff_pub.publish(vel_diff)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocity = float(data.speed)
        self.velocity_pub.publish(self.__current_velocity)

    def __set_max_tree_v(self, data: Float32):
        self.__max_tree_v = float(data.data)

    def __get_max_velocity(self, data: Float32):
        self.__max_velocity = float(data.data)

    def __get_speed_limit(self, data: Float32):
        self.__speed_limit = float(data.data)

    def __set_trajectory(self, data: Path):
        self.__trajectory = data

    def __set_speed_limits_opendrive(self, data: Float32MultiArray):
        self.__speed_limits_OD = data.data

    def __current_position_callback(self, data: PoseStamped):
        """
        if len(self.__speed_limits_OD) < 1 or self.__trajectory is None:
            return

        agent = data.pose.position
        current_wp = self.__trajectory.poses[self.__current_wp_index].\
            pose.position
        next_wp = self.__trajectory.poses[self.__current_wp_index + 1].\
            pose.position

        # distances from agent to current and next waypoint
        d_old = abs(agent.x - current_wp.x) + abs(agent.y - current_wp.y)
        d_new = abs(agent.x - next_wp.x) + abs(agent.y - next_wp.y)
        if d_new < d_old:
            # update current waypoint and corresponding speed limit
            self.__current_wp_index += 1
            self.__speed_limit = \
                self.__speed_limits_OD[self.__current_wp_index]
            self.speed_limit_pub.publish(self.__speed_limit)
        """


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
