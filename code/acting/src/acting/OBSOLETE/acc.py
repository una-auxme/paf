#!/usr/bin/env python
import ros_compatibility as roscomp
import rospy
from carla_msgs.msg import CarlaSpeedometer
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from rospy import Publisher, Subscriber, Duration
from simple_pid import PID
from std_msgs.msg import Float32, Bool


class Acc(CompatibleNode):
    """
    This node implements the ACC functionality of the acting
    """

    def __init__(self):
        super(Acc, self).__init__('acc')
        self.loginfo('Acc node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.dist_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/acc_distance",
            self.__get_current_dist,
            qos_profile=1)

        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/max_velocity",
            qos_profile=1)

        # Use for testing
        # self.d_dist_pub: Publisher = self.new_publisher(
        #     Float32,
        #     f"/carla/{self.role_name}/d_dist",
        #     qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_velocity,
            qos_profile=1)

        self.emergency_pub: Publisher = self.new_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=QoSProfile(depth=10,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.__velocity = None
        self.__dist = None
        self.__on = False
        self.__dist_last_received_at = 0
        self.__v_max = 0

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('VehicleController node running')
        pid = PID(2, 0, 0.2)  # todo: tune

        def loop(timer_event=None):
            """
            Calculates the result of the PID controller and publishes it.
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__dist is None:
                if self.__on:
                    self.logdebug("ACC hasn't received a distance"
                                  " yet and can therefore not publish a "
                                  "velocity")
                return
            if self.__velocity is None:
                self.logdebug("ACC hasn't received the velocity of the ego "
                              "vehicle yet and can therefore not publish a "
                              "velocity")
                return

            if self.__dist < 0 and self.__on:
                self.loginfo("ACC turned off by sending dist < 0")
                self.__on = False
                self.__dist = None  # to check if new dist was published
                return

            if self.__on and rospy.get_rostime() - \
               self.__dist_last_received_at\
               > Duration(1):
                self.logwarn("ACC hasn't received a distance for more than 1 "
                             "second. ACC turning off")
                self.velocity_pub.publish(0)
                self.__on = False
                self.__dist = None  # to check if new dist was published
                return

            if not self.__on:
                if self.__dist > 0 and self.__dist > \
                   self.calculate_optimal_dist() * 0.7:
                    self.logwarn("ACC on")
                    self.__on = True
                else:
                    self.logwarn("Unsuitable conditions to turn on ACC. "
                                 "ACC staying off")
                    return

            if self.__dist < 0.5:
                self.velocity_pub.publish(0)
                self.logwarn("ACC off")
                self.__on = False
                self.__dist = None  # to check if new dist was published
                return

            if self.__dist < self.calculate_optimal_dist() / 2:
                self.emergency_pub.publish(True)
                self.on = False
                self.__dist = None  # to check if new dist was published
                self.logwarn("Distance to car in front is to low for ACC to "
                             "handle. "
                             "Turning ACC off and triggering emergency")
                return

            # Use for testing
            # self.d_dist_pub.publish(self.calculate_optimal_dist()-self.__dist)

            pid.setpoint = self.calculate_optimal_dist()
            delta = pid(self.__dist)
            v = self.__velocity - delta
            v = max(v, 0)
            self.velocity_pub.publish(v)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __get_current_dist(self, data: Float32):
        self.__dist = data.data
        self.__dist_last_received_at = rospy.get_rostime()

    def __get_velocity(self, data: CarlaSpeedometer):
        self.__velocity = data.speed

    def calculate_optimal_dist(self) -> float:
        """
        Calculates the distance you have to keep to the vehicle in front to
        have t_reaction to react to the vehicle suddenly stopping
        The formula replicates official recommendations for safe distances
        """
        t_reaction = 1  # s
        t_breaking = 1  # s
        a = 8  # m/s^2
        v = self.__velocity
        s = - 0.5 * a * t_breaking ** 2 + v * t_breaking + v * t_reaction
        return s + 5


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('acc', args=args)

    try:
        node = Acc()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
