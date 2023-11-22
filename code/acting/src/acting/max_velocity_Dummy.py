#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher
import rospy
from std_msgs.msg import Float32

MAX_VELOCITY_HIGH: float = 5.0
MAX_VELOCITY_LOW: float = 5.0
STEERING: float = 0.0  # TODO: NO NEED IF TRAJECTORY DUMMY WORKS


class DummyVelocityPublisher(CompatibleNode):
    """
    This node publishes a constant max_velocity
    for Debugging and Parametertuning.
    """

    def __init__(self):
        super(DummyVelocityPublisher, self).__init__('dummy_const_vel_pub')
        self.control_loop_rate = self.get_param('control_loop_rate', 0.1)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/max_velocity",
            qos_profile=1)

        self.stanley_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/stanley_steer",
            qos_profile=1)

        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            qos_profile=1)

        self.checkpoint_time = rospy.get_time()
        self.switchVelocity = False
        self.driveVel = MAX_VELOCITY_HIGH

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('DUMMY_constant-velocity_publisher node running')

        def loop(timer_event=None):
            """
            Publishes velocity limits calculated in acting based on
            upcoming curves
            :param timer_event: Timer event from ROS
            :return:
            """

            # letzter timecheck < aktuelle Zeit - 20 Sekunden
            # = mehr Zeit vergangen als 20 Sekunden
            if (self.checkpoint_time < rospy.get_time() - 40.0):
                self.checkpoint_time = rospy.get_time()

                if (self.switchVelocity):
                    self.switchVelocity = False
                    self.driveVel = MAX_VELOCITY_HIGH

                else:
                    self.switchVelocity = True
                    self.driveVel = MAX_VELOCITY_LOW

            self.velocity_pub.publish(self.driveVel)
            # self.stanley_steer_pub.publish(STEERING)
            # self.pure_pursuit_steer_pub.publish(STEERING)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """

    roscomp.init('dummy_const_vel_pub', args=args)
    try:
        node = DummyVelocityPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
