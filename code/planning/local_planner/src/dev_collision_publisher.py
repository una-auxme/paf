#!/usr/bin/env python
# import rospy
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from carla_msgs.msg import CarlaRoute   # , CarlaWorldInfo
# from nav_msgs.msg import Path
# from std_msgs.msg import String
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaSpeedometer
import time


class DevCollisionCheck(CompatibleNode):
    """
    This is currently a test node. In the future this node will be
    responsible for detecting collisions and reporting them.
    """

    def __init__(self):
        super(DevCollisionCheck, self).__init__('DevCollisionCheck')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.pub_lidar = self.new_publisher(
            msg_type=Float32,
            topic='/paf/' + self.role_name + '/lidar_dist',
            qos_profile=1)

        self.pub_throttle = self.new_publisher(
            msg_type=Float32,
            topic='/paf/' + self.role_name + '/throttle',
            qos_profile=1)

        self.sub_ACC = self.new_subscription(
            msg_type=Float32,
            topic='/paf/' + self.role_name + '/ACC',
            callback=self.callback_ACC,
            qos_profile=1)

        self.logdebug("DevCollisionCheck started")
        self.last_position_update = None
        self.simulated_speed = 12  # m/s
        self.distance_to_collision = 0
        self.current_speed = 0
        self.velocity_sub = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

    def callback_ACC(self, msg: Float32):
        self.logdebug("ACC: " + str(msg.data))

    def __get_current_velocity(self, msg: CarlaSpeedometer):
        """
        Callback for current velocity
        :param msg:
        :return:
        """
        self.current_speed = msg.speed

    def run(self):
        """
        Control loop
        :return:
        """
        def loop(timer_event=None):
            while self.current_speed < 15:
                self.pub_throttle.publish(0.7)
            self.pub_throttle.publish(0.4)

            self.pub_collision.publish(30)
            time.sleep(0.3)
            self.pub_collision.publish(28)
            time.sleep(0.3)
            self.pub_collision.publish(26)
            time.sleep(0.3)
            self.pub_collision.publish(24)
            time.sleep(0.3)
            self.pub_collision.publish(22)
            time.sleep(0.3)
            self.pub_collision.publish(20)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the CollisionCheck node
    :param args:
    """
    roscomp.init('DevCollisionCheck')

    try:
        node = DevCollisionCheck()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
