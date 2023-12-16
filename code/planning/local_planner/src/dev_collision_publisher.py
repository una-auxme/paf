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
import time
# import numpy as np


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
            topic='/carla/' + self.role_name + '/lidar_dist_dev',
            qos_profile=1)

        self.pub_test_speed = self.new_publisher(
            msg_type=Float32,
            topic='/paf/' + self.role_name + '/test_speed',
            qos_profile=1)
        self.sub_ACC = self.new_subscription(
            msg_type=Float32,
            topic='/paf/' + self.role_name + '/acc_velocity',
            callback=self.callback_ACC,
            qos_profile=1)

        self.sub_manual = self.new_subscription(
            msg_type=Float32,
            topic='/paf/' + self.role_name + '/manual',
            callback=self.callback_manual,
            qos_profile=1)
        self.logerr("DevCollisionCheck started")
        self.last_position_update = None
        self.simulated_speed = 12  # m/s
        self.distance_to_collision = 0
        self.current_speed = 0
        self.manual_start = True
        self.acc_activated = False

    def callback_manual(self, msg: Float32):
        if self.manual_start:
            self.logerr("Manual start")
            self.manual_start = False
            self.pub_lidar.publish(Float32(data=25))
            time.sleep(0.2)
            self.pub_lidar.publish(Float32(data=25))
            time.sleep(0.2)
            self.pub_lidar.publish(Float32(data=24))
            # time.sleep(0.2)
            # self.pub_lidar.publish(Float32(data=20))
            # time.sleep(0.2)
            # self.pub_lidar.publish(Float32(data=20))
            # time.sleep(0.2)
            # self.pub_lidar.publish(Float32(data=20))

    def callback_ACC(self, msg: Float32):
        self.acc_activated = True
        # self.logerr("Timestamp: " + time.time().__str__())
        # self.logerr("ACC: " + str(msg.data))
        self.current_speed = msg.data

    def run(self):
        """
        Control loop
        :return:
        """
        def loop(timer_event=None):
            self.pub_test_speed.publish(Float32(data=13.8889))

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
