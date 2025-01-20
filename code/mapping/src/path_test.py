import rospy
from nav.msgs import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp


class Test(CompatibleNode):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.pub = rospy.Publisher("/test_trajectory", Path, queue_size=10)

        self.publisher = self.new_publisher(
            msg_type=Path,
            topic=self.get_param("~map_init_topic", "trajectory_test"),
            qos_profile=1,
        )

        # define how many times per second
        # will the data be published
        # let's say 10 times/second or 10Hz
        self.rate = rospy.Rate(0.5)
        # to keep publishing as long as the core is running

        self.new_timer(1.0 / self.rate, self.publish_new_map)

    def publish(self):
        pose_stamps = []
        for i in range(0, 10):
            temp = PoseStamped()
            temp.pose.position.x = i / 2
            temp.pose.position.y = i * 0.2
            pose_stamps.append(temp)

        data = pose_stamps

        self.loginfo(data)
        self.publisher.publish(data)
        self.rate.sleep()


if __name__ == "__main__":
    name = "PatHTest"
    roscomp.init(name)
    node = Test(name)
    node.spin()
