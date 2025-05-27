import rclpy
from std_msgs.msg import String
import time

rclpy.init()

node = rclpy.create_node("Bla")
pub = node.create_publisher(String, "/bla", 0)
msg = String()
msg.data = "TestTest"
while True:
    pub.publish(msg)
    time.sleep(0.5)
