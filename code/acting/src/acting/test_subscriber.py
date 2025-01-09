#!/usr/bin/env python

import rospy
from mapping.msg import Map


def callback(data):
    rospy.loginfo("Received message.")
    rospy.loginfo(data)


def main():
    rospy.init_node("test_subscriber")
    rospy.Subscriber(
        "/paf/ego_vehicle/mapping/init_data", Map, callback, queue_size=100
    )
    rospy.spin()


if __name__ == "__main__":
    main()
