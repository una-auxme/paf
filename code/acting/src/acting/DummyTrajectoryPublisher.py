#!/usr/bin/env python

"""
This node publishes a dummy trajectory between predefined points.
TODO: Implement this dummy to post a trajectory of serpentine-lines
to check if the steering controllers work
"""

import ros_compatibility as roscomp
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
from trajectory_interpolation import interpolate_route
import rospy
import numpy as np

TRAJECTORY_TYPE = 1  # 0 = Straight ; 1 = SineWave ; 2 = (old) Curve


class DummyTrajectoryPub(CompatibleNode):
    """
    Creates a node that publishes an interpolated trajectory between
    predefined points as a nav_msgs/Path message.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """
        super(DummyTrajectoryPub, self).__init__('dummy_trajectory_pub')
        self.loginfo('DummyTrajectoryPub node started')
        # basic info
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)

        self.current_trajectory = []
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "global"
        if (TRAJECTORY_TYPE == 2):
            # Static trajectory of a curve for testing purposes
            self.curve_trajectory = [
                (986.0, -5442.0),
                (986.0, -5463.2),
                (984.5, -5493.2),

                (984.5, -5563.5),
                (985.0, -5573.2),
                (986.3, -5576.5),
                (987.3, -5578.5),
                (988.7, -5579.0),
                (990.5, -5579.8),
                (1000.0, -5580.2),

                (1040.0, -5580.0),
                (1070.0, -5580.0),
                (1080.0, -5582.0),
                (1090.0, -5582.0),
                (1100.0, -5580.0),
                (1110.0, -5578.0),
                (1120.0, -5578.0),
                (1130.0, -5580.0),
                (1464.6, -5580.0),
                (1664.6, -5580.0)
            ]

        startx = 986.0
        starty = -5442.0
        # Generate a sine-wave with the global Constants to
        # automatically generate a trajectory with serpentine waves
        cycles = 4  # how many sine cycles
        resolution = 50  # how many datapoints to generate

        length = np.pi * 2 * cycles
        step = length / resolution  # spacing between values
        my_wave = np.sin(np.arange(0, length, step))
        x_wave = 2 * my_wave  # to have a serpentine line with +/- 2 meters
        # to have the serpentine line drive around the middle
        # of the road/start point of the car
        x_wave += startx
        traj_y = starty
        # starting point in the middle of the road,
        # sine wave swings around this later
        trajectory_wave = [(startx, traj_y)]
        for traj_x in x_wave:
            traj_y -= 2
            trajectory_wave.append((traj_x, traj_y))
        # back to the middle of the road
        trajectory_wave.append((startx, traj_y-2))
        # add a long straight path after the serpentines
        trajectory_wave.append((startx, starty-200))
        self.initial_trajectory = trajectory_wave
        self.updated_trajectory(self.initial_trajectory)

        # publisher for the current trajectory
        self.trajectory_publisher = self.new_publisher(
            Path,
            "/paf/" + self.role_name + "/trajectory",
            qos_profile=1)

        self.current_pos_sub = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

    def updated_trajectory(self, target_trajectory):
        """
        Updates the published Path message with the new target trajectory.
        :param: target_trajectory: the new target trajectory to be published
        :return:
        """
        self.current_trajectory = interpolate_route(target_trajectory, 0.25)
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "global"

        # clear old waypoints
        self.path_msg.poses.clear()

        for wp in self.current_trajectory:
            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.header.frame_id = "global"

            pos.pose.position.x = wp[0]
            pos.pose.position.y = wp[1]
            pos.pose.position.z = 37.6  # why??

            # currently not used therefore zeros
            pos.pose.orientation.x = 0
            pos.pose.orientation.y = 0
            pos.pose.orientation.z = 0
            pos.pose.orientation.w = 0

            # print(pos)

            self.path_msg.poses.append(pos)

    def __current_position_callback(self, data: PoseStamped):
        agent = data.pose.position
        self.x = agent.x
        self.y = agent.y
        self.z = agent.z
        # print("x: "+ str(agent.x))
        # print("y: "+ str(agent.y))
        # print("z: "+ str(agent.z))

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            # Continuously update path
            self.updated_trajectory(self.initial_trajectory)
            self.trajectory_publisher.publish(self.path_msg)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("dummy_trajectory_pub", args=args)
    try:
        node = DummyTrajectoryPub()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
