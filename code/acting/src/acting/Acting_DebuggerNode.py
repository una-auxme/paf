#!/usr/bin/env python

"""
This node provides full testability for all Acting
components by offering different testcases
to hopefully fully implement and tune Acting without
the need of working Perception and Planning components.
This also generates Lists of all the important values
to be saved in a file and plotted again.
TODO: emergency brake behavior
"""

import ros_compatibility as roscomp
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
import rospy
from rospy import Publisher, Subscriber
from carla_msgs.msg import CarlaSpeedometer

from trajectory_interpolation import interpolate_route

# since this dummy is supposed to test everything ACTING,
# Testers can choose which test to run from changing this Constant

# TEST_TYPE to choose which kind of Test to run:
# 0: Test Velocity Controller with constant one velocity
# const. velocity = MAX_VELOCITY_LOW
# const. steering = 0
# no trajectory
# TURN OFF stanley and PP Controllers in acting.launch!

# 1: Test Velocity Controller with changing velocity
# velocity = alternate all 20 secs: MAX_VELOCITY_LOW/_HIGH
# const. steering = 0
# no trajectory
# TURN OFF stanley and PP Controllers in acting.launch!

# 2: Test Steering Controller on chooseable trajectory
# velocity = MAX_VELOCITY_LOW TODO: maybe use velocity publisher?
# steering = STEERING_CONTROLLER_USED (see below)
# trajectory = TRAJECTORY_TYPE (see below)

# 3: Test Emergency Breaks on TestType 1
# TODO IMPLEMENT THIS TODO

# 4: Test Steering-PID in vehicleController
# TODO TODO
TEST_TYPE = 2                       # aka. TT

STEERING: float = 0.0               # for TT0: steering -> always straight
MAX_VELOCITY_LOW: float = 5.0      # for TT0/TT1: low velocity
MAX_VELOCITY_HIGH: float = 20.0     # for TT1: high velocity

STEERING_CONTROLLER_USED = 0  # for TT1/TT2: 0 = both ; 1 = PP ; 2 = Stanley
TRAJECTORY_TYPE = 1          # for TT2: 0 = Straight ; 1 = SineWave ; 2 = Curve


class Acting_Debugger(CompatibleNode):
    """
    Creates a node with testability for all acting components
    without the need of working/running perception or planning.
    """

    def __init__(self):
        """
        Constructor of the class
        :return:
        """
        super(Acting_Debugger, self).__init__('dummy_trajectory_pub')
        self.loginfo('Acting_Debugger node started')
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)

        # Publisher for Dummy Trajectory
        self.trajectory_pub: Publisher = self.new_publisher(
            Path,
            "/paf/" + self.role_name + "/trajectory",
            qos_profile=1)

        # Publisher for Dummy Velocity
        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/max_velocity",
            qos_profile=1)

        # Stanley: Publisher for Dummy Stanley-Steer
        self.stanley_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/stanley_steer",
            qos_profile=1)

        # PurePursuit: Publisher for Dummy PP-Steer
        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            qos_profile=1)

        # Publisher for Steeringcontrollers selector to test separately
        # Subscribed to in vehicle controller
        self.controller_selector_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/controller_selector_debug",
            qos_profile=1)

        # Subscriber of current_pos, used for TODO nothing yet
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

        # ---> EVALUATION/TUNING: Subscribers for plotting
        # Subscriber for max_velocity for plotting
        self.max_velocity_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/max_velocity",
            self.__get_max_velocity,
            qos_profile=1)

        # Subscriber for current_velocity for plotting
        self.current_velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

        # Subscriber for current_throttle for plotting
        self.current_throttle_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/throttle",
            self.__get_throttle,
            qos_profile=1)

        # Subscriber for PurePursuit_steer
        self.pure_pursuit_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            self.__get_purepursuit_steer,
            qos_profile=1)

        # Subscriber for Stanley_Steer
        self.stanley_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/stanley_steer",
            self.__get_stanley_steer,
            qos_profile=1)

        # Initialize all needed "global" variables here
        self.current_trajectory = []
        self.checkpoint_time = rospy.get_time()
        self.switchVelocity = False
        self.driveVel = MAX_VELOCITY_LOW

        self.__current_velocities = []
        self.__max_velocities = []
        self.__throttles = []
        self.time_set = False

        self.__purepursuit_steers = []
        self.__stanley_steers = []

        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "global"

        # Generate Trajectory as selected in TRAJECTORY_TYPE
        # Spawncoords at the simulationstart TODO: get from position
        startx = 984.5
        starty = -5442.0

        if (TRAJECTORY_TYPE == 0):  # Straight trajectory
            self.current_trajectory = [
                (startx, starty),
                (startx, starty-200)
            ]
            self.updated_trajectory(self.current_trajectory)

        elif (TRAJECTORY_TYPE == 1):  # Sinewave Serpentines trajectory
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
            self.current_trajectory = trajectory_wave
            self.updated_trajectory(self.current_trajectory)

        elif (TRAJECTORY_TYPE == 2):  # straight into 90° Curve
            self.current_trajectory = [
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
            self.updated_trajectory(self.current_trajectory)

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
            self.path_msg.poses.append(pos)

    def __current_position_callback(self, data: PoseStamped):
        agent = data.pose.position
        self.x = agent.x
        self.y = agent.y
        self.z = agent.z
        # TODO use this to get spawnpoint? necessary?

    def __get_max_velocity(self, data: Float32):
        self.__max_velocities.append(float(data.data))

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocities.append(float(data.speed))

    def __get_throttle(self, data: Float32):
        self.__throttles.append(float(data.data))

    def __get_stanley_steer(self, data: Float32):
        self.__stanley_steers.append(float(data.data))

    def __get_purepursuit_steer(self, data: Float32):
        self.__purepursuit_steers.append(float(data.data))

    def run(self):
        """
        Control loop
        :return:
        """
        self.checkpoint_time = rospy.get_time()

        def loop(timer_event=None):
            """
            Publishes different speeds, trajectories ...
            depending on the selected TEST_TYPE
            """
            if (TEST_TYPE == 0):
                self.drive_Vel = MAX_VELOCITY_LOW
                self.stanley_steer_pub.publish(STEERING)
                self.pure_pursuit_steer_pub.publish(STEERING)
                self.velocity_pub.publish(self.driveVel)

            elif (TEST_TYPE == 1):
                self.drive_Vel = MAX_VELOCITY_LOW
                if (self.checkpoint_time < rospy.get_time() - 20.0):
                    self.checkpoint_time = rospy.get_time()
                    if (self.switchVelocity):
                        self.driveVel = MAX_VELOCITY_HIGH
                    else:
                        self.driveVel = MAX_VELOCITY_LOW
                    self.switchVelocity = not self.switchVelocity
                self.stanley_steer_pub.publish(STEERING)
                self.pure_pursuit_steer_pub.publish(STEERING)
                self.velocity_pub.publish(self.driveVel)

            elif (TEST_TYPE == 2):
                # Continuously update path and publish it
                self.drive_Vel = MAX_VELOCITY_LOW
                self.updated_trajectory(self.current_trajectory)
                self.trajectory_pub.publish(self.path_msg)
                self.velocity_pub.publish(self.driveVel)

            elif (TEST_TYPE == 4):
                self.drive_Vel = MAX_VELOCITY_LOW
                self.stanley_steer_pub.publish(STEERING)
                self.pure_pursuit_steer_pub.publish(STEERING)

            if (STEERING_CONTROLLER_USED == 1):
                self.controller_selector_pub.publish(1)
            elif (STEERING_CONTROLLER_USED == 2):
                self.controller_selector_pub.publish(2)

            """# set starttime to when simulation is actually starting to run
            # to really get 10 secs plots every time
            if not self.time_set:
                self.checkpoint_time = rospy.get_time()
                self.time_set = True

            if (self.checkpoint_time < rospy.get_time() - 10.0):
                self.checkpoint_time = rospy.get_time()
                print(">>>>>>>>>>>> DATA <<<<<<<<<<<<<<")
                print(self.__max_velocities)
                print(self.__current_velocities)
                print(self.__throttles)
                print(len(self.__max_velocities))
                print(len(self.__current_velocities))
                print(len(self.__throttles))
                print(">>>>>>>>>>>> DATA <<<<<<<<<<<<<<")"""

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("Acting_Debugger", args=args)
    try:
        node = Acting_Debugger()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
