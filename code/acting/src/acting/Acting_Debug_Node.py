#!/usr/bin/env python

"""
This node provides full testability for all Acting
components by offering different testcases
to fully implement, test and tune Acting without
the need of working Perception and Planning components.
This also generates Lists of all the important values
to be saved in a file and plotted again.
"""

import math
import ros_compatibility as roscomp
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
import rospy
from rospy import Publisher, Subscriber
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl

from helper_functions import interpolate_route

"""
TEST_TYPE to choose which kind of Test to run:
- 0: Test Velocity Controller with constant one velocity
const. velocity = TARGET_VELOCITY_1
const. steering = 0
no trajectory
TURN OFF PP Controller in acting.launch!
- 1: Test Velocity Controller with changing velocity
velocity = alternate all 20 secs: TARGET_VELOCITY_1/_2
const. steering = 0
no trajectory
TURN OFF PP Controller in acting.launch!
- 2: Test Steering Controller on chooseable trajectory
velocity = TARGET_VELOCITY_1
steering = use acting.launch to selcet controller
trajectory = TRAJECTORY_TYPE (see below)
- 3: Test Emergency Breaks on TestType 1
const velocity = TARGET_VELOCITY_1
const steering = 0
no trajectory
Triggers emergency break after 15 Seconds
"""
TEST_TYPE = 2
FIXED_STEERING: float = 0  # if fixed steering needed
TARGET_VELOCITY_1: float = 10  # standard velocity
TARGET_VELOCITY_2: float = 0  # second velocity to switch to
# 0 = Straight ; 1 = Curve ; 2 = SineWave ; 3 = Overtake
TRAJECTORY_TYPE = 3

# This Component also prints collected data to the terminal,
# if wanted. Use the following Variables to select what to print.
# see lines 395 - 421 if you want to edit these!
PRINT_AFTER_TIME = 10.0  # How long after Simulationstart to print data
PRINT_TRAJECTORY = False  # True = prints the published trajectory
PRINT_VELOCITY_DATA = False  # True = print target and current velocities
PRINT_STEERING_DATA = False  # True = print stanley and pp steerings


class Acting_Debug_Node(CompatibleNode):
    """
    Creates a node with testability for all acting components
    without the need of working/running perception or planning.
    """

    def __init__(self):
        """
        Constructor of the class
        :return:
        """
        super(Acting_Debug_Node, self).__init__("dummy_trajectory_pub")
        self.loginfo("Acting_Debug_Node node started")
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)

        # Publisher for Dummy Trajectory
        self.trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/" + self.role_name + "/trajectory", qos_profile=1
        )

        # Publisher for Dummy Velocity
        self.velocity_pub: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/target_velocity", qos_profile=1
        )

        # PurePursuit: Publisher for Dummy PP-Steer
        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/pure_pursuit_steer", qos_profile=1
        )

        # Subscriber of current_pos, used for Steering Debugging
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1,
        )

        # ---> EVALUATION/TUNING: Subscribers for plotting
        # Subscriber for target_velocity for plotting
        self.target_velocity_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/target_velocity",
            self.__get_target_velocity,
            qos_profile=1,
        )

        # Subscriber for current_heading
        self.heading_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__get_heading,
            qos_profile=1,
        )

        # Subscriber for current_velocity
        self.current_velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1,
        )

        # Subscriber for current_throttle
        self.current_throttle_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/throttle",
            self.__get_throttle,
            qos_profile=1,
        )

        # Subscriber for Stanley_steer
        self.stanley_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/stanley_steer",
            self.__get_stanley_steer,
            qos_profile=1,
        )

        # Subscriber for PurePursuit_steer
        self.pure_pursuit_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            self.__get_purepursuit_steer,
            qos_profile=1,
        )

        # Subscriber for vehicle_steer
        self.vehicle_steer_sub: Subscriber = self.new_subscription(
            CarlaEgoVehicleControl,
            f"/carla/{self.role_name}/vehicle_control_cmd",
            self.__get_vehicle_steer,
            qos_profile=10,
        )

        # Publisher for emergency brake testing
        self.emergency_pub: Publisher = self.new_publisher(
            Bool, f"/paf/{self.role_name}/emergency", qos_profile=1
        )

        # Initialize all needed "global" variables here
        self.current_trajectory = []
        self.switchVelocity = False
        self.driveVel = TARGET_VELOCITY_1
        self.switch_checkpoint_time = rospy.get_time()
        self.switch_time_set = False
        self.checkpoint_time = rospy.get_time()
        self.time_set = False
        self.__current_velocities = []
        self.__max_velocities = []
        self.__throttles = []
        self.__current_headings = []
        self.__purepursuit_steers = []
        self.__stanley_steers = []
        self.__vehicle_steers = []
        self.positions = []

        # Initialize agent position
        self.x = 0
        self.y = 0
        self.z = 0

        # For visual purposes we set the trajectory height to vehicle heigt.
        # The trajectory z coordinated do not affect steering or else.
        # There is quite some confusion about visual height an actual height
        # This parameter might needs adustment
        self.z_visual = 0

        # Generate Trajectory as selected in TRAJECTORY_TYPE
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "global"
        # Spawncoords at the simulationstart
        startx = 984.5
        starty = -5442.0
        if TRAJECTORY_TYPE == 0:  # Straight trajectory
            self.current_trajectory = [(startx, starty), (startx, starty - 200)]

        elif TRAJECTORY_TYPE == 1:  # straight into 90° Curve
            self.current_trajectory = [
                (984.5, -5442.0),
                (984.5, -5563.5),
                (985.0, -5573.2),
                (986.3, -5576.5),
                (987.3, -5578.5),
                (988.7, -5579.0),
                (990.5, -5579.8),
                (1000.0, -5580.2),
                (1040.0, -5580.0),
                (1070.0, -5580.0),
            ]

        elif TRAJECTORY_TYPE == 2:  # Sinewave Serpentines trajectory
            # Generate a sine-wave with the global Constants to
            # automatically generate a trajectory with serpentine waves
            cycles = 4  # how many sine cycles
            resolution = 70  # how many datapoints to generate

            length = np.pi * 2 * cycles
            step = length / resolution  # spacing between values
            my_wave = np.sin(np.arange(0, length, step))
            x_wave = 1.5 * my_wave  # to have a serpentine line with +/-1.5 m
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
            trajectory_wave.append((startx, traj_y - 2))
            # add a long straight path after the serpentines
            trajectory_wave.append((startx, starty - 200))
            self.current_trajectory = trajectory_wave

        elif TRAJECTORY_TYPE == 3:  # 2 Lane Switches
            self.current_trajectory = [
                (startx, starty),
                (startx - 0.5, starty - 10),
                (startx - 0.5, starty - 20),
                (startx - 0.4, starty - 21),
                (startx - 0.3, starty - 22),
                (startx - 0.2, starty - 23),
                (startx - 0.1, starty - 24),
                (startx, starty - 25),
                (startx + 0.1, starty - 26),
                (startx + 0.2, starty - 27),
                (startx + 0.3, starty - 28),
                (startx + 0.4, starty - 29),
                (startx + 0.5, starty - 30),
                (startx + 0.6, starty - 31),
                (startx + 0.7, starty - 32),
                (startx + 0.8, starty - 33),
                (startx + 0.9, starty - 34),
                (startx + 1.0, starty - 35),
                (startx + 1.0, starty - 50),
                (startx + 1.0, starty - 51),
                (startx + 0.9, starty - 52),
                (startx + 0.8, starty - 53),
                (startx + 0.7, starty - 54),
                (startx + 0.6, starty - 55),
                (startx + 0.5, starty - 56),
                (startx + 0.4, starty - 57),
                (startx + 0.3, starty - 58),
                (startx + 0.2, starty - 59),
                (startx + 0.1, starty - 60),
                (startx, starty - 61),
                (startx - 0.1, starty - 62),
                (startx - 0.2, starty - 63),
                (startx - 0.3, starty - 64),
                (startx - 0.4, starty - 65),
                (startx - 0.5, starty - 66),
                (startx - 0.5, starty - 100),
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
            pos.pose.position.z = self.z_visual
            # currently not used therefore zeros
            pos.pose.orientation.x = 0
            pos.pose.orientation.y = 0
            pos.pose.orientation.z = 0
            pos.pose.orientation.w = 0
            self.path_msg.poses.append(pos)

    # ALL SUBSCRIBER-FUNCTIONS HERE
    def __current_position_callback(self, data: PoseStamped):
        agent = data.pose.position
        self.x = agent.x
        self.y = agent.y
        self.z = agent.z

        # TODO use this to get spawnpoint? necessary?
        # use to plot current_position to trajectory for steering test
        self.positions.append((self.x, self.y))

    def __get_heading(self, data: Float32):
        self.__current_headings.append(float(data.data))

    def __get_target_velocity(self, data: Float32):
        self.__max_velocities.append(float(data.data))

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocities.append(float(data.speed))

    def __get_throttle(self, data: Float32):
        self.__throttles.append(float(data.data))

    def __get_stanley_steer(self, data: Float32):
        r = 1 / (math.pi / 2)
        steering_float = float(data.data) * r
        self.__stanley_steers.append(steering_float)

    def __get_purepursuit_steer(self, data: Float32):
        r = 1 / (math.pi / 2)
        steering_float = float(data.data) * r
        self.__purepursuit_steers.append(steering_float)

    def __get_vehicle_steer(self, data: CarlaEgoVehicleControl):
        self.__vehicle_steers.append(float(data.steer))

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
            # Drive const. velocity on fixed straight steering
            if TEST_TYPE == 0:
                self.driveVel = TARGET_VELOCITY_1
                self.pure_pursuit_steer_pub.publish(FIXED_STEERING)
                self.velocity_pub.publish(self.driveVel)

            # Drive alternating velocities on fixed straight steering
            elif TEST_TYPE == 1:
                if not self.time_set:
                    self.drive_Vel = TARGET_VELOCITY_1
                    self.switch_checkpoint_time = rospy.get_time()
                    self.switch_time_set = True
                if self.switch_checkpoint_time < rospy.get_time() - 10:
                    self.switch_checkpoint_time = rospy.get_time()
                    self.switchVelocity = not self.switchVelocity
                    if self.switchVelocity:
                        self.driveVel = TARGET_VELOCITY_2
                    else:
                        self.driveVel = TARGET_VELOCITY_1
                self.pure_pursuit_steer_pub.publish(FIXED_STEERING)
                self.velocity_pub.publish(self.driveVel)

            # drive const. velocity on trajectoy with steering controller
            elif TEST_TYPE == 2:
                # Continuously update path and publish it
                self.drive_Vel = TARGET_VELOCITY_1
                self.updated_trajectory(self.current_trajectory)
                self.trajectory_pub.publish(self.path_msg)
                self.velocity_pub.publish(self.driveVel)

            # drive const. velocity on fixed straight steering and
            # trigger an emergency brake after 15 secs
            elif TEST_TYPE == 3:
                # Continuously update path and publish it
                self.drive_Vel = TARGET_VELOCITY_1
                if not self.time_set:
                    self.checkpoint_time = rospy.get_time()
                    self.time_set = True
                if self.checkpoint_time < rospy.get_time() - 15.0:
                    self.checkpoint_time = rospy.get_time()
                    self.emergency_pub.publish(True)
                self.pure_pursuit_steer_pub.publish(FIXED_STEERING)
                self.velocity_pub.publish(self.driveVel)

            # --- PRINT TO PLOT ---
            # set starttime to when simulation is actually starting to run
            # to really get X secs plots every time
            if not self.time_set:
                self.checkpoint_time = rospy.get_time()
                self.time_set = True
                if PRINT_TRAJECTORY:
                    print(">>>>>>>>>>>> TRAJECTORY <<<<<<<<<<<<<<")
                    print(self.current_trajectory)
                    print(">>>>>>>>>>>> TRAJECTORY <<<<<<<<<<<<<<")

            # Uncomment the prints of the data you want to plot
            if self.checkpoint_time < rospy.get_time() - PRINT_AFTER_TIME:
                self.checkpoint_time = rospy.get_time()
                print(">>>>>>>>>>>> DATA <<<<<<<<<<<<<<")
                if PRINT_VELOCITY_DATA:
                    print(">> TARGET VELOCITIES <<")
                    print(self.__max_velocities)
                    print(">> CURRENT VELOCITIES <<")
                    print(self.__current_velocities)
                    print(">> THROTTLES <<")
                    print(self.__throttles)
                if PRINT_STEERING_DATA:
                    print(">> PUREPURSUIT STEERS <<")
                    print(self.__purepursuit_steers)
                    print(">> STANLEY STEERS <<")
                    print(self.__stanley_steers)
                    print(">> ACTUAL POSITIONS <<")
                    print(self.positions)
                print(">>>>>>>>>>>> DATA <<<<<<<<<<<<<<")

        def loop_handler(timer_event=None):
            try:
                loop()
            except Exception as e:
                rospy.logfatal(e)

        self.new_timer(self.control_loop_rate, loop_handler)
        self.spin()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("Acting_Debug_NODE", args=args)
    try:
        node = Acting_Debug_Node()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
