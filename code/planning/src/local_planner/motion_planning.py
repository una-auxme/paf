#!/usr/bin/env python
# import tf.transformations
import ros_compatibility as roscomp
import rospy
import tf.transformations

from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import String, Float32, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from carla_msgs.msg import CarlaSpeedometer
import numpy as np

from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory.fot_wrapper \
    import run_fot
from perception.msg import Waypoint, LaneChange
import planning  # noqa: F401
from behavior_agent.behaviours import behavior_speed as bs

from utils import convert_to_ms, approx_obstacle_pos, \
    hyperparameters

# from scipy.spatial._kdtree import KDTree


class MotionPlanning(CompatibleNode):
    """
    This node selects speeds according to the behavior in the Decision Tree
    and the ACC.
    Later this Node should compute a local Trajectory and forward
    it to the Acting.
    """

    def __init__(self):
        super(MotionPlanning, self).__init__('MotionPlanning')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.3)

        self.target_speed = 0.0
        self.__curr_behavior = None
        self.__acc_speed = 0.0
        self.__stopline = None  # (Distance, isStopline)
        self.__change_point = None  # (Distance, isLaneChange, roadOption)
        self.published = False
        self.current_pos = None
        self.current_heading = None
        self.trajectory = None
        self.overtaking = False
        self.overtake_start = rospy.get_rostime()
        self.current_wp = None
        self.enhanced_path = None
        self.current_speed = None
        self.speed_limit = None
        # Subscriber
        self.speed_limit_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/speed_limit",
            self.__set_speed_limit,
            qos_profile=1)
        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)
        self.head_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__set_heading,
            qos_profile=1)

        self.trajectory_sub = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_global",
            self.__set_trajectory,
            qos_profile=1)
        self.current_pos_sub = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.__set_current_pos,
            qos_profile=1)
        self.curr_behavior_sub: Subscriber = self.new_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1)
        self.emergency_sub: Subscriber = self.new_subscription(
            Bool,
            f"/paf/{self.role_name}/unchecked_emergency",
            self.__check_emergency,
            qos_profile=1)
        self.acc_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/acc_velocity",
            self.__set_acc_speed,
            qos_profile=1)

        self.stopline_sub: Subscriber = self.new_subscription(
            Waypoint,
            f"/paf/{self.role_name}/waypoint_distance",
            self.__set_stopline,
            qos_profile=1)

        self.change_point_sub: Subscriber = self.new_subscription(
            LaneChange,
            f"/paf/{self.role_name}/lane_change_distance",
            self.__set_change_point,
            qos_profile=1)

        # Publisher
        self.traj_pub: Publisher = self.new_publisher(
            msg_type=Path,
            topic=f"/paf/{self.role_name}/trajectory",
            qos_profile=1)
        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/target_velocity",
            qos_profile=1)

        self.wp_subs = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_wp",
            self.__set_wp,
            qos_profile=1)
        # Publisher for emergency stop
        self.emergency_pub = self.new_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=1)

        self.logdebug("MotionPlanning started")
        self.counter = 0

    def __set_speed_limit(self, data: Float32):
        """Set current speed limit

        Args:
            data (Float32): Current speed limit
        """
        self.speed_limit = data.data

    def __get_current_velocity(self, data: CarlaSpeedometer):
        """Get current velocity from CarlaSpeedometer

        Args:
            data (CarlaSpeedometer): Current velocity
        """
        self.current_speed = float(data.speed)

    def __set_wp(self, data: Float32):
        """Recieve current waypoint index from ACC

        Args:
            data (Float32): Waypoint index
        """
        self.current_wp = data.data

    def __set_heading(self, data: Float32):
        """Set current Heading

        Args:
            data (Float32): Current Heading vom Subscriber
        """
        self.current_heading = data.data

    def __set_current_pos(self, data: PoseStamped):
        """set current position
        Args:
            data (PoseStamped): current position
        """
        self.current_pos = np.array([data.pose.position.x,
                                    data.pose.position.y,
                                    data.pose.position.z])

    def change_trajectory(self, distance: float):
        self.overtake_start = rospy.get_rostime()
        limit_waypoints = 30
        np_array = np.array(self.trajectory.poses)
        obstacle_position = approx_obstacle_pos(distance,
                                                self.current_heading,
                                                self.current_pos,
                                                self.current_speed)
        # trajectory_np = self.convert_pose_to_array(np_array)
        # wp = KDTree(trajectory_np[:, :2]).query(obstacle_position[0][:2])[1]
        selection = np_array[int(self.current_wp):int(self.current_wp) +
                             int(distance + limit_waypoints)]
        waypoints = self.convert_pose_to_array(selection)

        initial_conditions = {
            'ps': 0,
            'target_speed': self.current_speed,
            'pos': np.array([self.current_pos[0], self.current_pos[1]]),
            'vel': np.array([obstacle_position[2][0],
                             obstacle_position[2][1]]),
            'wp': waypoints,
            'obs': np.array([[obstacle_position[0][0],
                              obstacle_position[0][1],
                              obstacle_position[1][0],
                              obstacle_position[1][1]]])
        }
        result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success = run_fot(initial_conditions,
                                                     hyperparameters)
        if success:
            result = []
            for i in range(len(result_x)):
                position = Point(result_x[i], result_y[i], 0)
                quaternion = tf.transformations.quaternion_from_euler(0,
                                                                      0,
                                                                      iyaw[i])
                orientation = Quaternion(x=quaternion[0], y=quaternion[1],
                                         z=quaternion[2], w=quaternion[3])
                pose = Pose(position, orientation)
                pos = PoseStamped()
                pos.header.frame_id = "global"
                pos.pose = pose
                result.append(pos)
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "global"
            path.poses = list(np_array[:int(self.current_wp)]) + \
                result + list(np_array[int(self.current_wp + 25 + 30):])
            self.trajectory = path

    def __set_trajectory(self, data: Path):
        """get current trajectory global planning

        Args:
            data (Path): Trajectory waypoints
        """
        self.trajectory = data

    def convert_pose_to_array(self, poses: np.array):
        """convert pose array to numpy array

        Args:
            poses (np.array): pose array

        Returns:
            np.array: numpy array
        """
        result_array = np.empty((len(poses), 2))
        for pose in range(len(poses)):
            result_array[pose] = np.array([poses[pose].pose.position.x,
                                           poses[pose].pose.position.y])
        return result_array

    def __check_emergency(self, data: Bool):
        """If an emergency stop is needed first check if we are
        in parking behavior. If we are ignore the emergency stop.

        Args:
            data (Bool): True if emergency stop detected by collision check
        """
        # self.logerr("Emergency stop detected")
        if not self.__curr_behavior == bs.parking.name:
            # self.logerr("Emergency stop detected and executed")
            self.emergency_pub.publish(data)

    def update_target_speed(self, acc_speed, behavior):
        be_speed = self.get_speed_by_behavior(behavior)
        if not behavior == bs.parking.name:
            self.target_speed = min(be_speed, acc_speed)
        else:
            self.target_speed = be_speed
        # self.logerr("target speed: " + str(self.target_speed))
        self.velocity_pub.publish(self.target_speed)

    def __set_acc_speed(self, data: Float32):
        self.__acc_speed = data.data

    def __set_curr_behavior(self, data: String):
        self.__curr_behavior = data.data

    def __set_stopline(self, data: Waypoint) -> float:
        if data is not None:
            self.__stopline = (data.distance, data.isStopLine)

    def __set_change_point(self, data: LaneChange):
        if data is not None:
            self.__change_point = \
                (data.distance, data.isLaneChange, data.roadOption)

    def get_speed_by_behavior(self, behavior: str) -> float:
        speed = 0.0
        split = "_"
        self.loginfo("get speed")
        short_behavior = behavior.partition(split)[0]
        self.loginfo("short behavior: " + str(short_behavior))
        if short_behavior == "int":
            speed = self.__get_speed_intersection(behavior)
        elif short_behavior == "lc":
            speed = self.__get_speed_lanechange(behavior)
        elif short_behavior == "parking":
            speed = bs.parking.speed
        else:
            speed = self.__get_speed_cruise()
        return speed

    def __get_speed_intersection(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.int_app_init.name:
            speed = bs.int_app_init.speed
        elif behavior == bs.int_app_green.name:
            speed = bs.int_app_green.speed
        elif behavior == bs.int_app_no_sign.name:
            speed = self.__calc_speed_to_stop_intersection()
        elif behavior == bs.int_wait.name:
            speed == bs.int_wait.speed
        elif behavior == bs.int_enter_no_light:
            speed = bs.int_enter_no_light.speed
        elif behavior == bs.int_enter_empty_str.name:
            speed = bs.int_enter_empty_str.speed
        elif behavior == bs.int_enter_light.name:
            speed == bs.int_enter_light.speed
        elif behavior == bs.int_exit:
            speed = bs.int_exit.speed

        return speed

    def __get_speed_lanechange(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.lc_app_init.name:
            speed = bs.lc_app_init.speed
        elif behavior == bs.lc_app_blocked.name:
            speed = bs.lc_app_blocked.speed  # calc_speed_to_stop_lanechange()
        elif behavior == bs.lc_enter_init.name:
            speed = bs.lc_enter_init.speed
        elif behavior == bs.lc_exit.name:
            speed = bs.lc_exit.speed

        return speed

    def __get_speed_cruise(self) -> float:
        return self.__acc_speed

    def __calc_speed_to_stop_intersection(self) -> float:
        target_distance = 3.0
        virtual_stopline_distance = self.__calc_virtual_stopline()
        # calculate speed needed for stopping
        v_stop = max(convert_to_ms(10.),
                     convert_to_ms((virtual_stopline_distance / 30)
                                   * 50))
        if v_stop > convert_to_ms(50.0):
            v_stop = convert_to_ms(50.0)
        if virtual_stopline_distance < target_distance:
            v_stop = 0.0

    # TODO: Find out purpose
    def __calc_speed_to_stop_lanechange(self) -> float:
        if self.__change_point[0] != np.inf and self.__change_point[1]:
            stopline = self.__change_point[0]
        else:
            return 100

        v_stop = max(convert_to_ms(5.),
                     convert_to_ms((stopline / 30) ** 1.5
                                   * 50))
        if v_stop > convert_to_ms(50.0):
            v_stop = convert_to_ms(30.0)
        return v_stop

    def __calc_virtual_stopline(self) -> float:
        if self.__stopline[0] != np.inf and self.__stopline[1]:
            return self.__stopline[0]
        elif self.traffic_light_detected:
            return self.traffic_light_distance
        else:
            return 0.0

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            if self.trajectory is None or self.__acc_speed is None or \
                    self.__curr_behavior is None:
                return
            self.update_target_speed(self.__acc_speed, self.__curr_behavior)
            self.trajectory.header.stamp = rospy.Time.now()
            self.traj_pub.publish(self.trajectory)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the MotionPlanning node
    :param args:
    """
    roscomp.init('MotionPlanning')
    try:
        node = MotionPlanning()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
