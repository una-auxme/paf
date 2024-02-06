#!/usr/bin/env python
# import tf.transformations
import ros_compatibility as roscomp
import rospy
import tf.transformations

from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
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
        self.control_loop_rate = self.get_param("control_loop_rate", 0.1)

        self.target_speed = 0.0
        self.__curr_behavior = None
        self.__acc_speed = 0.0
        self.__stopline = None  # (Distance, isStopline)
        self.__change_point = None  # (Distance, isLaneChange, roadOption)
        self.__collision_point = None
        self.published = False
        self.current_pos = None
        self.current_heading = None
        self.trajectory = None
        self.overtaking = False
        self.current_wp = None
        self.enhanced_path = None
        self.current_speed = None
        self.speed_limit = None
        self.__corners = None
        self.__in_corner = False
        # Subscriber
        self.test_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/test",
            self.change_trajectory,
            qos_profile=1)
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

        self.change_point_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/collision",
            self.__set_collision_point,
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
        self.overtake_success_pub = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/overtake_success",
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

    def change_trajectory(self, distance_obj):
        limit_waypoints = 30
        pose_list = self.trajectory.poses
        count_retrys = 0

        def calculate_overtake(distance):
            obstacle_position = approx_obstacle_pos(distance,
                                                    self.current_heading,
                                                    self.current_pos,
                                                    self.current_speed)
            # trajectory_np = self.convert_pose_to_array(pose_list)
            # wp=KDTree(trajectory_np[:,:2]).query(obstacle_position[0][:2])[1]
            selection = pose_list[int(self.current_wp):int(self.current_wp) +
                                  int(distance + limit_waypoints)]
            waypoints = self.convert_pose_to_array(selection)

            initial_conditions = {
                'ps': 0,
                'target_speed': self.target_speed,
                'pos': np.array([self.current_pos[0], self.current_pos[1]]),
                'vel': np.array([obstacle_position[2][0],
                                obstacle_position[2][1]]),
                'wp': waypoints,
                'obs': np.array([[obstacle_position[0][0],
                                obstacle_position[0][1],
                                obstacle_position[1][0],
                                obstacle_position[1][1]]])
            }
            return run_fot(initial_conditions, hyperparameters)

        success_overtake = False
        while not success_overtake and count_retrys < 10:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, \
                costs, success = calculate_overtake(distance_obj)
            self.overtake_success_pub.publish(float(success))
            success_overtake = success
            count_retrys += 1
        if success_overtake:
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
            path.poses = pose_list[:int(self.current_wp)] + \
                result + pose_list[int(self.current_wp +
                                       distance_obj +
                                       30):]
            self.trajectory = path
        else:
            self.logerr("Overtake failed")
            self.overtake_success_pub.publish(-1)

    def __set_trajectory(self, data: Path):
        """get current trajectory global planning

        Args:
            data (Path): Trajectory waypoints
        """
        self.trajectory = data
        self.logerr("Trajectory received")
        self.__corners = self.__calc_corner_points()

    def __calc_corner_points(self):
        coords = self.convert_pose_to_array(np.array(self.trajectory.poses))
        x_values = np.array([point[0] for point in coords])
        y_values = np.array([point[1] for point in coords])

        angles = np.arctan2(np.diff(y_values), np.diff(x_values))
        angles = np.rad2deg(angles)
        angles[angles > 0] -= 360  # Convert for angles between 0 - 360 degree

        threshold = 1  # in degree
        curve_change_indices = np.where(np.abs(np.diff(angles)) > threshold)[0]

        sublist = self.create_sublists(curve_change_indices, proximity=5)

        coords_of_curve = [coords[i] for i in sublist]

        return coords_of_curve

    def create_sublists(self, points, proximity=5):
        sublists = []
        current_sublist = []

        for point in points:
            if not current_sublist:
                current_sublist.append(point)
            else:
                last_point = current_sublist[-1]
                distance = abs(point - last_point)

                if distance <= proximity:
                    current_sublist.append(point)
                else:
                    sublists.append(current_sublist)
                    current_sublist = [point]
        if current_sublist:
            sublists.append(current_sublist)

        filtered_list = [in_list for in_list in sublists if len(in_list) > 1]

        return filtered_list

    def get_cornering_speed(self):
        corner = self.__corners[0]
        pos = self.current_pos[:2]

        def euclid_dist(vector1, vector2):
            point1 = np.array(vector1)
            point2 = np.array(vector2)

            diff = point2 - point1
            sum_sqrt = np.dot(diff.T, diff)
            return np.sqrt(sum_sqrt)

        def map_corner(dist):
            if dist < 8:  # lane_change
                return 8
            elif dist < 25:
                return 6
            elif dist < 50:
                return 7
            else:
                8

        distance_corner = 0
        for i in range(len(corner) - 1):
            distance_corner += euclid_dist(corner[i], corner[i + 1])
        # self.logerr(distance_corner)

        if self.__in_corner:
            distance_end = euclid_dist(pos, corner[0])
            if distance_end > distance_corner + 2:
                self.__in_corner = False
                self.__corners.pop(0)
                self.loginfo("End Corner")
                return self.__get_speed_cruise()
            else:
                return map_corner(distance_corner)

        distance_start = euclid_dist(pos, corner[0])
        if distance_start < 3:
            self.__in_corner = True
            self.loginfo("Start Corner")
            return map_corner(distance_corner)
        else:
            return self.__get_speed_cruise()

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
            corner_speed = self.get_cornering_speed()
            self.target_speed = min(be_speed, acc_speed, corner_speed)
        else:
            self.target_speed = be_speed
        # self.target_speed = min(self.target_speed, 8)
        self.loginfo(f"Speed: {self.target_speed}")
        self.velocity_pub.publish(self.target_speed)
        # self.logerr(f"Speed: {self.target_speed}")
        # self.speed_list.append(self.target_speed)

    def __set_acc_speed(self, data: Float32):
        self.__acc_speed = data.data

    def __set_curr_behavior(self, data: String):
        self.__curr_behavior = data.data
        if data.data == bs.ot_enter_init.name:
            self.change_trajectory(self.__collision_point)

    def __set_stopline(self, data: Waypoint) -> float:
        if data is not None:
            self.__stopline = (data.distance, data.isStopLine)

    def __set_change_point(self, data: LaneChange):
        if data is not None:
            self.__change_point = \
                (data.distance, data.isLaneChange, data.roadOption)

    def __set_collision_point(self, data: Float32MultiArray):
        if data is not None:
            self.__collision_point = data.data[0]

    def get_speed_by_behavior(self, behavior: str) -> float:
        speed = 0.0
        split = "_"
        short_behavior = behavior.partition(split)[0]
        if short_behavior == "int":
            speed = self.__get_speed_intersection(behavior)
        elif short_behavior == "lc":
            speed = self.__get_speed_lanechange(behavior)
        elif short_behavior == "ot":
            speed = self.__get_speed_overtake(behavior)
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
        elif behavior == bs.int_app_to_stop.name:
            speed = self.__calc_speed_to_stop_intersection()
        elif behavior == bs.int_wait.name:
            speed == bs.int_wait.speed
        elif behavior == bs.int_enter.name:
            speed = bs.int_enter.speed
        elif behavior == bs.int_exit:
            speed = self.__get_speed_cruise()

        return speed

    def __get_speed_lanechange(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.lc_app_init.name:
            speed = bs.lc_app_init.speed
        elif behavior == bs.lc_app_blocked.name:
            speed = self.__calc_speed_to_stop_lanechange()
        elif behavior == bs.lc_app_free.name:
            speed = bs.lc_app_free.speed
        elif behavior == bs.lc_wait.name:
            speed = bs.lc_wait.speed
        elif behavior == bs.lc_enter_init.name:
            speed = bs.lc_enter_init.speed
        elif behavior == bs.lc_exit.name:
            speed = bs.lc_exit.speed

        return speed

    def __get_speed_overtake(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.ot_app_blocked.name:
            speed = self.__calc_speed_to_stop_overtake()
        elif behavior == bs.ot_app_free.name:
            speed = self.__get_speed_cruise()
        elif behavior == bs.ot_wait_stopped.name:
            speed = bs.ot_wait_stopped.speed
        elif behavior == bs.ot_wait_free.name:
            speed == self.__get_speed_cruise()
        elif behavior == bs.ot_enter_init.name:
            speed = self.__get_speed_cruise()
        elif behavior == bs.ot_enter_slow.name:
            speed = self.__calc_speed_to_stop_overtake()
        elif behavior == bs.ot_leave.name:
            speed = self.__get_speed_cruise()

        return speed

    def __get_speed_cruise(self) -> float:
        return self.__acc_speed

    def __calc_speed_to_stop_intersection(self) -> float:
        target_distance = 5.0
        virtual_stopline_distance = self.__calc_virtual_stopline()
        # calculate speed needed for stopping
        v_stop = max(convert_to_ms(10.),
                     convert_to_ms((virtual_stopline_distance / 30)
                                   * 50))
        if v_stop > bs.int_app_init.speed:
            v_stop = bs.int_app_init.speed
        if virtual_stopline_distance < target_distance:
            v_stop = 0.0
        return v_stop

    # TODO: Find out purpose
    def __calc_speed_to_stop_lanechange(self) -> float:
        stopline = self.__calc_virtual_change_point()

        v_stop = max(convert_to_ms(10.),
                     convert_to_ms((stopline / 30)
                                   * 50))
        if v_stop > bs.lc_app_init.speed:
            v_stop = bs.lc_app_init.speed
        if stopline < 5.0:
            v_stop = 0.0
        return v_stop

    def __calc_speed_to_stop_overtake(self) -> float:
        stopline = self.__calc_virtual_overtake()
        self.logerr(stopline)

        v_stop = max(convert_to_ms(10.),
                     convert_to_ms((stopline / 30)
                                   * 50))
        if stopline < 6.0:
            v_stop = 0.0
        return v_stop

    def __calc_virtual_change_point(self) -> float:
        if self.__change_point[0] != np.inf and self.__change_point[1]:
            return self.__change_point[0]
        else:
            return 0.0

    def __calc_virtual_stopline(self) -> float:
        if self.__stopline[0] != np.inf and self.__stopline[1]:
            return self.__stopline[0]
        else:
            return 0.0

    def __calc_virtual_overtake(self) -> float:
        self.logerr(f"Overtake point: {self.__collision_point}")
        if (self.__collision_point is not None) and \
                self.__collision_point != np.inf:
            return self.__collision_point
        else:
            return 0.0

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            if (self.__curr_behavior is not None and
                    self.__acc_speed is not None and
                    self.__corners is not None):
                self.trajectory.header.stamp = rospy.Time.now()
                self.traj_pub.publish(self.trajectory)
                self.update_target_speed(self.__acc_speed,
                                         self.__curr_behavior)
            else:
                self.velocity_pub.publish(0.0)
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
