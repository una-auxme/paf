#!/usr/bin/env python
# import rospy
# import tf.transformations
import ros_compatibility as roscomp
import rospy
import tf.transformations

from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import String, Float32, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


import numpy as np
from nav_msgs.msg import Path

# from behavior_agent.msg import BehaviorSpeed
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory.fot_wrapper \
    import run_fot
from perception.msg import Waypoint, LaneChange
import planning
from behavior_agent.behaviours import behavior_speed as bs

# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from carla_msgs.msg import CarlaRoute   # , CarlaWorldInfo
# from nav_msgs.msg import Path
# from std_msgs.msg import String
# from std_msgs.msg import Float32MultiArray


def convert_to_ms(speed):
    return speed / 3.6


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
        self.control_loop_rate = self.get_param("control_loop_rate", 0.2)

        self.target_speed = 0.0
        self.__curr_behavior = None
        self.__acc_speed = 0.0
        self.__stopline = None  # (Distance, isStopline)
        self.__change_point = None  # (Distance, isLaneChange, roadOption)
        self.counter = 0
        self.speed_list = []
        self.__trajectory = None

        # Subscriber
        self.trajectory_sub = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
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
        
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        # Publisher
        self.traj_pub: Publisher = self.new_publisher(
            Path,
            f"/paf/{self.role_name}/trajectory",
            qos_profile=1)
        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/target_velocity",
            qos_profile=1)

        # Publisher for emergency stop
        self.emergency_pub = self.new_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=1)

        self.logdebug("MotionPlanning started")
        self.published = False
        self.current_pos = None

    def __set_current_pos(self, data: PoseStamped):
        """set current position

        Args:
            data (PoseStamped): current position
        """
        self.current_pos = np.array([data.pose.position.x,
                                    data.pose.position.y])

    def __set_trajectory(self, data: Path):
        """get current trajectory global planning

        Args:
            data (Path): Trajectory waypoints
        """
        if len(data.poses) and not self.published > 0:
            self.published = True
            np_array = np.array(data.poses)
            selection = np_array[5:17]
            waypoints = self.convert_pose_to_array(selection)
            self.logerr(waypoints)
            obs = np.array([[waypoints[5][0]-0.5, waypoints[5][1], waypoints[5][0]+0.5, waypoints[5][1]-2]])
            initial_conditions = {
                'ps': 0,
                'target_speed': self.__acc_speed,
                'pos': np.array([983.5807552562393, 5370.014637890163]),
                'vel': np.array([5, 1]),
                'wp': waypoints,
                'obs': obs
            }
            hyperparameters = {
                "max_speed": 25.0,
                "max_accel": 15.0,
                "max_curvature": 15.0,
                "max_road_width_l": 3.0,
                "max_road_width_r": 0.5,
                "d_road_w": 0.5,
                "dt": 0.2,
                "maxt": 20.0,
                "mint": 6.0,
                "d_t_s": 0.5,
                "n_s_sample": 2.0,
                "obstacle_clearance": 0.1,
                "kd": 1.0,
                "kv": 0.1,
                "ka": 0.1,
                "kj": 0.1,
                "kt": 0.1,
                "ko": 0.1,
                "klat": 1.0,
                "klon": 1.0,
                "num_threads": 0,  # set 0 to avoid using threaded algorithm
            }
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success = run_fot(initial_conditions,
                                                         hyperparameters)
            if success:
                print("Success!")
                print("result_x: ", result_x)
                print("result_y: ", result_y)
                result = []
                for i in range(len(result_x)):
                    position = Point(result_x[i], result_y[i], 703)
                    quaternion = tf.transformations.quaternion_from_euler(0,
                                                                            0,
                                                                            iyaw[i])
                    orientation = Quaternion(x=quaternion[0], y=quaternion[1],
                                            z=quaternion[2], w=quaternion[3])
                    pose = Pose(position, orientation)
                    pos = PoseStamped()
                    pos.header.stamp = rospy.Time.now()
                    pos.header.frame_id = "global"
                    pos.pose = pose
                    result.append(PoseStamped)
                path = Path()
                path.header.stamp = rospy.Time.now()
                path.path_backup.header.frame_id = "global"
                path.poses = list(np_array[:5]) + result + list(np_array[17:])
                self.traj_pub.publish(path)

    def convert_pose_to_array(self, poses: np.array):
        """convert pose array to numpy array

        Args:
            poses (np.array): pose array

        Returns:
            np.array: numpy array
        """
        def get_x_y(pose):
            return np.array([pose.pose.position.x, pose.pose.position.y])
        return np.vectorize(get_x_y)(poses)

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
        # self.logerr(f"Speed: {self.target_speed}")
        # self.speed_list.append(self.target_speed)

    def __set_acc_speed(self, data: Float32):
        self.__acc_speed = data.data

    def __set_trajectory(self, data: Path):
        self.__trajectory = data

    def __set_curr_behavior(self, data: String):
        self.__curr_behavior = data.data
        self.update_target_speed(self.__acc_speed, self.__curr_behavior)

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
            speed = self.__get_speed_cruise()

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
        else:
            return 0.0

    def run(self):
        """
        Control loop
        :return:
        """

        # def loop(timer_event=None):
        #     self.logerr(self.speed_list)

        # self.new_timer(self.control_loop_rate, loop)
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
