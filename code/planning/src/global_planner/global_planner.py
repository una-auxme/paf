#!/usr/bin/env python
import rospy
import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from xml.etree import ElementTree as eTree

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from carla_msgs.msg import CarlaRoute   # , CarlaWorldInfo
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

from preplanning_trajectory import OpenDriveConverter

RIGHT = 1
LEFT = 2
FORWARD = 3


class PrePlanner(CompatibleNode):
    """
    This node is responsible for collecting all data needed for the
    preplanning and calculate a trajectory based on the OpenDriveConverter
    from preplanning_trajectory.py.
    Subscribed/needed topics:
    - OpenDrive Map:          /carla/{role_name}/OpenDRIVE
                 or:          /carla/world_info
    - global Plan:            /carla/{role_name}/global_plan
    - current agent position: /paf/{role_name}/current_pos
    Published topics:
    - preplanned trajectory:  /paf/{role_name}/trajectory
    - prevailing speed limits:/paf/{role_name}/speed_limits_OpenDrive
    """

    def __init__(self):
        super(PrePlanner, self).__init__('DevGlobalRoute')

        self.path_backup = Path()

        self.odc = None
        self.global_route_backup = None
        self.agent_pos = None
        self.agent_ori = None

        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.distance_spawn_to_first_wp = self.get_param(
            "distance_spawn_to_first_wp", 100)

        self.map_sub = self.new_subscription(
            msg_type=String,
            topic=f"/carla/{self.role_name}/OpenDRIVE",
            callback=self.world_info_callback,
            qos_profile=10)

        # uncomment /paf/hero/global_plan and comment /carla/... for dev_launch
        self.global_plan_sub = self.new_subscription(
            msg_type=CarlaRoute,
            topic='/carla/' + self.role_name + '/global_plan',
            callback=self.global_route_callback,
            qos_profile=10)
        # self.global_plan_sub = self.new_subscription(
        #     msg_type=CarlaRoute,
        #     topic='/paf/' + self.role_name + '/global_plan',
        #     callback=self.global_route_callback,
        #     qos_profile=10)

        self.current_pos_sub = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.position_callback,
            qos_profile=1)

        self.path_pub = self.new_publisher(
            msg_type=Path,
            topic='/paf/' + self.role_name + '/trajectory_global',
            qos_profile=1)

        self.speed_limit_pub = self.new_publisher(
            msg_type=Float32MultiArray,
            topic=f"/paf/{self.role_name}/speed_limits_OpenDrive",
            qos_profile=1)
        self.logdebug('PrePlanner-Node started')

        # uncomment for self.dev_load_world_info() for dev_launch
        # self.dev_load_world_info()

    def global_route_callback(self, data: CarlaRoute) -> None:
        """
        when the global route gets updated a new trajectory is calculated with
        the help of OpenDriveConverter and published into
        '/paf/ self.role_name /trajectory'
        :param data: global Route
        """
        if data is None:
            self.logwarn("global_route_callback got called with None")
            return

        if self.odc is None:
            self.logwarn("PrePlanner: global route got updated before map... "
                         "therefore the OpenDriveConverter couldn't be "
                         "initialised yet")
            self.global_route_backup = data
            return

        if self.agent_pos is None or self.agent_ori is None:
            self.logwarn("PrePlanner: global route got updated before current "
                         "pose... therefore there is no pose to start with")
            self.global_route_backup = data
            return

        x_start = self.agent_pos.x      # 983.5
        y_start = self.agent_pos.y      # -5433.2
        x_target = data.poses[0].position.x
        y_target = data.poses[0].position.y
        if abs(x_start - x_target) > self.distance_spawn_to_first_wp or \
           abs(y_start - y_target) > self.distance_spawn_to_first_wp:
            self.logwarn("PrePlanner: current agent-pose doesnt match the "
                         "given global route")
            self.global_route_backup = data
            return

        self.global_route_backup = None

        # get the first turn command (1, 2, or 3)
        ind = 0
        for i, opt in enumerate(data.road_options):
            if opt == LEFT or opt == RIGHT or opt == FORWARD:
                x_turn = data.poses[i].position.x
                y_turn = data.poses[i].position.y
                ind = i
                break
        # if first target point is turning point
        if x_target == x_turn and y_target == y_turn:
            x_target = None
            y_target = None

        x_turn_follow = data.poses[ind+1].position.x
        y_turn_follow = data.poses[ind+1].position.y

        # Trajectory for the starting road segment
        self.odc.initial_road_trajectory(x_start, y_start,
                                         x_turn, y_turn,
                                         x_turn_follow, y_turn_follow,
                                         x_target, y_target,
                                         0, data.road_options[0])

        n = len(data.poses)
        # iterating through global route to create trajectory
        for i in range(1, n-1):
            # self.loginfo(f"Preplanner going throug global plan {i+1}/{n}")

            x_target = data.poses[i].position.x
            y_target = data.poses[i].position.y
            action = data.road_options[i]

            x_target_next = data.poses[i+1].position.x
            y_target_next = data.poses[i+1].position.y
            self.odc.target_road_trajectory(x_target, y_target,
                                            x_target_next, y_target_next,
                                            action)

        self.odc.target_road_trajectory(data.poses[n-1].position.x,
                                        data.poses[n-1].position.y,
                                        None, None,
                                        data.road_options[n-1])
        # trajectory is now stored in the waypoints
        # waypoints = self.odc.waypoints
        waypoints = self.odc.remove_outliner(self.odc.waypoints)
        way_x = waypoints[0]
        way_y = waypoints[1]
        way_yaw = waypoints[2]
        speed_limits = Float32MultiArray(data=waypoints[3])
        self.speed_limit_pub.publish(speed_limits)

        # Transforming the calculated waypoints into a Path msg
        stamped_poses = []
        for i in range(len(way_x)):
            position = Point(way_x[i], way_y[i], 0)  # way_speed[i])
            quaternion = tf.transformations.quaternion_from_euler(0,
                                                                  0,
                                                                  way_yaw[i])
            orientation = Quaternion(x=quaternion[0], y=quaternion[1],
                                     z=quaternion[2], w=quaternion[3])
            pose = Pose(position, orientation)
            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.header.frame_id = "global"
            pos.pose = pose
            stamped_poses.append(pos)

        self.path_backup.header.stamp = rospy.Time.now()
        self.path_backup.header.frame_id = "global"
        self.path_backup.poses = stamped_poses
        self.path_pub.publish(self.path_backup)
        self.loginfo("PrePlanner: published trajectory")

    def world_info_callback(self, opendrive: String) -> None:
        """
        when the map gets updated a mew OpenDriveConverter instance is created
        (needed for the trajectory preplanning)
        :param opendrive: updated CarlaWorldInformation
        """
        self.loginfo("PrePlanner: MapUpdate called")

        if type(opendrive) is str:
            root = eTree.fromstring(opendrive)
        else:
            root = eTree.fromstring(opendrive.data)

        roads = root.findall("road")
        road_ids = [int(road.get("id")) for road in roads]
        junctions = root.findall("junction")
        junction_ids = [int(junction.get("id")) for junction in junctions]

        odc = OpenDriveConverter(
            roads=roads, road_ids=road_ids,
            junctions=junctions, junction_ids=junction_ids)

        odc.convert_roads()
        odc.convert_junctions()
        odc.filter_geometry()

        self.odc = odc

        if self.global_route_backup is not None:
            self.loginfo("PrePlanner: Received a map update retrying "
                         "route preplanning")
            self.global_route_callback(self.global_route_backup)

    def position_callback(self, data: PoseStamped):
        """
        when the position gets updated it gets stored into self.agent_pos and
        self.agent_ori
        (needed for the trajectory preplanning)
        :param data: updated CarlaWorldInformation
        """
        self.agent_pos = data.pose.position
        self.agent_ori = data.pose.orientation
        if self.global_route_backup is not None:
            self.loginfo("PrePlanner: Received a pose update retrying "
                         "route preplanning")
            self.global_route_callback(self.global_route_backup)

    def dev_load_world_info(self):
        file_path = \
            "/workspace/code/planning/src/global_planner/string_world_info.txt"
        with open(file_path, 'r') as file:
            file_content = file.read()
        self.logerr("DATA READ")
        self.world_info_callback(file_content)

    def run(self):
        """
        Control loop
        :return:
        """

        # def loop(timer_event=None):
        #     if len(self.path_backup.poses) < 1:
        #         return

        #     # # Continuously update paths time to update car position in rviz
        #     # # TODO: remove next lines when local planner exists
        #     self.path_backup.header.stamp = rospy.Time.now()
        #     self.path_pub.publish(self.path_backup)

        # self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the PrePlanner node
    :param args:
    """
    roscomp.init('PrePlanner')

    try:
        node = PrePlanner()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
