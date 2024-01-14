#!/usr/bin/env python
import os
import rospy
import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaRoute, CarlaWorldInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
import xmltodict


class DevGlobalRoute(CompatibleNode):

    def __init__(self):
        super(DevGlobalRoute, self).__init__('DevGlobalRoute')
        self.role_name = self.get_param("role_name", "hero")
        self.from_txt = self.get_param("from_txt", True)

        if self.from_txt:
            self.global_route_txt = self.get_param(
                'global_route_txt',
                "/code/planning/global_planner/src/global_route.txt")
        else:
            self.sampling_resolution = self.get_param('sampling_resolution',
                                                      100.0)
            # consecutively increasing sequence ID for header_msg
            self.seq = 0
            self.routes = self.get_param(
                'routes', "/opt/leaderboard/data/routes_devtest.xml")

        self.map_sub = self.new_subscription(
            msg_type=CarlaWorldInfo,
            topic="/carla/world_info",
            callback=self.world_info_callback,
            qos_profile=10)

        self.global_plan_pub = self.new_publisher(
            msg_type=CarlaRoute,
            topic='/paf/' + self.role_name + '/global_plan',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.loginfo('DevGlobalRoute-Node started')

    def world_info_callback(self, data: CarlaWorldInfo) -> None:
        """
        publishes the global_route (with route instructions) according to the
        given map and waypoints of a file containing the test routes
        :param data: updated CarlaWorldInformation
        """
        self.loginfo("DevRoute: MapUpdate called")
        if self.from_txt:
            try:
                with open(f"/workspace{self.global_route_txt}", "r") as txt:
                    input_routes = txt.read()
            except FileNotFoundError:
                self.logerr(f"/workspace{self.global_route_txt} not found... "
                            f"current working directory is'{os.getcwd()}'")
                raise

            global_routes = input_routes.split("---")
            header_list = []
            road_options_list = []
            poses_list = []
            for route in global_routes:
                if "seq:" not in route:
                    continue
                seq = int(route.split("seq: ")[1].split("\n")[0])
                secs = int(route.split("secs: ")[1].split("\n")[0])
                nsecs = int(route.split("nsecs:")[1].split("\n")[0])
                frame_id = route.split('"')[1]
                header_list.append(
                    Header(seq, rospy.Time(secs, nsecs), frame_id))
                road_options_str = route.split("[")[1].split("]")[0].split(",")
                road_options_list.append([int(road_option)
                                          for road_option in road_options_str])
                poses_str = route.split("position:")[1:]
                poses = []
                for pose in poses_str:
                    x = float(pose.split("x: ")[1].split("\n")[0])
                    y = float(pose.split("y: ")[1].split("\n")[0])
                    z = float(pose.split("z: ")[1].split("\n")[0])
                    position = Point(x, y, z)
                    orientation_str = pose.split("orientation:")[1]
                    x = float(orientation_str.split("x: ")[1].split("\n")[0])
                    y = float(orientation_str.split("y: ")[1].split("\n")[0])
                    z = float(orientation_str.split("z: ")[1].split("\n")[0])
                    w = float(orientation_str.split("w: ")[1].split("\n")[0])
                    orientation = Quaternion(x, y, z, w)
                    poses.append(Pose(position, orientation))
                poses_list.append(poses)
            self.global_plan_pub.publish(header_list[0], road_options_list[0],
                                         poses_list[0])
        else:

            with open(self.routes, 'r', encoding='utf-8') as file:
                my_xml = file.read()

            # Use xmltodict to parse and convert the XML document
            routes_dict = xmltodict.parse(my_xml)
            route = routes_dict['routes']['route'][0]
            town = route['@town']

            if town not in data.map_name:
                self.logerr(f"Map '{data.map_name}' doesnt match routes "
                            f"'{town}'")
                return

            # Convert data into a carla.Map
            carla_map = carla.Map(data.map_name, data.opendrive)
            # carla_map.save_to_disk(".\map.xml")
            # Create GlobalRoutePlanner
            grp = GlobalRoutePlanner(carla_map, self.sampling_resolution)

            # plan the route between given waypoints
            route_trace = []
            waypoints = route['waypoints']['position']
            prepoint = waypoints[0]
            for waypoint in waypoints[1:]:
                start = carla.Location(float(prepoint['@x']),
                                       float(prepoint['@y']),
                                       float(prepoint['@z']))
                origin = carla.Location(float(waypoint['@x']),
                                        float(waypoint['@y']),
                                        float(waypoint['@z']))
                part_route_trace = grp.trace_route(start, origin)
                route_trace.extend(part_route_trace)
                prepoint = waypoint

            # parse the global route into the CarlaRoute format
            road_options = []
            poses = []
            for waypoint, road_option in route_trace:
                location = waypoint.transform.location
                position = Point(location.x, location.y, location.z)

                rotation = waypoint.transform.rotation
                quaternion = tf.transformations.quaternion_from_euler(
                    rotation.roll, rotation.pitch, rotation.yaw)
                orientation = Quaternion(quaternion[0], quaternion[1],
                                         quaternion[2], quaternion[3])

                poses.append(Pose(position, orientation))
                road_options.append(road_option)

                # self.loginfo(f"{location}: {road_option}")

            self.seq += 1
            header = Header(self.seq, rospy.Time.now(), "/map")
            self.global_plan_pub.publish(header, road_options, poses)
        self.loginfo("DevRoute: published global_plan")


if __name__ == "__main__":
    """
          main function starts the NavManager node
          :param args:
    """
    roscomp.init('DevGlobalRoute')

    try:
        DevGlobalRoute()
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
