import copy
from xml.etree import ElementTree as eTree
import help_functions
from typing import Tuple
from math import sin, cos, degrees


# Check small distance between two points
SMALL_DIST = 0.001
# Step size for lane change
STEP_SIZE = 0.5
# distance between points in linear interpolation
INTERVALL = 1.0
# difference to the target point
TARGET_DIFF = 5
LINE = 0.0
# Value to convert mph in m/s
MPH = 2.24
# Value to convert kmh/h in m/s
KMH = 3.6
# Default speed for passing a junction in m/s
DEFAULT_MS = 12
# Road value from openDrive format
ROAD = -1
# Change lane commands from leaderboard
CHANGE_LEFT = 5
CHANGE_RIGHT = 6
CHANGE_FOLLOW = 4
# Turn commands from leaderboard
LEFT = 1
RIGHT = 2
STRAIGHT = 3


class OpenDriveConverter:
    """
    The OpenDriveConverter needs am OpenDrive Map and the road options from
    the Carla leaderboard to calculate the global trajectory.
    """
    def __init__(self, path=None, roads=None, road_ids=None,
                 junctions=None, junction_ids=None):

        if roads is None or road_ids is None:
            self.roads, self.road_ids = self.list_xodr_properties(
                path, name="road")
        else:
            self.roads = roads
            self.road_ids = road_ids

        if junctions is None or junction_ids is None:
            self.junctions, self.junction_ids = self.list_xodr_properties(
                path, name="junction")
        else:
            self.junctions = junctions
            self.junction_ids = junction_ids

        # All x, y coordinates for each road
        self.road_coord = None
        # hold the expected action after reaching a waypoint
        self.next_action = None
        self.follow_type = None
        self.follow_tag = None
        # list that contains trajectory data
        self.waypoints = None
        self.reference = None
        # id of the current road
        self.road_id = None
        # id of the following road
        self.follow_id = None
        # id of the predecessor road
        self.pt = None
        self.reference_l = None
        self.follow_section = None
        self.direction = None
        self.old_id = None
        self.width = None
        self.point_size = None
        self.geometry_data = None
        """ A list of geometry data for each road
            format: [[road0], [road1],...]
                road0:  [[x_points], [y_points], [heading], [curvature],
                         [length]]
                    x_points:   x positions of all road segments (m)
                    y_points:   y positions of all road segments (m)
                    heading:    orientation value in rad
                    curvature:  curvature value (m) (if value 0 -> line)
                    length:     length of the reference line (m)
            Example of accessing the ith road, first y value:
                geometry_data[i][1][0]
        """

    def list_xodr_properties(self, path: str, name: str):
        """ Filter properties out of the xodr file
            :param path: reference to the xodr file
            :param name: name of the property to filter
            :return elements: list of the preferred elements
            :return element_ids: list of the id values for each element
        """
        # find reference to root node of xodr file
        root = eTree.parse(path).getroot()
        elements = root.findall(name)
        element_ids = [int(element.get("id")) for element in elements]
        return elements, element_ids

    def convert_roads(self):
        """ Filter all road elements in a list. Every road index belongs
        to the same list index.
        """
        max_id = int(max(self.road_ids))
        roads_extracted = list()
        j = 0
        for i in range(max_id + 1):
            if int(self.roads[j].get("id")) == i:
                roads_extracted.append(self.roads[j])
                j += 1
            else:
                roads_extracted.append(None)
        self.roads = roads_extracted

    def convert_junctions(self):
        """ Filter all junction elements in a list. Every junction index
        belongs to the same list index.
        """
        max_id = int(max(self.junction_ids))
        junctions_extracted = list()
        j = 0
        for i in range(max_id + 1):
            if int(self.junctions[j].get("id")) == i:
                junctions_extracted.append(self.junctions[j])
                j += 1
            else:
                junctions_extracted.append(None)
        self.junctions = junctions_extracted

    def filter_geometry(self):
        """ Extract all the geometry information for each road
        use the initialised roads object from the init function
        this function is only used once when the OpenDrive map is received
        """
        geometry_data = list()
        for road in self.roads:
            if road is None:
                geometry_data.append(None)
            else:
                plan_view = road.find("planView")
                geometry = plan_view.findall("geometry")
                x = list()
                y = list()
                heading = list()
                curvature = list()
                length = list()
                for geo in geometry:
                    x.append(float(geo.get("x")))
                    y.append(float(geo.get("y")))
                    heading.append(float(geo.get("hdg")))
                    length.append(float(geo.get("length")))
                    if geo[0].tag == "arc":
                        # arc values in the file have the unit 1/m
                        # we need to transform them in m
                        arc_value = float(geo.find("arc").get("curvature"))
                        radius = 1 / arc_value
                        curvature.append(radius)
                    else:
                        curvature.append(0.0)
                geometry_data.append([x, y, heading, curvature, length])
        assert (len(self.roads) == len(geometry_data))
        self.geometry_data = geometry_data

    def find_current_road(self, x_curr: float, y_curr: float):
        """ Extract the current road that fits to the x and y coordinate
        Needed to find the starting road of the agent and for every new
        waypoint we receive from Carla.
            :param x_curr: the current global x position of the agent
            :param y_curr: the current global y position of the agent
            :return: curr_road_id: The id of the current road
        """
        diff_list = list()
        diff_index_list = list()
        j = 0
        for road in self.geometry_data:
            if road is None:
                j += 1
                continue
            for i in range(len(road[0])):
                diff = help_functions.euclid_dist((road[0][i], road[1][i]),
                                                  (x_curr, y_curr))
                diff_list.append(diff)
                diff_index_list.append(j)
            j += 1
        min_diff = min(diff_list)
        min_diff_index = diff_list.index(min_diff)
        selected_road_id = diff_index_list[min_diff_index]
        junction = int(self.roads[selected_road_id].get("junction"))
        agent_position = (x_curr, y_curr)

        predecessor, successor = self.get_pred_succ(selected_road_id)
        # Successor and predecessor are a junction -> selected road correct
        if self.geometry_data[predecessor] is None and \
           self.geometry_data[successor] is None:
            current_id = selected_road_id
        # no junction recognized or current road is junction
        else:
            current_id = self.calculate_intervalls_id(agent_position,
                                                      selected_road_id,
                                                      predecessor,
                                                      successor,
                                                      junction)
        return current_id

    def calculate_intervalls_id(self, agent: Tuple[float, float],
                                current: int, pred: int, succ: int,
                                junction: int):
        """
        The function assumes, that the current chosen road is not the only
        one that is possible. The current raad is calculated based on all
        possible scenarios not assuming that the succ and pred road are
        both junctions.
        :param agent: current position of the agent with x and y coordinate
        :param current: the current assumed road id
        :param pred: id value of the predecessor road
        :param succ: id value of the successor road
        :param junction: id value of the junction
        :return: id_value: the id value of the calculated road
        """
        id_value = None
        dist = None
        # current road is a junction -> check pred and succ
        if junction != ROAD:
            dist = self.get_dist_list(succ, pred, None, agent)
            min_dist = self.get_min_dist(dist)
            ind = dist.index(min_dist)
            if ind == 0 or ind == 1:
                id_value = succ
            else:
                id_value = pred
        # Only one possible other road and current road is not a junction
        elif self.geometry_data[pred] is None or self.geometry_data[succ] \
                is None:
            if pred is None:
                road = succ
            else:
                road = pred
            dist = self.get_dist_list(road, current, None, agent)
            min_dist = self.get_min_dist(dist)
            if len(min_dist) == 1:
                ind = dist.index(min_dist)
                if ind == 0 or ind == 1:
                    id_value = road
                else:
                    id_value = current
            else:
                # calculate special case when two roads are chosen
                id_value = self.get_special_case_id(road, current, agent)
        # two possible other roads and current not a junction
        else:
            dist = self.get_dist_list(pred, current, succ, agent)
            min_dist = self.get_min_dist(dist)
            if len(min_dist) == 1:
                ind = dist.index(min_dist)
                if ind == 0 or ind == 1:
                    id_value = pred
                elif ind == 2 or ind == 3:
                    id_value = current
                else:
                    id_value = succ
            else:
                # calculate special case
                ind = [i for i in range(len(dist)) if dist[i] == min_dist[0]]
                if ind[0] == 0 or ind[0] == 1:
                    first = pred
                    if ind[1] == 2 or ind[1] == 3:
                        second = current
                    else:
                        second = succ
                elif ind[0] == 2 or ind[0] == 3:
                    first = current
                    second = succ
                id_value = self.get_special_case_id(first, second, agent)
        return id_value

    def get_special_case_id(self, road: int, current: int,
                            agent: Tuple[float, float]):
        """ When the function get_min_dist() returns two solutions with the
        same distance, this function calculated the distance based on the
        interpolation of the two possible roads.
        :param road: id value of the successor or predecessor road
        :param current: id value of the current road
        :param agent: current position ot the agent with x and y coordinate
        :return: final_id: the calculated id of the current road
        """
        final_id = None
        list_r = self.interpolation(road)
        list_c = self.interpolation(current)

        dist_r = [help_functions.euclid_dist(
            agent, (list_r[0][i], list_r[1][i]))
            for i in range(len(list_r[0]))]
        dist_c = [help_functions.euclid_dist(
            agent, (list_c[0][i], list_c[1][i]))
            for i in range(len(list_r[0]))]
        value_r = min(dist_r)
        value_c = min(dist_c)
        if value_r < value_c:
            final_id = road
        else:
            final_id = current
        return final_id

    def get_min_dist(self, dist: list):
        """ Calculate the minimum distance value from a distance list.
        :param dist: list containing all distance values
        :return: min_dist: the minimum distance/distances from the list
        """
        min_v = float("inf")
        min_dist = list()
        for d in dist:
            if d < min_v:
                min_v = d
        # check if there are two minimum distances
        for d in dist:
            if d == min_v:
                min_dist.append(d)
        return min_dist

    def get_dist_list(self, pred, current, succ, agent):
        """ Calculate the distances between the endpoints of a possible road
        and the current agent position.
        :param pred: id value of the predecessor road
        :param current: id value of the assumed road
        :param succ: id value of the successor road
        :param agent: current agent position with x and y coordinate
        :return: dist: list with all distance values
        """
        options = [pred, current, succ]
        dist = list()
        for opt in options:
            if opt is None:
                continue
            # check shortest distance to start or endpoints
            start, end = self.get_endpoints(opt)
            start_d = help_functions.euclid_dist(agent, start)
            dist.append(start_d)
            end_d = help_functions.euclid_dist(agent, end)
            dist.append(end_d)
        return dist

    def initial_road_trajectory(self, x_curr: float, y_curr: float,
                                x_target: float, y_target: float,
                                x_next_t: float, y_next_t: float,
                                x_first_t: float, y_first_t: float,
                                yaw: int, command: int):
        """ Create the trajectory on the initial road.
        The agent has to be located on the map. This case has some
        special requirements. We have to define the driving direction
        and the trajectory to the next road segment. The start case
        is treated in an extra method because this kind of planning
        is only needed at the beginning, if we are placed in
        a new town.
            :param x_curr: the current global x position of the agent
            :param y_curr: the current global y position of the agent
            :param x_target: x position of the target waypoint (turn command)
            :param y_target: y position of the target waypoint (turn command)
            :param x_next_t: x position of point after the target point
            :param y_next_t: y position of point after the target point
            :param x_first_t: x position of a lane change before turn
            :param y_first_t: y position of a lane change before turn
            :param yaw: current yaw value of the agent position. Needed for
                        special case calculation, when the agent starts on a
                        junction.
            :param command: next action from the leaderboard
        """
        self.road_id = self.find_current_road(x_curr=x_curr,
                                              y_curr=y_curr)
        self.old_id = self.road_id
        predecessor, successor = self.get_pred_succ(road_id=self.road_id)
        self.follow_id, follow_section_id = self.\
            get_initial_next_road_id(predecessor=predecessor,
                                     successor=successor,
                                     x_target=x_target, y_target=y_target,
                                     yaw=yaw)
        agent_position = (x_curr, y_curr)
        # Interpolate the road_id
        points = self.interpolation(self.road_id)
        points = self.check_point_order(points=points,
                                        x_target=x_next_t,
                                        y_target=y_next_t)
        self.reference = copy.deepcopy(points)
        widths = self.lane_widths(self.road_id)
        self.width = widths[-1]
        self.direction = self.right_or_left(points, x_curr, y_curr,
                                            self.width)
        points = self.calculate_midpoints(points)
        # Check if lane change on first road is needed
        if x_first_t is None and y_first_t is None:
            target = (x_target, y_target)
        else:
            target = (x_first_t, y_first_t)
        # Check if points contain the target point
        min_dist = float("inf")
        index = None
        for i in range(len(points[0])):
            point = (points[0][i], points[1][i])
            dist = help_functions.euclid_dist(point, target)
            if dist < min_dist:
                min_dist = dist
                index = i
        # If min_dist is smaller than 8m, the target point is reached
        # delete points bigger than the target point and calculate
        # last points new based on the target lane for the next action
        if min_dist <= TARGET_DIFF:
            p = points.copy()
            points = self.target_reached(x_target, y_target,
                                         x_next_t, y_next_t, command,
                                         index, self.reference,
                                         follow_section_id, p,
                                         widths, self.direction, True)
        # Find and remove the points that are not needed to pass the
        # first road from the agent position to the end of the road
        min_dist = float("inf")
        index = None
        for i in range(len(points[0])):
            x = points[0][i]
            y = points[1][i]
            point = (x, y)
            dist = help_functions.euclid_dist(point, agent_position)
            if dist < min_dist:
                min_dist = dist
                index = i
        del points[0][0:index]
        del points[1][0:index]
        del points[2][0:index]
        del points[3][0:index]
        self.point_size = len(points[0])
        self.pt = points
        self.follow_section = follow_section_id
        self.waypoints = points

    def calculate_midpoints(self, points: list):
        """ Calculate the trajectory points in the middle
        of a lane and return the points
            :param points: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :return: points: same format as in the parameter,
                     direction: True or False depending on the side
                                of the reference line
        """
        # Calculate points in the middle of the road
        points = self.update_points(points, self.direction, self.width)
        return points

    def target_road_trajectory(self, x_target: float, y_target: float,
                               x_next_t: float, y_next_t: float,
                               command: int):
        """ Calculate the trajectory to the next waypoint
        The waypoints are given by the Carla Leaderboard and the next
        action for the agent
            :param x_target: x position of the target waypoint
            :param y_target: y position of the target waypoint
            :param x_next_t: x position of the next target waypoint
            :param y_next_t: y position of the next target waypoint
            :param command: next progress the agent has to take after he
                            reaches the target point
        """
        while True:
            target = (x_target, y_target)
            still_calculated = False
            if self.pt is not None:
                for i in range(len(self.pt[0])):
                    point = (self.pt[0][i], self.pt[1][i])
                    dist = help_functions.euclid_dist(point, target)
                    if dist < TARGET_DIFF:
                        # Trajectory still calculated
                        still_calculated = True
                        break
            if still_calculated is True:
                # command is turn action
                if command <= 3:
                    if x_next_t is not None and y_next_t is not None:
                        predecessor, successor = self.get_pred_succ(
                            road_id=self.road_id)
                        follow, follow_section_id = self. \
                            get_initial_next_road_id(predecessor=predecessor,
                                                     successor=successor,
                                                     x_target=x_next_t,
                                                     y_target=y_next_t,
                                                     yaw=self.pt[2][-1])
                        self.follow_id = self.next_action_id(
                            x_next_t, y_next_t,
                            follow_section_id,
                            self.pt)
                    break
                # command is lane action
                else:
                    min_dist = float("inf")
                    index = None
                    for i in range(len(self.pt[0])):
                        point = (self.pt[0][i], self.pt[1][i])
                        dist = help_functions.euclid_dist(point, target)
                        if dist < min_dist:
                            min_dist = dist
                            index = i
                    widths = self.lane_widths(self.road_id)
                    points = self.target_reached(x_target, y_target,
                                                 x_next_t, y_next_t,
                                                 command, index,
                                                 self.reference_l,
                                                 self.follow_section,
                                                 self.pt, widths,
                                                 self.direction, False)
                    points = copy.deepcopy(points)
                    start = len(self.waypoints[0]) - len(points[0])
                    for i in range(len(points)):
                        self.waypoints[0][start + i] = points[0][i]
                        self.waypoints[1][start + i] = points[1][i]
                        self.waypoints[2][start + i] = points[2][i]
                        self.waypoints[3][start + i] = points[3][i]
                    self.pt = points
                    self.point_size = len(points[0])
                    break
            else:
                self.road_id = self.follow_id
                predecessor, successor = self.get_pred_succ(
                    road_id=self.road_id)
                self.follow_id, follow_section_id = self. \
                    get_initial_next_road_id(predecessor=predecessor,
                                             successor=successor,
                                             x_target=x_target,
                                             y_target=y_target,
                                             yaw=self.pt[2][-1])
                # Interpolate the road_id
                points = self.interpolation(self.road_id)
                points = self.check_point_order(points=points,
                                                x_target=x_target,
                                                y_target=y_target)
                self.reference[0] += copy.deepcopy(points[0])
                self.reference[1] += copy.deepcopy(points[1])
                self.reference[2] += copy.deepcopy(points[2])
                self.reference[3] += copy.deepcopy(points[3])
                reference_line = copy.deepcopy(points)
                # all lane widths in this road
                # first width is for the road next to the reference line
                # and so on
                widths = self.lane_widths(self.road_id)
                old_w = self.lane_widths(self.old_id)
                # if previous lane width was the rightmost one
                if old_w.index(self.width) + 1 == len(old_w):
                    last_p = (self.pt[0][-1], self.pt[1][-1])
                    min_diff = float("inf")
                    w_min = None
                    for width in widths:
                        p, v = self.update_one_point(
                            points[0][0], points[1][0],
                            points[0][1], points[1][1],
                            points[0][2], points[1][2],
                            self.direction, width)
                        diff = help_functions.euclid_dist(p, last_p)
                        if diff < min_diff:
                            min_diff = diff
                            w_min = width
                    self.width = w_min
                else:
                    min_diff = float("inf")
                    min_width = None
                    for width in widths:
                        diff = abs(width - self.width)
                        if diff < min_diff:
                            min_diff = diff
                            min_width = width
                    self.width = min_width
                points = self.calculate_midpoints(points)
                if command == LEFT or command == RIGHT or command == STRAIGHT:
                    if x_next_t is not None and y_next_t is not None:
                        self.follow_id = self.next_action_id(x_next_t,
                                                             y_next_t,
                                                             follow_section_id,
                                                             points)
                self.pt = points
                self.reference_l = reference_line
                self.follow_section = follow_section_id

                if command >= 4:
                    min_dist = float("inf")
                    index = None
                    for i in range(len(points[0])):
                        point = (points[0][i], points[1][i])
                        dist = help_functions.euclid_dist(point, target)
                        if dist < min_dist:
                            min_dist = dist
                            index = i
                    points = self.target_reached(x_target, y_target,
                                                 x_next_t, y_next_t,
                                                 command,
                                                 index, reference_line,
                                                 follow_section_id, points,
                                                 widths, self.direction, False)
                    self.add_waypoints(points)
                    self.pt = points
                    self.old_id = self.road_id
                    self.point_size = len(points[0])
                    break
                self.pt = points
                self.old_id = self.road_id
                self.add_waypoints(points)
                break

    def target_reached(self, target_x: float, target_y: float,
                       x_next_t: float, y_next_t: float,
                       command: int, index: int,
                       reference_line, follow_section_id: int,
                       points_calc, widths: list,
                       direction: bool, initial: bool):
        """ If a lane change is detected, the trajectory needs to be
        interpolated again and update the waypoints where the lane change
        should occur.
        :param target_x: x coordinate of the target point
        :param target_y: y coordinate of the target point
        :param x_next_t: x coordinate of the next target point
        :param y_next_t: y coordinate of the next target point
        :param command: action command from the leaderboard
        :param index: index of the starting point of the lane change
        :param reference_line: the current interpolated reference line
        :param follow_section_id: if value of the following road
        :param points_calc: current calculated trajectory points
        :param widths: possible lane width values
        :param direction: drive on the right or left side of the road
        :param initial: indicates if it is the first interpolated road or not
        :return: points_calc: new calculated points
        """
        if command == CHANGE_LEFT or \
           command == CHANGE_RIGHT or command == CHANGE_FOLLOW:
            if command == CHANGE_LEFT:
                points = reference_line
                last_width = self.width
                step_size = STEP_SIZE
                ind = widths.index(last_width)
                if ind == 0:
                    return points_calc
                new_width = widths[ind-1]
                self.width = new_width
                diff = abs(last_width - new_width)
                steps = int(diff / step_size)
                first_widths = [last_width - step_size * i
                                for i in range(steps)]
                for i in range(len(first_widths)):
                    p1_x = points[0][index + i]
                    p1_y = points[1][index + i]
                    p2_x = points[0][index + i + 1]
                    p2_y = points[1][index + i + 1]
                    if i != len((points[0])) - 2:
                        p3_x = points[0][index + i + 2]
                        p3_y = points[1][index + i + 2]
                    point, v = self.update_one_point(p1_x, p1_y,
                                                     p2_x, p2_y,
                                                     p3_x, p3_y,
                                                     direction,
                                                     first_widths[i])
                    points_calc[0][index + i] = point[0]
                    points_calc[1][index + i] = point[1]
                for i in range(index + len(first_widths), len(points[0])-1):
                    p1_x = points[0][i]
                    p1_y = points[1][i]
                    p2_x = points[0][i + 1]
                    p2_y = points[1][i + 1]
                    if i != len((points[0])) - 2:
                        p3_x = points[0][i + 2]
                        p3_y = points[1][i + 2]
                    else:
                        p3_x = points[0][-1]
                        p3_y = points[1][-1]
                    point, v = self.update_one_point(p1_x, p1_y,
                                                     p2_x, p2_y,
                                                     p3_x, p3_y,
                                                     direction,
                                                     new_width)
                    points_calc[0][i] = point[0]
                    points_calc[1][i] = point[1]
                point, v = self.update_one_point(p2_x, p2_y,
                                                 target_x, target_y,
                                                 target_x, target_y,
                                                 direction,
                                                 new_width)
                points_calc[0][i + 1] = point[0]
                points_calc[1][i + 1] = point[1]
            # change lane right
            elif command == CHANGE_RIGHT:
                points = reference_line
                last_width = self.width
                step_size = STEP_SIZE
                ind = widths.index(last_width)
                if ind == len(widths) - 1:
                    return points_calc
                new_width = widths[ind + 1]
                self.width = new_width
                diff = abs(last_width - new_width)
                steps = int(diff / step_size)
                first_widths = [last_width + step_size * i
                                for i in range(steps)]
                for i in range(len(first_widths)):
                    p1_x = points[0][index + i]
                    p1_y = points[1][index + i]
                    p2_x = points[0][index + i + 1]
                    p2_y = points[1][index + i + 1]
                    if i != len((points[0])) - 2:
                        p3_x = points[0][index + i + 2]
                        p3_y = points[1][index + i + 2]
                    point, v = self.update_one_point(p1_x, p1_y,
                                                     p2_x, p2_y,
                                                     p3_x, p3_y,
                                                     direction,
                                                     first_widths[i])
                    points_calc[0][index + i] = point[0]
                    points_calc[1][index + i] = point[1]
                for i in range(index + len(first_widths), len(points[0])-1):
                    p1_x = points[0][i]
                    p1_y = points[1][i]
                    p2_x = points[0][i + 1]
                    p2_y = points[1][i + 1]
                    if i != len((points[0])) - 2:
                        p3_x = points[0][i + 2]
                        p3_y = points[1][i + 2]
                    point, v = self.update_one_point(p1_x, p1_y,
                                                     p2_x, p2_y,
                                                     p3_x, p3_y,
                                                     direction,
                                                     new_width)
                    points_calc[0][i] = point[0]
                    points_calc[1][i] = point[1]
                point, v = self.update_one_point(p2_x, p2_y,
                                                 target_x, target_y,
                                                 target_x, target_y,
                                                 direction,
                                                 new_width)
                points_calc[0][i + 1] = point[0]
                points_calc[1][i + 1] = point[1]
        # passing a junction action
        else:
            if x_next_t is not None and y_next_t is not None:
                predecessor, successor = self.get_pred_succ(
                    road_id=self.road_id)
                follow, follow_section_id = self. \
                    get_initial_next_road_id(predecessor=predecessor,
                                             successor=successor,
                                             x_target=x_next_t,
                                             y_target=y_next_t,
                                             yaw=self.pt[2][-1])
                self.follow_id = self.next_action_id(x_next_t, y_next_t,
                                                     follow_section_id,
                                                     points_calc)
            if initial is False:
                del points_calc[0][index + 1:]
                del points_calc[1][index + 1:]
                del points_calc[2][index + 1:]
                del points_calc[3][index + 1:]
        return points_calc

    def rad_to_degree(self, radians):
        """ Convert radians value to degrees
            :param radians: heading value in rad
            :return: deg: degree value
        """
        radians = abs(radians)
        deg = degrees(radians)
        if deg > 360:
            deg = 360 - degrees(radians)
        return deg

    def next_action_id(self, x_next_t: float, y_next_t: float,
                       sec_id: int, points: list):
        """ Calculate the next road id for the given action from
        the leaderboard
        :param x_next_t: x coordinate of the next target point
        :param y_next_t: y coordinate of the next target point
        :param sec_id: section id of the road or the junction
        :param points: points: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        :return: action_id: the id of the next road to take
        """
        current_road = self.roads[sec_id]
        # if current road is not a junction, the follow_road is needed
        # to calculate the new action road
        if current_road is None:
            junction = sec_id
            incoming_road = self.road_id
            possible_road_ids = self.filter_road_ids(junction,
                                                     incoming_road)
            last_point_x = points[0][-1]
            last_point_y = points[1][-1]
            last_point = (last_point_x, last_point_y)
            action_id = self.calculate_action_id(possible_road_ids,
                                                 last_point,
                                                 x_next_t, y_next_t)
        else:
            action_id = self.follow_id
        return action_id

    def calculate_action_id(self, possible_road_ids: list,
                            last_point: Tuple[float, float],
                            x_next_t: float, y_next_t: float):
        """ Calculate the next road to take from the junction based
        on the next action from the leaderboard
            :param possible_road_ids: list of the next possible road ids
            :param last_point: last calculated point of the trajectory
            :param x_next_t: x coordinate of the next target point
            :param y_next_t: y coordinate of the next target point
            :return: possible_road_ids: the id of the next road
        """
        endpoint_values = list()
        # filter the endpoints for each possible road
        for road_id in possible_road_ids:
            start, end = self.get_endpoints(road_id)
            dist_start = help_functions.euclid_dist(start, last_point)
            dist_end = help_functions.euclid_dist(end, last_point)
            if dist_start < dist_end:
                endpoint_values.append(end)
            else:
                endpoint_values.append(start)
        min_diff = float("inf")
        min_pt = None
        # calculate the closest endpoint to the next target point
        for pt in endpoint_values:
            diff = help_functions.euclid_dist(pt, (x_next_t, y_next_t))
            if diff < min_diff:
                min_diff = diff
                min_pt = pt
        index = endpoint_values.index(min_pt)
        return possible_road_ids[index]

    def filter_road_ids(self, junction: int, incoming: int):
        """ Filter the road id values of all connecting roads
        that are linked to the incoming road
            :param junction: id value of the junction
            :param incoming: id value of the incoming road
            :return: road_ids: list of the connecting road ids
        """
        j = self.junctions[junction]
        jun = j.findall("connection")
        roads = [j for j in jun if j.get("incomingRoad") == str(incoming)]
        road_ids = [int(r.get("connectingRoad")) for r in roads]
        for id in road_ids:
            widths = self.lane_widths(id)
            if len(widths) == 0:
                road_ids.remove(id)
        return road_ids

    def lane_widths(self, road_id: int):
        """ Filter all lane width values from a given road
            :param road_id: the id value of the examined road
            :return: widths: list of all width values
        """
        road = self.roads[road_id]
        lanes = road.find("lanes")
        sections = lanes.findall("laneSection")
        # if the road contains more than one lane section we need to find
        # the whole widths values
        if len(sections) > 1:
            driv = []
            direction = sections[0].find("right")
            side = "right"
            if direction is None:
                direction = sections[0].find("left")
                side = "left"
            for lane in sections:
                drive = []
                direction = lane.find(side)
                lane_ids = direction.findall("lane")
                for ids in lane_ids:
                    if ids.get("type") == "driving":
                        if drive.__contains__(ids) is False:
                            drive.append(ids)
                driv.append(drive)
                max_size = len(driv[0])
                driving = driv[0]
                for d in driv:
                    if len(d) > max_size:
                        max_size = len(d)
                        driving = d
        else:
            direction = sections[0].find("right")
            if direction is None:
                direction = sections[0].find("left")
            lanes = direction.findall("lane")
            driving = [lane for lane in lanes if lane.get("type") == "driving"]
        if len(driving) == 0:
            return []
        width = driving[0].find("width").get("a")
        # agent drives in the middle of the street
        widths = list()
        width = float(width)
        middle = width / 2.0
        widths.append(middle)
        for i in range(1, len(driving)):
            widths.append(middle + width * i)
        return widths

    def right_or_left(self, points: list, x_agent: float, y_agent: float,
                      width: float):
        """ Define on which side of the reference line the trajectory
        is running. If it returns true the update point function
        will choose the correct function for the update of the
        trajectory points
            :param points: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param x_agent: current x position of the agent
            :param y_agent: current y position of the agent
            :param width: width of the lane
            :return: direction: True or False if the agent is on the right
                                side of the reference line or not
        """
        direction = True
        x_start = points[0][0]
        y_start = points[1][0]
        x_follow = points[0][1]
        y_follow = points[1][1]
        point_x = points[0][2]
        point_y = points[1][2]
        point1, v = self.update_one_point(x_start, y_start,
                                          x_follow, y_follow,
                                          point_x, point_y,
                                          True, width)
        point2, v = self.update_one_point(x_start, y_start,
                                          x_follow, y_follow,
                                          point_x, point_y,
                                          False, width)
        agent = (x_agent, y_agent)
        dist_1 = help_functions.euclid_dist(point1, agent)
        dist_2 = help_functions.euclid_dist(point2, agent)
        if dist_1 > dist_2:
            direction = False
        return direction

    def update_one_point(self, point1_x: float, point1_y: float,
                         point2_x: float, point2_y: float,
                         point3_x: float, point3_y: float,
                         right: bool, width: float):
        """ Update the coordinates of a point width the given
        width value for the correct lane
            :param point1_x: x coordinate of the point to update
            :param point1_y: y coordinate of the point to update
            :param point2_x: x coordinate of the following point
            :param point2_y: y coordinate of the following point
            :param point3_x: x coordinate of the next following point
            :param point3_y: y coordinate of the next following point
            :param right: True or False for the correct side of the lane
            :param width: value for the width of the lane
            :return: point: coordinates of the updated point
                     vector: last added vector
        """
        first = (point1_x, point1_y)
        second = (point2_x, point2_y)
        vector = help_functions.sub_vector(second, first)
        # if dist to the next point is to small -> outliers
        if vector[0] < SMALL_DIST and vector[1] < SMALL_DIST:
            vector = help_functions.sub_vector((point3_x, point3_y), first)
        if right is True:
            vec = help_functions.perpendicular_vector_right(vector)
        else:
            vec = help_functions.perpendicular_vector_left(vector)
        vector = help_functions.unit_vector(vec, width)
        point = help_functions.add_vector(first, vector)
        return point, vector

    def update_points(self, p_list: list, right: bool, width: float):
        """ Update the coordinates of a point list width the given
        width value for the correct lane
            :param p_list: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param right: True or False if the agent is on the right
                          side of the reference line or not
            :param width: width of the lane
            :return: p_list: point list of the format from the parameter
        """
        i = 0
        for i in range(len(p_list[0]) - 1):
            point1_x = p_list[0][i]
            point1_y = p_list[1][i]
            point2_x = p_list[0][i + 1]
            point2_y = p_list[1][i + 1]
            # point if dist is too small
            if i != len((p_list[0])) - 2:
                point3_x = p_list[0][i + 2]
                point3_y = p_list[1][i + 2]
            point, vector = self.update_one_point(point1_x, point1_y,
                                                  point2_x, point2_y,
                                                  point3_x, point3_y,
                                                  right, width)
            p_list[0][i] = point[0]
            p_list[1][i] = point[1]
        point = (p_list[0][i + 1], p_list[1][i + 1])
        point = help_functions.add_vector(point, vector)
        p_list[0][i + 1] = point[0]
        p_list[1][i + 1] = point[1]
        return p_list

    def add_waypoints(self, points: list):
        """ Add calculated points to the trajectory list
            :param points: list of all trajectory points
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        """
        x = copy.deepcopy(points[0])
        y = copy.deepcopy(points[1])
        hdg = copy.deepcopy(points[2])
        speed = copy.deepcopy(points[3])
        self.waypoints[0] += x
        self.waypoints[1] += y
        self.waypoints[2] += hdg
        self.waypoints[3] += speed

    def check_point_order(self, points: list, x_target: float,
                          y_target: float):
        """ Check if the trajectory points have the correct order
            :param points: list of all trajectory points
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param x_target: x coordinate of the target point
            :param y_target: y coordinate of the target point
            :return: points: same format as the parameter points
        """
        target = (x_target, y_target)
        start_x = points[0][0]
        start_y = points[1][0]
        end_x = points[0][-1]
        end_y = points[1][-1]
        start = (start_x, start_y)
        end = (end_x, end_y)
        if self.pt is not None:
            x_last = self.pt[0][-1]
            y_last = self.pt[1][-1]
            last = (x_last, y_last)
            dist_start = help_functions.euclid_dist(last, end)
            dist_end = help_functions.euclid_dist(last, start)
        else:
            dist_start = help_functions.euclid_dist(start, target)
            dist_end = help_functions.euclid_dist(end, target)
        # if start distance is smaller, this point is nearer to the target,
        # but on the first list entry -> sort list
        if dist_start < dist_end:
            x = points[0][::-1]
            y = points[1][::-1]
            yaw = points[2][::-1]
            speed = points[3][::-1]
            points = [x, y, yaw, speed]
        return points

    def get_speed(self, road_id: int):
        """ Filter and calculate the max_speed for the road
            :param road_id: id value for the road
            :return: speed: speed value for the road in m/s
        """
        road = self.roads[road_id]
        speed_type = road.find("type").find("speed").get("unit")
        if road.get("junction") == "-1":
            if speed_type == "km/h":
                speed = road.find("type").find("speed")
                speed = float(speed.get("max")) / KMH
            elif speed_type == "mph":
                speed = road.find("type").find("speed")
                speed = float(speed.get("max")) / MPH
        else:
            speed = DEFAULT_MS
        return speed

    def interpolation(self, road_id: int):
        """ Interpolate over a complete road
            :param road_id: id value of the current road
            :return: waypoints: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        """
        x = list()
        y = list()
        yaw = list()
        speed = list()
        size = len(self.geometry_data[road_id][0])
        max_speed = self.get_speed(road_id)
        for i in range(size):
            x_start = self.geometry_data[road_id][0][i]
            y_start = self.geometry_data[road_id][1][i]
            start = (x_start, y_start)
            length = self.geometry_data[road_id][4][i]
            hdg = self.geometry_data[road_id][2][i]
            if self.geometry_data[road_id][3][i] == LINE:
                xd = length * cos(hdg)
                yd = length * sin(hdg)
                end = help_functions.add_vector(start, (xd, yd))
                points = help_functions.linear_interpolation(
                    start=start,
                    end=end,
                    interval_m=INTERVALL)
            else:
                radius = self.geometry_data[road_id][3][i]
                end = help_functions.end_of_circular_arc(
                    start_point=start, angle=hdg,
                    length=length, radius=radius)
                points = help_functions.circular_interpolation(
                    start=start,
                    end=end,
                    arc_radius=radius)
            for j in range(len(points)):
                x.append(points[j][0])
                y.append(points[j][1])
                yaw.append(hdg)
                speed.append(max_speed)
        points = [x, y, yaw, speed]
        # Delete duplicates and very close points
        points = self.remove_outliner(points)
        return points

    def remove_outliner(self, points):
        """ Function checks if distance between two following trajectory
        points is to small or to big. Delete and update waypoints if necessary.
        :param points: points: list of all trajectory points
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        :return: points: updated waypoint list
        """
        delete_index = []
        for i in range(0, len(points[0]) - 1):
            p = (points[0][i], points[1][i])
            p_next = (points[0][i + 1], points[1][i + 1])
            dist = help_functions.euclid_dist(p, p_next)
            # point is to close to the following point (0.5m)
            if dist < 0.5:
                delete_index.append(i+1)
            # outliner point
            elif dist > 3:
                delete_index.append(i+1)
        # delete the points with the calculated indices
        number = 0
        for i in delete_index:
            i -= number
            del points[0][i]
            del points[1][i]
            del points[2][i]
            del points[3][i]
            number += 1
        return points

    def get_endpoints(self, road_id: int):
        """ Calculate the startpoint and endpoint of a given road
            :param road_id: the road id of the examined road
            :return: start_point, end_point
                start_point: x and y coordinate of the starting point
                end_point: x and y coordinate of the ending point
        """
        size = len(self.geometry_data[road_id][0])
        x_start = self.geometry_data[road_id][0][0]
        y_start = self.geometry_data[road_id][1][0]
        start_point = (x_start, y_start)

        length = self.geometry_data[road_id][4][size - 1]
        hdg = self.geometry_data[road_id][2][size - 1]
        # calculate the start point of the last part of the road
        # to calculate the endpoint of the whole road
        x = self.geometry_data[road_id][0][size - 1]
        y = self.geometry_data[road_id][1][size - 1]
        last_start = (x, y)

        # check the last curvature value to see if it is line or arc
        if self.geometry_data[road_id][3][size-1] == LINE:
            xd = length * cos(hdg)
            yd = length * sin(hdg)
            # subtract a small value due to inaccuracy
            if xd > 0:
                xd -= 0.05
            else:
                xd += 0.05
            if yd > 0:
                yd -= 0.05
            else:
                yd += 0.05
            end_point = help_functions.add_vector(last_start, (xd, yd))
        else:
            radius = self.geometry_data[road_id][3][size-1]
            end_point = help_functions.end_of_circular_arc(
                start_point=last_start, angle=hdg,
                length=length, radius=radius)
        return start_point, end_point

    def get_initial_next_road_id(self, predecessor: int,
                                 successor: int,
                                 x_target: float, y_target: float,
                                 yaw: int):
        """ Find the next road to drive
        When the agent starts driving it is not sure if he has to follow his
        successor or his predecessor. This function calculates the next road
        id, based on the dist to the target point. The road, who is nearer to
        this point is chosen
            :param predecessor: id value for predecessor road
            :param successor: id value for successor road value
            :param x_target: x position of the target waypoint
            :param y_target: y position of the target waypoint
            :param yaw: current yaw value
            :return: final_id: the id value of the chosen road
                     section_id: the id value of the chosen predecessor
                                 or successor
        """
        # value of predecessor or successor (can also be a junction id)
        section_id = None
        target = (x_target, y_target)
        # When succ or pred are a junction than i should calculate
        # the next id
        if predecessor is None:
            final_id = successor
        elif successor is None:
            final_id = predecessor
        else:
            min_distances = list()
            x_road_p, y_road_p, pred = self.\
                get_next_road_point(predecessor, yaw)
            point1, point2 = self.get_endpoints(pred)
            dist1 = help_functions.euclid_dist(point1, target)
            min_distances.append(dist1)
            dist2 = help_functions.euclid_dist(point2, target)
            min_distances.append(dist2)

            x_road_s, y_road_s, succ = self.\
                get_next_road_point(successor, yaw)
            point3, point4 = self.get_endpoints(succ)
            dist3 = help_functions.euclid_dist(point3, target)
            min_distances.append(dist3)
            dist4 = help_functions.euclid_dist(point4, target)
            min_distances.append(dist4)

            dist = min(min_distances)
            ind = min_distances.index(dist)
            if ind == 0 or ind == 1:
                final_id = pred
                section_id = predecessor
            else:
                final_id = succ
                section_id = successor
        return final_id, section_id

    def get_pred_succ(self, road_id: int):
        """ Find the predecessor and the successor road of the current road
        If there is only a successor or only a predecessor, this
        function handles these cases
            :param road_id: id of the current road
            :return: predecessor: road id of the predecessor or None
                                  if there is no predecessor
                     successor: road id of the successor or None
                                if there is no successor
        """
        # start road reference
        curr_road = self.roads[road_id]
        link = curr_road.find("link")
        # Road needs a successor or predecessor
        assert (len(link) > 0)
        # if only one following road
        if len(link) == 1:
            next_road_id = link[0].get("elementId")
            next_road_tag = link[0].tag
            if next_road_tag == "predecessor":
                predecessor = int(next_road_id)
                successor = None
            else:
                predecessor = None
                successor = int(next_road_id)
        # predecessor and successor -> only choose which direction
        # to drive
        else:
            predecessor = int(curr_road.find("link").find("predecessor").
                              get("elementId"))
            successor = int(curr_road.find("link").find("successor").
                            get("elementId"))
        return predecessor, successor

    def get_next_road_point(self, road_id: int, yaw: int):
        """ The function returns the x and y coordinate for a given road
            :param road_id: the id value of the preferred road
            :param yaw: yaw value of the agent
            :return: x, y, road_id
                x: value of the x coordinate
                y: value of the y coordinate
                road_id: id of the chosen road
        """
        line_list = list()
        # check if it is a junction
        if self.geometry_data[road_id] is None:
            connections = self.junctions[road_id].findall("connection")
            # search through all possible connections and select the first
            # connectingRoad which is linear (if we have no command at a
            # junction we pass it linear)
            for c in connections:
                # line indicates if the whole road is of the type line
                line = True
                id = int(c.get("connectingRoad"))
                road_data = self.geometry_data[id]
                for i in range(len(road_data[0])):
                    # check if there is a straight road
                    # can also have a small radius -> (1/0.001)
                    if abs(road_data[3][i]) >= (1 / 0.001):
                        line = False
                        break
                if line is True:
                    line_list.append(id)
            min_diff = float("inf")
            for li in line_list:
                y = self.geometry_data[li][2][0]
                diff = abs(abs(yaw) - abs(self.rad_to_degree(y)))
                if diff < min_diff:
                    min_diff = diff
                    road_id = li
        # first x value on this road
        x = self.geometry_data[road_id][0][0]
        # first value on this road
        y = self.geometry_data[road_id][1][0]
        return x, y, road_id
