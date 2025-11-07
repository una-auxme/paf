uint8 TYPE_UNKNOWN=0
uint8 TYPE_INTERSECTION=1
uint8 TYPE_LANECHANGE=2

uint8 waypoint_type
float32 distance
# Road option enum: from agents.navigation.local_planner import RoadOption
int8 road_option
geometry_msgs/Point position
