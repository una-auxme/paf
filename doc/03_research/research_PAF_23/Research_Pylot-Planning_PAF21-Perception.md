# Sprint 0: Research Samuel Kühnel

## Pylot

## Planning

- 4 different options
- **Waypoint Planner**: Auto follows predefined waypoints. It recognizes traffic lights and stops at obstacles, but cannot avoid them
- **Freenet-Optimal-Trajecotry-Planner**: CPP code with Python wrapper ([GitHub](https://github.com/erdos-project/frenet_optimal_trajectory_planner))
→ Predefined line that is used for orientation → Can avoid obstacles!

![freenet_gif](https://github.com/erdos-project/frenet_optimal_trajectory_planner/raw/master/img/fot2.gif)

- **RRT\*-Planner**: RRT* algorithm for path planning ([GitHub](https://github.com/erdos-project/rrt_star_planner))
  - Creates random nodes
  - Adds nodes to the graph that are not blocked by objects on the road
  - Generally terminates as soon as a node is found in the target area
  - RRT*: Searches for the shortest path

![rrt_star_gif](https://github.com/erdos-project/rrt_star_planner/raw/master/img/rrtstar.gif)

- **Hybrid A\* planner**: Hybrid A* algorithm for path planning ([GitHub](https://github.com/erdos-project/hybrid_astar_planner))
  - Calculates the shortest path between two nodes from a graph
  - Similar to Dijkstra's algorithm
  - Nodes are estimated based on their costs and promising nodes are selected first
  - Hybrid A* algorithm: Not always optimal solution, but in the neighborhood of the optimal solution.

![hybrid_astar_gif](https://github.com/erdos-project/hybrid_astar_planner/raw/master/img/straight_obstacle.gif)

## PAF 21-2

### Perception

### Obstacle detection

- Detect objects via semantic lidar sensor
- Provides x and y coordinates, as well as distance value
- Additional information on position change in a time interval → Calculation of speed possible
- Detects the object and returns either the value "Vehicle" or "Pedestrian"

### TrafficLightDetection

![diagramm.png](https://github.com/ll7/paf21-2/raw/main/docs/imgs/trafficlightdetection_diagram.jpg)

- **FusionCamera** saves images from **RGBCamera** and **DepthCamera** with timestamp and then synchronizes with **SegmentationCamera**
- Neural network based on [ResNet18](https://pytorch.org/hub/pytorch_vision_resnet/) (predefined PyTorch network)
- Generally only traffic lights up to 100m distance
- Canny algorithm to filter contours

### Problems and solutions

- Red background distorts traffic light phase detection → **Solution**: Narrow section of the traffic light image for phase detection
- Yellow painted traffic lights distort traffic light phase detection → **Solution**: Filter out red and green sections beforehand using masks and convert remaining image to grayscale and add masks again.
- **Problem without solution**: European traffic lights can sometimes not be recognized at the stop line.

## Resumee

### Perception

- Status quo: LIDAR Sensor and Big Neural Network
- Possible focus: Using smaller (pretrained) models to improve overall performance fast
- Taking known issues into account

### Planning

- Currently decision tree for evaluating the current position
- Trying out different heuristics → already given as repo
