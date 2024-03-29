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
