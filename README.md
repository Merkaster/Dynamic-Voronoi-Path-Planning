# Dynamic-Voronoi-Path-Planning
Path planing for mobile robotics avoiding obstacles using Dynamic Voronoi diagrams . Was builded using ROS 

Using ROS we tried to build an algorithm that uses voronoi diagrams to plan a path for a mobile robot(P3dx) from a starting position to a target position using no map.
This algorith was built using ROS and Python.
We used MobileSim for the visualization and RosAria for the control of the robot.
The algorithm creates dynamic voronoi diagrams by detecting if an obstacle is ahead.
After the voronoi diagram has been computed. Dijkstra was used for the shortest path.
The robot has 3 choices:
  - Moving to an already pre-computed voronoi node following the path that was generated by Dijkstra.
  - Going to the target position.
  - Creating a new Voronoi diagram.
