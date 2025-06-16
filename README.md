# Path-Planners-Interactive-GUI (CPP Python bridge)

###  Classical Robotics Path Planners written from scratch, visualized in interactive GUI - capable of adding obstacles, start-end points and visualizing visited and potential nodes.
* This repository features 3 main path planning algorithms which are written in C++, working as a backend for the GUI written in python.
* C++ algorithms and C++ <-> python bridge (via boost python) is self written.
* The credit for GUI goes to Dr. Claus Brenner.

## A* Algorithm
| Simple obstacle free path | Obstacle path | 
|:-------:|:-----------------:|
|  <img src="Astar/results/ss_1.png" width="700"> | <img src="Astar/results/ss_4.png" width="700">|

## BFS Algorithm
| Simple obstacle free path | Obstacle path | 
|:-------:|:-----------------:|
|  <img src="BFS/results/ss_3.png" width="700"> | <img src="BFS/results/ss_4.png" width="700">|

## BFS Algorithm
| Simple obstacle path | Complex Obstacle path | 
|:-------:|:-----------------:|
|  <img src="Dijkstra/results/ss_2.png" width="700"> | <img src="Dijkstra/results/ss_1.png" width="700">|
