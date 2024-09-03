# Implementation of various pathfinding algorithms in Python for navigating a grid-based map. The algorithms included are:

- **A* Search**
- **Breadth-First Search (BFS)**
- **Depth-First Search (DFS)**
- **Uniform Cost Search (UCS)**

These algorithms are applied to a grid map to find the optimal path from a start position to a goal position, avoiding obstacles.

## Project Description

The project uses the following parameters and a grid map for the navigation algorithms:

### Parameters

- `sx`: Start x-coordinate (float)
- `sy`: Start y-coordinate (float)
- `gx`: Goal x-coordinate (float)
- `gy`: Goal y-coordinate (float)
- `grid_size`: Size of each grid cell (float)
- `robot_radius`: Radius of the robot (float), affecting obstacle avoidance
- `fig_dim`: Dimensions of the grid for visualization (int)
- `map_xlsx`: Path to the Excel file containing the grid map data (string)

### Grid Map

The grid map is defined in the `map15X20` variable, which is a 2D matrix where:

- `1` represents obstacles
- `0` represents free space

Example of `map15X20`:
1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	1
1	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	1	1
1	0	0	1	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	1	0	0	0	1	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
1	0	1	0	1	0	0	0	1	0	0	0	0	0	0	0	0	1	0	1	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1

## Algorithms

### A* Search
The A* algorithm finds the shortest path using a combination of actual path cost and estimated cost to the goal. It is efficient and effective for grid-based pathfinding.

### Breadth-First Search (BFS)
BFS explores all possible moves level by level from the start node, guaranteeing the shortest path in an unweighted grid.

### Depth-First Search (DFS)
DFS explores as far as possible along each branch before backtracking. It does not guarantee the shortest path but is useful for exploring paths.

### Uniform Cost Search (UCS)
UCS expands the least costly node first. It guarantees the shortest path in weighted grids.

## Installation

1. Clone this repository:

   ```bash
   git clone https://github.com/yourusername/advanced-pathfinding-algorithms.git
