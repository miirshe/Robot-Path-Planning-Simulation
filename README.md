# Robot Path Planning Simulation

## Overview
This project implements A* pathfinding algorithm for robot navigation in dynamic environments, featuring both single-target and multi-target path planning capabilities. The simulation provides real-time visualization and detailed performance metrics.

## Features

### Single-Target Path Planning
- Efficient A* algorithm implementation
- Real-time path visualization
- Performance metrics tracking
- Detailed path reporting
- Color-coded visualization

### Multi-Target Path Planning
- Sequential target visiting
- Multiple target optimization
- Distinct target visualization
- Comprehensive performance tracking
- Path optimization between targets

## Visualization Elements
- 🟢 Walls (Green)
- ⬜ Free Space (White)
- 🔴 Robot Start Position (Red)
- 🔵 Target 1 (Blue)
- 🟣 Target 2 (Magenta)
- 🟡 Target 3 (Yellow)
- 🔷 Target 4 (Cyan)
- 💛 Path (Yellow)

## Performance Metrics
- Nodes Explored
- Path Length
- Execution Time
- Targets Reached
- Environment Statistics

## Output Examples

### Single-Target Path Planning
<img width="843" alt="Screenshot 2025-05-13 at 11 17 45 PM" src="https://github.com/user-attachments/assets/52a0c8a0-b375-4b72-9ee3-729f63e77945" />

*Single target path planning visualization showing the optimal path from start to target*

### Multi-Target Path Planning
<img width="843" alt="Screenshot 2025-05-13 at 11 15 34 PM" src="https://github.com/user-attachments/assets/7c6b038b-a29d-49e9-b9e4-9b30fab56451" />
*Multi-target path planning visualization showing the sequence of paths to visit all targets*

## How to Run

### Compilation
```bash
# Compile single target simulation
g++ -std=c++11 astar_single_target_sim.cpp -o astar_single_target

# Compile multi target simulation
g++ -std=c++11 astar_multi_target_sim.cpp -o astar_multi_target
```

### Execution
```bash
# Run single target simulation
./astar_single_target

# Run multi target simulation
./astar_multi_target
```

## Performance Reports
The simulation generates detailed performance reports:
- `single_target_report.txt` for single-target simulation
- `performance_report.txt` for multi-target simulation

## Project Structure
```
robot-path-planner/
├── astar_single_target_sim.cpp    # Single target implementation
├── astar_multi_target_sim.cpp     # Multi target implementation
├── single_target_report.txt       # Single target performance report
├── performance_report.txt         # Multi target performance report
├── single_target_output.png       # Single target visualization
├── multi_target_output.png        # Multi target visualization
└── README.md                      # Project documentation
```

## Requirements
- C++11 or higher
- Terminal with ANSI color support
- Unix-like environment (for clear screen functionality)

## Author
[Abdikafi Isse Isak - miirshe]

## Course
MCS240021 Advanced Algorithms 
