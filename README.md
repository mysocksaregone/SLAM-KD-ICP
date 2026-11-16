# Robot Localization with KD-Trees and ICP

<!-- TODO: sections to fill in

## Overview
What the project is, why odometry drifts, and how ICP fixes it.

## How it works
### KD-trees
Median-split build, nearest-neighbor search with pruning, complexity
(build O(M log M), query ~O(log M) average, O(M) worst case).
### ICP
The two-step loop, SVD alignment, outlier rejection, convergence.

## Setup
ROS 2 version, dependencies (numpy, scikit-learn, matplotlib, nav2_map_server),
clone into ros2_ws/src, colcon build, source install/setup.bash.

## Running
- Unit tests: python3 -m pytest test/
- Extract bag / offline ICP / benchmarks: scripts/
- Live on the bag: launch file + ros2 bag play --clock + RViz 2D Pose Estimate

## Code layout
One line per file (robot_localization/, test/, scripts/, launch/, rviz/).

## Results
Benchmark plots (results/), odom vs ICP path, ICP time per scan.

## Failure cases
At least two, with explanation.

## Lessons learned
-->
