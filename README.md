# -Information-Fusion-Project

The code is provided for research and educational purposes only.
The author does not assume any responsibility for improper use or damages caused by this software.

## Overview
This project studies the **map fusion problem** in a multi-robot system.  
Each robot builds a local occupancy grid map using its onboard sensors. These local maps can then be fused into a consistent global map.  

The project investigates both theoretical and experimental aspects:
- Mathematical framework based on the **Kullback-Leibler Average (KLA)** for information fusion  
- Initial simulation experiments  
- More realistic tests with noisy sensors  
- Algorithms based on **travelled distance** and **local efficiency**  

## Features
- Occupancy grid representation  
- Probabilistic map fusion using KLA  
- Noise models for realistic sensing conditions  
- Efficiency metrics related to robot trajectories  
- Example experiments and resulting fused maps  
