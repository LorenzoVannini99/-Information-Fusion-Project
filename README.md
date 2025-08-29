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


# Autonomous Agent – Information Fusion for Multi-Robot Mapping

## Mathematical Framework

### Occupancy Grid
- The environment is divided into cells θ_k ∈ {0,1}.
- Each robot i assigns a probability P_i(θ_k) ∈ [0,1].
- A threshold ε is used:
  - Free if P_i(θ_k) < ε
  - Occupied if P_i(θ_k) > 1 − ε
  - Uncertain otherwise
If no prior knowledge is available:


$$ SE = \frac{\sigma}{\sqrt{n}} $$

