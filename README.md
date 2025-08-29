# Information-Fusion-Project

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


## Mathematical Framework

### Occupancy Grid
We model the environment as an **occupancy grid**, where each cell is a binary random variable:

$$
\theta_k \in [ 0,1 ], \quad k = 1, \dots, M
$$

The ith robot assigns a probability of occupancy to the cell:

$$
P_i(\theta_k) \in [0,1], \quad i = 1,\dots,N
$$

A threshold 

$$ 
\varepsilon \in [0,0.5]
$$ 

allows classification:

**Free** if : 

$$ P_i(\theta_k) < \varepsilon $$

**Occupied** if :

$$ P_i(\theta_k) > 1-\varepsilon $$  

**Uncertain** otherwise  

If no prior knowledge is available:

$$
P_0(\theta_k) = \tfrac{1}{2}
$$



### Fusion with Kullback–Leibler Average (KLA)

We want to fuse the N probability distributions 

$$  p_1(x), \dots, p_N(x) $$   

The **weighted Kullback–Leibler average** is defined as:

$$ 
\bar{p} = \underset{p}{argmin}
\sum_{i=1}^N \pi_i \int p(x) \log \frac{p(x)}{p_i(x)} \, dx
$$ 

where the weights 

$$ \pi_i \ge 0 $$ 

and


$$ \sum_{i} \pi_i = 1 $$ 


This has a closed form solution:

$$ 
\bar{p}(x) = \frac{\prod_{i=1}^N \, p_i(x)^{\pi_i}}
{\int \prod_{i=1}^N \, p_i(x)^{\pi_i} \, dx}
$$ 


### Discrete Case (Occupancy Probabilities)

For a given cell the fusion becomes:

$$
\bar{p}(\theta_j) = 
\frac{\prod_{i=1}^N \, p_i(\theta_j)^{\pi_i}}
{\prod_{i=1}^N \, p_i(\theta_j)^{\pi_i}
+ \prod_{i=1}^N (1 - p_i(\theta_j))^{\pi_i}}
$$

If uniform weights are chosen 
$$ \pi_i = \tfrac{1}{N} $$
, each robot contributes equally.

