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

The KLA is essentially a **geometric mean of probability densities**, normalized so that the result is still a valid probability distribution. In the discrete occupancy setting, this produces a fused estimate that balances the beliefs of all robots while discounting unreliable or noisy ones.  

The exponential efficiency model captures the idea that the longer a robot explores, the less reliable its measurements become. By embedding this into the fusion formula, we prevent a single robot with a long path (and possibly degraded sensing) from dominating the global map.

### Discrete Case (Occupancy Probabilities)

For a given cell the fusion becomes:


$$ \bar{p}(\theta_j) = \frac{\prod_{i=1}^N \, p_i(\theta_j)^{\pi_i}} {\prod_{i=1}^N \, p_i(\theta_j)^{\pi_i} + \prod_{i=1}^N (1 - p_i(\theta_j))^{\pi_i}} $$


If uniform weights are chosen 

$$ \pi_i = \tfrac{1}{N} $$

each robot contributes equally so : 

$$ \bar{p}(\theta_j) = \frac{\prod_{i=1}^N \, p_i(\theta_j)^{	\tfrac{1}{N}}} {\prod_{i=1}^N \, p_i(\theta_j)^{	\tfrac{1}{N}} + \prod_{i=1}^N (1 - p_i(\theta_j))^{	\tfrac{1}{N}}} $$


### Sensor Noise and Efficiency

In more realistic settings, we model sensor measurements as noisy:

**Gaussian noise**
  
$$
\xi \sim \mathcal{N}(0, \sigma^2)
$$

**Uniform noise**

$$
\xi \sim \sigma^2 U(0,1), \quad \sigma \in \mathbb{R}
$$

To account for sensor degradation or trajectory effects, we assign an efficiency factor:

$$
\eta_i(d) = e^{-\alpha d}, \quad \alpha > 0
$$

where d is the travelled distance of robot i. This means robots that explore longer trajectories contribute less confidently to the fusion.



### Fusion with Efficiency Weights

Replacing the raw probabilities $$ p_i(\theta) $$ with efficiency-adjusted ones:

$$
p_i^\eta(\theta) = \frac{1}{2} + \left(p_i(\theta) - \tfrac{1}{2}\right) \eta_i
$$

The fusion rule becomes:

$$ \bar{p}(\theta) = \frac{\prod_{j=1}^N \, \left[ \tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]^{\pi_j}} {\prod_{j=1}^N \, \left[ \tfrac{1}{2}  (p_j(\theta) - \tfrac{1}{2})\eta_j \right]^{\pi_j} + \prod_{j=1}^N \, \left(1 - \left[\tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]\right)^{\pi_j}} $$


This formulation ensures that:
- Perfectly reliable robots ($$\eta=1$$) contribute their full measurement.  
- Unreliable robots ($$\eta \to 0$$) contribute only prior information.  ### Fusion with Efficiency Weights

Replacing the raw probabilities $$ p_i(\theta) $$ with efficiency-adjusted ones:

$$
p_i^\eta(\theta) = \frac{1}{2} + \left(p_i(\theta) - \tfrac{1}{2}\right) \eta_i
$$

The fusion rule becomes:

$$
\bar{p}(\theta) = \frac{\prod_{j=1}^N \, \left[ \tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]^{\pi_j}} {\prod_{j=1}^N \, \left[ \tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]^{\pi_j} + \prod_{j=1}^N \, \left(1 - \left[\tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]\right)^{\pi_j}} $$

This formulation ensures that:
- Perfectly reliable robots contribute their full measurement.  
- Unreliable robots contribute only prior information.  

## License and Disclaimer
This repository is distributed under the MIT License. The software is provided *“as is”*, without warranty of any kind. The author assumes no responsibility for improper use or damages.


