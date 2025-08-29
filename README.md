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

Each of the $$N$$ robots provides its own estimate $$P_i(\theta_k)$$.  
The question is: **how to combine these estimates into a single coherent probability**?

A naive idea is the arithmetic mean:

$$
\bar{P}(\theta_k) = \frac{1}{N}\sum_{i=1}^N P_i(\theta_k)
$$

but this does not account for different sensor reliabilities and may be misleading if one robot is very noisy.

To solve this, we use the **Kullback–Leibler Average (KLA)**.

### Fusion with Kullback–Leibler Average (KLA)

We want to fuse the $$N$$ probability distributions 

$$  p_1(x), \dots, p_N(x) $$   

The **weighted Kullback–Leibler average** is defined as:

$$ 
\bar{p} = \underset{p}{argmin}
\sum_{i=1}^N \pi_i \int p(x) \log \frac{p(x)}{p_i(x)}  dx
$$ 

The idea is to find a single distribution  that is "closest" on average to all the other distributions in the set, with the weights $$\pi_i$$ determining which distributions are more important.

where the weights 

$$ \pi_i \ge 0 $$ 

and


$$ \sum_{i} \pi_i = 1 $$ 

### Problem Solution 
The objective function to minimize is the weighted sum of KL divergences, subject to the normalization constraint $\int p(x) \, dx = 1$. We use the method of Lagrange multipliers.

The Lagrangian is defined as:

$$L(p, \lambda) = \sum_{i=1}^{N} \pi_i \int p(x) \log \frac{p(x)}{p_i(x)}  dx + \lambda \left( \int p(x)  dx - 1 \right)$$

We can rewrite the KL divergence term using properties of logarithms:

$$L(p, \lambda) = \sum_{i=1}^{N} \pi_i \int p(x) (\log p(x) - \log p_i(x))  dx + \lambda \left( \int p(x)  dx - 1 \right)$$

To find the minimum, we take the functional derivative of the Lagrangian with respect to $p(x)$ and set it to zero:

$$
\frac{\delta L}{\delta p(x)} = 0
$$

Taking the derivatives term by term:

$$
\frac{\delta}{\delta p(x)} \left( \sum_{i=1}^{N} \pi_i \int p(x) \log p(x)  dx \right) = \sum_{i=1}^{N} \pi_i (\log p(x) + 1)
$$

$$
\frac{\delta}{\delta p(x)} \left( - \sum_{i=1}^{N} \pi_i \int p(x) \log p_i(x) dx \right) = - \sum_{i=1}^{N} \pi_i \log p_i(x)
$$

$$
\frac{\delta}{\delta p(x)} \left( \lambda \int p(x) dx \right) = \lambda
$$

Summing these terms and setting the result to zero:

$$
\sum_{i=1}^{N} \pi_i (\log p(x) + 1) - \sum_{i=1}^{N} \pi_i \log p_i(x) + \lambda = 0
$$

Since $\sum_{i=1}^{N} \pi_i = 1$, the equation simplifies to:
$$
\log p(x) + 1 - \sum_{i=1}^{N} \pi_i \log p_i(x) + \lambda = 0
$$

Now, we solve for $\log p(x)$:

$$
\log p(x) = \sum_{i=1}^{N} \pi_i \log p_i(x) - (1 + \lambda)
$$

Exponentiating both sides:

$$
p(x) = e^{\sum_{i=1}^{N} \pi_i \log p_i(x)} \cdot e^{-(1 + \lambda)}
$$

Using the property of logarithms that $\sum a \log b = \log (\prod b^a)$:

$$
\sum_{i=1}^{N} \pi_i \log p_i(x) = \log \left( \prod_{i=1}^{N} p_i(x)^{\pi_i} \right)
$$

Substituting this back into the equation for $p(x)$:

$$
p(x) = e^{\log \left( \prod_{i=1}^{N} p_i(x)^{\pi_i} \right)} \cdot e^{-(1 + \lambda)}
$$

$$
p(x) = \left( \prod_{i=1}^{N} p_i(x)^{\pi_i} \right) \cdot e^{-(1 + \lambda)}
$$

We can define a normalization constant $$Z = e^{-(1 + \lambda)}$$:

$$
p(x) = Z \prod_{i=1}^{N} p_i(x)^{\pi_i}
$$

Finally, we use the constraint $\int p(x) \, dx = 1$ to find $Z$:
$$
\int Z \prod_{i=1}^{N} p_i(x)^{\pi_i} \, dx = 1
$$

$$
Z \int \prod_{i=1}^{N} p_i(x)^{\pi_i} \, dx = 1
$$

$$
Z = \frac{1}{\int \prod_{i=1}^{N} p_i(x)^{\pi_i} \, dx}
$$

Substituting $Z$ back into the expression for $p(x)$ gives the final closed-form solution:

$$
\bar{p}(x) = \frac{\prod_{i=1}^{N} p_i(x)^{\pi_i}}{\int \prod_{j=1}^{N} p_j(x)^{\pi_j} \, dx}
$$

This result shows that the KL-average distribution is the normalized weighted geometric mean of the individual distributions.


### Intuition
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

where $$d$$ is the travelled distance of robot $$i$$. This means robots that explore longer trajectories contribute less confidently to the fusion.



### Fusion with Efficiency Weights

Replacing the raw probabilities $$p_i(\theta)$$ with efficiency-adjusted ones:

$$
p_i^\eta(\theta) = \frac{1}{2} + \left(p_i(\theta) - \tfrac{1}{2}\right) \eta_i
$$

The fusion rule becomes:

$$ \bar{p}(\theta) = \frac{\prod_{j=1}^N \, \left[ \tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]^{\pi_j}} {\prod_{j=1}^N \, \left[ \tfrac{1}{2}  (p_j(\theta) - \tfrac{1}{2})\eta_j \right]^{\pi_j} + \prod_{j=1}^N \, \left(1 - \left[\tfrac{1}{2} + (p_j(\theta) - \tfrac{1}{2})\eta_j \right]\right)^{\pi_j}} $$


This formulation ensures that:
- Perfectly reliable robots ($$\eta=1$$) contribute their full measurement.  
- Unreliable robots ($$\eta \to 0$$) contribute only prior information.  

## License and Disclaimer
This repository is distributed under the MIT License. The software is provided *“as is”*, without warranty of any kind. The author assumes no responsibility for improper use or damages.


