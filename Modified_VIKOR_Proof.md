This section provides the mathematical proof for the modified VIKOR methodology of Multi-Criteria Decision-Making.
In the paper, instead of using the standard VIKOR equations for calculating the VIKOR index, we have modified the S and R values with the $D_i^+$ and $D_i^-$ values calculated in the TOPSIS phase.


**Standard VIKOR:**

$$
S_i = \sum_{j=1}^{n} w_j
\frac{f_j^* - f_{ij}}{f_j^* - f_j^-},
\quad
R_i = \max_j \left[
w_j
\frac{f_j^* - f_{ij}}{f_j^* - f_j^-}
\right]
$$

**Modified VIKOR:**

$$
S_i = D_i^+ =
\sqrt{
\sum_{j=1}^{n}
w_j
\left( v_{ij} - v_j^+ \right)^2
},
\quad
R_i = D_i^- =
\sqrt{
\sum_{j=1}^{n}
w_j
\left( v_{ij} - v_j^- \right)^2
}
$$

Where:

* $$v_{ij} = \text{normalised value of criterion } j \text{ for alternative } i$$
* $$v_j^+ = \text{ideal (best) value for criterion } j$$ (PIS)
* $$v_j^- = \text{negative ideal (worst) value for criterion } j$$ (NIS)
* $$w_j = \text{criterion weight}$$



Using ideal distances from TOPSIS $D_i^+$ and $D_i^-$ as $S$ and $R$ in the VIKOR index improves:

* Ranking consistency
* Robustness to noise
* Closeness to the ideal solution

compared to the standard VIKOR method.

---
Our aim is to prove that our AHTOVIK approach provides better ranking consistency under criterion perturbations, exhibits robustness to noise due to attenuated sensitivity coefficients and achieves greater closeness to the positive ideal solution compared to the standard VIKOR formulation. For this purpose, the theoretical foundation for analysing the sensitivity and stability in MCDM established by Opricovic and Tzeng (2004) [1] is considered. We focus on their comparative analysis of VIKOR and TOPSIS approaches and extending these foundational results we intend to prove our claims.

Opricovic and Tzeng proved that the Euclidean distance measure $D_i^+$ in TOPSIS method is less sensitive to noise and small perturbations in the decision matrix compared to the $S_i$ in VIKOR method. This is because the $S_i$ relies on an additive aggregation. They also empirically proved that the rank order in TOPSIS remains more stable under data uncertainty. 
The formal comparison of sensitivity of $S_i$ and $D_i^+$ proved that $D_i^+$ exhibits smoother responses to small perturbations $δv_{ij}$. With these observations, we intend to show that the AHTOVIK index inherits these stability properties and thus offers better ranking consistency, noise robustness, and improved closeness to ideal solutions.

Ranking Consistency
From [1], $D_i^+$ is less sensitive to data variations than $S_i$. Thus, we can conclude the formalization of this by using the partial derivaties as follows, 

$$\frac{\partial S_i}{\partial f_{ij}} = -\frac{w_j}{\Delta_j}$$

$$\frac{\partial D_i^+}{\partial f_{ij}} = \frac{w_j(v_{ij}-1)}{D_i^+\Delta_j}$$

Hence the inequality showing the TOPSIS sensitivity is less than that of VIKOR sensitivity is as follows,

$$\Big|\frac{\partial D_i^+}{\partial f_{ij}}\Big| = \frac{w_j|1-v_{ij}|}{D_i^+\Delta_j} < \frac{w_j}{\Delta_j} = \Big|\frac{\partial S_i}{\partial f_{ij}}\Big|$$

The above equation quantifies the noise sensitivity inequality proved in [1]. Now we need to define the modified VIKOR Index $Q_{iMod}$ as follows,

$$Q_{iMod}= \alpha S_i + (1-\alpha) D_i^+, \quad 0 \leq \alpha \leq 1$$

The partial derivative of the modified  $Q_i$ ($Q_{iMod}$) with respect to a change in the criterion value $f_{ij}$ is:

$$\frac{\partial Q_{iMod}}{\partial f_{ij}} = \frac{w_j}{\Delta_j}\left[-\alpha + (1-\alpha)\frac{v_{ij}-1}{D_i^+}\right]$$

Taking the absolute value, the marginal sensitivity of $Q_{iMod}$ is shown to be less than the sensitivity of the purely linear $S_i$ (where $\alpha=1$):

$$\left|\frac{\partial Q_{iMod}}{\partial f_{ij}}\right| = \frac{w_j}{\Delta_j}\left|\alpha - (1-\alpha)\frac{|1-v_{ij}|}{D_i^+}\right| < \frac{w_j}{\Delta_j}$$


This reduction in sensitivity implies improved ranking consistency under small data perturbations ($\delta$):

$$|\delta Q_{iMod}| < |\delta S_i|$$

The above equation implies there is no rank reversal for small $\delta$ values ensuring consistent rankings under perturbations. 








