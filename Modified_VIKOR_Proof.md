This section provides the mathematical proof for the modified VIKOR methodology of Multi-Criteria Decision-Making.
In the paper, instead of using the standard VIKOR equations for calculating the VIKOR index, we have modified the S and R values with the Di+ and Di- values calculated in the TOPSIS phase.
\textbf{Standard VIKOR:} \\

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
* $$v_j^+ = \text{ideal (best) value for criterion } j$$
* $$v_j^- = \text{negative ideal (worst) value for criterion } j$$
* $$w_j = \text{criterion weight}$$


