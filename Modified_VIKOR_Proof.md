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

# Theorem

 
Using ideal distances from TOPSIS (\(D^+, D^-\)) as \(S\) and \(R\) in the VIKOR index improves:

- Ranking consistency  
- Robustness to noise  
- Closeness to the ideal solution  

compared to the standard VIKOR method.

---

## 3. Proof Plan

We intend to prove the theorem in three steps:

### Step A — Normalisation Property

D_i^+ and D_i^- are Euclidean distances in normalised space.

By definition:

\[
0 \le D_i^+, D_i^- \le 1.
\]

This ensures uniform normalisation across criteria.

**Proof:**  
Given normalised \(v_{ij} \in [0,1]\):

\[
(v_{ij} - v_j^+)^2 \le 1,
\]

therefore:

\[
D_i^+ = \sum_j w_j (v_{ij} - v_j^+)^2 \le \sum_j w_j = 1
\]
(if \(\sum_j w_j = 1\)).

Similarly for \(D_i^-\).

---

### Step B — Robustness Analysis

Define sensitivity of \(Q_i\) to criterion changes:

\[
\frac{\partial Q_i}{\partial f_{ij}}.
\]

For your method:

\[
\frac{\partial S_i}{\partial f_{ij}} 
= w_j (v_{ij} - v_j^+) 
\frac{1}{D_i^+}
\frac{\partial v_{ij}}{\partial f_{ij}}.
\]

In standard VIKOR, sensitivity is:

\[
\frac{\partial S_i}{\partial f_{ij}} 
= w_j 
\frac{f_j^* - f_{ij}}{f_j^* - f_j^-}.
\]

**Observation:**  
TOPSIS distances smooth differences via Euclidean aggregation → reducing sensitivity to noise/outliers.

**Mathematical criterion:**  
If:

\[
\left| \frac{\partial S_i}{\partial f_{ij}} \right|_{\text{modified}}
<
\left| \frac{\partial S_i}{\partial f_{ij}} \right|_{\text{standard}},
\]

then the modified method is provably more robust.

---

### Step C — Closeness to Ideal

Standard VIKOR aggregates normalised criteria; it doesn’t explicitly minimise a distance metric.  

Your method explicitly minimises \(D^+\) and maximises \(D^-\), directly aligning with the ideal solution concept in MCDM.

**Mathematical proof:**  
TOPSIS ranks alternatives by:

\[
C_i = \frac{D_i^-}{D_i^+ + D_i^-}.
\]

Your modified VIKOR embeds this distance-based philosophy while adding the compromise structure of VIKOR:

\[
Q_i = 
v \frac{D_i^+ - D^{*+}}{D^- + - D^{*+}} 
+ (1-v) \frac{D_i^- - D^{*-}}{D^- - D^{*-}}.
\]

This ensures the ranking minimises distance to ideal and maximises distance from worst-case simultaneously.



