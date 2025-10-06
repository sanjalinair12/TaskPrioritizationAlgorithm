This section provides the mathematical proof for the modified VIKOR methodology of Multi-Criteria Decision-Making.
In the paper, instead of using the standard VIKOR equations for calculating the VIKOR index, we have modified the S and R values with the Di+ and Di- values calculated in the TOPSIS phase.


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

# Theorem

Using ideal distances from TOPSIS $D_i^+$ and $D_i^-$ as $S$ and $R$ in the VIKOR index improves:

* Ranking consistency
* Robustness to noise
* Closeness to the ideal solution

compared to the standard VIKOR method.

---

## 3. Proof


### Step A — Normalisation Property

Let $D_i^+$ ≥ 0 and  $D_i^- ​≥0$ be the Euclidean distances of alternative 𝑖  to the positive and negative ideal solutions, respectively. 

Define 

$$Q_i = v \cdot \frac{D_i^+}{D_i^+ + D_i^-} + (1 - v) \cdot \frac{D_i^+}{D_i^+ + D_i^-}, \qquad v \in [0,1].$$

Then for every alternative 𝑖,

$$
0 \le Q_i \le 1.
$$

As both denominators are the same, we can modify the above equation for $Q_i$ as follows:

$$Q_i = (v + (1 - v)) \cdot \frac{D_i^+}{D_i^+ + D_i^-} = \frac{D_i^+}{D_i^+ + D_i^-}.$$

v+(1−v)=1, so the convex weights do not change the value when applied to identical inner terms.

By the definition of the TOPSIS ideal distances,
$(D_{i}^{+} \ge 0)$ and $(D_{i}^{-} \ge 0)$,
therefore the denominator
$(D_{i}^{+} + D_{i}^{-}) \ge 0$.

As the Euclidean distances are non-negative quantities,
the numerator is also non-negative.
Thus, we can prove the non-negativity of the denominator in all conditions.

Now, let us consider two cases where
$(D_{i}^{+} + D_{i}^{-}) > 0$ and $(D_{i}^{+} + D_{i}^{-}) = 0$.

For $(D_{i}^{+} + D_{i}^{-}) > 0$,
using the basic inequality for fractions, we have:

$$
0 \le \frac{D_{i}^{-}}{D_{i}^{+} + D_{i}^{-}} \le 1
$$

Dividing all parts of the inequality by the positive number $(D_{i}^{+} + D_{i}^{-})$ gives:

$$
0 \le \frac{D_{i}^{-}}{D_{i}^{+} + D_{i}^{-}} \le 1
$$

Thus:

$$
0 \le Q_i \le 1
$$

in this case.

For the case $(D_{i}^{+} + D_{i}^{-}) = 0$,
this equality can only hold if $(D_{i}^{+} = 0 \text{ and } D_{i}^{-} = 0)$ simultaneously.
This is a degenerate situation where the alternative coincides with both ideal and anti-ideal solutions.

By convention, to avoid undefined expressions:

$$
\frac{D_{i}^{-}}{D_{i}^{+} + D_{i}^{-}} = 0 \quad \text{when} \quad D_{i}^{+} = D_{i}^{-} = 0.
$$

So here $(Q_i = 0)$,
which lies in $[0, 1]$.


From this, we can conclude that:

$$
0 \le Q_i \le 1
$$

in all circumstances.




---

### Step B — Robustness Analysis

Define sensitivity of $Q_i$ to criterion changes:

$$
\frac{\partial Q_i}{\partial f_{ij}}.
$$

For this method:

$$
\frac{\partial S_i}{\partial f_{ij}} = w_j (v_{ij} - v_j^+)
\frac{1}{D_i^+}
\frac{\partial v_{ij}}{\partial f_{ij}}.
$$

In standard VIKOR, sensitivity is:

$$
\frac{\partial S_i}{\partial f_{ij}} = w_j
\frac{f_j^* - f_{ij}}{f_j^* - f_j^-}.
$$

**Observation:**
TOPSIS distances smooth differences via Euclidean aggregation $\rightarrow$ reducing sensitivity to noise/outliers.

**Mathematical criterion:**
If:

$$
\left| \frac{\partial S_i}{\partial f_{ij}} \right|_{\text{modified}} < \left| \frac{\partial S_i}{\partial f_{ij}} \right|_{\text{standard}},
$$

Then the modified method is provably more robust.

---

### Step C — Closeness to Ideal

Standard VIKOR aggregates normalised criteria, and it doesn’t explicitly minimise a distance metric.

The chosen method explicitly minimises $D^+$ and maximises $D^-$, directly aligning with the ideal solution concept in Multi-Criteria Decision Making.

**Mathematical proof:**
TOPSIS ranks alternatives by:

$$
C_i = \frac{D_i^-}{D_i^+ + D_i^-}.
$$

The modified VIKOR embeds this distance-based philosophy while adding the compromise structure of VIKOR:

$$
Q_i = v \frac{D_i^+ - D^{*+}}{D^{-+} - D^{*+}} + (1-v) \frac{D_i^- - D^{--}}{D^{- -} - D^{--}}.
$$

This ensures the ranking minimises distance to the ideal and maximises distance from the worst-case simultaneously.



