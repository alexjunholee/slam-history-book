# Ch.6b — Certifiable SLAM: Past the Local Minimum

The lineage from Lu-Milios to g2o and GTSAM left one issue unresolved: pose graph optimization is non-convex, so Gauss-Newton and LM may return local minima. Practitioners relied on a rule of thumb, "with an odometry initial guess, it usually solves fine," yet some deployed backends converged to the wrong solution without raising an alarm. In 2015 Luca Carlone at MIT began replacing that rule of thumb with mathematics. Carlone's work on Lagrangian duality led to Rosen's SE-Sync in 2019, Briales-Gonzalez-Jimenez's Cartan-Sync, Yang-Carlone's TEASER, and Papalia's CORA. Together, this lineage moved the backend from non-convex optimization that usually worked to convex surrogates with certificates of global optimality. The tools came from outside SLAM: Shor relaxation from operations research, Burer-Monteiro factorization from mathematical optimization, Riemannian optimization from differential geometry, and Kirchhoff's Matrix-Tree Theorem from graph theory.

---

## 6b.1 The Old Anxiety of Local Minima

Ch.6 §6.7 identified dependence on initial values as the first problem of a graph SLAM backend. The cost function is non-convex in rotation variables $\boldsymbol{R}_i \in \mathrm{SO}(3)$, so Gauss-Newton can enter the wrong basin when the initial estimate is far from the truth. The parking-garage example in Handbook §6.1 shows the symptom clearly: of four random initializations, one converges to the same global minimum as SE-Sync, while the other three settle at twisted local minima with the garage floor visibly folded.

Through the late 2000s, the community followed two responses: trust odometry to provide a good initial value, or enforce loop-closure verification and outlier removal at the front end. Both worked, but neither determined whether the converged value was the true minimum. Huang and Dissanayake pointed to a simple issue around 2010: even with a good initial guess, ambiguous data can stop the optimizer at the wrong answer. PGO was also formalized as NP-hard around the same time, although g2o usually worked in the field. Backend theorists of the mid-2010s focused on this gap between worst-case theory and average practice. Convergence of Gauss-Newton does not imply global optimality. Under ideal second-order conditions a local minimum has zero gradient and a positive-semidefinite Hessian, but a numerical solver may also stop earlier because of tolerances or conditioning. Failure is least visible when the backend reports convergence.

> 🔗 **Borrowed.** Ch.6's robust kernels (Huber, Cauchy) and this chapter's GNC share a root in the robust statistics and duality theorem of [Black & Rangarajan (1996)](https://cs.brown.edu/people/mjblack/Papers/ijcv1996.pdf). One branch changed cost weights to reduce outlier influence; the other used the same principle to navigate non-convexity.

---

## 6b.2 Shor Relaxation — A Tool From Operations Research

PGO's non-convexity comes from the rotation constraint $\boldsymbol{R}_i \in \mathrm{SO}(d)$. Orthogonality, $\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}$, is quadratic. In three dimensions, $\det(\boldsymbol{R})=+1$ is cubic as written, but right-handedness can instead be expressed with quadratic cross-product relations among the columns. Together with the quadratic objective, these constraints give a **QCQP** (Quadratically Constrained Quadratic Program). Operations research had used a convex relaxation for QCQP since 1987: [Naum Shor's relaxation](https://link.springer.com/article/10.1007/BF01582220).

Using the identity $\boldsymbol{x}^\top \boldsymbol{M}\boldsymbol{x} = \mathrm{tr}(\boldsymbol{M}\boldsymbol{x}\boldsymbol{x}^\top)$, Shor introduces the lifted variable $\boldsymbol{X} \triangleq \boldsymbol{x}\boldsymbol{x}^\top$. The original QCQP then has a linear objective subject to $\boldsymbol{X} \succeq 0$ and rank 1. Dropping the rank-1 constraint leaves a convex **semidefinite program (SDP)**. The search space grows from $n$ to $n(n+1)/2$ in exchange for convexity.

$$d^* = \min_{\boldsymbol{X}\in\mathbb{S}^n} \mathrm{tr}(\boldsymbol{C}\boldsymbol{X}) \;\; \text{s.t.} \;\; \mathrm{tr}(\boldsymbol{A}_i\boldsymbol{X})=b_i,\; \boldsymbol{X}\succeq 0.$$

The duality inequality $d^* \le p^*$ makes the relaxation useful. The SDP minimum is a lower bound on the original QCQP minimum. For any candidate $\hat{\boldsymbol{x}}$, the quantity $f(\hat{\boldsymbol{x}}) - d^*$ upper-bounds its suboptimality. This is the source of the term "certifiable": even without solving the original problem globally, one can bound the solution's error from optimality. If the SDP solution $\boldsymbol{X}^*$ has rank 1, then $\boldsymbol{X}^* = \boldsymbol{x}^*\boldsymbol{x}^{*\top}$ and $\boldsymbol{x}^*$ is the global minimum of the original QCQP. The papers that follow ask how often this occurs in SLAM.

Carlone's two papers at IROS and ICRA in 2015 ([Carlone et al. 2015 "Lagrangian duality in 3D SLAM"](https://arxiv.org/abs/1506.00746) and [Carlone & Dellaert 2015 "Planar pose graph optimization"](https://doi.org/10.1109/ICRA.2015.7139264)) are the starting point. They showed empirically that the duality gap is mostly zero in 2D PGO and suggested extension to 3D. Carlone had just finished his 2014 TRO survey of g2o/GTSAM initialization techniques and had seen how often, when odometry conflicted with loop closures, the optimizer halted at the wrong point. The 2015 paper reported "the empirical fact that the duality gap is typically zero" without giving a closed condition for when it holds.

In the same period, [Briales & Gonzalez-Jimenez (2017)](https://arxiv.org/abs/1702.03235)'s Cartan-Sync extended the program to SO(3) synchronization. On the mathematical side, Boumal-Absil-Sepulchre were refining Riemannian optimization, and Burer-Monteiro's low-rank SDP factorization had existed since 2003. Rosen and colleagues assembled these materials in one paper in 2019.

---

## 6b.3 SE-Sync — What Rosen 2019 Assembled

[Rosen, Carlone, Bandeira, Leonard's SE-Sync (IJRR 2019)](https://arxiv.org/abs/1612.07386) became the standard reference for certifiable SLAM. Rosen completed his doctorate with John Leonard at MIT; Leonard, together with Ch.4's Durrant-Whyte, had helped settle the name "SLAM" in the early 1990s. Afonso Bandeira, an expert in SDP and synchronization, contributed the theoretical proof for the globality of rank-deficient second-order critical points. The paper combined backgrounds in robotics, SLAM, mathematical optimization, and applied mathematics. It assembled Shor relaxation, translation elimination, Burer-Monteiro low-rank parameterization, and Boumal's Riemannian staircase around the single problem of PGO.

The method proceeds in three steps. First, because translation becomes linear least squares once rotation is fixed, $\boldsymbol{t}$ is eliminated in closed form (Problem 6.2). Carlone had noted this in his 2014 TRO survey, and Rosen made it the first step of the convex relaxation. Second, Shor relaxation lifts the remaining rotation-only problem $\min_{\boldsymbol{R}\in\mathrm{SO}(d)^n} \mathrm{tr}(\tilde{\boldsymbol{Q}}\boldsymbol{R}^\top\boldsymbol{R})$ to an SDP (Problem 6.3). Third, because the $dn \times dn$-dimensional SDP would overwhelm interior-point methods at a few thousand poses, Burer-Monteiro reparameterization $\boldsymbol{Z} = \boldsymbol{Y}^\top \boldsymbol{Y}$ turns it into a low-dimensional unconstrained problem on the Stiefel manifold (Problem 6.4).

Two theorems justify the method. Theorem 6.1 gives **exact recovery**: if measurement noise is below a constant $\beta$, the SDP relaxation has a unique solution $\boldsymbol{Z}^*$ of rank $d$ that recovers the global minimum of the original MLE. Unlike the rank-1 condition for a generic vector QCQP, SE-Sync uses $\boldsymbol{Z}=\boldsymbol{R}^\top\boldsymbol{R}$ with $\boldsymbol{R}\in\mathbb{R}^{d\times dn}$, so an exact solution has rank $d$. The caveat is that $\beta$ depends on the ground-truth matrix and is therefore unknown before the instance is solved. Theorem 6.2 applies a result of Boumal et al.: a rank-deficient second-order critical point on the Stiefel manifold is the global minimum. Together, the theorems enable the Riemannian Staircase. It starts at low rank, finds a second-order critical point, checks for rank deficiency, and increases the rank by one if the check fails. Once the rank reaches $dn + 1$, every $\boldsymbol{Y}$ is row-rank-deficient, so the process halts in finite steps. On practical datasets, one step usually suffices.

On standard benchmarks such as sphere, torus, and garage, SE-Sync converged at g2o/GTSAM speed while returning an a posteriori certificate. g2o and GTSAM were fast but silent on when to trust the answer; Rosen's algorithm returns one additional number, a suboptimality bound. When that bound is zero, the solution is provably globally optimal. Twenty years after Lu-Milios, backend researchers had an explicit answer to whether a solution was the minimum.

> 📜 **Prediction vs. outcome.** In §8.2 of the IJRR 2019 paper, Rosen wrote that "the algebraic simplification we have shown could be extended to anisotropic noise, outliers, and a variety of sensor modalities." The prediction partially held. Holmes-Barfoot's 2023 landmark-SLAM extension, Papalia's 2024 CORA range-measurement extension, and Yang-Carlone's TEASER line followed. The most ambitious extension, applying SE-Sync to the perspective projection of visual SLAM, had not arrived by 2026. Projection is a rational function and does not fold easily into polynomial optimization.

> 🔗 **Borrowed.** The Burer-Monteiro factorization at the heart of SE-Sync is the low-rank SDP method of [Burer & Monteiro (2003)](https://link.springer.com/article/10.1007/s10107-002-0352-8), later sharpened by [Boumal-Voroninski-Bandeira (2016)](https://arxiv.org/abs/1605.08101)'s Riemannian proof of the globality of second-order critical points, which Rosen brought into SLAM. From pure math to the robot backend, sixteen years.

---

## 6b.4 The Unexpected Equivalence of Graph Laplacian and Fisher Information

§6.2 asks a different question: once the global minimum is found, how close is the estimate to the truth? The Cramér-Rao Lower Bound and Fisher Information Matrix provide the answer. In a simplified PGO model with fixed rotations, Rosen-Khosoussi-Barfoot showed that FIM is exactly a Kronecker product of the graph's weighted reduced Laplacian.

$$\mathcal{I} = \boldsymbol{J}^\top \boldsymbol{\Sigma}^{-1} \boldsymbol{J} = \boldsymbol{L}_w \otimes \boldsymbol{I}_3.$$

The graph structure alone yields an approximation of estimation accuracy without actual measurements. By Kirchhoff's Matrix-Tree Theorem, the determinant of the reduced Laplacian equals the number of weighted spanning trees, which corresponds to D-optimality (determinant of the information matrix). Algebraic connectivity (Fiedler value) corresponds to E-optimality (worst-case variance). By the 2020s, Kirchhoff's 1847 theorem for electrical circuits had become a theoretical basis for measurement selection and active SLAM. In active SLAM, "maximize FIM" translates into spectral manipulation of the Laplacian.

[The post-2014 work of Kasra Khosoussi and Timothy Barfoot](https://arxiv.org/abs/1709.08601) established this connection. Khosoussi did his doctorate in Sydney under Dissanayake and Huang, then went through MIT and Toronto. In the form generalized to 3D PGO, the Kronecker combination of the Laplacian and SE(3) adjoint representation appears, letting topological and geometric information be handled separately. That the "measurement selection criterion" can be approximated by a Laplacian six times smaller than the full FIM provides the mathematical basis for "loop closure selection," which Ch.6 left in place without developing.

The EKF-SLAM consistency problem in Ch.4 §4.8 also connects to this result. Read through the CRLB, the overconfidence Julier-Uhlmann identified in 2001 means that linearization overestimates Fisher information. Handbook §6.2 therefore places the FIM chapter next to convex relaxation. Finding the global minimum and estimating its accuracy form a pair.

> 🔗 **Borrowed.** [Kirchhoff's Matrix-Tree Theorem (1847)](https://en.wikipedia.org/wiki/Kirchhoff%27s_theorem), born as an analysis tool for electrical circuits, was transplanted through combinatorics into measurement-design literature and, in the 2010s through Khosoussi, became the language of SLAM active perception. A 180-year migration path.

---

## 6b.5 Extensions and Limits — TEASER, CORA, and Lasserre's Wall

After SE-Sync, work expanded in two directions: certifiable estimators robust to outliers, and extended measurement models for range, landmarks, and anisotropic noise.

Outliers came first. As Ch.6 §6.7 noted, imperfect loop-closure verification introduces mismatches into real-world pose graphs, and optimization collapses above a certain outlier ratio even with Huber or Cauchy kernels. By around 2017, certifiable methods needed to address them.

[Yang, Shi, Carlone's TEASER (TRO 2020)](https://arxiv.org/abs/2001.07715) is representative, finding the global optimum in 3D point-cloud registration with up to 99% outliers. It solves truncated least squares inside a GNC wrapper, with SDP relaxation on the rotation subproblem, and returns a certificate. The method splits scale, translation, and rotation into separate certifiable subproblems while preserving a global-optimality guarantee for each. The follow-up [Yang & Carlone (2022)](https://arxiv.org/abs/2109.03349) generalized this through Lasserre moment relaxation as "certifiably robust estimation."

[Papalia et al.'s CORA (2024)](https://arxiv.org/abs/2403.09295) extended the line to range-aided SLAM. Used directly, the range measurement $(\|\boldsymbol{t}_j - \boldsymbol{t}_i\| - \tilde r_{ij})^2$ becomes quartic and is no longer a QCQP. Papalia introduced an auxiliary unit vector $\boldsymbol{b}_{ij} \in S^{d-1}$ and used bearing lifting to recast it as one. CORA showed that the relaxation is tight in the single-robot case but generally not exact in multi-robot settings, narrowing the conditions under which Shor relaxation works.

On the landmark side, [Holmes & Barfoot (2023)](https://arxiv.org/abs/2308.05631) used the Schur complement to eliminate landmarks in advance, leaving a PGO that SE-Sync can solve directly. Holmes, Khosoussi, and Rosen later co-authored Ch.6 of the Handbook in 2025.

Limits also appeared. Generalizing anisotropic noise and truncated-quadratic outliers to a POP (Polynomial Optimization Problem) calls for Lasserre's moment relaxation, but the derived SDP is **degenerate**: constraint qualification fails and the Riemannian Staircase no longer converges. Yang's 2022 sparse monomial basis offers a workaround, but its specialized solver remains slower than a general local solver. No algorithm is yet both fast and certifiable. Visual SLAM and VIO face a deeper limit, the structural incompatibility of perspective projection and IMU preintegration, treated in the 🧭 section.

> 📜 **Prediction vs. outcome.** At ICRA 2015 Carlone wrote that "theoretical explanation for why most instances have a tight Lagrangian dual is needed." Ten years later, only part of the answer had arrived. The exact-recovery theorem of Rosen-Carlone-Bandeira-Leonard gave a sufficient condition, "noise below $\beta$," but no way to compute $\beta$ in advance for an actual SLAM instance. As of 2026, no **a priori** condition predicts when tightness breaks; per-instance certificates serve in its place.

---

## 🧭 Still open

**The boundary where tightness breaks.** SE-Sync's exact-recovery theorem offered the sufficient condition "noise below $\beta$," but there is no way to compute $\beta$ on an actual instance. An a priori test for tightness would guide algorithm design. Systematic study of how relaxation fails under heavy outliers or extremely sparse graphs remains limited.

**Integration with visual SLAM and VIO.** Perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$ is rational, not polynomial. Multiplying through the denominator adds a new variable and auxiliary constraint per feature, and ORB-SLAM3's thousands of map points push the SDP beyond real-time scale. Forster's 2015 IMU preintegration tangles the exponential map with bias drift, resisting incorporation into POP. As of 2026, the visual/VIO mainstream of Ch.7, Ch.8, and Ch.13 sits outside certifiable guarantees. This is the lineage's largest unresolved gap.

**Online certification and scale.** SE-Sync is batch. Incremental certifiable SLAM, which resolves the SDP at each new measurement, is not mature. As iSAM2 did for conventional SAM, certifiable SLAM needs an incremental formulation. Warm starts, incremental rank increases, and composition of partial certificates remain open, while moment-relaxation solvers are still too slow at city scale.

**Outlier-majority.** Current certifiable robust estimators rest on the "minority outlier" assumption. When the majority is contaminated, certification needs multiple hypotheses, as in list-decodable regression, and work in statistics is only beginning. Cheng-Shi-Carlone pursued this direction around 2024, but no tool has become a standard comparable to TEASER.

---

The brief discussion of local-minimum convergence in Ch.6 §6.7 developed into a ten-year theoretical program. Ch.6 of *The SLAM Handbook*, co-authored by Carlone-Khosoussi-Rosen-Holmes-Barfoot-Dissanayake, devotes 34 pages to the subject, evidence of the lineage's current weight. During the same decade, the learning-based SLAM of Ch.12, Ch.13, and Ch.16 followed another path: one lineage tried to prove that a solution was global, while the other asked a neural network to predict it. Whether they will meet remains unanswered in 2026. Ch.19 groups this chapter's 🧭 items under "gaps in backend theory."

Ch.7 returns to the front end, treating the backend as given and asking what runs on top of it.
