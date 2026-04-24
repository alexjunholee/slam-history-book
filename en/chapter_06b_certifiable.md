# Ch.6b — Certifiable SLAM: Past the Local Minimum

The lineage Ch.6 recorded, from Lu-Milios to g2o and GTSAM, left one thing unresolved. Pose graph optimization is non-convex. The solutions Gauss-Newton and LM return may be local minima. Practitioners lived with a folk observation: "with an odometry initial guess, it usually solves fine" — but in some deployments the backend converged at the wrong spot and no alarm went off. In 2015 Luca Carlone at MIT began to replace that folklore with mathematics. From Carlone's Lagrangian duality attempt to Rosen's SE-Sync in 2019, then Briales-Gonzalez-Jimenez's Cartan-Sync, Yang-Carlone's TEASER, and Papalia's CORA, this lineage rewrites the SLAM backend from "a non-convex optimization that empirically solves well" to "a convex surrogate whose global optimality is provably certifiable." The tools all came from outside SLAM: Shor relaxation from operations research, Burer-Monteiro factorization from mathematical optimization, Riemannian optimization from differential geometry, Kirchhoff's Matrix-Tree from graph theory. The names of the people who gathered these onto one table over ten years are the body of this chapter.

---

## 6b.1 The Old Anxiety of Local Minima

Ch.6 §6.7 flagged initial-value dependency as the first problem of the graph SLAM backend. The cost function is non-convex on rotation variables $\boldsymbol{R}_i \in \mathrm{SO}(3)$, so when the initial estimate is far from the truth, Gauss-Newton gets pulled into the wrong basin. The parking garage example in Handbook §6.1 shows the symptom starkly: four random initializations, one sticks to the global minimum SE-Sync reaches, the other three settle at twisted local minima with the garage floor visibly folded.

Through the late 2000s the community's response ran along two lines: trust odometry for initial-value quality, or do loop closure verification and outlier removal thoroughly at the front end. Both worked, but neither was a tool for judging whether the converged value was the true minimum. The issue Huang and Dissanayake pointed out around 2010 was simple. No matter how good the initial guess, if the data itself is ambiguous the optimizer can stop at a wrong answer. That PGO is NP-hard was formalized around the same time. And yet in the field g2o usually solved fine. That gap — theory speaking to the worst case while practice looks at the average — is where the backend theorists of the mid-2010s dug in. Every point where Gauss-Newton halts is *locally* optimal. The gradient is zero and the Hessian positive definite. The answers, however, are completely different. The moment the backend signals "converged" is also the moment when failure is least visible.

> 🔗 **Borrowed.** Ch.6's robust kernels (Huber, Cauchy) and this chapter's GNC share a root in [Black & Rangarajan (1996)](https://cs.brown.edu/people/mjblack/Papers/ijcv1996.pdf)'s robust statistics and duality theorem. One branch changed cost weights to reduce outlier influence; the other diverted the same principle to avoid non-convexity.

---

## 6b.2 Shor Relaxation — A Weapon From Outside

PGO's non-convexity comes from the rotation constraint $\boldsymbol{R}_i \in \mathrm{SO}(d)$. That constraint can in fact be written as quadratic equations — orthogonality $\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}$ and $\det(\boldsymbol{R}) = +1$. The objective is quadratic too. PGO falls exactly onto a **QCQP** (Quadratically Constrained Quadratic Program). And QCQP had a convex relaxation tool vetted in operations research since 1987: [Naum Shor's 1987 relaxation](https://link.springer.com/article/10.1007/BF01582220).

Using the identity $\boldsymbol{x}^\top \boldsymbol{M}\boldsymbol{x} = \mathrm{tr}(\boldsymbol{M}\boldsymbol{x}\boldsymbol{x}^\top)$, Shor lifts the lifting variable $\boldsymbol{X} \triangleq \boldsymbol{x}\boldsymbol{x}^\top$ so the original QCQP becomes a linear-objective problem under "$\boldsymbol{X} \succeq 0$ and rank-1," then drops the rank-1 constraint to leave a convex **semidefinite program (SDP)**. Search space grows from $n$ to $n(n+1)/2$ in exchange for convexity.

$$d^* = \min_{\boldsymbol{X}\in\mathbb{S}^n} \mathrm{tr}(\boldsymbol{C}\boldsymbol{X}) \;\; \text{s.t.} \;\; \mathrm{tr}(\boldsymbol{A}_i\boldsymbol{X})=b_i,\; \boldsymbol{X}\succeq 0.$$

The usefulness sits in the duality inequality $d^* \le p^*$. The SDP minimum is a lower bound on the original QCQP minimum. Given any candidate $\hat{\boldsymbol{x}}$, the quantity $f(\hat{\boldsymbol{x}}) - d^*$ upper-bounds how far that candidate sits from optimality. This is where the name "certifiable" comes from. Even without globally solving the original problem, one can bound how bad the solution is. If the SDP solution $\boldsymbol{X}^*$ happens to be rank-1, then $\boldsymbol{X}^* = \boldsymbol{x}^*\boldsymbol{x}^{*\top}$ and $\boldsymbol{x}^*$ is the global minimum of the original QCQP. How often this "favorable situation" occurs in SLAM becomes the subject of the papers that follow.

Carlone's two papers at IROS and ICRA in 2015 ([Carlone et al. 2015 "Lagrangian duality in 3D SLAM"](https://arxiv.org/abs/1506.00746) and [Carlone & Dellaert 2015 "Planar pose graph optimization"](https://doi.org/10.1109/ICRA.2015.7139264)) are the starting point. They showed empirically that the duality gap is mostly zero in 2D PGO and suggested extension to 3D. Carlone had just finished his 2014 TRO survey of g2o/GTSAM initialization techniques and had seen how often, when odometry conflicted with loop closures, the optimizer halted at the wrong point. The 2015 paper reported "the empirical fact that the duality gap is typically zero" without giving a closed condition for when it holds.

In the same period [Briales & Gonzalez-Jimenez (2017)](https://arxiv.org/abs/1702.03235)'s Cartan-Sync pushed the program through SO(3) synchronization. On the math side Boumal-Absil-Sepulchre were refining Riemannian optimization, and Burer-Monteiro's low-rank SDP factorization had been around since 2003. The scattered materials got assembled in one paper in 2019.

---

## 6b.3 SE-Sync — What Rosen 2019 Assembled

[Rosen, Carlone, Bandeira, Leonard's SE-Sync (IJRR 2019)](https://arxiv.org/abs/1612.07386) is the canon of certifiable SLAM. Rosen finished his doctorate with John Leonard at MIT, and Leonard was the MIT researcher who, with Ch.4's Durrant-Whyte, helped settle the name "SLAM" in the early 1990s. Co-author Afonso Bandeira, a math-side expert on SDP and synchronization, took the theoretical section proving the globality of rank-deficient second-order critical points. The four authors' differing backgrounds — robotics, SLAM, mathematical optimization, applied mathematics — tell you what the paper assembled. What this paper did was not invention but assembly: Shor relaxation, translation elimination, Burer-Monteiro low-rank parameterization, Boumal's Riemannian staircase, ingredients aged ten-odd years each in different lineages, meshed together on the single problem of PGO.

The assembly runs in three steps. First, from the observation that translation becomes linear least squares once rotation is fixed, $\boldsymbol{t}$ is eliminated in closed form (Problem 6.2). Carlone had pointed this out in his 2014 TRO survey, and Rosen slotted it as the first step of the convex relaxation. Second, apply Shor relaxation to the remaining rotation-only problem $\min_{\boldsymbol{R}\in\mathrm{SO}(d)^n} \mathrm{tr}(\tilde{\boldsymbol{Q}}\boldsymbol{R}^\top\boldsymbol{R})$ and lift to an SDP (Problem 6.3). Third, the $dn \times dn$-dimensional SDP would crush interior-point methods at a few thousand poses, so reparameterize with Burer-Monteiro $\boldsymbol{Z} = \boldsymbol{Y}^\top \boldsymbol{Y}$ and turn it into a low-dimensional unconstrained problem on the Stiefel manifold (Problem 6.4).

Two theorems justify the assembly. Theorem 6.1 is **exact recovery**: if measurement noise is below some constant $\beta$, the SDP relaxation's unique solution carries the global minimum of the original MLE as a rank-1 object. It was the first quantitative answer to "how much noise can we tolerate." The caveat: $\beta$ depends on the ground-truth matrix, so it is unknown before seeing the instance. Theorem 6.2 brings over a result of Boumal et al., guaranteeing that a second-order critical point on the Stiefel manifold, if rank-deficient, is the global minimum. These enable the Riemannian Staircase: start at small rank, find a second-order critical point, check rank-deficiency; if it fails, raise rank by one. Once rank hits $dn + 1$, every $\boldsymbol{Y}$ becomes row rank-deficient, so the process must halt in finite steps. On practical datasets, one step usually suffices.

On standard benchmarks like sphere, torus, and garage, SE-Sync converged at g2o/GTSAM speed while returning an a posteriori certificate. g2o and GTSAM were fast but silent on when to trust the answer; Rosen's algorithm spits out one more number at the end — a suboptimality bound. When it is zero, the solution is provably global-optimal. Twenty years after Lu-Milios, backend researchers could mark yes or no on "is this solution actually the minimum."

> 📜 **Prediction vs. outcome.** In §8.2 of the IJRR 2019 paper, Rosen wrote that "the algebraic simplification we have shown could be extended to anisotropic noise, outliers, and a variety of sensor modalities." The prediction partially held. Holmes-Barfoot's 2023 landmark-SLAM extension, Papalia's 2024 CORA range-measurement extension, and Yang-Carlone's TEASER line actually followed. But the most ambitious extension — "SE-Sync covers the perspective projection of visual SLAM" — did not arrive even in 2026. A structural barrier surfaced: projection is a rational function and does not fold easily into polynomial optimization. `[diverted]`

> 🔗 **Borrowed.** The Burer-Monteiro factorization at the heart of SE-Sync is the low-rank SDP method of [Burer & Monteiro (2003)](https://link.springer.com/article/10.1007/s10107-002-0352-8), later sharpened by [Boumal-Voroninski-Bandeira (2016)](https://arxiv.org/abs/1605.08101)'s Riemannian proof of the globality of second-order critical points, which Rosen brought into SLAM. From pure math to the robot backend, sixteen years.

---

## 6b.4 The Unexpected Equivalence of Graph Laplacian and Fisher Information

§6.2 asks a different question. Once the global minimum is found, how close is that estimate to the truth? The answer is the Cramér-Rao Lower Bound and the Fisher Information Matrix. In a simplified PGO model with rotations fixed, Rosen-Khosoussi-Barfoot showed, strikingly, that FIM falls out exactly as a Kronecker product of the graph's weighted reduced Laplacian.

$$\mathcal{I} = \boldsymbol{J}^\top \boldsymbol{\Sigma}^{-1} \boldsymbol{J} = \boldsymbol{L}_w \otimes \boldsymbol{I}_3.$$

Knowing the graph structure alone yields an approximation of estimation accuracy without actual measurements. By Kirchhoff's Matrix-Tree Theorem the determinant of the reduced Laplacian equals the number of weighted spanning trees, which corresponds to D-optimality (determinant of the information matrix). Algebraic connectivity (Fiedler value) corresponds to E-optimality (worst-case variance). A theorem Kirchhoff proved in 1847 for electrical circuits had become, 180 years later, the theoretical basis for measurement selection and active SLAM. In active SLAM "maximize FIM" translates into spectral manipulation of the Laplacian.

[The post-2014 work of Kasra Khosoussi and Timothy Barfoot](https://arxiv.org/abs/1709.08601) established this connection. Khosoussi did his doctorate in Sydney under Dissanayake and Huang, then went through MIT and Toronto. In the form generalized to 3D PGO, the Kronecker combination of the Laplacian and SE(3) adjoint representation appears, letting topological and geometric information be handled separately. That the "measurement selection criterion" can be approximated by a Laplacian six times smaller than the full FIM provides the mathematical basis for "loop closure selection," which Ch.6 left in place without developing.

The EKF-SLAM consistency problem Ch.4 §4.8 flagged touches this result too. The over-confidence phenomenon Julier-Uhlmann pointed out in 2001, re-read through CRLB, says the linearization approximation over-estimates Fisher information. That is why Handbook §6.2 places the FIM chapter next to convex relaxation. Finding the global minimum and knowing how accurate that minimum is have to be treated as a pair.

> 🔗 **Borrowed.** [Kirchhoff's Matrix-Tree Theorem (1847)](https://en.wikipedia.org/wiki/Kirchhoff%27s_theorem), born as an analysis tool for electrical circuits, was transplanted through combinatorics into measurement-design literature and, in the 2010s through Khosoussi, became the language of SLAM active perception. A 180-year migration path.

---

## 6b.5 Extensions and Limits — TEASER, CORA, and Lasserre's Wall

After SE-Sync the front widened in two directions. First, certifiable estimators robust to outliers. Second, extended measurement models such as range, landmark, and anisotropic noise.

The outlier side pressed first. As Ch.6 §6.7 noted, real-world pose graphs have mismatches when loop closure verification is imperfect, and even with Huber or Cauchy kernels, above a certain outlier ratio the optimization collapses. By around 2017 the pressure on the certifiable lineage to say something about outliers was clear.

The representative is [Yang, Shi, Carlone's TEASER (TRO 2020)](https://arxiv.org/abs/2001.07715), which finds the global optimum in 3D point cloud registration with up to 99% outliers. It solves truncated least squares inside a GNC wrapper with SDP relaxation on the rotation subproblem, returning a certificate alongside. The trick was splitting scale, translation, and rotation into separate certifiable subproblems, each passed along with a global-optimality guarantee. The follow-up [Yang & Carlone (2022)](https://arxiv.org/abs/2109.03349) generalized this through Lasserre moment relaxation as "certifiably robust estimation."

Range-aided SLAM is [Papalia et al. CORA (2024)](https://arxiv.org/abs/2403.09295)'s position. The range measurement $(\|\boldsymbol{t}_j - \boldsymbol{t}_i\| - \tilde r_{ij})^2$, used as is, becomes quartic and escapes QCQP, so Papalia introduced an auxiliary unit vector $\boldsymbol{b}_{ij} \in S^{d-1}$ and through bearing lifting put it back inside QCQP. CORA showed that while relaxation is tight in the single-robot case, in multi-robot settings it is generally not exact, narrowing "when does Shor work."

On the landmark side, [Holmes & Barfoot (2023)](https://arxiv.org/abs/2308.05631) used Schur complement to eliminate landmarks in advance, leaving a PGO SE-Sync takes directly. That Holmes, Khosoussi, and Rosen co-authored Ch.6 of the Handbook is evidence this lineage gathered at one table in 2025.

Walls also appeared. Generalizing anisotropic noise and truncated-quadratic outliers to POP (Polynomial Optimization Problem) calls for Lasserre's moment relaxation, but the derived SDP is **degenerate**: constraint qualification fails and the Riemannian Staircase's convergence breaks. Yang's 2022 sparse monomial basis offers a workaround, but the specialized solver is still slower than a general local solver. An algorithm holding both speed and provability has yet to appear. Visual SLAM and VIO face a deeper wall — the structural incompatibility of perspective projection and IMU preintegration — treated in the 🧭 section.

> 📜 **Prediction vs. outcome.** At ICRA 2015 Carlone wrote that "theoretical explanation for why most instances have a tight Lagrangian dual is needed." Ten years on, only part of the answer has arrived. The exact recovery theorem of Rosen-Carlone-Bandeira-Leonard gave a sufficient condition "noise below $\beta$," but no way to compute $\beta$ in advance on an actual SLAM instance. An **a priori** condition for when tightness breaks is, as of 2026, still replaced by per-instance certificates. `[in progress]`

---

## 🧭 Still open

**The boundary where tightness breaks.** SE-Sync's exact recovery theorem offered the sufficient condition "noise below $\beta$," but there is no way to compute $\beta$ on an actual instance. Judging in advance when tightness holds is what would advance algorithm design. Systematic study of how relaxation fails under heavy outliers or extremely sparse graphs is still early.

**Integration with visual SLAM and VIO.** Perspective projection $\pi(\boldsymbol{X}) = [X/Z, Y/Z]$ is rational, not polynomial. Multiplying through the denominator adds a new variable and auxiliary constraint per feature, and ORB-SLAM3's thousands of map points push the SDP outside real-time size. Forster's 2015 IMU preintegration tangles the exponential map with bias drift, resisting POP incorporation. As of 2026, Ch.7, Ch.8, and Ch.13's visual/VIO mainstream sit outside certifiable guarantees. The largest unsolved position for the lineage.

**Online certification and scale.** SE-Sync is batch. Incremental certifiable SLAM, re-solving the SDP at each new measurement, is not mature. The incrementalization that iSAM2 solved for batch SAM has to be solved again on the certifiable side. Warm-starting, incremental rank increases, composing partial certificates — all open, and city-scale moment-relaxation solver speed is still a problem.

**Outlier-majority.** Current certifiable robust estimators rest on the "minority outlier" assumption. When the majority is contaminated, multi-hypothesis certification such as list-decodable regression is needed, and that direction has just begun on the statistics side. Follow-up work by Cheng-Shi-Carlone around 2024 tried to extend in this direction, but no standard tool has settled in the way TEASER did.

---

This whole chapter is a footnote to the one-line "local-minimum convergence" sentence Ch.6 passed through (§6.7) — that folk observation replaced by a ten-year theoretical program. The fact that Ch.6 of *The SLAM Handbook*, co-authored by Carlone-Khosoussi-Rosen-Holmes-Barfoot-Dissanayake, treats this subject across 34 pages is itself evidence of the lineage's present weight. Over the same ten years, Ch.12, Ch.13, and Ch.16's learning-based SLAM moved along a different path: one side toward proving the globality of the solution, the other toward having a neural network predict it. Whether the two lineages will meet has no answer even in 2026. In Ch.19 the 🧭 items of this chapter are harvested as "gaps in backend theory."

The main line resumes in Ch.7, where the backend is taken as settled and the question shifts to what runs on top of it.
