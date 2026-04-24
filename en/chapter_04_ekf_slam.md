# Ch.4 — Smith-Cheeseman and the Rise and Fall of EKF-SLAM

The photogrammetry, SfM, and bundle adjustment covered in Part I all assumed one thing. The camera sits still, or there is enough time after capture to crunch all the images offline as a batch. Hartley-Zisserman's geometry, RANSAC's robust estimation, Levenberg-Marquardt's iterative optimization — these tools knew how to measure the world, but did not ask where a moving robot was *right now*. Part II starts from that question. Building a map while knowing your own position, refusing to give up on the estimate while uncertainty piles up. The problem of probabilistic mapping opened in a small memo out of SRI International.

In 1986 Randall Smith and Peter Cheeseman wanted to treat mathematically how uncertain a robot's measurement is when it measures something in space. The idea that came out of SRI International inherited Kalman's (1960) filter mathematics but extended it in a different direction, propagating uncertainty not over a single state estimate but over a whole *network of spatial relationships*. Several years later Hugh Durrant-Whyte in Sydney and John Leonard at MIT welded onto that mathematics the problem statement "a robot estimates its own position while building a map." The acronym "SLAM" is the product of that join.

---

## 4.1 The mathematics of uncertain spatial relationships — Smith, Self, Cheeseman (1988)

In 1986 Randall Smith and Peter Cheeseman at SRI International wanted to capture in equations how error propagates when a robot accumulates measurements across several places. Their working notes came out in 1988 as ["Estimating Uncertain Spatial Relationships in Robotics"](https://arxiv.org/abs/1304.3111). The question itself was clear. When a robot measures B from A and then C from B, how does the uncertainty from A to C get computed?

The [Kalman filter](https://www.cs.unc.edu/~welch/kalman/kalmanPaper.html) already existed. It had been used since 1960 for radar tracking, ballistic calculation, and satellite orbit correction. What Smith and Cheeseman did was reformulate Kalman's covariance propagation equations to fit the composition of spatial transforms. Put the robot pose $\mathbf{x}_r$ and the landmark positions $\mathbf{m}_i$ into a single state vector, and maintain the joint covariance $\mathbf{P}$ over all of it.

$$\mathbf{x} = [\mathbf{x}_r^\top,\ \mathbf{m}_1^\top,\ \ldots,\ \mathbf{m}_N^\top]^\top$$

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rm} \\ \mathbf{P}_{mr} & \mathbf{P}_{mm} \end{bmatrix}$$

The off-diagonal block $\mathbf{P}_{rm}$ was the point. Robot-pose uncertainty and landmark-position uncertainty are *correlated*, and only by tracking that correlation can the estimate stay consistent. The paper proved this explicitly, and the whole SLAM field took its starting position from there.

> 🔗 **Borrowed.** Smith-Cheeseman's (1988) spatial-relationship mathematics inherits directly from Kalman's (1960) covariance propagation. A technique for tracking a single moving object became a framework for tracking a robot and every element of its map at once.

---

## 4.2 How the name "SLAM" settled in

There is no "SLAM" in the 1988 Smith-Cheeseman paper. Hugh Durrant-Whyte, who had moved from Oxford to Sydney, and John Leonard at MIT were, in the early 1990s, each calling the same problem a different name in their own labs. Once the two groups started citing each other, a shared term was needed, and "SLAM" converged into place. Researchers' memories differ on which document used it first. No canonical first-use paper exists.

Leonard and Durrant-Whyte's 1991 paper, ["Simultaneous Map Building and Localization for an Autonomous Mobile Robot"](https://doi.org/10.1109/IROS.1991.174711), is often cited as an early instance that put this problem front and center in a mainstream robotics title. That mapping and localization are inseparably entangled, and must be done simultaneously — that intuition sat there before the acronym did.

"Simultaneous Localization and Mapping," abbreviated SLAM. For the next ten years this name was the center of gravity the field converged around.

> 🔗 **Borrowed.** [Bar-Shalom's multi-target tracking](https://archive.org/details/trackingdataasso0000bars) (multi-target tracking, collected as a 1988 monograph) supplied a framework for estimating the states of many objects at once. Leonard and Durrant-Whyte can be read as mapping "target position" to "landmark position" and "tracker position" to "robot pose" inside that framework. A case of radar technology translated into indoor robot mapping.

---

## 4.3 The EKF-SLAM formulation

Applying the Extended Kalman Filter (EKF) to SLAM was less a choice than a natural convergence. The EKF had been used for nonlinear system estimation since before 1988 and runs in two stages — predict and update.

Predict stage: when the robot moves, the motion model $f(\cdot)$ predicts the state, and the Jacobian $\mathbf{F}$ propagates the covariance.

$$\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, \mathbf{u})$$
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$$

Update stage: when a sensor measurement $\mathbf{z}$ arrives, the Jacobian $\mathbf{H}$ of the observation model $h(\cdot)$ yields a Kalman gain $\mathbf{K}$ that updates the state and covariance.

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top(\mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

Iterating these two stages is the whole of EKF-SLAM. The structure is simple, and that simplicity carried a scalability ceiling from the start.

The problem is state dimension. Put a 6DOF pose together with $N$ 3D landmarks into the state, and the state vector has dimension $6 + 3N$; the covariance matrix is an $O(N^2)$ structure of $(6+3N)^2$ entries. A single update costs $O(N^2)$ both for the Kalman gain (inverting $\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$) and for the covariance update. 100 landmarks gives $306 \times 306 \approx$ 94k entries; 1,000 landmarks gives $3006 \times 3006 \approx$ 9M. The number of landmarks a regular PC in the early 2000s could keep in real time topped out in the tens to low hundreds.

That [Andrew Davison's MonoSLAM (2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf) was locked to a few dozen landmarks in its live demos was no accident. The $O(N^2)$ wall of EKF-SLAM set that number.

---

## 4.4 The scalability wall

When Davison ran real-time 3D tracking from a single webcam at ICCV 2003, he was mapping a desk-sized space with a few dozen features. In an environment with no commercial SLAM systems, a real-time monocular demo was a rare thing to see. The issue was that its ceiling came not from the algorithm but from the size of the covariance matrix.

At 100 landmarks the covariance matrix is $306 \times 306$ (6DOF pose + 100 3D landmarks, state dimension $6 + 3 \times 100 = 306$). At 1,000 it is $3006 \times 3006$. Every time step that matrix has to be updated along with a matrix inversion. On top of that, because the EKF keeps the full joint distribution in one block, adding a new landmark immediately generates cross-correlations with every existing landmark. As the map grows, update cost grows exponentially.

The fix attempted through the mid-2000s was submaps. Partition the whole map into overlapping small regions, run an EKF inside each submap, and connect submaps by a separate structure. [Chong and Kleeman (1999)](http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/integrated1/chong_feature_map.pdf) proposed an early form. Information loss at submap boundaries, difficulty of loop closure, and implementation complexity added friction to putting submap approaches into practice.

> 🔗 **Borrowed.** The submap-partitioning idea of Chong-Kleeman (1999) carries forward into the local-window optimization of modern SLAM. ORB-SLAM's local map and VINS-Mono's sliding window sit conceptually on the same principle. Only the implementation tool changed, from EKF to bundle adjustment.

---

## 4.5 The consistency problem: Julier-Uhlmann's counterexample

A deeper flaw in EKF-SLAM broke at ICRA 2001. Simon Julier and Jeffrey Uhlmann analyzed the behavior of EKF-based SLAM through numerical experiments and showed that the filter trusts itself too much. Their paper title at IEEE ICRA was ["A Counter Example to the Theory of Simultaneous Localization and Map Building"](https://doi.org/10.1109/ROBOT.2001.933257). Provocative, and the content matched.

The core that secondary literature summarizes when citing this paper is that EKF-SLAM is asymptotically *overconfident*. The actual estimation error grows, while the covariance (uncertainty) the filter computes converges below its true value. This is inconsistency.

The cause sits in linearization error. The EKF approximates nonlinear motion and observation models by a first-order Taylor expansion. When this approximation error accumulates step by step, the covariance begins to underestimate the real error. Once the robot becomes overconfident that "I am here," the filter trusts subsequent measurements less, and errors pile up without correction.

In 2007 [Shoudong Huang and Gamini Dissanayake](https://doi.org/10.1109/TRO.2007.903811) dissected the cause of this inconsistency more precisely. The paper's central diagnosis was that basic constraints among the Jacobians evaluated at the current state estimate break down, and this is the main cause of EKF-SLAM's inconsistency; as a result, the variance of the robot's heading angle (yaw) can wrongly converge to zero when it should be maintained. Later observability-based analyses, in which the system's observable degrees of freedom change with the point of linearization and the filter injects spurious information into unobservable directions, take this paper as their starting point.

> 📜 **Prediction vs. outcome.** After Julier and Uhlmann's 2001 counterexample, attempts to design a consistent estimator kept coming. Variants in the filter family (the Unscented Kalman Filter (UKF), Invariant EKF, robust covariance) were proposed for nearly a decade. Looking back from 2026, though, the practical resolution of this problem came *not from the filter but from optimization*. [iSAM](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf) (Kaess et al., 2008), [g2o](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf) (Kümmerle et al., 2011), and GTSAM effectively replaced the filter. Refreshing the Jacobian linearization through iterative optimization rather than freezing it at the current estimate avoids the inconsistency structurally. The seat the counterexample reserved for a "new filter" was eventually filled not by a filter but by a different structure. `[abandoned]`

---

## 4.6 FastSLAM — divide and conquer

What attacked EKF-SLAM's $O(N^2)$ wall from a different direction was [FastSLAM](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf). Michael Montemerlo, Sebastian Thrun (Stanford), Daphne Koller, and Ben Wegbreit presented it at AAAI 2002.

The key observation is Rao-Blackwellization. Given the robot path $x_{0:t}$, the position estimates of each landmark become *mutually independent*. So one can represent the path with a particle filter (each particle standing for one possible path) and run a separate landmark EKF independently for each particle.

With $K$ particles and $N$ landmarks the per-step complexity is $O(K \log N)$, sublinear in $N$ unlike EKF-SLAM's $O(N^2)$ (when using KD-tree-based landmark search). As landmark count rises, per-particle EKFs stay mutually independent, so there is no need to keep the full $N \times N$ covariance. $K$ is fixed at tens to hundreds, and the practical gain was large.

FastSLAM worked. In indoor environments it held real-time up to a few hundred landmarks, and the technology transferred quickly. But problems accumulated. Particle depletion was the first: as the map grows, most particles come to represent bad paths, and effective sample count drops sharply. Reweighting paths in a loop-closure event is hard, and adding more particles did not solve drift accumulation in large-scale environments.

[FastSLAM 2.0](https://www.ijcai.org/Proceedings/03/Papers/165.pdf) (Montemerlo et al. 2003) improved the proposal distribution, but as long as the methodology stayed inside the filter paradigm there was a ceiling to scalability. The method that ultimately got around that ceiling was not in the filter family.

---

## 4.7 The EKF's exit

Graph-based approaches became real from 2005 onward, and EKF-SLAM stepped back from the main line. Once [Feng Lu and Evangelos Milios's 1997 graph idea](https://doi.org/10.1023/A:1008854305733) combined with [Olson-Leonard-Teller's (2006)](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf) efficient solver, and then with the real-time factorization techniques of g2o, GTSAM, and iSAM2, the EKF's strength of "incremental update" was no longer a differentiator.

The difference showed up at loop closure — correcting map error when the robot returns to its starting point. The EKF has to update the entire covariance matrix at that moment. Cost: $O(N^2)$. Graph optimization adds one new edge to the pose graph and refactors a sparse matrix. Using the sparse structure, the cost is far lower.

Around 2010, choosing the EKF as a backend for a new SLAM system became uncommon. It survived only under special constraints — very limited compute resources, real-time filter requirements.

> 📜 **Prediction vs. outcome.** Durrant-Whyte and Bailey's [2006 IEEE Robotics & Automation Magazine tutorial](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf) discussed SLAM's scalability and projected submap decomposition and the information filter as the solutions for large-scale environments. The information filter (the EKF's inverse-covariance form) was expected to keep computation from slowing with landmark count, using a sparse information matrix. The actual development ran differently. The information-filter family (SEIF and so on) accrued marginalization error in the course of forcing sparsity. Submaps were absorbed into some systems but did not become the mainstream solution. What dominated the 2010s was factor graph plus iterative optimization. `[diverted]`

---

## 4.8 🧭 Still open

Filter vs. optimization coexistence. The EKF stepping back from backend primacy does not mean it disappeared. As of 2026, some autonomous-driving implementations still prefer filter-based backends. Optimization-based SLAM needs iterative convergence, and real-time guarantees are sometimes hard. In low-cost embedded systems, sparse EKFs and UKFs reappear. The filter did not die; use case and constraint decide the mix.

Non-Gaussian uncertainty. The most basic assumption of the EKF is that uncertainty follows a Gaussian distribution. Real-world sensor errors are often multi-modal or heavy-tailed. A single Gaussian severely oversimplifies actual uncertainty, especially under asymmetric perceptual aliasing (different places looking the same). Particle filters can, in theory, represent non-Gaussian distributions but are impractical in high-dimensional state. Stein particles, normalizing flows, and learning-based uncertainty estimation are being tried, but as of 2026 the forms validated inside real-time SLAM are limited.

---

While EKF-SLAM was hitting its real-time ceiling around 100 landmarks, at Imperial College Andrew Davison was using that small number to prove something else. A single camera, no other sensors, in real time. The numerical limit stayed the same; the way it was handled changed.
