# Ch.4 — Smith-Cheeseman and the Rise and Fall of EKF-SLAM

The photogrammetry, SfM, and bundle adjustment of Part I shared one assumption: either the camera remained still or there was time after capture to process every image offline as a batch. Hartley-Zisserman's geometry, RANSAC's robust estimation, and Levenberg-Marquardt's iterative optimization could measure the world, but they did not ask where a moving robot was *right now*. Part II begins with that question. The robot must build a map, locate itself, and maintain the estimate as uncertainty accumulates. Probabilistic mapping began with a small memo from SRI International.

In 1986 Randall Smith and Peter Cheeseman set out to formalize uncertainty in a robot's spatial measurements. Their work at SRI International inherited Kalman's (1960) filter mathematics but extended it from a single state estimate to an entire *network of spatial relationships*. Several years later, Hugh Durrant-Whyte in Sydney and John Leonard at MIT coupled that mathematics to a new problem statement: a robot estimates its own position while building a map. The acronym "SLAM" emerged from that union.

---

## 4.1 The mathematics of uncertain spatial relationships — Smith, Self, Cheeseman (1988)

In 1986 Randall Smith and Peter Cheeseman at SRI International set out to express how error propagates when a robot accumulates measurements across several places. Their working notes appeared in 1988 as ["Estimating Uncertain Spatial Relationships in Robotics"](https://arxiv.org/abs/1304.3111). The question was clear: when a robot measures B from A and then C from B, how is the uncertainty from A to C computed?

The [Kalman filter](https://www.cs.unc.edu/~welch/kalman/kalmanPaper.html) already existed. It had been used since 1960 for radar tracking, ballistic calculation, and satellite-orbit correction. Smith and Cheeseman reformulated Kalman's covariance-propagation equations for the composition of spatial transforms. They placed the robot pose $\mathbf{x}_r$ and landmark positions $\mathbf{m}_i$ in one state vector and maintained the joint covariance $\mathbf{P}$ over the entire state.

$$\mathbf{x} = [\mathbf{x}_r^\top,\ \mathbf{m}_1^\top,\ \ldots,\ \mathbf{m}_N^\top]^\top$$

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rm} \\ \mathbf{P}_{mr} & \mathbf{P}_{mm} \end{bmatrix}$$

The off-diagonal block $\mathbf{P}_{rm}$ records the crucial relation: robot-pose uncertainty and landmark-position uncertainty are *correlated*. Only by tracking that correlation can the estimate remain consistent. The paper demonstrated this explicitly, establishing a starting point for the SLAM field.

> 🔗 **Borrowed.** Smith-Cheeseman's (1988) spatial-relationship mathematics inherits directly from Kalman's (1960) covariance propagation. A technique for tracking a single moving object became a framework for tracking a robot and every element of its map at once.

---

## 4.2 How the name "SLAM" settled in

There is no "SLAM" in the 1988 Smith-Cheeseman paper. In the early 1990s, Hugh Durrant-Whyte, who had moved from Oxford to Sydney, and John Leonard at MIT used different names for the same problem in their respective labs. Once the groups began citing each other, they needed a shared term, and "SLAM" gradually became standard. Researchers' memories differ on which document used it first. No canonical first-use paper exists.

Leonard and Durrant-Whyte's 1991 paper, ["Simultaneous Map Building and Localization for an Autonomous Mobile Robot"](https://doi.org/10.1109/IROS.1991.174711), is often cited as an early mainstream robotics paper to state the problem in its title. The title captured the intuition before the acronym existed: mapping and localization are inseparable and must be performed simultaneously.

The name was "Simultaneous Localization and Mapping," abbreviated SLAM. For the next ten years, the field converged around it.

> 🔗 **Borrowed.** [Bar-Shalom's multi-target tracking](https://archive.org/details/trackingdataasso0000bars) (multi-target tracking, collected as a 1988 monograph) supplied a framework for estimating the states of many objects at once. Leonard and Durrant-Whyte can be read as mapping "target position" to "landmark position" and "tracker position" to "robot pose" inside that framework. This was a case of radar technology translated into indoor robot mapping.

---

## 4.3 The EKF-SLAM formulation

The Extended Kalman Filter (EKF) was already standard in nonlinear system estimation before 1988, making its application to SLAM unsurprising. It runs in two stages: predict and update.

During the predict stage, the motion model $f(\cdot)$ predicts the state as the robot moves, and the Jacobian $\mathbf{F}$ propagates the covariance.

$$\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, \mathbf{u})$$
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$$

During the update stage, a sensor measurement $\mathbf{z}$ arrives, and the Jacobian $\mathbf{H}$ of the observation model $h(\cdot)$ yields a Kalman gain $\mathbf{K}$ that updates the state and covariance.

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top(\mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

These two stages define EKF-SLAM. The structure is simple, and that simplicity imposed a scalability ceiling from the start.

The problem is state dimension. A state containing a 6DOF pose and $N$ 3D landmarks has dimension $6 + 3N$; its covariance matrix contains $(6+3N)^2$ entries, an $O(N^2)$ structure. A single update costs $O(N^2)$ both for the Kalman gain (inverting $\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$) and for the covariance update. 100 landmarks gives $306 \times 306 \approx$ 94k entries; 1,000 landmarks gives $3006 \times 3006 \approx$ 9M. A regular PC in the early 2000s could maintain only tens to low hundreds of landmarks in real time.

[Andrew Davison's MonoSLAM (2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf) was limited to a few dozen landmarks in its live demos because EKF-SLAM's $O(N^2)$ wall set the ceiling.

---

## 4.4 The scalability wall

When Davison ran real-time 3D tracking from a single webcam at ICCV 2003, he mapped a desk-sized space with a few dozen features. With no commercial SLAM systems available, a real-time monocular demo was rare. Its ceiling came from the size of the covariance matrix.

At 100 landmarks the covariance matrix is $306 \times 306$ (6DOF pose + 100 3D landmarks, state dimension $6 + 3 \times 100 = 306$). At 1,000 it is $3006 \times 3006$. Every time step that matrix has to be updated along with a matrix inversion. On top of that, because the EKF keeps the full joint distribution in one block, adding a new landmark immediately generates cross-correlations with every existing landmark. As the map grows, update cost grows quadratically.

The main workaround through the mid-2000s was submapping: partition the map into small overlapping regions, run an EKF inside each one, and connect them through a separate structure. [Chong and Kleeman (1999)](http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/integrated1/chong_feature_map.pdf) proposed an early form. Information loss at submap boundaries, difficult loop closure, and implementation complexity made these approaches hard to deploy.

> 🔗 **Borrowed.** The submap-partitioning idea of Chong-Kleeman (1999) carries forward into the local-window optimization of modern SLAM. ORB-SLAM's local map and VINS-Mono's sliding window sit conceptually on the same principle. Only the implementation tool changed, from EKF to bundle adjustment.

---

## 4.5 The consistency problem: Julier-Uhlmann's counterexample

A deeper flaw in EKF-SLAM surfaced at ICRA 2001. Simon Julier and Jeffrey Uhlmann analyzed EKF-based SLAM through numerical experiments and showed that the filter trusts itself too much. Their IEEE ICRA paper was titled ["A Counter Example to the Theory of Simultaneous Localization and Map Building"](https://doi.org/10.1109/ROBOT.2001.933257). The title was provocative, and the content matched it.

Secondary literature summarizes the result this way: EKF-SLAM becomes asymptotically *overconfident*. Actual estimation error grows while the covariance (uncertainty) computed by the filter converges below its true value. This is inconsistency.

The cause sits in linearization error. The EKF approximates nonlinear motion and observation models by a first-order Taylor expansion. When this approximation error accumulates step by step, the covariance begins to underestimate the real error. Once the robot becomes overconfident that "I am here," the filter trusts subsequent measurements less, and errors pile up without correction.

In 2007 [Shoudong Huang and Gamini Dissanayake](https://doi.org/10.1109/TRO.2007.903811) analyzed the cause of this inconsistency more precisely. They found that basic constraints among Jacobians evaluated at the current state estimate break down, driving EKF-SLAM's inconsistency. As a result, the variance of the robot's heading angle (yaw) can wrongly converge to zero when it should remain nonzero. Later observability-based analyses start from this result: the system's observable degrees of freedom change with the linearization point, and the filter injects spurious information into unobservable directions.

> 📜 **Prediction vs. outcome.** After Julier and Uhlmann's 2001 counterexample, researchers spent nearly a decade designing consistent estimators. Filter variants included the Unscented Kalman Filter (UKF), Invariant EKF, and robust covariance methods. From the vantage point of 2026, however, the practical resolution came *not from filtering but from optimization*. [iSAM](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf) (Kaess et al., 2008), [g2o](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf) (Kümmerle et al., 2011), and GTSAM effectively replaced the filter. Iterative optimization refreshes the Jacobian linearization rather than freezing it at the current estimate, avoiding the inconsistency structurally. Optimization, not a new filter, filled the gap exposed by the counterexample.

---

## 4.6 FastSLAM — divide and conquer

[FastSLAM](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf) attacked EKF-SLAM's $O(N^2)$ wall from another direction. Michael Montemerlo, Sebastian Thrun (Stanford), Daphne Koller, and Ben Wegbreit presented it at AAAI 2002.

Rao-Blackwellization supplies the key observation. Given the robot path $x_{0:t}$, the position estimates of each landmark become *mutually independent*. The path can therefore be represented by a particle filter (each particle standing for one possible path), with a separate landmark EKF running independently for each particle.

With $K$ particles and $N$ landmarks the per-step complexity is $O(K \log N)$, logarithmic in $N$ for fixed $K$ rather than quadratic as in EKF-SLAM (when using KD-tree-based landmark search). As landmark count rises, per-particle EKFs stay mutually independent, so there is no need to keep the full $N \times N$ covariance. $K$ is fixed at tens to hundreds, and the practical gain was large.

FastSLAM maintained real-time operation with a few hundred landmarks in indoor environments and saw rapid adoption. Problems accumulated nonetheless. Particle depletion came first: as the map grows, most particles represent poor paths, and the effective sample count drops sharply. Reweighting paths during loop closure is difficult, and adding more particles did not solve drift accumulation in large-scale environments.

[FastSLAM 2.0](https://www.ijcai.org/Proceedings/03/Papers/165.pdf) (Montemerlo et al. 2003) improved the proposal distribution, but the filter paradigm still imposed a scalability ceiling. Graph optimization eventually bypassed it.

---

## 4.7 The EKF's exit

Graph-based approaches became practical from 2005 onward, and EKF-SLAM receded from the main line. [Feng Lu and Evangelos Milios's 1997 graph idea](https://doi.org/10.1023/A:1008854305733) combined with [Olson-Leonard-Teller's (2006)](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf) efficient solver and then with the real-time factorization techniques of g2o, GTSAM, and iSAM2. The EKF's advantage of incremental updates was no longer distinctive.

The difference appeared at loop closure, when map error must be corrected as the robot returns to its starting point. The EKF has to update the entire covariance matrix at that moment, at a cost of $O(N^2)$. Graph optimization adds one edge to the pose graph and refactors a sparse matrix, at far lower cost.

Around 2010, choosing the EKF as the backend for a new SLAM system became uncommon. It survived chiefly under special constraints, such as very limited compute resources or a requirement for real-time filtering.

> 📜 **Prediction vs. outcome.** Durrant-Whyte and Bailey's [2006 IEEE Robotics & Automation Magazine tutorial](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf) discussed SLAM's scalability and projected submap decomposition and the information filter as solutions for large-scale environments. The information filter (the EKF's inverse-covariance form) was expected to use a sparse information matrix to keep computation from slowing as landmark count grew. Development took another route. The information-filter family (SEIF and related methods) accrued marginalization error while forcing sparsity. Submaps entered some systems but did not become the mainstream solution. Factor graphs with iterative optimization dominated the 2010s.

---

## 4.8 🧭 Still open

Filter vs. optimization coexistence. The EKF's retreat from backend primacy does not mean it disappeared. As of 2026, some autonomous-driving implementations still prefer filter-based backends. Optimization-based SLAM needs iterative convergence, which can make real-time guarantees difficult. Sparse EKFs and UKFs reappear in low-cost embedded systems. The mix depends on the use case and its constraints.

Non-Gaussian uncertainty. The EKF assumes that uncertainty follows a Gaussian distribution. Real-world sensor errors are often multimodal or heavy-tailed. A single Gaussian severely oversimplifies actual uncertainty, especially under asymmetric perceptual aliasing (different places looking the same). Particle filters can represent non-Gaussian distributions in theory but become impractical in high-dimensional states. Stein particles, normalizing flows, and learning-based uncertainty estimation are being tried, but few forms have been validated inside real-time SLAM as of 2026.

---

While EKF-SLAM reached its real-time ceiling at around 100 landmarks, Andrew Davison at Imperial College used that same limited landmark budget to prove that a single camera could track in real time without other sensors. The numerical limit remained; the design around it changed.
