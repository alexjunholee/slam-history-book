# Ch.7c — When Time Must Flow Smoothly: Continuous-Time Trajectory

Ch.7b's preintegration compressed IMU measurements into a relative factor between discrete keyframes. That compression presumes discrete endpoints: to fold the hundred inertial samples between two keyframes into one factor, each endpoint must have a definite timestamp. The assumption is harmless when a camera shutter opens and closes globally once per frame. Asynchronous sensors break it.

In 2012, in Toronto, [Paul Furgale, Timothy Barfoot, and Gabe Sibley](https://asrl.utias.utoronto.ca/~tdb/bib/furgale_iros12.pdf) formalized the question in an IROS paper. In an image captured by a rolling shutter, each row is projected from a different pose in time. A vehicle travels several meters while a spinning LiDAR completes one rotation, and the IMU produces samples at 1 kHz while the camera runs at 30 Hz. Furgale, Barfoot, and Sibley represented pose as a function of time $t$ rather than a frame and chose the B-spline. That choice became the standard starting point for continuous-time trajectory estimation.

Ten years later, the Handbook placed this branch alongside manifolds as one of SLAM's "two fundamental tools." Discrete keyframes cover most visual-inertial systems; continuous-time trajectory estimation addresses sensors that do not fit that clock.

---

## 7c.1 Limits of discrete-time

Ch.7b's preintegration handles one mismatch: the IMU runs faster than the camera. It does not address four others.

First, rolling shutter. A consumer CMOS camera reads one frame from top to bottom over tens of milliseconds. In a fast-moving camera, the first and last rows are captured from different poses. This distortion falls outside the photometric-consistency model assumed by Ch.8's DSO and LSD-SLAM. The Cremers group therefore added a B-spline trajectory to [Basalt](https://arxiv.org/abs/1904.06504) in 2019.

Second, spinning LiDAR motion distortion. As Ch.17 notes, the Velodyne HDL-64E completes one rotation at 10 Hz. If a vehicle moves at 10 m/s during those 100 ms, points within one scan are captured from poses 1 m apart. LOAM corrected the distortion indirectly inside the odometry loop; a continuous trajectory instead provides the pose at the instant each point was captured.

Third, event cameras. The DVS described in Ch.18 produces asynchronous events at μs granularity per pixel. Events have no frame, so [Mueggler et al. 2015](https://arxiv.org/abs/1502.00796) formulated event SLAM on an SE(3) B-spline trajectory.

Fourth, a high-rate IMU may be fused with several sensors running at different frequencies. When a system ingests a 200 Hz IMU, a 20 Hz camera, and a 10 Hz LiDAR, placing a discrete state node at every measurement time is impractical. The factor graph swells when the number of states grows with the number of measurements.

The four problems share one structure: measurement time $t_i$ is not controlled. Observations arrive asynchronously, and the estimator must know the pose at each arrival. A continuous-time representation separates measurement time, estimation time, and query time.

---

## 7c.2 Parametric spline: the Furgale line

Furgale, Barfoot, and Sibley chose the B-spline in 2012. The trajectory is written as a sum of basis functions, $\mathbf{p}(t) = \sum_k \Psi_k(t)\,\mathbf{c}_k$, with coefficients $\mathbf{c}_k$ as the optimization variables. Local support is the defining property: at any time $t$, only a handful of bases (usually four) are nonzero. Querying the pose at an arbitrary time $t_i$ therefore has constant cost and preserves factor-graph sparsity.

> 🔗 **Borrowed.** The mathematical skeleton of the B-spline comes from [de Boor's 1978 *A Practical Guide to Splines*](https://link.springer.com/book/10.1007/978-1-4612-6333-3). Furgale lifted that structure onto SE(3) and placed the coefficients as variable nodes in the factor graph, transplanting a tool from numerical analysis into SLAM optimization.

The trade-offs were clear. Closely spaced coefficients overfit, while wide spacing misses fast motion, and the choice depended on experience. A linear B-spline applied directly to SE(3) also produces interpolated poses outside the manifold.

In 2013, Oxford's [Steven Lovegrove et al.](https://www.roboticsproceedings.org/rss09/p11.html) proposed the cumulative B-spline. Rearranging the basis as a cumulative product rather than a sum, $T(t) = \prod_k \exp\bigl(\tilde\Psi_k(t) \log(T_k T_{k-1}^{-1})\bigr) \cdot T_0$, keeps each factor on the Lie group. This became a standard representation in later rolling-shutter, event-camera, and VIO papers. Basalt, [Mueggler's event SLAM](https://arxiv.org/abs/1502.00796), and [Kerl et al. 2015 dense rolling shutter VO](https://doi.org/10.1109/ICCV.2015.172) all used the cumulative B-spline.

The parametric spline remains common in real-time VIO and event systems because computation is light and the code is simple. It lacks a natural way to place a motion prior over the trajectory. Where observations are sparse, the spline remains smooth without a physically grounded reason. A GP-based branch adds such a prior.

---

## 7c.3 SDE-based GP: the Barfoot line and STEAM

In 2014, the Barfoot group in Toronto opened a second branch with ["Batch Continuous-Time Trajectory Estimation as Exactly Sparse Gaussian Process Regression"](https://www.roboticsproceedings.org/rss10/p01.pdf). Barfoot, Tong, and Särkkä treated the trajectory as a Gaussian process rather than a basis sum. A kernel $\mathcal{K}(t, t')$ defines the prior, and the posterior remains a conditional Gaussian when observations arrive.

A dense GP has one problem: with $N$ observations, inverting the kernel matrix $K$ costs $O(N^3)$. Barfoot, Tong, and Särkkä identified a family of kernels that avoids this cost. When the trajectory is the solution of a linear time-invariant stochastic differential equation $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + L\mathbf{w}(t)$, the inverse $K^{-1}$ of its kernel has a block-tridiagonal structure. In factor-graph terms, binary factors connect only consecutive state nodes, not distant ones.

> 🔗 **Borrowed.** Reinterpreting the GP posterior as a factor-graph prior follows the SDE-GP connection in [Särkkä's 2013 *Bayesian Filtering and Smoothing*](https://users.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf), which the Barfoot group brought into SLAM. The Rasmussen-Williams GP textbook writes the kernel in closed form, but real-time SLAM needs a sparse inverse. Särkkä's SDE representation supplied the bridge.

The result was **STEAM** (Simultaneous Trajectory Estimation and Mapping). At RSS 2015, [Sean Anderson and Barfoot 2015, "Full STEAM Ahead"](https://www.roboticsproceedings.org/rss11/p45.pdf) formalized STEAM with a constant-velocity prior. It augments the state with pose $\mathbf{p}(t)$ and velocity $\mathbf{v}(t)$, with pose following from the white-noise integral of velocity. Anderson tightened the sparsity proof that same year, and it became the foundation of the Barfoot group's later continuous-time papers.

STEAM's second advantage was GP interpolation. With only a small number of control poses, the estimator can query any intermediate pose as the posterior mean. Even when a spinning LiDAR captures 10,000 points at 10,000 instants within one scan, the model uses only one control point per scan. Computation scales with the number of control points rather than observations.

In 2019, Tang and Barfoot's [open-source STEAM release](https://github.com/utiasASRL/steam) gave academia and industry a directly usable library. The same year, the Dellaert group's GTSAM received a GP continuous-time factor in contrib. The two paths had converged.

---

## 7c.4 Continuous-time on the Lie group

Whether parametric or nonparametric, SLAM needs a trajectory on SE(3). Lifting a Euclidean spline or GP onto the group is not straightforward. The common approach works in the tangent space: interpolate linearly there, then return the result to the manifold with the exponential map.

On the B-spline side, [Sommer, Demmel et al. 2020, "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups"](https://arxiv.org/abs/1911.08860) derived the SE(3) cumulative-spline Jacobian in closed form. The CVPR paper supplied a standard B-spline trajectory formulation with real-time derivatives for rolling-shutter VIO, event cameras, and visual-inertial systems. Basalt and later work from the Cremers group used this result.

On the GP side, Anderson and Barfoot proposed a "local variable" construction. Near each control pose $T_k$, they define a local perturbation $\xi_k(t) = \log(T(t)\,T_k^{-1})$ and run the GP on it. A GP is difficult to define directly on the global manifold, but a Euclidean GP can be defined in the tangent space around each control point. Crossing between control points introduces an adjoint, the same Lie-group operation that appears in Ch.7b's on-manifold preintegration. From 2015 onward, both tools used a common Lie-group grammar.

> 🔗 **Borrowed.** [Anderson-Barfoot 2015 ICRA](https://doi.org/10.1109/ICRA.2015.7138984) systematically developed the use of a GP in a Lie-group local variable. Several later continuous-time LiDAR and VIO papers used the same construction: run a GP between two consecutive control points and apply the adjoint when crossing between them.

The practical difference between a spline and GP is the motion prior. A spline estimates coefficients directly without one. A GP carries an SDE-derived prior, such as constant velocity or white jerk. Where observations are sparse, the prior supports the GP, while the spline relies on neighboring observations. Attempts to combine them (Johnson et al. 2020) have appeared, but the choice remains application-dependent.

---

## 7c.5 The line descends to applications: LiDAR and VIO

It took ten years for the theory to reach deployed applications. Around 2022, continuous-time became a common solution in three areas.

First, LiDAR motion distortion. Paris's [Pierre Dellenbach et al. 2022, "CT-ICP"](https://arxiv.org/abs/2109.12979) parameterized each scan with two poses (a "start pose" and an "end pose") and interpolated linearly between them. Despite its simple model, CT-ICP beat prior LOAM and FAST-LIO accuracy on the KITTI, NCLT, and Newer College benchmarks. The same year, Toronto's [Keenan Burnett et al. 2022, "Are We Ready for Radar to Replace Lidar?"](https://arxiv.org/abs/2206.05432) and [STEAM-ICP](https://github.com/utiasASRL/steam_icp) applied GP-based continuous time to the Aeva FMCW LiDAR. The sensor reports Doppler velocity with each point, which maps directly to STEAM's velocity state. Without a continuous-time representation, that information could not enter the estimator directly.

Second, rolling-shutter VIO. Basalt, the [Cremers group rolling-shutter VO](https://doi.org/10.1109/CVPR.2016.71), and follow-ups to [OKVIS](https://doi.org/10.1177/0278364914554813) query each image row's capture time on a B-spline trajectory. Rather than assuming a global shutter, they model the rolling shutter directly.

Third, event cameras. After the 2010s difficulties described in Ch.18, several lines of event SLAM in the 2020s used continuous-time trajectories. Each event's μs timestamp is queried against a B-spline or GP to obtain the pose at that instant, and event-image consistency supplies the residual. Continuous-time trajectories fit event cameras because neither assumes frames.

> 🔗 **Borrowed.** CT-ICP is a combination that lays intra-scan continuous-time linear interpolation on top of the point-to-plane objective of [Besl and McKay 1992 ICP](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf). Classic registration and Furgale's continuous-time spirit met inside one system, thirty years apart.

---

## 📜 Prediction vs. outcome

> In the Future Work of their 2012 IROS paper, Furgale, Barfoot, and Sibley wrote two expectations. One was that "continuous-time representation will become the natural language for unifying rolling shutter and high-rate IMU sampling"; the other was "follow-up work proving compatibility with a sparse factor graph." Both were realized within a decade. Barfoot, Tong, and Särkkä 2014 completed the sparse GP proof, while rolling-shutter VIO and event SLAM of the 2020s use the cumulative B-spline as their standard representation. The authors did not anticipate one development. In 2012, the implicit division of labor placed discrete-keyframe ORB-SLAM in the mainstream and continuous time with specialty sensors. Development also moved in the opposite direction: when Burnett released STEAM-ICP using Doppler velocity from an FMCW LiDAR, continuous time became a way to exploit sensor-specific signals.

---

## 🔗 Borrowed (summary)

Three further lineages underpin the chapter.

Särkkä's SDE-GP textbook anchored the equations of Barfoot, Tong, and Särkkä 2014. De Boor's 1978 spline text supplied Furgale 2012 with its basis functions, and Anderson and Barfoot's 2015 local-variable technique brought GPs onto Lie groups. Continuous-time trajectory estimation brings numerical analysis, probability theory, and Lie-group differential geometry together within SLAM.

---

## 🧭 Still open

**Learning-based continuous-time prior.** The motion prior that an SDE provides embeds physical assumptions such as constant-velocity or white-jerk. Real driving, walking, and UAV trajectories often violate these assumptions. In 2023-2024, attempts appeared to learn data-driven priors with neural SDE or neural ODE and plug them into the continuous-time factor graph. A system that layers a learned prior while keeping real-time sparse structure is still at the validation stage.

**Integration of VIO and continuous-time.** Ch.7b's preintegration remains the dominant formulation in keyframe-based VIO. As of 2026, no consensus exists on whether continuous-time trajectories should replace preintegration or coexist with it in a hybrid. Le Gentil's [GP-augmented preintegration line](https://arxiv.org/abs/2007.04144) connects the two, but deployed systems such as ORB-SLAM3 and VINS-Fusion still rely on discrete-time preintegration.

**Online sliding window for edge deployment.** STEAM- and B-spline-based systems slow as control points accumulate. Marginalizing past control points while preserving the consistency of the continuous-time posterior remains technically difficult. Continuous-time SLAM cannot become a long-lived standard on embedded platforms such as cars and drones without solving this problem.

---

Ch.7b optimized discrete-time preintegration; this chapter followed the continuous-time alternative. The two tools need not compete. Since 2024, systems have placed an IMU preintegration factor and a continuous-time LiDAR factor side by side in one SLAM estimator. Ch.8 returns to the visual line, where DSO and VI-DSO use the Forster factor and the direct photometric approach develops separately from both preintegration supplements.
