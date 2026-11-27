# Chapter 10. Continuous-Time Trajectory Estimation

Chapter 9's preintegration compressed high-rate IMU readings into a single relative factor between discrete keyframes. Yet that compression rests on discrete endpoints: folding a hundred inertial samples between two keyframes into one factor assumes each endpoint has a well-defined timestamp. That assumption holds when a global shutter exposes every pixel simultaneously, but asynchronous sensors break it.

In 2012, working in Toronto, [Paul Furgale, Timothy Barfoot, and Gabe Sibley](https://asrl.utias.utoronto.ca/~tdb/bib/furgale_iros12.pdf) tackled that mismatch at IROS. Inside a rolling-shutter image, each row is exposed from a slightly different pose; a vehicle travels several meters during a single rotation of a spinning LiDAR; and an IMU fires at 1 kHz while a camera runs at 30 Hz. Rather than indexing poses by frame number, Furgale, Barfoot, and Sibley modeled the trajectory as a continuous function of time $t$ parameterized by a B-spline—establishing the canonical starting point for continuous-time trajectory estimation.

A decade later, *The SLAM Handbook* placed continuous-time trajectories alongside Lie-group manifolds as one of SLAM's "two fundamental tools." Discrete keyframes remain the workhorse of standard visual-inertial pipelines; continuous-time estimation steps in whenever sensors refuse to march to a single clock.

---

## 10.1 Limits of discrete-time

Chapter 9's preintegration resolves one rate mismatch—an IMU sampling far faster than a frame camera—but four common sensor regimes still resist discrete keyframes.

First is the rolling shutter. Consumer CMOS sensors read out an image sequentially from top to bottom over tens of milliseconds, so under fast camera motion the top and bottom rows observe the world from different poses. That intra-frame skew violates the global-shutter photometric model assumed by Chapter 11's DSO and LSD-SLAM, prompting Cremers's group to adopt a B-spline trajectory in [Basalt](https://arxiv.org/abs/1904.06504) (2019).

Second is motion distortion in spinning LiDARs. As Chapter 21 details, a Velodyne HDL-64E sweeps a full revolution at 10 Hz: at 10 m/s, the vehicle advances a full meter over those 100 ms, scattering each scan's points across different poses along that arc. Whereas LOAM unwound this distortion indirectly inside its odometry loop, a continuous-time trajectory yields the exact sensor pose at every point's timestamp.

Third are event cameras. A Dynamic Vision Sensor (DVS) emits asynchronous pixel events at microsecond resolution with no concept of a frame (→ Chapter 22), leading [Mueggler et al. 2015](https://arxiv.org/abs/1502.00796) to formulate event-based SLAM directly on an SE(3) B-spline.

Fourth is multi-rate sensor fusion across heterogeneous clocks. Combining a 200 Hz IMU, a 20 Hz camera, and a 10 Hz LiDAR in a discrete factor graph would bloat the state vector if every asynchronous measurement spawned its own node.

All four problems share one structural root: measurement timestamps $t_i$ arrive on independent schedules, yet the estimator must know the pose at each arrival instant. A continuous-time representation decouples *when measurements arrive* from *how the trajectory is parameterized* and *when it is queried*.

---

## 10.2 Parametric spline: the Furgale line

Furgale, Barfoot, and Sibley (2012) built that continuous trajectory out of B-splines, expressing the path as a linear combination of basis functions $\mathbf{p}(t) = \sum_k \Psi_k(t)\,\mathbf{c}_k$ whose control-point coefficients $\mathbf{c}_k$ serve as the optimization variables. The decisive property is local support: at any instant $t$, only a small window of basis functions (typically four for a cubic spline) is nonzero, keeping pose queries at an arbitrary time $t_i$ constant-time and preserving sparsity in the factor graph.

> 🔗 **Borrowed.** The B-spline machinery comes directly from [de Boor's 1978 *A Practical Guide to Splines*](https://link.springer.com/book/10.1007/978-1-4612-6333-3). By lifting those bases onto SE(3) and treating the control coefficients as variable nodes in a factor graph, Furgale transplanted a classic tool of numerical analysis into SLAM optimization.

Working with splines still entailed practical trade-offs. Placing control points too densely invited overfitting, while spacing them too widely smoothed away agile motion, leaving knot spacing largely a matter of empirical tuning. Worse, applying a linear Euclidean B-spline directly to SE(3) matrices pulls interpolated poses off the Lie group manifold.

In 2013, Oxford's [Steven Lovegrove et al.](https://www.roboticsproceedings.org/rss09/p11.html) resolved the manifold issue with the **cumulative B-spline**. Replacing the linear sum with a cumulative product of exponential updates, $T(t) = \prod_k \exp\bigl(\tilde\Psi_k(t) \log(T_k T_{k-1}^{-1})\bigr) \cdot T_0$, guarantees that every interpolated pose stays strictly on the Lie group. That formulation quickly became standard across rolling-shutter, event-camera, and VIO systems, underpinning Basalt, [Mueggler's event SLAM](https://arxiv.org/abs/1502.00796), and [Kerl et al.'s (2015) dense rolling-shutter VO](https://doi.org/10.1109/ICCV.2015.172).

Because evaluation is lightweight and implementation straightforward, parametric splines remain widely used in real-time VIO and event pipelines. Yet without an explicit motion prior, trajectory behavior across sparsely observed gaps hinges on the basis degree and knot placement; while penalties can be attached to spline coefficients or derivatives, the Gaussian-process branch took the more principled route of defining a probabilistic prior over trajectories from the outset.

---

## 10.3 SDE-based GP: the Barfoot line and STEAM

In 2014, Barfoot's group in Toronto opened a second branch with ["Batch Continuous-Time Trajectory Estimation as Exactly Sparse Gaussian Process Regression"](https://www.roboticsproceedings.org/rss10/p01.pdf). Rather than expanding the trajectory in a fixed basis, Barfoot, Tong, and Särkkä modeled it as a continuous-time Gaussian process governed by a kernel $\mathcal{K}(t, t')$, so that incoming measurements update the prior into a conditional Gaussian posterior.

A generic dense GP hits an $O(N^3)$ wall when inverting the kernel matrix $K$ across $N$ observations. Barfoot, Tong, and Särkkä bypassed that bottleneck by restricting attention to kernels generated by linear time-invariant stochastic differential equations (SDEs) of the form $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + L\mathbf{w}(t)$. Because such Markovian SDEs produce an inverse kernel matrix $K^{-1}$ that is strictly block-tridiagonal, binary factors in the resulting factor graph link only adjacent state nodes rather than coupling distant times.

> 🔗 **Borrowed.** Turning an SDE-driven Gaussian process prior into a sparse factor graph builds on the SDE–GP equivalence formalized in [Särkkä's 2013 *Bayesian Filtering and Smoothing*](https://users.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf). Where Rasmussen and Williams's GP textbook emphasizes closed-form covariance kernels, real-time SLAM demanded a sparse precision matrix—and Särkkä's SDE formulation provided it.

This insight gave rise to **STEAM** (Simultaneous Trajectory Estimation and Mapping). At RSS 2015, [Sean Anderson and Barfoot 2015, "Full STEAM Ahead"](https://www.roboticsproceedings.org/rss11/p45.pdf) instantiated STEAM with a white-noise-on-acceleration (constant-velocity) prior, augmenting the state with pose $\mathbf{p}(t)$ and velocity $\mathbf{v}(t)$. That same year, Anderson tightened the sparsity proof, laying the theoretical foundation for a decade of continuous-time work from Toronto.

STEAM's second payoff was exact GP interpolation. From a sparse set of control poses, the estimator can query the posterior mean at any intermediate instant: even when a spinning LiDAR records 10,000 points at 10,000 distinct timestamps within a single sweep, the estimator may allocate only one control state per scan, keeping the optimization dimension far smaller than the observation count (even though evaluating and accumulating the individual residuals still scales with the measurements).

In 2019, Tang and Barfoot's [open-source STEAM release](https://github.com/utiasASRL/steam) put the framework into wider academic and industrial hands, the same year GTSAM's `contrib` module gained a GP continuous-time factor—marking the convergence of the Toronto and Atlanta factor-graph traditions.

---

## 10.4 Continuous-time on the Lie group

Whether parametric or GP-based, practical SLAM requires trajectories on SE(3), and lifting Euclidean interpolation onto a curved manifold demands care. Both branches converged on the same strategy: formulate smooth updates in the local tangent space, then map them back onto the group via the exponential map.

On the B-spline side, [Sommer, Demmel et al. 2020, "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups"](https://arxiv.org/abs/1911.08860) derived closed-form analytical Jacobians for cumulative splines on SE(3). That CVPR paper supplied a fast, exact differentiation engine for rolling-shutter VIO and event SLAM, quickly adopted in Basalt and subsequent work from Cremers's group.

On the GP side, Anderson and Barfoot introduced a "local variable" parameterization: around each control pose $T_k$, they define a tangent-space perturbation $\xi_k(t) = \log(T(t)\,T_k^{-1})$ and run the GP over that Euclidean local coordinate. Stepping across intervals between consecutive control poses introduces an adjoint transformation—the exact same Lie-group operator used in Chapter 9's manifold preintegration. By 2015, both lines spoke a unified Lie-group calculus.

> 🔗 **Borrowed.** [Anderson-Barfoot 2015 ICRA](https://doi.org/10.1109/ICRA.2015.7138984) systematized Gaussian processes inside Lie-group local coordinates. Subsequent continuous-time LiDAR and VIO estimators widely adopted this recipe: evolve a Euclidean GP locally between adjacent control states and propagate across boundaries with the group adjoint.

Comparing the two branches clarifies how they handle motion priors. B-spline coefficients or their derivatives can be regularized with explicit smoothness penalties, whereas an SDE-based GP bakes a physical prior—such as constant velocity or white jerk—directly into the kernel precision matrix. Where measurements are sparse, the GP leans naturally on its stochastic prior, while a spline leans more heavily on neighboring observations. Hybrid formulations (such as Johnson et al. 2020) have been explored, though in practice the choice still tracks the target sensor suite.

---

## 10.5 The line descends to applications: LiDAR and VIO

Originally developed largely for multi-sensor calibration, continuous-time trajectories moved into mainstream front ends around 2022 across three domains.

First came LiDAR deskewing and registration. In Paris, [Pierre Dellenbach et al. 2022, "CT-ICP"](https://arxiv.org/abs/2109.12979) parameterized each LiDAR sweep with just two endpoints—a start pose and an end pose—interpolating linearly between them; despite its simplicity, CT-ICP surpassed LOAM and FAST-LIO in accuracy on KITTI, NCLT, and Newer College. That same year in Toronto, [Keenan Burnett et al. 2022, "Are We Ready for Radar to Replace Lidar?"](https://arxiv.org/abs/2206.05432) and [STEAM-ICP](https://github.com/utiasASRL/steam_icp) applied GP continuous-time trajectories to Aeva FMCW LiDARs, whose per-point Doppler velocity measurements map directly onto STEAM's continuous velocity state—information impossible to exploit cleanly without an explicit velocity trajectory.

Second was rolling-shutter VIO. Systems such as Basalt, the [Cremers group's rolling-shutter VO](https://doi.org/10.1109/CVPR.2016.71), and extensions of [OKVIS](https://doi.org/10.1177/0278364914554813) query the B-spline at every image row's exposure time, modeling shutter skew explicitly rather than forcing a global-shutter approximation.

Third were event cameras. After struggling through the 2010s with brittle frame-like heuristics, multiple event-SLAM pipelines in the 2020s queried a B-spline or GP at each event's microsecond timestamp and minimized event-image consistency residuals—a natural fit because neither the sensor nor the trajectory presupposes frames.

> 🔗 **Borrowed.** CT-ICP pairs intra-scan continuous-time interpolation with the classic point-to-plane ICP objective descended from [Besl and McKay 1992 ICP](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf). Thirty years after Besl and McKay, classical surface registration met Furgale's continuous-time philosophy inside a single estimator.

---

## Prediction vs. outcome

> In the Future Work section of their 2012 IROS paper, Furgale, Barfoot, and Sibley articulated two expectations: that continuous-time representations would become the natural language for unifying rolling-shutter cameras and high-rate IMUs, and that follow-up proofs would establish compatibility with sparse factor graphs. Within a decade, both materialized: Barfoot, Tong, and Särkkä (2014) delivered the sparse GP proof, and cumulative B-splines became the standard backbone for rolling-shutter VIO and event SLAM in the 2020s. What the authors did not foresee was a cross-pollination in the other direction. In 2012, discrete keyframes seemed destined for mainstream cameras while continuous time looked specialized; yet when Burnett's STEAM-ICP harnessed FMCW LiDAR Doppler velocities, continuous-time state representations emerged as an enabling way to ingest physics-rich sensor signals that discrete poses discard.

---

## Borrowed Foundations (summary)

Three foundational borrowings anchor this chapter's trajectory mathematics. De Boor's 1978 spline treatise supplied Furgale (2012) with local-support basis functions; Särkkä's 2013 SDE–GP formulation gave Barfoot, Tong, and Särkkä (2014) a sparse block-tridiagonal precision matrix; and Anderson and Barfoot's 2015 local-variable construction embedded Gaussian processes cleanly on Lie groups. Together, they fused numerical approximation theory, stochastic processes, and differential geometry inside SLAM.

---

## Still open

**Learning-based continuous-time prior.** SDE motion priors encode clean physical idealizations—constant velocity or white-noise jerk—that real driving, walking, and agile drone flight routinely violate. In 2023–2024, researchers began exploring neural SDEs and neural ODEs to learn data-driven motion priors inside continuous-time factor graphs, though keeping real-time sparsity intact with a learned prior remains at the validation stage.

**Integration of VIO and continuous-time.** Discrete preintegration from Chapter 9 still dominates keyframe-based VIO. As of 2026, no consensus settles whether continuous-time trajectories should replace preintegration or run alongside it: Le Gentil's [GP-augmented preintegration line](https://arxiv.org/abs/2007.04144) bridges both worlds, yet widely deployed pipelines like ORB-SLAM3 and VINS-Fusion stick with discrete factors.

**Online sliding window for edge deployment.** Both STEAM and B-spline estimators slow down if control states accumulate unchecked. Marginalizing older control points while preserving a consistent continuous-time posterior remains mathematically and algorithmically delicate—a barrier that must fall before continuous-time SLAM becomes standard on resource-constrained cars and drones.

---

Continuous-time trajectories and discrete preintegration are not mutually exclusive: since 2024, multi-sensor estimators have routinely paired a discrete IMU preintegration factor with a continuous-time LiDAR factor inside the same graph. On the pure-vision side, meanwhile, DSO and VI-DSO built upon Forster's discrete factor while taking a wholly different approach to image measurements—abandoning keypoints altogether to optimize raw pixel intensities directly (→ Chapter 11).