# Ch.7c — When Time Must Flow Smoothly: Continuous-Time Trajectory

The preintegration Ch.7b organized was an engineering move that compressed IMU measurements into a relative factor between discrete keyframes. That compression presumes the unit called "keyframe." For the hundred inertial samples arriving between two keyframes to fold into one factor, both endpoints of the factor must carry a definite timestamp. In a system where the camera shutter opens and closes globally once per frame, the assumption is harmless. The trouble sat elsewhere.

In 2012, in Toronto, [Paul Furgale, Timothy Barfoot, and Gabe Sibley](https://asrl.utias.utoronto.ca/~tdb/bib/furgale_iros12.pdf) formalized the question in an IROS paper. In one image captured by a rolling shutter, each row is projected from a different pose in time. While a spinning LiDAR completes one rotation, the vehicle travels several meters. The IMU pours out samples at 1 kHz while the camera runs at 30 Hz. The most natural way to tie these sensors into a single optimization was to treat the pose not as a "frame" but as a "function of time $t$." Furgale, Barfoot, and Sibley picked the B-spline, and that choice became the official starting point of the branch known as continuous-time trajectory estimation.

Ten years later, the Handbook places this branch, alongside manifolds, as one of the "two fundamental tools" of SLAM. The reason earlier chapters of this book have never pulled this tool out is that the grammar of Visual-Inertial was largely completed on discrete keyframes. This chapter fills that gap.

---

## 7c.1 Limits of discrete-time

What Ch.7b's preintegration solved was a single axis: "the IMU is faster than the camera." Four more axes it did not solve.

First, rolling shutter. A consumer CMOS camera reads one frame top-to-bottom over tens of milliseconds. In a fast-moving camera the first row and the last row are captured from different poses. When Ch.8's DSO and LSD-SLAM assumed photometric consistency, this distortion sat outside the model. That is why the Cremers group mounted a B-spline trajectory onto [Basalt](https://arxiv.org/abs/1904.06504) in 2019.

Second, spinning LiDAR motion distortion. As seen in Ch.17, the Velodyne HDL-64E completes one rotation at 10 Hz. If the vehicle runs at 10 m/s during those 100 ms, the points within a single scan are captured from poses 1 m apart. LOAM corrected this distortion indirectly inside the odometry loop, but the principled solution was a trajectory representation that allowed querying "the pose at the instant each point was captured."

Third, event cameras. The DVS recorded in Ch.18 pours asynchronous events at μs granularity per pixel. Events have no "frame." [Mueggler et al. 2015](https://arxiv.org/abs/1502.00796) formulated event SLAM on an SE(3) B-spline trajectory because no other option was available.

Fourth, fusing a high-rate IMU with several sensors of heterogeneous frequency at once. When a system ingests a 200 Hz IMU, a 20 Hz camera, and a 10 Hz LiDAR, placing a discrete state node at every measurement time is not realistic. The moment the number of states tracks the number of measurements, the factor graph swells.

The four problems share one structure. The measurement time $t_i$ is not controlled. Observations arrive whenever, and the estimator has to know the pose at that time. The decoupling of "measurement time / estimation time / query time" is the essential advantage of a continuous-time representation.

---

## 7c.2 Parametric spline: the Furgale line

The tool Furgale, Barfoot, and Sibley chose in 2012 was the B-spline. Write the trajectory as a sum of basis functions, $\mathbf{p}(t) = \sum_k \Psi_k(t)\,\mathbf{c}_k$, and take the coefficients $\mathbf{c}_k$ as the optimization variables. The core of the B-spline is local support. At any time $t$ only a handful of bases (usually four) are nonzero; the rest are exactly zero. Querying the pose at an arbitrary time $t_i$ costs a constant, and the sparsity of the factor graph is preserved.

> 🔗 **Borrowed.** The mathematical skeleton of the B-spline is the classic [de Boor (1978) *A Practical Guide to Splines*](https://link.springer.com/book/10.1007/978-1-4612-6333-3). What Furgale did was lift that skeleton onto SE(3) and place the coefficients as variable nodes in the factor graph. It is the path by which a tool from the numerical-analysis textbook was transplanted into SLAM optimization.

The weaknesses of the form were clear. Spacing the coefficients tightly overfits; spacing them widely misses fast motion. The spacing choice depended on experience. And laying a linear B-spline directly onto SE(3) makes the interpolation result leave the manifold.

In 2013 Oxford's [Steven Lovegrove et al.](https://www.roboticsproceedings.org/rss09/p11.html) proposed the cumulative B-spline. Rearranging the basis not as a sum but as a cumulative product, $T(t) = \prod_k \exp\bigl(\tilde\Psi_k(t) \log(T_k T_{k-1}^{-1})\bigr) \cdot T_0$, closes each factor on the Lie group. This form became the native language of subsequent rolling-shutter, event-camera, and VIO papers. Basalt, the [Mueggler event SLAM](https://arxiv.org/abs/1502.00796), and [Kerl et al. 2015 dense rolling shutter VO](https://doi.org/10.1109/ICCV.2015.172) all stood on the cumulative B-spline.

The parametric spline has been used steadily in real-time VIO and event systems for its light computation and simple code. What it lacks is a natural way to layer a prior (motion prior) over the trajectory. In stretches where observations are sparse, the spline is smooth but smooth without grounds. Another branch fills that gap.

---

## 7c.3 SDE-based GP: the Barfoot line and STEAM

In the same 2014, the Barfoot group in Toronto opened a second branch. [Barfoot, Tong, and Särkkä 2014, "Batch Continuous-Time Trajectory Estimation as Exactly Sparse Gaussian Process Regression"](https://www.roboticsproceedings.org/rss10/p01.pdf) — the title was already the claim. Treat the trajectory not as a basis sum but as a Gaussian process. The prior over the trajectory is given by a kernel $\mathcal{K}(t, t')$, and when observations arrive the posterior closes as a conditional Gaussian.

The pure form of GP has one problem. For a large observation count $N$, inverting the kernel matrix $K$ costs $O(N^3)$. What Barfoot, Tong, and Särkkä showed was that a family of kernels exists for which this cost can be avoided. When the trajectory is defined as the solution of a linear time-invariant stochastic differential equation $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + L\mathbf{w}(t)$, the inverse $K^{-1}$ of its kernel $K$ has a block-tridiagonal structure. Read as a factor graph: binary factors exist only between consecutive state nodes, and none between distant nodes.

> 🔗 **Borrowed.** The frame "reinterpret the GP posterior as a prior on the factor graph" is the SDE-GP connection laid out in [Särkkä 2013 *Bayesian Filtering and Smoothing*](https://users.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf), which the Barfoot group pulled into SLAM. The Rasmussen-Williams GP textbook writes the kernel in closed form, but real-time SLAM wants a sparse inverse. Särkkä's SDE representation was the bridge.

The practical payoff is **STEAM** (Simultaneous Trajectory Estimation and Mapping). At RSS 2015, [Sean Anderson and Barfoot 2015, "Full STEAM Ahead"](https://www.roboticsproceedings.org/rss11/p45.pdf) formalized a constant-velocity-prior STEAM. Augment the state with pose $\mathbf{p}(t)$ and velocity $\mathbf{v}(t)$, and let the pose follow from the white-noise integral of velocity. That same year Anderson tightened the sparsity proof, and that piece became the backbone of every continuous-time paper out of the Barfoot group thereafter.

STEAM's second advantage was GP interpolation. Keep only a small number of control poses, and query the pose at any time between them as the posterior mean. Within one scan of a spinning LiDAR, even if 10,000 points are captured at 10,000 different instants, the control points number only one per scan. The computation scales with the number of control points, not with the number of observations.

In 2019, the release of Tang and Barfoot's [STEAM open source](https://github.com/utiasASRL/steam) gave academia and industry a directly usable library. The same year, the Dellaert group's GTSAM received a GP continuous-time factor in contrib. It was the convergence of the two paths.

---

## 7c.4 Continuous-time on the Lie group

Parametric or nonparametric, SLAM wants a trajectory on SE(3). Lifting a Euclidean spline or GP onto SE(3) is not technically simple. The common approach is to lean on the tangent space: linearly interpolate there, then lay the result back onto the manifold with the exponential map.

On the B-spline side, [Sommer, Demmel et al. 2020, "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups"](https://arxiv.org/abs/1911.08860) compiled the SE(3) cumulative-spline Jacobian in closed form. This CVPR paper supplied a standard formulation for B-spline trajectories with real-time derivatives that rolling-shutter VIO, event cameras, and visual-inertial systems could all use. Basalt and the follow-up work from the Cremers group stood on this result.

On the GP side, Anderson and Barfoot proposed a "local variable" construction. Near each control pose $T_k$ define a local perturbation $\xi_k(t) = \log(T(t)\,T_k^{-1})$, and run the GP on it. Defining a GP directly on the global manifold is hard, but on the tangent space around each control point a Euclidean GP stands. When crossing between control points, an adjoint appears, and the reason that adjoint must be there shares the root of Ch.7b preintegration's on-manifold discussion. The fact that both tools share the same Lie-group grammar became clear from 2015 on.

> 🔗 **Borrowed.** The path of transplanting a GP into a Lie-group local variable was first systematized by [Anderson-Barfoot 2015 ICRA](https://doi.org/10.1109/ICRA.2015.7138984). Their trick ("run a GP only between two consecutive control points, and correct with the adjoint when stepping across control points") every continuous-time LiDAR and VIO paper inherited thereafter.

The practical difference between spline and GP is the presence or absence of a motion prior. The spline estimates coefficients directly, with no prior. The GP carries a prior derived from an SDE, built in as constant-velocity or white-jerk and so on. Where observations are sparse, the prior fills in for the GP; the spline is filled by its neighboring observations. Attempts to combine the two (Johnson et al. 2020) have appeared, but one is chosen by application.

---

## 7c.5 The line descends to applications: LiDAR and VIO

It took ten years for the theory to come down into applications. Starting around 2022, continuous-time became a de facto standard in three arenas.

First, LiDAR motion distortion. Paris's [Pierre Dellenbach et al. 2022, "CT-ICP"](https://arxiv.org/abs/2109.12979) parameterized each scan with two poses (a "start pose" and an "end pose") and linearly interpolated between them. A simple continuous-time model, yet it beat the accuracy of prior LOAM and FAST-LIO on the KITTI, NCLT, and Newer College benchmarks. The same year, Toronto's [Keenan Burnett et al. 2022, "Are We Ready for Radar to Replace Lidar?"](https://arxiv.org/abs/2206.05432) and [STEAM-ICP](https://github.com/utiasASRL/steam_icp) applied GP-based continuous-time to the Aeva FMCW LiDAR. The Aeva sensor outputs Doppler velocity alongside each point, and that velocity maps directly to STEAM's velocity state. Without the continuous-time representation, this information had no way of being used.

Second, rolling-shutter VIO. Basalt, the [Cremers group rolling-shutter VO](https://doi.org/10.1109/CVPR.2016.71), and the follow-ups to [OKVIS](https://doi.org/10.1177/0278364914554813) query each image row's capture time on a B-spline trajectory. Unlike the earlier VIO that assumed a global shutter and routed around the problem, the rolling shutter itself is handled inside the model.

Third, event cameras. After the 2010s frustrations recorded in Ch.18, the event SLAM of the 2020s nearly all stood on continuous-time trajectories. The μs timestamp of each event is queried against a B-spline or GP to obtain the pose at that instant, and the residual is computed with event-image consistency. The fact that an event is a "frameless sensor" and that continuous-time is "a representation that needs no frame assumption" locked together naturally.

> 🔗 **Borrowed.** CT-ICP is a combination that lays intra-scan continuous-time linear interpolation on top of the point-to-plane objective of [Besl and McKay 1992 ICP](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf). Classic registration and Furgale's continuous-time spirit met inside one system, thirty years apart.

---

## 📜 Prediction vs. outcome

> In the Future Work of their 2012 IROS paper, Furgale, Barfoot, and Sibley wrote two expectations. One was that "continuous-time representation will become the natural language for unifying rolling shutter and high-rate IMU sampling"; the other was "follow-up work proving compatibility with a sparse factor graph." Both landed within a decade. Barfoot, Tong, and Särkkä 2014 closed the sparse GP proof, and the rolling-shutter VIO and event SLAM of the 2020s use the cumulative B-spline as their native language. One development the authors did not predict: in 2012 an implicit division of labor was assumed — "the discrete-keyframe-based ORB-SLAM will be the mainstream; continuous-time is for specialty sensors." What actually came out was a push in the opposite direction too. When Burnett released STEAM-ICP using the Doppler velocity of an FMCW LiDAR, continuous-time became not an appendix for handling specialty sensors but an active representation that draws out a sensor's capability. `[hit]`

---

## 🔗 Borrowed (summary)

Beyond the three boxes scattered above, one more gathering of other lines this chapter has leaned on.

Without Särkkä's SDE-GP textbook, Barfoot, Tong, and Särkkä 2014 would have had no anchor for its equations. Without de Boor's 1978 spline classic, Furgale 2012 would have had to stack basis functions from scratch. Without the local-variable technique of Anderson and Barfoot 2015, transplanting GP to the Lie group would have taken longer. Continuous-time trajectory estimation is the spot where three streams, numerical analysis, probability theory, and Lie-group differential geometry, converge on the narrow point called SLAM.

---

## 🧭 Still open

**Learning-based continuous-time prior.** The motion prior that an SDE provides embeds physical assumptions such as constant-velocity or white-jerk. Real driving, walking, and UAV trajectories often violate these assumptions. In 2023-2024, attempts appeared to learn data-driven priors with neural SDE or neural ODE and plug them into the continuous-time factor graph. A system that layers a learned prior while keeping real-time sparse structure is still at the validation stage.

**Integration of VIO and continuous-time.** Ch.7b's preintegration remains the de facto standard for keyframe-based VIO. Whether continuous-time trajectories can replace preintegration, or whether a hybrid where the two tools coexist is better, has no conclusion as of 2026. Le Gentil's [GP-augmented preintegration line](https://arxiv.org/abs/2007.04144) is trying to lay a bridge, but at the deployment-system level of ORB-SLAM3 and VINS-Fusion, discrete-time preintegration is still the workhorse.

**Online sliding window for edge deployment.** STEAM- and B-spline-based systems slow down as control points accumulate. The problem of marginalizing out past control points while preserving the consistency of the continuous-time posterior is technically tricky. If continuous-time SLAM is to be pushed as a ten-year standard on embedded platforms like cars and drones, this gap has to be filled first.

---

If Ch.7b was the engineering that squeezed out discrete-time efficiency to the end, this chapter traced a line that grew outside it — "when time must flow smoothly." The two tools do not compete. Configurations that place an IMU preintegration factor and a continuous-time LiDAR factor side by side in a single SLAM system have been reported steadily since 2024. Ch.8 picks up the visual line directly: it is where DSO and VI-DSO deploy the Forster factor, and where the direct photometric approach, independent of both preintegration supplements, comes to its own conclusions.
