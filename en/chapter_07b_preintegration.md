# Ch.7b — From a Shaking Sensor to a Constraint: The Invention of IMU Preintegration

Sydney, 2009. At ACFR (the Australian Centre for Field Robotics), doctoral student Todd Lupton was working through a problem with his advisor, Salah Sukkarieh. When a drone moves aggressively, the IMU produces measurements at 200 Hz, too many to place individually in a factor graph. Keyframes arrive only a few times per second; how can the dozens or hundreds of IMU measurements between them become a single unit? Lupton's answer at IROS became the seed of preintegration. Six years later, at RSS 2015, Christian Forster, Davide Scaramuzza, Luca Carlone, and Frank Dellaert carried that seed onto the SO(3) manifold, making the IMU a first-class citizen of the factor graph. Those equations sit behind the line "we used Forster 2016" in Ch.7's ORB-SLAM3, Ch.8's VI-DSO, and Ch.17's LIO-SAM and FAST-LIO.

---

## 7b.1 MEMS and the "democratization of sensors"

Preintegration became necessary because IMUs became cheap.

Strapdown inertial navigation has its roots in 1950s aerospace. Ring laser gyros for submarines and missiles cost tens of thousands of dollars, beyond the reach of most robotics laboratories. MEMS (Micro-Electro-Mechanical Systems) changed that. Analog Devices' ADXL and InvenSense's MPU series reduced the price of a six-axis IMU to a few dollars. The iPhone received an IMU in 2007, and by the early 2010s research drones and handheld devices carried MEMS units as a matter of course. This price collapse, driven by billions of smartphones, coincided with Visual SLAM's growing concern with monocular scale ambiguity (Ch.5 §🧭).

The measurement model is simple. The accelerometer gives the specific force $\tilde{\mathbf{a}} = \mathbf{R}_w^b(\mathbf{a}^w - \mathbf{g}^w) + \mathbf{b}^a + \boldsymbol{\eta}^a$ with gravity included, and the gyroscope gives the angular velocity $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_b^b + \mathbf{b}^g + \boldsymbol{\eta}^g$. Here $\mathbf{b}$ is bias and $\boldsymbol{\eta}$ is white noise. Three consequences matter: gravity is always present, bias drifts slowly over time (random walk), and MEMS noise is high-frequency. An estimator therefore needs a gravity-aligned world frame and a bias model that accounts for changes in temperature and power state.

---

## 7b.2 First attempt — Lupton & Sukkarieh (2009 / 2012)

The problem was the factor graph's time axis. Kaess's iSAM2, covered in Ch.6, takes keyframe-rate poses as nodes, while the IMU produces dozens of measurements between keyframes. Making every measurement a node makes the graph unwieldy; discarding them loses information.

In [Visual-Inertial-Aided Navigation for High-Dynamic Motion (IROS 2009, TRO 2012)](https://doi.org/10.1109/TRO.2011.2170332), Lupton and Sukkarieh integrated the IMU measurements between keyframes $i$ and $j$ *once* to build a relative increment. Treating that increment as one factor kept the raw measurements out of the graph. The name "pre-integration" came from this construction.

Two obstacles limited the implementation. It represented rotation with Euler angles, which suffer from gimbal lock and do not form a manifold. Bias posed the larger problem. Every BA iteration shifts the bias estimate and therefore the increment, forcing Lupton's scheme to reintegrate the IMU sequence. Reprocessing hundreds of measurements per keyframe at every pass eroded real-time performance and delayed the idea's spread by six years.

---

## 7b.3 The decisive turn — Forster-Carlone (2015 / 2017)

At RSS 2015, Christian Forster, a doctoral student at ETH Zürich, joined Scaramuzza (UZH), Carlone (Georgia Tech, later MIT), and Dellaert (Georgia Tech, creator of GTSAM) to publish [IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf). The extended version appeared in IEEE TRO 2017 as [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321). The paper united UZH's agile-drone experiments, Georgia Tech's GTSAM factor-graph language, and Carlone's optimization theory.

They made three changes. First, they defined $\Delta\mathbf{R}_{ij}$ rigorously as a relative rotation on the SO(3) manifold and redefined $\Delta\mathbf{v}_{ij}, \Delta\mathbf{p}_{ij}$ to make them *independent of gravity and the initial state*. These are not physical increments but mathematically state-independent quantities, allowing the IMU factor to be evaluated from only the poses and velocities at its two endpoints. Second, they propagated the covariance $\boldsymbol{\Sigma}_{ij}$ analytically with a right-Jacobian construction that moves noise to the end of the exponential map.

The third change proved decisive: **linear correction via the bias first-order Jacobian**. When the bias shifts during BA, a first-order correction using precomputed partial derivatives replaces full reintegration. It is Lupton's Euclidean linearization applied on SO(3). The Jacobian is computed during the first integration between keyframes and reused through hundreds of graph-optimization iterations. A reintegration taking several milliseconds became a Jacobian-vector product taking several microseconds, allowing the IMU factor to enter real-time BA.

Adoption accelerated when Forster's implementation entered GTSAM as a reference. Later systems did not rewrite the equations; they included `ImuFactor`.

> 🔗 **Borrowed.** Forster's manifold preintegration uses the SO(3) right-Jacobian formalism later organized in [Barfoot 2017. *State Estimation for Robotics*](https://doi.org/10.1017/9781316671528). Exponential maps and Jacobians for small rotational variations were already the common language of robotic state estimation; Forster rewrote IMU preintegration in that language. Moving Lupton's Euler-angle formulation onto SO(3) removed the earlier limitation.

> 🔗 **Borrowed.** [Lupton & Sukkarieh 2012](https://doi.org/10.1109/TRO.2011.2170332) first proposed the bias first-order Jacobian. Forster et al. TRO 2016 §VIII-B acknowledges the debt explicitly: "we follow [Lupton-Sukkarieh] but operate directly on SO(3)." Moving the Euclidean approximation onto the manifold made the computation real-time.

---

## 7b.4 The three schools of practical VIO

Once Forster's formulation settled in, Visual-Inertial Odometry (VIO) systems branched into three lines between 2017 and 2022.

The first line is the filter family, whose roots precede Forster. At ICRA 2007, UC Riverside's Anastasios Mourikis and Stergios Roumeliotis introduced the [MSCKF (Multi-State Constraint Kalman Filter)](https://doi.org/10.1109/ROBOT.2007.364024). It places past camera poses in the filter state and uses stochastic cloning to marginalize observed 3D points. This was an early influential real-time visual-inertial system built on an EKF without preintegration. In 2021, NASA JPL's Mars helicopter Ingenuity used an MSCKF-family estimator on Mars. Guoquan Huang's group at the University of Delaware open-sourced [OpenVINS](https://doi.org/10.1109/ICRA40945.2020.9196524) in 2020.

The second line is the optimization family. Its representative system is [VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729), published in TRO 2018 by Shaojie Shen's HKUST group with doctoral student Tong Qin. They placed Forster's formulation as an IMU factor inside tightly coupled sliding-window BA and provided a procedure for estimating scale and gravity direction separately during initialization. Released as code, it became the conference VIO baseline from 2019 to 2022. When Ch.7's ORB-SLAM3 reported an average ATE of 0.043 m over the eleven EuRoC sequences, VINS-Mono was the comparison method at 0.110 m in the same table.

The third line is the direct family: VI-DSO (2018), Basalt (2019), and [DM-VIO (2022)](https://doi.org/10.1109/LRA.2021.3140129), covered in Ch.8. TUM's Cremers group placed Forster's inertial factor on top of DSO's photometric BA. DM-VIO added *delayed marginalization*. Premature marginalization before IMU initialization converges can lock in a wrong prior and cause long-term drift, so the method maintains two marginalization priors in parallel and merges them only after gravity and scale become observable.

---

## 7b.5 Observability — what cannot be seen

Visual-Inertial systems do not see everything.

Analyses by Huang's group from the early 2010s converged on one conclusion: the null space of an unanchored visual-inertial system is **four-dimensional**. It contains three dimensions of global position and one dimension of yaw around gravity. IMU and camera alone can never recover absolute coordinates or rotation about the gravity axis. GPS restores position; a magnetic field or external anchor restores yaw. Pure VIO cannot observe this four-dimensional subspace.

*Roll and pitch*, however, are observable because the accelerometer provides a vertical reference through gravity. Adding an IMU also resolves the monocular scale ambiguity identified in Ch.5.

Degenerate motion is harder. Under pure straight-line motion, global orientation is unobservable; under pure rotation, feature depth is unobservable; under constant acceleration, monocular scale is unobservable. VIO scale therefore wavers when a drone hovers or a car travels straight at constant speed. Practitioners see scale "lock in" during takeoff, braking, or cornering.

> 📜 **Prediction vs. outcome.** Forster et al. in TRO 2017 §IX named three directions: integrating time synchronization with online extrinsic calibration, validating the bias random-walk assumption under long-term operation, and extending to asynchronous sensors such as event cameras and rolling shutter. As of 2026, the first has been standardized as VINS-Mono, Kalibr, and OpenVINS put the time offset onto the state vector; the second holds for navigation-grade IMUs but remains affected by temperature and power-supply variation on consumer MEMS; the third has found one branch of the answer in Le Gentil's GP continuous-time preintegration. The predictions were largely on target, but instead of the single extension the authors sketched, the line split into three.

---

## 7b.6 The branch into continuous-time

At RSS 2021, Cédric Le Gentil and his advisor Teresa Vidal-Calleja at UTS (University of Technology Sydney) released [Continuous Integration over SO(3) for IMU Preintegration](https://roboticsproceedings.org/rss17/p075.pdf). Again in Sydney, only a few kilometers from Lupton's ACFR, they approached the same problem from another angle.

Forster's preintegration uses discrete time. It assumes piecewise-constant IMU measurements between samples and applies Euler integration. The assumption breaks when asynchronous sensors such as LiDAR or event cameras are fused: a LiDAR point arriving mid-scan has no unambiguous discrete bin, and interpolation error accumulates. Le Gentil instead modeled the IMU as a **Gaussian Process**, treating angular velocity as a continuous function. The state can then be evaluated at any time $\tau$, allowing asynchronous measurements to enter naturally. This direction intersects the B-spline, STEAM, and GPMP lineages.

---

## 7b.7 Terrain of borrowings

> 🔗 **Borrowed.** The skeleton that evaluates and optimizes an IMU factor on a factor graph is the [Dellaert GTSAM](https://gtsam.org/) tradition from Ch.6 unchanged. Forster's `ImuFactor` plugs into GTSAM's `NoiseModelFactor` interface and is optimized inside a single `Values` object alongside visual reprojection factors. The inheritance was of software structure, not of mathematics.

> 🔗 **Borrowed.** The practice of treating bias as a random walk comes from the Kalman-filter state-propagation convention Ch.4 recorded. Well before Lupton, the navigation community used a model that "puts the bias in the state and gives it small process noise," and in the preintegration era this was reinterpreted as the bias random-walk factor.

---

## 🧭 Still open

**Real-time detection of visual-inertial observability.** The four-dimensional null space and degenerate-motion table are theoretically settled, but runtime detection remains unfinished. The FEJ (First-Estimate Jacobian) line of Hesch, Li, and Huang preserves the null space at the linearization point, yet as of 2026 no broadly agreed method detects entry into and exit from a degenerate regime and feeds that information back to the control loop. When drone control and VIO estimation share compute resources, the controller may detect estimator failure too late.

**Unification of preintegration and continuous-time.** Forster's discrete increments and Le Gentil's continuous GP representation solve the same problem in different mathematical languages. When combining LiDAR, event cameras, and frame cameras, the representation that should underpin the estimator remains an engineering choice. B-spline continuous-time BA offers a partial answer, but most deployed systems still use Forster's discrete factor.

**Learning-based IMU bias models.** The bias random-walk assumption holds on navigation-grade IMUs, but consumer MEMS depart from it because of temperature hysteresis and power-supply transients. The TLIO and RoNIN line used LSTMs and Transformers to learn bias models for IMU-only odometry; more recent work models the bias distribution itself with conditional diffusion. How this approach fits inside a Forster factor, and how much of preintegration's mathematics remains when a learned model supplies the bias dynamics, is the next question.

---

Lupton's idea remained limited by Euler angles for six years. Forster moved it to SO(3) and introduced bias-Jacobian correction; Le Gentil, again in Sydney, extended it into continuous time. Three generations of work sit behind one line in ORB-SLAM3, one in VI-DSO, and one in LIO-SAM. Ch.7c follows the continuous-time branch, while Ch.8 continues the visual lineage with direct methods (DSO and VI-DSO) that use Forster's preintegration factor.
