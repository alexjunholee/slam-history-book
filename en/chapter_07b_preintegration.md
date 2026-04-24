# Ch.7b — From a Shaking Sensor to a Constraint: The Invention of IMU Preintegration

Sydney, 2009. At ACFR (the Australian Centre for Field Robotics), a doctoral student named Todd Lupton was working through a problem in front of his advisor Salah Sukkarieh. When a drone moves aggressively, the IMU spits out measurements at 200Hz, and a factor graph has no place to put all of them. Keyframes arrive a few times per second; how do you bundle the dozens or hundreds of IMU measurements that fall between them into a single unit? The answer Lupton submitted to IROS was the seed of preintegration. Six years later, at RSS 2015, Christian Forster, together with Davide Scaramuzza, Luca Carlone, and Frank Dellaert, carried that seed onto the SO(3) manifold — and the IMU became a first-class citizen of the factor graph. The stage of this chapter is the inside of the equations that Ch.7's ORB-SLAM3, Ch.8's VI-DSO, and Ch.17's LIO-SAM and FAST-LIO dispatch with the line "we used Forster 2016."

---

## 7b.1 MEMS and the "democratization of sensors"

Preintegration became necessary because IMUs became cheap.

Strapdown inertial navigation has its roots in 1950s aerospace. Submarine and missile ring laser gyros were tens-of-thousands-of-dollars hardware, and the robotics community had no occasion to use them. What shifted the current was MEMS (Micro-Electro-Mechanical Systems). Analog Devices' ADXL and InvenSense's MPU series pulled a six-axis IMU down to a few dollars. The iPhone got an IMU in 2007, and by the early 2010s research drones and handheld devices carried a MEMS IMU as a matter of course. The moment when billions of smartphones drove the unit price down overlapped with the moment when Visual SLAM started taking monocular scale ambiguity (Ch.5 §🧭) seriously.

The measurement model is simple. The accelerometer gives the specific force $\tilde{\mathbf{a}} = \mathbf{R}_w^b(\mathbf{a}^w - \mathbf{g}^w) + \mathbf{b}^a + \boldsymbol{\eta}^a$ with gravity baked in, and the gyroscope gives the angular velocity $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_b^b + \mathbf{b}^g + \boldsymbol{\eta}^g$. Here $\mathbf{b}$ is bias and $\boldsymbol{\eta}$ is white noise. The facts the equations forced were more important than the equations themselves. Gravity is always mixed in, bias drifts slowly with time (random walk), and MEMS noise is high-frequency. The IMU was a fussy companion that forced a gravity-aligned world frame and whose bias changed a little with every shift in temperature and power state.

---

## 7b.2 First attempt — Lupton & Sukkarieh (2009 / 2012)

The problem was the time axis of the factor graph. Kaess's iSAM2, which Ch.6 covered, takes keyframe-rate poses as nodes. But the IMU throws dozens of measurements between keyframes. Making every measurement a node blows up the graph; discarding them loses information.

The answer from Lupton and Sukkarieh's [Visual-Inertial-Aided Navigation for High-Dynamic Motion (IROS 2009, TRO 2012)](https://doi.org/10.1109/TRO.2011.2170332) was a detour. Numerically integrate the IMU measurements between keyframes $i$ and $j$ *once* to build a relative increment. Treat that increment as a single factor, and the raw IMU measurements never need to enter the graph. The name "pre-integration" came from here.

The idea was right, but the implementation had two obstacles. The rotation representation was Euler angles — which have gimbal lock and are not a manifold. The more crippling issue was bias. Every time BA runs, the bias estimate shifts, and when the bias shifts, so does the increment. Lupton's scheme required re-integrating the IMU sequence at every BA iteration. The cost of redoing hundreds of measurements per keyframe at every pass cut into real time. That limitation delayed the idea's spread by six years.

---

## 7b.3 The decisive turn — Forster-Carlone (2015 / 2017)

At RSS 2015, Christian Forster, a doctoral student at ETH Zürich, together with Scaramuzza (UZH), Carlone (Georgia Tech, later MIT), and Dellaert (Georgia Tech, creator of GTSAM), released [IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf). The extended version appeared in IEEE TRO 2017 as [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321). The author list itself is a lineage. UZH's agile drone experiments, Georgia Tech's GTSAM factor-graph language, and Carlone's optimization theory met in one paper.

There were three redefinitions. First, they defined $\Delta\mathbf{R}_{ij}$ rigorously as a relative rotation on the SO(3) manifold, and redefined $\Delta\mathbf{v}_{ij}, \Delta\mathbf{p}_{ij}$ so as to be *independent of gravity and the initial state*. These quantities are not physical increments but quantities constructed to be state-independent in the math. Because of this, the IMU factor could be evaluated knowing only the poses and velocities at the two ends. Second, they propagated the covariance $\boldsymbol{\Sigma}_{ij}$ analytically with a right-Jacobian trick that pushes noise to the tail of the exponential map.

The decisive move was the third. **Linear correction via the bias first-order Jacobian.** When the bias shifts a little during BA iteration, do not re-integrate the whole increment — apply a first-order correction using precomputed partial derivatives. The same idea as Lupton's Euclidean linearization, but operating on SO(3). Compute it once when you first integrate between keyframes, and no matter how many hundreds of times graph optimization iterates, the Jacobian does not have to be touched again. A re-integration of several milliseconds shrank to a Jacobian-vector product of several microseconds. This was where the IMU factor entered real-time BA.

The final wedge was Forster's implementation landing in GTSAM as a reference. Later systems did not rewrite the equations. They `#include`d `ImuFactor`.

> 🔗 **Borrowed.** Forster's manifold preintegration uses, as is, the SO(3) right-Jacobian apparatus organized by [Barfoot 2017. *State Estimation for Robotics*](https://doi.org/10.1017/9781316671528). The Lie-group calculation that substitutes small rotational variations with exponential maps and Jacobians was the common language of state estimation in robotics, and Forster rewrote IMU preintegration in this language. Where Lupton had been stuck in Euler angles, the same problem, moved into the SO(3) dialect, came loose.

> 🔗 **Borrowed.** The bias first-order Jacobian idea itself was first put forward by [Lupton & Sukkarieh 2012](https://doi.org/10.1109/TRO.2011.2170332). Forster et al. TRO 2016 §VIII-B acknowledges the debt explicitly: "we follow [Lupton-Sukkarieh] but operate directly on SO(3)." Carrying the Euclidean approximation onto the manifold, the same math became real-time.

---

## 7b.4 The three schools of practical VIO

Once Forster's formulation settled in, Visual-Inertial Odometry (VIO) systems branched into three lines between 2017 and 2022.

The first line is the filter family, and its root goes back further than Forster. The starting point was UC Riverside's Anastasios Mourikis and Stergios Roumeliotis at ICRA 2007 with [MSCKF (Multi-State Constraint Kalman Filter)](https://doi.org/10.1109/ROBOT.2007.364024). The scheme puts past camera poses on the filter state and uses stochastic cloning to marginalize out observed 3D points. It was the first case of running Visual-Inertial in real time on an EKF skeleton without preintegration. In 2021, the estimator that NASA JPL's Mars helicopter Ingenuity ran on Mars was from the MSCKF family. Guoquan Huang's group at the University of Delaware open-sourced [OpenVINS](https://doi.org/10.1109/ICRA40945.2020.9196524) in 2020.

The second line is the optimization family. Its signature work is HKUST's Shaojie Shen group and his doctoral student Tong Qin with [VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729) in TRO 2018. They took Forster's formulation as is, planted it as an IMU factor inside a sliding-window tightly-coupled BA, and laid out a procedure for separately estimating scale and gravity direction during initialization. The code was released, and from 2019 to 2022 it became the VIO baseline at conferences. When Ch.7's ORB-SLAM3 was reported at an average ATE of 0.043m over the eleven EuRoC sequences, the side compared at 0.110m in the same table was VINS-Mono.

The third line is the direct family. VI-DSO (2018), Basalt (2019), and [DM-VIO (2022)](https://doi.org/10.1109/LRA.2021.3140129), covered in Ch.8, belong here. TUM's Cremers group stacked Forster's inertial factor on top of DSO's photometric BA. DM-VIO added *delayed marginalization*. If you marginalize rashly before IMU initialization converges, a wrong prior locks in and causes long-term drift; the method maintains two marginalization priors in parallel and merges them into the final prior after gravity and scale are observed.

---

## 7b.5 Observability — what cannot be seen

Visual-Inertial systems do not see everything.

The analyses Huang's group organized from the early 2010s converged on one conclusion. The null space of a clueless visual-inertial system is **four-dimensional**. Three dimensions of global position and one dimension of yaw around gravity. Absolute coordinates and rotation about the gravity axis cannot ever be known from IMU and camera alone. Add GPS, and position comes back; add a magnetic field or external anchor, and yaw comes back. Pure VIO structurally cannot see this four-dimensional subspace.

Interesting is that *roll and pitch are visible*. The accelerometer reads the horizontal through gravity. This is also where the monocular scale ambiguity Ch.5 pointed out gets resolved by attaching an IMU.

The trickier side is degenerate motion. Under pure straight-line motion, global orientation is unobservable; under pure rotation, feature depth is unobservable; under constant acceleration, monocular scale in its entirety is unobservable. This is why VIO scale wavers when a drone hovers or a car goes straight at constant speed. Practitioners know it from experience. At the moment of takeoff, the moment of braking, the moment of cornering, scale "locks in."

> 📜 **Prediction vs. outcome.** Forster et al. in TRO 2017 §IX named three directions: integrating time synchronization with online extrinsic calibration, validating the bias random-walk assumption under long-term operation, and extending to asynchronous sensors such as event cameras and rolling shutter. As of 2026, the first has been standardized as VINS-Mono, Kalibr, and OpenVINS put the time offset onto the state vector; the second holds for navigation-grade IMUs but remains affected by temperature and power-supply variation on consumer MEMS; the third has found one branch of the answer in Le Gentil's GP continuous-time preintegration. The predictions were largely on target, but instead of the single extension the authors sketched, the line split into three. `[partial hit + diverted]`

---

## 7b.6 The branch into continuous-time

At RSS 2021, Cédric Le Gentil and his advisor Teresa Vidal-Calleja at UTS (University of Technology Sydney) in Sydney released [Continuous Integration over SO(3) for IMU Preintegration](https://roboticsproceedings.org/rss17/p075.pdf). Sydney again. A few kilometers from Lupton's ACFR, the same problem was looked at again from a different angle.

Forster's preintegration is discrete. It assumes the IMU measurements are piecewise-constant between samples and does Euler integration. The assumption breaks when asynchronous sensors such as LiDAR or event cameras get mixed in. It is ambiguous which discrete bin to attach a LiDAR point that arrived in the middle of a scan, and interpolation error accumulates. Le Gentil's answer was to model the IMU as a **Gaussian Process**, treating the angular velocity as a continuous function. The state can be evaluated at an arbitrary time $\tau$, so asynchronous measurements enter naturally. This direction, which meets the B-spline, STEAM, and GPMP lineages, deserves its own separate lineage.

---

## 7b.7 Terrain of borrowings

> 🔗 **Borrowed.** The skeleton that evaluates and optimizes an IMU factor on a factor graph is the [Dellaert GTSAM](https://gtsam.org/) tradition from Ch.6 unchanged. Forster's `ImuFactor` plugs into GTSAM's `NoiseModelFactor` interface and is optimized inside a single `Values` object alongside visual reprojection factors. The inheritance was of software structure, not of mathematics.

> 🔗 **Borrowed.** The practice of treating bias as a random walk comes from the Kalman-filter state-propagation convention Ch.4 recorded. Well before Lupton, the navigation community used a model that "puts the bias in the state and gives it small process noise," and in the preintegration era this was reinterpreted as the bias random-walk factor.

---

## 🧭 Still open

**Real-time detection of visual-inertial observability.** The four-dimensional null space and the table of degenerate motions are theoretically settled, but the mechanism by which a running system decides "I am currently in a degenerate regime" is unfinished. The FEJ (First-Estimate Jacobian) line of Hesch, Li, and Huang preserves the null space at the linearization point, but as of 2026 there is no standard method to catch the start and end of degenerate conditions at runtime and feed them back into the control loop. In systems where drone control and VIO estimation run on the same CPU, this gap leads to actual crashes.

**Unification of preintegration and continuous-time.** Forster's discrete increments and Le Gentil's GP continuous representation solve the same problem in different mathematical languages. When mixing LiDAR, event, and frame cameras, which representation to lay at the bottom is still a matter of engineering choice. B-spline continuous-time BA has given a partial answer, but most deployed systems still use Forster's discrete factor.

**Learning-based IMU bias models.** The bias random-walk assumption holds on navigation-grade IMUs, but on consumer MEMS it drifts because of temperature hysteresis and power-supply transients. The TLIO and RoNIN line learned the bias of IMU-only odometry with LSTMs and Transformers, and more recently attempts have appeared that model the bias distribution itself with conditional diffusion. How this approach fits inside a Forster factor, and how much of preintegration's mathematical elegance will hold up once learning is doing the propping, is the next question.

---

The idea Lupton started in Sydney sat for six years against the wall of Euler angles; Forster moved it to SO(3) and picked the lock of the bias Jacobian; Le Gentil, in Sydney again, branched it into continuous time. Three generations of work are stacked behind one line in ORB-SLAM3, one line in VI-DSO, one line in LIO-SAM. The continuous-time branch that Le Gentil opened is the subject of Ch.7c; readers following the main visual lineage continue at Ch.8, where the direct methods (DSO and VI-DSO) receive the preintegration factor from Forster's hand.
