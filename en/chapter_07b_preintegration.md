# Chapter 9. IMU Preintegration

Sydney, 2009. At the Australian Centre for Field Robotics (ACFR), doctoral student Todd Lupton and his advisor Salah Sukkarieh faced a timing mismatch common to agile flight. An onboard IMU streams measurements at 200 Hz—far too many to insert as individual nodes into a factor graph—while camera keyframes arrive only a few times per second. How can dozens or hundreds of inertial samples between two keyframes be compressed into a single optimization factor? Lupton's IROS paper planted the seed of preintegration. Six years later, at RSS 2015, Christian Forster, Davide Scaramuzza, Luca Carlone, and Frank Dellaert lifted that seed onto the SO(3) manifold, turning the IMU into a first-class citizen of the factor graph. Those equations now sit quietly behind the phrase "we used Forster 2016" in ORB-SLAM3, VI-DSO, and LIO-SAM (→ Chapters 8, 11, 21), whereas FAST-LIO takes the alternative filter path of propagating raw IMU readings through iterated updates.

---

## 9.1 MEMS and the "democratization of sensors"

Preintegration only became an urgent SLAM problem once IMUs became cheap enough to put on every robot.

Strapdown inertial navigation originated in 1950s aerospace, where ring-laser gyroscopes for submarines and missiles cost tens of thousands of dollars—far beyond a typical university lab's budget. MEMS (Micro-Electro-Mechanical Systems) sensors erased that barrier. Analog Devices' ADXL series and InvenSense's MPU chips drove the cost of a six-axis IMU down to a few dollars. When the iPhone added an IMU in 2007, smartphone-scale manufacturing flooded robotics with miniature inertial sensors; by the early 2010s, research drones and handheld rigs carried MEMS IMUs as standard hardware, just as Visual SLAM was confronting the monocular scale ambiguity highlighted in Chapter 5 ("Still open").

Working with these chips begins with a straightforward physical measurement model. The accelerometer reports specific force $\tilde{\mathbf{a}} = \mathbf{R}_w^b(\mathbf{a}^w - \mathbf{g}^w) + \mathbf{b}^a + \boldsymbol{\eta}^a$ (gravity included), while the gyroscope reports angular velocity $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_b^b + \mathbf{b}^g + \boldsymbol{\eta}^g$, where $\mathbf{b}$ denotes slowly drifting sensor bias and $\boldsymbol{\eta}$ denotes white noise. Three practical consequences follow immediately: gravity is always folded into the acceleration reading, forcing the estimator to track or align with the vertical axis; biases drift over time as a random walk modulated by temperature and power fluctuations; and high-frequency MEMS noise accumulates rapidly if integrated blindly.

---

## 9.2 First attempt — Lupton & Sukkarieh (2009 / 2012)

At the heart of the problem lay a mismatch of clocks in the factor graph. Smoothing backends like Kaess's iSAM2 place state nodes at keyframe timestamps, yet the IMU fires dozens of times between consecutive keyframes (→ Chapter 6). Turning every inertial sample into its own graph node blows up the state dimension; throwing samples away forfeits high-rate motion cues.

In [Visual-Inertial-Aided Navigation for High-Dynamic Motion (IROS 2009, TRO 2012)](https://doi.org/10.1109/TRO.2011.2170332), Lupton and Sukkarieh proposed integrating all IMU readings between keyframes $i$ and $j$ *once* to form a single relative motion increment. Wrapping that summary into one factor kept raw inertial samples out of the graph and gave the technique its name: "pre-integration."

Yet two limitations held back Lupton's original formulation. First, it parameterized rotation with Euler angles, exposing the integrator to gimbal lock and ignoring the curved geometry of the rotation group. Second, and more painfully, any update to the estimated IMU bias during nonlinear optimization invalidated the preintegrated increment, seemingly forcing full reintegration across hundreds of samples per keyframe. Lupton introduced a first-order bias correction to avoid redoing that work, but the Euclidean machinery could not handle 3D rotations cleanly. Six years would pass before Forster recast both the integration and the bias correction directly on SO(3).

---

## 9.3 The decisive turn — Forster-Carlone (2015 / 2017)

At RSS 2015, ETH Zürich doctoral student Christian Forster joined Davide Scaramuzza (UZH), Luca Carlone (Georgia Tech, later MIT), and Frank Dellaert (Georgia Tech, creator of GTSAM) to present [IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf), followed by the expanded journal paper [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321) in *IEEE TRO* 2017. The collaboration fused UZH's agile-drone experimentation, Georgia Tech's factor-graph framework, and Carlone's optimization rigor.

Their reformulation rested on three mathematical pillars. First, they defined the relative rotation $\Delta\mathbf{R}_{ij}$ natively on the SO(3) manifold and reformulated $\Delta\mathbf{v}_{ij}, \Delta\mathbf{p}_{ij}$ so that all three preintegrated quantities are *independent of gravity and the initial pose and velocity*. Because these quantities depend solely on raw inertial readings and bias estimates, the factor can be re-evaluated across optimization steps without repeating numerical integration, while the residual cleanly couples endpoint states, gravity, and bias updates. Second, using the right-Jacobian algebra of SO(3), they shifted white-noise perturbations to the outside of the exponential map, enabling exact analytical propagation of the preintegrated covariance $\boldsymbol{\Sigma}_{ij}$.

The third pillar made real-time bundle adjustment possible: **first-order linear bias correction on the manifold**. Rather than reintegrating hundreds of IMU samples whenever the optimizer tweaks a bias vector, the method computes partial derivatives with respect to gyroscope and accelerometer biases during the initial integration pass and applies a linear update via Jacobian-vector multiplication across all subsequent iterations. Extending Lupton's Euclidean approximation onto SO(3) turned a multi-millisecond reintegration bottleneck into a few-microsecond matrix multiply.

Once Forster's implementation landed in GTSAM as `ImuFactor`, adoption across the robotics community became effortless: subsequent SLAM systems rarely re-derived the algebra from scratch, simply linking against the factor.

> 🔗 **Borrowed.** Forster's manifold preintegration builds on the SO(3) right-Jacobian machinery later systematized in [Barfoot 2017. *State Estimation for Robotics*](https://doi.org/10.1017/9781316671528). Exponential maps and perturbation Jacobians were already standard in robotic state estimation; Forster translated IMU preintegration into that shared idiom, eliminating the singularities of Lupton's Euler-angle scheme.

> 🔗 **Borrowed.** The first-order bias Jacobian originated in [Lupton & Sukkarieh 2012](https://doi.org/10.1109/TRO.2011.2170332), a debt Forster et al. explicitly credit in TRO 2016 §VIII-B: "we follow [Lupton-Sukkarieh] but operate directly on SO(3)." Lifting that Euclidean trick onto the Lie group unlocked real-time optimization.

---

## 9.4 The three schools of practical VIO

With Forster's formulation in place, Visual-Inertial Odometry (VIO) matured along three distinct architectural schools between 2017 and 2022.

The first school comprises EKF-based filters, whose lineage predates preintegration. At ICRA 2007, Anastasios Mourikis and Stergios Roumeliotis at UC Riverside introduced the [MSCKF (Multi-State Constraint Kalman Filter)](https://doi.org/10.1109/ROBOT.2007.364024), maintaining a sliding window of past camera poses via stochastic cloning without storing 3D landmarks in the filter state; projecting linearized observation equations onto the left null space of the feature Jacobian eliminates feature-position errors cleanly. As an early, highly influential real-time VIO architecture built without preintegration, the MSCKF lineage later guided NASA JPL's Mars helicopter Ingenuity in 2021 and was modernized for research use in Guoquan Huang's open-source [OpenVINS](https://doi.org/10.1109/ICRA40945.2020.9196524) (2020).

The second school uses feature-based factor-graph optimization, exemplified by [VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729) (TRO 2018) from Tong Qin and Shaojie Shen's group at HKUST. VINS-Mono embedded Forster's preintegration factor inside tightly coupled sliding-window bundle adjustment, paired with a dedicated initialization procedure to recover metric scale and the gravity vector. Released as open-source code, it served as the field's default VIO baseline from 2019 to 2022; in Chapter 8's comparison table across the 11 EuRoC sequences, ORB-SLAM3's 0.043 m average ATE was benchmarked directly against VINS-Mono's 0.110 m.

The third school pairs preintegration with direct photometric front ends: VI-DSO (2018), Basalt (2019), and [DM-VIO (2022)](https://doi.org/10.1109/LRA.2021.3140129) from Daniel Cremers's group at TUM (→ Chapter 11). These systems layer Forster's inertial factor onto photometric bundle adjustment; DM-VIO further introduced *delayed marginalization*, holding two marginalization priors in parallel until scale and gravity become observable so that premature marginalization cannot lock early initialization errors into long-term drift.

---

## 9.5 Observability — what cannot be seen

Adding an IMU resolves monocular scale, yet Visual-Inertial systems still leave fundamental degrees of freedom unobservable.

Theoretical analyses by Huang's group beginning in the early 2010s established that an unanchored VIO estimator possesses a **four-dimensional null space**: three global translation coordinates and global yaw about the gravity axis. No combination of camera frames and inertial readings can determine absolute world position or compass heading without external anchors such as GPS or magnetometers.

By contrast, *roll and pitch* become observable because the accelerometer senses gravity's vertical pull, and inertial excitation resolves the monocular scale ambiguity left open in Chapter 5 (→ Chapter 5).

Even so, degenerate motion profiles can collapse observability in practice. While global yaw remains unobservable under any trajectory, insufficient camera parallax or lack of acceleration variation blurs the boundary between scene depth, metric scale, and accelerometer bias. Prolonged hovering or constant-velocity straight-line motion leaves those states weakly constrained; although takeoffs, braking, and turns inject useful excitation, no isolated maneuver guarantees reliable scale recovery.

> 📜 **Prediction vs. outcome.** In §IX of their 2017 TRO paper, Forster et al. highlighted three open tasks: folding online time-offset and extrinsic calibration into the estimator, testing whether the bias random-walk model holds over long missions, and generalizing preintegration to asynchronous sensors such as rolling shutters and event cameras. By 2026, online temporal and extrinsic calibration had become standard practice across Kalibr, VINS-Mono, and OpenVINS; the random-walk bias model held up well on navigation-grade IMUs while still drifting under thermal and voltage swings on consumer MEMS; and asynchronous fusion found a promising answer in Le Gentil's Gaussian-Process continuous-time preintegration. Rather than converging on a single extension, the agenda branched into three.

---

## 9.6 The branch into continuous-time

At RSS 2021, Cédric Le Gentil and his advisor Teresa Vidal-Calleja at the University of Technology Sydney (UTS)—just a few kilometers from Lupton's ACFR—returned to the same timing problem from a continuous-time perspective in [Continuous Integration over SO(3) for IMU Preintegration](https://roboticsproceedings.org/rss17/p075.pdf).

Forster's discrete formulation assumes piecewise-constant inertial measurements between samples and integrates via discrete steps. Yet when fusing asynchronous sensors such as spinning LiDARs or event cameras, the estimator must query poses at arbitrary instants *between* IMU sample timestamps. While asynchrony does not invalidate piecewise-constant integration by itself, it makes sub-sample interpolation critical. Le Gentil modeled the inertial signal as a **Gaussian Process**, treating angular velocity as a continuous function so the state can be evaluated at any timestamp $\tau$—an idea that intersects directly with the B-spline, STEAM, and GPMP continuous-time trajectory frameworks.

---

## 9.7 Borrowed Mathematical Foundations

> 🔗 **Borrowed.** The computational scaffolding that evaluates and optimizes an IMU factor comes directly from Chapter 6's [Dellaert GTSAM](https://gtsam.org/) ecosystem. Forster's `ImuFactor` plugs into GTSAM's `NoiseModelFactor` interface and coexists inside a single `Values` state container alongside visual reprojection factors—a direct inheritance of software architecture rather than mathematical equations.

> 🔗 **Borrowed.** Modeling slowly varying sensor bias as a random walk dates back to the Kalman-filter state-augmentation conventions covered in Chapter 4. Long before Lupton, navigation engineers augmented filter states with bias variables driven by small process noise; preintegration simply recast that convention as a bias random-walk factor inside a smoothing graph.

---

## Still open

**Real-time detection of visual-inertial observability.** Although the four-dimensional null space and degenerate motion regimes are well characterized in theory, runtime observability monitoring remains immature. First-Estimate Jacobian (FEJ) techniques pioneered by Hesch, Li, and Huang prevent spurious information gain along unobservable directions at the linearization point, yet as of 2026 no widely adopted runtime monitor detects entry into and exit from degenerate motion fast enough to warn a tightly coupled flight controller before estimation degrades.

**Unification of preintegration and continuous-time.** Forster's discrete increments and Le Gentil's continuous Gaussian-Process preintegration tackle the same sensor-fusion problem in different mathematical idioms. When combining frame cameras, LiDARs, and event cameras in a single estimator, choosing a unified trajectory parameterization remains an open engineering trade-off; continuous-time B-spline BA offers one bridge, yet most deployed pipelines still rely on Forster's discrete factor.

**Learning-based IMU bias models.** Whereas navigation-grade IMUs obey random-walk bias statistics cleanly, consumer MEMS units deviate substantially under temperature hysteresis and power-supply transients. Data-driven inertial odometry methods such as TLIO and RoNIN used LSTMs and Transformers to learn motion and bias corrections, and newer work models bias dynamics with conditional diffusion. How to embed learned bias priors cleanly inside Forster's analytical factor without forfeiting its theoretical guarantees remains an open question.

---

What began as Lupton's Euler-angle prototype stayed constrained for six years until Forster lifted both the integration and the bias Jacobian onto SO(3), and Le Gentil—working once again in Sydney—opened the continuous-time branch. Three generations of inertial research now sit behind a single factor line in ORB-SLAM3, VI-DSO, and LIO-SAM. Chapter 10 follows that continuous-time trajectory branch to its asynchronous sensors, while Chapter 11 returns to the visual front end where DSO and VI-DSO combine Forster's preintegration factor with direct photometric alignment.