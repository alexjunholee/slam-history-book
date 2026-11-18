# Chapter 4. EKF-SLAM

The photogrammetry, Structure from Motion, and bundle adjustment of Part I all shared a comfortable luxury: either the camera stayed still, or the system could wait until all photographs were collected and process them offline in a single batch. Hartley and Zisserman's projective geometry, RANSAC, and Levenberg-Marquardt could reconstruct a scene with extraordinary precision, yet none of them answered the question a moving robot faces at every instant: *where am I right now?* Part II begins with that real-time demand. An autonomous robot must build a map, localize itself inside it, and keep both estimates coherent as sensor uncertainty compounds—a line of probabilistic mapping that began with a short memo at SRI International.

In 1986, Randall Smith and Peter Cheeseman set out to formalize how spatial uncertainty propagates across a robot's measurements. Borrowing the mathematics of Kalman's (1960) filter, they expanded it from tracking a single state vector to maintaining an entire *network of spatial relationships*. A few years later, Hugh Durrant-Whyte in Sydney and John Leonard at MIT fused that probabilistic machinery with a new problem formulation: estimating a robot's pose while simultaneously constructing the map around it. Out of that union came the name "SLAM."

---

## 4.1 The mathematics of uncertain spatial relationships — Smith, Self, Cheeseman (1988)

At SRI International in 1986, Randall Smith, Matthew Self, and Peter Cheeseman tackled a concrete geometric question: if a robot measures frame B from frame A, and then measures frame C from frame B, how does uncertainty accumulate from A to C? Presented at UAI 1986 and published in 1988 as ["Estimating Uncertain Spatial Relationships in Robotics"](https://arxiv.org/abs/1304.3111), their paper cast spatial composition into probabilistic form.

The [Kalman filter](https://www.cs.unc.edu/~welch/kalman/kalmanPaper.html) was already well established by then, having guided radar tracking, ballistics, and satellite-orbit estimation since 1960. Smith, Self, and Cheeseman adapted Kalman's covariance-propagation equations to chains of spatial transforms. Instead of estimating the robot and the environment separately, they stacked the robot pose $\mathbf{x}_r$ and all landmark positions $\mathbf{m}_i$ into one joint state vector paired with a single joint covariance matrix $\mathbf{P}$:

$$\mathbf{x} = [\mathbf{x}_r^\top,\ \mathbf{m}_1^\top,\ \ldots,\ \mathbf{m}_N^\top]^\top$$

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rm} \\ \mathbf{P}_{mr} & \mathbf{P}_{mm} \end{bmatrix}$$

The off-diagonal block $\mathbf{P}_{rm}$ captured the central insight of probabilistic mapping: uncertainties in the robot's pose and in the landmark positions are tightly *correlated*. Only by tracking those cross-correlations explicitly could a robot update both its map and its own location consistently—a demonstration that laid the mathematical foundation for SLAM.

> 🔗 **Borrowed.** Smith-Self-Cheeseman's (1988) spatial-relationship mathematics inherits directly from Kalman's (1960) covariance propagation. A filter originally crafted to track a single moving target became a framework for jointly estimating a robot and every landmark in its map.

---

## 4.2 How the name "SLAM" settled in

Nowhere in the 1988 Smith-Self-Cheeseman paper does the word "SLAM" appear. Through the early 1990s, Hugh Durrant-Whyte—who had moved from Oxford to Sydney—and John Leonard at MIT were studying the exact same problem under different names in their respective laboratories. Once their groups began reading and citing each other's work, they needed a common label, and "SLAM" gradually caught on. Memories among researchers differ on which report printed the acronym first, and no single canonical "coining" paper exists.

Still, Leonard and Durrant-Whyte's 1991 IROS paper, ["Simultaneous Map Building and Localization for an Autonomous Mobile Robot"](https://doi.org/10.1109/IROS.1991.174711), stands out as an early milestone that placed the core idea right in its title: mapping and localization are inseparable and must be solved together. Once that phrase shortened into "Simultaneous Localization and Mapping"—SLAM—the field spent the next decade converging around it.

> 🔗 **Borrowed.** [Bar-Shalom's multi-target tracking](https://archive.org/details/trackingdataasso0000bars) (collected as a 1988 monograph) already offered a formal framework for tracking multiple objects simultaneously. Leonard and Durrant-Whyte effectively mapped "target position" to "landmark position" and "tracker position" to "robot pose," translating radar-tracking machinery into indoor mobile robotics.

---

## 4.3 The EKF-SLAM formulation

Because the Extended Kalman Filter (EKF) was already the standard tool for nonlinear state estimation well before 1988, applying it to SLAM was an obvious next step. Every cycle alternates between two stages: prediction and update.

In the predict stage, as the robot executes a control input $\mathbf{u}$, the motion model $f(\cdot)$ advances the state estimate while its Jacobian $\mathbf{F}$ propagates the covariance forward:

$$\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, \mathbf{u})$$
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$$

In the update stage, a new sensor observation $\mathbf{z}$ arrives. Using the Jacobian $\mathbf{H}$ of the observation model $h(\cdot)$, the filter computes the Kalman gain $\mathbf{K}$ and corrects both the state estimate and the joint covariance:

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top(\mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

The algebra is compact and self-contained, yet that very compactness built a hard scalability ceiling directly into EKF-SLAM.

The bottleneck comes from state dimension. A state containing one 6DOF pose and $N$ 3D landmarks has length $6 + 3N$, meaning its covariance matrix holds $(6+3N)^2$ entries—growing as $O(N^2)$. Even when each sensor measurement has fixed dimension, refreshing the entire dense covariance matrix costs $O(N^2)$ per step, and computing the Kalman gain requires multiplying the state–observation covariance by the inverse of $\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$ (whose size scales with the observation dimension). At 100 landmarks, the covariance matrix is already $306 \times 306 \approx$ 94k entries; by 1,000 landmarks, it balloons to $3006 \times 3006 \approx$ 9M entries. On an ordinary desktop computer of the early 2000s, real-time execution topped out at a few dozen to a couple hundred landmarks.

That $O(N^2)$ ceiling is precisely why [Andrew Davison's MonoSLAM (2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf) had to limit its live demos to a few dozen landmarks.

---

## 4.4 The scalability wall

When Davison unveiled real-time 3D tracking from a single handheld webcam at ICCV 2003, his map covered only a desk-sized workspace populated with a few dozen features. At a time when commercial SLAM did not yet exist, seeing a single camera run live in 3D was a breakthrough, yet the size of the covariance matrix dictated exactly how far that camera could roam.

With 100 landmarks, the state vector reaches $6 + 3 \times 100 = 306$ variables and the covariance matrix sits at $306 \times 306$; at 1,000 landmarks, it grows to $3006 \times 3006$. Every frame demands matrix updates and an inversion across that growing table. Worse still, because the EKF maintains one dense joint distribution, adding even a single new landmark immediately creates cross-correlations with every landmark already in the map, locking the update cost into quadratic growth.

Through the mid-2000s, the standard workaround was **submapping**: dividing the environment into small overlapping local regions, running an independent EKF inside each one, and linking the submaps through an outer coordination layer, as explored early on by [Chong and Kleeman (1999)](http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/integrated1/chong_feature_map.pdf). In practice, however, information lost across submap boundaries, awkward loop closures, and brittle implementation complexity kept submapping from becoming an easy general fix.

> 🔗 **Borrowed.** Chong-Kleeman's (1999) submaps and the local-window optimizations in modern SLAM share the goal of keeping computation bounded—a concern echoed in ORB-SLAM's local map and VINS-Mono's sliding window. Because their map partitioning and state marginalization mechanics differ, though, the parallel reflects a shared engineering constraint rather than direct algorithmic descent.

---

## 4.5 The consistency problem: Julier-Uhlmann's counterexample

Even if computers had been fast enough to swallow an $O(N^2)$ matrix, a deeper mathematical flaw surfaced at ICRA 2001. In a provocatively titled paper—["A Counter Example to the Theory of Simultaneous Localization and Map Building"](https://doi.org/10.1109/ROBOT.2001.933257)—Simon Julier and Jeffrey Uhlmann ran numerical experiments showing that EKF-SLAM systematically trusts its own estimates too much.

Later literature summarizes their finding as asymptotic *overconfidence*: as a trajectory lengthens, the filter's true estimation error grows while its reported covariance shrinks below the actual error. In filter terminology, the estimator becomes **inconsistent**.

Linearization error is the culprit. Because the EKF approximates nonlinear motion and observation models with first-order Taylor expansions evaluated at the current estimate, small approximation biases accumulate at every step and steadily deflate the computed covariance. Once the filter becomes overly certain that "the robot is right here," it downweights incoming sensor measurements, locking errors permanently into the map.

In 2007, [Shoudong Huang and Gamini Dissanayake](https://doi.org/10.1109/TRO.2007.903811) pinpointed the structural mechanism behind this collapse. Evaluating Jacobians at shifting state estimates breaks fundamental geometric relationships among those Jacobians, causing the estimated variance of the robot's heading angle (yaw) to collapse toward zero when physically it must remain nonzero. Subsequent observability analyses built directly on that insight: whenever the linearization point drifts, the system's observable subspace shifts with it, tricking the filter into injecting phantom certainty along unobservable directions.

> 📜 **Prediction vs. outcome.** Following Julier and Uhlmann's 2001 counterexample, researchers spent nearly a decade building more consistent filters—exploring the Unscented Kalman Filter (UKF), the Invariant EKF, and covariance-inflation schemes. By 2026, however, iterative optimization had emerged as another major answer, anchored by [iSAM](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf) (Kaess et al., 2008), [g2o](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf) (Kümmerle et al., 2011), and GTSAM, even as filter-based VIO kept advancing in parallel. Optimization can relinearize past states, yet relinearization by itself does not guarantee consistency unless unobservable directions and marginalization linearization points are also handled carefully.

---

## 4.6 FastSLAM — divide and conquer

At AAAI 2002, Michael Montemerlo, Sebastian Thrun (Stanford), Daphne Koller, and Ben Wegbreit attacked EKF-SLAM's $O(N^2)$ bottleneck from a completely different angle with [FastSLAM](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf).

Their breakthrough rested on **Rao-Blackwellization**: conditioned on a specific robot trajectory $x_{0:t}$, the positions of all landmarks become *mutually independent*. That conditional independence meant FastSLAM could represent possible robot trajectories with a particle filter—each particle carrying one candidate path—while running a tiny, independent EKF for every landmark inside each particle.

With $K$ particles and $N$ landmarks, using a KD-tree for landmark lookup brought the per-step cost down to $O(K \log N)$—logarithmic in $N$ rather than quadratic when $K$ is fixed. Because landmark EKFs inside each particle never cross-correlate, the algorithm avoids maintaining a dense $N \times N$ covariance matrix altogether. With $K$ typically set to tens or hundreds of particles, the practical speedup was dramatic.

FastSLAM handled hundreds of landmarks in real-time indoor runs and won rapid adoption, yet it traded one failure mode for another. As trajectories stretched out, **particle depletion** set in: most particles drifted onto poor paths, leaving too few effective samples to represent uncertainty properly. Reweighting particle histories during large loop closures proved brittle, and simply throwing more particles at the problem could not stop drift in expansive environments. Even after [FastSLAM 2.0](https://www.ijcai.org/Proceedings/03/Papers/165.pdf) (Montemerlo et al. 2003) sharpened the proposal distribution, filtering still hit an architectural wall—one that graph optimization would soon step right past.

---

## 4.7 The EKF's exit

Once graph-based optimization turned practical after 2005, EKF-SLAM stepped back from the center of the field. [Feng Lu and Evangelos Milios's 1997 pose-graph formulation](https://doi.org/10.1023/A:1008854305733), paired first with [Olson, Leonard, and Teller's (2006)](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf) fast solver and then with the sparse real-time factorizations of g2o, GTSAM, and iSAM2, neutralized the EKF's longtime advantage in incremental speed.

The contrast became unmistakable whenever a robot closed a loop. Returning to a previously mapped spot forces an EKF to propagate corrections across its entire dense covariance matrix at $O(N^2)$ cost. A pose graph simply inserts one new constraint edge and updates a sparse matrix factorization for a fraction of the compute.

By around 2010, designing a brand-new SLAM backend around a standard EKF had become the exception rather than the rule, surviving mainly on tightly constrained hardware or in applications requiring strict real-time filtering.

> 📜 **Prediction vs. outcome.** In their influential [2006 IEEE Robotics & Automation Magazine tutorial](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf), Durrant-Whyte and Bailey examined SLAM scalability and pointed toward submap decomposition and the information filter as the likely paths to large-scale mapping. Because the information filter (the inverse-covariance dual of the EKF) works with a sparse information matrix, researchers hoped it would stay fast as landmark counts soared. History took a different turn: sparse information filters such as SEIF accumulated approximation errors when forcing sparsity during marginalization, and submaps saw only selective adoption, while factor graphs solved by iterative optimization swept the 2010s.

---

## Still open

**Filter vs. optimization coexistence.** Losing dominance as the default SLAM backend did not erase the EKF. Even in 2026, certain autonomous-driving stacks and low-cost embedded platforms still choose sparse EKFs or UKFs because their fixed per-step compute avoids the unpredictable convergence iterations of nonlinear optimization. Which paradigm wins depends closely on the platform's latency guarantees and compute budget.

**Non-Gaussian uncertainty.** Standard EKFs assume that uncertainty fits a single Gaussian bell curve, whereas real sensors often produce heavy-tailed or multimodal errors—especially under perceptual aliasing, when two different hallways look identical and create competing pose hypotheses. Particle filters handle multimodal distributions in principle yet collapse in high-dimensional state spaces. Alternatives such as Stein particles, normalizing flows, and learned uncertainty models are under active exploration, though few have been battle-tested inside real-time SLAM as of 2026.

---

Even as EKF-SLAM ran into its practical ceiling around 100 landmarks, Andrew Davison at Imperial College used that exact tight budget to prove a remarkable point: a single camera, working entirely without laser scanners, IMUs, or stereo rigs, could track in real time. The numerical bottleneck remained, yet the system design built around it was about to change the direction of Visual SLAM.
