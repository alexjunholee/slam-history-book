# Chapter 5. Real-Time Monocular SLAM: MonoSLAM and PTAM

EKF-SLAM gave robotics a way to estimate a map alongside its own uncertainty, yet its dense covariance matrix ran straight into an $O(N^2)$ scaling wall in the number of landmarks $N$. Nothing was wrong with the probability theory; the bottleneck was baked into the architecture. Over the span of four years, Andrew Davison and Georg Klein offered two sharply different answers to that ceiling.

In 2003, Davison—then at Oxford—achieved real-time 3D tracking of a handheld camera and its surrounding scene on an ordinary desktop PC. Carrying forward the probabilistic spatial-relations framework laid down by Smith and Cheeseman in 1988 and the EKF-SLAM architecture built on it by Leonard and Durrant-Whyte, he stripped away the laser scanners, stereo rigs, and IMUs that robotics teams usually depended on. Pairing the 1994 Shi-Tomasi corner detector with a Kalman predict-update loop on a single camera, he produced a combination that looked audacious by the standards of the day.

Four years later, in 2007, Klein and Murray at Oxford broke through the EKF wall in another way altogether: splitting pose tracking and map optimization into two parallel threads, an architectural pattern that became the backbone of Visual SLAM for the next decade.

---

## 5.1 The 2003 demo

At ICCV 2003, Davison unveiled [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654), one of the earliest live demonstrations of real-time SLAM driven solely by a single moving camera.

Mainstream SLAM in 2003 leaned heavily on range sensors. LiDAR scanners measured 2D distances directly, while stereo rigs recovered pixel-level depth through triangulation. A monocular camera starts with no depth reading at all: inferring 3D structure requires observing the scene across at least two viewpoints, and any initial depth uncertainty ripples straight through the entire EKF state vector. Although the geometry was well understood on paper, turning it into a real-time loop was notoriously fragile.

By flying solo with one camera—skipping both IMUs and multi-camera stereo calibration—MonoSLAM proved that pure monocular real-time SLAM was genuinely possible, even as the EKF's expanding covariance matrix kept the resulting map tight and small.

---

## 5.2 The beauty and the wall of the EKF

Four years after the ICCV demo, Davison, Ian Reid, Nicholas Molton, and Olivier Stasse published the complete archival account of [MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049) in IEEE TPAMI (2007).

MonoSLAM transplanted the joint-state formulation of [Smith-Self-Cheeseman (1988)](https://arxiv.org/abs/1304.3111) and [Leonard-Durrant-Whyte (1991)](https://ieeexplore.ieee.org/document/174711/) (Chapter 4) directly onto a moving monocular camera. Its augmented state vector $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$ packed the 13-dimensional camera state $\mathbf{x}_v \in \mathbb{R}^{13}$—3 for position, 4 for quaternion orientation, 3 for linear velocity, and 3 for angular velocity—together with $N$ 3D landmarks $\mathbf{y}_i \in \mathbb{R}^3$. Because a free-moving handheld camera needed a smooth dynamics model between frames, velocity and angular velocity had to ride inside the state vector alongside pose. Frame after frame, the filter maintained the full $(13+3N)\times(13+3N)$ covariance matrix $\mathbf{P}$ through the standard predict-update equations from Chapter 4 §4.3: predicting via the motion model $f$ and its Jacobian $\mathbf{F}$ ($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$), then updating with the Kalman gain derived from the projection Jacobian $\mathbf{H}_i$.

That exact covariance update, $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$, dictated the system's computational ceiling. Dominated by $(13+3N)^2$ matrix operations, its cost scaled as $O(N^2)$ with the number of landmarks $N$; in §III of the PAMI paper, the authors noted that real-time 30 Hz operation could sustain "about 100" features.

> 🔗 **Borrowed.** MonoSLAM's EKF state vector transplants the Smith-Cheeseman-Durrant-Whyte (1988-1991) probabilistic spatial-relations representation directly onto a monocular camera. While the Kalman filter dates back to 1960, Leonard and Durrant-Whyte established the convention of packing robot pose and landmarks into a single "augmented state vector" in 1991.

Even as they acknowledged that ceiling by pointing toward submapping as future work, partitioning an EKF was notoriously tricky because its dense covariance matrix explicitly linked every pair of landmarks. That same compute budget also shaped how MonoSLAM picked features: by relying on [Shi-Tomasi (1994)](https://doi.org/10.1109/CVPR.1994.323794) "Good Features to Track," the system invested its scarce state-vector slots exclusively in corners likely to stay trackable. As the PAMI paper reports, active map management kept roughly 12 features reliably in view per frame under a wide-angle lens—and as long as that tight set stayed locked on target, the filter ran smoothly.

> 🔗 **Borrowed.** The Shi-Tomasi 1994 corner detector was already at work in MonoSLAM before PTAM adopted it. The design rule of "select good features, then track them" forms a direct Shi-Tomasi → MonoSLAM → PTAM lineage.

---

## 5.3 2007, the same year

Maintaining only a dozen active features laid bare the EKF's fundamental bottleneck—and at Oxford, Georg Klein was confronting that exact same limit.

In 2007, the very year Davison's definitive MonoSLAM paper appeared in TPAMI, Klein and David Murray presented [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852) at ISMAR. That simultaneity was no coincidence; both works grew out of the same academic circle. Klein had earned his PhD under Tom Drummond at Cambridge before joining Murray—who had previously supervised Davison—as a postdoctoral researcher at Oxford. PTAM carried forward MonoSLAM's commitment to real-time monocular operation, yet made a decisive break to achieve scale: Klein threw out the EKF entirely.

---

## 5.4 The split

PTAM's defining insight was to decouple camera pose **tracking** from 3D **mapping** and execute them in two separate, parallel threads.

In EKF-SLAM, tracking and mapping were welded inside a single sequential loop: every incoming frame had to predict the camera state, match landmarks, and pay for a full joint covariance update. PTAM broke that coupling. Its **tracking thread** did one job as fast as possible—estimating only the current camera pose at frame rate by projecting already-mapped 3D points from nearby keyframes and aligning them with image observations. Meanwhile, its **mapping thread** worked asynchronously in the background, adding new keyframes whenever the camera explored fresh ground and refining the 3D map with bundle adjustment—without ever stalling the live tracker.

On the mapping thread, bundle adjustment minimized the robust sum of reprojection errors across a set of keyframes $\mathcal{K}$ and 3D map points $\mathcal{P}$:

$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$

In this objective, $\mathbf{T}_k \in SE(3)$ denotes the pose of keyframe $k$, $\mathbf{p}_j \in \mathbb{R}^3$ is a 3D landmark, $\pi$ is the camera projection function, $\mathbf{z}_{kj}$ is the observed pixel coordinate of point $j$ in keyframe $k$ with measurement covariance $\mathbf{\Sigma}_{kj}$, and $\rho$ is a robust loss kernel such as Huber. Solving this objective iteratively via Levenberg–Marquardt in a background thread meant that optimization cost never interrupted real-time camera tracking.

> 🔗 **Borrowed.** The bundle adjustment running in PTAM's mapping thread directly applies [Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"](https://doi.org/10.1007/3-540-44480-7_21). The century-old photogrammetry lineage traced in Part I had moved straight into the heart of a real-time SLAM backend: what EKF-SLAM could not afford because of its dense covariance matrix, PTAM achieved by letting keyframe BA run asynchronously from tracking.

That single architectural split transformed map scale overnight. Freed from the EKF's $O(N^2)$ ceiling, PTAM scaled effortlessly from MonoSLAM's few dozen landmarks to maps spanning hundreds of keyframes, each packed with hundreds of patch features.

Initialization underwent a similar upgrade. Rather than assuming a known calibration pattern, PTAM asked the user to translate the camera smoothly sideways, estimated the essential matrix between the first two keyframes using a 5-point solver descended from [Nistér (2004)](https://doi.org/10.1109/TPAMI.2004.17) (specifically citing the follow-up by Stewénius, Engels, and Nistér 2006), and triangulated an initial 3D point cloud.

Algebraically, the essential matrix $\mathbf{E}$ is a $3\times 3$ matrix tying normalized corresponding points $(\mathbf{p}, \mathbf{p}')$ across two views through the epipolar constraint ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$. Factoring internally as $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$—where $\mathbf{t}_\times$ is the skew-symmetric matrix of translation and $\mathbf{R}$ is rotation—$\mathbf{E}$ has five degrees of freedom. Although five point pairs suffice minimally, the resulting polynomial system yields up to ten complex candidate roots; Nistér's breakthrough was solving that polynomial fast enough to run inside a real-time RANSAC loop, giving PTAM a lightweight way to bootstrap its first keyframe pair.

> 🔗 **Borrowed.** PTAM's 5-point essential-matrix initialization follows the minimal-solver lineage opened by David Nistér's 2004 "An Efficient Solution to the Five-Point Relative Pose Problem" (the PTAM paper directly cites the follow-up Stewénius·Engels·Nistér 2006 ISPRS). Using the minimum five correspondences needed for a monocular camera's initial map, PTAM embedded the solver inside RANSAC to recover the relative pose of the first two keyframes at near-real-time speed.

> 🔀 **Structural parallel.** Leonard-Durrant-Whyte submaps and PTAM's keyframe map both keep computation bounded by working over local subsets. Yet the PTAM paper does not cite submapping as a design inspiration; its keyframes grew naturally out of separating tracking from mapping and feeding local bundle adjustment—a structure ORB-SLAM later formalized with a covisibility graph.

---

## 5.5 The diffusion of the new architecture

Designed expressly for augmented reality—as announced in its subtitle, "Small AR Workspaces"—PTAM delivered tracking stable and responsive enough to anchor virtual objects directly onto live video.

Industry picked up the pattern quickly. By the early 2010s, commercial AR platforms including Metaio (a German AR startup acquired by Apple in 2015) and Qualcomm's Vuforia SDK had adopted PTAM-style split tracking-and-mapping designs, helping bring reliable planar AR to consumer smartphones.

In academia, PTAM's blueprint proved even more foundational. When Raul Mur-Artal, J.M.M. Montiel, and Juan D. Tardós introduced [ORB-SLAM](https://arxiv.org/abs/1502.00956) in 2015, they kept PTAM's parallel-thread backbone while swapping patch features for ORB descriptors, organizing keyframes with a covisibility graph, and adding a third thread for loop closure. Three years later, Qin, Li, and Shen's [VINS-Mono](https://arxiv.org/abs/1708.03852) (2018) carried the same multi-threaded split—separating sliding-window optimization from loop closure—into visual-inertial odometry (VIO).

---

## 5.6 Davison vs Klein & Murray — a view comparison

Seen side by side in 2007, Davison's TPAMI MonoSLAM paper and Klein and Murray's ISMAR PTAM paper mark a clean fork in SLAM's evolution: MonoSLAM brought the probabilistic EKF paradigm to maturity, while PTAM broke its scaling limits with a new architecture.

Davison had a principled reason for holding onto the EKF: it maintained an explicit joint probability distribution over the entire system, tracking both the uncertainty of every landmark and the cross-correlations linking them. From that perspective, adopting bundle adjustment meant giving up explicit, continuously updated map covariance in exchange for raw scalability.

Klein and Murray happily made that trade. For augmented reality, the priority was rock-solid camera pose tracking at frame rate—not tracking landmark covariance to the centimeter—and periodic bundle adjustment kept the map accurate enough.

The broader community embraced the same trade-off. Throughout the 2010s, keyframe bundle adjustment and graph optimization swept Visual SLAM, confining EKF-SLAM largely to severely compute-starved platforms. Yet Davison's underlying insistence on consistent probabilistic inference never vanished. Instead of stepping straight onto the PTAM track, his lab evolved its representation in stages—from EKF to bundle adjustment, then to factor graphs, and onward to Gaussian Belief Propagation (GBP) and the Robot Web. Looking back twenty-three years later in Chapter 18 of the *SLAM Handbook*, Davison summarizes that arc as EKF → BA → factor graph → GBP: not as a narrow critique of MonoSLAM, but as an illustration of a broader principle that every shift in mathematical representation invites a redesign of the system around it.

---

## Prediction vs. outcome

> **Davison 2007 PAMI MonoSLAM**: In the Conclusion, Davison named larger indoor and outdoor environments, faster motion, and complex scenes with occlusion and lighting changes as the next tasks. He specifically proposed a submap strategy and CMOS cameras running above 100 Hz, and suggested extending the sparse map to a dense representation of "higher-order entities" (surfaces, etc.).
>
> These predictions met different fates. Submaps, PTAM's keyframe structure, and ORB-SLAM's covisibility graph share a concern with limiting computation, though that parallel alone does not establish direct lineage—and no system achieved hierarchical scaling while keeping the EKF intact, as hierarchy arrived only with the move to BA-based architectures. High-frame-rate sensing materialized in the 2010s through event-camera research. Meanwhile, robustness to dynamic scenes remains open as of 2026: systems such as DynaSLAM and FlowSLAM have tackled the problem, yet none has entered the standard baseline pipeline. Although Davison did not flag IMU fusion explicitly in Future Work (despite citing related work in the body), the VIO boom of the 2010s pursued that path aggressively. Above all, MonoSLAM's commitment to probabilistic consistency endured inside factor graphs and GBP—a trajectory Davison revisits in Handbook Ch.18 around how changing representations reshape systems, within which this history situates MonoSLAM.

> **Klein & Murray 2007 PTAM**: In §8 (Failure modes / Mapping inadequacies), Klein and Murray listed the system's limitations: corner-based tracking's vulnerability to motion blur, the geometric poverty of a point-cloud-centric map, and "not designed to close large loops in the SLAM sense." They stated plainly that global consistency across large loops was outside PTAM's design scope.
>
> Eight years later, ORB-SLAM (2015) tackled those exact boundaries head-on, adding [DBoW2](http://doriangalvez.com/papers/GalvezTRO12.pdf)-based appearance loop closure, covisibility-graph keyframe management, and ORB descriptors in place of raw patches to solve the large-scale mapping problem PTAM had left aside. While Klein and Murray did not prescribe appearance-based loop closure specifically, the failure modes they documented became the direct checklist for the generation that followed.

---

## Still open

**Monocular scale recovery.** Every pure monocular system from MonoSLAM to PTAM faces an unavoidable geometric ambiguity: a single camera cannot recover absolute metric scale without an external reference. Fusing an IMU resolves scale through gravity and linear acceleration, yet in vision-only monocular SLAM the problem remains open even in 2026. Learned monocular depth networks such as [MiDaS](https://arxiv.org/abs/1907.01341) and [Depth Anything](https://arxiv.org/abs/2401.10891) predict rich relative depth from a single image, but grounding those predictions in true metric units still depends on external cues such as a known ground-plane height or recognized object dimensions.

**Environmental generality of a single VO system.** MonoSLAM stayed within an indoor desk, PTAM targeted "Small AR Workspaces," and ORB-SLAM2 expanded to indoor, outdoor, and RGB-D domains—yet tracking still breaks down under harsh lighting shifts or featureless surfaces. As of 2026, no single monocular pipeline handles bright city streets, dark indoor corridors, nighttime scenes, and blank white walls with equal reliability. Multimodal rigs pairing cameras, LiDAR, and IMUs bridge much of that gap, leaving the universality of camera-only systems unresolved.

**Feature tracking in low light and dynamic scenes.** Both MonoSLAM and PTAM implicitly assumed steady illumination and a static world, assumptions most feature-based SLAM systems still lean on in 2026. ORB extraction falters in dim light, while crowded scenes full of moving people corrupt static triangulation with dynamic points. Learned optical flow and semantic segmentation help mitigate these failures, yet neither has yielded a lightweight, general-purpose real-time fix.

---

Even as PTAM's tracking-and-mapping split broke MonoSLAM's landmark limit, it left one crucial problem deliberately untouched: loop closure. As keyframes piled up along long trajectories, small drift errors accumulated until the camera returned to a familiar spot and found the map misaligned—a failure Klein and Murray had explicitly declared out of scope. Elsewhere in robotics, graph-SLAM researchers had already spent a decade forging the tool built to close that loop.
