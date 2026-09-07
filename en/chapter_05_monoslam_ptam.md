# Ch.5 — MonoSLAM → PTAM: The Real-Time Daydream and the Split Revolution

EKF-SLAM provided a way to estimate a map together with its uncertainty, but its covariance matrix hit a structural wall that grew as $O(N^2)$ in the number of landmarks $N$. The theory was not wrong; the limit followed from the design. Davison and Klein responded in different ways.

In 2003, Davison plugged a single webcam into a laptop in an Imperial College lab. He carried over the probabilistic spatial-relations mathematics that Smith and Cheeseman had laid down in 1988 and the EKF-SLAM structure that Leonard and Durrant-Whyte had built on it, but used a single camera as the sensor. With no IMU or stereo rig, he combined the 1994 Shi-Tomasi corner detector with a Kalman predict-update loop and ran the system in real time. By the standards of the day, it was an unusual combination.

Four years later, in 2007, Klein and Murray at Oxford offered a different answer. They split tracking and mapping into two threads, an architecture that became the backbone of Visual SLAM for the next ten years.

---

## 1. The 2003 demo

At ICCV 2003, Davison's [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654) stirred the room. The individual components were familiar; seeing them work together in real time was not.

The mainstream of SLAM at the time used laser sensors. LiDAR delivered 2D ranges directly, and stereo cameras recovered depth at the pixel level. A monocular camera had no depth information to begin with. Estimating 3D structure from a single camera required at least two frames, and uncertainty in the initial depth estimate propagated through the entire EKF state vector. The theory existed; real-time implementation remained difficult.

Davison chose a monocular camera for practical reasons. An IMU meant extra hardware, and stereo carried a calibration burden. He wanted "to prove it with one camera," reasoning that other sensors could be added later. That logic held. The EKF's capacity to absorb those additions did not.

---

## 2. The beauty and the wall of the EKF

[MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049), published in IEEE PAMI in 2007 with Davison, Ian Reid, Nicholas Molton, and Olivier Stasse as co-authors, was the full account of the ICCV 2003 demo.

MonoSLAM's state vector transplanted the formulation of [Smith-Self-Cheeseman (1988)](https://arxiv.org/abs/1304.3111) and [Leonard-Durrant-Whyte (1991)](https://ieeexplore.ieee.org/document/174711/) (Ch.4) directly onto a monocular camera. It packed a camera state $\mathbf{x}_v \in \mathbb{R}^{13}$ (position 3, quaternion orientation 4, velocity 3, angular velocity 3) and landmarks $\mathbf{y}_i \in \mathbb{R}^3$ into one vector $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$. The system maintained the full $(13+3N)\times(13+3N)$ covariance $\mathbf{P}$ frame by frame in a predict-update loop. The predict step propagated the covariance through the Jacobian $\mathbf{F}$ of the camera motion model $f$ ($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$); the update step computed the Kalman gain from the Jacobian $\mathbf{H}_i$ of the projection function and refreshed the state and covariance. The EKF predict-update equations are identical to those in Ch.4 §4.3. The state vector now carried camera velocity and angular velocity along with pose because a moving camera needs a dynamics model.

The dominant cost in the covariance update $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$ came from a $(13+3N)^2$ matrix multiplication, or $O(N^2)$ in the number of landmarks $N$. §III of the paper states that "about 100" features could be sustained at 30 Hz in real time.

> 🔗 **Borrowed.** MonoSLAM's EKF state vector transplants the Smith-Cheeseman-Durrant-Whyte (1988-1991) probabilistic spatial-relations representation directly onto a monocular camera. The Kalman filter itself had existed since 1960, but Leonard and Durrant-Whyte established the practice of placing robot pose and landmarks in one "augmented state vector" in 1991.

That number exposed the system's ceiling, which the paper acknowledged by proposing a submapping strategy as future work. Building a hierarchy inside an EKF was difficult because the covariance matrix carried every correlation between every pair of landmarks, with none omitted.

MonoSLAM's choice of [Shi-Tomasi (1994)](https://doi.org/10.1109/CVPR.1994.323794) corners followed the same constraint. "Good Features to Track" selected points likely to remain trackable. Restricting the state vector to such corners made the EKF update more stable. The PAMI paper states that map management kept about 12 features stably visible per frame with a wide-angle lens. The EKF ran as long as that bounded set tracked well.

> 🔗 **Borrowed.** The Shi-Tomasi 1994 corner detector was already in use in MonoSLAM, not first in PTAM. The design philosophy of "select good features, then track them" is the direct Shi-Tomasi → MonoSLAM → PTAM lineage.

---

## 3. 2007, the same year

That bounded feature count exposed the EKF's ceiling. Klein confronted the same limit at Oxford.

In 2007, Klein and Murray presented [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852) at ISMAR. The same year, PAMI carried the finished version of Davison's MonoSLAM. Their appearance in the same year reflected a direct lineage.

Klein was then a doctoral student in Murray's group, which continued the Oxford Active Vision Laboratory where Davison had recently completed his doctorate under Murray. Klein saw in MonoSLAM not the EKF itself but proof that a monocular camera could run in real time.

The next problem was scale, and Klein discarded the EKF.

---

## 4. The split

PTAM separated tracking (camera pose tracking) from mapping (3D map construction) and ran them in two parallel threads.

In the EKF, the two tasks were coupled inside one loop. Each frame ran a predict-update cycle: predict the state when the camera moves, then update it once landmarks are found in the image.

PTAM separated the tasks. The tracking thread estimates only the camera pose in each frame. It matches the 2D projections of 3D points visible from the current keyframe set to the actual observations and computes the pose in real time. The mapping thread runs bundle adjustment whenever a new keyframe is added. Because tracking runs independently, slow mapping does not block it.

The bundle adjustment on the mapping thread minimized the sum of reprojection errors over a keyframe set $\mathcal{K}$ and a 3D point set $\mathcal{P}$:
$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$
where $\mathbf{T}_k \in SE(3)$ is the pose of keyframe $k$, $\mathbf{p}_j \in \mathbb{R}^3$ is a 3D point, $\pi$ is the camera projection function, $\mathbf{z}_{kj}$ is the observed pixel coordinate of point $j$ in keyframe $k$, $\mathbf{\Sigma}_{kj}$ is the measurement covariance, and $\rho$ is a robust kernel such as the Huber function. The mapping thread solved this optimization iteratively with Levenberg–Marquardt. Running asynchronously, it did not affect the real-time behavior of the tracking thread.

> 🔗 **Borrowed.** The bundle adjustment on PTAM's mapping thread directly applies [Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"](https://doi.org/10.1007/3-540-44480-7_21). The hundred-year photogrammetry tradition covered in Part I moved into the center of a real-time SLAM backend. Large joint updates were costly in EKF-SLAM because of covariance-matrix size; splitting the threads let keyframe BA run asynchronously from tracking.

The split had large consequences. Because the mapping thread ran bundle adjustment asynchronously, the number of landmarks in the map was no longer bound by the EKF's $O(N^2)$ constraint. PTAM used hundreds of keyframes, each holding hundreds of patch features, compared with MonoSLAM's tens of landmarks.

PTAM also changed initialization. As the user moved the camera slowly, the system estimated the essential matrix with a 5-point algorithm from the [Nistér 2004](https://doi.org/10.1109/TPAMI.2004.17) line (the PTAM paper cites the follow-up Stewénius·Engels·Nistér 2006) and recovered the initial 3D structure from the first keyframe pair. This too was borrowed.

The essential matrix $\mathbf{E}$ is a $3\times 3$ matrix capturing the pure geometric relation between two camera frames, satisfying ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$ for corresponding point pairs $(\mathbf{p}, \mathbf{p}')$. $\mathbf{E}$ decomposes internally as $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$ ($\mathbf{t}_\times$ is the skew-symmetric matrix of the translation, $\mathbf{R}$ is the rotation), so it has 5 degrees of freedom. Five point correspondences make the problem minimal, but they do not give a unique solution: the polynomial system has up to ten candidate solutions over the complex numbers. Nistér's contribution was to solve this system efficiently enough for a real-time RANSAC loop. PTAM used the solver during initialization to estimate the relative pose between the first two keyframes and triangulate the initial 3D point cloud.

> 🔗 **Borrowed.** PTAM's 5-point essential-matrix initialization follows the minimal-solver lineage opened by David Nistér's 2004 "An Efficient Solution to the Five-Point Relative Pose Problem" (the PTAM paper directly cites the follow-up Stewénius·Engels·Nistér 2006 ISPRS). Nistér's solver used the minimum number of correspondences needed to build a monocular camera's initial map. PTAM placed it inside a RANSAC loop to estimate the relative pose of the first two keyframes at near-real-time speed.

> 🔗 **Borrowed.** PTAM's keyframe structure traces back to the Leonard-Durrant-Whyte submap idea. The notion that "if the full map is hard to optimize at once, break it into regions" was expressed in PTAM as a set of keyframes. The covisibility graph of the subsequent ORB-SLAM is a more refined version of this keyframe management.

---

## 5. The diffusion of the new architecture

PTAM was designed for AR (augmented reality) workspaces; the paper's title states "Small AR Workspaces" explicitly. Because the tracking thread ran reliably in real time, it could be integrated directly into AR applications.

Commercial adoption was fast. In the early 2010s, Metaio (a German AR startup, acquired by Apple in 2015) and Qualcomm's Vuforia SDK adopted tracking/mapping split structures similar to PTAM's. These commercial SDKs helped spread stable planar AR on consumer smartphones.

The academic effect was more direct. [ORB-SLAM](https://arxiv.org/abs/1502.00956), published in 2015 by Raul Mur-Artal, J.M.M. Montiel, and Juan D. Tardós, inherited PTAM's structure. It swapped patch features for ORB descriptors, refined keyframe management with a covisibility graph, and added loop closure on top. Without PTAM, ORB-SLAM's blueprint would have been different.

Qin, Li, and Shen's [VINS-Mono](https://arxiv.org/abs/1708.03852) (2018) also uses two threads for sliding-window optimization and loop closure, extending the tracking/mapping split into Visual-Inertial Odometry (VIO).

---

## 6. Davison vs Klein & Murray — a view comparison

Two papers came out in 2007. MonoSLAM PAMI was the finished version of the 2003 demo. PTAM came out the same year, with a new structure that broke through MonoSLAM's limits.

One reason MonoSLAM retained the EKF was its explicit joint representation of state and uncertainty. The covariance matrix represented state uncertainty explicitly, tracking both the uncertainty of each landmark and the covariance between landmarks. From this viewpoint, bundle adjustment traded explicit uncertainty representation for scalability.

Klein & Murray paid that price willingly. What mattered in AR applications was real-time tracking of the camera pose. There was no need to track map uncertainty at the centimeter level. Refining the map periodically through bundle adjustment was enough.

The field accepted this trade. From the 2010s onward, graph-based optimization and bundle adjustment became mainstream, while EKF-SLAM receded except in applications with severely limited compute resources. MonoSLAM's concern with probabilistic consistency did not disappear. Rather than join the PTAM lineage directly, Davison's lab moved in stages toward factor-graph-based estimation and then Gaussian Belief Propagation (GBP) and the Robot Web. Twenty-three years later, in Ch.18 of the *SLAM Handbook*, Davison describes this trajectory as EKF → BA → factor graph → GBP. He does not name and assess MonoSLAM directly, but recasts the history around a general principle: each change of representation prompts a redesign of the system.

---

## 📜 Prediction vs. outcome

> **Davison 2007 PAMI MonoSLAM**: In the Conclusion, Davison named larger indoor and outdoor environments, faster motion, and complex scenes with occlusion and lighting changes as the next tasks. He specifically proposed a submap strategy and CMOS cameras running above 100 Hz, and suggested extending the sparse map to a dense representation of "higher-order entities" (surfaces, etc.).
>
> These predictions met different fates. Submaps, PTAM's keyframe structure, and ORB-SLAM's covisibility graph share a concern with limiting computation, but this comparison alone does not establish direct inheritance. No system, however, achieved hierarchical scaling while retaining the EKF; that hierarchy arrived with the shift to BA-based architectures. High-frame-rate cameras took concrete form through 2010s event-camera research. Robustness to dynamic scenes remains open as of 2026. DynaSLAM and FlowSLAM are among the attempts, but no solution has yet entered the baseline pipeline. Davison did not flag IMU integration directly in Future Work (though the body cites related work), and the VIO boom of the 2010s pursued that direction. The concern with probabilistic consistency survived in factor graphs and GBP. Twenty-three years later, in Handbook Ch.18, Davison discusses system redesign through changes of representation. Placing MonoSLAM within that lineage is the interpretation of this history.

> **Klein & Murray 2007 PTAM**: In §8 (Failure modes / Mapping inadequacies), Klein and Murray listed the system's limitations: corner-based tracking's vulnerability to motion blur, the geometric poverty of a point-cloud-centric map, and "not designed to close large loops in the SLAM sense." They stated plainly that global consistency across large loops was outside PTAM's design scope.
>
> In 2015 ORB-SLAM directly addressed those limitations. It added [DBoW2](http://doriangalvez.com/papers/GalvezTRO12.pdf)-based appearance loop closure and covisibility-graph-based keyframe management, and replaced patch features with ORB descriptors. ORB-SLAM took on the map-scaling task that PTAM had excluded. Klein & Murray did not explicitly identify appearance-based loop closure as the answer, but the limits they marked became the starting point for the subsequent lineage.

---

## 🧭 Still open

**Monocular scale recovery.** From MonoSLAM to PTAM, every monocular system carries scale ambiguity. A single image cannot determine absolute distance; this is a geometric fact. Adding an IMU makes scale observable through gravity direction and accelerometer readings. In pure monocular systems without an IMU, however, scale recovery remains unsolved even in 2026. Learning-based monocular depth estimation ([MiDaS](https://arxiv.org/abs/1907.01341), [Depth Anything](https://arxiv.org/abs/2401.10891)) estimates relative depth from a single image, but converting it to metric scale still requires an external reference (a ground-plane assumption, a known object size, and so on).

**Environmental generality of a single VO system.** MonoSLAM handled only indoor desktop scenes. PTAM explicitly limited its scope to "Small AR Workspaces." ORB-SLAM2 later tried to span indoor, outdoor, and RGB-D settings, but tracking still fails under extreme lighting changes or in low-texture spaces. As of 2026, no single pipeline robustly handles indoor corridors, outdoor downtowns, nighttime environments, and textureless white walls at once. Multimodal fusion (camera + LiDAR + IMU) covers some of this range, but the generality of a camera-only system remains unsettled.

**Feature tracking in low light and dynamic scenes.** MonoSLAM assumed sufficient lighting and a static scene; PTAM did the same in 2007. As of 2026, most feature-based SLAM systems still carry these assumptions implicitly. ORB features can fail entirely in low light, while scenes crowded with moving people cause dynamic points to be misclassified as static. Learning-based optical flow and semantic segmentation attempt to address the problem, but no system has become a real-time, general-purpose solution.

---

PTAM's tracking/mapping split left loop closure unresolved. As keyframes accumulated, so did error, becoming visible when the camera completed a loop. The PTAM paper itself declared loop closure, correcting accumulated error when the camera returned to a known place, out of scope. Elsewhere, graph-SLAM researchers had spent a decade preparing an answer.
