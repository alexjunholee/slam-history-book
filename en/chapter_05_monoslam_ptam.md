# Ch.5 — MonoSLAM → PTAM: The Real-Time Daydream and the Split Revolution

The previous chapter showed how EKF-SLAM completed a probabilistically consistent recipe for building maps, and how its covariance matrix ran into a structural wall that grew as $O(N^2)$ in the number of landmarks $N$. Not because the theory was wrong — that is just how the design was shaped. From there Davison and Klein walked in different directions.

In 2003, Davison plugged a single webcam into a laptop in an Imperial College lab. He carried over the probabilistic spatial-relations math that Smith and Cheeseman had laid down in 1988 and the EKF-SLAM scaffolding Leonard and Durrant-Whyte had stacked on top of it, but the sensor was a single camera. With no IMU and no stereo rig, he bolted on the Shi-Tomasi 1994 corner detector and a Kalman predict-update loop and ran the whole thing in real time. By the standards of the day, it was a reckless combination.

Four years later, in 2007, Klein and Murray at Oxford put forward a different answer the same year. They split tracking and mapping into two threads. That split became the backbone of Visual SLAM for the next ten years.

---

## 1. The 2003 demo

At ICCV 2003, Davison's [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654) stirred the room. Not because the result was astonishing. What was shocking was *that it was possible at all*.

The mainstream of SLAM at the time was laser sensors. LiDAR delivered 2D ranges directly, and stereo cameras recovered depth at the pixel level. A monocular camera had no depth information to begin with. Estimating 3D structure from a single camera required at least two frames, and the uncertainty of the initial depth estimate propagated through the entire EKF state vector. Possible in theory, but running it in real time was a different question.

Davison chose monocular for practical reasons. An IMU was extra hardware, and stereo carried a calibration burden. What he wanted was "to prove it with one camera." If the proof went through, the rest could be stacked on top. That logic was right. What was wrong was whether the EKF was the kind of structure that could actually take on that "rest."

---

## 2. The beauty and the wall of the EKF

[MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049), published in IEEE PAMI in 2007 with Davison, Ian Reid, Nicholas Molton, and Olivier Stasse as co-authors, was the finished-paper form of the ICCV 2003 demo.

MonoSLAM's state vector transplanted the formulation of [Smith-Cheeseman (1988)](https://arxiv.org/abs/1304.3111) and [Leonard-Durrant-Whyte (1991)](https://ieeexplore.ieee.org/document/174711/) (Ch.4) directly onto a monocular camera. A camera state $\mathbf{x}_v \in \mathbb{R}^{13}$ — position 3, quaternion orientation 4, velocity 3, angular velocity 3 — together with the set of landmarks $\mathbf{y}_i \in \mathbb{R}^3$ were packed into a single vector $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$, and the full $(13+3N)\times(13+3N)$ covariance $\mathbf{P}$ was maintained frame by frame in a predict-update loop. The predict step propagated the covariance through the Jacobian $\mathbf{F}$ of the camera motion model $f$ ($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$); the update step computed the Kalman gain from the Jacobian $\mathbf{H}_i$ of the projection function and refreshed state and covariance. The EKF predict-update equations themselves are identical to those in Ch.4 §4.3. What changed was that the state vector now carried camera velocity and angular velocity along with pose (a camera is a moving body and needs its dynamics modeled).

The dominant cost in the covariance update $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$ came from a $(13+3N)^2$ matrix multiplication — $O(N^2)$ in the number of landmarks $N$. §III of the paper states that the upper limit on the number of features sustainable at 30 Hz real-time processing was "about 100."

> 🔗 **Borrowed.** MonoSLAM's EKF state vector structure transplants the Smith-Cheeseman-Durrant-Whyte (1988-1991) probabilistic spatial-relations representation directly onto a monocular camera. The Kalman filter itself had been around since 1960, but the practice of putting robot pose and landmarks into the same "augmented state vector" was established in the Leonard-Durrant-Whyte 1991 style.

That number exposed the system's ceiling. Davison knew it. In the paper he suggested extensions to a sub-mapping strategy as a future direction. Building hierarchical structure inside an EKF was hard, though. The covariance matrix carried every correlation between every pair of landmarks, with none omitted.

Reading [Shi-Tomasi (1994)](https://doi.org/10.1109/CVPR.1994.323794) corners as MonoSLAM's visual feature of choice fits the same context. The selection criterion in "Good Features to Track" was to pick points that are good to track. If only corners unlikely to fail tracking in the first place enter the state vector, the EKF update stays more stable. The PAMI paper states that map management is configured so that about 12 features stay stably visible per frame with a wide-angle lens. As long as that bounded set of features all tracked well, the EKF ran.

> 🔗 **Borrowed.** The Shi-Tomasi 1994 corner detector was already in use in MonoSLAM, not first in PTAM. The design philosophy of "select good features, then track them" is the direct Shi-Tomasi → MonoSLAM → PTAM lineage.

---

## 3. 2007, the same year

That bounded number exposed the ceiling of the EKF. The person at Oxford looking up at that ceiling was Klein.

In 2007, Klein and Murray put [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852) into ISMAR. The same year, PAMI carried the finished version of Davison's MonoSLAM. The two papers appearing in a single year was not coincidence.

Klein was then a doctoral student in Murray's group. The Murray group was the direct descendant of the Oxford Active Vision Laboratory — the very room where, a few years earlier, Davison had been a doctoral student. Murray had been Davison's advisor. Klein could not have not seen MonoSLAM. What he saw was not the EKF but the bare fact that a monocular camera ran in real time.

The possibility had been confirmed, and what remained was "how to scale it up." Klein decided to throw out the EKF.

---

## 4. The split

PTAM's core idea was one thing. Separate tracking (camera pose tracking) and mapping (3D map construction) and run them in two parallel threads.

In the EKF the two were tangled inside the same loop. One predict-update cycle per frame: predict the state when the camera moves, then update again once landmarks are found in the image.

PTAM unwound this. The tracking thread does nothing but estimate the camera pose every frame. It matches the 2D projections of the 3D points visible from the current keyframe set against the actual observations and computes the pose in real time. The mapping thread runs bundle adjustment every time a new keyframe is added. Because the tracking thread runs independently, it did not matter if mapping was slow.

The bundle adjustment on the mapping thread minimized the sum of reprojection errors over a keyframe set $\mathcal{K}$ and a 3D point set $\mathcal{P}$:
$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$
where $\mathbf{T}_k \in SE(3)$ is the pose of keyframe $k$, $\mathbf{p}_j \in \mathbb{R}^3$ is a 3D point, $\pi$ is the camera projection function, $\mathbf{z}_{kj}$ is the observed pixel coordinate of point $j$ in keyframe $k$, $\mathbf{\Sigma}_{kj}$ is the measurement covariance, and $\rho$ is a robust kernel such as the Huber function. The mapping thread solved this optimization iteratively with Levenberg–Marquardt. Running asynchronously, it did not affect the real-time behavior of the tracking thread.

> 🔗 **Borrowed.** The bundle adjustment run on PTAM's mapping thread is a direct application of [Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"](https://doi.org/10.1007/3-540-44480-7_21). The hundred-year photogrammetry tradition covered in Part I finally took its proper place in a SLAM backend here. In the EKF, full BA had been impossible because of the size limits of the covariance matrix. Splitting threads lifted that limit.

The split looks simple, but the outcome was different. Because the mapping thread ran bundle adjustment asynchronously, the number of landmarks that could enter the map stepped outside the EKF's $O(N^2)$ constraint. PTAM used keyframes in the hundreds. Each keyframe held hundreds of patch features. That was a different world from MonoSLAM's tens of landmarks.

The initial map-building method was different too. In PTAM's initialization, with the user slowly moving the camera, the system estimated the essential matrix with a 5-point algorithm along the [Nistér 2004](https://doi.org/10.1109/TPAMI.2004.17) line (the PTAM paper cites the follow-up Stewénius·Engels·Nistér 2006) and recovered the initial 3D structure from the first keyframe pair. That too was borrowed.

The essential matrix $\mathbf{E}$ is a $3\times 3$ matrix capturing the pure geometric relation between two camera frames, satisfying ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$ for corresponding point pairs $(\mathbf{p}, \mathbf{p}')$. $\mathbf{E}$ decomposes internally as $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$ ($\mathbf{t}_\times$ is the skew-symmetric matrix of the translation, $\mathbf{R}$ is the rotation), so it has 5 degrees of freedom. Five point correspondences therefore suffice to obtain a unique solution (up to ten real solutions). Nistér's contribution was to solve this 5-point system efficiently using a Gröbner basis, making it fast enough to run inside a RANSAC loop in real time. PTAM used this solver together with RANSAC during initialization to estimate the relative pose between the first two keyframes and triangulated the initial 3D point cloud.

> 🔗 **Borrowed.** PTAM's 5-point essential-matrix initialization follows the minimal-solver lineage opened by David Nistér 2004 "An Efficient Solution to the Five-Point Relative Pose Problem" (the PTAM paper directly cites the follow-up Stewénius·Engels·Nistér 2006 ISPRS). The 5-point solver was a minimal solver that used the smallest number of correspondences needed to build a monocular camera's initial map, and PTAM put this solver inside a RANSAC loop to estimate the relative pose of the first two keyframes at near-real-time speed.

> 🔗 **Borrowed.** PTAM's keyframe structure traces back to the Leonard-Durrant-Whyte submap idea. The notion that "if the full map is hard to optimize at once, break it into regions" was expressed in PTAM as a set of keyframes. The covisibility graph of the subsequent ORB-SLAM is a more refined version of this keyframe management.

---

## 5. The diffusion of the new architecture

PTAM was designed for AR (augmented reality) workspaces. The paper's title states "Small AR Workspaces" explicitly. The tracking thread had good reproducibility and solid real-time behavior, so it could be dropped into AR applications directly.

Commercial absorption was fast. In the early 2010s Metaio (a German AR startup, acquired by Apple in 2015) and Qualcomm's Vuforia SDK adopted a tracking/mapping split structure similar to PTAM's. Stable planar AR ran on consumer smartphones for the first time.

The academic effect was more direct. [ORB-SLAM](https://arxiv.org/abs/1502.00956), published in 2015 by Raul Mur-Artal, J.M.M. Montiel, and Juan D. Tardós, inherited PTAM's structure. It swapped patch features for ORB descriptors, refined keyframe management with a covisibility graph, and added loop closure on top. Without PTAM, ORB-SLAM's blueprint would have been different.

Qin, Li, and Shen's [VINS-Mono](https://arxiv.org/abs/1708.03852) (2018) also has a two-threaded structure of sliding-window optimization plus loop closure. A case of the tracking/mapping-split lineage extending into Visual-Inertial Odometry (VIO).

---

## 6. Davison vs Klein & Murray — a view comparison

Two papers came out in 2007. MonoSLAM PAMI was the finished version of the 2003 demo. PTAM came out the same year, with a new structure that broke through MonoSLAM's limits.

The reason MonoSLAM stayed with the EKF lay in probabilistic consistency. The EKF managed state uncertainty explicitly through the covariance matrix. The math tracked how uncertain each landmark in the map was and how covariances between landmarks were connected. From this viewpoint, bundle adjustment was a least-squares optimization — a trade that gave up uncertainty representation in exchange for scalability.

Klein & Murray paid that price willingly. What mattered in AR applications was real-time tracking of the camera pose. There was no need to track map uncertainty at the centimeter level. Refining the map periodically through bundle adjustment was enough.

After that, the field tilted toward this trade. From the 2010s on, graph-based optimization and bundle adjustment became mainstream, and EKF-SLAM mostly stepped back from the front except in applications where compute resources are extremely limited. The probabilistic concern MonoSLAM held on to did not vanish, however. Instead of jumping across into the PTAM lineage, Davison's lab shifted direction over several steps toward factor-graph-based estimation and then toward Gaussian Belief Propagation (GBP) and the Robot Web. Twenty-three years later in SLAM Handbook Ch.18, Davison reads the same flow as a sequence of representation changes going EKF → BA → factor graph → GBP. There is no passage where he names MonoSLAM and evaluates it directly; he recasts the whole thing into the general principle that each change of representation triggers a redesign of the system.

---

## 📜 Prediction vs. outcome

> **Davison 2007 PAMI MonoSLAM**: In the Conclusion, Davison named larger indoor and outdoor environments, faster motion, and complex scenes with occlusion and lighting changes as the next tasks. As concrete means he raised a sub-map strategy and CMOS cameras running above 100 Hz, and he also mentioned room for extending the sparse map to a dense representation of "higher-order entities" (surfaces, etc.).
>
> These predictions met different fates. The sub-map idea was partially absorbed via PTAM's keyframe structure and ORB-SLAM's covisibility graph. No system, however, achieved hierarchical scaling while keeping the EKF — the layering arrived together with the shift to BA-based architectures. High-frame-rate cameras took concrete form along a different path in 2010s event-camera research. Robustness to dynamic scenes is still open as of 2026. Attempts like DynaSLAM and FlowSLAM exist, but no "solution built into the baseline pipeline" has appeared yet. IMU integration was not flagged directly by Davison in Future Work (though related work is referenced in the body), and the 2010s VIO research boom took that direction. The probabilistic-consistency concern itself was not scrapped but migrated toward factor graphs and GBP — twenty-three years on, in Handbook Ch.18, Davison himself describes this migration as "a sequence of representation changes" and relocates MonoSLAM as one step in the lineage. `[in progress]`

> **Klein & Murray 2007 PTAM**: In §8 (Failure modes / Mapping inadequacies), Klein and Murray listed the system's limitations: corner-based tracking's vulnerability to motion blur, the geometric poverty of a point-cloud-centric map, and "not designed to close large loops in the SLAM sense." They stated plainly that global consistency across large loops was outside PTAM's design scope.
>
> In 2015 ORB-SLAM aimed squarely at those limitations. [DBoW2](http://doriangalvez.com/papers/GalvezTRO12.pdf)-based appearance loop closure and covisibility-graph-based keyframe management were added, and the features were swapped from patches to ORB descriptors. Where PTAM drew a line saying "not our problem," ORB-SLAM picked up the map-scaling task. Klein & Murray themselves did not write that "appearance-based loop closure is the answer" in so many words, but their flagging of the limit points landed exactly as the starting point of the subsequent lineage. `[partial hit]`

---

## 🧭 Still open

**Monocular scale recovery.** From MonoSLAM to PTAM, every monocular system carries scale ambiguity. That absolute distance cannot be known from a single image is a geometric fact. Add an IMU and scale becomes observable through gravity direction and accelerometer readings. In pure monocular systems without an IMU, however, scale recovery remains unsolved even in 2026. Learning-based monocular depth estimation ([MiDaS](https://arxiv.org/abs/1907.01341), [Depth Anything](https://arxiv.org/abs/2401.10891)) estimates relative depth from a single image, but converting it to metric scale still requires an external reference (a ground-plane assumption, a known object size, and so on).

**Environmental generality of a single VO system.** MonoSLAM handled only indoor desktop scenes. PTAM self-limited its scope to "Small AR Workspaces." ORB-SLAM2 later tried to span indoor, outdoor, and RGB-D, but tracking failure still occurs in scenes with extreme lighting changes or low-texture spaces. As of 2026 no single pipeline robustly handles indoor corridors, outdoor downtowns, nighttime environments, and textureless white walls all at once. Multi-modal fusion (camera + LiDAR + IMU) covers some of this, but the generality of a camera-only system is still unsettled.

**Feature tracking in low light and dynamic scenes.** What MonoSLAM required was sufficient lighting and a static scene. PTAM in 2007 was the same. As of 2026 these two assumptions still hold implicitly in most feature-based SLAM systems. ORB features fail to detect at all in low light, and in scenes crowded with moving people, dynamic points get misclassified as static. Attempts to route around the problem with learning-based optical flow or semantic segmentation exist, but no system has settled in as a real-time, general-purpose solution.

---

What PTAM established as a tracking/mapping split did not solve one thing. As keyframes piled up, the accumulated error blew up at loops. The map grew; the drift grew with it. Closing a loop — correcting the accumulated error when the camera returned to a known place — was declared out of scope in the PTAM paper itself. That problem had been in preparation elsewhere for a decade.
