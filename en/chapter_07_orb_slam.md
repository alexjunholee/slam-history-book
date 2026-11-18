# Chapter 8. Feature-Based SLAM: ORB-SLAM

By the early 2010s, Chapter 6's graph SLAM lineage had established pose-graph optimization as the standard backend language. Kümmerle's g²o (2011) and Kaess's iSAM2 (2012) made iterative optimization practical on large maps and brought loop-closure costs down to interactive speeds. Part 3 turns to the front end that feeds those graphs: which visual cues should a system extract, and how should it track them?

When Klein and Murray split tracking and mapping into two parallel threads in PTAM (2007), the design produced an impressive lab prototype that nonetheless broke down beyond small indoor workspaces. Eight years later at the University of Zaragoza, Raúl Mur-Artal combined that multi-threaded architecture with Rublee's ORB descriptor (2011), Gálvez-López's DBoW2 visual vocabulary (2012), and Strasdat's Essential graph (2011). PTAM had been a nimble proof of concept; ORB-SLAM became the field's ten-year standard.

---

## 8.1 ORB-SLAM (2015): A Tripod of Design Choices

Published in *IEEE Transactions on Robotics*, [Mur-Artal, Montiel & Tardós 2015. ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671) built a complete monocular SLAM pipeline around ORB features, resting on three tightly coordinated architectural decisions: a three-thread execution split, a binary descriptor, and a hierarchical graph structure.

Where PTAM ran two threads (Tracking and Mapping), ORB-SLAM introduced Loop Closing as a third. This dedicated thread recognizes revisited places with DBoW2, optimizes the pose graph over the Essential graph, and finally triggers global bundle adjustment (BA). Decoupling loop closing from local mapping keeps the Tracking thread running at frame rate without stalling during global map corrections.

> 🔗 **Borrowed.** PTAM's Tracking–Mapping split (Klein & Murray, 2007) carried directly into ORB-SLAM's Tracking–LocalMapping structure, a debt Mur-Artal acknowledged explicitly in §3 of the paper. By adding a third thread, ORB-SLAM isolated loop closure as an independent asynchronous module.

Choosing the ORB (Oriented FAST and Rotated BRIEF) descriptor solved patent, speed, and invariance constraints simultaneously. SIFT and SURF were encumbered by patents, while BRIEF was fast but brittle under in-plane rotation. Presented by Rublee et al. at ICCV 2011, ORB augmented FAST corners with rotation invariance and achieved roughly two orders of magnitude—about 100 times—faster computation than SIFT, with binary descriptors suited to real-time Hamming-distance matching on a CPU.

To gain scale invariance as well, ORB-SLAM constructs an 8-level image pyramid with scale factor $s$ (set to 1.2) and detects FAST keypoints independently at each level. An intensity centroid defines each keypoint's orientation: the first-order moment of patch intensity gives its center of mass, and the resulting angle $\theta$ rotates the BRIEF bit-comparison pattern to produce a 256-bit rotation-invariant descriptor. Matching two descriptors then reduces to an XOR followed by a popcount instruction.

> 🔗 **Borrowed.** The descriptor from [Rublee et al. 2011. ORB](https://doi.org/10.1109/ICCV.2011.6126544) gave the system its name. Rather than inventing a new feature extractor, Mur-Artal assembled proven components into a cohesive pipeline—and that front-end choice stayed in the system's name for the next decade.

ORB-SLAM also broke with PTAM's aggressive keyframe insertion policy, pruning redundancy through a **covisibility graph** whose edge weights count shared map landmarks between keyframes. Two keyframes are linked whenever they share at least 15 landmarks, and Local Mapping uses this graph to carve out a local window for bounded BA.

On KITTI sequence 00—a full 4.5 km loop—ORB-SLAM achieved 1.2% translation drift, whereas PTAM, its primary point of comparison at the time, failed to close the large loop at all. Although absolute scale remained ambiguous in both monocular systems, DBoW2 and the Essential graph enabled ORB-SLAM to detect the revisit and distribute accumulated drift across the trajectory.

For global loop correction, ORB-SLAM relies on a sparser subgraph called the **Essential graph**, which retains only the spanning tree, loop-closure edges, and strong covisibility links sharing 100 or more landmarks. When a loop is detected, the system optimizes the full trajectory as a pose graph over this sparse skeleton; even with thousands of keyframes, the edge count stays small enough that optimization finishes within seconds.

> 🔗 **Borrowed.** The Essential graph grew out of the hierarchical optimization strategy in [Strasdat et al. 2011. Double Window Optimisation](https://doi.org/10.1109/ICCV.2011.6126517). Strasdat had separated a local metric window from a global pose-graph window to curb optimization cost; Mur-Artal generalized that idea into the Essential graph.

Place recognition for both relocalization and loop closing is handled by DBoW2. [Gálvez-López & Tardós 2012. DBoW2](https://doi.org/10.1109/TRO.2012.2197158) organizes binary ORB descriptors into a vocabulary tree via hierarchical k-medians clustering (seeded with k-means++). Fixing a branching factor $k_w$ and tree depth $L_w$ yields $k_w^{L_w}$ leaf "words"—the DBoW2 paper demonstrates $k_w=10$, $L_w=6$ for a one-million-word vocabulary, and the public ORB-SLAM release ships with a vocabulary of comparable size. Each word is weighted by TF-IDF (Term Frequency–Inverse Document Frequency) so that rare, discriminative visual words outweigh ubiquitous ones, and each keyframe's weighted BoW vector is indexed in an inverted file. When a new frame arrives, descending the tree takes O(log(k^L))=O(L) comparisons, and the inverted index retrieves candidate keyframes directly without scanning the entire map.

At every incoming frame, the Tracking thread predicts the camera motion, matches features against the prior frame and local map, and refines the pose $\mathbf{T}_{cw} \in SE(3)$ via motion-only bundle adjustment. Given 3D–2D correspondences $\{(\mathbf{X}_i, \mathbf{u}_i)\}$ between world map points $\mathbf{X}_i$ and image observations $\mathbf{u}_i$, it minimizes the reprojection error

$$\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \left\| \mathbf{u}_i - \pi(\mathbf{T}\mathbf{X}_i) \right\|^2$$

where $\pi$ is the camera projection function. In practice, this optimization wraps the residual in a robust loss with observation weights, holding map points fixed while updating only the current camera pose; if tracking is lost, EPnP inside RANSAC generates initial pose hypotheses for relocalization, while the separate Local Mapping thread refines neighboring keyframes and map points through local BA.

---

## 8.2 ORB-SLAM2 (2017) — Stereo/RGB-D

For all its robustness, ORB-SLAM (2015) remained strictly monocular, leaving absolute scale unobservable: from pixels alone, a 10 m corridor is indistinguishable from a 100 m one. Mur-Artal and Tardós closed that gap two years later.

[Mur-Artal & Tardós 2017. ORB-SLAM2](https://doi.org/10.1109/TRO.2017.2705103) extended the architecture to stereo and RGB-D cameras. A stereo rig recovers metric scale from its calibrated baseline via triangulation, while an RGB-D sensor measures depth directly.

Crucially, the three-thread backbone remained unchanged; only the front end adapted to the sensor. In stereo mode, the system extracts ORB features from rectified left–right image pairs: features matched across both views yield **stereo observations** with direct depth estimates, whereas unmatched features are retained as **monocular observations** that still constrain rotation and distant structure. Points with depth are further partitioned into close and far landmarks using a threshold proportional to the baseline.

Unlike monocular mode—which must initialize from two frames via an Essential Matrix or Homography and still inherits scale ambiguity—**stereo initialization** succeeds on the very first frame. From horizontal disparity $d$ between the left and right images, baseline $b$, and focal length $f$, depth follows immediately as

$$Z = \frac{b \cdot f}{d}$$

Points whose depth $Z$ falls below $Z_{\max}=40b$ are immediately instantiated as 3D map points. RGB-D initialization works identically by reading $Z$ at pixel $(u, v)$ from the depth map and back-projecting into 3D. With metric scale fixed from frame one, Local BA can run right away.

On the EuRoC MAV Machine Hall 01 sequence, stereo ORB-SLAM2 achieved an absolute translation error of 0.035 m in Table II—outperforming Stereo LSD-SLAM, the primary baseline in that comparison—while ranking near the top of published methods on KITTI odometry.

When the paper appeared in *IEEE Transactions on Robotics* in May 2017, Mur-Artal and Tardós simultaneously pushed the source code to GitHub. A two-person team at Zaragoza had delivered monocular, stereo, and RGB-D SLAM inside one clean codebase; thousands of GitHub stars and widespread ROS wrappers soon followed.

---

## 8.3 ORB-SLAM3 (2021): Atlas and Visual-Inertial

Published in *IEEE Transactions on Robotics* in 2021, [Campos et al. 2021. ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644) signaled a generational handoff at Zaragoza: Carlos Campos, who completed his doctorate under Tardós, took first authorship alongside Mur-Artal and Tardós.

ORB-SLAM3 introduced two major capabilities: the **Atlas** multi-map system and tightly coupled **Visual-Inertial** (VI) integration.

Atlas allows the system to maintain multiple disconnected maps concurrently. Earlier versions attempted to relocalize inside the active map when tracking broke, yet if recovery failed and a fresh map had to be initialized, the old map was lost. Atlas instead suspends the current map upon tracking failure and starts a new active map; whenever the camera later revisits previously mapped terrain, place recognition triggers a seamless merge into a unified map.

Visual-Inertial mode incorporates high-rate IMU measurements using the manifold preintegration framework that Forster et al. introduced at RSS 2015 ("IMU Preintegration on Manifold") and expanded in *IEEE TRO* 2016 as [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321). Inertial cues bridge aggressive motions where visual tracking would otherwise blur out, and accelerometer readings—together with the inferred gravity vector—resolve monocular scale ambiguity.

Rather than inserting every high-rate IMU sample into the factor graph, the system integrates all inertial measurements between consecutive keyframes $i$ and $j$ once. Modeling raw gyroscope and accelerometer readings as $\tilde{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t + \mathbf{b}_g + \mathbf{n}_g$ and $\tilde{\mathbf{a}}_t = \mathbf{a}_t + \mathbf{b}_a + \mathbf{n}_a$ (with slowly varying biases $\mathbf{b}$ and noise $\mathbf{n}$), the preintegrated rotation, velocity, and position increments take the form

$$\Delta\mathbf{R}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp}\bigl((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\bigr)$$
$$\Delta\mathbf{v}_{ij} = \sum_{k=i}^{j-1} \Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t$$
$$\Delta\mathbf{p}_{ij} = \sum_{k=i}^{j-1}\!\left[\Delta\mathbf{v}_{ik}\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\right].$$

Here $\mathrm{Exp}(\cdot)$ maps elements of the Lie algebra $\mathfrak{so}(3)$ onto the rotation group. Whenever bias estimates update during bundle adjustment, a first-order Jacobian correction adjusts these increments without repeating the integration, allowing ORB-SLAM3 to optimize inertial factors jointly with visual reprojection residuals in a single factor graph. Chapter 9 traces the full derivation of these equations from Lupton's Euler-angle prototype to Forster's manifold formulation.

> 🔗 **Borrowed.** Campos built ORB-SLAM3's inertial backend directly on Forster et al.'s On-Manifold Preintegration (TRO 2016, originating at RSS 2015), embedding SO(3) manifold integration and analytical bias-Jacobian updates straight into the visual-inertial factor graph.

In Table II of the paper, mono-inertial ORB-SLAM3 achieves a mean RMSE ATE (Absolute Trajectory Error) of 0.043 m across all 11 EuRoC MAV sequences, compared with 0.110 m for VINS-Mono and 0.119 m for stereo-inertial Kimera.

Together, VI mode and Atlas enabled a drone or handheld camera to survive lost tracking, drastic illumination shifts, and fragmented sessions while merging everything back into a consistent map—an upgrade that transformed the system's operational reach, not merely its version number.

---

## 8.4 Why It Is Still the Baseline in the 2020s

Even in 2023 and beyond, conference papers routinely kept ORB-SLAM3 in their comparison tables as the standard yardstick against which new algorithms were measured.

Part of that endurance is algorithmic predictability: ORB features tolerate moderate illumination shifts, binary descriptors match rapidly, and extracting dense sets of corners in real time keeps tracking resilient. While learned features can achieve higher accuracy on familiar benchmarks, they can also fail unpredictably out of distribution; ORB's failure modes are thoroughly understood.

Reproducibility and engineering breadth matter just as much. Open-source code, mature ROS bindings, and thousands of field deployments mean labs naturally turn to ORB-SLAM3 as their first testbed. Having monocular, stereo, RGB-D, and visual-inertial modes unified in one repository makes it a common baseline across disparate hardware setups.

Nor have learned alternatives displaced it across the board. DROID-SLAM (Teed & Deng, 2021) outperforms ORB-SLAM3 on several benchmarks, yet as its own authors report, long sequences such as EuRoC and TartanAir demand a 24 GB-class GPU, and inference on TartanAir runs at 8 fps—short of real time. ORB-SLAM3 runs entirely on a CPU and operates reliably even on ARM and embedded boards.

---

## Prediction vs. outcome

> 📜 **Prediction vs. outcome.** In Section IX-C of the 2015 ORB-SLAM paper, Mur-Artal outlined two Future Work directions: "Points at Infinity," which proposed exploiting distant zero-parallax points to constrain rotation, and "Dense Map Reconstruction," which envisioned compact keyframes serving as the geometric scaffold for dense mapping. A decade later, VI-SLAM and related formulations had largely absorbed rotation stabilization from distant cues, while the NeRF-SLAM and 3D Gaussian Splatting waves of the 2020s revived the "sparse skeleton plus dense overlay" paradigm with neural and splat representations. Meanwhile, the modality extensions noted in the paper—RGB-D, stereo, and IMU—materialized systematically across ORB-SLAM2 (2017) and ORB-SLAM3 (2021).

> 📜 **Prediction vs. outcome.** In the Conclusions of the 2021 ORB-SLAM3 paper, Campos et al. singled out low-texture environments (such as endoscopic video) as the system's primary failure mode and advocated photometric techniques for all four data-association stages. Between 2023 and 2025, however, the broader community gravitated toward learned front ends like SuperPoint and LightGlue, leaving photometric alignment largely to the DSO and LDSO lineage. As of 2026, the official ORB-SLAM3 main branch still relies on classical ORB descriptors: the authors' photometric vision and the community's learned-feature momentum diverged.

---

## Still open

**Long-term map reuse.** Although Atlas enables multi-map storage and merging, appearance-based map merging still breaks down under drastic lighting changes. A morning map and an evening revisit represent the same physical space, yet DBoW2 frequently misses the match once shadows and color balance shift. Sustaining autonomy across day–night and seasonal cycles remains unresolved as of 2024.

**The place of the pure-vision baseline.** Learned front ends such as SuperPoint + SuperGlue, LightGlue, and DINOv2-derived descriptors now surpass ORB-SLAM3 on many curated sequences, yet their generalization outside training distributions remains uneven and can fall below classical ORB. Existing benchmarks are not yet broad enough to declare a universal replacement.

**Drift at large outdoor scale.** On multi-kilometer urban drives, ORB-SLAM3 still trails LiDAR SLAM. Purely visual localization over city-scale GPS-denied routes remains unsolved as of 2026: dynamic traffic, changing illumination, and textureless road stretches compound drift, leaving a persistent precision gap relative to survey-grade LiDAR.

---

Even as the ORB-SLAM trilogy was cementing feature-based SLAM as the practical default, two major developments unfolded alongside it. First, ORB-SLAM3's jump to visual-inertial autonomy rested directly on Forster's preintegration equations—a mathematical breakthrough with its own lineage, traced next in Chapter 9 (followed by its continuous-time generalization in Chapter 10). Second, Newcombe and Engel were pursuing the exact opposite front-end philosophy in Chapter 11: discarding keypoints altogether to optimize raw image brightness directly. The two visual schools marched to the same 2010s timetable from opposite starting points—ORB-SLAM3 leading EuRoC in 2021, while DSO bested ORB-SLAM2 in texture-poor TUM corridors—and together defined the limits of geometric vision.