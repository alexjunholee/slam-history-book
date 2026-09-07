# Ch.7 — Feature-based Lineage: The ORB-SLAM Trilogy

Ch.6's graph SLAM work established pose graph optimization as the standard language of SLAM. Kümmerle's g²o (2011) and Kaess's iSAM2 (2012) made iterative optimization feasible on large maps and reduced loop-closure cost to a practical level. Part 3 therefore turns to the front end. Which features should a system extract, and how should it track them?

When Klein and Murray split tracking and mapping into two threads with PTAM in 2007, the result was a lab demo that broke down beyond small indoor scenes. At the University of Zaragoza in 2015, Raúl Mur-Artal combined that structure with Rublee's ORB descriptor (2011), Gálvez-López's DBoW2 visual vocabulary (2012), and Strasdat's Essential graph idea (2011). PTAM was a fast prototype; ORB-SLAM became a ten-year standard.

---

## 7.1 ORB-SLAM (2015): A Tripod of Design Choices

[Mur-Artal, Montiel & Tardós 2015. ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671), published in *IEEE Transactions on Robotics*, describes SLAM built on ORB features. Each major component reflects a separate design choice.

The system runs three threads: Tracking, Local Mapping, and Loop Closing. PTAM had used two (Tracking and Mapping); Mur-Artal added Loop Closing as a third. This thread recognizes places with DBoW2, optimizes the pose graph over the Essential graph, and finally runs global bundle adjustment (BA). The separation keeps Tracking real-time without making it wait for map edits.

> 🔗 **Borrowed.** PTAM's Tracking–Mapping split (Klein & Murray, 2007) carried directly into ORB-SLAM's Tracking–LocalMapping structure. Mur-Artal acknowledged the debt in §3 of the paper. ORB-SLAM added a third thread and isolated loop closure as an independent module.

Mur-Artal chose the ORB (Oriented FAST and Rotated BRIEF) descriptor for specific reasons. SIFT and SURF carried patent restrictions, while BRIEF was fast but weak under rotation. ORB added rotation invariance to FAST keypoints; Rublee et al. presented it at ICCV 2011. The original experiments reported roughly two orders of magnitude, or about 100 times, higher speed than SIFT, and its binary representation allows matching by Hamming distance in real time on a CPU.

ORB obtains scale invariance from an image pyramid. The original image is shrunk by a scale factor $s$ (1.2 in ORB-SLAM) over 8 levels, and FAST keypoints are detected independently at each level. An intensity centroid defines each keypoint's orientation: the first-order moment of pixel intensity gives the patch center, and the orientation angle $\theta$ rotates the BRIEF bit-comparison pairs. The result is a 256-bit rotation-invariant descriptor. XOR followed by popcount computes the Hamming distance between two descriptors.

> 🔗 **Borrowed.** The descriptor from [Rublee et al. 2011. ORB](https://doi.org/10.1109/ICCV.2011.6126544) gave the system its name. The Zaragoza team did not design ORB; Mur-Artal assembled an existing set of tools into a pipeline. That front-end choice remained in the system's name for the next ten years.

The keyframe-selection policy departs from PTAM's. PTAM added keyframes aggressively; ORB-SLAM removes redundancy using a covisibility graph. In the **covisibility graph**, edge weights count the landmarks shared between keyframes. Two keyframes connect when they share 15 or more landmarks. Local Mapping uses this graph to select a local window and runs BA only within it.

On KITTI sequence 00 (a full 4.5 km loop), ORB-SLAM recorded 1.2% translation drift. PTAM, the comparison target at the time, could not close the large loop. Absolute scale remained ambiguous in both monocular systems. The Essential graph and DBoW2 allowed ORB-SLAM to recognize the loop and absorb the drift.

The **Essential graph** is a subgraph of the covisibility graph. It keeps only edges with 100 or more shared landmarks, the spanning tree, and the loop-closure edges. When a loop is detected the whole graph is optimized as a pose graph. Even with thousands of keyframes the edges of the Essential graph stay sparse. Optimization finishes within seconds.

> 🔗 **Borrowed.** The Essential graph idea came from the hierarchical optimization structure of [Strasdat et al. 2011. Double Window Optimisation](https://doi.org/10.1109/ICCV.2011.6126517). Strasdat separated a local window from a global window to cut optimization cost. Mur-Artal generalized this into a sparse pose graph called the Essential graph.

Place recognition for loop closure is handled by DBoW2. [Gálvez-López & Tardós 2012. DBoW2](https://doi.org/10.1109/TRO.2012.2197158) is a vocabulary tree for binary descriptors. ORB descriptors are hierarchically clustered with k-medians (k-means++ seeding) to build a tree-structured vocabulary. Once the branching factor $k_w$ and depth $L_w$ are fixed, the number of leaf nodes (words) becomes $k_w^{L_w}$. The DBoW2 paper reports an example with $k_w=10$, $L_w=6$ trained into a vocabulary of one million words, and the public ORB-SLAM implementation uses a vocabulary of similar size. Each word carries a TF-IDF (Term Frequency–Inverse Document Frequency) weight: the more frequently a given word appears across the entire keyframe database, the lower its IDF weight, so discriminative words carry more influence. A keyframe is represented by this weighted BoW vector and stored in an inverted index. When a new frame arrives, descending the vocabulary tree to determine the word takes O(log(k^L))=O(L), and the inverted index pulls up candidate keyframes directly. The whole map is never traversed.

The Tracking thread estimates the current pose in every frame. After feature matching with the previous frame, motion-only bundle adjustment refines $\mathbf{T}_{cw} \in SE(3)$. The basic reprojection objective over 3D–2D correspondences $\{(\mathbf{X}_i, \mathbf{u}_i)\}$ is:

$$\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \left\| \mathbf{u}_i - \pi(\mathbf{T}\mathbf{X}_i) \right\|^2$$

Here $\pi$ is the camera projection function, $\mathbf{X}_i$ is a map point in world coordinates, and $\mathbf{u}_i$ is its image observation. Pose optimization uses a robust loss and observation weights, holding map points fixed while changing only the current camera pose. EPnP and RANSAC supply initial pose hypotheses during relocalization. The separate Local Mapping thread performs local BA over neighboring keyframes and map points.

---

## 7.2 ORB-SLAM2 (2017) — Stereo/RGB-D

ORB-SLAM (2015) was monocular only. A single camera cannot recover scale: image pixels alone cannot distinguish a 10 m corridor from a 100 m one. Mur-Artal and Tardós returned to this problem in 2016.

[Mur-Artal & Tardós 2017. ORB-SLAM2](https://doi.org/10.1109/TRO.2017.2705103) addresses the problem by adding stereo and RGB-D. Stereo has a known baseline and triangulates depth directly; RGB-D provides a measured depth value. Both recover metric scale.

The structure is the same three threads as mono. Only the front end changes with the sensor type. Stereo extracts ORB from a rectified image pair and computes depth by left-right matching. Features matched across the pair provide **stereo observations**, while features seen in only one image provide **monocular observations**. Points with depth estimates are further classified as close or far using a threshold proportional to the baseline length.

**Stereo initialization** runs immediately from the first frame, unlike monocular initialization. The monocular mode builds a map from the Essential Matrix or Homography between two frames and retains scale ambiguity. Stereo computes depth at the first keyframe from the horizontal disparity $d$ between left and right images, the baseline $b$, and focal length $f$:

$$Z = \frac{b \cdot f}{d}$$

Feature points with depth $Z$ below the threshold $Z_{\max}=40b$ are registered as 3D map points at once. RGB-D initialization works on the same principle. The depth value $Z$ at pixel $(u, v)$ is read from the depth image, and back-projection yields the 3D coordinate. In both cases, because scale is fixed, Local BA can run right after the first frame.

On the Machine Hall 01 sequence of the EuRoC MAV (Micro Aerial Vehicle) dataset, ORB-SLAM2 (stereo) recorded an absolute translation error of 0.035 m in Table II. The same table uses Stereo LSD-SLAM as the comparison target, showing lower error for ORB-SLAM2 under those evaluation conditions. ORB-SLAM2 also ranked near the top among methods then published on KITTI odometry.

On the day in May 2017 when the paper appeared in IEEE TRO, Mur-Artal and Tardós pushed the source to GitHub alongside it. Two people in the Zaragoza team released mono, stereo, and RGB-D modes on a single codebase. GitHub stars passed several thousand afterward, and ROS wrappers came out of the community.

---

## 7.3 ORB-SLAM3 (2021): Atlas and Visual-Inertial

[Campos et al. 2021. ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644), published in IEEE Transactions on Robotics in 2021, has a different author list. The first author is not Mur-Artal but Carlos Campos. Mur-Artal is listed as a coauthor alongside Tardós. Campos had done his PhD at the University of Zaragoza under Tardós. The lineage moved down a generation.

ORB-SLAM3 added two core extensions: **Atlas** (multi-map) and **Visual-Inertial** mode.

Atlas holds several separate maps simultaneously. When tracking fails, the existing map is suspended and a new one starts; if the system later revisits the same place, it merges the maps. ORB-SLAM and ORB-SLAM2 already attempted relocalization in the existing map. When recovery failed and a new map was started, however, they lacked Atlas's ability to retain and merge separate maps. ORB-SLAM3 presents Atlas as a response to that limitation. ORB-SLAM3 reinitializes after failure while retaining the previous map.

Visual-Inertial (VI) mode integrates IMU data. Campos adopted the formulation that Forster et al. proposed at RSS 2015 as "IMU Preintegration on Manifold" and extended in IEEE TRO 2016 as [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321). The IMU bridges rapid motions that can make visual tracking fail. VI-SLAM also resolves a monocular camera's scale ambiguity: accelerometer measurements provide absolute scale together with the direction of gravity.

The IMU measurements between keyframes $i$ and $j$ are integrated once. With accelerometer and gyroscope readings modeled as $\tilde{\mathbf{a}}_t = \mathbf{a}_t + \mathbf{b}_a + \mathbf{n}_a$ and $\tilde{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t + \mathbf{b}_g + \mathbf{n}_g$, where $\mathbf{b}$ is bias and $\mathbf{n}$ is noise, the relative rotation, velocity, and position increments are

$$\Delta\mathbf{R}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp}\bigl((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\bigr)$$
$$\Delta\mathbf{v}_{ij} = \sum_{k=i}^{j-1} \Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t$$
$$\Delta\mathbf{p}_{ij} = \sum_{k=i}^{j-1}\!\left[\Delta\mathbf{v}_{ik}\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\right].$$

Here $\mathrm{Exp}(\cdot)$ is the exponential map of $\mathfrak{so}(3)$. When bias shifts during BA, a first-order Jacobian correction avoids reintegration. ORB-SLAM3 adds these preintegrated terms as inertial edges in the factor graph and optimizes them jointly with the visual reprojection residual. Ch.7b gives the full derivation, from Lupton's Euler-angle attempt to Forster's manifold formulation.

> 🔗 **Borrowed.** Campos used the Forster et al. On-Manifold Preintegration formulation (TRO 2016, originating at RSS 2015) as the core of ORB-SLAM3's inertial integration. Forster's formulas integrate continuous IMU measurements on the SO(3) manifold with bias correction. ORB-SLAM3 incorporated the formulation into factor graph optimization.

On the mean RMSE ATE (Absolute Trajectory Error) across all 11 EuRoC MAV sequences, ORB-SLAM3 (mono-inertial) is reported at 0.043 m in Table II. In the same table VINS-Mono comes in at 0.110 m, and Kimera (stereo-inertial) at 0.119 m.

Combined, VI mode and Atlas let a UAV or handheld device return to a previous map after lighting changes or lost tracking. These additions changed the character of the system, not only its version number.

---

## 7.4 Why It Is Still the Baseline in the 2020s

In 2023, conference papers still included ORB-SLAM3 in comparison tables. New methods reported how much they improved on it. Although the algorithm had been stable since 2021, its benchmark role persisted.

ORB features remain reasonably stable under illumination change, the binary descriptor is fast to compute, and a system can extract many of them in real time to reduce tracking failures. Learned features are more accurate on some datasets but can fail in new environments. ORB's behavior is more predictable.

Reproducibility also matters. The code is public, ROS integration is solid, and thousands of real-world use cases are documented. Labs routinely run ORB-SLAM3 first when evaluating a new system. Because one codebase supports mono, stereo, RGB-D, and IMU, it provides a common baseline across several settings.

The learned alternatives do not beat it consistently. DROID-SLAM (Teed & Deng, 2021) beats ORB-SLAM3 on several sequences. But as the paper itself reports, large sequences such as EuRoC and TartanAir need a 24 GB-class GPU, and on TartanAir it runs at 8 fps, not real time. ORB-SLAM3, by contrast, runs CPU-only, and community reports confirm basic operation on ARM and embedded platforms.

---

## 📜 Prediction vs. outcome

> 📜 **Prediction vs. outcome.** Mur-Artal laid out two Future Work directions in Section IX-C of the 2015 ORB-SLAM paper. "Points at Infinity" proposed using distant points that lack parallax, and therefore cannot serve as ordinary map points, to estimate rotation. "Dense Map Reconstruction" suggested that compact keyframe selection could provide a skeleton for dense reconstruction. Ten years later, VI-SLAM and follow-on work had partly absorbed the first direction. The NeRF-SLAM and Gaussian Splatting line of the 2020s revisited the second with a "sparse skeleton + dense overlay" structure in different representations. The modality extensions the authors also identified (RGB-D, stereo, IMU) appeared in ORB-SLAM2 (2017) and ORB-SLAM3 (2021) under separate problem statements.

> 📜 **Prediction vs. outcome.** In the Conclusions of the 2021 ORB-SLAM3 paper, Campos et al. identified low-texture environments as the system's main failure mode and proposed photometric techniques suited to the four data-association problems, citing endoscopic imagery as one example. Between 2023 and 2025, the community concentrated more heavily on learned front ends such as SuperPoint and LightGlue, while photometric integration continued separately in DSO and LDSO. The official ORB-SLAM3 repository still uses the traditional ORB descriptor in its main branch as of 2026. The authors' photometric direction and the community's learned-feature work diverged.

---

## 🧭 Still open

Long-term map reuse. Atlas made multi-map maintenance possible, but map merging still fails under large lighting changes. A morning map and an evening revisit should merge as the same place, yet DBoW2 misses when appearance changes substantially. Groups working on long-term outdoor autonomy across seasonal change continue to study the problem. As of 2024, there is no complete answer.

The place of the pure-vision baseline. Learned-feature systems have begun to beat ORB-SLAM3 on standard benchmarks. SuperPoint + SuperGlue, LightGlue, and DINOv2-based features show lower error on particular sequences, but generalization remains separate. Outside the training distribution, learned features sometimes perform worse than traditional ORB. Existing experiments are not broad enough to support a claim of consistent superiority.

Drift at large outdoor scale. ORB-SLAM3 still lags LiDAR SLAM on urban driving and paths beyond several kilometers. Urban-scale localization in GPS-denied environments with a pure camera remains unsolved as of 2026. When changes in visual conditions, dynamic objects, and textureless stretches combine, drift accumulates. The gap to LiDAR survey precision is narrowing but has not closed.

---

During the years when the ORB-SLAM trilogy set the feature-based standard, Newcombe and Engel took the opposite approach: use image brightness directly instead of extracting feature points. The two lineages developed side by side through the 2010s, and comparisons exposed their respective limits. ORB-SLAM3 led the EuRoC benchmark in 2021, while DSO beat ORB-SLAM2 in the TUM corridors. They followed the same timetable from different starting points.
