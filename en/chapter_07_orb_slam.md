# Ch.7 — Feature-based Lineage: The ORB-SLAM Trilogy

Ch.6's graph SLAM revolution locked in pose graph optimization as the standard language of SLAM. Kümmerle's g²o (2011) and Kaess's iSAM2 (2012) made iterative optimization feasible on large maps and brought the cost of loop closure down to a practical level. That current, with optimization theory racing toward completion, is the starting line of Part 3. Where Part 2 asked "how do we reduce error," Part 3 begins with the premise that the question already has an answer. What remained was the front end. Which features do we extract, and how do we track them.

When Klein and Murray split tracking and mapping into two threads with PTAM in 2007, it was a lab demo. The idea had been proven in depth, not breadth, and it broke down beyond small indoor scenes. When Raúl Mur-Artal carried that structure over at the University of Zaragoza in 2015, he brought three things along. Rublee's ORB descriptor (2011), Gálvez-López's DBoW2 visual vocabulary (2012), and Strasdat's Essential graph idea (2011). PTAM was a fast prototype; ORB-SLAM was a ten-year standard.

---

## 7.1 ORB-SLAM (2015): A Tripod of Design Choices

[Mur-Artal, Montiel & Tardós 2015. ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671) is a paper in IEEE Transactions on Robotics. The title is plain. SLAM that uses ORB features. Pull the paper apart, though, and every choice is a design judgment.

The system's skeleton is three threads: Tracking, Local Mapping, Loop Closing. PTAM too had two threads (Tracking and Mapping). Mur-Artal added Loop Closing as a third thread. Loop Closing recognizes places with DBoW2, optimizes the pose graph through the Essential graph, and finally runs a global bundle adjustment (BA). This separation lets Tracking hold real time without waiting for map edits.

> 🔗 **Borrowed.** The Tracking–Mapping split of PTAM (Klein & Murray, 2007) carried directly into ORB-SLAM's Tracking–LocalMapping. Mur-Artal named the debt in §3 of the paper. ORB-SLAM extended the two-thread structure into three threads and isolated loop closure as an independent module.

Mur-Artal had a reason for picking the ORB (Oriented FAST and Rotated BRIEF) descriptor at the front end. SIFT and SURF came with patent trouble, and BRIEF was fast but weak under rotation. ORB adds rotation invariance on top of FAST keypoints, and Rublee et al. presented it at ICCV 2011. Computation is tens of times faster than SIFT, and since it is binary, matching runs on Hamming distance. It runs in real time on CPU.

The way ORB gets scale invariance is an image pyramid. The original image is shrunk by a scale factor s (1.2 in ORB-SLAM) over 8 levels to build a pyramid, and FAST keypoints are detected independently at each level. The orientation of a keypoint is defined by the intensity centroid: the first-order moment of pixel intensity inside the patch gives the center, and the orientation angle θ is applied to the BRIEF bit-comparison pairs to produce a rotation-invariant descriptor. The result is a 256-bit binary vector. Similarity between two descriptors is computed by XOR followed by popcount, that is, Hamming distance.

> 🔗 **Borrowed.** The descriptor from [Rublee et al. 2011. ORB](https://doi.org/10.1109/ICCV.2011.6126544) became the name of the system itself. ORB was not designed by the Zaragoza team. Mur-Artal took the tools that existed and assembled a pipeline. It is a case where a front-end choice named the system for the next ten years.

The keyframe selection policy departs from PTAM's. PTAM added keyframes aggressively. ORB-SLAM removes redundancy based on a covisibility graph. The **covisibility graph** is a graph whose edge weights are the number of landmarks shared between keyframes. A pair of keyframes connects when they share 15 or more landmarks. Local Mapping uses this graph to pick a local window and runs BA only inside that window.

On KITTI sequence 00 (a full 4.5 km loop) ORB-SLAM recorded 1.2% translation drift. PTAM, the comparison target at the time, cannot close the loop. It has no scale to begin with. That ORB-SLAM closed the loop on the same sequence and absorbed the drift was thanks to the Essential graph and DBoW2.

The **Essential graph** is a subgraph of the covisibility graph. It keeps only edges with 100 or more shared landmarks, the spanning tree, and the loop-closure edges. When a loop is detected the whole graph is optimized as a pose graph. Even with thousands of keyframes the edges of the Essential graph stay sparse. Optimization finishes within seconds.

> 🔗 **Borrowed.** The Essential graph idea came from the hierarchical optimization structure of [Strasdat et al. 2011. Double Window Optimisation](https://doi.org/10.1109/ICCV.2011.6126517). Strasdat separated a local window from a global window to cut optimization cost. Mur-Artal generalized this into a sparse pose graph called the Essential graph.

Place recognition for loop closure is handled by DBoW2. [Gálvez-López & Tardós 2012. DBoW2](https://doi.org/10.1109/TRO.2012.2197158) is a vocabulary tree for binary descriptors. ORB descriptors are hierarchically clustered with k-medians (k-means++ seeding) to build a tree-structured vocabulary. Once the branching factor $k_w$ and depth $L_w$ are fixed, the number of leaf nodes (words) becomes $k_w^{L_w}$. The DBoW2 paper reports an example with $k_w=10$, $L_w=6$ trained into a vocabulary of one million words, and the public ORB-SLAM implementation uses a vocabulary of similar size. Each word carries a TF-IDF (Term Frequency–Inverse Document Frequency) weight: the more frequently a given word appears across the entire keyframe database, the lower its IDF weight, so discriminative words carry more influence. A keyframe is represented by this weighted BoW vector and stored in an inverted index. When a new frame arrives, descending the vocabulary tree to determine the word takes O(log(k^L))=O(L), and the inverted index pulls up candidate keyframes directly. The whole map is never traversed.

The Tracking thread estimates the current pose every frame. With the previous frame's pose as an initial value, feature matching runs, and then **EPnP** (Efficient Perspective-n-Point) computes the pose $\mathbf{T}_{cw} \in SE(3)$. EPnP minimizes reprojection error over 3D–2D correspondences $\{(\mathbf{X}_i, \mathbf{u}_i)\}$:

$$\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \left\| \mathbf{u}_i - \pi(\mathbf{T}\mathbf{X}_i) \right\|^2$$

Here $\pi$ is the camera projection function, $\mathbf{X}_i$ is the world coordinate of a map point, and $\mathbf{u}_i$ is the image coordinate. After the initial estimate, RANSAC removes outliers, and a g²o-based local BA on inliers alone jointly optimizes the pose of the current keyframe and its covisibility-graph neighbors, along with the map points.

---

## 7.2 ORB-SLAM2 (2017) — Stereo/RGB-D

ORB-SLAM (2015) was mono-only. A single camera cannot know scale. There is no way to read out of image pixels whether "this corridor is 10 m or 100 m." That was the problem that sent Mur-Artal and Tardós back to work in 2016.

[Mur-Artal & Tardós 2017. ORB-SLAM2](https://doi.org/10.1109/TRO.2017.2705103) solves the problem by adding stereo and RGB-D. Stereo knows the baseline, so depth is triangulated directly. RGB-D has a depth sensor giving a measurement. In both cases scale arrives.

The structure is the same three threads as mono. Only the front end changes with the sensor type. Stereo extracts ORB from a rectified image pair and computes depth by left-right matching. Feature points near the baseline are classified as **stereo landmarks**, while far points where depth estimation is impossible are classified as **monocular landmarks**. This mixed approach uses the strengths of stereo and mono at once.

**Stereo initialization**, unlike mono, runs immediately from the first frame. Mono initialization builds the map through the Essential Matrix or Homography between two frames and leaves scale ambiguity behind. Stereo computes depth at the first keyframe from the horizontal disparity $d$ between left and right images together with the baseline $b$ and the focal length $f$:

$$Z = \frac{b \cdot f}{d}$$

Feature points with depth $Z$ below the threshold $Z_{\max}=40b$ are registered as 3D map points at once. RGB-D initialization works on the same principle. The depth value $Z$ at pixel $(u, v)$ is read from the depth image, and back-projection yields the 3D coordinate. In both cases, because scale is fixed, Local BA can run right after the first frame.

On the Machine Hall 01 sequence of the EuRoC MAV (Micro Aerial Vehicle) dataset, ORB-SLAM2 (stereo) recorded an absolute translation error of 0.035 m in Table II. The same table has Stereo LSD-SLAM as the comparison target, so the precision advantage of the feature-based line at the time shows up as numbers.  ORB-SLAM2 also ranked near the top among then-published methods on KITTI odometry.

On the day in May 2017 when the paper appeared in IEEE TRO, Mur-Artal and Tardós pushed the source to GitHub alongside it. Two people in the Zaragoza team released mono, stereo, and RGB-D modes on a single codebase. GitHub stars passed several thousand afterward, and ROS wrappers came out of the community.

---

## 7.3 ORB-SLAM3 (2021): Atlas and Visual-Inertial

[Campos et al. 2021. ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644), published in IEEE Transactions on Robotics in 2021, has a different author list. The first author is not Mur-Artal but Carlos Campos. Mur-Artal is listed as a coauthor alongside Tardós. Campos had done his PhD at the University of Zaragoza under Tardós. The lineage moved down a generation.

The core extensions of ORB-SLAM3 are two. **Atlas** (multi-map) and **Visual-Inertial** mode.

Atlas is a structure that holds several separate maps simultaneously. When tracking fails, the existing map is closed and a new map is started; later, when the same place is revisited, the two maps are merged. In ORB-SLAM and ORB-SLAM2, a tracking failure was fatal. Lose it once and you started over. Campos pointed to this as the limitation he had run into most often through his PhD, and Atlas was the answer. ORB-SLAM3 re-initializes after a failure and remembers the previous map.

Visual-Inertial (VI) mode integrates IMU data. Campos carried over the formulation that Forster et al. proposed at RSS 2015 as "IMU Preintegration on Manifold" and extended in IEEE TRO 2016 as [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321). The IMU fills in the tracking that Visual SLAM tends to lose in fast motion. VI-SLAM also resolves the scale ambiguity of a monocular camera. The accelerometer measurements of the IMU provide absolute scale together with the direction of gravity.

The core of preintegration is that IMU measurements between keyframes $i$ and $j$ are integrated once and stored as relative increments ($\Delta\mathbf{R}_{ij}$, $\Delta\mathbf{v}_{ij}$, $\Delta\mathbf{p}_{ij}$) on the SO(3) manifold. When bias shifts during BA, a first-order Jacobian correction is applied without re-integration. ORB-SLAM3 adds these preintegrated terms as inertial edges in the factor graph and optimizes them jointly with the visual reprojection residual. (Ch.7b traces the full derivation — from Lupton's Euler-angle attempt to Forster's manifold formulation — for readers who want the mathematics behind this single line of ORB-SLAM3.)

> 🔗 **Borrowed.** Campos took the Forster et al. On-Manifold Preintegration formulation (TRO 2016, originating at RSS 2015) as the core of the ORB-SLAM3 Inertial integration. Forster's formulas give a way to integrate continuous IMU measurements on the SO(3) manifold with bias correction. ORB-SLAM3 hooked this formulation into factor graph optimization.

On the mean RMSE ATE (Absolute Trajectory Error) across all 11 EuRoC MAV sequences, ORB-SLAM3 (mono-inertial) is reported at 0.043 m in Table II. In the same table VINS-Mono comes in at 0.110 m, and Kimera (stereo-inertial) at 0.119 m.

When VI mode and Atlas are combined, a UAV or a handheld device can return to a previous map even through lighting changes or lost tracking. It was not a version number change so much as a change in the character of the system.

---

## 7.4 Why It Is Still the Baseline in the 2020s

In 2023, conference papers still put ORB-SLAM3 in the comparison table. When a new method is announced, "how much better than ORB-SLAM3" is the reference line. The algorithm stopped moving in 2021 and the benchmark role did not.

Robustness comes first. ORB features hold up reasonably under illumination change, the binary descriptor is fast to compute, and many of them can be extracted in real time to reduce tracking failures. Learned features are more accurate on certain datasets, but can fall apart in new environments. The behavior of ORB is predictable.

Reproducibility is there too. The code is public, ROS integration is solid, and thousands of real-world use cases are documented. Running ORB-SLAM3 as the first step when evaluating a new system in a lab has been standard practice for a long time. Because a single codebase supports mono, stereo, RGB-D, and IMU, one can line up "our method vs ORB-SLAM3 (stereo)" or "our method vs ORB-SLAM3 (mono-inertial)" in parallel. One baseline covers several settings.

The learned alternatives do not beat it consistently. DROID-SLAM (Teed & Deng, 2021) beats ORB-SLAM3 on several sequences. But as the paper itself reports, large sequences such as EuRoC and TartanAir need a 24 GB-class GPU, and on TartanAir it runs at 8 fps, not real time. ORB-SLAM3, by contrast, runs CPU-only, and community reports confirm basic operation on ARM and embedded platforms.

---

## 📜 Prediction vs. outcome

> 📜 **Prediction vs. outcome.** Mur-Artal laid out two Future Work directions in Section IX-C of the 2015 ORB-SLAM paper. One was "Points at Infinity," the idea of using distant points that lack parallax, and so cannot be folded in as ordinary map points, for rotation estimation. The other was "Dense Map Reconstruction," a suggestion that a compact keyframe selection would provide a good skeleton for dense reconstruction. Looking back ten years later, the first direction was partly absorbed by VI-SLAM and follow-on work, and the second direction ended up realized by the NeRF-SLAM and Gaussian Splatting line of the 2020s, which implement a "sparse skeleton + dense overlay" structure with different materials. The follow-on modality extensions the authors pointed to — RGB-D, stereo, IMU — were added not in this Section but in ORB-SLAM2 (2017) and ORB-SLAM3 (2021), under separate problem statements. `[partial hit]`

> 📜 **Prediction vs. outcome.** Campos et al. in the Conclusions of the 2021 ORB-SLAM3 paper admitted that the main failure mode of ORB-SLAM3 is low-texture environments, and pointed to the development of photometric techniques suited to the four data association problems as the next direction (citing endoscopic imagery as one example). Between 2023 and 2025 the more prominent current ran ahead of that direction: research that transplanted learned front ends such as SuperPoint and LightGlue into ORB-SLAM3, while integration of the photometric line continued separately in the DSO and LDSO context. The main branch of the official ORB-SLAM3 repository still holds the traditional ORB descriptor as of 2026. The authors' central axis (photometric) and the actual attention of the community (learned feature) rolled on past each other. `[in progress + diverted]`

---

## 🧭 Still open

Long-term map reuse. Atlas made multi-map maintenance possible, but in environments with large lighting changes map merging still fails. The goal is to recognize the place of a morning map and an evening revisit as the same place, but when appearance shifts are large, DBoW2's place recognition misses. Research groups that need long-term autonomy in outdoor settings with seasonal change are holding on to this problem. As of 2024 there is no full answer.

The place of the pure-vision baseline. Learned-feature systems have begun to beat ORB-SLAM3 on standard benchmarks. The SuperPoint + SuperGlue combination, LightGlue, and DINOv2-based features show lower error on particular sequences. Generalization is a different problem. Cases are reported where, outside the training distribution, learned features produce worse results than traditional ORB. Making the claim "consistently outperforms" will need broader experiments than exist today.

Drift at large outdoor scale. ORB-SLAM3 still lags LiDAR SLAM on urban driving and paths beyond several kilometers. Urban-scale localization in GPS-denied environments with a pure camera remains unsolved as of 2026. When changes in visual conditions, dynamic objects, and textureless stretches combine, drift accumulates. The gap to LiDAR survey precision is narrowing but has not closed.

---

The same years that the ORB-SLAM trilogy set the standard of the feature-based lineage, Newcombe and Engel were making the opposite choice. Do not pull out feature points; use the brightness information of the whole image directly. The two lineages grew side by side through the 2010s, and by taking each other as the reference they exposed each other's limits. While ORB-SLAM3 led the EuRoC benchmark in 2021, DSO beat ORB-SLAM2 in the TUM corridors. Same timetable, different starting point.
