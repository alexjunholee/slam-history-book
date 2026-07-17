# Ch.15b — Where the Static-World Assumption Breaks: Dynamic and Deformable SLAM

GS-SLAM, SplaTAM, and MonoGS all retained the static-world assumption. A separate line of research had addressed moving and deforming environments from the outset.

In 2015, Javier Fuentes-Pacheco, Ruiz-Ascencio, and Rendón-Mancha published [*Visual simultaneous localization and mapping: a survey*](https://link.springer.com/article/10.1007/s10462-012-9365-8) in *Artificial Intelligence Review*. Its final section addressed "Dynamic and Deformable Environments." Earlier papers had studied moving objects, but most treated them as outliers for RANSAC to reject. The survey was an early account that treated dynamic environments as a distinct topic. Ten years later, the 2025 *SLAM Handbook* devoted 37 pages to it. Its six authors were Lukas Schmid, José María Martínez Montiel, Shoudong Huang, Daniel Cremers, José Neira, and Javier Civera. Although static scenes had been SLAM's starting point, self-driving cars, domestic service robots, and endoscopes all had to operate in changing environments.

---

## 15b.1 Three axes

In Handbook Ch.15 §15.1, Schmid et al. revise the earlier definition of "dynamic SLAM." They define an environment as dynamic or static relative to *the observation*, not as an intrinsic property of the environment. The same physical motion may appear as a short-term change to one robot and a long-term change to another, depending on the ratio between observation rate $\text{Obs}$ and change rate $\text{Dyn}$. When $\text{Dyn} \ll \text{Obs}$, motion is visible between frames; when $\text{Dyn} \gg \text{Obs}$, the scene changes between visits.

This perspective defines three axes. The observation axis distinguishes short-term from long-term change. The reconstruction axis distinguishes pose-only estimation, joint scene geometry, and full 4D spatio-temporal understanding. The time axis separates online from offline methods. Earlier accounts often reduced the field to removing dynamic objects, but that task occupies only one region of this three-axis space. Researchers working in different regions of the taxonomy had used the same terminology for different problems.

---

## 15b.2 Short-term: from masking to multi-object SLAM

The earliest solution removed moving regions from the measurements.

Berta Bescos, then a doctoral student at Zaragoza, published [DynaSLAM](https://arxiv.org/abs/1806.05620) in RA-L in 2018. The system added Mask R-CNN to the ORB-SLAM2 frontend, masking people and cars before excluding those regions from keypoint extraction. On the TUM-RGBD walking sequence, this direct approach reduced ATE to single-digit centimeters.

During the same period, Martin Rünz at UCL tracked moving objects separately rather than removing them. Under Lourdes Agapito's supervision, he released [Co-Fusion (Rünz & Agapito, 2017)](https://arxiv.org/abs/1706.06629) and [MaskFusion (Rünz et al., 2018)](https://arxiv.org/abs/1804.09194) in consecutive years. Each object received its own surfel model, and the system jointly estimated camera and object trajectories. At ICRA 2018, Raluca Scona of Edinburgh and Stefan Leutenegger of Imperial presented another approach in [StaticFusion](https://arxiv.org/abs/1806.05628). It separated dynamic regions through residual clustering without semantic segmentation, avoiding errors from that component.

The next approach included moving objects in the estimated state. Jun Zhang at QUT led [VDO-SLAM (Zhang et al., 2020)](https://arxiv.org/abs/2005.11052), which represented each dynamic object as a variable in the factor graph. The camera pose $T_i^w \in SE(3)$ and the pose of object $k$, $T_{k,i}^w \in SE(3)$, appeared in the same graph. A constant-velocity factor enforced continuity in each object's linear and angular velocities, and joint optimization operated over the product manifold of the camera and object SE(3) states. In 2021, Bescos at Zaragoza implemented the same principle on ORB-SLAM2 in [DynaSLAM II (Bescos et al., 2021)](https://arxiv.org/abs/2010.07820). Yuheng Qiu at CMU extended it to articulated objects such as human bodies in [AirDOS](https://arxiv.org/abs/2109.09903), published in RA-L in 2022.

> 🔗 **Borrowed.** VDO-SLAM's factor-graph extension directly follows the iSAM tradition established by Dellaert and Kaess in the graph-SLAM work discussed in Ch.6. Dynamic SLAM represented a moving car by adding its state variables and measurement factors to the map.

A third approach used inertial information. Song, Lim, Lee, and Myung at KAIST URL published [DynaVINS](https://arxiv.org/abs/2208.11500) in RA-L in 2022 without semantic masks or multi-object tracking. During bundle adjustment, the method reduced the factor weights of observations that disagreed with the pose prior from IMU preintegration, limiting the influence of dynamic features on the joint state. The same group's [DynaVINS++](https://arxiv.org/abs/2410.15373), published in RA-L in 2024, reformulated the method as adaptive truncated least squares. It also addressed the failure mode in which dynamic features affected IMU-bias estimation and caused divergence.

The Handbook groups this work under §15.2.3, "Dense Dynamic SLAM," and presents Schmid's [Dynablox (Schmid et al., 2023)](https://arxiv.org/abs/2304.10049) as a current LiDAR MOS system. [AnyCam](https://arxiv.org/abs/2503.23282) (2025) uses a transformer backbone to recover 4D structure directly from ordinary video, extending the simultaneous tracking-and-reconstruction approach introduced by Rünz in 2017.

---

## 15b.3 Long-term: maps across time

Short-term dynamics describe motion between frames, whereas long-term dynamics describe changes between visits, such as a chair moved overnight. Research on this problem followed a different lineage.

Under Michaud's supervision at Sherbrooke, Mathieu Labbé developed [RTAB-Map](https://introlab.github.io/rtabmap/) beginning in 2013, drawing directly on models of human memory. It organized short-term, working, and long-term memory hierarchically and moved nodes according to time and observation frequency. A node remained in working memory during a session, moved to long-term memory if it was not revisited often, and was discarded if it no longer carried useful information. In a 2019 JFR paper, Labbé described how this structure scaled to multi-session SLAM. Hyungtae Lim, working under Ayoung Kim at KAIST, took a different approach in [ERASOR](https://arxiv.org/abs/2103.04316) (2021). He formulated map cleaning as scene differencing, identifying points that disappeared between two traversals of the same location.

Handbook §15.3 repeatedly distinguishes **absence of evidence from evidence of absence**. A mapping system must determine whether an object has disappeared or was simply not observed. Without this distinction, map cleaning can erase valid objects and change detection can misclassify occluded regions. Schmid's [Panoptic Multi-TSDF](https://arxiv.org/abs/2109.10165), published in RA-L in 2022, addressed the problem with independent submaps for each object and a local-consistency rule for active and inactive states. The same group's [Khronos](https://arxiv.org/abs/2402.13817) (2024) further used graduated non-convexity for robust association. After loop closure, it performed deformable geometric change detection and estimated when each object changed, extending a metric-semantic map into a 4D spatio-temporal representation.

> 🔗 **Borrowed.** Panoptic Multi-TSDF adapts the multi-map management introduced by Atlas in the ORB-SLAM work of Ch.7. It replaces keyframe submaps with panoptic object submaps while retaining the principle of partitioning a map that has become too large or heterogeneous.

LiDAR research addressed the same problem separately. Jang, Lee, Nahrendra, and Myung at KAIST URL released [Chamelion](https://arxiv.org/abs/2602.08189) in 2026. It combined scene-mixing augmentation with a dual-head network to perform change detection without ground truth in transient environments such as construction sites and frequently rearranged indoor spaces. Khronos built a 4D representation from RGB-D and panoptic inputs, whereas Chamelion addressed long-term maintenance of point-cloud maps.

Another line of work modeled recurring changes. Beginning in 2014, Tomáš Krajník and Achim Lilienthal at Örebro in Sweden developed **frequency maps**, representing periodic events such as commuter traffic and day-night lighting changes with a Fourier basis. In 2019, Martin Magnusson's group at the Stockholm Royal Institute of Technology consolidated this work as Maps of Dynamics (MoD), encoding *typical motion patterns* directly in the map. A statement such as "people usually walk to the left in this corridor" became part of the representation. [Changing-SLAM (Schmid et al., 2023)](https://arxiv.org/abs/2301.09479) combined a Kalman filter for short-term changes with semantic class matching for long-term changes in an ORB-SLAM extension.

---

## 15b.4 Deformable: when the shape itself changes

Deformable SLAM addresses scenes in which even the background changes shape, a problem studied extensively by Civera and Montiel in Zaragoza.

An earlier starting point was [DynamicFusion](https://grail.cs.washington.edu/projects/dynamicfusion/), the 2015 CVPR best paper by Newcombe, Fox, and Seitz at Microsoft Research. It placed an embedded deformation graph over KinectFusion's canonical TSDF to reconstruct non-rigid objects such as faces and torsos in real time. Each graph node carried a rotation and translation that the system optimized in every frame. In related work, Matthias Innmann at TU München added color information in [VolumeDeform](https://arxiv.org/abs/1603.08161) (2016). In 2017, Miroslava Slavcheva introduced [KillingFusion](https://campar.in.tum.de/pub/slavcheva2017cvpr/slavcheva2017cvpr.pdf), using Killing-vector-field regularization to permit topological changes such as a hand separating from the torso. Under Tedrake's supervision at MIT, Wei Gao's [SurfelWarp](https://arxiv.org/abs/1904.13073) (2019) used surfels instead of a TSDF to support exploration more readily.

> 🔗 **Borrowed.** DynamicFusion directly adapted the embedded deformation graph published by Sumner, Schmid, and Pauly in computer graphics in 2007. A sparse control graph for mesh deformation became the variable representation for real-time non-rigid SLAM.

Monocular deformable SLAM developed in Zaragoza. Juan Lamarca, who completed his doctorate under Montiel, published [DefSLAM](https://arxiv.org/abs/1908.08918) in RA-L in 2021. The system recomputed a template at each keyframe with isometric NRSfM and combined an ORB frontend with Lucas-Kanade optical flow to maintain tracks. It assumed planar topology. In 2023, Juan J. Gómez Rodríguez from the same group removed this limitation with [NR-SLAM](https://arxiv.org/abs/2308.04036), using a dynamic deformation graph for arbitrary topology and a visco-elastic model for temporal regularization. Handbook §15.4.2 groups this work as the "monocular line of deformable SLAM."

Many applications are medical. Song at Tsinghua released [MIS-SLAM](https://ieeexplore.ieee.org/document/8458232) in 2018 to track intraoperative organ deformation with stereo endoscopy. The Jayender group at Children's National developed EMDQ (Expectation Maximization + Dual Quaternion), which estimated a smooth deformation field over SURF features. Both systems targeted intraoperative navigation for minimally invasive surgery.

Handbook §15.4.1 highlights a fundamental problem: **Floating Map Ambiguity**. Without a prior, observations cannot distinguish the rigid motion of a non-rigid object from the camera's rigid motion. The image alone cannot determine whether a hand moved 30 cm or the camera moved 30 cm. This differs from the conventional scale ambiguity of monocular SLAM because scale, trajectory, and deformation become coupled in one ill-posed problem. DefSLAM and NR-SLAM partially constrain the ambiguity with isometric and visco-elastic priors, but no principled solution existed as of 2026.

> 📜 **Prediction vs. outcome.** In §7, Future Work, of DynamicFusion (2015), Newcombe identified extension to larger scenes and topological changes, along with integration with loop closure, as the next challenges. KillingFusion addressed topological change in 2017, and the surfel-based SurfelWarp (2019) partially addressed larger scenes. Loop closure did not appear until Khronos introduced deformable geometric change detection in 2024, nine years later.

---

## 15b.5 One way to read the connections

These works do not divide cleanly into three fixed schools. The papers, collaborations, and movement of researchers nevertheless reveal three overlapping lines. **The Zaragoza line** (Montiel, Neira, Civera, Lamarca, Rodríguez) pushed monocular geometry from MonoSLAM (Ch.5) and ORB-SLAM (Ch.7) into DynaSLAM, DefSLAM, and NR-SLAM. **The dense dynamic-reconstruction line** ran from KinectFusion to DynamicFusion and from SLAM++ to Co-Fusion and MaskFusion, linking work by Davison, Newcombe, Agapito, Rünz, and Cremers across several institutions. **Schmid's path** from the Cremers group at TUM through the Carlone group at MIT to JPL connects Dynablox, Panoptic Multi-TSDF, and Khronos. This is the chapter's interpretation of the lineage, not a taxonomy explicitly declared by the Handbook.

---

## 🧭 Still open

**Absence vs. evidence of absence.** Determining whether an object has disappeared from the map or was merely occluded remains a foundational problem in long-term SLAM. Schmid's Panoptic Multi-TSDF provided a partial answer through active submaps. The Handbook treats occlusion above 70% as an extreme-environment case that remains difficult; it does not report this as a universal error threshold for Panoptic Multi-TSDF. As of 2026, no paper had claimed a principled solution.

**Floating Map Ambiguity.** Deformable SLAM still relies on isometric and visco-elastic priors to separate the camera's rigid motion from an object's rigid motion. The conditions under which observations alone can identify both motions remain unknown. Lamarca's [2023 IJRR paper](https://arxiv.org/abs/2302.03710) described some observation conditions, but no general theory exists.

**Online deformable SLAM.** DefSLAM and NR-SLAM approach real-time operation, but no system performs Khronos-level change-aware integration online from monocular RGB input. Its optimization cost exceeds real-time limits. GPU acceleration and learned priors may help, but no validated pipeline has yet appeared.

**The real-world gap in medical MIS.** MIS-SLAM and NR-SLAM operate on phantoms and ex vivo data, but robustness declines in surgical environments containing blood, smoke, tool occlusions, and abrupt lighting changes. Gaussian-based methods such as EndoGS (2024) are emerging, but no system has been reported at deployment level.

---

The question running through this chapter is how to represent a world that changes. GS-SLAM and NeRF-SLAM improved the speed or compactness of scene representations while retaining a static-world assumption. Ch.16 follows DUSt3R and its successors along another route: learning the geometric prior rather than refining the representation pipeline alone.
