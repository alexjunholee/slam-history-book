# Chapter 19. Dynamic and Deformable SLAM

GS-SLAM, SplaTAM, and MonoGS all retained SLAM's classic static-world assumption. Yet a parallel line of research had been grappling with moving and deforming environments almost from the start.

In 2015, Javier Fuentes-Pacheco, Ruiz-Ascencio, and Rendón-Mancha closed their survey [*Visual simultaneous localization and mapping: a survey*](https://link.springer.com/article/10.1007/s10462-012-9365-8) in *Artificial Intelligence Review* with a dedicated section on "Dynamic and Deformable Environments"—an early recognition that moving objects were more than mere outliers for RANSAC to discard. Eleven years later, the April 21, 2026 manuscript of the *SLAM Handbook* devoted 37 pages to the subject, co-authored by Lukas Schmid, José María Martínez Montiel, Shoudong Huang, Daniel Cremers, José Neira, and Javier Civera. Static scenes had been a convenient starting point, but autonomous vehicles, domestic service robots, and surgical endoscopes all operate in worlds that refuse to stand still.

---

## 19.1 Three Axes of Dynamic and Deformable SLAM

In Handbook Ch.15 §15.1, Schmid et al. overturn the conventional definition of "dynamic SLAM." Rather than treating dynamism as an intrinsic property of the environment, they define a scene as static or dynamic *relative to the observation*—specifically, the ratio between observation rate $\text{Obs}$ and scene change rate $\text{Dyn}$. When $\text{Dyn} \ll \text{Obs}$, motion unfolds continuously between consecutive frames; when $\text{Dyn} \gg \text{Obs}$, objects shift unnoticed between separate visits.

From that insight, the Handbook maps the field along three axes: an *observation axis* separating short-term frame-to-frame motion from long-term change across visits, a *reconstruction axis* spanning pose-only tracking, joint scene geometry, and full 4D spatio-temporal understanding, and a *time axis* separating online execution from offline reconstruction. Earlier surveys often equated dynamic SLAM with masking out moving objects—a task that occupies only one narrow corner of this three-dimensional space, explaining why researchers working in different corners frequently talked past one another under the same label.

---

## 19.2 Short-term: from masking to multi-object SLAM

The earliest response to short-term motion was simply to delete moving pixels before they could corrupt localization.

In 2018, Berta Bescos—then a doctoral student at the University of Zaragoza—published [DynaSLAM](https://arxiv.org/abs/1806.05620) in RA-L, pairing Mask R-CNN with the ORB-SLAM2 frontend to mask out people and cars prior to keypoint extraction. On the TUM-RGBD walking sequence, that straightforward filtering dropped ATE to single-digit centimeters.

Rather than discarding dynamic regions, other researchers tracked them alongside the background. Working under Lourdes Agapito at UCL, Martin Rünz introduced [Co-Fusion (Rünz & Agapito, 2017)](https://arxiv.org/abs/1706.06629) and [MaskFusion (Rünz et al., 2018)](https://arxiv.org/abs/1804.09194), assigning each moving object its own surfel model and jointly estimating camera and object trajectories. Taking a geometry-first route at ICRA 2018, Raluca Scona (Edinburgh) and Stefan Leutenegger (Imperial) presented [StaticFusion](https://arxiv.org/abs/1806.05628), clustering photometric residuals to segment dynamic regions without relying on semantic detectors—and thus avoiding segmentation failures.

A deeper integration brought moving objects directly inside the SLAM state vector. Led by Jun Zhang at QUT, [VDO-SLAM (Zhang et al., 2020)](https://arxiv.org/abs/2005.11052) placed both the camera pose $T_i^w \in SE(3)$ and each dynamic object's pose $T_{k,i}^w \in SE(3)$ into a unified factor graph, enforcing smooth linear and angular motion via constant-velocity factors and optimizing jointly over the product manifold. Bescos brought the same principle into ORB-SLAM2 with [DynaSLAM II (Bescos et al., 2021)](https://arxiv.org/abs/2010.07820), and Yuheng Qiu at CMU extended it to articulated bodies such as walking humans in [AirDOS](https://arxiv.org/abs/2109.09903) (RA-L 2022).

> 🔗 **Borrowed.** VDO-SLAM's factor-graph extension directly inherits the iSAM tradition established by Dellaert and Kaess in graph SLAM (→ Chapter 6). Dynamic SLAM modeled a moving car by inserting its pose variables and measurement factors straight into the map graph.

A third strategy bypassed semantic masks and object trackers altogether by leaning on inertial sensing. In [DynaVINS](https://arxiv.org/abs/2208.11500) (RA-L 2022), Song, Lim, Lee, and Myung at KAIST URL downweighted visual bundle-adjustment factors whenever features clashed with the pose prior from IMU preintegration. Their follow-up, [DynaVINS++](https://arxiv.org/abs/2410.15373) (RA-L 2024), recast that weighting as adaptive truncated least squares and closed a subtle failure mode in which dynamic outliers could poison IMU-bias estimation and trigger divergence.

The Handbook groups these threads under §15.2.3, "Dense Dynamic SLAM," highlighting Schmid's [Dynablox (Schmid et al., 2023)](https://arxiv.org/abs/2304.10049) for LiDAR moving-object segmentation alongside transformer-based [AnyCam](https://arxiv.org/abs/2503.23282) (2025), which recovers 4D structure directly from casual video in a modern continuation of Rünz's 2017 simultaneous tracking-and-reconstruction agenda.

---

## 19.3 Long-term: maps across time

Short-term methods track motion visible between consecutive frames; long-term dynamic SLAM confronts scenes that transform between visits—a chair shifted overnight, or a construction site reconfigured over weeks.

Drawing inspiration from human memory models, Mathieu Labbé (supervised by Michaud at Sherbrooke) began building [RTAB-Map](https://introlab.github.io/rtabmap/) in 2013 with a hierarchy of short-term, working, and long-term memory. Map nodes stayed in working memory during active exploration, migrated to long-term memory when unvisited, and were pruned once redundant—a design Labbé scaled to multi-session mapping in a 2019 JFR paper. In LiDAR mapping, Hyungtae Lim, Sungwon Hwang, and Hyun Myung at KAIST URL targeted lingering "ghost" trails of moving vehicles in [ERASOR](https://arxiv.org/abs/2103.04316) (2021), comparing vertical height profiles between accumulated maps and single scans before applying region-wise ground-plane fitting to strip away dynamic points.

At the heart of long-term mapping, Handbook §15.3 stresses a crisp epistemic distinction: **absence of evidence is not evidence of absence**. A map-maintenance algorithm must tell whether an object genuinely vanished or was merely occluded from the current viewpoint; otherwise, cleanup passes risk deleting permanent fixtures or mislabeling hidden regions as empty. Schmid's [Panoptic Multi-TSDF](https://arxiv.org/abs/2109.10165) (RA-L 2022) addressed this by giving each object its own TSDF submap governed by local consistency rules for active and inactive states. The same group's [Khronos](https://arxiv.org/abs/2402.13817) (2024) added graduated non-convexity for robust data association and ran deformable geometric change detection after loop closures, reconstructing *when* each object changed to yield a true 4D spatio-temporal map.

> 🔗 **Borrowed.** Panoptic Multi-TSDF fuses volumetric submapping—maintaining separate local TSDF volumes—with object-centric mapping, which treats individual objects as independent map models.

Point-cloud mapping pursued the same challenge from a LiDAR perspective. In 2026, Jang, Lee, Nahrendra, and Myung at KAIST URL introduced [Chamelion](https://arxiv.org/abs/2602.08189), pairing scene-mixing data augmentation with a dual-head network to detect structural changes without ground-truth supervision in transient sites such as construction zones and rearranged interiors—complementing Khronos's panoptic RGB-D 4D reconstruction with long-term point-cloud upkeep.

Still another research stream modeled *recurring* temporal patterns rather than one-off edits. Starting in 2014, Tomáš Krajník and Achim Lilienthal at Örebro University encoded periodic cycles—commuter flows, day-night illumination shifts—as Fourier **frequency maps**. In 2019, Martin Magnusson's group at KTH consolidated these ideas into Maps of Dynamics (MoD), embedding typical motion tendencies ("pedestrians usually keep left in this corridor") directly inside the map, while [Changing-SLAM (Schmid et al., 2023)](https://arxiv.org/abs/2301.09479) coupled Kalman filtering for short-term motion with semantic class matching for long-term changes inside an ORB-SLAM extension.

---

## 19.4 Deformable: when the shape itself changes

Even harder than moving rigid bodies are scenes where the background itself bends and stretches—a domain championed in robotics by Civera and Montiel at Zaragoza.

A landmark early system was [DynamicFusion](https://grail.cs.washington.edu/projects/dynamicfusion/), the CVPR 2015 Best Paper by Newcombe, Fox, and Seitz at Microsoft Research, which draped an embedded deformation graph over KinectFusion's canonical TSDF to reconstruct deforming faces and torsos in real time by optimizing per-node rotations and translations every frame. Follow-on RGB-D systems rapidly loosened its assumptions: Matthias Innmann at TU München incorporated color cues in [VolumeDeform](https://arxiv.org/abs/1603.08161) (2016); Miroslava Slavcheva introduced Killing-vector-field regularization in [KillingFusion](https://campar.in.tum.de/pub/slavcheva2017cvpr/slavcheva2017cvpr.pdf) (2017) so topologies could split or merge, such as fingers separating from a palm; and Wei Gao, supervised by Tedrake at MIT, swapped voxels for surfels in [SurfelWarp](https://arxiv.org/abs/1904.13073) (2019) to support open-ended exploration.

> 🔗 **Borrowed.** DynamicFusion directly adapted the embedded deformation graph introduced by Sumner, Schmid, and Pauly in computer graphics in 2007—turning a sparse mesh-animation rig into the state parameterization for real-time non-rigid SLAM.

Meanwhile, Zaragoza pushed deformable SLAM into the depth-free monocular regime. Juan Lamarca, completing his doctorate under Montiel, presented [DefSLAM](https://arxiv.org/abs/1908.08918) (RA-L 2021), combining an ORB frontend and Lucas-Kanade tracking with isometric non-rigid SfM (NRSfM) templates recomputed at each keyframe under a planar-topology assumption. In 2023, Juan J. Gómez Rodríguez from the same group lifted that topological restriction in [NR-SLAM](https://arxiv.org/abs/2308.04036), introducing a dynamic deformation graph and visco-elastic temporal regularization—a trajectory Handbook §15.4.2 terms the "monocular line of deformable SLAM."

Much of the practical pull came from minimally invasive surgery (MIS). Song at Tsinghua built [MIS-SLAM](https://ieeexplore.ieee.org/document/8458232) (2018) to track intraoperative organ deformation from stereo endoscopes, while the Jayender group at Children's National developed EMDQ (Expectation Maximization + Dual Quaternion) to estimate smooth deformation fields over SURF features for surgical navigation.

Yet non-rigid SLAM confronts a subtle theoretical barrier highlighted in Handbook §15.4.1 as **Floating Map Ambiguity**: without strong priors, image observations alone cannot separate a rigid motion of the deforming scene from a rigid motion of the camera. Vision by itself cannot tell whether a hand shifted 30 cm right or the camera shifted 30 cm left. Unlike standard monocular scale ambiguity, scale, camera trajectory, and surface deformation entangle into a single ill-posed problem; DefSLAM and NR-SLAM tame it with isometric and visco-elastic priors, but a principled general resolution remained absent as of 2026.

> 📜 **Prediction vs. outcome.** In §7 (Future Work) of DynamicFusion (2015), Newcombe pointed to larger environments, topological changes, and loop-closure integration as the next milestones. KillingFusion delivered topological transitions in 2017 and surfel-based SurfelWarp expanded spatial reach in 2019; by 2024, Khronos unified a deformation graph, loop closures, and change detection for long-term scenes whose object layouts evolve over time.

---

## 19.5 One way to read the connections

These contributions do not sort neatly into airtight schools, yet tracking co-authorships and institutional moves reveals three intertwined lineages. **The Zaragoza line** (Montiel, Neira, Civera, Lamarca, Rodríguez) carried monocular geometry from MonoSLAM (Chapter 5) and ORB-SLAM (Chapter 8) straight into DynaSLAM, DefSLAM, and NR-SLAM. **The dense dynamic-reconstruction line** ran from KinectFusion to DynamicFusion and from SLAM++ to Co-Fusion and MaskFusion, linking Davison, Newcombe, Agapito, Rünz, and Cremers across labs. Finally, **Schmid's path**—moving from Cremers's group at TUM through Carlone's lab at MIT to JPL—connects Dynablox, Panoptic Multi-TSDF, and Khronos. This three-part synthesis reflects the chapter's reading of the historical record rather than an explicit taxonomy from the Handbook.

---

## Still open

**Absence vs. evidence of absence.** Distinguishing an object that truly left the scene from one hidden behind occlusion remains foundational for long-term mapping. Schmid's Panoptic Multi-TSDF offered a partial solution via active submaps, and the Handbook notes extreme occlusion above 70% as a regime that remains acute (without treating 70% as a universal threshold specific to Panoptic Multi-TSDF). As of 2026, no paper claimed a complete, principled answer.

**Floating Map Ambiguity.** Deformable SLAM still leans on isometric and visco-elastic priors to disentangle camera motion from non-rigid object motion. While Lamarca's [2023 IJRR paper](https://arxiv.org/abs/2302.03710) characterized observability under specific conditions, no general theory dictates when visual measurements alone suffice.

**Online deformable SLAM.** Although DefSLAM and NR-SLAM run near real time, no monocular RGB pipeline performs Khronos-caliber change-aware 4D integration online; the optimization load still outstrips live budgets, and GPU-accelerated or learned-prior pipelines have yet to close the gap.

**The real-world gap in medical MIS.** Systems like MIS-SLAM and NR-SLAM perform reliably on phantoms and ex vivo tissue, yet falter amid real surgical conditions—blood, cautery smoke, instrument occlusions, and harsh endoscopic lighting. Gaussian-based approaches such as EndoGS (2024) are emerging, but clinical deployment remains out of reach.

---

Both NeRF-SLAM and 3DGS-SLAM accelerated or sharpened scene reconstruction while keeping the world static; dynamic SLAM relaxed that static premise inside classical and dense pipelines. Soon after, DUSt3R and its successors posed a more radical alternative: instead of refining a specialized SLAM pipeline piece by piece, why not learn 3D geometric priors directly from vast collections of images (→ Chapter 20)?
