# Ch.15b — Where the Static-World Assumption Breaks: Dynamic and Deformable SLAM

Ch.15 ended with Gaussian Splatting holding the static-world assumption intact. GS-SLAM, SplaTAM, and MonoGS all treated the scene as fixed. A parallel lineage had refused that assumption from the start.

In 2015, ETH Zürich's Javier Fuentes-Pacheco, together with Ruiz-Ascencio and Rendón-Mancha, published [*Visual simultaneous localization and mapping: a survey*](https://link.springer.com/article/10.1007/s10462-012-9365-8) in Artificial Intelligence Review. The last chapter of that survey was "Dynamic and Deformable Environments." Papers dealing with moving objects had appeared earlier, but most treated them as outliers for RANSAC to reject. The Fuentes-Pacheco survey was the first document to declare dynamic environments an independent field. Ten years later, in 2025, the SLAM Handbook devotes 37 pages to this topic, the largest allocation of any chapter. There are six authors: MIT's Lukas Schmid, TU München's Daniel Cremers, UTS's Shoudong Huang, and Zaragoza's Montiel, Neira, and Civera. Three schools across three continents converged on one chapter because of what happened between 2015 and 2025. The static world had been the starting point of every SLAM system, but the robots that actually existed — self-driving cars on the street, service robots in homes, endoscopes inside organs — had to work outside that assumption.

---

## 15b.1 Three axes

The frame Schmid et al. sketch in Handbook Ch.15 §15.1 rewrites the earlier definition of "dynamic SLAM." Whether an environment is dynamic or static is not a property of the environment but *a property of the observation*. The same physical motion becomes short-term dynamic for one robot and long-term dynamic for another. The ratio between observation rate $\text{Obs}$ and change rate $\text{Dyn}$ decides it. When $\text{Dyn} \ll \text{Obs}$, motion is visible between frames; when $\text{Dyn} \gg \text{Obs}$, the scene has changed between visits.

This perspective produces three axes. The observation axis splits short-term from long-term. The reconstruction axis decides whether to estimate pose only, scene geometry as well, or to go all the way to 4D spatio-temporal understanding. The time axis splits online from offline. Earlier accounts compressed the field into the single question "how do we remove dynamic objects," but seen in this three-axis space, that question occupies only one corner of eight octants. This is why the field fractured. Researchers standing in different octants had been using the same words differently.

---

## 15b.2 Short-term: from masking to multi-object SLAM

The first solution was simple. Erase what moves.

Berta Bescos, then a doctoral student at Zaragoza, published [DynaSLAM](https://arxiv.org/abs/1806.05620) in RA-L in 2018. The system inserted Mask R-CNN into the ORB-SLAM2 frontend and masked people and cars in advance. Masked regions were excluded from keypoint extraction. Simple, but it worked. On the TUM-RGBD walking sequence, ATE dropped to single-digit centimeters.

In the same period, UCL's Martin Rünz made a different choice. Do not erase the moving objects — track them separately. Under Lourdes Agapito's supervision, he released [Co-Fusion (Rünz & Agapito, 2017)](https://arxiv.org/abs/1706.06629) and, the following year, [MaskFusion (Rünz et al., 2018)](https://arxiv.org/abs/1804.09194) back to back. Each object was assigned its own surfel model, and camera trajectory and object trajectories were estimated jointly. Edinburgh's Raluca Scona and Imperial's Stefan Leutenegger took yet another route at ICRA 2018 with [StaticFusion](https://arxiv.org/abs/1806.05628). Without semantic segmentation, they separated dynamic regions using residual clustering alone. A direction that does not depend on segmentation errors.

At this point the idea shifts once more. What if we bring moving objects into the state and estimate them together? QUT's Jun Zhang led [VDO-SLAM (Zhang et al., 2020)](https://arxiv.org/abs/2005.11052), which promoted each dynamic object to a variable in the factor graph. Camera pose $T_i^w \in SE(3)$ and object $k$'s pose $T_{k,i}^w \in SE(3)$ coexisted in the same graph. A constant-velocity factor imposed continuity on the object's linear and angular velocities. Joint optimization ran over the product manifold of SE(3) and object SE(3). Zaragoza's Bescos implemented the same idea on an ORB-SLAM2 base in 2021 with [DynaSLAM II (Bescos et al., 2021)](https://arxiv.org/abs/2010.07820). CMU's Yuheng Qiu extended it to articulated bodies — objects with joints, like humans — in [AirDOS](https://arxiv.org/abs/2109.09903), published in RA-L in 2022.

> 🔗 **Borrowed.** VDO-SLAM's factor-graph extension inherits directly from the iSAM tradition Dellaert and Kaess established in Ch.6 graph SLAM. Adding one more variable and one more factor, in dynamic SLAM, became the act of putting a single moving car on the map.

A third angle came in from the inertial side. KAIST URL's Song, Lim, Lee, and Myung published [DynaVINS](https://arxiv.org/abs/2208.11500) in RA-L in 2022 without using either semantic masks or multi-object tracking. Observations that disagreed with the pose prior from IMU preintegration had their factor weights reduced during bundle adjustment, cutting off the path through which dynamic features could leak into the joint state. The same group's [DynaVINS++](https://arxiv.org/abs/2410.15373), RA-L 2024, reformulated this as adaptive truncated least squares, catching even the failure mode where dynamic features back-propagate into IMU bias estimation and diverge.

The Handbook organizes this lineage under §15.2.3 "Dense Dynamic SLAM" and places Schmid's own [Dynablox (Schmid et al., 2023)](https://arxiv.org/abs/2304.10049) as the current form of LiDAR MOS. The 2025 [AnyCam](https://arxiv.org/abs/2503.23282) pulls 4D directly from everyday video using a transformer backbone. It is the 2025 version of the "simultaneous tracking + reconstruction" line that Rünz opened in 2017.

---

## 15b.3 Long-term: maps across time

If short-term is motion between frames, long-term is change between visits. The chair I saw yesterday has been pushed aside today. This problem grew from a different lineage.

Sherbrooke's Mathieu Labbé, under Michaud's supervision, developed [RTAB-Map](https://introlab.github.io/rtabmap/) from 2013, borrowing directly from human memory models. It placed short-term, working, and long-term memory in a hierarchy and moved nodes according to time and observation frequency. Within a session a node stayed in working memory; if not frequently revisited it descended to long-term memory; if it lost meaning it was discarded. In a 2019 JFR paper, Labbé laid out how this structure scales to multi-session SLAM. At KAIST, under Ayoung Kim, Hyungtae Lim published [ERASOR](https://arxiv.org/abs/2103.04316) in 2021, taking a different angle. He turned the problem of making the map clean into scene differencing — finding points that had disappeared between two passes through the same place.

A frame that runs through all of Handbook §15.3: **absence of evidence vs evidence of absence**. One has to distinguish whether the chair is not there from whether one simply did not see it. Without this distinction, map cleaning erases legitimate objects and change detection misjudges occluded regions. Schmid's [Panoptic Multi-TSDF](https://arxiv.org/abs/2109.10165), RA-L 2022, addressed this with a submap structure. Each object was managed as an independent submap, and active and inactive were separated under local consistency. The same group's [Khronos](https://arxiv.org/abs/2402.13817), 2024, went one step further. It robustified association with graduated non-convexity, and even after loop closure it ran deformable geometric change detection, estimating the moment of change for each object. The point at which a metric-semantic map turns into a 4D spatio-temporal map.

> 🔗 **Borrowed.** The submap structure of Panoptic Multi-TSDF reweaves, in different material, the multi-map management Atlas introduced in Ch.7 ORB-SLAM. The keyframe submap has become a panoptic object submap, but the principle — split when a single map grows too large — remains.

The same question rolled along separately on the LiDAR side. KAIST URL's Jang, Lee, Nahrendra, and Myung released [Chamelion](https://arxiv.org/abs/2602.08189) in 2026, stacking scene-mixing augmentation on top of a dual-head network to run change detection without ground truth in transient environments (construction sites, frequently rearranged indoor spaces) where structure flips moment to moment. Where Khronos built 4D on the RGB-D and panoptic side, Chamelion carries the same question toward long-term map maintenance on point clouds.

Another axis in this lineage is research on recurrence. Örebro's Tomáš Krajník and Achim Lilienthal, in Sweden, developed **frequency maps** from 2014, modeling periodic events — commuter traffic flow, day-night lighting changes — on a Fourier basis. Stockholm Royal Institute of Technology's Martin Magnusson group consolidated this in 2019 as Maps of Dynamics (MoD), encoding *typical motion patterns* directly into the map. "People walk to the left in this corridor" becomes part of the map. The 2023 [Changing-SLAM (Schmid et al., 2023)](https://arxiv.org/abs/2301.09479) attempted to handle short-term with a Kalman filter and long-term with semantic class matching, simultaneously, on top of an ORB-SLAM extension.

---

## 15b.4 Deformable: when the shape itself changes

What happens when even the background moves? Civera and Montiel in Zaragoza stood in front of this question for a long time.

The start was elsewhere. The 2015 CVPR best paper was [DynamicFusion](https://grail.cs.washington.edu/projects/dynamicfusion/), by Microsoft Research's Newcombe, Fox, and Seitz. An embedded deformation graph was layered on KinectFusion's canonical TSDF, reconstructing non-rigid objects (faces, torsos) in front of the camera in real time. A deformation graph with rotation and translation assigned to each node was optimized every frame. In the same line, TU München's Matthias Innmann added color information with [VolumeDeform](https://arxiv.org/abs/1603.08161) in 2016, and in 2017 Miroslava Slavcheva introduced [KillingFusion](https://campar.in.tum.de/pub/slavcheva2017cvpr/slavcheva2017cvpr.pdf), bringing in Killing vector field regularization to allow topology changes — a hand parting from the torso. At MIT, under Tedrake's supervision, Wei Gao's 2019 [SurfelWarp](https://arxiv.org/abs/1904.13073) chose surfels over TSDF to gain exploration friendliness.

> 🔗 **Borrowed.** DynamicFusion's embedded deformation graph was lifted directly from the ED graph Sumner, Schmid, and Pauly published in computer graphics in 2007. A sparse control graph for mesh deformation became the variable representation for real-time non-rigid SLAM.

The monocular story played out in Zaragoza. Juan Lamarca, who finished his doctorate under Montiel, published [DefSLAM](https://arxiv.org/abs/1908.08918) in RA-L in 2021. He recomputed a template at each keyframe with isometric NRSfM and mixed an ORB frontend with Lucas-Kanade optical flow to maintain traces. A limitation that assumed planar topology. The same group's Juan J. Gómez Rodríguez removed that limitation in 2023 with [NR-SLAM](https://arxiv.org/abs/2308.04036), handling arbitrary topology with a dynamic deformable graph and adding temporal regularization through a visco-elastic model. Handbook §15.4.2 organizes this lineage as the "monocular line of deformable SLAM."

The applications cluster on the medical side. Tsinghua's Song released [MIS-SLAM](https://ieeexplore.ieee.org/document/8458232) in 2018, tracking the deformation of intraoperative organs with stereo endoscopy. Children's National's Jayender group developed EMDQ (Expectation Maximization + Dual Quaternion), which estimated a smooth deformation field over SURF features. Intra-operative navigation in the real environment of minimally invasive surgery is the target of these systems.

One fundamental problem highlighted in Handbook §15.4.1: **Floating Map Ambiguity**. Without a prior, the rigid motion of a non-rigid object cannot be distinguished from the rigid motion of the camera. Whether the hand moved 30 cm or the camera moved 30 cm — observation alone says neither. Recovery of absolute scale here differs in character from the long-standing scale ambiguity of monocular SLAM. Not only scale but trajectory and deformation couple simultaneously and become ill-posed. DefSLAM and NR-SLAM break this ambiguity partially with isometric and visco-elastic priors, but no principled solution exists as of 2026.

> 📜 **Prediction vs. outcome.** In §7 Future Work of DynamicFusion (2015), Newcombe listed "extension to larger scenes and topology changes" and "integration with loop closure" as the next challenges. Topology change was answered in 2017 by KillingFusion. Large-scale scenes were addressed in part by the surfel-based SurfelWarp (2019). Integration with loop closure did not appear until 2024, under the name deformable geometric change detection in Khronos. Nine years in total. `[partial hit]`

---

## 15b.5 The intellectual lineage of three schools

The arrangement of the six Handbook authors in this chapter is itself the evidence.

**The Zaragoza school** (Montiel, Neira, Civera, Lamarca, Rodríguez) is the home of deformable geometry, running from MonoSLAM (Ch.5) through ORB-SLAM (Ch.7), DynaSLAM, DefSLAM, and NR-SLAM. A twenty-year tradition of pushing geometry to the limit in the monocular setting. **The Imperial/TUM lineage** (Davison, Newcombe, Rünz, Cremers) takes the dense and learning-based axis. KinectFusion (Ch.9) led to DynamicFusion; SLAM++ (Ch.18) led to Co-Fusion and MaskFusion. As the Cremers group shifted toward change-aware SLAM in the 2020s, it became the center of a new lineage. **The Cambridge/ETH/MIT lineage** (Schmid, Leutenegger, Agapito) converged on panoptic 4D. Schmid himself completed his doctorate under Cremers, passed through the Carlone group at MIT, and went on to JPL. That trajectory overlaps with the sequence KillingFusion → Dynablox → Panoptic Multi-TSDF → Khronos.

The six-author composition of Handbook Ch.15 reproduces these three schools almost exactly. That the field has split into three branches is self-evidenced by the author list.

---

## 🧭 Still open

**Absence vs evidence of absence.** Distinguishing whether an object has disappeared from the map or was merely occluded remains a foundational difficulty of long-term SLAM. Schmid's Panoptic Multi-TSDF gave a partial answer with an active-submap structure, but in outdoor large-scale environments and in settings with occlusion above 60%, the decision error remains large. As of 2026, no paper has claimed a principled solution.

**Floating Map Ambiguity.** In deformable SLAM, separating the camera's rigid motion from the object's rigid motion is still only worked around with isometric and visco-elastic priors. What conditions identify the two motions without a prior, and what observations break the ambiguity, are unresolved. Lamarca's [2023 IJRR paper](https://arxiv.org/abs/2302.03710) laid out some of the observation conditions, but no general theory yet.

**Online deformable SLAM.** DefSLAM and NR-SLAM come close to real time, but no system runs change-aware integration at the Khronos level online on a monocular RGB input. The optimization cost crosses the real-time limit. GPU acceleration and learned priors open possibilities, but no validated pipeline has appeared yet.

**The real-world gap in medical MIS.** MIS-SLAM and NR-SLAM work on phantoms and ex vivo data, but robustness drops in the actual surgical environment — blood, smoke, tool occlusion, abrupt lighting changes. Gaussian-based attempts such as 2024's EndoGS are appearing, but no system has been reported to reach deployment level.

---

## Note: a reframing recommendation for Ch.18 §18.4

Seen from this chapter, the title of Ch.18 §18.4, "The overheating and failure of Semantic SLAM," reads differently. Dense dynamic SLAM, change-aware SLAM, and deformable SLAM achieved real success between 2020 and 2025 by using semantic as an *auxiliary cue*. What failed was the prediction, in the SLAM++ mode, that "semantic will take over the SLAM frontend" — the **object-as-landmark** route. Narrowing the §18.4 title to "The contraction of the object-as-landmark route" and cross-referencing this chapter would be the natural revision. The concrete edits will be handled separately in Phase D3-B.

---

The question Ch.15b has been circling, what is the right representation for a world that changes, connects back to the main line at a different level. In the GS-SLAM and NeRF-SLAM lineages, the representation problem was approached by making rendering faster or more compact. Ch.16 approaches it differently: not by improving the representation pipeline, but by learning the geometry prior itself. DUSt3R and its successors begin there.
