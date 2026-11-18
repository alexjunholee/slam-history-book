# Chapter 11. Direct Methods: From DTAM to DSO

Richard Newcombe was Andrew Davison's doctoral student at Imperial College London. Having witnessed MonoSLAM's 30-landmark ceiling firsthand, Newcombe inverted its core premise in 2011: instead of distilling an image down to a handful of corners, why not use every pixel? Davison had achieved real-time CPU operation under the assumption that a few sparse points sufficed; backed by a single GPU, Newcombe proved that an estimator could also run in real time over the entire image. DTAM descended directly from MonoSLAM while reversing its most fundamental front-end choice.

Whereas Chapters 8 through 10 tracked sparse geometric features and coupled them with inertial or continuous-time motion models, the feature-based paradigm still threw away almost all visual data: Harris corners and ORB descriptors compressed a frame to a few hundred points and ignored the rest. **Direct methods** instead minimize raw pixel-intensity residuals directly, spanning fully dense pipelines that optimize every pixel and sparse direct pipelines that select an informative subset.

In Munich that same year, Daniel Cremers was building toward a parallel vision. Drawing on the variational machinery of computer vision—Gauss-Newton image alignment and optical-flow regularization—Cremers and his student Jakob Engel introduced LSD-SLAM in 2014 and DSO in 2016. Across different sampling densities, both camps tested the same hypothesis: image brightness carries richer geometric signal than extracted keypoints.

---

## 11.1 Every Pixel: DTAM

Presented at ICCV 2011, [Newcombe, Lovegrove & Davison 2011. DTAM](https://doi.org/10.1109/ICCV.2011.6126513) ("Dense Tracking and Mapping in Real-Time") demonstrated simultaneous tracking and mapping over every pixel at frame rate.

DTAM splits cleanly into two tightly coupled stages. For tracking, whole-frame photometric alignment compares the live image against a synthetic view rendered from the current model, optimizing camera pose solely from pixel-intensity differences without extracting a single feature or descriptor. For mapping, multi-baseline stereo accumulates a dense cost volume per keyframe and solves for a continuous inverse-depth map regularized by total variation:

$$E(\mathbf{u}) = \sum_{i} \rho\left( I_i\bigl(\pi(KT_i\mathbf{p}(\mathbf{u}))\bigr) - I_r\bigl(\pi(\mathbf{p}(\mathbf{u}))\bigr) \right) + \lambda \,\text{TV}(\mathbf{u})$$

Here $\mathbf{u}$ denotes the inverse-depth map, $\mathbf{p}(\mathbf{u})$ the 3D point back-projected from $\mathbf{u}$, $K$ the camera intrinsics, $T_i$ the rigid transform from the reference frame to frame $i$, and $\pi$ perspective projection. The first term sums the photometric discrepancy under a robust Huber penalty $\rho$, while the second term $\text{TV}(\mathbf{u}) = \|\nabla \mathbf{u}\|_1$ smooths depth noise while preserving sharp object boundaries. Executing this dense optimization in real time demanded GPU parallelism—specifically a single Nvidia GTX 480 in the paper's §3 configuration.

> 🔗 **Borrowed.** DTAM's dense volumetric thinking drew partly on surface-fusion methods from range imaging, notably the TSDF framework of [Curless & Levoy 1996](https://doi.org/10.1145/237170.237269), adapted here to passive monocular video. That same year, [KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378) (ISMAR 2011), also led by Newcombe, brought the volumetric approach directly to depth cameras.

Videos of DTAM reconstructing cluttered indoor scenes in real time drew tens of thousands of views shortly after the ICCV 2011 talk. Yet its vulnerabilities were equally clear: heavy GPU dependence, fragility under illumination shifts, and an inability to scale beyond small workspaces.

<!-- DEMO: dtam_photometric_residual.html -->

---

## 11.2 Tracking the Edges: LSD-SLAM

Three years later, [Engel, Schöps & Cremers 2014. LSD-SLAM](https://doi.org/10.1007/978-3-319-10605-2_54) ("Large-Scale Direct Monocular SLAM") broke DTAM's dependence on GPUs by moving from dense to **semi-dense** reconstruction. Rather than estimating depth on flat, uninformative walls, LSD-SLAM restricts tracking and mapping to pixels whose spatial intensity gradient exceeds a threshold—retaining edges and texture transitions directly from gradient magnitude without running a corner detector.

Tracking warps the current frame onto the active keyframe via direct SE(3) image alignment, minimizing photometric residuals with Gauss-Newton optimization. Each keyframe stores its own semi-dense inverse-depth map inside a pose graph, and loop closures are proposed via appearance-based relocalization and verified through depth consistency.

> 🔗 **Borrowed.** Direct Gauss-Newton image alignment stands on a deep lineage in computer vision: the [Lucas & Kanade 1981](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf) tracker and its inverse-compositional reformulation by [Baker & Matthews 2004](https://doi.org/10.1023/B:VISI.0000011205.11775.fd) are the direct mathematical ancestors of LSD-SLAM's front end, transplanted by Cremers's group from optical flow into full-pipeline SLAM.

Crucially, LSD-SLAM ran in real time on a standard CPU. While its keyframe and pose-graph architecture echoed PTAM's tracking–mapping split on the surface, its underlying residual remained purely photometric rather than relying on binary descriptors like ORB or BRIEF.

Field demos cycling tens of meters outdoors proved that direct monocular SLAM could scale beyond desktop workspaces, and LSD-SLAM posted competitive results on KITTI. Even so, abrupt lighting changes remained an Achilles' heel: entering a tunnel, facing bright window backlight, or catching a sudden flash broke brightness constancy and could destabilize tracking immediately.

<!-- DEMO: lsd_slam_semidense.html -->

---

## 11.3 Sparse Direct Perfected: DSO

First posted on arXiv in 2016 and published in *PAMI* two years later, [Engel, Koltun & Cremers 2018. DSO (PAMI)](https://doi.org/10.1109/TPAMI.2017.2658577) ("Direct Sparse Odometry") moved even further along the sparsity spectrum—sampling far fewer pixels than LSD-SLAM or DTAM while coupling them with rigorous photometric calibration and windowed bundle adjustment.

DSO samples roughly 2,000 candidate pixels per keyframe, distributed across the image by local gradient strength relative to surrounding blocks, and jointly maintains about 2,000 active points across all keyframes in its sliding optimization window. Rather than freezing past depths, DSO runs full sliding-window photometric bundle adjustment over camera poses, inverse depths, and per-frame affine brightness parameters $(a_i, b_i)$, marginalizing old frames via the Schur complement so window cost stays bounded by window size, active point count, and solver iterations.

To make brightness comparisons physically faithful, DSO models the camera imaging pipeline in three explicit layers: offline calibration removes lens vignetting and inverts the nonlinear camera response (gamma) curve into linear irradiance, while exposure times $t_i$ (when known) combine with online affine brightness parameters $(a_i, b_i)$ inside the photometric residual over a local pixel patch:

$$E_{pj} = \sum_{\mathbf{p} \in \mathcal{N}_p} w_{\mathbf{p}} \left\| \left( I_j\!\left[\mathbf{p}'\right] - \frac{t_j e^{a_j}}{t_i e^{a_i}} I_i[\mathbf{p}] - \left(b_j - \frac{t_j e^{a_j}}{t_i e^{a_i}} b_i\right) \right) \right\|_\gamma$$

Here $t_i, t_j$ are the frame exposure times, $(a_i, b_i)$ and $(a_j, b_j)$ are the per-frame affine gain and bias parameters, and $\|\cdot\|_\gamma$ denotes the Huber loss applied to vignetting- and response-corrected intensities. DSO was the first direct SLAM system to separate static optical calibration from dynamic per-frame photometric state cleanly inside real-time optimization.

> 🔗 **Borrowed.** Separating nonlinear camera response recovery from linear irradiance traces back to [Debevec & Malik 1997](https://doi.org/10.1145/258734.258884) in high-dynamic-range imaging. DSO inherited that photometric response formulation, calibrating the static response curve offline while optimizing per-frame affine brightness parameters online.

On the TUM Monocular benchmark, DSO outperformed ORB-SLAM2 across numerous sequences, achieving noticeably lower ATE in feature-starved indoor corridors with blank walls—compelling evidence that direct residuals exploit subtle edge and shading cues discarded by keypoint detectors.

> 📜 **Prediction vs. outcome.** Because DSO depended on prior photometric calibration, follow-up work quickly targeted removing that setup step. In 2018, Bergmann, Wang, and Cremers introduced [online photometric calibration](https://doi.org/10.1109/LRA.2017.2777002), jointly recovering exposure times, the response function, and vignetting attenuation on the fly—achieving real-time calibration and VO accuracy matching pre-calibrated runs on auto-exposure video.

> 📜 **Prediction vs. outcome.** Although DTAM achieved real-time dense tracking on a single GPU, it did not immediately make passive monocular dense reconstruction practical for deployment; real-time dense monocular mapping had to wait for the NeRF and 3D Gaussian Splatting wave of the 2020s. Instead, Newcombe's parallel 2011 system KinectFusion achieved immediate GPU-based dense reconstruction by swapping in an active RGB-D depth sensor.

---

## 11.4 VI-DSO and the Lineage Extended

At ICRA 2018, von Stumberg, Usenko, and Cremers bridged direct odometry and inertial sensing in [VI-DSO](https://doi.org/10.1109/ICRA.2018.8462905). Adding an IMU resolved monocular scale ambiguity while giving the tracker an inertial bridge across sudden exposure swings and motion blur—precisely where pure photometric alignment falters.

VI-DSO embeds the manifold IMU preintegration factor from [Forster et al.'s 2017 paper](https://doi.org/10.1109/TRO.2016.2597321) directly inside DSO's sliding-window photometric bundle adjustment, stabilizing metric scale and boosting resilience under challenging illumination.

Cremers's group extended this direct–inertial marriage in [Basalt](https://arxiv.org/abs/1904.06504) (2019) and [DM-VIO](https://doi.org/10.1109/LRA.2021.3140129) (2022), coupling photometric tracking tightly with inertial optimization (and, in DM-VIO, delayed marginalization). Through the late 2010s and early 2020s, this direct VIO lineage evolved alongside feature-based VIO systems like VINS-Mono and OpenVINS, each cultivating its own research ecosystem.

> 🔗 **Borrowed.** VI-DSO incorporates [Forster et al. 2017. On-Manifold Preintegration (IEEE TRO)](https://doi.org/10.1109/TRO.2016.2597321) verbatim as its inertial layer, stacking Forster's SO(3) preintegration directly atop DSO's photometric bundle adjustment.

---

## 11.5 Limits of the Direct Method

Direct methods differ in how densely they sample the image—DTAM uses every pixel, LSD-SLAM retains high-gradient edges, and DSO spreads points across local intensity gradients while relaxing thresholds in low-contrast patches—yet all share the same underlying residual: raw brightness. Even where corners are scarce, intensity gradients still constrain pose, though truly uniform surfaces leave photometric residuals insensitive to camera motion.

Despite DSO's benchmark wins on controlled datasets, however, most production SLAM systems as of 2026 remain feature-based for three practical reasons.

First is photometric fragility on consumer hardware. DSO assumes calibrated vignetting, a stable response curve, and predictable exposure, whereas smartphone and embedded ISPs run opaque multi-frame HDR fusion, auto-exposure, and dynamic white balance on the fly—breaking those calibration assumptions out of the box.

Second is sensitivity to abrupt illumination changes. Under aggressive auto-exposure or strong backlight, brightness can jump discontinuously from frame to frame; because DSO's affine model absorbs only smooth global gain and bias shifts, a passing cloud outdoors or a flickering fluorescent lamp indoors can still trigger tracking loss.

Third is deployment portability. Field robotics teams frequently swap lenses or camera modules, where ORB-SLAM runs out of the box across diverse sensors while DSO expects per-camera vignetting and response calibration. Meanwhile, learned keypoints and matchers such as [SuperPoint](https://arxiv.org/abs/1712.07629) (2018) and [LightGlue](https://arxiv.org/abs/2306.13643) (2023) blunted the direct camp's core critique by capturing richer visual context than handcrafted descriptors while keeping descriptor matching's modularity.

<!-- DEMO: photometric_calibration_demo.html -->

---

## Still open

**Direct tracking under rapid lighting change.** Photometric consistency breaks down under auto-exposure, harsh backlight, and tunnel exits. While VI-DSO's inertial prior cushions brief outages, no mature pipeline dynamically models spatially varying illumination in real time, and learned photometric normalization remains too heavy for embedded frame rates.

**The shared weakness on textureless surfaces.** Feature trackers lose corners on blank walls, yet direct photometric residuals likewise flatten out wherever intensity gradients vanish—leaving corridors, empty warehouses, and uniform snowfields hazardous for both schools. Semi-dense LSD-SLAM sidestepped flat patches, but could not cure geometric degeneracy when remaining gradient pixels clustered too sparsely.

**A possible transition to a learned photometric model.** Classical direct SLAM models scene appearance through simple affine scalars or pre-calibrated response curves, whereas neural radiance fields represent appearance with learned networks. How such learned photometric models might enter real-time direct tracking—and where that blurs the boundary between direct and learned SLAM—remains an open question as of 2026.

---

Even as Newcombe and Engel were pushing passive photometric alignment to its limits, a parallel solution had already emerged in 2011 through Newcombe's own KinectFusion: rather than wresting dense depth from brightness constancy in monocular video, change the sensor itself. By measuring 3D depth directly, RGB-D cameras unlocked real-time dense reconstruction without photometric assumptions—the lineage traced next in Chapter 12.