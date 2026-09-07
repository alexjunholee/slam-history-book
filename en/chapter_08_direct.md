# Ch.8 — The Direct Lineage: From DTAM to DSO

Richard Newcombe was Andrew Davison's doctoral student. After seeing MonoSLAM's 30-landmark ceiling firsthand at Imperial College, he took the opposite approach in 2011: use every pixel. Davison had demonstrated real-time operation with the EKF premise that tracking a few points was sufficient. With a single GPU, Newcombe showed that a system could also operate in real time on the full image. DTAM descended directly from MonoSLAM while reversing its methodological choice.

The ORB-SLAM lineage in Ch.7 extracted features first and tracked only those features. Harris corners and ORB descriptors reduced an image to a few hundred points; the remaining pixels were discarded. Direct methods use pixel-intensity differences as measurements rather than geometric errors between matched features. They include dense methods using all pixels and sparse methods selecting only a subset.

In Munich that same year, Daniel Cremers was pursuing a related approach. He brought the variational machinery of computer vision, including Gauss-Newton image alignment and the formalism of optical flow, into a complete SLAM system. Cremers's student Jakob Engel introduced LSD-SLAM in 2014 and DSO in 2016. At different sampling densities, both methods compared pixel intensities directly instead of extracting features.

---

## 1. Every Pixel: DTAM

[Newcombe, Lovegrove & Davison 2011. DTAM](https://doi.org/10.1109/ICCV.2011.6126513), presented at ICCV 2011, stands for "Dense Tracking and Mapping in Real-Time." It performs tracking and mapping simultaneously, using every pixel while operating in real time.

The system has two parts. The tracking stage performs photometric alignment by comparing the entire current frame with a cost volume. It extracts no features and matches no descriptors, minimizing only pixel-intensity differences. The mapping stage estimates a depth map through multi-baseline stereo and maintains a smooth, dense 3D model with total variation regularization.

$$E(\mathbf{u}) = \sum_{i} \rho\left( I_i\bigl(\pi(KT_i\mathbf{p}(\mathbf{u}))\bigr) - I_r\bigl(\pi(\mathbf{p}(\mathbf{u}))\bigr) \right) + \lambda \,\text{TV}(\mathbf{u})$$

Here $\mathbf{u}$ is the inverse depth map, $\mathbf{p}(\mathbf{u})$ the 3D point obtained by back-projecting $\mathbf{u}$, $K$ the camera intrinsic matrix, $T_i$ the rigid body transform of frame $i$ relative to the reference frame, $\pi$ the perspective projection, $\rho$ the Huber loss, and $\text{TV}(\mathbf{u}) = \|\nabla \mathbf{u}\|_1$ the total variation regularizer. Running this optimization in real time requires a GPU. DTAM used a single Nvidia GTX 480, the commodity configuration described in the paper's §3.

> 🔗 **Borrowed.** DTAM's dense volumetric approach drew partly on depth-camera research, particularly the TSDF formulation of [Curless & Levoy 1996](https://doi.org/10.1145/237170.237269), but applied it to a monocular camera. [KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378) (2011, ISMAR), also led by Newcombe, developed the corresponding depth-sensor approach.

A video of an entire indoor scene being reconstructed in real time appeared on YouTube shortly after the 2011 ICCV talk and received tens of thousands of views. The limitations were equally visible: the system required a GPU, was fragile under lighting changes, and could not scale to large outdoor environments.

<!-- DEMO: dtam_photometric_residual.html -->

---

## 2. Tracking the Edges: LSD-SLAM

[Engel, Schöps & Cremers 2014. LSD-SLAM](https://doi.org/10.1007/978-3-319-10605-2_54) replaced DTAM's dense formulation with a semi-dense one and eliminated its dependence on a GPU. "Large-Scale Direct Monocular SLAM" tracks only pixels whose gradient magnitude exceeds a threshold. It ignores flat wall regions and retains pixels near edges with sufficient gradient. Rather than using a corner detector, it selects pixels solely by the strength of their intensity gradient.

The tracking stage performs direct image alignment in SE(3). It warps the current frame onto a keyframe and minimizes the photometric residual with Gauss-Newton optimization. The map is keyframe-based, and each keyframe carries its own semi-dense depth map. A pose graph maintains connections between keyframes. Loop closure finds candidates through appearance-based relocalization and verifies them with a depth-consistency check.

> 🔗 **Borrowed.** Gauss-Newton photometric registration is a classic of the image alignment field. The [Lucas & Kanade 1981](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf) tracker and its inverse compositional reformulation ([Baker & Matthews 2004](https://doi.org/10.1023/B:VISI.0000011205.11775.fd)) are the direct ancestors of LSD-SLAM's frontend. The Cremers group transplanted the language of the variational image-processing community into the entire SLAM pipeline.

LSD-SLAM ran in real time on a CPU. Its use of keyframes and pose-graph optimization superficially resembled PTAM's tracking-and-mapping split, but the underlying measurement differed: LSD-SLAM used pixel intensity rather than binary descriptors such as ORB or BRIEF.

LSD-SLAM also released footage of operation in large outdoor environments. A demo in which a semi-dense map was built while riding a bicycle for tens of meters showed that the direct approach could scale. On the KITTI benchmark, it was competitive with the leading feature-based methods of the time.

Lighting changes remained a serious problem. Entering a tunnel, facing backlight through a window, or encountering a sudden flash violated photometric consistency and could destabilize the system immediately.

<!-- DEMO: lsd_slam_semidense.html -->

---

## 3. Sparse Direct Perfected: DSO

[Engel, Koltun & Cremers 2018. DSO (PAMI)](https://doi.org/10.1109/TPAMI.2017.2658577) first appeared on arXiv in 2016. "Direct Sparse Odometry" is sparser than LSD-SLAM and uses far fewer pixels than DTAM, while applying a more thorough photometric calibration.

The system selects roughly 2,000 high-gradient pixels in each keyframe. This is more than ORB-SLAM2's default setting (nFeatures=1000) and far fewer than LSD-SLAM's semi-dense set of all pixels with a gradient. DSO performs sliding-window bundle adjustment over these pixels, optimizing camera pose, inverse depth, and affine brightness parameters $(a_i, b_i)$. It marginalizes frames that leave the window and uses the Schur complement to eliminate variables and limit the size of the remaining window problem. Total computation depends on window size, pixel count, and iteration count.

DSO separates the camera's photometric model into three layers. First, prior calibration corrects vignetting, the falloff in brightness toward the edges of the lens. Second, it inverts the camera response function, or gamma curve, in advance to convert the sensor's nonlinear light measurements into a linear intensity domain. Third, it uses exposure time $t_i$ as input metadata and estimates the remaining affine brightness changes $(a_i, b_i)$ online:

$$E_{pj} = \sum_{\mathbf{p} \in \mathcal{N}_p} w_{\mathbf{p}} \left\| \left( I_j\!\left[\mathbf{p}'\right] - \frac{t_j e^{a_j}}{t_i e^{a_i}} I_i[\mathbf{p}] - \left(b_j - \frac{t_j e^{a_j}}{t_i e^{a_i}} b_i\right) \right) \right\|_\gamma$$

Here $t_i, t_j$ are exposure times, $(a_i, b_i)$ and $(a_j, b_j)$ are the affine brightness parameters of each frame (gain and bias), and $\|\cdot\|_\gamma$ is the Huber loss. Photometric calibration corrects vignetting during preprocessing, and the residual above is applied to the corrected intensities. DSO was the first direct SLAM system to divide exposure variation, vignetting, and the response curve between a separate calibration stage and real-time optimization variables.

> 🔗 **Borrowed.** The formal basis of photometric camera calibration traces to the HDR-recovery work of [Debevec & Malik 1997](https://doi.org/10.1145/258734.258884). DSO likewise uses response-corrected intensities, but calibrates the response function in advance. The per-frame affine brightness parameters are the quantities adjusted online.

On the TUM monocular dataset, DSO was reported to outperform ORB-SLAM2 across several sequences. In feature-poor environments, such as indoor corridors with large flat walls, DSO achieved a lower ATE than ORB-SLAM2. These results supported the claim that photometric methods retain information discarded by feature extraction.

> 📜 **Prediction vs. outcome.** DSO required prior photometric calibration, and follow-up work soon addressed that dependency. In 2018, Bergmann, Wang, and Cremers proposed [online photometric calibration](https://doi.org/10.1109/LRA.2017.2777002), jointly estimating exposure time, the response function, and vignetting attenuation while the system ran. On the evaluated auto-exposure videos, the paper reported real-time calibration and VO accuracy on par with pre-calibrated input. This follow-up directly answered DSO's dependence on prior photometric calibration.

> 📜 **Prediction vs. outcome.** DTAM achieved real-time dense SLAM on a single GPU, leaving broader access to dense reconstruction as a natural next step. Pure monocular dense reconstruction did not become deployable in real time until NeRF and 3DGS emerged in the 2020s. KinectFusion, also led by Newcombe, instead achieved GPU-based dense reconstruction with an RGB-D depth sensor in 2011 by changing the sensor rather than solving the monocular problem.

---

## 4. VI-DSO and the Lineage Extended

At ICRA 2018, von Stumberg, Usenko, and Cremers presented [VI-DSO](https://doi.org/10.1109/ICRA.2018.8462905), which combined DSO with an IMU. Inertial measurements could support pose tracking during rapid lighting changes, a major failure mode for direct photometric methods. The IMU could also resolve the scale ambiguity of a monocular camera.

VI-DSO adds an IMU preintegration factor to DSO's windowed photometric bundle adjustment. The IMU preintegration scheme was borrowed from [Forster et al.'s 2017 paper](https://doi.org/10.1109/TRO.2016.2597321). This recovered scale and improved robustness under extreme lighting.

Follow-up work from the Cremers group, [Basalt](https://arxiv.org/abs/1904.06504) (2019) and [DM-VIO](https://doi.org/10.1109/LRA.2021.3140129) (2022), continued in the same direction. Both paired a direct photometric frontend with a tightly coupled inertial backend. This lineage developed in parallel with feature-based VIO systems such as VINS-Mono and OpenVINS, each with its own ecosystem.

> 🔗 **Borrowed.** VI-DSO directly uses the manifold preintegration formulation of [Forster et al. 2017. On-Manifold Preintegration (IEEE TRO)](https://doi.org/10.1109/TRO.2016.2597321), placing Forster's inertial layer above DSO's photometric layer.

---

## 5. Limits of the Direct Method

Direct methods retain more image information. They use pixels that feature detectors discard, including regions whose gradients are low but consistent. The photometric residual also provides a continuous optimization landscape without the discretization imposed by descriptor matching.

As of 2026, however, most deployed systems remain feature-based for several reasons.

Direct methods depend on photometric calibration. The vignetting correction, response-curve correction, and exposure control assumed by DSO are not readily available from a consumer camera. Smartphone cameras apply HDR fusion, auto-exposure, and real-time white balance internally without exposing that pipeline to the user. These processes violate DSO's photometric assumptions.

Lighting changes remain difficult as well. Under auto-exposure or backlight, inter-frame brightness can change sharply and violate the direct method's assumption of photometric consistency. DSO's affine brightness model can absorb only gradual drift, so a passing cloud outdoors or a flickering fluorescent tube indoors remains a leading cause of tracking failure.

Although DSO beat ORB-SLAM2 on sequences from controlled datasets, engineers deploying systems on robots often chose ORB-SLAM. It runs on many camera models without separate photometric calibration and can continue working after a camera change. DSO requires vignetting and response-curve calibration for each camera.

Learned features such as [SuperPoint](https://arxiv.org/abs/1712.07629) (2018) and [LightGlue](https://arxiv.org/abs/2306.13643) (2023) also weakened the direct method's central criticism that features discard information. They retain more information than handcrafted descriptors while preserving the practical advantages of descriptor matching.

<!-- DEMO: photometric_calibration_demo.html -->

---

## 🧭 Still open

**Direct tracking under rapid lighting change.** The direct method assumes that a scene's brightness distribution remains stable across frames, an assumption violated by auto-exposure, strong backlight, and tunnel-to-outdoor transitions. VI-DSO's IMU assistance partially mitigates the problem, but no complete solution dynamically estimates the lighting model itself. Learning-based photometric correction is being explored as an alternative but is not yet deployable in real time.

**The shared weakness on textureless surfaces.** Feature-based methods fail on walls without corners, while the photometric residual in direct methods becomes largely insensitive to pose changes on surfaces without gradients. Both approaches are weak in indoor corridors, large warehouses, and homogeneous outdoor terrain. Semi-dense LSD-SLAM retained only pixels with a gradient, but it did not resolve the degeneracy that arises when those pixels are too sparsely distributed.

**A possible transition to a learned photometric model.** Current direct SLAM systems represent the photometric model with a simple affine brightness correction or a fixed camera response function. Neural radiance field research instead represents scene appearance with a neural network. Whether such a model can become part of real-time direct SLAM, and where that would place the boundary between direct and learned methods, remain open questions as of 2026.

A parallel approach had already appeared in 2011 through Newcombe's KinectFusion: replace the monocular camera with a different sensor. By measuring depth directly, an RGB-D camera enabled dense reconstruction without relying on brightness consistency. Direct methods tried to model photometric consistency, whereas RGB-D systems removed that assumption by changing the measurement source.
