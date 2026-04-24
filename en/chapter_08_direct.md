# Ch.8 — The Direct Lineage: From DTAM to DSO

Richard Newcombe was Andrew Davison's doctoral student. Having witnessed MonoSLAM's 30-landmark ceiling firsthand at Imperial College, in 2011 he made the opposite bet—use every pixel. Where Davison had proved real-time viability by leaning on the EKF's logic that "tracking a few points is enough," Newcombe strapped on a single GPU and showed that real-time was still possible even when the full frame was in play. DTAM is a direct descendant of MonoSLAM, but its methodological DNA is inverted.

The ORB-SLAM lineage from Ch.7 was a method of extracting features first and tracking only those features. Harris corners and ORB descriptors filtered the image down to a few hundred points, and the rest of the pixels were thrown away. The direct lineage refused this trade. There are no pixels to discard—the image itself is the measurement.

In Munich that same year, Daniel Cremers was walking a different path. He was porting computer vision's variational machinery (Gauss-Newton image alignment, the formal language of optical flow) into SLAM as a whole. Cremers's student Jakob Engel delivered LSD-SLAM in 2014 and DSO in 2016. The two papers posed the same question at different densities. What happens if, instead of extracting features, you compare pixel intensities directly?

---

## 1. Every Pixel: DTAM

[Newcombe, Lovegrove & Davison 2011. DTAM](https://doi.org/10.1109/ICCV.2011.6126513), presented at ICCV 2011, stands for "Dense Tracking and Mapping in Real-Time." As the name says, it does tracking and mapping at once, using every pixel, in real time.

The system has two parts. The tracking stage performs photometric alignment by comparing the whole current frame against a cost volume. No feature extraction, no descriptor matching—only pixel intensity differences are minimized. The mapping stage estimates a depth map via multi-baseline stereo and maintains a smooth dense 3D model with total variation regularization.

$$E(\mathbf{u}) = \sum_{i} \rho\left( I_i\bigl(\pi(KT_i\mathbf{p}(\mathbf{u}))\bigr) - I_r\bigl(\pi(\mathbf{p}(\mathbf{u}))\bigr) \right) + \lambda \,\text{TV}(\mathbf{u})$$

Here $\mathbf{u}$ is the inverse depth map, $\mathbf{p}(\mathbf{u})$ the 3D point obtained by back-projecting $\mathbf{u}$, $K$ the camera intrinsic matrix, $T_i$ the rigid body transform of frame $i$ relative to the reference frame, $\pi$ the perspective projection, $\rho$ the Huber loss, and $\text{TV}(\mathbf{u}) = \|\nabla \mathbf{u}\|_1$ the total variation regularizer. Running this optimization in real time requires a GPU. DTAM did not hide that premise. It ran on a single Nvidia GTX 480 (the commodity system configuration described in the paper's §3).

> 🔗 **Borrowed.** DTAM's dense volumetric approach took partial inspiration from depth-camera work, in particular the TSDF idea of [Curless & Levoy 1996](https://doi.org/10.1145/237170.237269), but the key difference was its application to a monocular camera. A reverse current then followed: [KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378) (2011, ISMAR), led by Newcombe himself, completed the same idea in depth-sensor form.

Video of an entire indoor scene being reconstructed in real time went up on YouTube right after the 2011 ICCV talk and racked up tens of thousands of views. The weaknesses were equally plain. It did not run without a GPU and it was fragile under lighting changes. Scaling it to large outdoor environments was out of reach.

<!-- DEMO: dtam_photometric_residual.html -->

---

## 2. Tracking the Edges: LSD-SLAM

[Engel, Schöps & Cremers 2014. LSD-SLAM](https://doi.org/10.1007/978-3-319-10605-2_54) gave up DTAM's dense formulation and dropped the GPU dependency along with it. "Large-Scale Direct Monocular SLAM" works in semi-dense mode, tracking only those pixels whose gradient magnitude exceeds a threshold. Flat regions of a wall are ignored; only pixels near edges with sufficient gradient survive. No corner detector is used—the strength of the intensity gradient is the sole criterion for pixel selection.

The tracking stage is a direct image alignment in SE(3). The current frame is warped directly onto a keyframe and the photometric residual is minimized by Gauss-Newton. The map is keyframe-based, and each keyframe carries its own semi-dense depth map. Keyframe connections are maintained as a pose graph, and loop closure finds candidates by appearance-based relocalization and then verifies them with a depth consistency check.

> 🔗 **Borrowed.** Gauss-Newton photometric registration is a classic of the image alignment field. The [Lucas & Kanade 1981](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf) tracker and its inverse compositional reformulation ([Baker & Matthews 2004](https://doi.org/10.1023/B:VISI.0000011205.11775.fd)) are the direct ancestors of LSD-SLAM's frontend. The Cremers group transplanted the language of the variational image-processing community into the entire SLAM pipeline.

The practical significance of LSD-SLAM was that it ran in real time on a CPU. The structure—keyframes only, pose graph optimization—resembled PTAM's tracking/mapping split on the surface, but underneath it was different. There are no binary descriptors like ORB or BRIEF; pixel intensity is the only measurement.

LSD-SLAM also released footage of operation in large outdoor environments. A demo in which a semi-dense map was built while riding a bicycle for tens of meters showed that the direct approach could scale. On the KITTI benchmark it was competitive with the top-tier feature-based methods of the time.

Lighting changes, however, were the problem. Entering a tunnel, backlight through a window, a sudden flash—the moment photometric consistency was assumed, scenes like these destabilized the system immediately.

<!-- DEMO: lsd_slam_semidense.html -->

---

## 3. Sparse Direct Perfected: DSO

[Engel, Koltun & Cremers 2018. DSO (PAMI)](https://doi.org/10.1109/TPAMI.2017.2658577) first appeared on arXiv in 2016. "Direct Sparse Odometry" carries its positioning in the name. More sparse than LSD-SLAM, but with far fewer pixels than DTAM, in exchange for doing photometric calibration thoroughly.

The system selects roughly 2,000 high-gradient pixels in each keyframe. That is more than ORB-SLAM2's default setting (nFeatures=1000), and far fewer than LSD-SLAM's semi-dense set (all pixels with gradient). On these pixels it performs a sliding window bundle adjustment in which the optimization variables include camera pose, inverse depth, and affine brightness parameters $(a_i, b_i)$. Frames that fall out of the window are removed by marginalization, and the Schur complement is used to keep the computational cost at O(N) throughout.

DSO separates the camera's photometric model into three layers. First, vignetting (the falloff in brightness toward the edges of the lens) is corrected through prior calibration. Second, the camera response function (gamma curve, the sensor's non-linear recording of light) is also inverted in advance to convert measurements into a linear intensity domain. Third, the per-frame varying exposure time and affine brightness change are estimated as real-time optimization variables $(t_i, a_i, b_i)$:

$$E_{pj} = \sum_{\mathbf{p} \in \mathcal{N}_p} w_{\mathbf{p}} \left\| \left( I_j\!\left[\mathbf{p}'\right] - \frac{t_j e^{a_j}}{t_i e^{a_i}} I_i[\mathbf{p}] - \left(b_j - \frac{t_j e^{a_j}}{t_i e^{a_i}} b_i\right) \right) \right\|_\gamma$$

Here $t_i, t_j$ are exposure times, $(a_i, b_i)$ and $(a_j, b_j)$ the affine brightness parameters of each frame (gain and bias), and $\|\cdot\|_\gamma$ the Huber loss. Vignetting is corrected in the preprocessing step through photometric calibration, and the residual above is applied to the corrected intensities. Splitting the camera's exposure variation, vignetting, and response curve between a separate calibration stage and real-time optimization variables was something direct SLAM saw for the first time in DSO.

> 🔗 **Borrowed.** The formal basis of photometric camera calibration traces to the HDR-recovery work of [Debevec & Malik 1997](https://doi.org/10.1145/258734.258884). The photometric model they set up to recover a camera response function from multiple photographs was imported by DSO as real-time SLAM optimization variables.

On the TUM monocular dataset, DSO was reported to outperform ORB-SLAM2 across several sequences. In feature-poor environments (indoor corridors with large flat walls) DSO achieved a lower ATE than ORB-SLAM2. This was the empirical grounding for the claim that using photometric information is using more information.

> 📜 **Prediction vs. outcome.** DSO required prior photometric calibration, and that dependency soon became the target of follow-up work. One direction came in 2018 from Bergmann, Wang, and Cremers with [online photometric calibration](https://doi.org/10.1109/LRA.2017.2777002)—estimate exposure, response, and vignetting jointly during the SLAM run rather than in advance. Even so, the deployment barrier from the end-user perspective remained as of 2026. The process of reliably extracting photometric parameters from consumer cameras has not been fully automated and still requires per-camera presetting. `[in progress]`

> 📜 **Prediction vs. outcome.** DTAM was real-time dense SLAM that leaned on a single GPU, and widening access to dense reconstruction sat as the natural next task. The path to realization was not a straight line. Pure monocular dense did not arrive in real-time-deployable form until NeRF and 3DGS emerged in the 2020s. Instead, KinectFusion, led by Newcombe himself, completed GPU dense reconstruction using an RGB-D depth sensor right in 2011—routing around the problem by swapping the sensor. `[diverted]`

---

## 4. VI-DSO and the Lineage Extended

In 2018, von Stumberg, Usenko, and Cremers presented [VI-DSO](https://doi.org/10.1109/ICRA.2018.8462905), which combined DSO with an IMU, at ICRA 2018. The motivation was simple. In the failure mode that hurt photometric direct methods most—rapid lighting change—inertial measurements from an IMU could support pose tracking. The IMU could also resolve the scale ambiguity of the monocular camera.

VI-DSO adds an IMU preintegration factor to DSO's windowed photometric bundle adjustment. The IMU preintegration scheme was borrowed from [Forster et al.'s 2017 paper](https://doi.org/10.1109/TRO.2016.2597321). The result: scale was recovered, and robustness improved under extreme lighting.

Follow-up work from the Cremers group, [Basalt](https://arxiv.org/abs/1904.06504) (2019) and [DM-VIO](https://doi.org/10.1109/LRA.2021.3140129) (2022), continued in the same direction. The structure is a direct photometric frontend with a tightly coupled inertial backend on top. This lineage proceeded in parallel with feature-based VIO (VINS-Mono, OpenVINS), and each formed its own ecosystem.

> 🔗 **Borrowed.** VI-DSO's IMU preintegration uses the manifold preintegration formulation of [Forster et al. 2017. On-Manifold Preintegration (IEEE TRO)](https://doi.org/10.1109/TRO.2016.2597321) directly. It is a stacked structure: Forster's inertial layer placed on top of DSO's photometric layer.

---

## 5. Limits of the Direct Method

Direct methods use, in principle, more information. They pull the pixels that feature detectors throw away—regions where the gradient is low but consistent—into tracking. The photometric residual gives a continuous optimization landscape without the discretization that descriptor matching imposes.

And yet, as of 2026, the majority of deployed systems are feature-based. The reasons sit in several layers.

First, the dependence on photometric calibration. The vignetting correction, response curve correction, and exposure control that DSO assumes are not simply available from a consumer camera. Smartphone cameras apply HDR fusion, auto-exposure, and real-time white balance internally, and that pipeline is not exposed to the user. DSO's photometric model breaks its basic assumptions on these cameras.

Second, lighting change. Under auto-exposure or backlight—situations where inter-frame brightness changes sharply—the direct method's core assumption of photometric consistency collapses immediately. DSO's affine brightness model can only absorb gradual drift, so scenes where a cloud passes outdoors or a fluorescent tube flickers indoors remained a leading cause of tracking failure.

Third, even where DSO beat ORB-SLAM2 on controlled-dataset sequences, engineers putting something on an actual robot picked ORB-SLAM. ORB-SLAM runs on many camera models without separate photometric calibration. Swap the camera and it still works. DSO required per-camera vignetting and response-curve calibration.

Fourth, learned features such as [SuperPoint](https://arxiv.org/abs/1712.07629) (2018) and [LightGlue](https://arxiv.org/abs/2306.13643) (2023) blunted the direct method's central critique that "features discard information." They preserve far more information than handcrafted descriptors did while keeping the practical advantages of descriptor matching. At the very point where direct was attacking feature-based, learned features filled the gap.

<!-- DEMO: photometric_calibration_demo.html -->

---

## 🧭 Still open

**Direct tracking under rapid lighting change.** The foundational premise of the direct method—that the brightness distribution of a scene is preserved across frames—collapses immediately under auto-exposure cameras, strong backlight, or tunnel-to-outdoor transitions. VI-DSO's IMU assistance eases this partially, but a full solution that dynamically estimates the lighting model itself does not yet exist. Learning-based photometric correction is being explored as an alternative, but has not arrived in a real-time-deployable form.

**The double weakness of textureless + direct.** Feature-based methods fail in front of a wall with no corners. Direct methods see the residual vanish on surfaces without gradient. Both approaches are weak in indoor corridors, large warehouses, and homogeneous outdoor terrain. Semi-dense LSD-SLAM hedged by selectively using pixels that do have gradient, but it did not solve the degeneracy that arises when those pixels are not distributed densely enough.

**A possible transition to a learned photometric model.** Current direct SLAM's photometric model is expressed as a simple affine brightness correction or a fixed camera response function. Work in the neural radiance field family is exploring scene appearance as a neural network. Whether this can enter the photometric layer of real-time direct SLAM, and if it does, where the boundary between direct and learned will be drawn, are open questions as of 2026.

Meanwhile, running parallel to the direct lineage, a different exit had already been opened in 2011. Newcombe himself showed it via KinectFusion. Rather than keeping the photometric assumptions of a monocular camera, change the sensor. An RGB-D camera, which measures depth directly, enabled dense reconstruction regardless of brightness change. Where the direct method tried to uphold photometric consistency through its equations, RGB-D struck the assumption itself off the list of questions.
