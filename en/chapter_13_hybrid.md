# Chapter 16. Combining Learning and Geometry: From CodeSLAM to DROID-SLAM

When Michael Bloesch presented CodeSLAM at CVPR 2018, he was working under Andrew Davison at Imperial College London's Dyson Robotics Lab—the very group where Richard Newcombe had built DTAM in 2011 and where Jan Czarnowski (DeepFactors, 2020), Edgar Sucar, and Tristan Laidlow would soon carry dense learned mapping forward. Bloesch's system bridged two traditions that had rarely spoken the same language: Davison's view of SLAM as probabilistic inference, honed since 2002, and the mid-2010s conviction that visual scene representations could be learned from data.

---

## 16.1 CodeSLAM — latent code and the map

Traditional monocular SLAM treated depth directly as an optimization variable, whether across a few hundred sparse landmarks or, as in [DTAM](https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf) (Newcombe et al. 2011), at every pixel. That choice meant the state space scaled with image resolution: a single 640×480 dense depth map packed 307,200 independent variables, inflating compute, sharpening sensitivity to initialization, and leaving little room to enforce prior structure.

Rather than optimizing each pixel's depth directly, [Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271) optimized a compact **latent code** that generated the entire depth map. Trained on real depth distributions, a variational autoencoder (VAE) learned a low-dimensional bottleneck approximating the manifold of plausible depth maps. Restricting inference to that manifold compressed the optimization space from hundreds of thousands of per-pixel depths to a few hundred latent variables.

> 🔗 **Borrowed.** CodeSLAM's latent depth representation builds on the encoder-decoder bottleneck introduced in [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114). Although training follows the standard VAE objective, during SLAM inference **z** becomes a deterministic MAP optimization variable without stochastic sampling. A framework conceived for generative image modeling was thus repurposed, years later, as a compact parameterization for geometric SLAM optimization.

In operation, the VAE encoder extracts an initial latent code **z** from each keyframe image, and the decoder reconstructs a dense depth map from **z**. The system then jointly optimizes camera poses and latent codes by minimizing photometric inconsistency across views alongside a prior penalty that pulls **z** toward a standard normal distribution:

$$E(\mathbf{z}, T) = \sum_{i,j} \rho\bigl(I_j(\pi(T_{ij}, D_\mathbf{z}(u_i), u_i)) - I_i(u_i)\bigr) + \lambda \|\mathbf{z}\|^2$$

Here $D_\mathbf{z}$ denotes the decoder, $\pi$ the camera projection, $\rho$ a robust cost function, and $T_{ij}$ the relative pose between keyframes. The regularization term $\lambda\|\mathbf{z}\|^2$ corresponds to the negative log-likelihood of the standard normal prior $p(\mathbf{z}) = \mathcal{N}(0, I)$, arising directly from MAP inference under a Gaussian prior.

> 🔗 **Borrowed.** Dellaert and Kaess's [GTSAM](https://gtsam.org/tutorials/intro.html) factor graph would later provide the backend structure for DeepFactors (§16.2). Czarnowski reformulated CodeSLAM's joint pose-latent optimization as an explicit factor graph in which a learned latent variable appeared alongside conventional pose nodes, letting graph edges capture the coupling between learned and geometric states.

CodeSLAM outperformed earlier methods at recovering dense geometry from sparse views, yet it could not run in real time—both VAE evaluation and the iterative optimization loop were too slow, a limitation the authors acknowledged openly.

> 📜 **Prediction vs. outcome.** CodeSLAM proved that dense SLAM could embed a compact learned representation inside geometric optimization, even though speed and scale remained bottlenecks. DeepFactors (2020), from the same Imperial group, pushed closer to real-time operation without reaching deployment-grade throughput. Teed and Deng at Princeton later achieved broad support across monocular, stereo, and RGB-D inputs via a different architecture altogether: pairing a learned correspondence frontend with Dense Bundle Adjustment (DBA).

---

## 16.2 DeepFactors — Imperial Dyson Lab, factor graph integration

Two years after CodeSLAM, Jan Czarnowski—also advised by Davison at the Imperial Dyson Robotics Lab—released [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2969036) in an effort to turn CodeSLAM's core idea into a complete SLAM pipeline.

DeepFactors embedded CodeSLAM's latent-depth parameterization and joint optimization inside an explicit factor graph, separated tracking from mapping, and added a keyframe-selection policy. On an NVIDIA GTX 1080, frame-to-keyframe tracking ran at roughly 250 Hz, yet computing the network Jacobian took several hundred milliseconds per keyframe and stalled the mapping loop. Even so, DeepFactors proved the design's viability and pointed toward a broader lesson: rather than replacing the entire SLAM stack end to end, learning could be injected selectively into modular geometric optimization.

Around the same time, Daniel Cremers's group at TU Munich reached that same architectural principle from the opposite direction. Where Davison's lab embedded VAE latent codes into a factor graph, Cremers's team injected neural predictions into its 2016 direct sparse odometry framework, [DSO](https://arxiv.org/abs/1607.02565). [Yang, Wang, Stückler, Cremers 2018. DVSO](https://arxiv.org/abs/1807.02570) fed monocular neural depth into DSO as "virtual stereo" observations, and [Yang, von Stumberg, Wang, Cremers 2020. D3VO](https://arxiv.org/abs/2003.01060) went further by adding self-supervised predictions of depth, pose, and uncertainty as extra factors in DSO's graph. Subsequent systems—[Wimbauer et al. 2021. MonoRec](https://arxiv.org/abs/2011.11814) and [Wimbauer et al. 2023. Behind the Scenes](https://arxiv.org/abs/2301.07668)—carried this direct-plus-learned paradigm into dynamic dense reconstruction and single-view density fields. Despite having no institutional overlap with Imperial, Munich arrived at the same hybrid principle: keeping classical optimization intact while feeding it learned priors.

Across the Atlantic, a 2020 optical-flow network from Princeton soon supplied the building block for a very different hybrid SLAM architecture in 2021.

---

## 16.3 RAFT — recurrent optical flow

At ECCV 2020, Zachary Teed and Jia Deng at Princeton introduced [Recurrent All-Pairs Field Transforms (RAFT)](https://arxiv.org/abs/2003.12039). Although built purely for optical-flow estimation rather than SLAM, RAFT's architecture soon became the engine behind DROID-SLAM. Its pipeline rests on three components:

1. **Feature encoder:** a convolutional network extracts dense feature maps from each input image.
2. **Correlation volume:** pixel-to-pixel similarities across all pairs are compiled into a 4D correlation volume organized as a four-level pyramid.
3. **Update operator:** a Gated Recurrent Unit (GRU) repeatedly queries the correlation pyramid and refines the predicted flow field.

The phrase "all-pairs" captures what set RAFT apart. Rather than restricting search to a local neighborhood or relying on coarse-to-fine warping as PWC-Net had done, RAFT evaluates every candidate correspondence and refines the flow field at a fixed one-eighth image resolution before upsampling. Against the strongest published baselines of the time, RAFT cut KITTI F1-all error by 16% and Sintel final-pass endpoint error by 30%.

RAFT originated outside SLAM, yet Teed noticed a striking structural parallel between its recurrent update operator and iterative bundle adjustment: a GRU incrementally refining a pixel flow field looked remarkably like an optimizer iteratively refining camera poses and scene depth.

---

## 16.4 DROID-SLAM — the update operator and BA

At NeurIPS 2021, Teed and Deng turned that insight into [DROID-SLAM](https://arxiv.org/abs/2108.10869)—short for "Differentiable Recurrent Optimization-Inspired Design"—making the division of labor between learning and geometry explicit.

Its frontend mirrors RAFT: a CNN encoder extracts feature maps, builds an all-pairs correlation volume, and runs a GRU update operator to refine dense optical flow. Unlike RAFT, however, DROID-SLAM estimates flow simultaneously across every edge of a keyframe graph rather than between a single isolated image pair. Its backend then runs Dense Bundle Adjustment (DBA) over keyframe poses and per-pixel inverse depths, taking the GRU's 2D flow correspondences as geometric constraints and solving the resulting linear system via the Schur complement. What binds the two halves together is the **DBA layer**: flow estimates and confidence weights from the GRU flow into DBA, and the updated poses and depths feed back to re-anchor the next GRU iteration in a tightly coupled recurrent loop.

> 🔗 **Borrowed.** The core idea behind DBA dates back a decade to Newcombe's DTAM (2011), an early milestone in per-pixel photometric bundle adjustment. DROID-SLAM paired that dense optimization structure with far more resilient correspondences from learned flow, despite having no direct institutional lineage connecting Teed to Newcombe.

> 🔗 **Borrowed.** Teed and Deng carried DROID-SLAM's update operator straight over from RAFT. All-pairs recurrent refinement, originally crafted for two-frame optical flow, proved naturally compatible with the iterative updates of bundle adjustment.

On the EuRoC MAV benchmark, DROID-SLAM achieved a lower RMSE ATE than ORB-SLAM3, the reigning state of the art, while demonstrating strong accuracy on synthetic TartanAir sequences and real indoor and outdoor scenes—especially under lighting shifts and low-texture surfaces where sparse feature matchers falter. In a later Handbook retrospective, Teed noted that on EuRoC `V1_02`, global optimization dropped frontend-only ATE from 16.5 cm to 1.2 cm: classical bundle adjustment reached single-centimeter precision once fed with learned dense correspondences.

That division of labor separated the 2021 hybrid architecture from the 2015 end-to-end wave. Whereas PoseNet tried to regress camera pose directly from pixels without geometric constraints—and failed to generalize—Teed and Deng assigned dense feature extraction and correspondence matching to neural networks while leaving geometric consistency and uncertainty propagation to bundle adjustment. Neural modules replaced hand-crafted descriptors, not the underlying optimization geometry.

---

## 16.5 The Imperial Dyson Lab lineage

How did one laboratory end up bridging so many of these steps? Over the two decades following MonoSLAM in 2002, Andrew Davison's Dyson Robotics Lab at Imperial College London served as an incubator for successive waves of dense and learned SLAM:

- **Richard Newcombe** (Davison advisee, Imperial): DTAM (2011), [KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf) (2011); later Oculus → Meta Reality Labs.
- **Michael Bloesch** (Davison advisee, Imperial): CodeSLAM (2018), tactile and inertial SLAM research.
- **Jan Czarnowski** (Davison advisee, Imperial): DeepFactors (2020).
- **Edgar Sucar** (Davison group, Imperial): [iMAP](https://arxiv.org/abs/2103.12352) (2021), launching the NeRF-SLAM lineage.
- **Tristan Laidlow** (Davison group, Imperial): dense 3D reconstruction, later contributing to neural implicit SLAM.

Across those shifts—from sparse monocular landmarks to dense latent codes and neural implicit fields—the lab held fast to factor graphs and probabilistic uncertainty. In [FutureMapping](https://arxiv.org/abs/1803.11288) (2018) and [FutureMapping 2](https://arxiv.org/abs/1910.14139) (2019, with Ortiz), Davison articulated a vision of Spatial AI in which a map combines multiple geometric and semantic representations inside a single probabilistic graph; CodeSLAM and DeepFactors were early testbeds for that program.

By contrast, Teed and Deng built DROID-SLAM independently at Princeton. Yet Newcombe and Davison's DTAM stood as a clear conceptual precursor to DROID-SLAM's DBA—a reminder that technical lineages often cross-pollinate even without direct institutional ties.

---

## 16.6 2023–2025 — extensions after DROID

After DROID-SLAM, researchers pushed the hybrid paradigm along several distinct tracks—some swapping out its map representation, others accelerating its backend, and still others extending its DBA machinery to unconstrained video.

Several groups targeted the map representation itself. [GO-SLAM](https://arxiv.org/abs/2309.02436) (Zhang et al. 2023) kept DROID-SLAM's dense-flow tracking and augmented it with online loop closure, full bundle adjustment, and an Instant-NGP-style multi-resolution hash-encoded implicit map. [NICER-SLAM](https://arxiv.org/abs/2302.03594) (Zhu et al. 2023) pursued RGB-only dense SLAM from outside the DROID lineage altogether, optimizing tracking and mapping jointly over a single hierarchical neural implicit field. [SplaTAM](https://arxiv.org/abs/2312.02126) (Keetha et al. 2024) shifted to 3D Gaussian Splatting for the map and drove tracking via silhouette-guided differentiable rendering rather than dense optical flow, aligning it more closely with the 3DGS trajectory.

Within Princeton, the original DROID team focused instead on speed and memory. [DPV-SLAM](https://arxiv.org/abs/2408.01654) (Lipson, Teed, Deng 2024), built atop [DPVO](https://github.com/princeton-vl/DPVO) (Deep Patch Visual Odometry), replaced dense pixel grids with sparse learned patches and added proximity-based loop closure plus CUDA block-sparse BA, running roughly 2.5× faster with substantially lower memory than DROID-SLAM.

Meanwhile, a broader 2024–2025 wave emerged around Naver Labs' [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al. 2023), which bypassed classical SfM pipelines by predicting 3D pointmaps directly from image pairs (examined in detail in Chapter 20). The same Revaud group extended pairwise inference to multi-view sequences with working memory in [Cabon et al. 2025. MUSt3R](https://arxiv.org/abs/2503.01661), aiming to handle both offline SfM and online VO/SLAM within one network. Even within this foundation-model ecosystem, DROID-style components resurfaced: [Li et al. 2024. MegaSAM](https://arxiv.org/abs/2412.04463) adapted DROID-SLAM's differentiable DBA to dynamic scenes and uncalibrated video by jointly optimizing camera intrinsics at inference time, and NVIDIA's [Huang et al. 2025. ViPE](https://arxiv.org/abs/2508.10934) fused DROID-SLAM's dense flow, cuvslam's sparse keypoints, and monocular depth predictions inside a single DBA backend to annotate YouTube-scale video. Together, these developments showed that Teed and Deng's 2021 separation of learned correspondence from geometric optimization had become a versatile template—even as parallel lineages like NICER-SLAM and SplaTAM tested whether implicit or Gaussian maps could replace it outright.

> 📜 **Prediction vs. outcome.** DROID-SLAM established an influential reference architecture by coupling differentiable DBA with end-to-end learned correspondence. Three years later, its creators streamlined the design in DPV-SLAM, while GO-SLAM, NICER-SLAM, and SplaTAM swapped the map representation for neural fields or Gaussian splats. By 2026, however, none of these variants on the "learned frontend plus classical backend" recipe had coalesced into a single general-purpose standard.

---

## Still open

**Out-of-distribution generalization.** Learned priors remain tethered to their training distributions. When environments diverge sharply from training data—in wide outdoor spaces, irregular textures, or nighttime scenes—the VAEs in CodeSLAM and DeepFactors can bias optimization toward wrong depths, and DROID-SLAM's flow estimator degrades as well. Diverse synthetic benchmarks such as TartanAir narrow the gap, yet a persistent sim-to-real divide meant that as of 2026 no learned SLAM system worked reliably across all environments.

**Real-time throughput at scale.** On an NVIDIA RTX 2080Ti, DROID-SLAM averages 10–15 fps and slows further as the keyframe graph expands, bottlenecked by DBA. As of 2026, that compute footprint remained out of reach for low-power mobile robots or AR/VR headsets demanding 30 Hz or more, and thinning keyframes or approximating BA inevitably trades away accuracy.

**Learned loop closure.** DROID-SLAM has no dedicated relocalization module; in his Handbook retrospective, Teed noted explicitly that "DROID-SLAM doesn't include any relocalization module, so large loops with lots of drift cannot be closed." Although its backend runs global BA across keyframes, severe accumulated drift prevents it from recovering the correspondences needed to snap large loops shut. Bridging the learned place-recognition lineage of Chapter 13 (such as NetVLAD) with DROID-style factor graphs in Chapter 16 remains an open integration challenge.

---

DROID-SLAM relied on inverse-depth keyframes—a potent dense representation in 2021—just as [NeRF](https://arxiv.org/abs/2003.08934) (Neural Radiance Field) was proposing a radically different alternative from computer graphics: encoding the entire 3D scene as a continuous function rather than as points, lines, planes, or meshes, and enforcing photometric consistency via differentiable rendering.

At Imperial College in 2021, Edgar Sucar reframed that graphics breakthrough as a SLAM problem: could a single MLP replace KinectFusion's TSDF voxel grid altogether? Fourteen months of work produced his answer—iMAP.
