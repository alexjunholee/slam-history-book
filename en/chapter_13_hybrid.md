# Ch.13 — The Hybrid Victory: From CodeSLAM to DROID-SLAM

When Michael Bloesch presented CodeSLAM at CVPR 2018, he was affiliated with the Dyson Robotics Lab at Imperial College London and advised by Andrew Davison. Richard Newcombe had built DTAM in the same lab in 2011. Jan Czarnowski would release DeepFactors there in 2020, and Edgar Sucar and Tristan Laidlow would continue related work. Bloesch combined Davison's view of SLAM as probabilistic inference, developed since 2002, with the mid-2010s premise that representations could be learned.

---

## 13.1 CodeSLAM — latent code and the map

Traditional monocular SLAM estimated depth as an optimization variable, whether for a few hundred sparse landmarks or, as in [DTAM](https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf) (Newcombe et al. 2011), for every pixel. The dimension of this variable space grew with image resolution. A 640×480 dense depth map for one keyframe contains 307,200 independent variables, making optimization expensive, initialization sensitive, and prior information difficult to incorporate.

[Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271) optimized a low-dimensional **latent code** that generated the depth map rather than optimizing each depth value directly. A variational autoencoder (VAE) trained on real depth distributions learned a bottleneck space that approximated the manifold of realistic depth maps. Restricting optimization to this manifold reduced the number of variables from hundreds of thousands to hundreds.

> 🔗 **Borrowed.** CodeSLAM's latent depth representation uses the encoder-decoder latent-space structure established in [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114). Training follows the VAE framework, but during SLAM inference, **z** becomes a MAP optimization variable without stochastic sampling. A representation developed for generative image models became, a decade later, a low-dimensional space for SLAM optimization.

For each keyframe, a VAE encoder extracts a latent code **z** from the image, and a decoder reconstructs a dense depth map from that code. The system jointly optimizes the camera pose and **z**. A photometric loss enforces consistency, while a latent prior regularizes **z** toward its prior distribution.

Written as an equation, the objective is:

$$E(\mathbf{z}, T) = \sum_{i,j} \rho\bigl(I_j(\pi(T_{ij}, D_\mathbf{z}(u_i), u_i)) - I_i(u_i)\bigr) + \lambda \|\mathbf{z}\|^2$$

$D_\mathbf{z}$ is the decoder, $\pi$ is the projection, $\rho$ is a robust cost, and $T_{ij}$ is the relative pose between keyframes. The latent-prior term $\lambda\|\mathbf{z}\|^2$ corresponds to the negative log-likelihood of the standard normal prior $p(\mathbf{z}) = \mathcal{N}(0, I)$ and arises directly from MAP inference under a Gaussian prior.

> 🔗 **Borrowed.** Dellaert and Kaess's [GTSAM](https://gtsam.org/tutorials/intro.html) factor graph provided the backend structure for DeepFactors. Czarnowski reformulated CodeSLAM's joint pose-latent optimization as an explicit factor graph in which a learned latent variable appeared alongside conventional pose nodes. Graph edges expressed the coupling between learned and geometric variables.

CodeSLAM surpassed earlier methods in reconstructing geometry from sparse input, but it did not operate in real time. Both VAE inference and the optimization loop were slow, a limitation the paper stated explicitly.

> 📜 **Prediction vs. outcome.** CodeSLAM showed that dense SLAM could incorporate a compact learned representation, but remained limited in speed and scale. DeepFactors (2020), from the same Imperial group, moved closer to real-time operation without reaching deployment-grade performance. Teed and Deng at Princeton later achieved broader support for monocular, stereo, and RGB-D inputs through a different design: a learned frontend combined with Dense Bundle Adjustment (DBA).

---

## 13.2 DeepFactors — Imperial Dyson Lab, factor graph integration

In 2020, Jan Czarnowski, also supervised by Davison at the Imperial Dyson Robotics Lab, released [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2969036). He sought to integrate CodeSLAM's approach into a complete SLAM pipeline.

DeepFactors retained CodeSLAM's factor graph and latent-depth representation, explicitly separated tracking from mapping, and introduced a keyframe-selection criterion. On an NVIDIA GTX 1080, tracking against keyframes ran at about 250 Hz, but computing the network Jacobian took several hundred milliseconds per keyframe and became the pipeline's bottleneck. The system demonstrated the approach without achieving deployment-grade real-time performance.

DeepFactors placed a learned representation in a factor graph as a node and applied geometric optimization in its latent space. Czarnowski's result favored replacing selected pipeline components with learnable modules rather than replacing the full pipeline end to end.

Around the same time, Daniel Cremers's group at TU Munich arrived at the same design principle from a different starting point. The Davison group paired CodeSLAM's VAE latent variables with a factor graph, whereas the Cremers group inserted neural predictions into its 2016 direct sparse odometry system, [DSO](https://arxiv.org/abs/1607.02565). [Yang, Wang, Stückler, Cremers 2018. DVSO](https://arxiv.org/abs/1807.02570) added neural depth to monocular DSO as "virtual stereo," synthesizing a second camera observation in a monocular setting. [Yang, von Stumberg, Wang, Cremers 2020. D3VO](https://arxiv.org/abs/2003.01060) incorporated depth, pose, and uncertainty as three self-supervised predictions and additional factors in DSO's factor graph. [Wimbauer et al. 2021. MonoRec](https://arxiv.org/abs/2011.11814) and [Wimbauer et al. 2023. Behind the Scenes](https://arxiv.org/abs/2301.07668) extended the approach to dense reconstruction of dynamic scenes and single-view density fields. Although the Cremers and Imperial groups were institutionally separate, both integrated neural predictions into classical optimization.

Princeton produced another form of the same hybrid design in 2021.

---

## 13.3 RAFT — recurrent optical flow

Zachary Teed and Jia Deng at Princeton presented [Recurrent All-Pairs Field Transforms (RAFT)](https://arxiv.org/abs/2003.12039) at ECCV 2020. RAFT addressed optical-flow estimation rather than SLAM.

RAFT's design later became the core of DROID-SLAM and consists of three parts.

1. Feature encoder: a CNN extracts feature maps from two images.
2. Correlation volume: the method stores similarities between all pixel pairs in a 4D volume with a four-level pyramid.
3. Update operator: a Gated Recurrent Unit (GRU) iteratively queries the correlation volume and refines the flow field.

The phrase "all-pairs" describes the method's distinguishing feature. Rather than examining only selected neighboring pixels, RAFT considers every candidate location and progressively refines the flow field at a fixed resolution. Unlike earlier coarse-to-fine methods such as PWC-Net, it retains a full-resolution flow field while querying the correlation pyramid. Against the best published results available at the time, the RAFT paper reported a 16% relative reduction in KITTI F1-all error and a 30% reduction in Sintel final-pass endpoint error.

RAFT did not originate in SLAM, but Teed recognized a structural similarity between its update operator and iterative bundle adjustment. A GRU refining a flow field could play a role analogous to an optimization step that refines pose and depth.

---

## 13.4 DROID-SLAM — the update operator and BA

At NeurIPS 2021, Teed and Deng presented [DROID-SLAM](https://arxiv.org/abs/2108.10869). DROID stands for "Differentiable Recurrent Optimization-Inspired Design."

Its architecture makes the hybrid design explicit.

The frontend has the same structure as RAFT. A CNN encoder extracts feature maps, an all-pairs correlation volume is built, and a GRU update operator iteratively estimates dense flow. The difference is that flow is estimated not between a single pair of images but simultaneously on every edge of a keyframe graph.

The backend performs DBA with pose and inverse depth as optimization variables. It uses the 2D correspondences from flow estimation as constraints for their joint optimization and solves the linear system efficiently with the Schur complement.

The **DBA layer** connects the two components. Flow and uncertainty estimates from the GRU enter DBA; the updated pose and depth then provide the reference for the next GRU iteration, forming a recurrent loop.

> 🔗 **Borrowed.** The idea behind DBA reaches back ten years to Newcombe's DTAM (2011), a precursor of photometric bundle adjustment over every pixel. DROID-SLAM combined that approach with the more robust correspondences supplied by learned flow, despite the separate institutional lineages of Newcombe and Teed.

> 🔗 **Borrowed.** Teed and Deng adapted DROID-SLAM's update operator directly from their work on RAFT. All-pairs recurrent refinement, originally designed for optical flow, proved structurally compatible with the iterative optimization of bundle adjustment.

On the EuRoC MAV dataset, DROID-SLAM recorded a lower RMSE ATE than ORB-SLAM3, then the state of the art. Evaluation covered synthetic TartanAir data as well as real indoor and outdoor sequences. The method was particularly robust to lighting changes and scarce texture compared with feature-based systems. In his later Handbook retrospective, Teed reported that on EuRoC V1_02, global optimization reduced the frontend-only ATE from 16.5 cm to 1.2 cm. Classical BA reached single-digit-centimeter accuracy using constraints supplied by learned correspondences.

DROID-SLAM differed from the end-to-end methods in Ch.12 by separating the roles of learning and geometry. PoseNet regressed pose directly without geometric constraints and did not generalize. Teed and Deng instead used learning for dense correspondence estimation and BA for enforcing geometric constraints. Neural networks handled feature extraction and dense matching, while geometric optimization maintained consistency and propagated uncertainty. The system replaced hand-designed features but retained the optimization structure, distinguishing the 2021 hybrid from the 2015 end-to-end approach.

---

## 13.5 The Imperial Dyson Lab lineage

The Imperial Dyson Robotics Lab connected CodeSLAM with several later hybrid systems.

Andrew Davison led SLAM research at Imperial for twenty years after MonoSLAM in 2002. His students and collaborators developed several related lines of work.

- **Richard Newcombe** (Davison advisee, Imperial): DTAM (2011), [KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf) (2011); later Oculus → Meta Reality Labs.
- **Michael Bloesch** (Davison advisee, Imperial): CodeSLAM (2018), touch and inertial SLAM research.
- **Jan Czarnowski** (Davison advisee, Imperial): DeepFactors (2020).
- **Edgar Sucar** (Davison group, Imperial): [iMAP](https://arxiv.org/abs/2103.12352) (2021), later extended to the NeRF-SLAM lineage.
- **Tristan Laidlow** (Davison group, Imperial): dense 3D reconstruction, later extended to the neural implicit SLAM lineage.

The group retained its emphasis on factor graphs and uncertainty as representations changed from sparse monocular landmarks to dense latent variables and then to implicit fields. In [FutureMapping](https://arxiv.org/abs/1803.11288) (2018) and [FutureMapping 2](https://arxiv.org/abs/1910.14139) (2019, with Ortiz), Davison proposed that a Spatial AI map should include both computational structure and multiple representations. The papers argued for combining diverse geometric and semantic representations in one probabilistic graph. CodeSLAM and DeepFactors were early experiments in that program.

Teed and Deng developed DROID-SLAM independently at Princeton. Nevertheless, DTAM by Newcombe and Davison was a conceptual predecessor of DROID-SLAM's DBA, showing that technical lineages need not follow direct institutional connections.

---

## 13.6 2023–2025 — extensions after DROID

After DROID-SLAM, researchers both extended its design and pursued alternatives.

[GO-SLAM](https://arxiv.org/abs/2309.02436) (Zhang et al. 2023) extended DROID-SLAM's tracking with online loop closure and full bundle adjustment, while mapping with an Instant-NGP-style neural implicit representation based on multi-resolution hash encoding. It combined DROID-family dense flow and BA for tracking with an implicit map, adding another hybrid layer.

[NICER-SLAM](https://arxiv.org/abs/2302.03594) (Zhu et al. 2023) followed a different design. Rather than building on DROID, it solved tracking and mapping jointly over a single hierarchical neural implicit representation. It pursued the same goal of RGB-only dense SLAM from outside the DROID lineage.

[SplaTAM](https://arxiv.org/abs/2312.02126) (Keetha et al. 2024) replaced the map representation with 3D Gaussian Splatting and based tracking on silhouette-guided differentiable rendering rather than DROID-style dense flow. It belongs more directly to the 3DGS lineage than to the DROID lineage.

[DPV-SLAM](https://arxiv.org/abs/2408.01654) (Lipson, Teed, Deng 2024) came from the same Princeton group as DROID-SLAM. Built on [DPVO](https://github.com/princeton-vl/DPVO) (Deep Patch Visual Odometry) rather than DROID, it added proximity-based loop closure and CUDA block-sparse BA. The resulting system was roughly 2.5× faster and used less memory than DROID-SLAM. Its main change was a patch-based sparse representation combined with efficient loop closure rather than a simple feature replacement.

Outside the DROID lineage, work in 2024–2025 followed Naver Labs' [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al. 2023). DUSt3R changed the conventional SfM procedure by directly predicting pointmaps from two images, as Ch.16 discusses in detail. The same Revaud group then added symmetric multi-view processing and working memory in [Cabon et al. 2025. MUSt3R](https://arxiv.org/abs/2503.01661), extending the image-pair formulation to many frames in an attempt to support both offline SfM and online VO/SLAM with one network. DROID-family components also appeared in this ecosystem. [Li et al. 2024. MegaSAM](https://arxiv.org/abs/2412.04463) extended DROID-SLAM's differentiable DBA to dynamic scenes and uncalibrated video, jointly optimizing camera intrinsics at inference time. NVIDIA's [Huang et al. 2025. ViPE](https://arxiv.org/abs/2508.10934) combined DROID-SLAM's dense-flow network, cuvslam's sparse points, and a monocular depth network as three constraints in one DBA, producing a large-scale annotation pipeline for unconstrained video at YouTube scale. These systems applied DROID's learned-frontend, classical-backend design to the harder conditions of calibration-free and dynamic scenes.

Several approaches developed in parallel: GO-SLAM placed a neural map above DROID tracking; DPV-SLAM redesigned the system around lightweight patch odometry; NICER-SLAM and SplaTAM based tracking on implicit or splatting representations; and MegaSAM and ViPE extended DROID's DBA to uncalibrated and dynamic settings. Teed and Deng's 2021 combination of a learned frontend and classical backend became a common starting point for these variants.

> 📜 **Prediction vs. outcome.** DROID-SLAM established a reference architecture that combined differentiable DBA with end-to-end learning. Three years later, the same group improved efficiency with DPV-SLAM. GO-SLAM, NICER-SLAM, and SplaTAM instead replaced the map with implicit or Gaussian-splatting representations. As of 2026, these variations on DROID's "learned frontend plus classical backend" design had not converged on a general-purpose solution.

---

## 🧭 Still open

Generalization of learned priors beyond the training distribution remains a primary problem. The VAEs in CodeSLAM and DeepFactors learn the depth distribution of their training data. In substantially different environments, including open outdoor scenes, non-uniform textures, and nighttime conditions, a learned prior can direct optimization toward an incorrect solution. DROID-SLAM's flow estimator also performs worse outside its training domain. As of 2026, no learned SLAM system operated reliably in every environment. Training on diverse synthetic data such as TartanAir helps, but a sim-to-real gap remains.

Real-time operation remains another constraint. DROID-SLAM averages about 10–15 fps on an NVIDIA RTX 2080Ti and slows as the keyframe graph grows, with DBA as the bottleneck. As of 2026, it was still impractical for low-power applications requiring at least 30 Hz, such as mobile robots and AR/VR. Attempts to reduce computation through fewer keyframes or approximate BA incur performance trade-offs.

Integrating learned loop closure also remains unresolved. DROID-SLAM does not handle loop closure explicitly. In his later Handbook retrospective, Teed wrote that "DROID-SLAM doesn't include any relocalization module, so large loops with lots of drift cannot be closed." Its sliding-window keyframe graph provides limited global consistency. Some efforts have attempted to integrate learned loop closure, including the place-recognition work in Ch.10, into DROID's factor graph, but they have not converged on a single system. The connection between the NetVLAD lineage in Ch.10 and the DROID lineage in Ch.13 remains open.

---

DROID-SLAM used an inverse-depth map, a strong dense representation in 2021, but [NeRF](https://arxiv.org/abs/2003.08934) (Neural Radiance Field) had proposed a different option in 2020: representing a scene as a continuous function rather than as points, lines, planes, or meshes. Differentiable rendering offered another way to enforce photometric consistency.

At Imperial College in 2021, Edgar Sucar treated this as a SLAM problem rather than a rendering task: could an MLP replace the TSDF voxel grid entirely? His answer, iMAP, followed after fourteen months of work.
