# Ch.13 — The Hybrid Victory: From CodeSLAM to DROID-SLAM

When Michael Bloesch presented CodeSLAM at CVPR 2018, his affiliation was the Dyson Robotics Lab at Imperial College London. His advisor was Andrew Davison. In the same lab, Richard Newcombe had built DTAM in 2011; in the same lab, Jan Czarnowski would release DeepFactors in 2020, and Edgar Sucar and Tristan Laidlow would extend the lineage. That is why CodeSLAM is more than a single paper. The creed Davison had been building since 2002 — "SLAM is probabilistic inference" — met for the first time, in a substantive way, the impulse brought by mid-2010s deep learning that "representations can be learned," and the meeting took place in Bloesch's 2018 paper.

---

## 13.1 CodeSLAM — latent code and the map

In traditional monocular SLAM, depth was something to be estimated. Whether as a few hundred sparse landmarks or, as in [DTAM](https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf) (Newcombe et al. 2011), every pixel, depth was ultimately an optimization variable. The dimension of that variable space scaled with image resolution. A dense depth map for a single keyframe at 640×480 means 307,200 independent variables. Optimization is heavy, initialization is sensitive, and priors are hard to inject.

The idea of [Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271) was simple. Instead of optimizing the depth map itself, optimize a low-dimensional latent vector (**latent code**) that generates it. Train a variational autoencoder (VAE) on real depth distributions, and its bottleneck latent space approximates the manifold on which "realistic depth maps" live. The optimization moves only on that manifold. The variables drop from hundreds of thousands to hundreds.

> 🔗 **Borrowed.** CodeSLAM's latent depth representation borrows the encoder-decoder latent space structure established in [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114). The training phase follows the VAE frame, but at SLAM inference time **z** is treated directly as a MAP optimization variable without stochastic sampling. A tool generative-model researchers devised for image synthesis re-emerged a decade later as the low-dimensional representation space for SLAM optimization.

The structure goes as follows. For each keyframe, a VAE encoder extracts a latent code **z** from the image. A decoder reconstructs a dense depth map from **z**. Camera pose and **z** are jointly optimized. A photometric loss enforces consistency, and a latent prior regularizes **z** toward the prior distribution.

Written as an equation, the objective is:

$$E(\mathbf{z}, T) = \sum_{i,j} \rho\bigl(I_j(\pi(T_{ij}, D_\mathbf{z}(u_i), u_i)) - I_i(u_i)\bigr) + \lambda \|\mathbf{z}\|^2$$

$D_\mathbf{z}$ is the decoder, $\pi$ is the projection, $\rho$ is a robust cost, and $T_{ij}$ is the relative pose between keyframes. The latent prior term $\lambda\|\mathbf{z}\|^2$ corresponds to the negative log-likelihood of the standard normal prior $p(\mathbf{z}) = \mathcal{N}(0, I)$, and is the regularizer that falls out naturally from MAP inference under a Gaussian prior.

> 🔗 **Borrowed.** The factor graph (Dellaert and Kaess's [GTSAM](https://gtsam.org/tutorials/intro.html)) provided the backend skeleton of DeepFactors. The pose–latent coupling that CodeSLAM handled with joint optimization was reformulated by Czarnowski as an explicit factor graph. It was only at DeepFactors that a learned latent variable took its place next to the traditional pose nodes as another graph variable. The interface between the two worlds was the graph's edge.

The ability to fill in geometry from sparse input surpassed prior methods. CodeSLAM itself, however, was not real-time. The VAE inference and optimization loop were slow. The paper said so plainly.

> 📜 **Prediction vs. outcome.** CodeSLAM showed that a compact learned representation could be brought inside dense SLAM, but left room on both speed and scale. The follow-up DeepFactors (2020), from the same Imperial group, pushed one step further toward real-time but did not reach deployment-grade performance, and the general-purpose coverage across monocular, stereo, and RGB-D was eventually achieved by a different team (Teed and Deng, Princeton) through a different design — learned frontend plus Dense Bundle Adjustment (DBA). `[in progress + diverted]`

---

## 13.2 DeepFactors — Imperial Dyson Lab, factor graph integration

In 2020, Jan Czarnowski, also supervised by Davison at the Imperial Dyson Robotics Lab, released [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2969036). Czarnowski's aim was to pull the CodeSLAM idea inside an actual SLAM pipeline.

DeepFactors kept CodeSLAM's factor graph plus latent depth structure, separated tracking and mapping explicitly, and introduced a keyframe selection criterion. On an NVIDIA GTX 1080, tracking ran at about 250Hz against keyframes, but the network Jacobian computation took several hundred milliseconds per keyframe and was the bottleneck of the whole pipeline. It pointed the direction but did not reach deployment-grade real-time.

What DeepFactors proved was a principle. A learned representation can enter the factor graph as a node, and geometry optimization can operate on that latent space. The conclusion Czarnowski reached was simple. The realistic path is not end-to-end replacement, but swapping parts of the pipeline for learnable modules.

Around the same time, Daniel Cremers's group at TU Munich reached the same principle. Their starting point differed from Imperial's. Where the Davison lineage stacked a factor graph on top of CodeSLAM's VAE latent, the Cremers group took their own 2016 direct sparse odometry ([DSO](https://arxiv.org/abs/1607.02565)) as the skeleton and injected neural prediction into it. [Yang, Wang, Stückler, Cremers 2018. DVSO](https://arxiv.org/abs/1807.02570) injected neural depth into monocular DSO as a "virtual stereo," hallucinating a second camera in a monocular setting; [Yang, von Stumberg, Wang, Cremers 2020. D3VO](https://arxiv.org/abs/2003.01060) added three kinds of self-supervised neural prediction — depth, pose, and uncertainty — as additional factors in DSO's factor graph. [Wimbauer et al. 2021. MonoRec](https://arxiv.org/abs/2011.11814) and [Wimbauer et al. 2023. Behind the Scenes](https://arxiv.org/abs/2301.07668) carried the same lineage toward dynamic scene dense reconstruction and single-view density fields. The human lineage is separate from the Imperial group, but the design principle, absorbing neural prediction into classical optimization structure, converged.

That principle appeared again in 2021, in another form, at Princeton.

---

## 13.3 RAFT — recurrent optical flow

Zachary Teed and Jia Deng (Princeton) presented [Recurrent All-Pairs Field Transforms (RAFT)](https://arxiv.org/abs/2003.12039) at ECCV 2020. RAFT was not a SLAM paper. It was an optical flow estimation paper.

Yet RAFT's design becomes the core of DROID-SLAM later. The structure splits into three parts.

1. Feature encoder: a CNN extracts feature maps from two images
2. Correlation volume: similarities between all pixel pairs are built into a 4D volume. 4-level pyramid
3. Update operator: Gated Recurrent Unit (GRU) based iterative refinement. Looks up the correlation volume and updates the flow field

The all-pairs in the name summarizes what sets this structure apart. Rather than looking only at specific neighboring pixels, it considers every candidate location at once and refines the flow field progressively at a fixed resolution. Unlike earlier coarse-to-fine methods (PWC-Net and others), it keeps the flow field at a single full resolution while looking up the correlation pyramid. It beat prior methods by 5%–15% on KITTI, Sintel, and FlyingThings3D.

RAFT is not an ancestor in the SLAM lineage. But Teed noticed that the same update operator structure is structurally similar to SLAM's iterative bundle adjustment. How different is a GRU refining a flow field from an optimization step refining pose and depth?

---

## 13.4 DROID-SLAM — the update operator and BA

NeurIPS 2021, [Teed & Deng. DROID-SLAM](https://arxiv.org/abs/2108.10869). The DROID in the title stands for "Differentiable Recurrent Optimization-Inspired Design."

Walking through the architecture reveals the intent of the hybrid design.

The frontend has the same structure as RAFT. A CNN encoder extracts feature maps, an all-pairs correlation volume is built, and a GRU update operator iteratively estimates dense flow. The difference is that flow is estimated not between a single pair of images but simultaneously on every edge of a keyframe graph.

The backend is DBA. Pose and inverse depth are the optimization variables. The 2D correspondences supplied by flow estimation are used as constraints to jointly optimize pose and depth. The Schur complement trick solves the linear system efficiently.

The connecting tissue is the **DBA layer**. The flow and uncertainty estimated by the GRU feed into the DBA. When the DBA updates pose and depth, the result refreshes the reference for the next GRU iteration. The two modules are connected in a loop.

> 🔗 **Borrowed.** The idea of DBA goes back ten years. Newcombe's DTAM (2011) was the precursor of photometric bundle adjustment using every pixel. DROID-SLAM married that idea to the more robust input of learned flow. Newcombe's and Teed's affiliations differ, but the logical lineage runs through.

> 🔗 **Borrowed.** The update operator in DROID-SLAM was ported directly from RAFT by the same authors (Teed and Deng). The insight was that all-pairs recurrent refinement, designed for optical flow, is structurally compatible with the iterative optimization of bundle adjustment. That the same person wrote both papers made the borrowing possible.

On the EuRoC MAV dataset, DROID-SLAM recorded a lower RMSE ATE than ORB-SLAM3, the then-state-of-the-art. On both TartanAir (synthetic) and real indoor and outdoor sequences. It was more robust than feature-based methods especially under lighting change and texture scarcity. The numbers Teed later reported on the EuRoC V1_02 sequence in his Handbook retrospective are striking. An ATE of 16.5cm with the frontend alone dropped to 1.2cm after global optimization. Classical BA converged to single-digit centimeters on the constraints supplied by learned correspondence.

Looking back at why Ch.12's pure end-to-end approach failed reveals why DROID-SLAM was different. PoseNet regressed pose directly without geometric constraints and failed to generalize. Teed and Deng divided the roles. Dense correspondence estimation was left to learning; geometric constraint enforcement was handled by BA. Neural networks played to their strengths in feature extraction and dense matching; the geometry optimizer played to its strengths in consistency enforcement and uncertainty propagation. Hand-designed features were replaced with learned features, but the optimization structure was kept. That is where the 2021 hybrid differed from the 2015 end-to-end.

---

## 13.5 The Imperial Dyson Lab lineage

The flow from CodeSLAM to DROID-SLAM is best seen by tracing the human lineage of the Imperial Dyson Robotics Lab.

Andrew Davison led SLAM research at Imperial for twenty years after MonoSLAM in 2002. His students and collaborators created branching points one after another.

- **Richard Newcombe** (Davison advisee, Imperial): DTAM (2011), [KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf) (2011). Later Oculus → Meta Reality Labs
- **Michael Bloesch** (Davison advisee, Imperial): CodeSLAM (2018), touch and inertial SLAM research
- **Jan Czarnowski** (Davison advisee, Imperial): DeepFactors (2020)
- **Edgar Sucar** (Davison group, Imperial): [iMAP](https://arxiv.org/abs/2103.12352) (2021), later extended to the NeRF-SLAM lineage
- **Tristan Laidlow** (Davison group, Imperial): dense 3D reconstruction, later extended to the neural implicit SLAM lineage

This lineage is one of the cases where "school" is not an exaggeration. The factor graph plus uncertainty philosophy carried across, changing shape from monocular sparse to dense latent, and from dense latent to implicit representation. In [FutureMapping](https://arxiv.org/abs/1803.11288) (2018) and [FutureMapping 2](https://arxiv.org/abs/1910.14139) (2019, with Ortiz), Davison sketched directly on the map the computational structure and representations a Spatial AI system should carry. The argument was to bind diverse geometric and semantic representations into one probabilistic graph. CodeSLAM and DeepFactors were the first experiments under that sketch.

Teed and Deng are outside this lineage. Princeton, independent path. But the logical predecessor of DROID-SLAM's DBA is DTAM, and DTAM is Newcombe and Davison's work. Lineages can extend logically without human connection.

---

## 13.6 2023–2025 — extensions after DROID

In the years after DROID-SLAM, research grew on top of it, and alongside it.

[GO-SLAM](https://arxiv.org/abs/2309.02436) (Zhang et al. 2023) extended DROID-SLAM's tracking with online loop closing and full bundle adjustment, and ran mapping on an Instant-NGP style neural implicit representation (multi-resolution hash encoding). Tracking is DROID-family dense flow plus BA; the map is an implicit representation. A second layer of hybrid.

[NICER-SLAM](https://arxiv.org/abs/2302.03594) (Zhu et al. 2023) took a different road. Instead of borrowing from DROID, it solved tracking and mapping simultaneously on a single hierarchical neural implicit representation. The goal of RGB-only dense SLAM is shared, but the path differs. It is a way of hitting the same problem from outside the DROID lineage.

[SplaTAM](https://arxiv.org/abs/2312.02126) (Keetha et al. 2024) swapped the map representation for 3D Gaussian Splatting, and rewrote tracking not as DROID-style dense flow but on silhouette-guided differentiable rendering. It is a coupling with the 3DGS lineage rather than a direct extension of the DROID lineage.

[DPV-SLAM](https://arxiv.org/abs/2408.01654) (Lipson, Teed, Deng 2024) came out of the same Princeton group as DROID-SLAM. Based on [DPVO](https://github.com/princeton-vl/DPVO) (Deep Patch Visual Odometry) rather than DROID, it added proximity-based loop closure and CUDA block-sparse BA to build a system roughly 2.5× faster and with a smaller memory footprint than DROID-SLAM. The core is not a feature swap but a patch-based sparse representation combined with efficient loop closure.

Outside the DROID lineage, extensions in 2024–2025 followed the path Naver Labs opened with [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al. 2023). After DUSt3R redefined the procedure of SfM itself by directly outputting pointmaps from two images (covered in detail in Ch.16), the same Revaud group introduced symmetric multi-view extension and working memory in [Cabon et al. 2025. MUSt3R](https://arxiv.org/abs/2503.01661), extending the image-pair-based structure to many frames. It is an attempt to let a single network handle both offline SfM and online VO/SLAM. More interesting is that DROID-family tools are recycled inside this ecosystem. [Li et al. 2024. MegaSAM](https://arxiv.org/abs/2412.04463) pushed DROID-SLAM's differentiable DBA toward dynamic scenes and uncalibrated video, jointly optimizing camera intrinsics during inference as well. NVIDIA's [Huang et al. 2025. ViPE](https://arxiv.org/abs/2508.10934) combined three kinds of constraints — DROID-SLAM's dense flow network, cuvslam's sparse points, and a monocular depth network — in a single DBA, industrializing it into a wild-video annotation pipeline at YouTube scale. The 2021 DROID design of learned frontend plus classical backend is being repeated in 2025 under harder conditions: calibration-free and dynamic scenes.

The pattern is not a single path. The route of stacking a neural map on top of DROID tracking as in GO-SLAM, the route of lightweight redesign with patch odometry as in DPV-SLAM, the route of rewriting tracking on implicit or splatting representations as in NICER-SLAM or SplaTAM, and the route of pushing DROID's DBA into uncalibrated and dynamic regimes as in MegaSAM and ViPE, all ran in parallel. The learned frontend plus classical backend framework that Teed and Deng released in 2021 became the common starting point for those branches.

> 📜 **Prediction vs. outcome.** DROID-SLAM set the reference point for a hybrid combining differentiable DBA with end-to-end learning. DPV-SLAM, released three years later by the same group, carried that reference point forward on efficiency. In contrast, the GO-SLAM, NICER-SLAM, and SplaTAM line branched off toward swapping the map representation for implicit or Gaussian splatting. DROID's "learned frontend plus classical backend" design is being varied along several branches, and which branch becomes the general-purpose solution has not yet been settled as of 2026. `[in progress]`

---

## 🧭 Still open

Generalization of learned priors outside the training distribution is the first problem. CodeSLAM's and DeepFactors's VAEs learn the depth distribution of the training data. In fully different environments (outdoor open-world, non-uniform texture, nighttime), a learned prior can even pull the optimization in the wrong direction. DROID-SLAM's flow estimator also drops in performance outside its training domain. As of 2026, "learned SLAM that works in any environment" does not yet exist. Approaches that train on diverse synthetic data such as TartanAir exist, but a sim-to-real gap remains.

The real-time constraint is still there. DROID-SLAM runs on average at about 10–15 fps on an NVIDIA RTX 2080Ti. It gets slower as the keyframe graph grows. DBA is the bottleneck. For applications that need real-time (30Hz+) low-power deployment, such as mobile robots or AR/VR, it is still not practical as of 2026. Lightweighting attempts (reducing keyframe count, approximate BA) exist, but carry performance trade-offs.

Learned integration of loop closure also remains unresolved. DROID-SLAM does not handle loop closure explicitly. Teed himself admitted, calmly, in his later Handbook retrospective that "DROID-SLAM doesn't include any relocalization module, so large loops with lots of drift cannot be closed." The keyframe graph is maintained in a sliding-window fashion, and global consistency is limited. Some efforts have tried to integrate learned loop closure (the place recognition work in Ch.12) into DROID's factor graph, but they have not converged into a single system. The point where the Ch.10 NetVLAD lineage and the Ch.13 DROID lineage meet is still open.

---

One question remains once we are here. Must the map representation stay as points, lines, and planes? DROID-SLAM's inverse depth map was the best dense representation available in 2021. But in 2020, [NeRF](https://arxiv.org/abs/2003.08934) (Neural Radiance Field) suggested a wholly different possibility. What if a scene is represented not as points or meshes but as a continuous function? If rendering is differentiable, photometric consistency can be enforced in a new way.

In 2021, at Imperial College, Edgar Sucar asked the same question — not as a rendering exercise but as a SLAM problem. Could an MLP replace the TSDF voxel grid entirely? The answer, and the fourteen months it took to arrive at it, is where Ch.14 begins.
