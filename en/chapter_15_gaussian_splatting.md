# Ch.15 — The Gaussian Splatting Era: From 3DGS to GS-SLAM

In Ch.14, iMAP and NICE-SLAM showed what it meant to remember space with an MLP. There was a price. The MLP was opaque. No one could tell which neuron stored which region, and every new observation touched the whole network. NICE-SLAM's sub-1 fps rate on an RTX 3090 was hard to reconcile with the phrase "real-time SLAM." The scene sat locked inside network parameters, with no window into it.

At SIGGRAPH in August 2023, Bernhard Kerbl (INRIA), Georgios Kopanas, Thomas Leimkuhler, and George Drettakis presented [their paper](https://arxiv.org/abs/2308.04079). Kerbl kept the implicit representation paradigm that NeRF had built up over three years, but made a different choice. While iMAP, NICE-SLAM, and Co-SLAM locked scenes inside MLPs and voxel grids, Kerbl scattered millions of small ellipsoids (Gaussian primitives) into space. The SLAM community tilted toward this representation within six months. Kerbl's choice was not a new invention. The roots lay in a twenty-year-old graphics technique, Matthias Zwicker's EWA splatting (2001), and the differentiable rendering spirit of NeRF carried over intact. The difference was in the form of the representation.

---

## The structure of 3DGS

Kerbl represented scenes as an explicit set of Gaussians. Each Gaussian has a position (mean) $\boldsymbol{\mu} \in \mathbb{R}^3$, a covariance matrix $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, an opacity $\alpha \in (0,1]$, and a color expressed in spherical harmonics coefficients. For training stability the covariance is factored into a scale vector $\mathbf{s}$ and a unit quaternion $\mathbf{q}$:

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

Rendering alpha-blends the projected 2D Gaussians in depth order. Each Gaussian's effective opacity $\alpha_i$ is the product of the learnable opacity $\sigma_i$ and the 2D Gaussian density $G_i(\mathbf{x})$ evaluated at the pixel location. The pixel color $C$ is

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = \sigma_i \cdot G_i(\mathbf{x})$$

Unlike NeRF, which numerically approximates a volume rendering integral, 3DGS drops directly onto a GPU rasterization pipeline. The tile-based rasterizer implements both the forward pass and the backward pass as custom CUDA kernels. Over 30 fps on a single RTX 3090. Compared to NICE-SLAM, which was running under 1 fps on the same card, the rendering is dozens of times faster.

Initialization uses a sparse point cloud from SfM. Training then iterates a **densification** procedure that splits, clones, and prunes Gaussians. When the view-space position gradient crosses a threshold, Gaussians with large scale split into two children, and Gaussians with small scale clone at the same position. Gaussians with low opacity are pruned periodically.

> 🔗 **Borrowed.** 3DGS's rasterization-based splatting descends directly from Zwicker et al.'s [EWA splatting (2001)](https://www.cs.umd.edu/~zwicker/publications/EWAVolumeSplatting-VIS01.pdf). Zwicker wrapped each point in an elliptical weighted-average kernel to render point clouds. Kerbl replaced that kernel with a learnable Gaussian and accelerated it with a GPU tile rasterizer.

---

## Structural fit between 3DGS and SLAM

Implicit representations did not suit SLAM. MLP-based NeRF had to retrain the whole network for every new observation, and catastrophic forgetting made incremental updates hard. Map expansion meant resizing the network. NICE-SLAM's voxel grid eased the problem but could not escape the resolution–memory trade-off.

3DGS solved this structurally. A Gaussian is an object explicitly sitting in space, so when a new keyframe comes in, one simply adds Gaussians to the corresponding region. The densification procedure meshed naturally with keyframe insertion, and rendering quality stayed at NeRF level while running in real time. GS-SLAM papers poured in through late 2023 on that arithmetic.

---

## GS-SLAM: the first attempt

Chi Yan (HKU) and collaborators posted [Yan et al. 2023. GS-SLAM](https://arxiv.org/abs/2311.11700) on arXiv in November 2023. It was the first system to integrate 3DGS into a SLAM pipeline.

GS-SLAM's structure followed the classical SLAM framework. Tracking estimates the pose of the current frame; Mapping updates the Gaussian map. Yan's contributions were twofold. First, adaptive Gaussian expansion: a mechanism that inserts Gaussians into low-coverage regions when a new keyframe is added. Second, geometry-aware Gaussian selection: during backpropagation of the rendering loss, only Gaussians with large contribution are selected for optimization, buying speed.

Tracking optimizes the pose against a rendered photometric loss. GS-SLAM's tracking loss is an L1 color loss over sampled pixels:

$$\mathcal{L}_{track} = \sum_m \|\mathbf{C}_m - \hat{\mathbf{C}}_m\|_1$$

At the mapping step, Yan uses a weighted sum of color L1 and depth L1. Meanwhile the 3DGS original paper's training loss, the $(1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$ combination ($\lambda=0.2$), is the default form inherited when training the Gaussian map. The reason this loss is differentiable with respect to pose is 3DGS's differentiable rasterizer.

On the Replica dataset it matched NICE-SLAM's PSNR while raising throughput. The limits were plain too. It assumed an RGB-D camera and was not validated on large-scale outdoor environments.

---

## SplaTAM: silhouette-based densification

Nikhil Keetha (Carnegie Mellon)'s [Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126) differed from GS-SLAM in design philosophy. Instead of a complicated selection mechanism, Keetha went with a simple silhouette-mask-based densification.

Keetha's central idea is the **silhouette mask**. New Gaussians are added in the region of the current view that the existing Gaussians do not explain — the empty part of the rendered mask. Rather than computing where Gaussians should be, look at where they are not, and fill those. A simple rule.

Tracking optimizes the pose; Mapping optimizes the Gaussian parameters. The strict separation of the two steps is the basis of stability. GS-SLAM's alternating tracking-and-mapping optimization causes interference, and Keetha's two-step structure sidesteps it.

> 🔗 **Borrowed.** SplaTAM's keyframe-based map management structure is a case of the PTAM (Klein & Murray, 2007) idea running again on a new representation. PTAM's way of selectively inserting keyframes to maintain the map became, in SplaTAM, the trigger for Gaussian densification.

On the Replica dataset SplaTAM recorded PSNR 34.11 dB. In the same paper's table, NICE-SLAM recorded 24.42 dB. The rendering-quality gap was clear.

In the Limitations & Future Work of his 2024 CVPR paper, Keetha listed sensitivity to motion blur, depth noise, and aggressive rotation, plus the direction of removing the dependence on known intrinsics and dense depth, as the next tasks. He also mentioned scalability improvements.

> 📜 **Prediction vs. outcome.** In the Limitations section of SplaTAM (2024) Keetha named sensitivity to motion blur, depth noise, and aggressive rotation, as well as removal of the known-intrinsics/dense-depth dependence, as explicit tasks. The depth-removal direction was answered the same year by Matsuki's MonoGS (2024 CVPR) with a monocular RGB setting. The intrinsics-free direction and the large-scale issue are still in progress as of 2024–2025. `[partial hit + in progress]`

---

## MonoGS: monocular RGB

Hidenobu Matsuki (Imperial College Dyson Robotics Lab)'s [Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741) removed one constraint. It runs 3DGS SLAM from a single monocular RGB camera, with no depth sensor.

The core difficulty in the monocular setting is scale. Recovering metric scale without depth is an unsolved problem even in SfM. Matsuki's answer was to optimize the Gaussian geometry directly. He added a geometric consistency loss between the rendered depth and neighboring Gaussians.

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

Here $\mathbf{s}_k \in \mathbb{R}^3$ is the scale vector of the k-th Gaussian, and $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$ is the mean of the three-axis scales. This isotropy regularization prevents Gaussians from degenerating into excessively thin plate shapes. Without depth supervision, monocular Gaussians tend to stick to the camera plane, and the regularizer suppresses that failure mode.

For tracking, Matsuki optimizes the pose directly with the Gaussian rendering photometric loss. At the first-frame initialization he uses a monocular depth prior to seed the Gaussian positions, and for later frames he runs a rendering-based refinement starting from the previous pose. Without a depth sensor, the core of keeping scale is the combination of isotropic regularization and geometry-consistency losses across keyframes.

> 🔗 **Borrowed.** MonoGS's use of a monocular depth prior descends from the Godard MonoDepth2 line covered in Ch.11. The insight of self-supervised monocular depth, that structure can be recovered without depth supervision, was absorbed here as a Gaussian initialization strategy.

Matsuki came out of Imperial's Dyson Robotics Lab. The same lineage as Sucar (iMAP) and Bloesch (CodeSLAM), both from the lab Davison supervised. MonoGS represents the lab's shift from implicit MLP to explicit Gaussian representations.

On the TUM-RGBD dataset, MonoGS recorded an average ATE RMSE of 4.44 cm monocular and 1.58 cm RGB-D. Rendering quality in the RGB-D setting on Replica reached an average PSNR of 37.50 dB, comparable to other same-generation GS-SLAM systems.

---

## RTG-SLAM and real-time processing

The next problem for the GS-SLAM lineage to solve was speed. GS-SLAM and SplaTAM were hard to call real-time. Peng Zhexi's group at Zhejiang University published [RTG-SLAM (SIGGRAPH 2024)](https://arxiv.org/abs/2404.19706) in 2024 with real-time as an explicit target.

RTG-SLAM's strategy is to control the number of Gaussians. Instead of optimizing all Gaussians equally, it selects those with large contribution from the current camera view and optimizes only those. Gaussians are initialized on a surfel (surface-element) basis, keeping geometry while reducing count. On the Replica dataset it approached real-time throughput.

---

## Vanished competitors

Starting in 2024, TSDF and occupancy grids were pushed out of the mainstream of SLAM mapping. They still see use in embedded systems or safety-critical environments, but on the research frontier they retreated to a supporting role. NeRF-based SLAM in the same year also moved to a supporting position, outpaced by 3DGS in rendering speed and update flexibility.

This is a shift in representation and in hardware affinity at the same time. GPU rasterizers are much better optimized than GPU ray marchers. The fact that 3DGS runs naturally on the existing graphics pipeline raised its adoption speed relative to NeRF.

> 🔗 **Borrowed.** The differentiable rendering spirit of 3DGS descends directly from NeRF. The idea of optimizing scene representations with gradients, and of linking observation and rendering through a photometric loss, is the legacy of Mildenhall et al. (2020). Kerbl swapped the representation (implicit MLP → explicit Gaussian) but inherited the paradigm.

> 📜 **Prediction vs. outcome.** Kerbl et al., in §7.4 Limitations of 3DGS (2023), named elongated artifacts and popping in under-observed regions, the absence of regularization, and memory consumption (over 20 GB during training, several hundred MB when rendering large scenes) as limits. As future work they proposed antialiasing, more principled culling, and borrowing point-cloud compression techniques. The memory axis (compression) was answered directly in 2024 by the [Compact 3DGS](https://arxiv.org/abs/2311.13681) line and [Niedermayr et al.](https://arxiv.org/abs/2401.02436). Dynamic scene extensions ([4DGS](https://arxiv.org/abs/2310.08528), [Deformable 3DGS](https://arxiv.org/abs/2309.13101)) and generation/editing ([DreamGaussian](https://arxiv.org/abs/2309.16653), [GaussianEditor](https://arxiv.org/abs/2311.14521)) are areas the original paper did not directly raise, but they branched off as separate lines around 2024. `[hit]`

---

## 🧭 Still open

Memory scaling. The number of Gaussians grows linearly with scene size. What was sufficient at a few hundred thousand on the indoor Replica dataset grows to tens of millions in an outdoor city block. Gaussian pruning and level-of-detail hierarchies are being studied, but there is no consensus yet on how to reasonably manage the memory–rendering-quality trade-off in large-scale environments. The Compact 3DGS line (Lee et al. 2024, Niedermayr et al. 2024) is exploring the compression direction.

Semantic integration. Attempts to attach semantic labels to Gaussians ([LangSplat](https://arxiv.org/abs/2312.16084), [LERF](https://arxiv.org/abs/2303.09553), and others) appeared in 2023–2024. But no method yet updates semantic Gaussians in real time within a SLAM pipeline while also preserving tracking quality. How to handle the interference that arises when semantic and geometry are jointly optimized is the core question.

Dynamic scene. 4DGS and Deformable 3DGS proposed the direction of adding a time dimension to Gaussians. In a SLAM setting, dynamic objects move differently from the background and must be handled separately. GS-SLAM (Yan et al. 2023), SplaTAM (Keetha et al. 2024), and MonoGS (Matsuki et al. 2024) all retain the static-world assumption. The full trajectory of how SLAM has dealt with moving objects, from mask-based outlier rejection through multi-object factor graphs to deformable reconstruction, is covered separately in Ch.15b.

But 3DGS left another question. Where do the Gaussians come from. From an SfM point cloud, or from a depth sensor. A pose must be known to place a Gaussian, and Gaussians must exist to estimate a pose. This chicken-and-egg kept the GS-SLAM lineage dependent on external initialization. DUSt3R and its successors, covered in Ch.16, picked a different starting point. Instead of putting geometry on top of a representation, they learn geometry from scratch.
