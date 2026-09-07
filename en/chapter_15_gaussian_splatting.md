# Ch.15 — The Gaussian Splatting Era: From 3DGS to GS-SLAM

iMAP and NICE-SLAM represented space with an MLP, but their representations were difficult to edit directly. Updating the global MLP in iMAP could affect other regions; NICE-SLAM reduced that problem with local feature grids. NICE-SLAM also ran below 1 fps on an RTX 3090, far from real-time SLAM. Its scene representation remained difficult to inspect within the network parameters.

At SIGGRAPH in August 2023, Bernhard Kerbl of INRIA, Georgios Kopanas, Thomas Leimkuhler, and George Drettakis presented [their paper](https://arxiv.org/abs/2308.04079). Kerbl retained the differentiable scene optimization that NeRF had developed over the preceding three years but changed the representation's form. Instead of encoding scenes in MLPs or voxel feature grids, as iMAP, NICE-SLAM, and Co-SLAM did, 3DGS represented them with millions of explicit ellipsoidal Gaussian primitives. The SLAM community adopted the representation within six months. Its rasterization drew on Matthias Zwicker's twenty-year-old EWA splatting technique (2001), while its optimization retained the differentiable-rendering framework associated with NeRF.

---

## The structure of 3DGS

Kerbl represented scenes as an explicit set of Gaussians. Each Gaussian has a position (mean) $\boldsymbol{\mu} \in \mathbb{R}^3$, a covariance matrix $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, an opacity $\alpha \in (0,1]$, and a color expressed in spherical harmonics coefficients. For training stability the covariance is factored into a scale vector $\mathbf{s}$ and a unit quaternion $\mathbf{q}$:

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

Rendering alpha-blends the projected 2D Gaussians in depth order. Each Gaussian's effective opacity $\alpha_i$ is the product of the learnable opacity $\sigma_i$ and the 2D Gaussian density $G_i(\mathbf{x})$ evaluated at the pixel location. The pixel color $C$ is

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = \sigma_i \cdot G_i(\mathbf{x})$$

Unlike NeRF, which numerically approximates a volume-rendering integral, 3DGS runs directly through a GPU rasterization pipeline. Its tile-based rasterizer implements both forward and backward passes as custom CUDA kernels. On a single RTX 3090, it renders at more than 30 fps after training. This novel-view rendering rate cannot be compared directly as a speed ratio with NICE-SLAM's full tracking-and-mapping throughput.

Initialization uses a sparse point cloud from SfM. Training then iterates a **densification** procedure that splits, clones, and prunes Gaussians. When the view-space position gradient crosses a threshold, Gaussians with large scale split into two children, and Gaussians with small scale clone at the same position. Gaussians with low opacity are pruned periodically.

> 🔗 **Borrowed.** 3DGS's rasterization-based splatting descends directly from Zwicker et al.'s [EWA splatting (2001)](https://www.cs.umd.edu/~zwicker/publications/EWAVolumeSplatting-VIS01.pdf). Zwicker wrapped each point in an elliptical weighted-average kernel to render point clouds. Kerbl replaced that kernel with a learnable Gaussian and accelerated it with a GPU tile rasterizer.

---

## Structural fit between 3DGS and SLAM

Implicit representations posed several difficulties for SLAM. An MLP-based NeRF had to update the entire network for each new observation, while catastrophic forgetting made incremental learning difficult. Expanding the map required increasing the network's capacity. NICE-SLAM's voxel grid mitigated these problems but retained a trade-off between resolution and memory.

3DGS addressed these problems with an explicit representation. When a new keyframe arrives, the system can add Gaussians only to the corresponding region. Densification aligns naturally with keyframe insertion, while rendering retains NeRF-level quality at real-time speed. GS-SLAM papers began appearing in quick succession in late 2023.

---

## GS-SLAM: an early attempt

Chi Yan (HKU) and collaborators posted [Yan et al. 2023. GS-SLAM](https://arxiv.org/abs/2311.11700) on arXiv in November 2023. Several 3DGS-SLAM manuscripts appeared around the same time; GS-SLAM was among the early systems to combine 3DGS with tracking and mapping.

GS-SLAM followed the classical SLAM framework: tracking estimated the pose of the current frame, and mapping updated the Gaussian map. Yan introduced two mechanisms. Adaptive Gaussian expansion inserted Gaussians into low-coverage regions when a new keyframe was added. Geometry-aware Gaussian selection optimized only Gaussians that contributed substantially to the rendering loss, reducing computation during backpropagation.

Tracking optimizes the pose against a rendered photometric loss. GS-SLAM's tracking loss is an L1 color loss over sampled pixels:

$$\mathcal{L}_{track} = \sum_m \|\mathbf{C}_m - \hat{\mathbf{C}}_m\|_1$$

During mapping, Yan used a weighted sum of color L1 and depth L1 losses. Training the Gaussian map also inherited the original 3DGS objective, $(1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$ with $\lambda=0.2$, as its default form. The differentiable rasterizer makes this loss differentiable with respect to pose.

On the Replica dataset, GS-SLAM matched NICE-SLAM's PSNR at higher throughput. It nevertheless required an RGB-D camera and was not validated in large-scale outdoor environments.

---

## SplaTAM: silhouette-based densification

[Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126), by Nikhil Keetha at Carnegie Mellon and his colleagues, used a simpler densification method than GS-SLAM based on a silhouette mask.

The **silhouette mask** identifies regions in the current view that the existing Gaussians do not explain. SplaTAM adds new Gaussians to these empty areas of the rendered mask, using absence of coverage as a direct densification criterion.

Tracking optimizes the pose, while mapping optimizes the Gaussian parameters. Tracking holds the map fixed, and mapping updates the Gaussians. GS-SLAM also separates pose and map variables, so separation alone does not distinguish SplaTAM.

> 🔗 **Borrowed.** SplaTAM applies PTAM's keyframe-based map-management principle (Klein & Murray, 2007) to a new representation. Selective keyframe insertion, which PTAM used to maintain its map, becomes the trigger for Gaussian densification in SplaTAM.

On the Replica dataset SplaTAM recorded PSNR 34.11 dB. In the same paper's table, NICE-SLAM recorded 24.42 dB. The rendering-quality gap was clear.

In the Limitations & Future Work section of the 2024 CVPR paper, Keetha listed sensitivity to motion blur, depth noise, and aggressive rotation. He also identified removal of the dependence on known intrinsics and dense depth, along with improved scalability, as future work.

> 📜 **Prediction vs. outcome.** In SplaTAM's Limitations section (2024), Keetha identified sensitivity to motion blur, depth noise, and aggressive rotation, as well as dependence on known intrinsics and dense depth. Matsuki's MonoGS (CVPR 2024) removed the depth requirement with a monocular RGB system in the same year. Work on unknown intrinsics and large-scale operation remained in progress in 2024–2025.

---

## MonoGS: monocular RGB

[Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741), by Hidenobu Matsuki at Imperial College's Dyson Robotics Lab and his colleagues, removed the depth-sensor requirement. It performs 3DGS SLAM from a single monocular RGB camera.

Scale is the central difficulty in the monocular setting because metric recovery without depth remains unsolved even in SfM. Matsuki optimized the Gaussian geometry directly and added a geometric-consistency loss between rendered depth and neighboring Gaussians.

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

Here $\mathbf{s}_k \in \mathbb{R}^3$ is the scale vector of the $k$-th Gaussian, and $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$ is the mean scale across its three axes. This isotropy regularization prevents Gaussians from degenerating into excessively thin plates. Without depth supervision, monocular Gaussians tend to remain near the camera plane, and the regularizer suppresses that failure mode.

For tracking, Matsuki directly optimizes pose with the photometric loss from Gaussian rendering. During first-frame initialization, a monocular depth prior provides the initial Gaussian positions. Later frames begin from the previous pose and undergo rendering-based refinement. Without a depth sensor, isotropic regularization and geometry-consistency losses across keyframes jointly maintain scale.

> 🔗 **Borrowed.** MonoGS's monocular depth prior follows the Godard MonoDepth2 lineage covered in Ch.11. MonoGS applies the premise that self-supervised monocular depth can recover structure without direct depth supervision to Gaussian initialization.

Matsuki worked at Imperial's Dyson Robotics Lab, as did Sucar on iMAP and Bloesch on CodeSLAM under Davison's supervision. MonoGS reflects the lab's transition from implicit MLPs to explicit Gaussian representations.

On the TUM-RGBD dataset, MonoGS recorded an average ATE RMSE of 4.44 cm monocular and 1.58 cm RGB-D. Rendering quality in the RGB-D setting on Replica reached an average PSNR of 37.50 dB, comparable to other same-generation GS-SLAM systems.

---

## RTG-SLAM and real-time processing

Speed remained a problem for GS-SLAM systems because neither GS-SLAM nor SplaTAM operated convincingly in real time. Peng Zhexi's group at Zhejiang University published [RTG-SLAM (SIGGRAPH 2024)](https://arxiv.org/abs/2404.19706) in 2024 with real-time performance as an explicit objective.

RTG-SLAM controls the number of Gaussians by optimizing only those that contribute substantially to the current camera view. It initializes Gaussians from surfels (surface elements), preserving geometry with fewer primitives. On the Replica dataset, it approached real-time throughput.

---

## Representation choices after 3DGS

The rapid growth of 3DGS papers after 2024 broadened the representation choices in SLAM mapping. TSDF and occupancy grids remain in use for embedded, planning, and safety applications, while NeRF and 3DGS are selected for different objectives. 3DGS papers emphasize rendering speed and explicit primitive updates, but that trend alone does not establish that the other representations left the mainstream.

The shift reflected both representation design and compatibility with available hardware. GPU rasterizers are more highly optimized than GPU ray marchers, and 3DGS's ability to use the existing graphics pipeline accelerated its adoption relative to NeRF.

> 🔗 **Borrowed.** 3DGS directly inherits differentiable scene optimization from NeRF. Mildenhall et al. (2020) established the use of gradients and a photometric loss to connect observations with rendering. Kerbl retained that framework while replacing the implicit MLP with explicit Gaussians.

> 📜 **Prediction vs. outcome.** In §7.4, Limitations, of the 3DGS paper (2023), Kerbl et al. identified elongated artifacts and popping in sparsely observed regions, the absence of regularization, and memory consumption of more than 20 GB during training and several hundred MB when rendering large scenes. They proposed antialiasing, more principled culling, and point-cloud compression as future work. The [Compact 3DGS](https://arxiv.org/abs/2311.13681) line and [Niedermayr et al.](https://arxiv.org/abs/2401.02436) directly addressed compression in 2024. The original paper did not explicitly propose dynamic-scene extensions such as [4DGS](https://arxiv.org/abs/2310.08528) and [Deformable 3DGS](https://arxiv.org/abs/2309.13101), or generation and editing systems such as [DreamGaussian](https://arxiv.org/abs/2309.16653) and [GaussianEditor](https://arxiv.org/abs/2311.14521), but these became separate research directions around 2024.

---

## 🧭 Still open

Memory scaling. The number of Gaussians grows linearly with scene size. A few hundred thousand primitives may suffice for the indoor Replica dataset, but an outdoor city block can require tens of millions. Researchers are studying Gaussian pruning and level-of-detail hierarchies, but no consensus exists on managing the trade-off between memory and rendering quality at large scale. The Compact 3DGS line (Lee et al. 2024, Niedermayr et al. 2024) explores compression.

Semantic integration. In 2023, [LERF](https://arxiv.org/abs/2303.09553) combined language features with NeRF, while [LangSplat](https://arxiv.org/abs/2312.16084) combined them with a Gaussian representation, and later work coupled semantic Gaussians to SLAM. A common protocol that compares real-time updates, tracking quality, and semantic accuracy across scenes and hardware has not settled. Interference between jointly optimized semantic and geometric variables remains a central evaluation target.

Dynamic scenes. 4DGS and Deformable 3DGS added a time dimension to Gaussians. In SLAM, dynamic objects move independently of the background and require separate treatment. GS-SLAM (Yan et al. 2023), SplaTAM (Keetha et al. 2024), and MonoGS (Matsuki et al. 2024) all retain a static-world assumption. Ch.15b separately traces SLAM's treatment of moving objects, from mask-based outlier rejection and multi-object factor graphs to deformable reconstruction.

3DGS also left its initialization unresolved. Gaussians could originate from an SfM point cloud or a depth sensor, but placing them required a known pose, while estimating a pose required an existing map. Systems handled this dependency differently: RGB-D methods used measured depth, while MonoGS formed initial depth hypotheses internally without an external depth predictor. DUSt3R and its successors, covered in Ch.16, instead learned geometry directly rather than initializing it from an existing representation.
