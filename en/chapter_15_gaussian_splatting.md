# Chapter 18. 3D Gaussian Splatting-Based SLAM

Although iMAP and NICE-SLAM brought neural fields into SLAM, implicit representations remained awkward to edit and slow to run. Updating iMAP's global MLP risked corrupting previously mapped rooms, and while NICE-SLAM curbed that interference with local feature grids, it still ran below 1 fps on an RTX 3090—leaving scene geometry locked inside opaque network weights.

At SIGGRAPH in August 2023, Bernhard Kerbl, Georgios Kopanas, Thomas Leimkuhler, and George Drettakis of INRIA offered a decisive turn in [their paper](https://arxiv.org/abs/2308.04079). Rather than abandoning the differentiable photometric optimization that NeRF had refined over the previous three years, 3D Gaussian Splatting (3DGS) replaced MLPs and voxel grids with millions of explicit ellipsoidal Gaussian primitives. Its rasterizer drew directly on Matthias Zwicker's twenty-year-old EWA splatting technique (2001), while its optimization preserved NeRF's differentiable-rendering loop. Within six months, the SLAM community had built a new wave of systems around it.

---

## 18.1 The structure of 3DGS

In 3DGS, a scene is an explicit collection of anisotropic 3D Gaussians. Each primitive carries a center $\boldsymbol{\mu} \in \mathbb{R}^3$, a 3D covariance matrix $\boldsymbol{\Sigma} \in \mathbb{R}^{3 \times 3}$, a learnable opacity $o \in (0,1]$, and view-dependent color parameterized by spherical harmonics. To keep covariance positive semi-definite during gradient descent, $\boldsymbol{\Sigma}$ is factored into a scaling vector $\mathbf{s}$ and a unit rotation quaternion $\mathbf{q}$:

$$\boldsymbol{\Sigma} = \mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$$

To render an image, 3D Gaussians are projected onto the image plane, sorted by depth, and alpha-blended in front-to-back order. Each primitive's effective opacity $\alpha_i$ multiplies its learnable opacity $o_i$ by the 2D Gaussian density $G_i(\mathbf{x})$ at pixel $\mathbf{x}$, yielding pixel color:

$$C = \sum_{i \in N} c_i \alpha_i \prod_{j<i}(1 - \alpha_j), \quad \alpha_i = o_i G_i(\mathbf{x})$$

Whereas NeRF approximates a continuous volume-rendering integral by marching samples along every ray, 3DGS feeds primitives straight into a GPU rasterization pipeline with custom CUDA tile-rasterizer kernels for both forward and backward passes. On a single RTX 3090, a trained scene renders at well over 30 fps—though this pure novel-view rendering speed cannot be compared directly as a simple speedup ratio against NICE-SLAM's full tracking-and-mapping loop.

Optimization typically starts from a sparse SfM point cloud and alternates parameter updates with adaptive **densification**—splitting, cloning, and pruning primitives. Where large view-space position gradients signal under-reconstructed detail, oversized Gaussians split into two children while tiny Gaussians clone themselves; meanwhile, near-transparent primitives whose opacity drops below a threshold are periodically culled.

> 🔗 **Borrowed.** 3DGS's rasterization-based splatting descends directly from Zwicker et al.'s [EWA splatting (2001)](https://www.cs.umd.edu/~zwicker/publications/EWAVolumeSplatting-VIS01.pdf), which wrapped point samples in elliptical weighted-average kernels for smooth point-cloud rendering. Kerbl replaced fixed kernels with learnable Gaussians and accelerated the pass with a GPU tile rasterizer.

---

## 18.2 Structural fit between 3DGS and SLAM

From a SLAM perspective, implicit fields fought against incremental mapping at an architectural level. A global MLP had to adjust shared weights for every new frame, inviting catastrophic forgetting and forcing capacity trade-offs as maps expanded; voxel grids localized those updates at the price of cubic memory growth.

An explicit Gaussian map aligned far more cleanly with incremental SLAM. Whenever a new keyframe reveals unobserved geometry, the system can insert Gaussians strictly in that local volume without disturbing distant regions. Densification pairs naturally with keyframe addition, while tile rasterization delivers NeRF-grade visual fidelity at interactive frame rates. Unsurprisingly, 3DGS-SLAM preprints began appearing in rapid succession before the end of 2023.

---

## 18.3 GS-SLAM: an early attempt

In November 2023, Chi Yan (HKU) and collaborators posted [Yan et al. 2023. GS-SLAM](https://arxiv.org/abs/2311.11700) to arXiv—one of the earliest attempts to wrap 3DGS inside a complete tracking-and-mapping pipeline.

Following classical SLAM separation, GS-SLAM alternates between estimating the current frame's pose and refining the Gaussian map. To make optimization tractable, Yan introduced two mechanisms: adaptive Gaussian expansion, which seeds new primitives in under-covered regions upon keyframe insertion, and geometry-aware Gaussian selection, which restricts backpropagation to primitives contributing meaningfully to the rendering residual.

During tracking, GS-SLAM optimizes camera pose by minimizing a rendered L1 photometric loss across sampled pixels:

$$\mathcal{L}_{track} = \sum_m \|\mathbf{C}_m - \hat{\mathbf{C}}_m\|_1$$

During mapping, it combines color and depth L1 losses alongside the original 3DGS photometric-structural objective, $(1-\lambda)\mathcal{L}_1 + \lambda\mathcal{L}_{D\text{-}SSIM}$ with $\lambda=0.2$—all end-to-end differentiable with respect to camera pose through the custom rasterizer. On Replica, GS-SLAM matched NICE-SLAM's PSNR at substantially higher throughput, though it still depended on an RGB-D sensor and remained unvalidated outdoors.

---

## 18.4 SplaTAM: silhouette-based densification

Soon after, Nikhil Keetha and colleagues at Carnegie Mellon simplified Gaussian densification in [Keetha et al. 2024. SplaTAM (CVPR)](https://arxiv.org/abs/2312.02126).

Like GS-SLAM, SplaTAM alternates between optimizing camera pose with the map frozen and updating Gaussian parameters with poses held fixed. What distinguishes SplaTAM is how it decides where the map needs new primitives: a **silhouette mask** flags pixels in the current view that existing Gaussians fail to cover, and SplaTAM seeds new Gaussians directly into those empty regions.

> 🔗 **Borrowed.** SplaTAM adapts PTAM's keyframe-based map-management principle (Klein & Murray, 2007) to an explicit Gaussian representation. Selective keyframe insertion—which PTAM used to expand a sparse landmark map—becomes the trigger for silhouette-guided densification in SplaTAM.

The visual gain over earlier neural-field SLAM was stark: on Replica, SplaTAM achieved a PSNR of 34.11 dB compared with NICE-SLAM's 24.42 dB in the paper's benchmark table. Even so, in the CVPR paper's Limitations & Future Work section, Keetha noted sensitivity to motion blur, depth noise, and rapid rotations, as well as reliance on known camera intrinsics, dense depth sensors, and limited spatial scalability.

> 📜 **Prediction vs. outcome.** In SplaTAM's Limitations section (2024), Keetha singled out sensitivity to motion blur, depth noise, and aggressive rotation alongside reliance on known intrinsics, dense depth, and small-scale scenes. Later that same year, Matsuki's MonoGS (CVPR 2024) eliminated the depth-sensor requirement with a monocular RGB formulation, while removing calibrated intrinsics and scaling to large outdoor scenes remained active challenges through 2024–2025.

---

## 18.5 MonoGS: monocular RGB

[Matsuki et al. 2024. MonoGS (CVPR)](https://arxiv.org/abs/2312.06741), developed by Hidenobu Matsuki and colleagues at Imperial College's Dyson Robotics Lab, dropped the depth camera altogether, running 3DGS SLAM from a single monocular RGB stream.

Moving to monocular RGB reintroduces SLAM's classic scale ambiguity: projective geometry alone cannot fix metric scale. Matsuki optimized Gaussian geometry directly from multiview photometric residuals and added an **isotropy loss** to stabilize under-constrained primitives, though the resulting monocular trajectory remains defined only up to an arbitrary scale factor:

$$\mathcal{L}_{iso} = \sum_k \| \mathbf{s}_k - \bar{s}_k \mathbf{1} \|_1$$

Here $\mathbf{s}_k \in \mathbb{R}^3$ denotes the axis scales of the $k$-th Gaussian and $\bar{s}_k = \frac{1}{3}\sum_j s_{k,j}$ their mean. Penalizing deviations from $\bar{s}_k$ keeps Gaussians from stretching into needle-like ellipsoids along viewing rays when parallax is weak, suppressing artifacts that otherwise destabilize both rendering and pose tracking.

For tracking, MonoGS optimizes pose directly against the photometric difference between rendered and observed pixels. New Gaussians at pixels with valid rendered depth are initialized near that depth with tight variance, while uncovered pixels start at the frame's median rendered depth with high variance so subsequent views can triangulate and refine them—though neither mechanism supplies metric scale.

> 🔗 **Distinction.** MonoGS uses neither a pretrained monocular depth predictor nor an external tracking module. Its initialization therefore does not derive from the MonoDepth2 lineage.

Coming out of Davison's Dyson Robotics Lab—following Bloesch's CodeSLAM and Sucar's iMAP—MonoGS marked the group's pivot from latent codes and implicit MLPs to explicit Gaussian primitives. In [the initial 2023 arXiv release of MonoGS](https://arxiv.org/abs/2312.06741v1), average ATE RMSE on TUM-RGBD reached 4.44 cm in monocular mode (after scale alignment) and 1.58 cm in RGB-D mode (with rigid alignment only), while RGB-D rendering on Replica averaged 37.50 dB PSNR—squarely alongside peer GS-SLAM systems.

---

## 18.6 RTG-SLAM and real-time processing

Even with fast CUDA rasterization, neither GS-SLAM nor SplaTAM ran comfortably in real time across the full tracking-and-mapping loop. In 2024, Peng Zhexi's group at Zhejiang University tackled throughput head-on in [RTG-SLAM (SIGGRAPH 2024)](https://arxiv.org/abs/2404.19706).

Rather than updating the entire primitive set, RTG-SLAM caps Gaussian count by optimizing only primitives that strongly affect the current view and initializing Gaussians from compact **surfels** (surface elements), preserving surface geometry with far fewer parameters. On Replica, that pruning brought the system close to genuine real-time throughput.

---

## 18.7 Representation choices after 3DGS

The surge of 3DGS papers after 2024 widened SLAM's map-representation toolkit without rendering earlier structures obsolete. TSDFs and occupancy grids remain workhorses for embedded robotics, path planning, and safety-critical navigation, while NeRF and 3DGS serve goals centered on photorealistic rendering and dense appearance. Emphasizing 3DGS's speed and explicit updates captures its strength, not the disappearance of other map types.

At the same time, 3DGS's rapid adoption owed as much to silicon as to geometry: modern GPUs are engineered around hardware rasterizers rather than volumetric ray marchers, so aligning scene representation with the graphics pipeline gave 3DGS a decisive engineering tailwind over NeRF.

> 🔗 **Borrowed.** 3DGS directly inherits differentiable scene optimization from NeRF. Mildenhall et al. (2020) established how photometric gradients connect pixel observations to 3D scene parameters; Kerbl preserved that optimization engine while swapping the implicit MLP for explicit Gaussians.

> 📜 **Prediction vs. outcome.** In §7.4 (Limitations) of the 2023 3DGS paper, Kerbl et al. flagged needle-like artifacts and "popping" in sparsely observed regions, the lack of geometric regularization, and heavy memory footprints—over 20 GB during training and several hundred megabytes at render time on large scenes—pointing toward antialiasing, principled culling, and point-cloud compression. Compression soon materialized in [Compact 3DGS](https://arxiv.org/abs/2311.13681) and [Niedermayr et al.](https://arxiv.org/abs/2401.02436) in 2024, while directions scarcely anticipated in the original paper—dynamic extensions like [4DGS](https://arxiv.org/abs/2310.08528) and [Deformable 3DGS](https://arxiv.org/abs/2309.13101), or generative editing pipelines like [DreamGaussian](https://arxiv.org/abs/2309.16653) and [GaussianEditor](https://arxiv.org/abs/2311.14521)—quickly branched into major subfields of their own.

---

## Still open

**Memory scaling.** Primitive counts grow roughly linearly with scene volume: a Replica room needs a few hundred thousand Gaussians, whereas an outdoor city block can demand tens of millions. Pruning heuristics, level-of-detail hierarchies, and compression lines such as Compact 3DGS (Lee et al. 2024, Niedermayr et al. 2024) explore ways to tame that footprint, yet no standard protocol balances memory against visual fidelity at large scale.

**Semantic integration.** Following [LERF](https://arxiv.org/abs/2303.09553) (2023) for NeRF, [LangSplat](https://arxiv.org/abs/2312.16084) (2023) attached open-vocabulary language embeddings to 3D Gaussians, and subsequent SLAM prototypes followed suit. Even so, the field lacks shared benchmarks evaluating live updates, tracking accuracy, and semantic fidelity together, and joint optimization of geometric and semantic attributes can still degrade both.

**Dynamic scenes.** While 4DGS and Deformable 3DGS added time to static splats, GS-SLAM (Yan et al. 2023), SplaTAM (Keetha et al. 2024), and MonoGS (Matsuki et al. 2024) all assumed a static world. Handling moving and deforming objects—from mask-based outlier rejection to multi-object factor graphs and non-rigid reconstruction—forms a distinct lineage traced in Chapter 19.

3DGS also left initialization caught in a familiar loop: seeding Gaussians requires known camera poses or depth, yet estimating poses requires an existing map. RGB-D pipelines leaned on depth sensors, while MonoGS bootstrapped depth hypotheses internally without an external depth network. A separate line of research led by DUSt3R bypassed that chicken-and-egg problem altogether by learning to predict 3D geometry directly from raw images (→ Chapter 20).
