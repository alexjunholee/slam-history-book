# Chapter 17. NeRF-Based SLAM

In 2021, Edgar Sucar at Imperial College shifted learned representations out of the tracking frontend and into *the map itself*. His iMAP system adapted NeRF—a technique born outside robotics—to turn an implicit neural field into a live SLAM map.

Just over a year earlier, in March 2020, Ben Mildenhall and his colleagues had posted [Mildenhall et al. 2020. NeRF](https://arxiv.org/abs/2003.08934) to arXiv, optimizing a continuous volumetric scene function from posed images to synthesize novel views. NeRF was conceived purely for offline rendering. At ICCV 2021, Sucar reframed it for SLAM, demonstrating that an implicit neural field could support joint optimization of both scene geometry and camera trajectory—in effect recasting the dense TSDF mapping of KinectFusion (Chapter 12) inside the weights of a neural network.

NeRF itself sat atop a sudden 2019 wave of coordinate-based 3D MLPs: [Park et al.'s DeepSDF](https://arxiv.org/abs/1901.05103) mapped 3D coordinates to signed distance, [Mescheder et al.'s Occupancy Networks](https://arxiv.org/abs/1812.03828) mapped them to occupancy probability, and [Sitzmann et al.'s SRN](https://arxiv.org/abs/1906.01618) stored local feature vectors rendered via differentiable ray marching. Mildenhall et al. unified this coordinate-field paradigm with volume-rendering integration and positional encoding to achieve photorealistic view synthesis. iMAP inherited that entire two-year lineage rather than any single paper.

---

## 17.1 NeRF: MLP-based spatial representation

At its core, NeRF encodes an entire 3D scene inside the weights of a single multilayer perceptron (MLP). Given a 3D position $(x, y, z)$ and viewing direction $(\theta, \phi)$, the network predicts local color $(r, g, b)$ and volume density $\sigma$. To turn those point queries into a rendered pixel, NeRF integrates along a camera ray leaving origin $\mathbf{o}$ in direction $\mathbf{d}$ over near and far bounds $t_n$ and $t_f$:

$$\hat{C}(\mathbf{r}) = \int_{t_n}^{t_f} T(t)\,\sigma\!\left(\mathbf{r}(t)\right) \mathbf{c}\!\left(\mathbf{r}(t), \mathbf{d}\right)\, dt$$

where $T(t) = \exp\!\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s))\, ds\right)$ is the accumulated transmittance—the probability that the ray travels to depth $t$ without hitting an occluding surface—approximated numerically by a piecewise Riemann sum.

> 🔗 **Borrowed.** The volume-rendering integral comes from the classic graphics paper [Kajiya & Von Herzen (1984)](https://courses.cs.duke.edu/cps296.8/spring03/papers/RayTracingVolumeDensities.pdf). For nearly four decades it served as a forward physics model for offline rendering; Mildenhall et al. made it differentiable, turning ray integration into an inverse optimization objective for reconstructing scenes from photographs.

Because standard MLPs struggle to represent high-frequency detail, Mildenhall et al. (2020) passed each spatial coordinate through a multi-frequency sine-cosine mapping before feeding it to the network, enabling sharp edges and fine textures:

$$\gamma(p) = \left(\sin(2^0 \pi p),\, \cos(2^0 \pi p),\, \ldots,\, \sin(2^{L-1} \pi p),\, \cos(2^{L-1} \pi p)\right)$$

> 🔗 **Borrowed.** Positional encoding appeared directly in Mildenhall et al. (2020). Later that year, [Tancik et al. (2020)](https://arxiv.org/abs/2006.10739) explained why Fourier features unlock high-frequency learning in coordinate MLPs through neural tangent kernel (NTK) theory in "Fourier Features Let Networks Learn High Frequency Functions."

Once trained against a per-pixel L2 photometric loss over posed input views, the MLP weights capture geometry and appearance with neither voxels nor meshes. Yet adapting the original NeRF to SLAM faced two steep hurdles: training took hours per scene, and camera poses were assumed known in advance from an external SfM pipeline such as COLMAP. A SLAM system had to recover camera poses and build the neural map together, near real time.

---

## 17.2 iMAP: early neural implicit SLAM

At ICCV 2021, Edgar Sucar of Imperial College's Dyson Robotics Lab answered that challenge with [Sucar et al. 2021. iMAP](https://doi.org/10.1109/ICCV48922.2021.00612). **iMAP** (Implicit MAP) ingested RGB-D streams and maintained a single MLP as its sole map representation, alternating between two tightly coupled optimization loops over that shared network.

In the *mapping* loop, iMAP samples rays from the latest keyframe alongside a random subset of past keyframes to update the MLP weights. In the *tracking* loop, it freezes those weights and optimizes the current camera pose against the rendering residual. Both loops minimize a combined photometric and geometric objective, $\mathcal{L}_{\text{color}} = \|\hat{C} - C\|_2^2$ and $\mathcal{L}_{\text{depth}} = \|\hat{D} - D\|_2^2$, using RGB-D depth measurements to anchor geometry during online updates.

As a proof of concept on small indoor rooms, iMAP succeeded, yet it immediately exposed two structural bottlenecks. Because every scene update modified the same global weights, learning a new room degraded previously mapped areas—a classic case of catastrophic forgetting that keyframe replay could only soften, never eliminate. Moreover, evaluating one monolithic MLP across an expanding space quickly exhausted its capacity and compute budget.

> 📜 **Prediction vs. outcome.** In iMAP's conclusion, Sucar wrote that "future directions for iMAP include how to make more structured and compositional representations that reason explicitly about the self similarity in scenes." Structured and compositional representations soon became the field's main direction: five months later, an ETH Zürich pre-release of NICE-SLAM partitioned space hierarchically with a multi-resolution voxel feature grid, and Wang et al.'s Co-SLAM (2023) paired hash-grid and coordinate encodings to reach 10–17 Hz on an RTX 3090 Ti. By contrast, explicit reasoning about scene self-similarity drew far less attention in mainstream NeRF-SLAM, and single-MLP architectures faded from central use.

---

## 17.3 NICE-SLAM: hierarchical grid and scalability

Zihan Zhu and Songyou Peng at ETH Zürich tackled iMAP's single-MLP bottleneck in [Zhu et al. 2022. NICE-SLAM](https://arxiv.org/abs/2112.12130) at CVPR 2022. **NICE-SLAM** (Neural Implicit Scalable Coding for SLAM) replaced the monolithic network with a multi-resolution voxel feature grid paired with lightweight MLP decoders.

By anchoring learnable feature vectors directly on an explicit spatial grid, NICE-SLAM offloaded most geometric storage from network weights to local grid cells. During rendering, each sample coordinate trilinearly interpolates its surrounding voxel features, and a compact MLP decodes them into color and occupancy. Three hierarchical geometry grids capture coarse-to-fine structure alongside a separate color grid and decoder; crucially, mapping a new region updates only the local voxel features there, sharply curbing catastrophic forgetting across the rest of the map.

Following iMAP's split between tracking and mapping, NICE-SLAM froze the grids and decoders to optimize pose, then unfroze local features during mapping. On Replica and ScanNet, it scaled to larger indoor spaces with crisper reconstruction detail than iMAP. Even so, dense 3D voxel grids imposed their own wall: memory grew with the cube of spatial resolution, restricting the system to one or two rooms and keeping frame rates well below real time.

Relief arrived soon after from graphics. At SIGGRAPH 2022, Thomas Müller's [Müller et al. 2022. Instant-NGP](https://nvlabs.github.io/instant-ngp/) combined multi-resolution hash encoding with fused GPU kernels, demonstrating scene training in seconds on supported NVIDIA GPUs. Though not a SLAM paper, Instant-NGP's hash grid was rapidly absorbed into subsequent NeRF-SLAM pipelines.

> 🔗 **Borrowed.** NICE-SLAM's multi-resolution feature grid emerged independently around the same time as Instant-NGP's hash encoding, yet Instant-NGP's compact hash table soon displaced dense feature grids in many NeRF-SLAM systems. Conceptually, the feature grid also extends KinectFusion's voxel representation (Chapter 12), swapping hand-crafted TSDF values for learned local features.

---

## 17.4 Co-SLAM and NeRF-SLAM: two integration directions

From late 2022 onward, successors to iMAP and NICE-SLAM branched along two complementary tracks: redesigning the implicit encoding for speed and memory efficiency, or decoupling neural mapping from a classical SLAM tracker.

Along the representation track, UCL's [Wang et al. (2023) **Co-SLAM**](https://arxiv.org/abs/2304.14377) paired a multi-resolution hash grid with a one-blob coordinate encoding. The hash table let densely observed surfaces converge rapidly, while the smooth coordinate encoding regularized unobserved holes—a hybrid that pushed Replica throughput to 15–17 Hz on an RTX 3090 Ti, bringing neural implicit SLAM near real time. At the same CVPR, [Johari et al.'s **ESLAM**](https://arxiv.org/abs/2211.11704) from Idiap and EPFL attacked voxel memory growth directly by replacing the 3D grid with multi-scale axis-aligned feature planes, dropping memory complexity from $O(n^3)$ to $O(n^2)$ while decoding TSDF values instead of raw density for faster convergence.

Antoni Rosinol at MIT pursued the second, architectural track in [**NeRF-SLAM**](https://arxiv.org/abs/2210.13641) (2022). Rather than optimizing camera pose through slow ray-marching gradients, NeRF-SLAM entrusted tracking and dense depth estimation to DROID-SLAM, extracted marginal covariances from its Hessian, and fed those posed, uncertainty-weighted depth maps into a dedicated Instant-NGP mapping thread.

> 🔗 **Borrowed.** NeRF-SLAM builds directly on the poses, dense depths, and uncertainties output by DROID-SLAM's recurrent updates and Dense Bundle Adjustment, pairing them with an Instant-NGP radiance field. Rather than replacing the pose estimator, the neural field consumes its outputs to construct a dense photometric map in real time.

By separating dense monocular tracking from neural mapping, NeRF-SLAM ensured that pose estimation no longer waited on differentiable rendering—allowing trackers and neural map representations to advance independently.

---

## 17.5 The structural limitation of iMAP

Why did iMAP's single-MLP design hit a ceiling so quickly, even after proving that joint pose-and-map optimization was feasible?

The root cause is lack of spatial locality. Querying any point in a single MLP evaluates the entire network, and updating weights for a newly visited corner alters predictions everywhere else, inducing catastrophic forgetting. As the explored volume grows, one fixed-parameter network must absorb ever more spatial variation, demanding larger layers and more gradient steps. Parameter count scales with network width and depth rather than scene volume, making a global implicit function fundamentally mismatched to expanding environments.

NICE-SLAM's voxel grid, Instant-NGP's hash table, and Co-SLAM's dual encoding all restored locality by partitioning space into regional parameters—isolating updates from previously mapped rooms and decoupling per-ray rendering cost from overall map size.

---

## Still open

**Real-time NeRF-SLAM.** Despite major speedups over iMAP and NICE-SLAM, even Co-SLAM's 10–17 Hz on a desktop RTX 3090 Ti remained far too demanding for embedded or mobile robots, let alone 30+ fps operation without high-end GPUs. Instant-NGP accelerated ray queries dramatically, yet closing the full tracking-and-mapping loop in real time remained constrained—paving the way for explicit Gaussian Splatting in Chapter 18.

**Large-scale outdoor environments.** Partitioning space into many local radiance fields, as in [Block-NeRF](https://arxiv.org/abs/2202.05263) (2022, Tancik et al.), enables neighborhood-scale view synthesis, yet integrating partitioned fields with SLAM loop closures and global metric consistency remained unresolved at city scale.

**Semantic and editable implicit maps.** Because a NeRF stores geometry and appearance implicitly for rendering, editing individual objects or attaching semantic labels is far more awkward than modifying a point cloud or TSDF. [LERF](https://arxiv.org/abs/2303.09553) embedded language features for open-ended 3D queries without supporting object removal, and while toolkits such as [Nerfstudio](https://arxiv.org/abs/2302.04264) spurred rapid experimentation with semantic and editable radiance fields, real-time SLAM integration remained an open research problem as of 2026.

---

Even as iMAP and NICE-SLAM refined implicit fields, another camp questioned whether neural weights or voxel grids were needed at all. Millions of explicit 3D ellipsoids promised far faster rasterization and direct spatial editing than any implicit MLP—and splatting already had a twenty-year history stretching back to 2001 EWA splatting. At SIGGRAPH 2023, Bernhard Kerbl and his colleagues fused trainable 3D Gaussians with a custom tile-based differentiable rasterizer, triggering an immediate shift across dense SLAM.
