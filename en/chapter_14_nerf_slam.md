# Ch.14 — NeRF Enters SLAM: iMAP → NICE-SLAM

In 2021, Edgar Sucar at Imperial College used a learned representation not for tracking but for *the map itself*. His iMAP system adapted NeRF, a method developed outside SLAM.

In March 2020, Ben Mildenhall and his colleagues posted [Mildenhall et al. 2020. NeRF](https://arxiv.org/abs/2003.08934) to arXiv. The paper synthesized photorealistic novel views from calibrated images, and the SLAM community initially treated it as a rendering method rather than a map-building technique. Fourteen months later, Sucar's ICCV 2021 presentation of iMAP demonstrated that NeRF could serve as the map representation itself. Extending the KinectFusion lineage from Ch.9, iMAP was an early system testing whether an implicit neural field could replace a TSDF voxel grid.

NeRF followed several coordinate-based MLP representations of 3D introduced around 2019. [Park et al.'s DeepSDF](https://arxiv.org/abs/1901.05103) represented object surfaces with an MLP that mapped coordinates to signed distance. [Mescheder et al.'s Occupancy Networks](https://arxiv.org/abs/1812.03828) mapped the same input to occupancy probability. [Sitzmann et al.'s SRN](https://arxiv.org/abs/1906.01618) stored a scene feature vector at each coordinate and formed images through differentiable ray marching. All three mapped coordinates to a field value. Mildenhall et al.'s 2020 NeRF added volume-rendering integration and positional encoding, applying the formulation to view synthesis. iMAP inherited this entire one-year lineage rather than one paper alone.

---

## NeRF: MLP-based spatial representation

NeRF represents an entire 3D space implicitly in one MLP. Its inputs are a spatial coordinate $(x, y, z)$ and viewing direction $(\theta, \phi)$; its outputs are the color $(r, g, b)$ and density $\sigma$ at that position. Volume rendering combines these local values into an image of the full scene.

Rendering uses the volume rendering equation. A ray leaving camera origin $\mathbf{o}$ in direction $\mathbf{d}$ is sampled along parameter $t$:

$$\hat{C}(\mathbf{r}) = \int_{t_n}^{t_f} T(t)\,\sigma\!\left(\mathbf{r}(t)\right) \mathbf{c}\!\left(\mathbf{r}(t), \mathbf{d}\right)\, dt$$

Here $T(t) = \exp\!\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s))\, ds\right)$ is the accumulated transmittance, or the probability that the ray reaches $t$ without being blocked. A piecewise Riemann sum approximates the integral.

> 🔗 **Borrowed.** The volume-rendering equation comes from the classical graphics paper [Kajiya & Von Herzen (1984)](https://courses.cs.duke.edu/cps296.8/spring03/papers/RayTracingVolumeDensities.pdf). For nearly forty years, it served as a physics-based tool for offline rendering. Mildenhall used differentiable rendering to turn it into an optimization objective for reconstructing a scene from images.

Mildenhall et al. (2020) used positional encoding to address the difficulty MLPs have in learning high-frequency spatial signals. Projecting coordinates $(x, y, z)$ through sine and cosine functions at multiple frequencies allows the network to learn fine textures and sharp boundaries:

$$\gamma(p) = \left(\sin(2^0 \pi p),\, \cos(2^0 \pi p),\, \ldots,\, \sin(2^{L-1} \pi p),\, \cos(2^{L-1} \pi p)\right)$$

> 🔗 **Borrowed.** NeRF's positional encoding appeared in the original Mildenhall et al. (2020) paper. The same year, [Tancik et al. (2020)](https://arxiv.org/abs/2006.10739) "Fourier Features Let Networks Learn High Frequency Functions" explained why this technique works through neural tangent kernel (NTK) theory.

NeRF trains by comparing images captured from known camera poses with rendered outputs and minimizing a per-pixel L2 loss. After optimization, the MLP weights encode the scene's geometry and appearance without explicit voxels or a mesh.

The original NeRF also had clear limitations. Training took hours, and each model represented one scene whose camera poses had already been estimated by an external SfM system such as COLMAP. A SLAM system instead had to estimate poses and learn the map jointly at close to real-time speed.

---

## iMAP: early neural implicit SLAM

At ICCV 2021, Edgar Sucar of Imperial College's Dyson Robotics Lab presented [Sucar et al. 2021. iMAP](https://doi.org/10.1109/ICCV48922.2021.00612). **iMAP** (Implicit MAP) took RGB-D input, optimized camera poses, and represented the map with a single MLP.

The system alternates between two optimization loops. The *mapping* loop samples rays from the current keyframe and a random set of past keyframes to update the MLP. The *tracking* loop freezes the MLP and optimizes the current frame's pose against the rendering loss. Both loops use the same MLP.

The objective has two terms: a color loss $\mathcal{L}_{\text{color}} = \|\hat{C} - C\|_2^2$ and a depth loss $\mathcal{L}_{\text{depth}} = \|\hat{D} - D\|_2^2$. Because iMAP uses RGB-D input, direct depth supervision stabilizes geometry learning.

iMAP was a proof of concept that operated on small indoor scenes but had two structural limitations. First, the single MLP forgot earlier regions as it learned new ones, exhibiting catastrophic forgetting. Keyframe replay partially mitigated this effect without removing its cause. Second, the MLP's representational capacity became insufficient as scenes grew because every forward pass treated the entire space as one function.

> 📜 **Prediction vs. outcome.** In iMAP's conclusion, Sucar wrote that "future directions for iMAP include how to make more structured and compositional representations that reason explicitly about the self similarity in scenes." Structured and compositional representations became central to later work. Five months later, an ETH Zürich pre-release of NICE-SLAM partitioned space hierarchically with a multi-resolution voxel feature grid. Wang et al.'s Co-SLAM (2023) later combined hash-grid and coordinate encodings and reported more than 15 Hz on an RTX 3090 Ti. Explicit reasoning about self-similarity received less attention in mainstream NeRF-SLAM, and approaches that refined a single MLP also became less central.

---

## NICE-SLAM: hierarchical grid and scalability

Zihan Zhu and Songyou Peng at ETH Zürich addressed iMAP's single-MLP limitation in [Zhu et al. 2022. NICE-SLAM](https://arxiv.org/abs/2112.12130), presented at CVPR 2022. **NICE-SLAM** (Neural Implicit Scalable Coding for SLAM) replaced the single MLP with a multi-resolution voxel feature grid and a small MLP decoder.

NICE-SLAM partitions space into an explicit voxel grid and stores a learnable feature vector at each voxel. During rendering, trilinear interpolation combines features from the voxels surrounding a sample coordinate, and a small MLP decodes them into color and occupancy. Most spatial information resides in the grid, reducing the required MLP size.

NICE-SLAM organized three grid resolutions hierarchically. The coarse grid stored overall geometry, the middle grid stored structural detail, and the fine grid stored texture. Adding a new region required updating only its corresponding voxel features, substantially reducing catastrophic forgetting elsewhere.

Like iMAP, NICE-SLAM froze the MLP and grid features while optimizing pose during tracking. During mapping, it updated the grid features. On Replica and ScanNet, the system handled larger spaces than iMAP and reconstructed finer details.

The grid introduced its own limitations because memory grew with the cube of resolution. The system could handle one or two indoor rooms but did not scale to multi-story buildings or outdoor environments. It also remained far from real-time operation.

At SIGGRAPH 2022, Thomas Müller's [Müller et al. 2022. Instant-NGP](https://nvlabs.github.io/instant-ngp/) addressed this bottleneck with a hash-table-based feature encoding. The representation reduced the memory growth of voxel grids and cut training time from minutes to seconds. Although Instant-NGP was not a SLAM paper, most subsequent NeRF-SLAM systems adopted hash encoding.

> 🔗 **Borrowed.** NICE-SLAM's multi-resolution feature grid was designed independently at roughly the same time as Instant-NGP's hash encoding, but Instant-NGP's hash grid soon replaced it in many NeRF-SLAM implementations. The feature grid also extends the grid-based representation of KinectFusion (Ch.9), replacing stored TSDF values with learned features.

---

## Co-SLAM and NeRF-SLAM: two integration directions

From late 2022 onward, systems following iMAP and NICE-SLAM developed in two directions: more efficient implicit representations and combinations of a NeRF map with a classical SLAM backend.

UCL's [Wang et al. (2023) **Co-SLAM**](https://arxiv.org/abs/2304.14377) followed the first direction. It combined a multi-resolution hash grid with one-blob encoding in a joint coordinate and parametric representation. The hash grid represented densely observed regions efficiently, while the coordinate encoding supplied a smooth prior over unobserved areas, balancing convergence speed with surface completeness. The paper reported more than 15 Hz on Replica with an RTX 3090 Ti, bringing NeRF-based SLAM close to real-time operation.

At the same CVPR, [Johari et al.'s **ESLAM**](https://arxiv.org/abs/2211.11704) from Idiap and EPFL addressed the same problem differently. It replaced the 3D feature grid with multi-scale axis-aligned feature planes, reducing memory growth from $O(n^3)$ to $O(n^2)$, and decoded TSDF rather than volume density to accelerate convergence.

Antoni Rosinol at MIT released [**NeRF-SLAM**](https://arxiv.org/abs/2210.13641) in 2023. It retained classical SLAM tracking and factor-graph optimization while replacing only the map representation with NeRF. A DROID-SLAM frontend supplied poses and dense depth; NeRF-SLAM used those poses, depths, and uncertainties to construct an Instant-NGP-based map in parallel.

> 🔗 **Borrowed.** The NeRF-SLAM backend uses Dellaert's factor-graph optimization (Ch.6). NeRF changed the map representation while leaving the graph-based mathematics of pose estimation established after 2005 intact.

Rosinol used a modular design, replacing only the map representation rather than introducing NeRF throughout the pipeline. Classical SLAM functions such as loop closure remained in place.

---

## The structural limitation of iMAP

iMAP showed that a single MLP could represent an entire scene and be updated near real time while jointly optimizing pose, although its measured performance remained limited.

A single MLP lacks locality: rendering any region requires evaluating the entire network. Learning a new region changes all weights and can degrade older regions through catastrophic forgetting. A growing scene also increases the spatial variation that one MLP must represent, requiring a larger network and more iterations. Representation capacity grows with parameter count, whereas scene complexity grows with spatial volume, making a nonlocal representation increasingly inefficient at larger scales.

NICE-SLAM's grid, Instant-NGP's hash encoding, and Co-SLAM's dual encoding all addressed locality. Partitioning space allows each component to represent its own region, reducing interference with previously learned areas and decoupling the rendering cost of one region from the total scene size.

---

## 🧭 Still open

**Real-time NeRF-SLAM.** As of 2023, iMAP and NICE-SLAM were far from real time. Co-SLAM reported more than 15 Hz on an RTX 3090 Ti but remained too slow for mobile and embedded robotic hardware. Gaussian Splatting (Ch.15) later addressed speed by returning to an explicit representation, while implicit neural fields still did not support conventional real-time SLAM at 30 fps or more without a consumer GPU. Instant-NGP greatly accelerated rendering, but the combined tracking-and-mapping loop remained constrained.

**Large-scale outdoor environments.** Methods such as [Block-NeRF](https://arxiv.org/abs/2202.05263) (2022, Tancik et al.) partition space into many local NeRFs, but do not yet integrate cleanly with SLAM's requirements for loop closure and global consistency. City-scale NeRF-SLAM remains an open problem.

**Semantic and editable implicit maps.** Because a NeRF map is optimized for rendering, inserting semantic labels and editing the map afterward are difficult. Removing an object or reclassifying a region is substantially harder than in a TSDF or point cloud. Language-guided NeRF editing is under development in systems such as [LERF](https://arxiv.org/abs/2303.09553) and the [Nerfstudio](https://arxiv.org/abs/2302.04264) ecosystem, but real-time integration with SLAM remained a research problem as of 2026.

---

As iMAP and NICE-SLAM developed implicit fields, other researchers reconsidered explicit map representations. Millions of small ellipsoids placed throughout space promised faster rendering and more intuitive editing than a map encoded in MLP weights or a feature grid. Before Bernhard Kerbl's SIGGRAPH 2023 paper, that possibility remained a hypothesis.
