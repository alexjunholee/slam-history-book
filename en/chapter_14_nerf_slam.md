# Ch.14 — The NeRF Shock and Its Graft onto SLAM: iMAP → NICE-SLAM

What if a learned representation were used not for tracking but for *the map itself*? Edgar Sucar at Imperial College answered that question with iMAP in 2021, and the material for the answer came from outside SLAM. It was NeRF.

In March 2020, Ben Mildenhall and colleagues posted [Mildenhall et al. 2020. NeRF](https://arxiv.org/abs/2003.08934) to arXiv, and the paper synthesized new-viewpoint photographs from eight images. Those photographs carried the grain of light and shadow. The SLAM community first read this as a rendering problem. Not a way to *build* a map, but a way to *display* one. Shifting that reading took fourteen months. At ICCV 2021, when Sucar presented iMAP, it became clear that NeRF could serve not as a rendering tool but as the map representation itself. iMAP extended the line of KinectFusion (Ch.9). It was the first implementation of the hypothesis that an implicit neural field could replace a TSDF voxel grid.

NeRF did not come out of thin air. Over the course of 2019 three strands of coordinate-based MLP representation of 3D went off almost simultaneously. [Park et al.'s DeepSDF](https://arxiv.org/abs/1901.05103) described object surfaces implicitly with an MLP that took coordinates in and returned signed distance; [Mescheder et al.'s Occupancy Networks](https://arxiv.org/abs/1812.03828) made the same coordinate input emit occupancy probability; [Sitzmann et al.'s SRN](https://arxiv.org/abs/1906.01618) stored a scene feature vector at each coordinate and composed images through differentiable ray marching. The same mathematical frame: feed coordinates in, get a field value out. Mildenhall et al. 2020 NeRF added volume rendering integration and positional encoding to this frame and closed it onto view synthesis. What iMAP inherited was not one paper but that whole one-year lineage.

---

## NeRF: MLP-based spatial representation

NeRF's core idea is that a single MLP holds the entire 3D space implicitly. Input is spatial coordinate $(x, y, z)$ and viewing direction $(\theta, \phi)$. Output is the color $(r, g, b)$ and density $\sigma$ at that position. How does this represent a full scene.

Rendering uses the volume rendering equation. A ray leaving camera origin $\mathbf{o}$ in direction $\mathbf{d}$ is sampled along parameter $t$:

$$\hat{C}(\mathbf{r}) = \int_{t_n}^{t_f} T(t)\,\sigma\!\left(\mathbf{r}(t)\right) \mathbf{c}\!\left(\mathbf{r}(t), \mathbf{d}\right)\, dt$$

Here $T(t) = \exp\!\left(-\int_{t_n}^{t} \sigma(\mathbf{r}(s))\, ds\right)$ is the accumulated transmittance — the probability the ray reaches $t$ unblocked. The integral is approximated by a piecewise Riemann sum.

> 🔗 **Borrowed.** The volume rendering equation comes from [Kajiya & Von Herzen (1984)](https://courses.cs.duke.edu/cps296.8/spring03/papers/RayTracingVolumeDensities.pdf), a classical graphics paper. For nearly forty years it was a physics-based tool for offline rendering; Mildenhall turned it into the loss function of a reverse-direction optimization.

The problem that MLPs fail to learn high-frequency spatial signals was solved in the Mildenhall et al. (2020) NeRF paper by positional encoding. Projecting coordinates $(x, y, z)$ through sine and cosine functions at multiple frequencies lets the network learn fine textures and sharp boundaries:

$$\gamma(p) = \left(\sin(2^0 \pi p),\, \cos(2^0 \pi p),\, \ldots,\, \sin(2^{L-1} \pi p),\, \cos(2^{L-1} \pi p)\right)$$

> 🔗 **Borrowed.** NeRF's positional encoding appeared in the original Mildenhall et al. (2020) paper. The same year, [Tancik et al. (2020)](https://arxiv.org/abs/2006.10739) "Fourier Features Let Networks Learn High Frequency Functions" explained why this technique works through neural tangent kernel (NTK) theory.

NeRF's training runs backward. Images taken from known camera poses are compared with rendered outputs, and the per-pixel L2 loss is minimized. Once optimization ends, the MLP weights themselves store the scene's geometry and appearance. No voxels, no mesh. Space sits inside the network parameters.

The original NeRF had clear weaknesses, though. Training took hours, and the model was specialized to one scene with camera poses obtained beforehand from external SfM such as COLMAP. Transplanting this to SLAM demands that pose estimation and map learning happen jointly, at something close to real time.

---

## iMAP: the first neural implicit SLAM

Edgar Sucar at Imperial College's Dyson Robotics Lab presented [Sucar et al. 2021. iMAP](https://doi.org/10.1109/ICCV48922.2021.00612) at ICCV 2021, and that was the attempt. **iMAP** (Implicit MAP) took RGB-D camera input and optimized poses while using a single MLP as the map.

The structure is two alternating optimization loops. The *mapping* loop samples rays from the current keyframe and a set of randomly sampled past keyframes and updates the MLP. The *tracking* loop freezes the MLP and optimizes the current frame's pose against rendering loss. Both loops operate on one shared MLP.

Two loss terms. A color loss $\mathcal{L}_{\text{color}} = \|\hat{C} - C\|_2^2$ and a depth loss $\mathcal{L}_{\text{depth}} = \|\hat{D} - D\|_2^2$. Because iMAP uses RGB-D, depth supervision is available and geometry learning stayed stable.

iMAP was a proof of concept. It ran on small indoor scenes but had two structural problems. First, a single MLP forgot earlier regions as new regions were added. The catastrophic forgetting problem of neural networks. Sucar partially mitigated this through keyframe replay, but that was not a root solution. Second, as scenes grew, the representational capacity of a single MLP ran short. An MLP's forward pass treats the whole space as one function regardless of parameter count.

> 📜 **Prediction vs. outcome.** Sucar wrote in the iMAP Conclusion that "future directions for iMAP include how to make more structured and compositional representations that reason explicitly about the self similarity in scenes." The structured and compositional direction did become the central stem of follow-up work. Five months later ETH Zürich's pre-release of NICE-SLAM cut space hierarchically with a multi-resolution voxel feature grid, and Wang et al.'s Co-SLAM (2023) fused hash grid and coordinate encoding to push near-real-time performance of 10–17 Hz on an RTX 3090Ti. The "self-similarity, explicitly reasoned about" branch, however, did not develop much in the main line of NeRF-SLAM, and the lineage that refined a single MLP also drifted from the center. `[partial hit]`

---

## NICE-SLAM: hierarchical grid and scalability

The direct answer to iMAP's single-MLP problem came from Zihan Zhu and Songyou Peng at ETH Zürich in [Zhu et al. 2022. NICE-SLAM](https://arxiv.org/abs/2112.12130), presented at CVPR 2022. **NICE-SLAM** (Neural Implicit Scalable Coding for SLAM) combined a multi-resolution voxel feature grid with a small MLP decoder in place of a single MLP.

The idea is to split space into an explicit voxel grid but place a learnable feature vector at each voxel. At rendering time, features from the voxels around the sample coordinate are combined by trilinear interpolation and pushed through the small MLP to produce color and occupancy. The MLP does not need to be large. Most of the spatial information lives in the grid.

NICE-SLAM stacked three resolutions of grid hierarchically: the coarse grid holds overall geometry shape, the middle grid holds structural detail, and the fine grid holds texture. When a new region is added, only the features of the corresponding voxels need updating, so catastrophic forgetting of other regions was largely reduced.

In tracking, NICE-SLAM, similarly to iMAP, froze the MLP and grid features and optimized pose. In mapping it updated the grid features. On Replica and ScanNet it handled larger spaces than iMAP and reached higher detail quality.

There were limits. The grid's own memory grew as the cube of resolution. A room or two indoors could be handled, but scaling to multi-story buildings or outdoors remained unsolved. Speed was also far from real time.

Thomas Müller's [Müller et al. 2022. Instant-NGP](https://nvlabs.github.io/instant-ngp/) attacked this bottleneck from another angle at SIGGRAPH 2022. A hash-table-based feature encoding solved the memory explosion of voxel grids and cut training time from minutes to seconds. Instant-NGP was not a SLAM paper, but subsequent NeRF-SLAM work almost uniformly adopted hash encoding.

> 🔗 **Borrowed.** NICE-SLAM's multi-resolution feature grid overlaps in time with Instant-NGP's hash encoding and was designed independently, but in actual NeRF-SLAM implementations Instant-NGP's hash grid quickly replaced the NICE-SLAM grid. It is also the logical succession from KinectFusion (Ch.9), which stored TSDF in a grid, to storing features in a grid.

---

## Co-SLAM and NeRF-SLAM: two integration directions

From late 2022 onward, several systems split after iMAP and NICE-SLAM. One direction was to make the implicit representation more efficient; the other was to couple the robust backend of classical SLAM with a NeRF map.

UCL's [Wang et al. (2023) **Co-SLAM**](https://arxiv.org/abs/2304.14377) belongs to the first. It used joint coordinate and parametric encoding, combining a multi-resolution hash grid with one-blob encoding. The two representations were designed to complement each other, aiming at fast convergence and surface completeness together. The hash grid quickly filled observed dense regions, while the coordinate encoding provided a smooth prior over unobserved regions. 15–17 Hz on Replica with an RTX 3090. The first point at which NeRF-based SLAM reached near-real-time territory.

In the same CVPR that year, Idiap/EPFL's [Johari et al.'s **ESLAM**](https://arxiv.org/abs/2211.11704) solved a similar problem from a different angle. Instead of a 3D feature grid it used multi-scale axis-aligned feature planes, cutting memory growth from $O(n^3)$ to $O(n^2)$, and took TSDF rather than volume density as the decoding target to accelerate convergence.

Antoni Rosinol (MIT) released [**NeRF-SLAM**](https://arxiv.org/abs/2210.13641) in 2023, taking a different approach. It kept classical SLAM tracking and backend (factor graph optimization) as they were and replaced only the map representation with NeRF. Poses and dense depth were both delivered by the DROID-SLAM frontend. Rosinol took those poses, depths, and uncertainties as input and built an Instant-NGP-based map in parallel.

> 🔗 **Borrowed.** The NeRF-SLAM backend operates on top of Dellaert's factor graph optimization (Ch.6). Even under the hypothesis that "NeRF can change the map," the core mathematics of pose estimation sat unchanged on the graph structure established after 2005.

Rosinol chose modularity. Rather than forcing NeRF into the whole pipeline, he swapped only the map representation layer. Classical SLAM functions such as loop closure stayed in place.

---

## The structural limit of iMAP and what it means

Looking back, iMAP's significance lies in the concept more than the performance. It was the first system to show that a single MLP could hold the whole scene and be updated in near real time while pose was also optimized.

The root problem of a single MLP is the absence of locality. Rendering any part of space goes through the entire MLP. Two consequences follow. First, learning a new region shifts all the weights and degrades the representation of old regions (catastrophic forgetting). Second, as the scene grows, the spatial variety a single MLP must carry increases, requiring a larger network and more iterations. Representation capacity is tied linearly to parameter count, while scene complexity scales with spatial volume. A representation without locality grows more unfavorable as scale increases.

NICE-SLAM's grid, Instant-NGP's hash encoding, Co-SLAM's dual encoding all answered this locality problem. Partitioning space locally so that each part remembers only its own region makes new information less intrusive to old memory, and decouples rendering cost for a given region from overall scene size.

---

## 🧭 Still open

**Real-time NeRF-SLAM.** As of 2023, iMAP and NICE-SLAM were far from real time, and Co-SLAM reached 10–17 Hz on an RTX 3090Ti to touch near-real-time, yet still fell short of real time on mobile and robotic embedded hardware. Gaussian Splatting (Ch.15) later handled the speed problem differently, through a return to explicit representations, but classical real-time SLAM with implicit neural fields themselves (30 fps or more, without a consumer GPU) remains unfinished. Even as Instant-NGP pushed rendering speed up dramatically, overall throughput of the joint tracking and mapping loop remains constrained.

**Large-scale outdoor environments.** Efforts such as [Block-NeRF](https://arxiv.org/abs/2202.05263) (2022, Tancik et al.) that partition space into many local NeRFs exist, but have not meshed cleanly with SLAM's loop closure and global consistency requirements. City-scale NeRF-SLAM is an open problem.

**Semantic and editable implicit maps.** A NeRF map is optimized for rendering, which makes semantic label insertion and post-hoc editing difficult. Operations like "remove this object from the map" or "classify this region for a different purpose" are far more inconvenient than on a TSDF or point cloud. Language-guided NeRF editing work (the [LERF](https://arxiv.org/abs/2303.09553), [Nerfstudio](https://arxiv.org/abs/2302.04264) ecosystem) is in progress, but real-time integration with SLAM pipelines is still at the research stage as of 2026.

---

While iMAP and NICE-SLAM pushed implicit fields to their limit, a part of the research community was looking the other way. Rather than confining the map implicitly inside MLP weights or a feature grid, scattering space with millions of small ellipsoids explicitly placed in it could give fast rendering and intuitive editing. Before Bernhard Kerbl's paper at SIGGRAPH 2023 it was still a hypothesis.
