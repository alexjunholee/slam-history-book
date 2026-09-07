# Ch.9 — Dense/RGB-D: From KinectFusion to BundleFusion

When Richard Newcombe of Imperial College London presented KinectFusion at ISMAR in November 2011, its demo video drew more attention than the paper. A single handheld Kinect reconstructed an entire room as a 3D mesh in real time. DTAM, which Newcombe had released earlier that year, had pursued the same goal with a monocular camera; KinectFusion achieved it with an RGB-D sensor. The system combined the TSDF representation devised by Curless and Levoy for graphics in 1996, the ICP tracker introduced to robotics by Besl and McKay in 1992, and Microsoft's $150 Kinect sensor from 2010. Their combination began a brief, intense period of dense SLAM research. Davison's MonoSLAM (Ch.5) had used real-time, CPU-only processing to track sparse landmarks from a monocular camera. Newcombe's DTAM (Ch.8) then used a GPU for dense reconstruction through direct photometric optimization, while KinectFusion used the GPU with an RGB-D depth stream.

---

## 9.1 Dense reconstruction before Kinect

Dense 3D reconstruction was possible before 2011, but not in *real time*.

Offline pipelines could merge point clouds acquired by stereo or structured-light scanners, but indoor scanning rigs cost hundreds of thousands of dollars and saw little use outside laboratories. The SLAM community was already obtaining practical results from sparse landmarks, while dense reconstruction remained largely a graphics problem.

[Curless and Levoy's 1996 SIGGRAPH paper, "A Volumetric Method for Building Complex Models from Range Images"](https://graphics.stanford.edu/papers/volrange/volrange.pdf) introduced the **TSDF (Truncated Signed Distance Function)** to this graphics pipeline. The method partitions 3D space into a uniform voxel grid and accumulates at each voxel the signed distance to the nearest surface. Moving from the sensor toward the surface, the sign convention assigns a positive value to the free space in front of the surface and a negative value to the solid behind it. Truncation clips the absolute value at a threshold $t$, giving $\text{TSDF}(x) = \text{clip}(d(x), -t, +t)$. Each incoming depth frame updates the value by a weighted average, reducing noise and sharpening the surface over time. Marching cubes then extracts the surface at the TSDF's zero-crossing.

The method was accurate, but the voxel grid consumed substantial memory, and the available hardware could not update it in real time. For the next fifteen years, Curless and Levoy's paper remained primarily part of the graphics literature.

During those fifteen years, GPUs entered the GPGPU era and Kinect appeared.

---

## 9.2 KinectFusion and TSDF

Microsoft released Kinect for the Xbox 360 at about $150 in 2010. The sensor measured depth through structured light and streamed VGA-resolution depth maps at 30 Hz. Its precision was below that of research-grade ToF (Time-of-Flight) cameras, but its price was also far lower. Hackers responded first: open-source drivers appeared within weeks of the launch, followed by research applications.

Newcombe had by then moved to Microsoft Research Cambridge, where he was developing GPU-based dense SLAM with Shahram Izadi's team. They already had the outline of a pipeline when Kinect launched, and its depth stream supplied the remaining input. The result, presented at ISMAR 2011, was [Newcombe et al. 2011. KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378).

> 🔗 **Borrowed.** KinectFusion's core representation, the TSDF, was devised by Curless & Levoy (1996) for offline 3D scanning. Newcombe's team made it real-time via GPU parallel voxel updates.

The pipeline has four stages.

First, depth preprocessing denoises the raw depth map with a bilateral filter and computes surface normals.

Second, ICP tracking aligns the current frame's point cloud with the virtual surface ray-cast from the previous TSDF. A point-to-plane variant of **ICP (Iterative Closest Point)** from [Besl & McKay (1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf) runs thousands of iterations on the GPU. Its output is the camera's 6-DoF pose.

The point-to-plane ICP objective transforms the current frame's point $\mathbf{p}_i$ by $T = (R, \mathbf{t})$ and matches it to $\hat{\mathbf{p}}_i$ on the ray-cast surface, whose normal is $\hat{\mathbf{n}}_i$. It then minimizes

$$E(R, \mathbf{t}) = \sum_i \bigl(\hat{\mathbf{n}}_i^\top (R\,\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i)\bigr)^2$$

Unlike the original Besl-McKay point-to-point cost ($\|R\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i\|^2$), this objective measures only the error along the normal and is therefore less sensitive to motion along the surface. With the small-rotation approximation $R \approx I + [\boldsymbol{\omega}]_\times$, $E$ becomes a linear least-squares problem in the 6-DoF vector $(\boldsymbol{\omega}, \mathbf{t})$. The GPU solves it through parallel reduction.

> 🔗 **Borrowed.** KinectFusion's tracking stage directly inherits ICP from Besl & McKay (1992), applying a classical robotics technique at GPU-scale density.

Third, TSDF integration projects the depth map into the voxel grid at the estimated pose and updates the TSDF values. The paper's main configuration uses a 512³ voxel grid covering a room-scale volume about 3 m on a side (§4.2, Fig. 13).

Fourth, surface rendering finds the TSDF's zero-crossing by ray marching to produce vertex and normal maps of the surface. The result becomes the reference surface for the next ICP step.

Newcombe presented the monocular dense SLAM system DTAM in the same year. The two projects were closely related: DTAM used the GPU to optimize monocular photometric consistency, while KinectFusion used it for depth integration. Several researchers worked on both.

> 🔗 **Borrowed.** The same researchers presented KinectFusion and DTAM in the same year. They share the use of a GPU for dense processing, but DTAM's photometric optimization and KinectFusion's depth alignment and TSDF fusion also differ in map representation and objective.

The 512³ TSDF updated at 30 Hz, and a single indoor room could be reconstructed as a dense mesh within minutes. Within a fixed room-scale volume, dense model-to-frame ICP used many surface measurements and produced low tracking drift. The model was still built from earlier pose estimates, however, and was not an absolute reference surface.

A 512³ voxel grid covers only a fixed spatial extent; once the camera leaves the room, voxels saturate or overwrite one another. The system had no loop closure, and Kinect's IR structured light did not work in sunlight. Outdoor use was therefore outside its scope from the outset.

---

## 9.3 Kintinuous — rolling volume

Soon after KinectFusion appeared, Whelan at Imperial College addressed this limitation by moving the fixed-size TSDF volume with the camera.

In July 2012, at the RSS workshop (RGB-D: Advanced Reasoning with Depth Cameras, Sydney), [Whelan et al. presented Kintinuous](https://www.cs.cmu.edu/~kaess/pub/Whelan12rssw.pdf), which introduced a "rolling TSDF volume." As the camera approached the boundary of the volume, slices on the far side were extracted as mesh and released, while new slices were attached in front. Memory stayed constant while the camera could move indefinitely.

A demo that traversed an entire indoor corridor exceeded KinectFusion's fixed spatial range, but loop closure was still missing. After a long corridor traversal returned to its origin, the mismatch between the two ends remained unresolved. This misalignment limited global map consistency independently of the level of surface detail.

---

## 9.4 ElasticFusion: Surfels and non-rigid deformation

After Kintinuous, Whelan replaced TSDF voxels with surfels.

A **surfel (surface element)** is a point with a position, normal, radius, and color. In computer graphics, [Pfister et al. (2000)](https://www.merl.com/publications/docs/TR2000-10.pdf) proposed the concept as a rendering representation. Unlike a regular voxel grid, a surfel structure follows the observed surface.

> 🔗 **Borrowed.** ElasticFusion adapted the surfel rendering technique of Pfister et al. (2000) as a SLAM map representation.

[Whelan et al. 2016. ElasticFusion](https://doi.org/10.1177/0278364916669237) combined a surfel-based dense map with loop closure through *non-rigid deformation*.

Loop closure had been difficult in earlier dense SLAM systems because updating a global mesh or voxel grid to satisfy a loop-closure constraint was expensive. ElasticFusion connected the surfel set to a deformation graph. When it detected a loop closure, it deformed the graph to distribute the error across the entire map, producing a non-rigid correction at the surfel-map level.

Concretely, each node $g_k$ of the deformation graph carries a position $\mathbf{v}_k$ and a rotation $R_k$ and translation $\mathbf{t}_k$. A surfel $s$ lies within the influence of its $K$ nearest nodes, and the surfel's deformed position is computed as

$$\tilde{\mathbf{p}}_s = \sum_{k \in \mathcal{N}(s)} w_k \bigl(R_k (\mathbf{p}_s - \mathbf{v}_k) + \mathbf{v}_k + \mathbf{t}_k\bigr)$$

The weight $w_k$ decreases with distance. When a loop-closure constraint is added, Gauss-Newton optimizes the graph nodes' $(R_k, \mathbf{t}_k)$ to distribute the error globally. The method can therefore correct the entire dense map consistently without rebuilding a TSDF from scratch.

The ElasticFusion paper reported strong indoor reconstruction results on the ICL-NUIM synthetic dataset. Sequences kt0·kt1·kt2 recorded ATE RMSE below 1.4 cm, with kt0·kt1 at 0.9 cm; kt3, in which global loop closure activates, was an unusually large exception. These figures apply to the paper's ICL-NUIM setting rather than to KITTI or TUM RGB-D.

---

## 9.5 BundleFusion: offline-SfM quality, online

In 2017, Dai, Nießner, Zollhöfer, Izadi, and Theobalt published [Dai et al. 2017. BundleFusion](https://doi.org/10.1145/3072959.3054739) in ACM Transactions on Graphics. KinectFusion and its successors had sought higher quality while preserving real-time operation. BundleFusion instead devoted extensive GPU computation to running SfM-grade bundle adjustment within an online system.

BundleFusion used hierarchical optimization. At the fastest layer, dense depth alignment between the current and previous frames provides an initial pose. The next layer corrects it through sparse frame-to-frame alignment with SIFT features. At the third layer, hierarchical global bundle adjustment re-optimizes the poses of accumulated frames, including earlier poses. This "retroactive pose correction" sought to approach online the result that an offline SfM pipeline obtains by aligning all data after collection. The system back-projects updated pose sequences into the TSDF for reintegration, preventing tracking errors from remaining embedded in the map.

Dai's team reported better results than ElasticFusion on TUM RGB-D. Its visual reconstruction quality approached that of the offline COLMAP pipeline by the standards of the time.

> 📜 **Prediction vs. outcome.** BundleFusion claimed real-time online global bundle adjustment at "unprecedented speed," proposing a way to bring offline SfM quality into online operation. GPU performance continued to increase, but from 2021 one major research path instead used COLMAP for camera poses and NeRF for scene representation. This was less a direct successor to online TSDF mapping than a turn toward novel-view synthesis and neural scene representation.

---

## 9.6 Co-evolution of hardware and algorithm

The six years from KinectFusion to BundleFusion reflect changes in both algorithms and hardware.

The first-generation Kinect used structured light. Its depth precision was a few millimeters at meter-scale range, but sunlight obscured the IR pattern. Kinect 2, released in 2013, switched to ToF, improving both precision and dynamic range. Intel's RealSense series followed. As sensor options expanded, algorithms could make different assumptions about depth quality, and researchers explored methods that either exploited lower noise or tolerated higher noise.

The CUDA ecosystem also matured. Between KinectFusion in 2011 and BundleFusion in 2017, GPU throughput and memory performance also improved. Any numerical comparison requires specifying the GPU models and arithmetic precision. The increasingly expensive real-time optimization in Whelan's ElasticFusion and Dai's BundleFusion depended on this hardware progress as well as on algorithm design.

Had Kinect been priced like research instrumentation rather than a consumer device, this line of work would probably have spread more slowly. A mass-market sensor helped set its pace.

> 📜 **Prediction vs. outcome.** The spatial range, drift, and outdoor limitations of KinectFusion's fixed 512³ volume shaped the research that followed. Kintinuous, ElasticFusion, and BundleFusion addressed volume extension in turn. Outdoor operation followed a different path because sunlight obscures IR structured-light patterns. Early Kinect-based dense SLAM remained largely indoors, while LiDAR became a main sensor for outdoor dense mapping. This limitation does not apply uniformly to every RGB-D sensing technology.

---

## 9.7 Why dense-only systems receded

Between 2011 and 2017, dense RGB-D SLAM appeared likely to become the main direction of Visual SLAM, but it did not.

Sparse backends continued to dominate. Practical SLAM systems after 2015, represented by [ORB-SLAM2](https://arxiv.org/abs/1610.06475) and [VINS-Mono](https://arxiv.org/abs/1708.03852), did not use dense maps by default. Several constraints reinforced this choice. A 512³ TSDF requires more than 512 MB, a substantial cost for mobile and embedded systems. [Voxblox](https://arxiv.org/abs/1611.03631), which stores TSDF values in hashed blocks, and [OctoMap](https://www.hrl.uni-bonn.de/papers/wurm10octomap.pdf), which stores occupancy probabilities in an octree, reduced memory use with different map representations but did not match sparse representations in efficiency. Real-time dense processing also required a GPU, making a KinectFusion-grade pipeline difficult to run on an autonomous vehicle's embedded processor or a lightweight drone platform. Kinect's IR depth sensing also failed outdoors, while commercially important applications such as autonomous driving and drones operated largely in outdoor environments.

During the same period, dense map data structures extended KinectFusion's fixed 512³ volume in several directions. [Museth's VDB (2013)](https://doi.org/10.1145/2487228.2487235) combined block hashing with an internal tree, leaving sparse regions empty while refining only the neighborhood of a surface. Released as OpenVDB, it became a major infrastructure for large sparse volumetric data and provides a useful comparison with the nvblox lineage in Ch.17. [Reijgwart et al. (2023)'s wavemap](https://arxiv.org/abs/2306.08125) compressed occupancy with a wavelet transform to adjust the trade-off between resolution and memory. Ramos and Ott pursued continuous-function representations. [O'Callaghan and Ramos (2012)'s GPOM (Gaussian Process Occupancy Map)](https://doi.org/10.1177/0278364911435991) used Gaussian Process regression to relate depth measurements and assign probabilities even to unmeasured voxels. [Ramos and Ott (2016)'s Hilbert Map](https://doi.org/10.1177/0278364916684382) learned Hilbert-space features with logistic regression to provide streamable probabilistic occupancy. [Behley and Stachniss (2018)'s SuMa](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/behley2018rss.pdf) adapted ElasticFusion's indoor RGB-D surfel representation to outdoor LiDAR, producing a surfel-based SLAM system that operated on KITTI (→ Ch.17). These approaches extended dense mapping from a single room to outdoor and city-scale environments while incorporating probabilistic uncertainty.

After NeRF appeared around 2020, high-quality dense reconstruction shifted toward NeRF and 3D Gaussian Splatting. The map representation changed, but measured RGB-D depth continued to constrain both tracking and the learned map geometry.

The period of dense RGB-D SLAM was brief, but its components persisted. TSDF representations entered occupancy mapping for autonomous driving, and ICP became a standard tracking method in LiDAR SLAM. Dense-only systems receded while their techniques spread into other architectures.

---

## 🧭 Still open

Large-scale outdoor dense reconstruction. Sunlight interference with IR structured light is a general limitation of active depth sensors. LiDAR operates at longer range but captures color and fine surface detail poorly. As of 2026, RGB-D still cannot densely process large outdoor environments. Learned stereo depth estimation has improved, but limitations in dark regions, on reflective surfaces, and at long range remain unresolved.

Dense reconstruction of dynamic scenes. Every system from KinectFusion to BundleFusion was designed for static scenes. Dense reconstruction in spaces occupied by moving people requires separating dynamic objects, using semantic segmentation, geometric residuals, or other motion-separation methods. [DynaSLAM](https://arxiv.org/abs/1806.05620) and [MaskFusion](https://arxiv.org/abs/1804.09194) attempted this, but their computational cost and robustness remained unsuitable for practical deployment.

Memory efficiency of voxel maps. Voxblox stores TSDF values in hashed blocks, while OctoMap stores occupancy probabilities in an octree; the two reduce memory use for different map representations. Dense representations at the scale of a building floor or city block nevertheless require tens of gigabytes. No general adaptive-resolution method automatically determines the appropriate resolution for each region. Implicit neural representations such as [Instant-NGP](https://arxiv.org/abs/2201.05989) address this problem, but real-time updates still trade off against query speed.

While dense SLAM reconstructed individual rooms as meshes, another lineage addressed the problem of recognizing a return to the same room. Place recognition, which asks whether a system has seen a place before, had developed at Oxford since 2003 independently of dense mapping. KinectFusion had no loop closure; the researchers working on that problem focused on recognizing locations rather than constructing denser maps.
