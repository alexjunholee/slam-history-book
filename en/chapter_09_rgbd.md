# Ch.9 — Dense/RGB-D: From KinectFusion to BundleFusion

In November 2011, when Richard Newcombe (Imperial College London) presented KinectFusion at ISMAR, the audience's attention gathered around the demo video more than the paper. A single handheld Kinect sensor filled an entire room with a 3D mesh in real time. It was the thing Newcombe's own DTAM, released earlier the same year, had been dreaming of with a monocular camera, now actually achieved with an RGB-D sensor. The lineage is clear: the TSDF representation that Curless and Levoy devised for the graphics community in 1996, the ICP tracker that Besl and McKay handed to robotics in 1992, and the Kinect sensor that Microsoft released at $150 in 2010. Where those three strands crossed, the short and intense age of dense SLAM opened. The same framework that Davison's MonoSLAM (Ch.5) used to track sparse landmarks with a monocular camera (real-time tracking, CPU only, no GPU) now reached a different conclusion in front of Kinect's depth stream. Newcombe's DTAM (Ch.8) had opened the door on GPUs by attempting dense reconstruction through direct photometric optimization, and KinectFusion closed the same door with an RGB-D sensor.

---

## 9.1 Dense reconstruction before Kinect

Dense 3D reconstruction was possible before 2011. What was not possible was *real-time*.

Offline pipelines merged point clouds acquired by stereo or structured-light scanners, given time. Indoor scanning rigs cost hundreds of thousands of dollars. No one outside the lab used the technology. The SLAM community was already getting practical results from sparse landmarks, and dense reconstruction was filed away as a graphics problem.

[Curless and Levoy's 1996 SIGGRAPH paper, "A Volumetric Method for Building Complex Models from Range Images"](https://graphics.stanford.edu/papers/volrange/volrange.pdf), represents the graphics-side approach of this era. The central idea was the **TSDF (Truncated Signed Distance Function)**. Partition 3D space into a uniform voxel grid, and at each voxel accumulate the signed distance to the nearest surface. The sign convention: moving from the sensor toward the surface, the front of the surface (free space) is positive, and the back (inside solid) is negative. Truncated means clipping that value within a threshold $t$ in absolute value, giving the form $\text{TSDF}(x) = \text{clip}(d(x), -t, +t)$. Each incoming depth frame updates the value as a weighted average, so that noise averages out and the surface sharpens over time. Surface extraction applies marching cubes to the TSDF's zero-crossing.

The method was accurate. But the voxel grid ate memory, and real-time update was impossible on the hardware of the day. Curless-Levoy's paper stayed in graphics textbooks for the next fifteen years.

In those fifteen years two things changed. GPUs entered the GPGPU era, and Kinect appeared.

---

## 9.2 KinectFusion and TSDF

Microsoft Research released Kinect for the Xbox 360 at $150 in 2010. The sensor measured depth via structured light and streamed VGA-resolution depth maps at 30Hz. Precision was below research-grade ToF (Time-of-Flight) cameras, but the price was one-hundredth. Hackers responded first. Within weeks of launch, open-source drivers appeared, and researchers followed.

Newcombe had by then moved to Microsoft Research Cambridge, where he was preparing a GPU-based dense SLAM with Shahram Izadi's team. By the time Kinect launched they already had the outline of a pipeline. The depth stream Kinect supplied filled in the rest. The result, presented at ISMAR 2011, was [Newcombe et al. 2011. KinectFusion](https://doi.org/10.1109/ISMAR.2011.6092378).

> 🔗 **Borrowed.** KinectFusion's core representation, the TSDF, was devised by Curless & Levoy (1996) for offline 3D scanning. Newcombe's team made it real-time via GPU parallel voxel updates.

The pipeline has four stages.

Depth preprocessing: denoise the raw depth map with a bilateral filter and compute surface normals.

ICP tracking: align the current frame's point cloud to the virtual surface ray-cast from the previous TSDF. A point-to-plane variant of [Besl & McKay (1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)'s **ICP (Iterative Closest Point)** is iterated thousands of times on the GPU. The output is the camera's 6-DoF pose.

The point-to-plane ICP objective is as follows. Transform the current frame's point $\mathbf{p}_i$ by $T = (R, \mathbf{t})$ and take its correspondence $\hat{\mathbf{p}}_i$ (the ray-cast surface) with normal $\hat{\mathbf{n}}_i$; minimize

$$E(R, \mathbf{t}) = \sum_i \bigl(\hat{\mathbf{n}}_i^\top (R\,\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i)\bigr)^2$$

Unlike the original Besl-McKay point-to-point cost ($\|R\mathbf{p}_i + \mathbf{t} - \hat{\mathbf{p}}_i\|^2$), this measures only the error along the normal, so it is less sensitive to sliding along the surface. Applying the small-rotation approximation $R \approx I + [\boldsymbol{\omega}]_\times$, $E$ turns into a linear least-squares problem in the 6-DoF vector $(\boldsymbol{\omega}, \mathbf{t})$, which the GPU solves in one shot via parallel reduction.

> 🔗 **Borrowed.** KinectFusion's tracking stage inherits Besl & McKay (1992) ICP directly. A technique from the classical robotics literature, pulled back out at GPU density.

TSDF integration: project the depth map into the voxel grid at the estimated pose and update the TSDF values. The paper's headline setting is a 512³ voxel grid covering a room-scale volume about 3m on a side (§4.2, Fig. 13).

Surface rendering: find the TSDF's zero-crossing by ray marching and render the mesh in real time. The result becomes the reference surface for the next ICP step.

Newcombe presented DTAM, monocular-camera dense SLAM, the same year. KinectFusion is its sister work. Where DTAM used the GPU to optimize monocular photometric consistency, KinectFusion threw the same GPU at depth integration. Hence the overlap in the two papers' author lists.

> 🔗 **Borrowed.** KinectFusion and DTAM are two dense systems presented the same year by the same researchers. The GPU dense pipeline philosophy of DTAM transferred naturally into KinectFusion; only the sensor differed.

The 512³ TSDF updated at 30Hz, and a single indoor room could be reconstructed as a dense mesh within minutes. Tracking drift was far smaller than feature-based methods, because ICP converges to the absolute surface.

The limits were just as clear. A 512³ voxel grid handles only a fixed spatial extent. Leave the room and voxels saturate or overwrite each other. No loop closure. And Kinect's IR structured light did not work under sunlight. Outdoors was out of scope from the start.

---

## 9.3 Kintinuous — rolling volume

Right after KinectFusion appeared, Whelan at Imperial College attacked this limit. If the fixed-size TSDF volume was the problem, move it with the camera.

In July 2012, at the RSS workshop (RGB-D: Advanced Reasoning with Depth Cameras, Sydney), [Whelan et al. presented Kintinuous](https://www.cs.cmu.edu/~kaess/pub/Whelan12rssw.pdf), which introduced a "rolling TSDF volume." As the camera approached the boundary of the volume, slices on the far side were extracted as mesh and released, while new slices were attached in front. Memory stayed constant while the camera could move indefinitely.

The demo of walking down an entire indoor corridor showed what KinectFusion could not. But loop closure was still missing. When you walked a long corridor back to the origin, the mismatch between the two ends was unresolved. Reconstruction quality was also behind the submap alignment methods that sparse SLAM had accumulated.

---

## 9.4 ElasticFusion: Surfels and non-rigid deformation

Whelan changed direction after Kintinuous. Instead of TSDF voxels, he chose surfels.

A **surfel (surface element)** is a point carrying position, normal, radius, and color. In computer graphics, [Pfister et al. (2000)](https://www.merl.com/publications/docs/TR2000-10.pdf) proposed the concept as a rendering representation. Compared to a voxel grid, the structure is irregular and hugs the surface.

> 🔗 **Borrowed.** ElasticFusion's surfel representation carries Pfister et al.'s (2000) graphics rendering technique over as a SLAM map representation.

[Whelan et al. 2016. ElasticFusion](https://doi.org/10.1177/0278364916669237) has two core contributions. First, a surfel-based dense map. Second, loop closure via *non-rigid deformation*.

Loop closure in prior dense SLAM was hard. Editing a global mesh or voxel grid to match loop-closure information was expensive. ElasticFusion connected the surfel set to a deformation graph and, when a loop closure was detected, deformed the graph to distribute error across the whole map. It was non-rigid deformation at the mesh level.

Concretely, each node $g_k$ of the deformation graph carries a position $\mathbf{v}_k$ and a rotation $R_k$ and translation $\mathbf{t}_k$. A surfel $s$ lies within the influence of its $K$ nearest nodes, and the surfel's deformed position is computed as

$$\tilde{\mathbf{p}}_s = \sum_{k \in \mathcal{N}(s)} w_k \bigl(R_k (\mathbf{p}_s - \mathbf{v}_k) + \mathbf{v}_k + \mathbf{t}_k\bigr)$$

(the weight $w_k$ is a distance-based falloff). When a loop closure constraint is added, the $(R_k, \mathbf{t}_k)$ of the graph nodes are optimized by Gauss-Newton to distribute the error globally. That is why the whole dense map could be corrected consistently without rebuilding the TSDF from scratch.

Measured not on KITTI or TUM RGB-D but on indoor reconstruction quality itself, ElasticFusion was state of the art at the time. On the ICL-NUIM synthetic dataset, sequences kt0·kt1·kt2 recorded ATE RMSE below 1.4cm, with kt0·kt1 at 0.9cm (kt3, where the global loop closure fires, was an exceptional large value). No prior system had reached that level while staying real-time.

---

## 9.5 BundleFusion: offline-SfM quality, online

In 2017, Dai, Nießner, Zollhöfer, Izadi, and Theobalt published [Dai et al. 2017. BundleFusion](https://doi.org/10.1145/3072959.3054739) in ACM Transactions on Graphics, approaching the problem from a different direction. Where the KinectFusion line tried to raise quality without compromising real-time, BundleFusion's goal was to throw as much GPU compute as possible at the problem and run SfM-grade bundle adjustment even inside an online system.

The central idea is hierarchical optimization. At the fastest layer, dense depth alignment between the current and previous frame sets an initial pose. The layer above corrects it with sparse frame-to-frame alignment using SIFT features, and at the third layer a sliding-window global bundle adjustment re-optimizes the poses of the accumulated frames. As frames accumulate, bundle adjustment re-estimates past poses as well. Called "retroactive pose correction," this approach aimed to reach online something close to what an offline SfM pipeline achieves by aligning data after collecting it all. Updated pose sequences are back-projected into the TSDF for re-integration, so tracking errors do not pile up in the map as-is.

The numbers Dai's team reported on TUM RGB-D beat ElasticFusion. Visual reconstruction quality came close to the offline COLMAP pipeline by the standards of the day.

> 📜 **Prediction vs. outcome.** BundleFusion claimed real-time online global bundle adjustment at "unprecedented speed," presenting a route to raise offline SfM quality online. GPU compute kept climbing after that, but attention moved from dense SLAM to NeRF (Neural Radiance Field). For high-quality indoor reconstruction the de facto standard since 2021 became the COLMAP + NeRF pipeline. The route BundleFusion tried to open was routed around through different technology. `[diverted]`

---

## 9.6 Co-evolution of hardware and algorithm

One way to read the six years from KinectFusion to BundleFusion is as algorithmic progress. A more accurate way is as a process in which hardware and algorithms pushed each other.

The first-generation Kinect used structured light. Depth precision was a few millimeters in the meter range, but the IR pattern was not picked up under sunlight. The Kinect 2, released in 2013, switched to ToF. Precision went up and dynamic range improved. Intel's RealSense series followed. As sensor options grew, the depth quality an algorithm could assume changed, and researchers experimented with either exploiting smaller noise or tolerating larger noise.

On the GPU side, the CUDA ecosystem matured. Between the Tesla architecture at the time of KinectFusion in 2011 and the Pascal architecture at the time of BundleFusion in 2017, floating-point throughput grew more than tenfold. That Whelan in ElasticFusion and Dai in BundleFusion could run ever-heavier optimization in real time was not the algorithm's achievement alone.

Had Kinect been $15,000 rather than $150, this flow would have started five years later. A consumer-market sensor set the pace of the research.

> 📜 **Prediction vs. outcome.** The limits KinectFusion exposed in 2011 with its fixed 512³ volume — spatial extent, drift, outdoor unsuitability — became the roadmap of the following research. Volume extension was attacked in turn by Kintinuous, ElasticFusion, and BundleFusion. The outdoor question reached a different conclusion. IR structured light does not pick up a pattern under sunlight. RGB-D-based dense SLAM stayed bound to indoors, and outdoor was taken over by LiDAR. `[diverted]`

---

## 9.7 The exit of dense-only

Between 2011 and 2017, dense RGB-D SLAM looked like it would become the main direction of Visual SLAM. The actual unfolding did not go that way.

Sparse backends kept dominating. The post-2015 practical SLAM systems represented by [ORB-SLAM2](https://arxiv.org/abs/1610.06475) and [VINS-Mono](https://arxiv.org/abs/1708.03852) did not take the dense map as their default. The reasons compounded. A 512³ TSDF requires more than 512MB, hard to afford on mobile platforms or embedded systems. Octree or hash-map variants ([Voxblox](https://arxiv.org/abs/1611.03631), [OctoMap](https://www.hrl.uni-bonn.de/papers/wurm10octomap.pdf)) tried to ease this, but the gap against sparse efficiency remained. Real-time dense processing presupposed a GPU, and running a KinectFusion-grade pipeline on an autonomous vehicle's embedded processor or a drone's lightweight platform was hard. Kinect's IR depth not working outdoors also held things back. Most commercialization-heavy fields like autonomous driving and drones were outdoor environments.

In that same span, the lineage of dense map data structures themselves scattered KinectFusion's 512³ fixed volume in several directions. [Museth's VDB (2013)](https://doi.org/10.1145/2487228.2487235) proposed a structure combining block hashing with an internal tree so that sparse regions are left empty and only the neighborhood of the surface is refined hierarchically; released as OpenVDB, it became the backbone of autonomous-driving dense maps today (connecting to Ch.17 LiDAR's nvblox line). [Reijgwart et al. (2023)'s wavemap](https://arxiv.org/abs/2306.08125) compressed occupancy with a wavelet transform to re-tune the resolution-memory trade-off. Another line, led by Ramos and Ott, moved the representation to a continuous function altogether. [O'Callaghan and Ramos (2012)'s GPOM (Gaussian Process Occupancy Map)](https://doi.org/10.1177/0278364911435991) linked depth measurements through Gaussian Process regression to fill even unmeasured voxels probabilistically, and [Ramos and Ott (2016)'s Hilbert Map](https://doi.org/10.1177/0278364916684382) learned Hilbert-space features with logistic regression to provide streamable probabilistic occupancy. [Behley and Stachniss (2018)'s SuMa](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/behley2018rss.pdf) took the surfel representation ElasticFusion used for indoor RGB-D out to outdoor LiDAR and built a surfel-based SLAM that worked on KITTI (→ Ch.17). Where KinectFusion stopped at a single room, these lines advanced outward: outdoor, city-scale, and into probabilistic uncertainty.

Around 2020, when NeRF appeared, demand for high-quality dense reconstruction shifted to NeRF and 3D Gaussian Splatting. RGB-D SLAM, inside a structure separating localization and mapping, narrowed to using depth as an auxiliary tracking cue.

The dense era was short, but traces remained. The TSDF representation carried into autonomous-driving occupancy maps, and ICP became the standard tracking tool in LiDAR SLAM. The approach retreated, but the parts scattered into other systems.

---

## 🧭 Still open

Large-scale outdoor dense reconstruction. The sunlight weakness of IR structured light is a general problem of active depth sensors. LiDAR handles longer range but is poor on color and fine surface detail. As of 2026, there is still no way to densely process outdoor large-scale environments with RGB-D. Stereo depth estimation is advancing quickly on learned models, and some research is exploring alternatives, but the limits in dark regions, reflective surfaces, and long range are not resolved.

Dense reconstruction of dynamic scenes. Every system from KinectFusion to BundleFusion was designed for static scenes. Densely reconstructing a space where people walk around requires separating dynamic objects, and that requires combining real-time semantic segmentation with dense SLAM. [DynaSLAM](https://arxiv.org/abs/1806.05620) and [MaskFusion](https://arxiv.org/abs/1804.09194) tried, but neither compute cost nor robustness reached practical deployment.

Memory efficiency of the TSDF family. Voxblox's hash structure and OctoMap's octree compression reduced the memory cost of the voxel grid. Yet dense representation at the building-floor or city-block scale still runs to tens of gigabytes. An adaptive-resolution dense map that automatically decides which resolution to keep in which region still has no general solution. Implicit neural representations such as [Instant-NGP](https://arxiv.org/abs/2201.05989) are approaching this problem, but real-time update and query speed still trade off.

While dense SLAM was filling one indoor room with mesh, the problem of returning to that room was being handled in a separate lineage that had been running since before KinectFusion existed. Place recognition — the question of "have we seen this place before?" — had been developing at Oxford since 2003, parallel to and independent of the dense mapping track. KinectFusion had no loop closure. The researchers who had already been working on that question were not building denser maps; they were asking a different one.
