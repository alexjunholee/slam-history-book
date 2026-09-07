# Ch.16 — Foundation 3D: From DUSt3R to VGGT

Philippe Weinzaepfel and Jerome Revaud of Naver Labs Europe released CroCo in 2022, proposing a cross-view self-supervised pretraining scheme that learned visual representations from two images of the same scene. The paper concerned feature learning. A year later, the same team used CroCo's architecture in DUSt3R to output pointmaps directly without calibration, turning the model into a reconstruction system. The work later reached the VGG group at Oxford, and by 2026 it had changed how SfM pipelines divided matching, calibration, and reconstruction.

---

## 16.1 DUSt3R — learned pointmap

For roughly ten years after 2013, 3D reconstruction followed the same procedure: find feature points, match them, estimate intrinsic and extrinsic camera parameters, build a point cloud through triangulation, and refine it with bundle adjustment. [COLMAP (Schönberger & Frahm, 2016)](https://openaccess.thecvf.com/content_cvpr_2016/html/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.html) was the most complete form of this pipeline. Errors dropped, but the structure of the procedure did not change.

[Shuzhe Wang et al. 2023. DUSt3R: Geometric 3D Vision Made Easy](https://arxiv.org/abs/2312.14132) bypasses this procedure. It takes two images as input and outputs 3D coordinates for each pixel directly. It does not require intrinsic parameters such as focal length or principal point. The output, called a pointmap, gives coordinates in a common 3D space rather than in image coordinates, without prior knowledge of the camera lens.

DUSt3R's transformer uses the encoder-decoder structure inherited from CroCo. Each image is encoded independently, then the decoder reads the other image's encoder output through cross-attention. Self-attention handles relations among pixels within one image; cross-attention learns correspondences between images implicitly from large-scale data instead of applying a coded matching rule. The paper assembled 8.5 million pairs from Habitat, MegaDepth, ARKitScenes, Static Scenes 3D, BlendedMVS, ScanNet++, CO3D-v2, and Waymo. Their annotations came from three sources: synthetic generation, reconstructions by SfM software, and dedicated sensors. Classical SfM therefore supplied part, not all, of the supervision for its learned successor.

> 🔗 **Borrowed.** DUSt3R's backbone comes from ViT ([Dosovitskiy et al. 2020](https://arxiv.org/abs/2010.11929)), and its direct precursor is CroCo ([Weinzaepfel et al. 2022](https://arxiv.org/abs/2210.10716), also from Naver Labs Europe). CroCo proposed cross-view self-supervised pretraining in which information from one image reconstructs masked regions in the other. DUSt3R retained CroCo's encoder-decoder structure and changed the task to pointmap prediction.

Both pointmaps already use the first camera's coordinate frame. Relative camera pose can be recovered with PnP-RANSAC from predicted 3D points and their pixel correspondences in the second image. For multiple image pairs, a separate global alignment adjusts the pointmaps and cameras. Pose estimation is derived from the pointmaps.

When extending to three or ten images, DUSt3R solves a global alignment. It is an optimization problem that registers the pointmaps of all image pairs into one common coordinate frame. Only at this stage does something resembling bundle adjustment appear, but it proceeds without feature matching or camera models.

---

## 16.2 Making matching explicit: MASt3R

DUSt3R's output is closer to reconstruction than to novel view synthesis. Yet it handles an important reconstruction subtask, finding precise pixel correspondences between two images, only implicitly. Replacing the explicit matching performed by SuperPoint+SuperGlue or LightGlue needed additional machinery.

[Vincent Leroy et al. 2024. Grounding Image Matching in 3D with MASt3R (ECCV)](https://arxiv.org/abs/2406.09756) adds a matching head to DUSt3R. It is trained to output a feature descriptor for each pixel along with the pointmap, with joint learning that keeps the 3D position and the feature consistent. The resulting features are anchored in 3D space rather than in the image plane. Matching simplifies into nearest-neighbor search over these feature descriptors.

> 🔗 **Borrowed.** MASt3R addresses the ambiguity in 2D descriptors that SuperGlue ([Sarlin et al. 2020](https://arxiv.org/abs/1911.11763)) handled with contextual reasoning. SuperGlue used a graph neural network to reduce ambiguity in 2D matching; MASt3R instead learns 3D structure directly.

Within months of MASt3R's release, multiple groups in the SLAM community reported experiments that replaced the SuperPoint+SuperGlue combination with MASt3R. In late 2024, [Riku Murai, Eric Dexheimer, Andrew Davison](https://arxiv.org/abs/2412.12392) at Imperial College London released MASt3R-SLAM, using MASt3R's matching as the frontend and graph-based global optimization as the backend. The system retained the classical SLAM architecture while replacing most of its internal components.

MASt3R's strength is that dense matching is possible without ground-truth calibration. As of 2026, researchers are testing DUSt3R or MASt3R in the initialization and matching stages of COLMAP-based SfM pipelines.

> 📜 **Prediction vs. outcome.** The DUSt3R paper itself did not include a dedicated "Future Work" section, but the structure of pairwise processing plus global alignment implies sequence processing and real-time operation as the next tasks. Spann3R arrived in August 2024 and MASt3R-SLAM at the end of 2024. The two follow-up works addressed sequential extension and SLAM integration within 6–12 months.

---

## 16.3 Spann3R — sequential processing

Batch processing has a practical constraint: in SLAM, the images are not all available in advance.

DUSt3R and MASt3R take a complete image set as input and register it in a batch. SLAM receives images in temporal order and must update the map at each frame.

[Hengyi Wang & Lourdes Agapito 2024. 3D Reconstruction with Spatial Memory (Spann3R)](https://arxiv.org/abs/2408.16061) reshapes DUSt3R's structure for sequential processing. It stores information from processed frames in a spatial-memory bank and applies cross-attention to that memory when a new frame arrives. Attention selects the past-frame information associated with each pixel of the new image.

> 🔗 **Borrowed.** Spann3R's spatial memory resembles other cross-attention memory mechanisms. It retains DUSt3R's pretrained ViT encoder-decoder and constructs memory keys from decoder outputs (geometric features) and image features, so lookup reflects both appearance and distance. DUSt3R's geometric representation becomes the index for sequential memory.

Spann3R retains DUSt3R's ability to work without a calibrated camera and updates the map incrementally as each image arrives. It is not fully real-time, but it moves the method from batch reconstruction toward SLAM.

---

## 16.4 VGGT — multi-view joint inference

Spann3R enabled sequential processing but retained DUSt3R's pairwise pointmaps and global alignment. At the start of 2025, Jianyuan Wang, Minghao Chen, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, and David Novotny at Oxford's VGG group took an arbitrary number of images as simultaneous input and produced camera pose, depth, and a point cloud in one forward pass.

[Jianyuan Wang et al. 2025. VGGT: Visual Geometry Grounded Transformer](https://arxiv.org/abs/2503.11651) turned DUSt3R's pairwise processing into multi-view joint inference. Exhaustively pairing N images for DUSt3R produces N(N-1)/2 pointmap pairs followed by global alignment. VGGT passes all N images through the transformer at once, and attention processes the relations among every image pair simultaneously.

> 🔗 **Borrowed.** There is an inversion in the fact that SfM reconstructions supplied part of DUSt3R's training supervision. VGGT then performs, within one model, functions that classical SfM separated into pairwise geometry estimation, graph construction, and global optimization. The stages of the classical pipeline have been absorbed in a different form.

In quantitative comparisons with DUSt3R, VGGT showed consistently better camera-pose accuracy and point-cloud quality. Processing was also faster because it required no global-alignment optimization. The change made the boundary between pose estimation and reconstruction less distinct.

---

## 16.5 Pose estimation and reconstruction converge

Traditional computer vision distinguished the two problems. Map-based localization finds the current position in an already-known map, and 3D reconstruction recovers the geometry of an unknown environment. SLAM was hard because it solved both at the same time.

Systems from DUSt3R to VGGT use shared learned representations for geometry and camera estimation. DUSt3R recovers pose from pointmaps through separate operations and joins multiple views with global alignment. VGGT outputs cameras and geometry in one forward pass. Their common direction does not imply identical postprocessing requirements.

DUSt3R, MASt3R, and VGGT have not discarded multi-view geometry. Their transformer weights encode principles implemented explicitly by the epipolar constraint, triangulation, and bundle adjustment. The change lies in how those principles are implemented: implicitly in model weights rather than as separate algorithms.

This implementation is harder to debug than Schönberger's COLMAP code. The causes of a DUSt3R failure are buried inside attention weights, returning interpretability to the list of unresolved problems.

> 📜 **Prediction vs. outcome.** The MASt3R paper closed briefly, suggesting that matching without ground-truth calibration was open to several downstream tasks. It was not an explicit prediction of pipeline reshaping. As of 2026 several photogrammetry software packages are evaluating DUSt3R/MASt3R as an initialization stage, and the pattern looks more like hybrid insertion than full replacement.

Within two years, one team at Naver Labs Europe released CroCo (2022), DUSt3R (2023), and MASt3R (2024), covering the path from pretraining method to matching system. The group centered on Weinzaepfel, Revaud, and Leroy was small compared with Google Brain, DeepMind, or Meta AI. Davison's group at Imperial College London then carried the work into SLAM with MASt3R-SLAM.

---

## 16.6 Another branch — semantic foundation enters the map

DUSt3R, MASt3R, and VGGT form the geometric branch of foundation 3D: they deal with pointmaps, camera poses, and geometric structure. Around 2022, the phrase also came to include a semantic branch that brought CLIP, DINO, and SAM into the map. The geometric branch reduced dependence on calibration; the semantic branch reduced dependence on a fixed label dictionary.

The semantic branch began in Luca Carlone's group at MIT. [Nathan Hughes et al. 2022. Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization](https://arxiv.org/abs/2201.13360) placed an online hierarchy of objects → places → rooms → buildings on top of Kimera's (Rosinol 2020) metric-semantic mesh. Its closed-set classifier remained limited to a predefined dictionary of roughly 100–1000 labels, but it showed that a hierarchical map could run in real time.

Foundation models removed the fixed-dictionary constraint. [Songyou Peng et al. 2023. OpenScene: 3D Scene Understanding with Open Vocabularies (CVPR)](https://arxiv.org/abs/2211.15654) came from the ETH/Pollefeys group, followed by [Qiao Gu et al. 2024. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA)](https://arxiv.org/abs/2309.16650) from a Montréal-MIT collaboration. OpenScene distilled CLIP features onto 3D point clouds, allowing natural-language queries such as "how close is this point to a chair." In ConceptGraphs, a VLM generated language descriptions as node attributes and an LLM described relations between objects. These methods connected concepts outside a predefined class dictionary to 3D representations. ConceptGraphs is distinct from methods that directly extend Hydra's hierarchy.

[Dominic Maggio et al. 2024. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs](https://arxiv.org/abs/2404.13696) turned this lineage toward tasks. Clio treats a natural-language task as an information bottleneck and retains only the level of abstraction that task needs in the scene graph. For an instruction such as "clean near the coffee machine," it preserves the coffee machine and surrounding objects while grouping unrelated details. The exposed layer of the hierarchy varies by task.

> 🔗 **Borrowed.** Clio is a direct successor to Hydra in the Carlone group, adding task-driven abstraction. ConceptGraphs is a separate open-vocabulary object-graph approach and should not be described as inheriting Hydra's objects-places-rooms hierarchy.

Ch.18 §18.4 traces the contraction of the object-as-landmark lineage in 2017–2019. Semantic SLAM later returned in hierarchical scene graphs, but that semantic branch and the geometric DUSt3R branch had not yet converged as of 2026. No reported end-to-end system attached CLIP features to VGGT's pointmap or combined Clio's scene graph with DUSt3R's calibration-free geometry. Their possible point of contact remains an open problem.

---

## 16.7 What classical SLAM still supplies

MASt3R-SLAM borrows the architecture of classical SLAM. Keyframe selection, loop closure, and map management remain necessary on top of the new representation. The DUSt3R family replaced the internal machinery of feature matching and reconstruction but retained these system-level decisions in their classical form.

The same pattern appears in the other recent lineages. NeRF-SLAM adopted NeRF as a map representation yet kept keyframe-based tracking. 3DGS-SLAM adopted Gaussians but performed loop closure in the classical way (Ch.15). Dynamic SLAM in Ch.15b changed the frontend for mask removal while retaining the backend. The representation changed more often than the system structure.

Foundation 3D followed this pattern as well. In 2025, Dominic Maggio, Hyungtae Lim, and Luca Carlone at MIT released [VGGT-SLAM](https://arxiv.org/abs/2505.12549). VGGT reconstructs local submaps; the system then optimizes 15-degree-of-freedom projective transforms between sequential submaps on \(\mathrm{SL}(4)\), including loop-closure constraints. The transformer supplies local geometry, but global graph optimization remains. Revaud wrote in Handbook Ch.13 that "a form of factor graph is still necessary." Real-time large-scale sequences and integration with the semantic branch in §16.6 remained unresolved in 2026–2027.

---

## 🧭 Still open

**Large-scale sequence processing.** The transformers in DUSt3R and VGGT require memory that scales quadratically with the number of images. Up to 100 images is realistic, but 1,000 or 10,000 is another matter. Spann3R's incremental approach is a partial answer, but smooth handling of large outdoor environments is unresolved. Sparse-attention and hierarchical-global-alignment approaches have not yet produced a consensus method.

**Loop closure for pointmaps.** In classical SLAM, loop closure recognizes a previously visited place and corrects accumulated error. The DUSt3R family must represent that place and propagate the correction through a pointmap-based map. MASt3R-SLAM uses the existing approach, but whether it is the best or a principled solution is unknown.

**Metric-scale generalization.** DUSt3R's pointmaps have relative scale. The depth ratio between two images is recovered, but absolute scale is unknown. As with Metric3D or Depth Anything v2, metric scale remains a problem for foundation 3D. The physical constraint of determining absolute scale without GPS or an IMU remains regardless of data scale.

**SLAM lineage or separate branch?** MASt3R-SLAM and VGGT-SLAM brought foundation 3D into SLAM systems in 2024–2025. Real-time operation on large sequences and integration with the semantic branch in §16.6 (Hydra → Clio and the separate ConceptGraphs branch) remain unclear. No common architecture yet combines geometric and semantic foundation models in one system.

---

NeRF and foundation models changed the internal machinery of reconstruction and localization, while keyframes, loop closure, and map management remained above the new representations. LiDAR-based SLAM developed in parallel over the same period. Its sensors and research culture differed, but it faced many of the same engineering problems with a rotating laser rather than a lens.
