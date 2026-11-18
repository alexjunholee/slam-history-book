# Chapter 20. 3D Foundation Models: From DUSt3R to VGGT

In 2022, Philippe Weinzaepfel and Jerome Revaud at Naver Labs Europe introduced CroCo, a cross-view self-supervised pretraining scheme that learned visual features by comparing pairs of images of the same scene. What began as a feature-learning paper transformed into a calibration-free 3D reconstruction engine a year later when the same team adapted CroCo's architecture in DUSt3R to regress 3D pointmaps directly. From Grenoble the lineage jumped to Oxford's Visual Geometry Group (VGG), and by 2026 it had redrawn how Structure-from-Motion pipelines partition feature matching, calibration, and geometric reconstruction.

---

## 20.1 DUSt3R — learned pointmap

For roughly a decade after 2013, 3D reconstruction followed a fixed recipe: detect keypoints, match descriptors across views, estimate camera intrinsics and extrinsics, triangulate a sparse point cloud, and refine everything with bundle adjustment. [COLMAP (Schönberger & Frahm, 2016)](https://openaccess.thecvf.com/content_cvpr_2016/html/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.html) brought that pipeline to engineering maturity; accuracy climbed, yet the decomposition itself stayed untouched.

[Shuzhe Wang et al. 2023. DUSt3R: Geometric 3D Vision Made Easy](https://arxiv.org/abs/2312.14132) sidestepped the entire sequence. Given just two uncalibrated images—with no focal length, principal point, or lens parameters—DUSt3R directly outputs a dense **pointmap**: a 3D coordinate in a shared space for every pixel in both views.

Architecturally, DUSt3R carries over CroCo's ViT encoder-decoder transformer. Each image is encoded independently, after which twin decoders exchange information via cross-attention: self-attention models spatial context within a frame, while cross-attention learns cross-view correspondences implicitly from scale rather than hand-coded epipolar rules. To train that mapping, the authors assembled 8.5 million image pairs spanning Habitat, MegaDepth, ARKitScenes, Static Scenes 3D, BlendedMVS, ScanNet++, CO3D-v2, and Waymo, drawing ground truth from synthetic renders, depth sensors, and classical SfM reconstructions alike—meaning SfM itself supplied part, though not all, of the supervision for its learned successor.

> 🔗 **Borrowed.** DUSt3R builds on the Vision Transformer ([Dosovitskiy et al. 2020](https://arxiv.org/abs/2010.11929)) via Naver Labs Europe's own CroCo ([Weinzaepfel et al. 2022](https://arxiv.org/abs/2210.10716)). CroCo trained a cross-view transformer to reconstruct masked patches in one view using context from another; DUSt3R preserved that encoder-decoder skeleton and redirected it toward dense pointmap regression.

Because both predicted pointmaps sit in the first camera's coordinate frame, relative pose follows immediately from PnP-RANSAC over the predicted 3D points and their 2D pixel locations in the second view. Extending beyond two views—to three, ten, or dozens of images—requires solving a global alignment optimization that registers all pairwise pointmaps into one consistent world frame. Only in that outer registration step does an explicit bundle-adjustment-like optimization reappear, operating directly on predicted 3D pointmaps without feature matching or explicit camera models.

---

## 20.2 Making matching explicit: MASt3R

While DUSt3R excelled at direct 3D reconstruction, it recovered pixel-to-pixel correspondences only implicitly through aligned pointmaps—leaving it unable to serve as a drop-in replacement for explicit matchers like SuperPoint+SuperGlue or LightGlue.

[Vincent Leroy et al. 2024. Grounding Image Matching in 3D with MASt3R (ECCV)](https://arxiv.org/abs/2406.09756) closed that gap by adding a dedicated local-feature matching head to DUSt3R. Trained jointly with pointmap regression, the head produces per-pixel descriptors anchored in 3D geometry rather than purely 2D image appearance, turning dense correspondence matching into fast nearest-neighbor lookup.

> 🔗 **Borrowed.** MASt3R tackles the descriptor ambiguities that SuperGlue ([Sarlin et al. 2020](https://arxiv.org/abs/1911.11763)) resolved via contextual graph neural networks. Rather than reasoning over 2D keypoint graphs, MASt3R grounds descriptors directly in learned 3D structure.

Within months of MASt3R's release, SLAM groups began swapping out SuperPoint+SuperGlue in favor of its 3D-grounded matcher. In late 2024, [Riku Murai, Eric Dexheimer, Andrew Davison](https://arxiv.org/abs/2412.12392) at Imperial College London unveiled MASt3R-SLAM, pairing MASt3R correspondence matching in the frontend with graph-based global optimization in the backend—preserving classical SLAM's system architecture while replacing nearly all of its internal modules. Because MASt3R matches densely without prior calibration, photogrammetry and SfM pipelines (including COLMAP workflows) were by 2026 actively testing DUSt3R and MASt3R as initialization and matching stages.

> 📜 **Prediction vs. outcome.** Although the DUSt3R paper lacked a formal "Future Work" section, its reliance on pairwise inference plus offline global alignment clearly pointed toward streaming sequence processing and real-time SLAM. Within 6–12 months, Spann3R (August 2024) and MASt3R-SLAM (late 2024) delivered both extensions.

---

## 20.3 Spann3R — sequential processing

Batch reconstruction enjoys a luxury SLAM never has: seeing every image upfront. DUSt3R and MASt3R ingest an entire unordered collection and solve global alignment afterward, whereas a live SLAM system receives frames one by one and must update its map on the fly.

[Hengyi Wang & Lourdes Agapito 2024. 3D Reconstruction with Spatial Memory (Spann3R)](https://arxiv.org/abs/2408.16061) adapted DUSt3R for sequential streams by introducing a spatial-memory bank. Whenever a new frame arrives, cross-attention queries stored representations from previously processed frames, retrieving relevant historical geometry for each incoming pixel.

> 🔗 **Borrowed.** Spann3R adopts a cross-attention memory bank built directly atop DUSt3R's pretrained ViT encoder-decoder. Memory keys combine geometric decoder features with appearance features, indexing prior observations by both visual similarity and 3D proximity.

Like DUSt3R, Spann3R runs without camera calibration while updating its reconstruction incrementally frame by frame. Though not yet fully real-time, it bridged the gap from offline batch alignment to sequential SLAM.

---

## 20.4 VGGT — multi-view joint inference

Spann3R enabled sequential input, yet still inherited DUSt3R's pairwise structure. In early 2025, Jianyuan Wang, Minghao Chen, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, and David Novotny at Oxford's VGG group eliminated pairwise decomposition altogether—taking an arbitrary number of views simultaneously and predicting camera poses, depth maps, and 3D point clouds in a single forward pass.

[Jianyuan Wang et al. 2025. VGGT: Visual Geometry Grounded Transformer](https://arxiv.org/abs/2503.11651) replaced DUSt3R's $N(N-1)/2$ pairwise evaluations and subsequent global alignment with unified multi-view attention across all $N$ images at once.

> 🔗 **Borrowed.** There is a striking historical inversion here: classical SfM reconstructions helped supervise DUSt3R, yet VGGT absorbs into a single transformer pass the very stages classical SfM kept distinct—pairwise geometry estimation, view-graph construction, and global bundle adjustment.

In quantitative benchmarks against DUSt3R, VGGT delivered consistently higher pose accuracy and cleaner point clouds while running far faster without a global-alignment optimization loop—blurring the boundary between pose estimation and dense reconstruction even further.

---

## 20.5 Pose estimation and reconstruction converge

Classical computer vision long treated localization and reconstruction as twin halves of a hard problem: localization finds the camera inside a known map, reconstruction recovers geometry from known poses, and SLAM was hard precisely because it solved both simultaneously.

From DUSt3R to VGGT, shared neural representations increasingly fuse those halves together. DUSt3R still extracted camera poses from pairwise pointmaps and reconciled multiple views via global alignment, whereas VGGT emits poses and geometry in one forward pass—though downstream applications may still refine them differently. Neither model abandons multi-view geometry: epipolar constraints, triangulation, and bundle adjustment are still at work, only encoded implicitly inside transformer attention weights rather than written out as explicit solvers.

That shift carries an unmistakable engineering trade-off. When Schönberger's COLMAP fails, an engineer can inspect feature matches, residuals, or RANSAC thresholds; when DUSt3R misfires, the root cause hides inside millions of attention weights, putting interpretability back among SLAM's open challenges.

> 📜 **Prediction vs. outcome.** The MASt3R paper closed modestly, noting that calibration-free matching opened doors to diverse downstream tasks rather than predicting a wholesale pipeline overhaul. By 2026, commercial and open photogrammetry tools were indeed integrating DUSt3R and MASt3R as initialization modules—pointing toward hybrid insertion rather than complete replacement.

Even the institutional story is remarkable. Within just two years, a small Naver Labs Europe team led by Weinzaepfel, Revaud, and Leroy—tiny compared with Google Brain, DeepMind, or Meta AI—pushed from CroCo (2022) to DUSt3R (2023) to MASt3R (2024), before Davison's group at Imperial College London carried the thread into live SLAM with MASt3R-SLAM.

---

## 20.6 Another branch — semantic foundation enters the map

DUSt3R, MASt3R, and VGGT form the *geometric* branch of 3D foundation models, freeing pointmaps and pose estimation from camera calibration. Over the same period, starting around 2022, a parallel *semantic* branch brought foundation models such as CLIP, DINO, and SAM into the map, freeing scene understanding from fixed object dictionaries.

That semantic lineage traces back to Luca Carlone's group at MIT. [Nathan Hughes et al. 2022. Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization](https://arxiv.org/abs/2201.13360) built an online hierarchy—objects → places → rooms → buildings—atop Kimera's (Rosinol 2020) metric-semantic mesh. Although limited to a closed set of roughly 100–1000 labels, Hydra proved that hierarchical semantic scene graphs could run in real time.

Vision-language foundation models soon dissolved that closed-set ceiling. [Songyou Peng et al. 2023. OpenScene: 3D Scene Understanding with Open Vocabularies (CVPR)](https://arxiv.org/abs/2211.15654) from the ETH/Pollefeys group distilled CLIP features onto 3D point clouds to support arbitrary natural-language queries ("how close is this point to a chair"). Shortly after, a Montréal–MIT team introduced [Qiao Gu et al. 2024. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA)](https://arxiv.org/abs/2309.16650), pairing VLM object descriptions with LLM-inferred inter-object relations—an open-vocabulary object-graph design distinct from systems directly extending Hydra's hierarchy.

Within the Carlone group itself, [Dominic Maggio et al. 2024. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs](https://arxiv.org/abs/2404.13696) carried Hydra's hierarchy into task-conditioned abstraction. Using an information-bottleneck formulation, Clio keeps only the scene details relevant to a natural-language instruction: asked to "clean near the coffee machine," it preserves fine objects around the counter while coarsening unrelated rooms.

> 🔗 **Borrowed.** Clio directly extends Hydra within the Carlone group by introducing task-driven abstraction. ConceptGraphs is an independent open-vocabulary object-graph lineage and should not be read as inheriting Hydra's objects-places-rooms hierarchy.

Even though object-as-landmark SLAM faded in 2017–2019, semantics returned vigorously at the scene-graph layer. Yet as of 2026, the semantic branch and DUSt3R's geometric branch remained largely separate: no unified end-to-end system attached CLIP embeddings directly onto VGGT's pointmaps or paired Clio's task-driven graphs with DUSt3R's uncalibrated geometry—a convergence left open for future work (→ Chapter 22 §22.4).

---

## 20.7 What classical SLAM still supplies

Look closely at MASt3R-SLAM and a striking architectural continuity emerges. Even after swapping feature extraction and matching for foundation models, it still leans on classical SLAM's scaffolding—keyframe selection, loop closure, and map management.

The same pattern holds across every recent wave: NeRF-SLAM kept keyframe-based tracking beneath an implicit field; 3DGS-SLAM adopted explicit Gaussians yet closed loops classically (→ Chapter 18); dynamic SLAM filtered or modeled moving objects in the frontend while keeping the backend intact (→ Chapter 19). Across a decade of upheaval, map representations turned over far faster than overall system architecture.

Foundation 3D proved no exception. In 2025, Dominic Maggio, Hyungtae Lim, and Luca Carlone at MIT released [VGGT-SLAM](https://arxiv.org/abs/2505.12549), using VGGT to reconstruct local submaps while optimizing 15-degree-of-freedom projective transforms between sequential submaps on \(\mathrm{SL}(4)\)—complete with loop-closure constraints. The transformer handles local geometry, while global graph optimization still holds the trajectory together. As Revaud observed in Handbook Ch.13, "a form of factor graph is still necessary." Achieving real-time performance over long sequences and merging that geometry with the semantic branch of §20.6 remained open challenges in 2026–2027.

---

## Still open

**Large-scale sequence processing.** Attention across all image tokens gives DUSt3R and VGGT memory complexity that grows quadratically with sequence length—tractable up to roughly 100 images, prohibitive at 1,000 or 10,000. Spann3R's streaming memory offers a partial bridge, yet sparse attention and hierarchical alignment have not yielded a consensus recipe for city-scale outdoor runs.

**Loop closure for pointmaps.** Classical SLAM detects revisited places and propagates metric corrections across a pose-landmark graph. How best to represent places and warp dense pointmaps consistently upon loop closure remains unsettled; MASt3R-SLAM relies on classical graph mechanisms, without proof that this is the principled long-term answer.

**Metric-scale generalization.** Like Metric3D or Depth Anything v2, DUSt3R recovers relative depth ratios between views rather than absolute metric scale. Without an IMU, GPS, or known physical baseline, scaling to unconstrained footage runs straight into the same projective physics that bounded monocular SLAM from the beginning.

**SLAM lineage or separate branch?** MASt3R-SLAM and VGGT-SLAM pulled 3D foundation models into SLAM pipelines across 2024–2025, yet real-time long-horizon execution and fusion with semantic scene graphs (Hydra → Clio alongside ConceptGraphs in §20.6) remain unresolved—no single architecture yet unites geometric and semantic foundation models.

---

Across Chapters 16 through 20, neural networks, radiance fields, Gaussians, and foundation transformers rebuilt the internal machinery of visual reconstruction while leaving keyframe management, loop closure, and graph backends standing overhead. Over those very same decades, an entirely separate community was wrestling with many of the same engineering problems—not through camera lenses, but through spinning beams of laser light.
