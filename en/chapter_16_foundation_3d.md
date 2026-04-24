# Ch.16 — Foundation 3D: From DUSt3R to VGGT

Philippe Weinzaepfel and Jerome Revaud of Naver Labs Europe released CroCo in 2022, proposing a cross-view self-supervised pretraining scheme that learned visual representations using the fact that two images captured the same scene as its only cue. It looked like a feature-learning paper. A year later, when the same team built a system on top of CroCo's architecture that output pointmaps directly without calibration, DUSt3R moved past feature learning and redefined multi-view geometry. The lineage that began at Naver Labs Europe carried over to the VGG group at Oxford, and as of 2026 it is rewriting the question of what SfM is.

---

## 16.1 DUSt3R — learned pointmap

For the ten years starting in 2013, 3D reconstruction followed the same procedure. Find feature points, match them, estimate intrinsic and extrinsic camera parameters, build a point cloud through triangulation, and refine the whole thing with bundle adjustment. [COLMAP (Schönberger & Frahm, 2016)](https://openaccess.thecvf.com/content_cvpr_2016/html/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.html) was the most complete form of this pipeline. Errors dropped but the structure of the procedure did not change.

[Shuzhe Wang et al. 2023. DUSt3R: Geometric 3D Vision Made Easy](https://arxiv.org/abs/2312.14132) routes around this procedure. It takes two images as input and outputs 3D coordinates for each pixel directly. It does not require intrinsic parameters (focal length, principal point). The output, called a pointmap, gives coordinates in a common 3D space rather than in image coordinates. You do not need to know what lens the camera is using.

DUSt3R's transformer uses the encoder-decoder structure inherited from CroCo. Each image is encoded independently, then the decoder refers to the other image's encoder output through cross-attention. If self-attention handles relations between pixels within a single image, cross-attention implicitly learns the correspondence between two images. Which pixel sees the same 3D point as which — rather than coding this as a rule, the model absorbs it as a pattern from large-scale data. DUSt3R's training data consists of millions of image pairs from MegaDepth, ScanNet, ARKitScenes, BlendedMVS, and others. COLMAP produced the ground truth. The inversion where classical SfM supplies the ground truth for the learning era happens here.

> 🔗 **Borrowed.** DUSt3R's backbone comes from ViT ([Dosovitskiy et al. 2020](https://arxiv.org/abs/2010.11929)). But the decisive foothold is CroCo ([Weinzaepfel et al. 2022](https://arxiv.org/abs/2210.10716)), prior work from within Naver Labs Europe. CroCo proposed a cross-view self-supervised pretraining where masked regions in one image are reconstructed from information in the other. DUSt3R inherited CroCo's encoder-decoder structure as is and only swapped the task to "pointmap prediction."

Once a pair of pointmaps is obtained from two images, camera pose is recovered by rigid alignment between these pointmaps. It is a generalization of Procrustes alignment. Pose estimation is no longer a separate step but a derivative of the pointmap.

When extending to three or ten images, DUSt3R solves a global alignment. It is an optimization problem that registers the pointmaps of all image pairs into one common coordinate frame. Only at this stage does something resembling bundle adjustment appear, but it proceeds without feature matching or camera models.

---

## 16.2 Swallowing matching: MASt3R

DUSt3R's output is closer to reconstruction than to novel view synthesis. Yet an important sub-task in reconstruction, finding precise pixel correspondences between two images (feature matching), DUSt3R handles only implicitly. To replace the explicit matching performed by SuperPoint+SuperGlue or LightGlue, additional machinery was needed.

[Vincent Leroy et al. 2024. Grounding Image Matching in 3D with MASt3R (ECCV)](https://arxiv.org/abs/2406.09756) adds a matching head to DUSt3R. It is trained to output a feature descriptor for each pixel along with the pointmap, with joint learning that keeps the 3D position and the feature consistent. The resulting features are anchored in 3D space rather than in the image plane. Matching simplifies into nearest-neighbor search over these feature descriptors.

> 🔗 **Borrowed.** MASt3R's 3D-anchored matching attacks the problem SuperGlue ([Sarlin et al. 2020](https://arxiv.org/abs/1911.11763)) was trying to solve (resolving ambiguity in 2D descriptors through context) from a different direction. SuperGlue reduced ambiguity in 2D matching with a graph neural network. MASt3R learns 3D structure directly, eliminating the source of the ambiguity itself.

Within months of MASt3R's release, multiple groups in the SLAM community reported experiments that replaced the SuperPoint+SuperGlue combination with MASt3R. In late 2024, [Riku Murai, Eric Dexheimer, Andrew Davison](https://arxiv.org/abs/2412.12392) at Imperial College London released MASt3R-SLAM, using MASt3R's matching as the frontend and graph-based global optimization as the backend. The classical SLAM architecture kept its shape while nearly all of the internal parts were swapped out.

MASt3R's strength is that dense matching is possible without ground-truth calibration. As of 2026, inserting DUSt3R or MASt3R into a COLMAP-based SfM pipeline is becoming standard in experimental setups.

> 📜 **Prediction vs. outcome.** The DUSt3R paper itself did not include a dedicated "Future Work" section, but the structure of pair-wise + global alignment implies sequence processing and real-time operation as the next tasks. Spann3R arrived in August 2024, MASt3R-SLAM at the end of 2024. The two follow-up works responded to sequential extension and SLAM integration within 6–12 months. `[in progress]`

---

## 16.3 Spann3R — sequential processing

But batch processing has a fundamental constraint. In SLAM the images are not all available in advance.

DUSt3R and MASt3R take a set of images as input and process them in batch. The approach is to lay out all the images in a bag at once and register them. SLAM is different. Images arrive in temporal order, and the system must update the map at each frame.

[Hengyi Wang & Lourdes Agapito 2024. 3D Reconstruction with Spatial Memory (Spann3R)](https://arxiv.org/abs/2408.16061) reshapes DUSt3R's structure for sequential processing. The core idea is spatial memory. Information from already-processed frames is stored in a memory bank, and when a new frame comes in the system performs cross-attention against this memory. Attention decides which information from past frames each pixel of the new image is related to.

> 🔗 **Borrowed.** Spann3R's spatial memory mechanism is similar in concept to cross-attention memory. Structurally, it inherits DUSt3R's pretrained ViT encoder-decoder as is, and sets up memory keys that combine the decoder output (geometric features) with image features so that memory lookup reflects appearance and distance at once. It is a route where the geometric representation DUSt3R captured is reused as the index of the sequential memory.

Spann3R carries over DUSt3R's property of working without a calibrated camera. As each sequential image comes in, the map built so far is updated incrementally. It is not fully real-time, but it is one step closer to SLAM application than DUSt3R's batch approach.

---

## 16.4 VGGT — multi-view joint inference

Spann3R made sequential processing possible. But DUSt3R's basic skeleton of pair-wise pointmap + global alignment remained. The VGG group at Oxford enters. Jianyuan Wang, Minghao Chen, Nikita Karaev, Andrea Vedaldi, Christian Rupprecht, and David Novotny pushed DUSt3R's logic all the way at the start of 2025. Rather than two images, they take an arbitrary number of multiple images as simultaneous input and output camera pose, depth, and point cloud in a single forward pass.

[Jianyuan Wang et al. 2025. VGGT: Visual Geometry Grounded Transformer](https://arxiv.org/abs/2503.11651) turned DUSt3R's pair-wise processing into genuine multi-view joint inference. To process N images with DUSt3R you compute N(N-1)/2 pairs of pointmaps and then solve global alignment. VGGT pushes N images through the transformer at once. Attention processes the relations among all image pairs simultaneously.

> 🔗 **Borrowed.** In the context where VGGT redefines COLMAP's role, there is an older lineage. The irony that COLMAP itself produces the ground truth of the learning era was noted earlier. And each stage of what COLMAP actually performs (pair-wise geometry estimation → graph construction → global optimization) is implicitly reproduced inside VGGT. The form is one where what classical SfM implemented as an algorithm is absorbed by a foundation model as weights.

In quantitative comparisons with DUSt3R, VGGT showed consistent advantage in camera pose estimation accuracy and point cloud quality. Processing speed is also faster because there is no global alignment optimization. And this is where a conceptual problem arises.

---

## 16.5 The boundary between pose estimation and reconstruction dissolves

Traditional computer vision distinguished the two problems. Camera pose estimation finds the current position in an already-known map, and 3D reconstruction recovers the geometry of an unknown environment. SLAM was hard because it solved both at the same time.

The systems from DUSt3R to VGGT are indifferent to this distinction. Predict a pointmap and pose comes out; get a pose and reconstruction comes out. The ordering itself of "first get the camera, then the point cloud" or "first get the point cloud, then the camera" is gone. A single forward pass outputs everything at once.

The multi-view geometry learned so far is not discarded. DUSt3R, MASt3R, and VGGT work because they have learned, inside the transformer weights, the geometric principles that epipolar constraint, triangulation, and bundle adjustment implement. What has been discarded is not those principles but the way of implementing them as explicit algorithms. Geometry has not vanished; it has been absorbed implicitly.

For the researcher, however, this is a real shift. You cannot debug DUSt3R the way you debugged Schönberger's COLMAP code. Where it failed and why is buried inside attention weights. The interpretability problem returns in a new form.

> 📜 **Prediction vs. outcome.** The MASt3R paper closed briefly, suggesting that matching without ground-truth calibration was open to several downstream tasks. It was not an explicit prediction of pipeline reshaping. As of 2026 several photogrammetry software packages are evaluating DUSt3R/MASt3R as an initialization stage, and the pattern looks more like hybrid insertion than full replacement. `[in progress]`

One research institute, Naver Labs Europe, walked through CroCo (2022) → DUSt3R (2023) → MASt3R (2024) within two years, a single team releasing in succession the stack from pretraining methodology to matching system. Not a large place like Google Brain, DeepMind, or Meta AI, but Naver Labs Europe, carried by a small team centered on Weinzaepfel, Revaud, and Leroy. The work of moving to the SLAM stage was taken up by the Davison group at Imperial College London (MASt3R-SLAM).

---

## 16.6 Another branch — semantic foundation enters the map

The narrative so far has been geometric foundation. DUSt3R, MASt3R, and VGGT deal with pointmaps, camera poses, and geometric structure. But around 2022 the phrase "foundation 3D" began to be used in the SLAM literature along two branches. One is the geometric lineage that started at Naver Labs Europe. The other is the semantic lineage that pulls CLIP, DINO, and SAM into the map. The former removed calibration; the latter removed the dictionary.

The semantic branch began in Luca Carlone's group at MIT. [Nathan Hughes et al. 2022. Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization](https://arxiv.org/abs/2201.13360) placed a hierarchical scene graph of objects → places → rooms → buildings online on top of Kimera's (Rosinol 2020) metric-semantic mesh. It sat inside the constraint that, as long as you use a closed-set classifier, a handbook would nail down as "100–1000 labels predefined dictionary," but it showed for the first time that a hierarchical map could run in real time.

The wall of the dictionary was torn down by foundation models. [Songyou Peng et al. 2023. OpenScene: 3D Scene Understanding with Open Vocabularies (CVPR)](https://arxiv.org/abs/2211.15654) came out of the ETH/Pollefeys group, and shortly after [Qiao Gu et al. 2024. ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA)](https://arxiv.org/abs/2309.16650) from a Montréal-MIT collaboration. OpenScene distilled CLIP features onto 3D point clouds, allowing queries like "how close is this point to a chair" to be answered by natural language. ConceptGraphs went a step further. Instead of class labels, a VLM generated language descriptions as node attributes, and an LLM narrated relations between objects. As Peng's open-vocabulary features combined with Hydra's hierarchical structure, scene graphs came to accept even concepts not in the dictionary.

[Dominic Maggio et al. 2024. Clio: Real-time Task-Driven Open-Set 3D Scene Graphs](https://arxiv.org/abs/2404.13696) turned this lineage toward tasks. A natural-language task given to the robot is interpreted as an information bottleneck, and only the level of abstraction needed for that task is left in the scene graph. In an instruction like "clean near the coffee machine," the coffee machine and the objects around it are preserved while unrelated details are grouped. Which layer of the hierarchical graph to expose varies by task.

> 🔗 **Borrowed.** ConceptGraphs and Clio inherit the Carlone-group scene graph (Armeni → Rosinol-Kimera → Hughes-Hydra → Maggio-Clio) accumulated over eight years, swapping node features for CLIP, VLM, and LLM outputs while the objects-places-rooms hierarchy survives.

This book does not cover this branch. The problem of loading semantic onto the map is a separate trajectory that follows Ch.18 §18.4's note on the market shrinkage of the object-as-landmark lineage in 2017–2019, and the full narration ends here. As a historical record, two points are left. One is that semantic SLAM did not "fail" but returned in the form of hierarchical scene graphs. The other is that geometric foundation (the DUSt3R lineage) and semantic foundation (the Hydra → ConceptGraphs → Clio lineage) have not yet met in earnest as of 2026. An end-to-end system that attaches CLIP features to VGGT's pointmap, or a configuration that combines Clio's scene graph with DUSt3R's calibration-free geometry, has not yet been reported. The point of encounter is handed over to Ch.19's open problems.

---

## 16.7 What remains of SLAM

MASt3R-SLAM borrows the architecture of classical SLAM. Keyframe selection, loop closure, map management — these structures were still needed on top of the new representation. The DUSt3R family replaced the insides of feature matching and reconstruction, but the system-level judgments of SLAM are reused in the form classical methods resolved them.

This observation is consistent with a pattern that repeats across Part 5. NeRF-SLAM adopted NeRF as a map representation yet kept keyframe-based tracking. 3DGS-SLAM adopted Gaussians yet did loop closure in the classical way (Ch.15). Dynamic SLAM in Ch.15b also only swapped the front end for mask removal while the back end stayed the same. The representation changes; the system structure survives.

The same pattern repeats for foundation 3D. In 2025, Dominic Maggio and Luca Carlone at MIT released [VGGT-SLAM](https://arxiv.org/abs/2505.12549). The configuration has VGGT reconstruct local submaps and a factor graph weave them into the global coordinate frame. The transformer absorbed geometry, but the factor graph survived. Revaud himself wrote in Handbook Ch.13 that "a form of factor graph is still necessary." The speed of absorption is unusual but the final form is still open. How far foundation 3D can handle real-time large-scale sequences, and where it will merge with the semantic branch noted in 16.6, are the things to watch in 2026–2027.

---

## 🧭 Still open

**The wall of large-scale sequence processing.** The transformers in DUSt3R and VGGT require memory that scales quadratically with the number of images. Up to 100 images is realistic, but 1,000 or 10,000 is another matter. Spann3R's incremental approach is a partial answer, but smooth handling of large outdoor environments is unresolved. Who is working on it now? Several groups are exploring sparse attention and hierarchical global alignment, but no method has consensus.

**How to define loop closure in this frame.** In classical SLAM, loop closure is the mechanism that recognizes a previously visited place and corrects accumulated error. In the DUSt3R family, how is "a previously visited place" represented, and in a pointmap-based map, how is correction propagated? MASt3R-SLAM handles it with the existing approach, but whether this is the best or a principled solution is unknown.

**Generalization of metric scale.** DUSt3R's pointmaps are in relative scale. The depth ratio between two images is recovered, but absolute scale is unknown. Just as Metric3D or Depth Anything v2 targeted metric depth, generalizing metric scale remains a problem for foundation 3D. Camera-independent metric is not easy even at foundation scale. The physical constraint of determining absolute scale without GPS or IMU exists regardless of data scale.

**Is this flow SLAM's future, or a separate branch?** Like 3DGS in Ch.15, foundation 3D is in the middle of being absorbed by the SLAM community. With MASt3R-SLAM and VGGT-SLAM arriving in succession in 2024–2025, the path of absorption has taken shape. But the point of real-time large-scale sequence operation, and the junction with the semantic branch noted in §16.6 (Hydra → ConceptGraphs → Clio), remains unclear. The form in which geometric foundation and semantic foundation meet in one system is a core axis of Ch.19's open problems.

---

The three chapters of Part 5 arrive at the same conclusion. Whether NeRF or foundation model, changing representation changes the insides of reconstruction and localization. The system-level structure of SLAM (keyframe, loop closure, map management) survives on top of the new representation. Ch.17 returns to a lineage that ran in parallel through this entire period: LiDAR-based SLAM. Different sensors, different culture, largely the same questions. Where Ch.14–16 asked what kind of representation holds a camera-based map, Ch.17 asks how the same engineering problems were approached when the input was a rotating laser rather than a lens.
