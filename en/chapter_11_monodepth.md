# Ch.11 — The Return of Depth Estimation: From Eigen to Depth Anything

The feature-based, direct, RGB-D, and place-recognition lineages each matured around geometric methods. ORB-SLAM reconstructed the world through epipolar geometry, DSO relied on photometric consistency, KinectFusion aligned surfaces with ICP, and RGB-D fusion pipelines closed loops with geometric features. Learning had little role in these systems. That boundary began to shift with a computer vision paper by a graduate student at NYU rather than with a SLAM paper.

Monocular depth estimation was one of the oldest ill-posed problems in computer vision. In principle, a single image cannot determine depth because projection discards the third dimension. Humans nevertheless infer depth with one eye from perspective, occlusion, texture gradients, and surface shading. In 2014, David Eigen at NYU tested whether a CNN could learn those statistical cues. The experiment began a lineage that would alter the SLAM pipeline ten years later.

---

## 1. Eigen 2014 — early CNN depth

Monocular depth estimation research predated 2014. In 2005, Ashutosh Saxena at Stanford published [Make3D, a system that combined support vector machines (SVMs) with a Markov Random Field (MRF) to predict a depth map from a single image](https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images). Make3D predicted coarse, piecewise-planar 3D structure primarily for outdoor scenes and became a representative learned monocular-depth system before CNNs.

[Eigen et al. 2014](https://arxiv.org/abs/1406.2283), by Eigen, Puhrsch, and Fergus, introduced a different approach. Its two-stage CNN used a coarse network to predict global structure and a fine network to refine local detail. Training used roughly 120,000 frames from 464 indoor scenes in NYU Depth v2. The method improved on Make3D by the standards of the time and demonstrated that a network could learn to estimate depth.

One weakness remained: **scale ambiguity**. The network's absolute scale depended on the training distribution and camera setting. A model trained on indoor NYU data could therefore produce incorrect scale on an outdoor scene. The issue remained central to monocular metric depth in 2024.

> 🔗 **Borrowed.** Eigen 2014 inherited the depth-estimation task from Make3D (Saxena 2005). It replaced the SVM and MRF with a CNN while retaining the task definition and evaluation metrics, including RMSE and threshold accuracy.

---

## 2. Garg → Godard — self-supervised depth

The bottleneck in supervised depth learning was data. The Kinect works well indoors, but outdoors, especially in sunlight, the infrared pattern washes out. Building a large-scale outdoor RGB-D dataset is expensive.

In 2016, [Ravi Garg at UCL introduced another approach](https://arxiv.org/abs/1603.04992): using stereo image pairs as the training signal. The system predicts depth from the left image, then uses that depth and the camera baseline to reconstruct the right image. Because the right image is observed, a photometric loss provides supervision without labels.

Clément Godard at UCL developed the idea into **MonoDepth** in [Godard et al. 2017](https://doi.org/10.1109/CVPR.2017.699). Its left-right consistency term imposes a two-way constraint between depths predicted from the left and right images. Adding structural similarity (SSIM) to the photometric loss improved stability in textureless regions. Stereo pairs were required only during training; inference used a single image. Under the paper's contemporary KITTI protocol, it reported lower error than the self-supervised methods listed in its comparison table.

> 🔗 **Borrowed.** Garg and Godard derived their photometric loss from the stereo-matching literature. They repurposed the intensity-consistency constraint from disparity estimation, [as organized by Scharstein and Szeliski (2002)](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf), as the training signal for a depth network.

In 2019, Godard's *MonoDepth2* ([Godard et al. 2019, ICCV](https://arxiv.org/abs/1806.01260)) extended self-supervision to monocular video instead of stereo pairs. A depth network and a pose network train jointly: the pose network predicts camera motion between consecutive frames, and the depth estimate warps the previous frame into the current one. Both networks optimize the resulting warping error. **Minimum reprojection loss** selects the source frame with the lowest photometric error, reducing errors in occluded regions. **Auto-masking** excludes pixels that move at the same speed as the camera, including the case of a stationary camera observing stationary objects.

The design still had several limitations. Moving objects and reflective surfaces violated photometric consistency, while the textureless sky provided little signal. Scale also remained ambiguous because video supervision recovers only relative scale between frames.

---

## 3. MiDaS — mixing datasets

[Ranftl et al. 2020](https://doi.org/10.1109/TPAMI.2020.3019967), **MiDaS** (Mixing Datasets for Zero-shot Cross-dataset Transfer), led by René Ranftl at Intel, trained one model on multiple datasets rather than a single dataset.

Depth units and scales differ across these datasets. NYU contains indoor metric depth, KITTI provides outdoor LiDAR points, ReDWeb derives stereo data from movies, and MegaDepth uses SfM reconstructions. Combining them without normalization would give the network inconsistent targets.

Ranftl addressed the mismatch with an **affine-invariant loss**. Before comparison, the method normalizes each image's predicted and ground-truth depth through an affine transformation consisting of scale and shift. It subtracts the median to remove the shift and divides by the mean absolute deviation from that median to remove scale. This scale-and-shift-invariant normalization eliminates unit mismatches across datasets, so the network learns relative depth ordering rather than absolute distance.

The original MiDaS mixed several datasets and demonstrated zero-shot transfer to datasets excluded from training. Later MiDaS releases expanded the training mixture to as many as 12 datasets. The family estimated relative depth across distributions as different as indoor and outdoor imagery, but it did not recover absolute scale.

Ranftl's team separately released [**DPT** (Dense Prediction Transformer)](https://arxiv.org/abs/2103.13413) in 2021, replacing the MiDaS backbone with a ViT-based architecture. DPT became the default backbone from MiDaS v3 onward, followed by a refinement in v3.1 (2022). The change substantially improved performance.

> 🔗 **Borrowed.** DPT in MiDaS v3 used a ViT-based encoder, while the later Depth Anything used an encoder pretrained with DINOv2. Replacing the backbone became a common source of performance gains in the foundation-model era, and DPT (Ranftl 2021) provided an early large-scale demonstration in depth estimation.

---

## 4. Depth Anything — foundation scale

In January 2024, **Depth Anything**, developed by Lihe Yang's team at TikTok Research ([Yang et al. 2024](https://arxiv.org/abs/2401.10891)), approached depth estimation through training-data scale. It used 1.5M labeled images, merged from existing datasets, and 62M unlabeled images. The method generated pseudo-labels for the unlabeled set and included them in training, using semantic-segmentation features as auxiliary supervision to improve their quality.

The model surpassed MiDaS and earlier methods across the major KITTI, NYU, ScanNet, and DIODE benchmarks. Its ViT-L backbone contained 335M parameters. Inference was far from real-time, reflecting the method's emphasis on prediction quality.

[**Depth Anything v2**](https://arxiv.org/abs/2406.09414), released later the same year, trained a teacher on synthetic datasets including Virtual KITTI and Hypersim, then trained student models on real images pseudo-labeled by that teacher. Such data covers surfaces that are difficult to annotate in real imagery, including reflective and transparent materials. Version 2 visibly improved edge detail and thin structures over v1.

Depth Anything still produces relative depth without scale.

[**ZoeDepth** (Shariq Farooq Bhat et al. 2023)](https://arxiv.org/abs/2302.12288) combined relative-depth pretraining with domain-specific metric-bin heads and automatic routing. [**Depth Anything v2** (2024)](https://arxiv.org/abs/2406.09414) offered metric models obtained by fine-tuning a strong relative-depth backbone on metric labels. [**Metric3D v2** (2024)](https://arxiv.org/abs/2404.15506) handled ambiguity across focal lengths through a canonical-camera-space transformation and converted predictions back with camera parameters at inference. All three extended the relative-depth lineage toward metric prediction, but generalization across training domains and camera settings remained a separate problem.

---

## 5. Re-entry into SLAM

CNN-SLAM incorporated monocular depth predictions into SLAM in 2017. Later systems also used learned depth for initialization. Monocular SLAM requires a sufficient baseline to triangulate from two frames and has ambiguous scale from the outset.

Adding a depth prior to the first frame accelerates initialization and supplies an initial scene structure. A metric model, or a prior calibrated against a known reference, can also initialize metric scale. [DROID-SLAM, released in 2021 by Teed and Deng](https://arxiv.org/abs/2108.10869), combines recurrent optical flow with BA. Follow-up work in that lineage incorporated monocular depth priors into geometric initialization.

Metric-calibrated depth predictions can also act as periodic scale anchors for monocular visual odometry and suppress scale drift. A relative-depth output such as MiDaS, however, cannot determine absolute scale without an external metric reference.

> 📜 **Prediction vs. outcome.** Eigen's 2014 paper identified integration with 3D geometric information, such as surface normals, as a natural extension. PAD-Net, VPD, and other systems later implemented parts of this joint multi-task approach. By 2024, however, shared ViT backbones arguably had greater impact than explicit task combination. The field developed along a different path from the one proposed.

> 📜 **Prediction vs. outcome.** MiDaS (2020) used a scale-and-shift-invariant loss and focused on relative depth. ZoeDepth and the metric models of Depth Anything v2 later used fine-tuning on metric labels, while Metric3D v2 corrected camera-model variation through a canonical space. Metric prediction became more broadly useful, but scale generalization to unseen cameras and domains remained in progress.

---

## 🧭 Still open

**Depth on reflective and transparent surfaces.** For glass, water, and metallic reflections, the image does not directly represent the physical surface. This optical ambiguity remains difficult even with additional synthetic training data, and generalization to real reflective scenes is unstable. Specialized approaches such as [ClearGrasp (Sajjan et al. 2020)](https://arxiv.org/abs/1910.02550) exist, but no general solution does. Even foundation-scale models produce large structural errors in this regime.

**Separating ego-depth and object-depth in dynamic scenes.** Moving cars and pedestrians violate photometric consistency. Self-supervised methods often mask moving objects, avoiding rather than solving the problem. Several later studies, including [Ranjan et al. (2019)](https://arxiv.org/abs/1805.09806), have attempted to estimate moving-object depth separately from ego-motion, but a practical solution remains difficult.

**Generalization of metric scale.** ZoeDepth and the metric variants of Depth Anything v2 learn scale from metric labels, while Metric3D v2 explicitly handles camera-parameter variation. CCTV and archival imagery may still lack reliable intrinsics or metadata. Metric depth that generalizes independently of training domain and camera model remains difficult even for foundation models and was a central open question in 2025.

---

By 2024, while Depth Anything was advancing depth benchmarks, a Cambridge paper had posed another unresolved problem to the SLAM community for nine years: estimating absolute pose directly from a single image without feature extraction, optimization, or an initial map. The paper was [PoseNet](https://arxiv.org/abs/1505.07427).
