# Ch.11 — The Return of Depth Estimation: From Eigen to Depth Anything

In Part 3 (Ch.7–10) the feature-based, direct, RGB-D, and place recognition lineages each reached maturity on their own terms. Geometry was everything: ORB-SLAM reconstructed the world with epipolar geometry, DSO relied on photometric consistency, KinectFusion stacked surfaces with ICP, and RGB-D fusion pipelines closed loops on geometric features. There was no room for learning to break in, or so it seemed. Part 4 is the story of that boundary collapsing, and the crack came from an unexpected direction — not from a SLAM researcher but from the computer vision side, from a single paper by a graduate student at NYU.

Monocular depth estimation was one of the oldest ill-posed problems in computer vision. Recovering depth from a single image is in principle impossible. A camera throws away depth information when it projects the 3D world onto 2D. Yet humans judge depth with one eye. Perspective, occlusion, texture gradient, surface shading. What if these could be learned statistically? In 2014, David Eigen at NYU put that question to a CNN. That one experiment was the start of a lineage that would rewrite the SLAM pipeline ten years later.

---

## 1. Eigen 2014 — the first CNN depth

There was monocular depth estimation research before 2014. Ashutosh Saxena (Make3D, Stanford) published [a system in 2005 that combined support vector machines (SVMs) with a Markov Random Field (MRF) to predict a depth map from a single image](https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images). The results were coarse and barely worked in structured indoor environments.

[Eigen et al. 2014](https://arxiv.org/abs/1406.2283), by Eigen, Puhrsch, and Fergus, changed the approach itself. A two-stage CNN in which a coarse network predicted global structure and a fine network refined local detail. The training data was NYU Depth v2 — 120,000 indoor scenes collected with a Kinect RGB-D camera. The numbers improved on Make3D by the standards of the day, but the proof of concept mattered more. Depth is learnable.

One decisive weakness remained, though. **Scale ambiguity.** The network learns relative depth structure, but absolute scale is tied to the distribution of the training data. Point a model trained on NYU indoors at an outdoor scene and the scale is wrong. This limit stayed with the whole field as an open problem until 2024.

> 🔗 **Borrowed.** Eigen 2014 inherited the depth estimation task itself from Make3D (Saxena 2005). Replacing the SVM and MRF with a CNN was the core swap; the task definition and evaluation metrics (RMSE, threshold accuracy) carried over.

---

## 2. Garg → Godard — self-supervised depth

The bottleneck in supervised depth learning was data. The Kinect works well indoors, but outdoors, especially in sunlight, the infrared pattern washes out. Building a large-scale outdoor RGB-D dataset is expensive.

In 2016, [Ravi Garg (UCL) opened another path](https://arxiv.org/abs/1603.04992). Use stereo image pairs as the training signal. Predict depth from the left image, then use that depth and the camera baseline to reconstruct the right image. The right image already exists, so a photometric loss provides supervision. No labels needed.

Clément Godard (UCL) systematized the idea as **MonoDepth** in [Godard et al. 2017](https://doi.org/10.1109/CVPR.2017.699). Left-right consistency: a two-way constraint that the depth predicted from the left must match the depth predicted from the right. Adding structural similarity (SSIM) to the photometric loss raised stability in textureless regions. The key was that stereo pairs are only needed at training time. Inference runs on a single image. On the KITTI benchmark it was the best self-supervised method at the time.

> 🔗 **Borrowed.** Garg's and Godard's photometric loss comes out of the stereo matching literature. The intensity consistency constraint from disparity estimation, [as organized by Scharstein and Szeliski (2002)](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf), was repurposed as the training signal for a depth network.

In 2019 Godard's *MonoDepth2* ([Godard et al. 2019, ICCV](https://arxiv.org/abs/1806.01260)) moved further into self-supervision, using monocular video instead of stereo pairs. A depth network and a pose network train together. The pose network predicts camera motion between consecutive frames, and the depth network's output warps the previous frame into the current one. The two networks jointly optimize to reduce the warping error. Two key devices were added. First, **minimum reprojection loss**: pick the source frame with the lowest photometric error to reduce errors in occluded regions. Second, **auto-masking**: automatically exclude pixels that move at the same speed as the camera (including a stationary camera plus stationary objects).

A clean design, but problems remained. Moving objects and reflective surfaces broke photometric consistency, and sky was worse because it has no texture. Scale was still ambiguous too — video supervision only resolves scale relatively between frames.

---

## 3. MiDaS — mixing datasets

[Ranftl et al. 2020](https://doi.org/10.1109/TPAMI.2020.3019967), **MiDaS** (Mixing Datasets for Zero-shot Cross-dataset Transfer), led by René Ranftl at Intel, asked a different question. What if you train on many datasets at once instead of one?

The problem was that depth units and scales differ between datasets. NYU is indoor in meters, KITTI is outdoor LiDAR points, ReDWeb is stereo from movies, MegaDepth is SfM reconstruction. Mixing them as-is confuses the network.

Ranftl's fix was an **affine-invariant loss.** Before training, normalize each image's depth prediction by an affine transformation (scale plus shift). Specifically, subtract the median from both prediction and ground truth to remove shift, then divide by the median absolute deviation (MAD) to remove scale, then compare. This scale-and-shift invariant normalization eliminates unit mismatches between datasets. The network then learns "which is farther, relatively" rather than "how far."

Trained on 12 datasets and over 1.9 million images, MiDaS showed practical cross-dataset generalization for the first time. It produced plausible relative depth on outdoor scenes, indoor scenes, historical photographs, and movie frames. No absolute scale, but depth ordering and structure held.

Ranftl's team later released [**DPT** (Dense Prediction Transformer)](https://arxiv.org/abs/2103.13413) separately in 2021, swapping the MiDaS backbone for a ViT-based one. From MiDaS v3 onward DPT became the default backbone, and v3.1 (2022) was its refinement. Performance jumped.

> 🔗 **Borrowed.** MiDaS v3 and later Depth Anything adopted CLIP, DINOv2, and ViT-family backbones as-is. A backbone swap alone producing a performance jump is a common pattern in the foundation model era, but its first large-scale confirmation in depth estimation was DPT (Ranftl 2021).

---

## 4. Depth Anything — foundation scale

In January 2024, **Depth Anything** by Lihe Yang's team at TikTok Research ([Yang et al. 2024](https://arxiv.org/abs/2401.10891)) solved the problem with scale. 1.5M labeled images (a merger of existing datasets) and 62M unlabeled images. Pseudo-labels were generated for the unlabeled set and included in training. To raise pseudo-label quality, semantic segmentation features served as auxiliary supervision.

The result surpassed MiDaS and all earlier methods across every major benchmark — KITTI, NYU, ScanNet, DIODE. The model size was 335M parameters on a ViT-L backbone. Inference speed was nowhere near real time, but quality came first.

[**Depth Anything v2**](https://arxiv.org/abs/2406.09414), released later the same year, added large amounts of synthetic data (Unreal Engine-based Virtual KITTI, Hypersim, and others). Synthetic data covers regions that are hard to annotate in real data, such as reflective and transparent surfaces. v2 visibly improved over v1 in edge detail and thin structures.

Depth Anything still produces relative depth, though. No scale.

[**ZoeDepth** (Shariq Farooq Bhat et al. 2023)](https://arxiv.org/abs/2302.12288) and [**Metric3D v2** (2024)](https://arxiv.org/abs/2404.15506) attacked this last problem from a different angle. Camera intrinsics (focal length, sensor size) are fed to the network as explicit input. The network learns that for the same scene a different focal length yields a different depth distribution. Metric depth results on in-the-wild data were qualitatively different from before. Not perfect, but usable in many practical scenarios.

---

## 5. Re-entry into SLAM

Around 2021, SLAM researchers started pulling monocular depth models into their pipelines. The entry point was initialization. Monocular SLAM is structurally awkward to initialize. Triangulating from two frames needs a sufficient baseline, and scale is ambiguous from the first step.

Injecting a depth prior into the first frame speeds initialization and roughly fixes scale. [DROID-SLAM, released in 2021 by Teed and Deng](https://arxiv.org/abs/2108.10869), ties recurrent optical flow to BA; follow-up work in that lineage experimented with bolting monocular depth priors onto the geometric initialization.

Scale recovery was more direct. Monocular visual odometry (VO) accumulates scale drift as it runs. Using depth network predictions as periodic scale anchors suppresses this drift. Not a perfect solution, a practical patch, but it held over much longer distances than pure VO.

> 📜 **Prediction vs. outcome.** Eigen mentioned in the 2014 paper that combining with 3D geometry information such as surface normals was a natural direction for extension. Joint multi-task learning was partly realized later in PAD-Net, VPD, and others. But as of 2024 the real impact arguably came less from combining tasks and more from sharing a ViT backbone. The predicted direction and the actual path diverged. `[diverted]`

> 📜 **Prediction vs. outcome.** MiDaS (2020) chose the detour of giving up absolute scale through its scale-and-shift invariant loss and focusing only on relative depth, which lined up with the sense that metric recovery is hard without camera parameters. In 2024 Depth Anything v2 and Metric3D v2 attacked this direction head-on by taking camera intrinsics as input, and in-the-wild metric depth came close to practical quality, though full camera independence is not there yet. `[in progress]`

---

## 🧭 Still open

**Depth on reflective and transparent surfaces.** With glass, water, and metallic reflections, what the camera captures is not the actual surface. This is a problem at the level of physical optics. Even with more synthetic training data, generalization on real-world reflective scenes remains unstable. Specialized approaches exist, such as [ClearGrasp (Sajjan et al. 2020)](https://arxiv.org/abs/1910.02550), but no general solution. Even foundation-scale models show structurally large errors in this regime.

**Separating ego-depth and object-depth in dynamic scenes.** In scenes with moving cars and pedestrians, photometric consistency breaks. Self-supervised methods mask out moving objects as a workaround, which avoids the problem rather than solving it. Jointly solving for the depth of moving objects separately from ego-motion has been attempted by several follow-up works including [Ranjan et al. (2019)](https://arxiv.org/abs/1805.09806), but remains a hard problem at the practical level.

**Generalization of metric scale.** Metric3D v2 and Depth Anything v2 have started producing metric depth conditioned on camera intrinsics. But situations where intrinsics are unknown are common. There are hundreds of smartphone models, and CCTV and historical archive photos do not even carry EXIF. Camera-independent metric depth is hard even at foundation model scale. As of 2025 this is the remaining core question in monocular depth.

---

In 2024, while Depth Anything was turning over the benchmarks, a paper from Cambridge had already spent nine years as an unfinished piece of homework in the SLAM community. Pull absolute pose straight out of a single image. No feature extraction, no optimization. No map to begin with. [PoseNet](https://arxiv.org/abs/1505.07427) was the name of that dream.
