# Chapter 14. Monocular Depth Estimation: From Eigen to Depth Anything

Through the mid-2010s, the feature-based, direct, RGB-D, and place-recognition lineages each matured around explicit geometry: ORB-SLAM triangulated points via epipolar constraints, DSO enforced photometric consistency, KinectFusion registered surfaces with ICP, and geometric verification anchored loop closures. Machine learning played almost no part in core depth or pose estimation. When that boundary finally shifted, the catalyst came not from a SLAM paper, but from a computer vision experiment by a graduate student at NYU.

Inferring depth from a single photograph was among the oldest ill-posed problems in vision: perspective projection collapses the third dimension, leaving infinitely many 3D scenes consistent with the same 2D image. Yet humans judge depth effortlessly with one eye using perspective convergence, occlusion, texture gradients, and shading. In 2014, David Eigen at NYU asked whether a convolutional neural network could learn those statistical cues directly from data—launching a lineage that would reshape SLAM initialization and scale recovery a decade later.

---

## 14.1 Eigen 2014 — early CNN depth

Data-driven monocular depth estimation did not start in 2014. Nearly a decade earlier, Ashutosh Saxena at Stanford had introduced [Make3D, combining support vector machines (SVMs) with a Markov Random Field (MRF) to predict coarse, piecewise-planar depth from single outdoor images](https://papers.nips.cc/paper/2921-learning-depth-from-single-monocular-images) (2005), establishing the pre-deep-learning benchmark.

In 2014, David Eigen, Christian Puhrsch, and Rob Fergus replaced those hand-engineered components with a two-stage CNN ([Eigen et al. 2014](https://arxiv.org/abs/1406.2283)). A coarse network predicted global scene layout, and a fine network refined local edges and object boundaries. Trained on roughly 120,000 frames across 464 indoor scenes from NYU Depth v2, the network substantially outperformed Make3D and proved that end-to-end CNNs could extract metric 3D structure from monocular pixels.

Yet one flaw persisted from the outset: **scale ambiguity**. Because absolute depth predictions remained tied to the training corpus and camera intrinsics, a network trained on indoor NYU bedrooms often misjudged metric scale when shown an outdoor street—a limitation still central to monocular metric depth a decade later in 2024.

> 🔗 **Borrowed.** Eigen (2014) inherited its task formulation directly from Make3D (Saxena 2005), swapping the SVM and MRF for a two-scale CNN while keeping the problem definition and standard evaluation metrics—RMSE and threshold accuracy—intact.

---

## 14.2 Garg → Godard — self-supervised depth

Supervised depth learning quickly hit a data bottleneck. While Kinect captured clean indoor depth maps, its infrared pattern washed out outdoors in sunlight, making large-scale outdoor RGB-D datasets costly and scarce.

In 2016, [Ravi Garg at UCL bypassed ground-truth depth labels](https://arxiv.org/abs/1603.04992) by turning stereo image pairs into a self-supervised training signal: predict a depth map from the left view, warp it across the known stereo baseline to reconstruct the right view, and train on the photometric reconstruction error against the real right image.

At UCL, Clément Godard refined that idea into **MonoDepth** ([Godard et al. 2017](https://doi.org/10.1109/CVPR.2017.699)), enforcing left–right consistency between depths predicted independently from each stereo image and combining L1 error with structural similarity (SSIM) to stabilize textureless surfaces. Stereo pairs were needed only during training; at test time, MonoDepth inferred depth from a single image while outperforming contemporary self-supervised baselines on KITTI.

> 🔗 **Borrowed.** Garg and Godard adapted their photometric training objective directly from classical stereo matching, repurposing the intensity-consistency assumptions of disparity estimation—[systematized by Scharstein and Szeliski (2002)](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf)—as a differentiable supervisory signal for neural networks.

Two years later, [*MonoDepth2* (Godard et al. 2019, ICCV)](https://arxiv.org/abs/1806.01260) eliminated stereo cameras altogether in favor of raw monocular video. Training a depth network and a relative-pose network jointly to minimize photometric reprojection loss across adjacent frames, MonoDepth2 introduced two crisp mechanisms: **minimum reprojection loss**, which selects the lowest-error source frame per pixel to handle occlusions cleanly, and **auto-masking**, which ignores pixels moving at the same velocity as the camera (including static scenes observed by a stationary camera).

Even so, self-supervised video training carried clear blind spots: moving vehicles and specular reflections broke brightness constancy, featureless sky offered no gradient, and monocular video alone could recover depth only up to an unknown scale factor.

---

## 14.3 MiDaS — mixing datasets

Single-dataset models remained trapped inside their own visual domains until [Ranftl et al. 2020](https://doi.org/10.1109/TPAMI.2020.3019967) introduced **MiDaS** (*Mixing Datasets for Zero-shot Cross-dataset Transfer*). Led by René Ranftl at Intel, MiDaS asked how one depth network could learn simultaneously from wildly incompatible data sources.

Incompatible scales stood in the way: NYU provided indoor metric meters, KITTI supplied sparse outdoor LiDAR returns, ReDWeb offered uncalibrated stereo disparity from web videos, and MegaDepth stored up-to-scale SfM reconstructions. Training naively across those targets would feed the network contradictory supervision.

Ranftl resolved the clash with an **affine-invariant loss**. Before computing error, MiDaS normalizes both predicted and ground-truth disparity maps per image—subtracting the median to remove global shift and dividing by the mean absolute deviation from that median to factor out unknown scale. Stripping away absolute units freed the network to learn consistent relative depth ordering across radically different scenes.

Trained across multiple datasets—and expanding to as many as 12 datasets in later iterations—MiDaS achieved strong zero-shot generalization from cluttered rooms to outdoor landscapes, albeit strictly in relative rather than metric depth. In 2021, Ranftl's group followed up with [**DPT** (Dense Prediction Transformer)](https://arxiv.org/abs/2103.13413), swapping MiDaS's convolutional backbone for a Vision Transformer (ViT). DPT powered MiDaS v3 and its v3.1 (2022) refinement, delivering substantial gains in fine spatial structure.

> 🔗 **Borrowed.** MiDaS v3 adopted a ViT encoder via DPT, just as Depth Anything later adopted a DINOv2-pretrained encoder. Backbone upgrades became a primary driver of accuracy in foundation-era vision, and DPT (Ranftl 2021) offered one of the earliest large-scale demonstrations in dense depth prediction.

---

## 14.4 Depth Anything — foundation scale

In January 2024, Lihe Yang's team at TikTok Research scaled data-centric depth estimation to foundation proportions with **Depth Anything** ([Yang et al. 2024](https://arxiv.org/abs/2401.10891)). Pairing 1.5M labeled images merged across existing datasets with 62M unlabeled images, the pipeline trained on teacher-generated pseudo-labels regularized by auxiliary semantic-segmentation features.

With a 335M-parameter ViT-L backbone, Depth Anything surpassed MiDaS and prior models across KITTI, NYU, ScanNet, and DIODE—prioritizing broad zero-shot accuracy over real-time inference speed. Months later, [**Depth Anything v2**](https://arxiv.org/abs/2406.09414) trained a high-capacity teacher exclusively on synthetic datasets (including Virtual KITTI and Hypersim) before distilling pseudo-labels onto real images, capturing crisp boundaries, thin structures, and reflective or transparent materials that real sensors struggle to annotate.

Like MiDaS, base Depth Anything predicts relative depth without metric scale. To bridge back to metric units, [**ZoeDepth** (Shariq Farooq Bhat et al. 2023)](https://arxiv.org/abs/2302.12288) combined relative pretraining with domain-routed metric-bin heads, [**Depth Anything v2** (2024)](https://arxiv.org/abs/2406.09414) released metric checkpoints fine-tuned on metric datasets, and [**Metric3D v2** (2024)](https://arxiv.org/abs/2404.15506) normalized focal-length differences via a canonical camera space before rescaling predictions at test time. Still, generalizing metric scale reliably across unseen cameras and scenes remained a separate challenge.

---

## 14.5 Re-entry into SLAM

Once monocular depth networks matured, they flowed straight back into SLAM—starting with CNN-SLAM in 2017 and accelerating through the 2020s. Monocular SLAM normally stalls until camera translation builds enough parallax to triangulate initial points, and even then its scale remains arbitrary.

Injecting a learned depth prior on frame one resolves initial structure immediately—and if the depth model is metric or calibrated against a known reference, it anchors metric scale from the start. Follow-on systems building on [DROID-SLAM (Teed & Deng, 2021)](https://arxiv.org/abs/2108.10869), which couples recurrent optical flow with bundle adjustment, demonstrated how monocular depth priors tighten geometric initialization, while metric-calibrated predictions can serve as periodic scale anchors to curb monocular scale drift (unlike purely relative models such as MiDaS, which still need an external scale cue).

> 📜 **Prediction vs. outcome.** Eigen's 2014 paper anticipated fusing depth prediction with complementary 3D cues like surface normals, a multi-task thread later explored by PAD-Net and VPD. By 2024, however, scaling shared self-supervised ViT backbones proved far more decisive than explicit multi-task heads—shifting the field along a different axis than originally sketched.

> 📜 **Prediction vs. outcome.** MiDaS (2020) deliberately sacrificed absolute scale via its scale-and-shift-invariant loss to unlock cross-dataset relative depth. Subsequent work like ZoeDepth, metric Depth Anything v2, and Metric3D v2 restored metric predictions through targeted fine-tuning and canonical camera transforms, yet reliable metric generalization across arbitrary unseen cameras remains an active frontier.

---

## Still open

**Depth on reflective and transparent surfaces.** Glass, water, and mirrors violate ordinary surface shading because pixel appearance originates behind or off the surface rather than on it. Even with synthetic training corpora, foundation-scale models still suffer severe structural errors on reflective scenes, and specialized remedies such as [ClearGrasp (Sajjan et al. 2020)](https://arxiv.org/abs/1910.02550) fall short of a general solution.

**Separating ego-depth and object-depth in dynamic scenes.** Passing cars and pedestrians break the static-world assumption behind photometric self-supervision. Most self-supervised pipelines mask moving regions out rather than modeling them; despite efforts such as [Ranjan et al. (2019)](https://arxiv.org/abs/1805.09806) to disentangle ego-motion from independent object motion, robust dynamic depth estimation remains difficult in practice.

**Generalization of metric scale.** While ZoeDepth, metric Depth Anything v2, and Metric3D v2 made major strides when intrinsics are known, real-world footage from uncalibrated CCTV or archival cameras frequently lacks reliable metadata. Recovering accurate metric depth invariant to both domain shift and unknown camera models remained one of the field's central open problems in 2025.

---

Even while monocular depth networks were progressing from Eigen to Depth Anything, a much bolder question had already gripped the community starting in 2015: why stop at predicting depth, when a neural network might skip feature extraction, optimization, and map building altogether to regress camera pose straight from a single image? That provocation began with [PoseNet](https://arxiv.org/abs/1505.07427).