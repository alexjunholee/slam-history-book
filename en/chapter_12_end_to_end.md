# Ch.12 — The End-to-End Frustration

Eigen's network estimated metric depth from pixels, and SfMLearner obtained geometric supervision without labels. Once learning could infer shape, researchers asked whether one network could perform the entire SLAM task rather than only pose estimation or loop closure. Work from 2015 to 2018 did not produce a successful solution.

In 2015, Alex Kendall, a PhD student working under Roberto Cipolla at the Cambridge Computer Laboratory, trained a neural network on Google Street View images to estimate a 6-DoF pose from a single photograph. [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336) drew immediate attention at ICCV in Santiago de Chile. It suggested that a single CNN might replace the feature extraction, matching, optimization, and map management developed over thirty years of SLAM research. Dozens of papers explored that possibility between 2015 and 2018, and nearly all encountered the same limitations.

---

## 12.1 PoseNet

PoseNet followed [AlexNet (Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks), adapting the high-level visual representations learned by ImageNet classification networks to pose estimation.

> 🔗 **Borrowed.** PoseNet uses the [GoogLeNet (Inception, Szegedy et al. 2014)](https://arxiv.org/abs/1409.4842) architecture as its backbone. It replaces the classification head with a 7-dimensional regression head for x, y, z, and four quaternion components, directly adapting an ImageNet feature hierarchy to localization.

On the Cambridge Landmarks dataset collected by Kendall, which included King's College Chapel, streets, a former hospital, and several other outdoor scenes, PoseNet achieved position errors of around 2 m and orientation errors of 5–8°, depending on the scene (§5 of the original paper). A single GPU produced a pose within 5 ms without feature extraction, RANSAC, or map lookup.

Follow-up studies changed different parts of the method. [Bayesian PoseNet (Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909) estimated pose uncertainty through Monte Carlo Dropout. [Walch et al.'s LSTM PoseNet](https://arxiv.org/abs/1611.07890), first released in 2016 and presented at ICCV 2017, used LSTMs to process spatial relationships among features from a single image, rather than a temporal image sequence. [Kendall and Cipolla's 2017 study](https://arxiv.org/abs/1704.00390) examined loss functions for pose regression. Its reprojection loss compared the image coordinates of 3D points projected with ground-truth and predicted poses; it was not a photometric loss based on brightness differences.

As benchmark performance improved, however, the gap became visible. On the same scenes, [Active Search (Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf) and DenseVLAD achieved position errors of roughly 0.2 m. The PoseNet family rarely improved beyond errors of several meters, revealing a fundamental limitation of regressing an absolute pose from one image.

---

## 12.2 DeepVO

To address PoseNet's reliance on a single image, Sen Wang of Heriot-Watt University in Edinburgh and his co-authors used image sequences. They presented [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429) at ICRA in 2017. A CNN influenced by FlowNet extracted optical-flow features from consecutive frame pairs, and an LSTM accumulated temporal context to estimate visual odometry directly.

> 🔗 **Borrowed.** DeepVO's training labels are KITTI's GPS/IMU ground truth, and its feature extraction design is borrowed directly from the optical flow CNN architecture of [FlowNet (Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852).

The LSTM was intended to use temporal context to suppress drift. DeepVO showed lower drift than DVO-SLAM or VISO2-M on parts of the KITTI sequences, but only under conditions resembling the training data in driving patterns, lighting, urban appearance, and speed profiles. When conditions differed, the accumulated context became a source of bias.

[Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813), released the same year by Tinghui Zhou at UC Berkeley, took a different approach. It jointly estimated depth and ego-motion through self-supervised learning, using photometric reprojection loss as the training signal and requiring no labels.

> 🔗 **Borrowed.** SfMLearner and [DSO (Engel et al. 2018)](https://arxiv.org/abs/1607.02565) share the principle of comparing brightness at geometrically corresponding image locations. DSO uses these differences to optimize poses and a map directly; SfMLearner uses them to train networks that predict depth and camera motion. SfMLearner's self-supervised approach continued in monocular-depth methods such as MonoDepth2. DROID-SLAM also uses differentiable geometry, but its pose and optical-flow supervision distinguishes it from this self-supervised line.

SfMLearner's evaluation on short VO snippets must be distinguished from ORB-SLAM's system performance over longer trajectories with loop closure.

---

## 12.3 Geometry, generalization, and uncertainty

Geometric constraints, generalization beyond the training distribution, and uncertainty estimation raised distinct questions for end-to-end pose regression and learned odometry around 2019–2020. [Sattler et al. (2019)](https://arxiv.org/abs/1903.07504), for example, examined the limits of absolute pose regression; these results should be distinguished from the temporal estimation problem addressed by learned odometry.

**First: absence of inductive bias.** Classical SLAM explicitly encoded geometric constraints accumulated over decades, including the epipolar constraint, rigid-body motion, scale invariance, and spatial continuity. A CNN had to infer them from data, while ImageNet classification offered little supervision for the metric geometry of 3D space. Even when a regression network estimated the correct pose, it was difficult to determine whether it had learned 3D structure or memorized combinations of lighting, color, and texture.

**Second: generalization beyond the training distribution.** A scene-specific PoseNet cannot directly supply absolute coordinates for an unseen scene. Learned odometry estimates relative motion, but deployment with a different camera, motion pattern, or image distribution requires its own validation. [TartanVO](https://proceedings.mlr.press/v155/wang21h.html), presented at CoRL 2020, treated cross-dataset generalization as a central design problem. Classical ORB-SLAM can also lose tracking when features or lighting change; recovery procedures and failure detection are necessary for both geometric and learned systems.

**Third: validating uncertainty estimates.** Path planning and obstacle avoidance need information about localization error as well as the estimated pose. An EKF updates covariance, and a factor graph can provide uncertainty around an estimate, but both depend on modeling and approximation choices. Bayesian PoseNet used dropout to estimate uncertainty and evaluated it for localization-error estimation and detection of whether the input showed the learned scene. These results do not guarantee calibration for arbitrary inputs outside the training distribution.

---

## 12.4 Reassessment

Kendall received his PhD at Cambridge in 2017 and co-founded Wayve that year. [His later research](https://alexgkendall.com/) included imitation learning and world models for autonomous driving, expanding from camera pose estimation to scene understanding and learned driving behavior.

[CNN-SLAM (Tateno, Tombari, Laina, and Navab, 2017)](https://arxiv.org/abs/1704.03489) fused CNN-predicted dense depth with depth measurements from direct monocular SLAM. It relied more on predicted depth in low-texture regions and on SLAM measurements where geometric estimation was reliable. It also used predicted depth to estimate absolute scale. Evaluated on two benchmarks, the method was an early example of combining learning and geometry.

> 📜 **Prediction vs. outcome.** The PoseNet paper (2015) proposed further work on training data obtained through multiview geometry, probabilistic extensions, and the limits on scene size that one model could learn. Bayesian PoseNet (2016) addressed probabilistic pose estimation and uncertainty evaluation. LSTM PoseNet's spatial feature processing does not demonstrate temporal integration, and these follow-up studies do not establish that all of the original tasks were resolved. Accuracy and generalization in absolute pose regression remained subjects of evaluation.

Some components remained useful in other settings. SfMLearner's photometric self-supervision continued in monocular-depth methods such as MonoDepth2 (Godard 2019). DROID-SLAM (Teed & Deng 2021) also uses differentiable geometry, but trains with pose and optical-flow supervision. DeepVO's LSTM-based temporal modeling also reappeared in modified form in visual-inertial learning research. The methods changed even as their individual ideas persisted.

> 📜 **Prediction vs. outcome.** In the SfMLearner paper (2017), Zhou identified dynamic-object handling and robustness to photometric noise as remaining tasks. Later self-supervised work, including [GeoNet (Yin & Shi 2018)](https://arxiv.org/abs/1803.02276), made partial progress. GeoNet jointly learned depth, optical flow, and camera motion while modeling static and dynamic regions separately.

---

## 12.5 The lesson that remained

Around 2020, a common design rule was to keep geometry explicit and use learning for features and priors.

> 🔗 **Borrowed.** CodeSLAM (Bloesch 2018) and DROID-SLAM (Teed & Deng 2021) implement this principle. Both retain the geometric structure of a factor graph or bundle adjustment and use learning for depth representation in CodeSLAM and for dense correspondence and recurrent updates in DROID-SLAM, retaining constraints that PoseNet had removed.

The classical pipeline was not uniformly superior to learning-based alternatives. PoseNet demonstrated operation under some lighting changes and motion blur where feature-based registration failed. In contrast, Sattler et al. (2019) found that the absolute pose regression methods they evaluated did not match the accuracy of structure-based localization. The comparison depended on the input conditions and evaluation task.

The important distinction was between directly regressing pose from an image and estimating pose from visual correspondences under geometric constraints. CodeSLAM and DROID-SLAM combined learned representations with the latter computation.

---

## 🧭 Still open

**Which inductive bias to inject, and how.** Even if geometry remains part of the algorithm, it is unclear which constraints should be encoded and at what level. Candidates include rigid-body motion and the epipolar constraint. Foundation models are again blurring this boundary, while GaussianSLAM and 3DGS-based systems explore how much geometry can reside within a learned representation.

**Calibration of learned uncertainty.** Bayesian PoseNet did not resolve this problem. Whether uncertainty estimates from deep learning remain calibrated to actual error, especially for out-of-distribution inputs, was still open as of 2026. Autonomous-driving applications make the question practically urgent.

**Redefining "end-to-end."** PoseNet implemented end to end as a learned direct mapping from image to pose. Since foundation models appeared in 2023, however, the term has begun to change. Researchers are again deciding which SLAM modules should be learned and which should remain explicit algorithms.

CodeSLAM, released by Andrew Davison's lab at Imperial College London in 2018, embodied this division between explicit geometry and learned representation.
