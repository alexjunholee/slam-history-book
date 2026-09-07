# Ch.12 — The End-to-End Frustration

Eigen's network estimated metric depth from pixels, and SfMLearner obtained geometric supervision without labels. Once learning could infer shape, researchers asked whether one network could perform the entire SLAM task rather than only pose estimation or loop closure. Work from 2015 to 2018 did not produce a successful solution.

In 2015, Alex Kendall, a PhD student working under Roberto Cipolla at the Cambridge Computer Laboratory, trained a neural network on Google Street View images to estimate a 6-DoF pose from a single photograph. [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336) drew immediate attention at ICCV in Santiago de Chile. It suggested that a single CNN might replace the feature extraction, matching, optimization, and map management developed over thirty years of SLAM research. Dozens of papers explored that possibility between 2015 and 2018, and nearly all encountered the same limitations.

---

## 12.1 PoseNet

PoseNet followed [AlexNet (Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks), adapting the high-level visual representations learned by ImageNet classification networks to pose estimation.

> 🔗 **Borrowed.** PoseNet uses the [GoogLeNet (Inception, Szegedy et al. 2014)](https://arxiv.org/abs/1409.4842) architecture as its backbone. It replaces the classification head with a 7-dimensional regression head for x, y, z, and four quaternion components, directly adapting an ImageNet feature hierarchy to localization.

On the Cambridge Landmarks dataset collected by Kendall, which included King's College Chapel, streets, a former hospital, and several other outdoor scenes, PoseNet achieved position errors of around 2 m and orientation errors of 5–8°, depending on the scene (§5 of the original paper). A single GPU produced a pose within 5 ms without feature extraction, RANSAC, or map lookup.

The paper triggered immediate follow-ups. [Bayesian PoseNet (Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909) tried to estimate pose uncertainty via Monte Carlo Dropout. LSTM PoseNet integrated sequence information. Variants with added geometric loss appeared. Kendall himself released a 2017 version combining a recurrent structure with photometric loss.

As benchmark performance improved, however, the gap became visible. On the same scenes, [Active Search (Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf) and DenseVLAD achieved position errors of roughly 0.2 m. The PoseNet family rarely improved beyond errors of several meters, revealing a fundamental limitation of regressing an absolute pose from one image.

---

## 12.2 DeepVO

To address PoseNet's reliance on a single image, Sen Wang of Heriot-Watt University in Edinburgh and his co-authors used image sequences. They presented [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429) at ICRA in 2017. A CNN influenced by FlowNet extracted optical-flow features from consecutive frame pairs, and an LSTM accumulated temporal context to estimate visual odometry directly.

> 🔗 **Borrowed.** DeepVO's training labels are KITTI's GPS/IMU ground truth, and its feature extraction design is borrowed directly from the optical flow CNN architecture of [FlowNet (Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852).

The LSTM was intended to use temporal context to suppress drift. DeepVO showed lower drift than DVO-SLAM or VISO2-M on parts of the KITTI sequences, but only under conditions resembling the training data in driving patterns, lighting, urban appearance, and speed profiles. When conditions differed, the accumulated context became a source of bias.

[Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813), released the same year by Tinghui Zhou at UC Berkeley, took a different approach. It jointly estimated depth and ego-motion through self-supervised learning, using photometric reprojection loss as the training signal and requiring no labels.

> 🔗 **Borrowed.** SfMLearner's photometric loss is mathematically identical to the intensity residual in classical direct SLAM. It placed the photometric principle of [DSO (Engel et al. 2018)](https://arxiv.org/abs/1607.02565) in a differentiable learning framework, and its self-supervision later appeared in MonoDepth2 and DROID-SLAM.

SfMLearner's evaluation on short VO snippets must be distinguished from ORB-SLAM's system performance over longer trajectories with loop closure.

---

## 12.3 Three causes of failure

Between 2019 and 2020, researchers increasingly examined the limitations of this work. In a 2019 talk, Sudeep Pillai of MIT, later TRI, organized the structural problems of end-to-end methods into three categories.

**First: absence of inductive bias.** Classical SLAM explicitly encoded geometric constraints accumulated over decades, including the epipolar constraint, rigid-body motion, scale invariance, and spatial continuity. A CNN had to infer them from data, while ImageNet classification offered little supervision for the metric geometry of 3D space. Even when a regression network estimated the correct pose, it was difficult to determine whether it had learned 3D structure or memorized combinations of lighting, color, and texture.

**Second: generalization failure.** Performance collapsed outside the training distribution. A PoseNet trained on Cambridge Landmarks was unusable on Oxford streets, and a DeepVO trained on KITTI accumulated exponentially growing drift on other vehicle datasets without radar. Classical ORB-SLAM also failed when feature detection broke down or lighting changed drastically, but its failure was predictable and the system could reinitialize. End-to-end models could instead return an incorrect estimate without indicating the magnitude of the error.

**Third: absence of uncertainty quantification.** SLAM cannot function only as a pose estimator because downstream systems, including path planning and obstacle avoidance, require the covariance of the localization estimate. EKFs and factor graphs propagate covariance naturally. Bayesian PoseNet attempted to estimate variance through dropout, but the calibration between that variance and actual position error was difficult to verify. On inputs outside the training distribution, it could return a confident but incorrect estimate, which is more dangerous for a robotic system than a visible failure.

---

## 12.4 Reassessment

After completing his PhD in 2019, Kendall moved to Wayve and shifted toward imitation learning and world-model research for autonomous driving. He continued to study learning-based localization but rejected absolute pose regression from a single image as the appropriate problem formulation.

Earlier, in 2017, Federico Tombari's group at TU Munich, later Google, developed [CNN-SLAM (Tateno et al. 2017)](https://arxiv.org/abs/1704.03489). It fused dense depth predicted by a CNN with the depth estimate from direct monocular SLAM. Because learning was confined to dense depth, the method was not fully end to end, but it tested whether a CNN could address scale ambiguity and low-texture regions in monocular SLAM. Results varied across scenes, and the method did not consistently improve accuracy.

> 📜 **Prediction vs. outcome.** In the PoseNet paper (2015), Kendall identified uncertainty estimation, temporal integration, and extension to larger scenes as the next tasks. Bayesian PoseNet (2016), LSTM PoseNet (2016), and multiple outdoor experiments pursued all three directions. Each encountered further limitations, and researchers ultimately abandoned the broader absolute-pose-regression approach. The proposed extensions could not overcome the weakness of the underlying formulation.

Some components remained useful in other settings. SfMLearner's photometric self-supervision continued in monocular-depth methods such as MonoDepth2 (Godard 2019). DROID-SLAM (Teed & Deng 2021) also uses differentiable geometry, but trains with pose and optical-flow supervision. DeepVO's LSTM-based temporal modeling also reappeared in modified form in visual-inertial learning research. The methods changed even as their individual ideas persisted.

> 📜 **Prediction vs. outcome.** In the SfMLearner paper (2017), Zhou identified dynamic-object handling and robustness to photometric noise as remaining tasks. Later self-supervised work, including [GeoNet (Yin & Shi 2018)](https://arxiv.org/abs/1803.02276), made partial progress. Self-supervised VO did not replace SLAM in the mainstream, but photometric self-supervision persisted after the field rejected end-to-end VO as the larger objective.

---

## 12.5 The lesson that remained

Around 2020, a common design rule was to keep geometry explicit and use learning for features and priors.

> 🔗 **Borrowed.** CodeSLAM (Bloesch 2018) and DROID-SLAM (Teed & Deng 2021) implement this principle. Both retain the geometric structure of a factor graph or bundle adjustment and use learning for depth representation in CodeSLAM and for dense correspondence and recurrent updates in DROID-SLAM, retaining constraints that PoseNet had removed.

The classical pipeline was not uniformly superior to learning-based alternatives. ORB-SLAM also failed in textureless environments, at night, and in rain. The distinction was that errors from end-to-end models were less interpretable and predictable than failures in classical SLAM.

Neither a particular dataset nor a particular architecture fully explained the failure. A direct mapping from image to pose omitted the geometric constraints accumulated over thirty years.

---

## 🧭 Still open

**Which inductive bias to inject, and how.** Even if geometry remains part of the algorithm, it is unclear which constraints should be encoded and at what level. Candidates include rigid-body motion and the epipolar constraint. Foundation models are again blurring this boundary, while GaussianSLAM and 3DGS-based systems explore how much geometry can reside within a learned representation.

**Calibration of learned uncertainty.** Bayesian PoseNet did not resolve this problem. Whether uncertainty estimates from deep learning remain calibrated to actual error, especially for out-of-distribution inputs, was still open as of 2026. Autonomous-driving applications make the question practically urgent.

**Redefining "end-to-end."** PoseNet's definition of end to end, learning a direct image-to-pose mapping, failed. Since foundation models appeared in 2023, however, the term has begun to change. Researchers are again deciding which SLAM modules should be learned and which should remain explicit algorithms.

CodeSLAM, released by Andrew Davison's lab at Imperial College London in 2018, embodied this division between explicit geometry and learned representation.
