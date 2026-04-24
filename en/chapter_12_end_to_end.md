# Ch.12 — The End-to-End Frustration

Chapter 11 made it possible to recover depth from a single monocular camera. Eigen's network pulled metric depth out of pixels, and SfMLearner produced geometric supervision without labels. Learning had been shown to see shape, and the obvious next question followed: pose estimation, loop closure, could the whole of SLAM be finished off in a single network? Between 2015 and 2018, this question failed to find its answer.

In 2015, Alex Kendall, a PhD student at the Cambridge Computer Laboratory, working under Roberto Cipolla, completed a project that trained a neural network on Google Street View images and took a single photograph as input to output a 6-DoF pose. The paper, named [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336), drew immediate attention at ICCV in Santiago de Chile. What if SLAM's thirty-year equation — feature extraction, matching, optimization, map management — could be compressed into a single CNN? Between 2015 and 2018 this question produced dozens of papers that, with almost no exception, reached the same conclusion.

---

## 12.1 PoseNet

What PoseNet inherited was [AlexNet (Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks). Kendall took the finding that deep CNNs trained for classification on ImageNet form high-level visual representations and repurposed that feature hierarchy for pose estimation.

> 🔗 **Borrowed.** PoseNet's backbone is the [GoogLeNet (Inception, Szegedy et al. 2014)](https://arxiv.org/abs/1409.4842) architecture. Remove the classification head, attach a 7-dimensional regression head (x, y, z, and four quaternion components) — that was the whole of it. A direct transplant of the feature hierarchy learned from ImageNet into localization.

On the Cambridge Landmarks dataset Kendall collected himself — King's College Chapel, streets, a former hospital, and several other outdoor scenes — PoseNet reached position errors around 2 m and orientation errors of 5-8° depending on the scene (per §5 of the original paper). By 2015 standards these were impressive numbers. A single GPU returned an answer within 5 ms, without feature extraction, RANSAC, or map lookup.

The paper triggered immediate follow-ups. [Bayesian PoseNet (Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909) tried to estimate pose uncertainty via Monte Carlo Dropout. LSTM PoseNet integrated sequence information. Variants with added geometric loss appeared. Kendall himself released a 2017 version combining a recurrent structure with photometric loss.

But as benchmark ceilings rose, the gap became visible. On the same scenes, [Active Search (Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf) and DenseVLAD achieved position errors on the order of 0.2 m. The PoseNet family rarely got below the several-meter range. Regressing an absolute pose from a single image had a principled limit.

---

## 12.2 DeepVO

If one of PoseNet's limits was single-image input, what about feeding in a sequence? Sen Wang (Heriot-Watt, Edinburgh) and co-authors presented [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429) at ICRA in 2017. A CNN influenced by FlowNet extracted optical flow features from consecutive frame pairs, and an LSTM accumulated temporal context to output VO directly.

> 🔗 **Borrowed.** DeepVO's training labels are KITTI's GPS/IMU ground truth, and its feature extraction design is borrowed directly from the optical flow CNN architecture of [FlowNet (Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852).

With the LSTM handling temporal modeling, the hope was drift suppression. The paper showed results where DeepVO had lower drift than DVO-SLAM or VISO2-M on parts of the KITTI sequences. But there were conditions. Driving patterns similar to the training sequences, similar lighting, similar urban scenes, similar speed profiles. When conditions diverged, the "context" the LSTM had accumulated turned into bias instead.

[Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813), released the same year by Tinghui Zhou (UC Berkeley), came at the problem from a different angle. Self-supervised learning estimated depth and ego-motion jointly, using photometric reprojection loss as the training signal. The strength was that training needed no labels.

> 🔗 **Borrowed.** SfMLearner's photometric loss is mathematically identical to the intensity residual of classical direct SLAM, moving the photometric principle of [DSO (Engel et al. 2018)](https://arxiv.org/abs/1607.02565) into a differentiable learning framework — the self-supervision idea survived downstream through MonoDepth2 and into DROID-SLAM.

That said, SfMLearner-only VO finished on the official KITTI leaderboard at less than half of ORB-SLAM's performance.

---

## 12.3 Three causes of failure

Between 2019 and 2020, papers in this area began a shared self-criticism. Sudeep Pillai (MIT, later TRI) in a 2019 talk systematized the structural limits of the end-to-end approach along three axes.

**First: absence of inductive bias.** Classical SLAM had carved into its algorithmic structure the geometric constraints accumulated over decades — epipolar constraint, rigid body motion assumption, scale invariance, spatial continuity. CNNs had to learn these from data. ImageNet's cats and cars do not teach the metric geometry of 3D space. Even when a regression network appears to hit the right pose, it was hard to tell whether that was because it understood 3D space or because it had memorized specific combinations of lighting, color, and texture.

**Second: generalization failure.** Step outside the training set and performance collapsed. A PoseNet trained on Cambridge Landmarks was unusable on Oxford streets. A DeepVO trained on KITTI saw drift grow exponentially on other vehicle datasets without radar. Classical ORB-SLAM also failed (it lost tracking when feature detection failed or lighting changed drastically), but that failure was predictable and it could reinitialize. End-to-end tended to be quietly wrong, and wrong without any signal of how wrong.

**Third: absence of uncertainty quantification.** The reason SLAM does not end as a simple pose estimator is that downstream systems — path planning, obstacle avoidance — demand the covariance of the localization estimate. EKF and factor graphs propagate covariance naturally. Bayesian PoseNet tried to estimate variance through dropout, but it was hard to verify whether that variance held a calibrated relationship with actual position error. Especially on inputs outside the training distribution, Bayesian PoseNet returned confident wrong answers instead, which for a robotic system is worse than failing visibly.

---

## 12.4 A record of reflection

Kendall did not look away from this failure. After completing his PhD in 2019, he moved to Wayve and turned toward imitation learning and world model research for autonomous driving. He had not given up on learning-based localization; he had judged that the problem statement "regress absolute pose from a single image" was wrong.

Federico Tombari's group (TU Munich, later Google) attempted [CNN-SLAM (Tateno et al. 2017)](https://arxiv.org/abs/1704.03489) around the same time. The approach fused dense depth predicted by a CNN with the depth measurement of direct monocular SLAM. It was not fully end-to-end in the sense that the learned part was confined to dense depth, but it was one branch of the hope that "maybe CNNs can solve the scale and low-texture problems of monocular SLAM." The results were uneven across scenes, and accuracy did not consistently come out ahead.

> 📜 **Prediction vs. outcome.** Kendall, in the PoseNet paper (2015), pointed to uncertainty estimation, temporal integration, and extension to larger-scale scenes as the next tasks. All three directions were pursued — Bayesian PoseNet (2016), LSTM PoseNet (2016), multiple outdoor extension experiments. But each attempt hit a new wall, and researchers in the end abandoned the whole approach. A reasonable prediction means nothing when the platform itself is wrong. `[abandoned]`

Some attempts survived in other directions. SfMLearner's photometric self-supervision was absorbed into the training strategy of MonoDepth2 (Godard 2019) and further into DROID-SLAM (Teed & Deng 2021). The LSTM-based temporal modeling DeepVO demonstrated reappeared in a modified form in visual-inertial learning research. The ideas did not disappear; their use changed.

> 📜 **Prediction vs. outcome.** Zhou, in the SfMLearner paper (2017), flagged dynamic object handling and robustness to photometric noise as remaining tasks. Follow-up self-supervised work including [GeoNet (Yin & Shi 2018)](https://arxiv.org/abs/1803.02276) partially pushed in this direction. But the path of self-supervised VO alone replacing SLAM never joined the mainstream. Photometric self-supervision carried its lineage forward, but the goal of end-to-end VO was rejected by the field. `[diverted]`

---

## 12.5 Settling of the lesson

Around 2020 the field reached a consensus. "Geometry as algorithm, learning as feature and prior" — roughly along those lines.

> 🔗 **Borrowed.** This principle's realization takes shape in Chapter 13's CodeSLAM (Bloesch 2018) and DROID-SLAM (Teed & Deng 2021), both of which keep the geometric skeleton of factor graph or bundle adjustment and confine the learned part to feature extraction or depth prior formation — the skeleton PoseNet threw out turned out not to be discardable.

The classical pipeline was not uniformly superior to the learning-based alternatives. ORB-SLAM too often failed in textureless environments, at night, in the rain. The problem was not the sturdiness of classical SLAM, but that end-to-end's errors were more opaque and more unpredictable.

The failure was not a problem of dataset or architecture. The path running straight from image to pose was missing thirty years of geometric knowledge.

---

## 🧭 Still open

**Which inductive bias to inject, and how.** The principle "geometry as algorithm" is right, but which geometry at which level should be coded is still an open question. Rigid body motion? Epipolar constraint? In the foundation model era this boundary is blurring again. GaussianSLAM and 3DGS-based systems are experimenting with ways to dissolve geometry into the learned representation.

**Calibration of learned uncertainty.** Even after Bayesian PoseNet's failure this problem is unsolved. Whether deep learning-based uncertainty estimates hold a calibrated relationship with actual error — especially on out-of-distribution inputs — remains open as of 2026. Autonomous driving is applying practical pressure on this question.

**Redefining "end-to-end."** The end-to-end as PoseNet defined it (image → pose, by learning alone) failed. But since the arrival of foundation models in 2023, the meaning of end-to-end is shifting. Which modules of SLAM to fill with learning and which to keep as algorithm — that dividing line itself is being renegotiated.

The principle "geometry as algorithm, learning as feature" settled in this period. In 2018, out of Andrew Davison's lab at Imperial College London in Kensington, came the first substantive implementation of that principle: CodeSLAM.
