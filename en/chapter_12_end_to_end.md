# Chapter 15. End-to-End Learning: From PoseNet to SfMLearner

Once Eigen demonstrated that a convolutional network could infer metric depth from raw pixels—and self-supervised pipelines soon began extracting geometric signals from unlabeled video—researchers posed an even bolder question: instead of merely predicting depth or recognizing revisited places, could a single neural network perform the entire SLAM task end to end? Between 2015 and 2018, dozens of papers chased that vision, and almost all ran into the same walls.

The wave broke at ICCV 2015 in Santiago de Chile, when Alex Kendall, a doctoral student working with Roberto Cipolla at the Cambridge Computer Laboratory, presented [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336). Trained on Google Street View and campus imagery, PoseNet regressed a 6-DoF camera pose directly from a single photograph—implying that one neural network might replace thirty years of feature extraction, RANSAC matching, bundle adjustment, and map maintenance.

---

## 15.1 PoseNet

Riding the momentum of [AlexNet (Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks), PoseNet repurposed visual representations learned for ImageNet object recognition directly for camera localization.

> 🔗 **Borrowed.** PoseNet takes [GoogLeNet (Inception, Szegedy et al. 2014)](https://arxiv.org/abs/1409.4842) as its backbone, swapping out the softmax classification head for a 7-dimensional regressor predicting 3D translation (x, y, z) and a 4-component orientation quaternion—adapting an ImageNet feature hierarchy straight into a pose estimator.

Evaluated on Kendall's Cambridge Landmarks dataset—spanning King's College Chapel, city streets, and a former hospital—PoseNet achieved roughly 2 m translation error and 5–8° orientation error depending on the scene (§5 of the original paper), running in 5 ms on a single GPU with no feature matching, RANSAC, or map lookup.

Successive papers tackled PoseNet's rough edges. [Bayesian PoseNet (Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909) used Monte Carlo Dropout to produce pose uncertainty estimates; [Walch et al.'s LSTM PoseNet](https://arxiv.org/abs/1611.07890) (released in 2016 and published at ICCV 2017) applied LSTMs over spatial feature maps within a single image rather than across temporal video frames; and [Kendall and Cipolla's 2017 study](https://arxiv.org/abs/1704.00390) replaced hand-tuned translation–rotation loss weights with a geometric reprojection loss comparing 3D scene points projected under predicted versus ground-truth poses (distinct from photometric brightness losses).

Even with those refinements, a stark accuracy gap remained: on the very same Cambridge scenes, classical structure-based methods such as [Active Search (Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf) and DenseVLAD localized within roughly 0.2 m—an order of magnitude sharper than PoseNet's multi-meter errors, exposing the limits of regressing absolute coordinates from an isolated image.

---

## 15.2 DeepVO

To move beyond single-frame absolute regression, Sen Wang and colleagues at Heriot-Watt University turned to temporal image sequences in [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429) (ICRA 2017). A CNN modeled after FlowNet extracted optical-flow cues across consecutive frame pairs, feeding into an LSTM that accumulated motion history to regress relative visual odometry end to end.

> 🔗 **Borrowed.** DeepVO supervises its network against KITTI's GPS/IMU ground-truth trajectories while borrowing its paired-frame feature extractor directly from the optical-flow architecture of [FlowNet (Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852).

By carrying temporal context across frames, the LSTM was meant to curb drift—and on selected KITTI test stretches resembling the training set in driving speed, lighting, and urban layout, DeepVO posted lower drift than DVO-SLAM or VISO2-M. Once deployed outside those familiar motion and appearance regimes, however, that learned temporal memory turned into systematic bias.

That same year, Tinghui Zhou at UC Berkeley bypassed ground-truth pose labels altogether in [Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813), jointly training depth and ego-motion networks via self-supervised photometric view synthesis.

> 🔗 **Borrowed.** Both SfMLearner and [DSO (Engel et al. 2018)](https://arxiv.org/abs/1607.02565) measure brightness discrepancy across geometrically warped pixels—yet DSO optimizes poses and inverse depths directly at test time, whereas SfMLearner uses photometric warping error as a training loss to regress depth and relative motion. SfMLearner's self-supervised objective lived on in monocular depth estimators like MonoDepth2, while DROID-SLAM later retained differentiable geometry under supervised optical-flow and pose training.

Even so, evaluating SfMLearner on short multi-frame VO snippets remained a world apart from running full-trajectory SLAM like ORB-SLAM with long-term mapping and loop closure.

---

## 15.3 Geometry, generalization, and uncertainty

By 2019–2020, critiques from researchers such as [Sattler et al. (2019)](https://arxiv.org/abs/1903.07504)—whose analysis targeted single-image absolute pose regression rather than sequential learned odometry—crystalized three foundational weaknesses across pure end-to-end learning pipelines.

**First: absence of inductive bias.** Classical SLAM builds directly on hard-won mathematical priors: epipolar geometry, SE(3) rigid-body kinematics, projective invariance, and temporal continuity. An unconstrained CNN trained from ImageNet weights had to rediscover 3D metric geometry implicitly from finite data, leaving it unclear whether a network that nailed a benchmark pose truly understood 3D structure or had merely memorized color, texture, and lighting shortcuts.

**Second: generalization beyond the training distribution.** A scene-specific PoseNet memorizes one environment's coordinate frame and cannot transfer to a new city block. Even sequential learned odometry—which estimates relative motion—requires fresh validation whenever camera intrinsics, speed profiles, or visual domains shift, motivating cross-dataset designs like [TartanVO](https://proceedings.mlr.press/v155/wang21h.html) (CoRL 2020). While classical ORB-SLAM can also fail when texture vanishes or exposure spikes, geometric systems expose explicit failure states and recovery routines rather than silently drifting out of distribution.

**Third: validating uncertainty estimates.** Downstream motion planners and obstacle avoidance stacks require calibrated uncertainty alongside pose estimates. EKFs propagate covariance explicitly and factor graphs expose local Hessian approximations (each subject to modeling assumptions), whereas Bayesian PoseNet's dropout variance—though helpful for flagging unfamiliar inputs or rough error trends—offers no reliable calibration guarantee once inputs leave the training distribution.

---

## 15.4 Reassessment

Kendall completed his PhD at Cambridge in 2017 and co-founded Wayve later that year, broadening [his research](https://alexgkendall.com/) from camera pose regression into end-to-end imitation learning and world models for autonomous driving.

Meanwhile, systems like [CNN-SLAM (Tateno, Tombari, Laina, and Navab, 2017)](https://arxiv.org/abs/1704.03489) pointed toward a more pragmatic middle ground: blending CNN-predicted dense depth with direct monocular SLAM measurements, leaning on learned priors in textureless regions while trusting geometric alignment where gradients were strong—and anchoring absolute scale from the network's depth predictions.

> 📜 **Prediction vs. outcome.** PoseNet (2015) outlined three next steps: scaling training data via multiview geometry, adding probabilistic uncertainty, and probing the scene-capacity limits of a single network. Bayesian PoseNet (2016) tackled probabilistic uncertainty, yet Walch et al.'s LSTM PoseNet explored spatial rather than temporal recurrence, leaving PoseNet's core accuracy and generalization gaps unresolved against structure-based localization.

Even as naive pose-regression systems stalled, individual ideas from the 2015–2017 wave survived in redesigned forms: SfMLearner's photometric self-supervision seeded MonoDepth2 (Godard 2019), differentiable geometry powered supervised optimization in DROID-SLAM (Teed & Deng 2021), and recurrent temporal features resurfaced in learned visual-inertial odometry.

> 📜 **Prediction vs. outcome.** In SfMLearner (2017), Zhou flagged moving objects and photometric noise as key open problems. Follow-on self-supervised networks such as [GeoNet (Yin & Shi 2018)](https://arxiv.org/abs/1803.02276) made partial headway by jointly estimating depth, optical flow, and ego-motion while disentangling rigid background from dynamic regions.

---

## 15.5 The lesson that remained

By around 2020, a clear engineering consensus had emerged: keep explicit 3D geometry in the loop, and deploy neural networks where classical heuristics struggle—feature extraction, dense correspondence, and learned priors.

> 🔗 **Borrowed.** CodeSLAM (Bloesch 2018) and DROID-SLAM (Teed & Deng 2021) exemplify this synthesis: both preserve explicit factor-graph or bundle-adjustment optimization while delegating dense depth representation (CodeSLAM) or dense optical-flow updates (DROID-SLAM) to neural networks, restoring the geometric constraints that PoseNet had discarded.

Classical pipelines did not win unconditionally: PoseNet could still produce coarse poses under heavy motion blur or severe lighting shifts where keypoint matchers collapsed. Yet Sattler et al. (2019) showed that across standard accuracy benchmarks, pure absolute pose regression fell far short of structure-based localization—turning on whether a task prized graceful degradation or centimeter precision.

The decisive boundary lay between *regressing pose directly from raw appearance* and *solving for pose from learned correspondences under explicit geometric constraints*. CodeSLAM and DROID-SLAM committed firmly to the latter.

---

## Still open

**Which inductive bias to inject, and how.** Even when geometry stays inside the system, it remains unsettled which constraints—from rigid-body motion to epipolar geometry—should be hard-coded versus learned, and at which layer. Foundation models and 3DGS-based SLAM systems continue shifting that boundary, testing how much geometry can live directly inside a learned scene representation.

**Calibration of learned uncertainty.** Bayesian PoseNet left uncertainty calibration unfinished. Ensuring that neural uncertainty estimates track true geometric error reliably—especially under out-of-distribution shifts critical to autonomous driving—remained unresolved as of 2026.

**Redefining "end-to-end."** PoseNet equated "end-to-end" with a single feedforward mapping from image to pose. Since the arrival of 3D foundation models around 2023, the label itself has shifted, reopening the debate over which SLAM modules belong inside a differentiable network and which should stay explicit algorithms.

---

The hard lessons of 2015–2018 gave birth to a new hybrid paradigm: pair learned representations with classical bundle adjustment rather than replacing geometry outright. Released by Andrew Davison's lab at Imperial College London in 2018, CodeSLAM embodied that exact division of labor—launching the hybrid synthesis examined next in Chapter 16.