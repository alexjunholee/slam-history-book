# Ch.2 — The Classical CV Toolbox: Harris to SIFT, and on to ORB

Bundle adjustment requires "corresponding points," the same physical location found independently in two or more images. The surveyor planted targets in the field by hand; computer vision had to hand that role to an algorithm. Feature detection and description began with that handoff.

In the late 1970s, Hans Moravec tried to locate salient points in the environment with a camera on the Stanford Cart project. The work was written up in his 1980 Stanford doctoral thesis, ["Obstacle Avoidance and Navigation in the Real World by a Seeing Robot Rover"](https://frc.ri.cmu.edu/~hpm/project.archive/robot.papers/1975.cart/1980.html.thesis/index.html). Moravec supplied a quantitative criterion for selecting trackable points from intensity changes in neighboring patches. In 1988, Chris Harris and Mike Stephens formalized that intuition in terms of the eigenvalues of the autocorrelation matrix. Lucas and Kanade had laid down the framework for pixel tracking seven years earlier. Lowe absorbed both ideas and built a descriptor invariant to scale and rotation. Rublee produced a faster, patent-free alternative. The SLAM front end runs on this lineage.

---

## 2.1 The idea of a corner: from Moravec to Harris

A point where an image patch changes substantially under a small camera motion is called a "corner." Moravec's (1977) criterion was simple. If the Sum of Squared Differences (SSD) against neighboring pixels is large in every direction (up, down, left, and right), the point counts as a corner.

Harris and Stephens replaced this with continuous differentiation at the 1988 Alvey Vision Conference in ["A Combined Corner and Edge Detector"](https://www.bmva.org/bmvc/1988/avc-88-023.html). For image $I$, shifting a window $W$ around point $(x,y)$ by $(\Delta x, \Delta y)$ and approximating the intensity change gives:

$$M = \sum_{(x,y) \in W} \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}$$

The two eigenvalues $\lambda_1, \lambda_2$ of $M$ classify the point: two large eigenvalues indicate a corner, one large eigenvalue indicates an edge, and two small eigenvalues indicate a flat region. Harris avoided the eigenvalue decomposition altogether by using the score $R = \det(M) - k \cdot \text{tr}(M)^2$. $k$ is typically 0.04–0.06.

> 🔗 **Borrowed.** Harris's (1988) autocorrelation-matrix idea refined Moravec's (1977) SSD-based corner search through continuous differentiation. The prototype of the concept was in the Stanford Cart report.

In 1994, Jianbo Shi and Carlo Tomasi showed in ["Good Features to Track"](https://cecas.clemson.edu/~stb/klt/shi-tomasi-good-features-cvpr1994.pdf) (CVPR 1994) that using $\min(\lambda_1, \lambda_2)$ directly, in place of the Harris score, is more stable for optical-flow tracking. This criterion is the Shi-Tomasi corner detector. Thirty years later, OpenCV still exposes it under the same `goodFeaturesToTrack` function name.

---

## 2.2 The archetype of tracking: Lucas-Kanade and KLT

Harris's matrix $M$ finds the point. Finding the same point again in the next frame is a separate problem. Bruce Lucas and Takeo Kanade, in the 1981 paper ["An Iterative Image Registration Technique"](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf), formulated inter-frame pixel motion as a minimization problem under the brightness constancy assumption.

The brightness-constancy assumption states that the intensity of pixel $(x,y)$ is the same before and after the motion.

$$I(x, y, t) = I(x + u, y + v, t + 1)$$

A Taylor expansion followed by linearization gives:

$$I_x u + I_y v + I_t = 0$$

One equation, two unknowns. Lucas-Kanade adds the assumption that pixels inside a $3\times3$ or $5\times5$ window move with the same $(u,v)$, producing an overdetermined system solved by least squares.

$$\begin{pmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{pmatrix} \begin{pmatrix} u \\ v \end{pmatrix} = -\begin{pmatrix} \sum I_x I_t \\ \sum I_y I_t \end{pmatrix}$$

The matrix on the left is the same structure matrix $M$ as Harris's. Corner detection and optical flow rest on the same mathematics.

Tomasi and Kanade, in the 1991 tech report ["Detection and Tracking of Point Features"](https://cecas.clemson.edu/~stb/klt/tomasi-kanade-techreport-1991.pdf), gave a concrete implementation that selects tracking-window quality by the eigenvalue criterion and refines displacement through Newton-Raphson iteration. Bouguet (Intel, 2000) later added an image-pyramid-based coarse-to-fine strategy so the tracker would converge under large motion, and this combination became the KLT (Kanade-Lucas-Tomasi) tracker. Real-time VIO systems such as [VINS-Mono](https://arxiv.org/abs/1708.03852) (2018) still run a front end descended from this work. A least-squares tracker from 1981 runs inside the VIO of a smartphone-class drone more than forty years later.

> 🔗 **Borrowed.** Lucas-Kanade (1981) → KLT tracker → Qin et al.'s VINS-Mono (2018): optical flow proposed 37 years earlier survives unchanged as the feature-tracking backbone of real-time VIO.

---

## 2.3 SIFT — invariance and the patent

KLT fits the case of a single camera moving a little at a time. Connecting the same point across images taken by different cameras, on different days, is a different problem altogether. A change in viewpoint alters the patch's shape, size, and orientation for the same point, and a plain pixel comparison no longer works. That is why a **descriptor** is needed.

David Lowe (UBC) presented the idea at ICCV 1999. The talk was titled "Object Recognition from Local Scale-Invariant Features," and the demo compared 128-dimensional vectors to match the same object across different photographs. Five years later, Lowe published the full account, ["Distinctive Image Features from Scale-Invariant Keypoints"](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf), in IJCV. It is the paper now cited as SIFT. SIFT (Scale-Invariant Feature Transform) runs in two stages.

**Detection stage.** Compute DoG (Difference of Gaussians) at several scales and select local extrema as keypoints. DoG is an approximation of the Laplacian of Gaussian. Let $L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)$ denote the Gaussian-smoothed image:

$$D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)$$

Here $k$ is the ratio between adjacent scales (typically $2^{1/s}$, where $s$ is the number of scales per octave). Searching for extrema across multiple octaves allows the same point to be detected under a change of scale.

**Descriptor stage.** A $16\times16$ window around the keypoint is divided into $4\times4$ blocks, and the 8-bin gradient-orientation histogram in each block is concatenated into a 128-dimensional vector. Rotating the patch into the keypoint's dominant gradient direction also provides rotation invariance.

The result was a 128-dimensional descriptor robust to scale, rotation, and partial affine deformation. Before KITTI and standardized SLAM benchmarks, that robustness made SIFT useful for matching images across changes in scale and viewpoint.

Lowe filed a patent on SIFT in March 2000, and it was granted in March 2004 (US6711293B1, with priority from March 1999). The patent imposed licensing fees for commercial use, and until it expired in March 2020 it was one of the motivations for efforts to replace SIFT.

> 📜 **Prediction vs. outcome.** In "9 Conclusions" of the 2004 SIFT paper, Lowe listed the descriptor's possible extensions as "view matching for 3D reconstruction, motion tracking and segmentation, robot localization, image panorama assembly, epipolar calibration." Most of those predictions proved accurate: SfM, SLAM, panoramas, and early vision-based robot localization leaned on SIFT in the late 2000s. SIFT's position in long-term correspondence weakened after CNNs arrived. Following AlexNet in 2012, object-recognition work shifted to CNNs, and learned descriptors such as SuperPoint and R2D2 gradually took over the local-descriptor role in SLAM. The application domains were predicted correctly; the descriptor technology changed.

---

## 2.4 SURF — a speed–accuracy compromise

SIFT's 128-dimensional descriptor was accurate but slow, taking hundreds of milliseconds per image on the desktop CPUs of the time. It was not usable for real-time SLAM. Herbert Bay (ETH Zürich) presented ["SURF: Speeded-Up Robust Features"](https://people.ee.ethz.ch/~surf/eccv06.pdf) at ECCV 2006. The method relied on two ideas.

SURF detects keypoints with the *determinant of the Hessian matrix* instead of DoG. It approximates the second Gaussian derivatives with box filters on an integral image to speed up computation. The descriptor is 64-dimensional, half of SIFT's. The neighborhood of the keypoint is split into $4\times4$ subregions, and in each subregion four values from Haar wavelet responses $d_x, d_y$, $(\sum d_x,\, \sum d_y,\, \sum|d_x|,\, \sum|d_y|)$, are concatenated into a $4\times4\times4=64$-dimensional vector. A 128-dimensional extension (SURF-128) exists, but the default is 64-dimensional.

SURF was 3–7 times faster than SIFT. But accuracy comparisons depended on the detectors, descriptor designs, and evaluation conditions as well as dimensionality, and Bay could not avoid a patent either (ETH Zürich patent). SIFT lost ground on speed; SURF lost ground on both accuracy and patent restrictions. ORB addressed both problems at once.

> 🔗 **Borrowed.** Lowe's (1999/2004) DoG scale-space → Bay's (2006) Hessian integral image: two answers for achieving scale invariance. DoG is theoretically elegant; the Hessian approximation is engineered to be fast.

---

## 2.5 ORB — binary descriptor and release from the patent

In 2011, Ethan Rublee (Willow Garage), Vincent Rabaud, Kurt Konolige, and Gary Bradski presented ["ORB: An Efficient Alternative to SIFT or SURF"](https://www.gwylab.com/download/ORB_2012.pdf) at ICCV. Willow Garage was also the birthplace of ROS, and the title reflected the goal: a feature that robotics researchers could use in practice.

ORB combines and improves two existing techniques.

**Detection.** [FAST](https://www.edwardrosten.com/work/rosten_2006_machine.pdf) (Features from Accelerated Segment Test, Rosten & Drummond 2006) tests a 16-pixel circle around a candidate and declares it a corner if a contiguous arc is sufficiently brighter or darker. It is more than 10 times faster than SIFT's DoG. ORB adds a Harris score on top of FAST and keeps only the strong responses.

**Descriptor.** [BRIEF](https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf) (Binary Robust Independent Elementary Features, Calonder et al. 2010) compares the intensities of randomly chosen point pairs in the patch around a keypoint to produce a 256-bit string by default. Matching uses Hamming distance instead of Euclidean distance, so the distance is computed by XOR followed by a population count of the differing bits.

BRIEF's weak point was the lack of rotation invariance. Rublee built **rBRIEF (rotated BRIEF)** by rotating the patch to align with the FAST corner's intensity centroid. This supplied the missing orientation invariance.

$$\theta = \text{atan2}(m_{01},\, m_{10}), \quad m_{pq} = \sum_{x,y} x^p y^q I(x,y)$$

ORB ran 100 times faster than SIFT, carried no patent restriction, and entered OpenCV immediately. [ORB-SLAM](https://arxiv.org/abs/1502.00956) (Mur-Artal et al. 2015), as the name indicates, was built on ORB, and the line continued through the trilogy. ORB-SLAM3 still used the same front end in 2021.

> 🔗 **Borrowed.** Calonder et al.'s (2010) BRIEF → Rublee et al.'s (2011) ORB: adding intensity-centroid-based orientation estimation to a binary descriptor secured rotation invariance.

---

## 2.6 Learned descriptors

If ORB is the practical peak, the next question follows: are learned features better than hand-designed ones? Yi et al.'s 2016 [LIFT](https://arxiv.org/abs/1603.09114) (Learned Invariant Feature Transform, ECCV 2016) tried to replace the three stages of detection, orientation estimation, and description with CNNs. It connected three separately trained networks in a pipeline.

In 2018, DeTone et al.'s [SuperPoint](https://arxiv.org/abs/1712.07629) (CVPRW 2018) trained keypoint detection and a 256-dimensional descriptor jointly under a self-supervised scheme called homographic adaptation. It was pretrained on synthetic data and adapted to real images, and later became one of the learned local features widely tested in SLAM research.

Even so, traditional descriptors have not disappeared as of 2026. ORB is faster than SuperPoint on embedded devices, and its behavior is more predictable than that of learned descriptors, which can generalize unpredictably on out-of-domain images. DINOv2-based features have entered place recognition through systems such as [AnyLoc](https://arxiv.org/abs/2308.00688) (Keetha et al. 2023), but ORB-SLAM3 has used ORB since its 2021 release. Moravec's 1977 intuition still runs on robots in the 2020s.

---

## 2.7 🧭 Still open

**Generalization limits of learned descriptors.** SuperPoint, R2D2, DISK, and others outperform classical methods inside the training domain, but behave inconsistently in new environments (underwater, thermal, low-light). There is no consensus on which family is more reliable. The question remains open in 2026.

**Failure modes of wide-baseline matching.** Harris- or ORB-based matching can degrade under large viewpoint changes; the extent depends on the scene, rotation axis, and matching conditions. Affine-covariant detectors (ASIFT, MSER) patched part of the gap, but there is no complete solution. [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al. 2023) opened a path by bypassing matching itself, though it is still too early to judge whether this is the end of the descriptor problem or a detour around it.

---

Harris's intuition and Lowe's invariance supplied the foundation, and Rublee's speed optimization made it practical. Together they formed a usable toolbox. Each technique was designed to operate on one or two images. Connecting dozens or hundreds of images simultaneously and with geometric consistency required another layer.

---

*References*

- Harris, C. & Stephens, M. (1988). A Combined Corner and Edge Detector. *Proc. Alvey Vision Conference*.
- Lucas, B. D. & Kanade, T. (1981). An Iterative Image Registration Technique with an Application to Stereo Vision. *IJCAI*.
- Shi, J. & Tomasi, C. (1994). Good Features to Track. *CVPR*.
- [Lowe, D. G. (2004). Distinctive Image Features from Scale-Invariant Keypoints.](https://doi.org/10.1023/B:VISI.0000029664.99615.94) *IJCV 60(2)*.
- Bay, H., Tuytelaars, T. & Van Gool, L. (2006). SURF: Speeded-Up Robust Features. *ECCV*.
- Calonder, M. et al. (2010). BRIEF: Binary Robust Independent Elementary Features. *ECCV*.
- [Rublee, E. et al. (2011). ORB: An Efficient Alternative to SIFT or SURF.](https://doi.org/10.1109/ICCV.2011.6126544) *ICCV*.
- DeTone, D., Malisiewicz, T. & Rabinovich, A. (2018). SuperPoint: Self-Supervised Interest Point Detection and Description. *CVPRW*. [arXiv:1712.07629](https://arxiv.org/abs/1712.07629)
