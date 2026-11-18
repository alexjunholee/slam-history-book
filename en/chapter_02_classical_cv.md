# Chapter 2. Classical Features: From Harris to ORB

Bundle adjustment depends on "corresponding points"—the same physical locations identified reliably across two or more images. In field surveying, humans planted physical targets to create those points; in computer vision, algorithms had to take over that job. Feature detection and description grew directly out of that handoff.

The lineage began in the late 1970s with Hans Moravec's work on the Stanford Cart project, documented in his 1980 Stanford doctoral thesis, ["Obstacle Avoidance and Navigation in the Real World by a Seeing Robot Rover"](https://frc.ri.cmu.edu/~hpm/project.archive/robot.papers/1975.cart/1980.html.thesis/index.html). Moravec offered a concrete rule for picking trackable points by measuring how much image intensity changed across neighboring patches. In 1988, Chris Harris and Mike Stephens formalized that intuition using the eigenvalues of an autocorrelation matrix, while Bruce Lucas and Takeo Kanade had already established the pixel-tracking framework seven years earlier. Later, David Lowe united detection and description in a form invariant to scale and rotation, and Ethan Rublee built a faster, patent-free binary alternative. Today's SLAM front ends still run on this sequence of ideas.

---

## 2.1 The idea of a corner: from Moravec to Harris

In computer vision, a "corner" is any point where a small image patch changes sharply regardless of which direction the camera nudges it. Sliding a patch across a flat wall produces little change, and sliding along a straight edge leaves the pattern unaltered along the edge itself; only a corner or distinct texture reveals motion in every direction. Moravec's (1977) detector captured this idea with a simple rule: shift a window one pixel in each cardinal and diagonal direction—up, down, left, and right—and if the minimum Sum of Squared Differences (SSD) against neighboring patches stays large across all directions, mark the point as a corner.

At the 1988 Alvey Vision Conference, Chris Harris and Mike Stephens replaced Moravec's discrete directional shifts with continuous differentiation in ["A Combined Corner and Edge Detector"](https://www.bmva.org/bmvc/1988/avc-88-023.html). Shifting a window $W$ around pixel $(x,y)$ in image $I$ by an arbitrary small displacement $(\Delta x, \Delta y)$ and expanding the intensity change yields a $2\times2$ structure matrix:

$$M = \sum_{(x,y) \in W} \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}$$

Inspecting the two eigenvalues $\lambda_1, \lambda_2$ of $M$ immediately classifies the local geometry: two large eigenvalues signal a corner, a single large eigenvalue indicates an edge, and two small eigenvalues mark a flat region. To spare early hardware the cost of explicit eigenvalue decomposition, Harris introduced the response score $R = \det(M) - k \cdot \text{tr}(M)^2$, where $k$ is typically chosen between 0.04 and 0.06.

> 🔗 **Borrowed.** Harris's (1988) autocorrelation-matrix formulation generalized Moravec's (1977) SSD-based corner search into continuous image derivatives, building directly on the prototype laid out in the Stanford Cart report.

Six years later, Jianbo Shi and Carlo Tomasi showed in ["Good Features to Track"](https://cecas.clemson.edu/~stb/klt/shi-tomasi-good-features-cvpr1994.pdf) (CVPR 1994) that thresholding $\min(\lambda_1, \lambda_2)$ directly—instead of relying on the Harris response score—yields features that stay far more stable during optical-flow tracking. Known as the Shi-Tomasi corner detector, that criterion remains so practical that OpenCV still exposes it three decades later under the name `goodFeaturesToTrack`.

---

## 2.2 The archetype of tracking: Lucas-Kanade and KLT

Finding a distinctive corner in one frame is only half the task; the system still has to locate that exact same point in the next frame. In their 1981 paper ["An Iterative Image Registration Technique"](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf), Bruce Lucas and Takeo Kanade formulated frame-to-frame pixel motion as an optimization problem resting on the brightness constancy assumption—the premise that a moving surface point keeps the same intensity $(x,y)$ from one moment to the next:

$$I(x, y, t) = I(x + u, y + v, t + 1)$$

Applying a first-order Taylor expansion for small displacements $(u, v)$ linearizes this relation into the classic optical-flow constraint:

$$I_x u + I_y v + I_t = 0$$

Because a single equation cannot pin down two unknowns, Lucas and Kanade assumed that all pixels inside a small $3\times3$ or $5\times5$ window move together with the same velocity $(u,v)$, turning the problem into an overdetermined linear least-squares system:

$$\begin{pmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{pmatrix} \begin{pmatrix} u \\ v \end{pmatrix} = -\begin{pmatrix} \sum I_x I_t \\ \sum I_y I_t \end{pmatrix}$$

Notably, the matrix on the left is the exact same structure matrix $M$ that Harris used to find corners. Both corner selection and optical-flow tracking turn on the conditioning of the very same matrix.

In their 1991 technical report ["Detection and Tracking of Point Features"](https://cecas.clemson.edu/~stb/klt/tomasi-kanade-techreport-1991.pdf), Tomasi and Kanade turned this equation into an end-to-end tracker that screens windows by their eigenvalues and refines displacement via Newton-Raphson iteration. Jean-Yves Bouguet (Intel, 2000) later added a coarse-to-fine image pyramid so the solver could converge even across large frame-to-frame motions, completing what became known as the **KLT (Kanade-Lucas-Tomasi) tracker**. More than forty years after Lucas and Kanade's paper, real-time visual-inertial odometry pipelines such as [VINS-Mono](https://arxiv.org/abs/1708.03852) (2018) still rely on a KLT-descended front end.

> 🔗 **Borrowed.** Lucas-Kanade (1981) → KLT tracker → Qin et al.'s VINS-Mono (2018): the optical-flow formulation proposed 37 years earlier endures as the feature-tracking backbone of modern real-time VIO.

---

## 2.3 SIFT — invariance and the patent

KLT excels when a single camera glides smoothly from one frame to the next, but matching the same physical point across images captured by different cameras, on different days, or from sharply different angles is a much harder problem. Wide viewpoint changes warp, scale, and rotate the surrounding patch until raw pixel comparisons fail altogether. Overcoming that barrier called for a compact numerical **descriptor** that stays recognizable even when the viewpoint shifts.

David Lowe (UBC) introduced that idea at ICCV 1999 in a talk titled "Object Recognition from Local Scale-Invariant Features," matching objects across photographs using 128-dimensional vectors. Five years later, he published the definitive IJCV paper, ["Distinctive Image Features from Scale-Invariant Keypoints"](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf), cementing **SIFT (Scale-Invariant Feature Transform)** as a cornerstone of computer vision. SIFT operates in two stages:

**Detection stage.** To locate keypoints consistently across changes in camera distance, SIFT computes the Difference of Gaussians (DoG)—a fast approximation to the Laplacian of Gaussian—across multiple image scales and selects local extrema across both space and scale. Letting $L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)$ denote the Gaussian-smoothed image, the DoG response between adjacent scales is:

$$D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)$$

In this expression, $k$ denotes the scale multiplier between adjacent levels (typically $2^{1/s}$, where $s$ is the number of scales per octave). Searching across multiple octaves ensures that the same physical point is recovered even as it grows larger or smaller in the frame.

**Descriptor stage.** Once a keypoint is detected, SIFT assigns a canonical orientation from its dominant local gradient direction so that subsequent measurements remain rotation-invariant. It then divides a $16\times16$ window around the keypoint into $4\times4$ sub-blocks, computes an 8-bin gradient-orientation histogram in each block, and concatenates the histograms into a 128-dimensional vector.

The resulting 128-dimensional descriptor proved remarkably resilient to changes in scale, rotation, and partial affine distortion. Long before KITTI or standardized SLAM benchmarks existed, that robustness made SIFT the default engine for matching photographs across wide viewpoint changes.

Yet SIFT also carried a legal constraint. Lowe filed a patent in March 2000 that was granted in March 2004 (US6711293B1, claiming priority from March 1999). Until the patent expired in March 2020, commercial deployments faced licensing fees—a barrier that spurred a race to invent faster, patent-free alternatives.

> 📜 **Prediction vs. outcome.** In "9 Conclusions" of the 2004 SIFT paper, Lowe forecast applications spanning "view matching for 3D reconstruction, motion tracking and segmentation, robot localization, image panorama assembly, epipolar calibration." Those predictions largely came true: SfM, SLAM, panorama stitching, and early visual robot localization all leaned heavily on SIFT in the late 2000s. After AlexNet arrived in 2012, however, object recognition migrated to CNNs, and learned local descriptors such as SuperPoint and R2D2 gradually challenged SIFT in SLAM. Lowe anticipated the applications accurately; what changed was the underlying descriptor technology.

---

## 2.4 SURF — a speed–accuracy compromise

For all its accuracy, SIFT was far too slow for real-time robotics, consuming hundreds of milliseconds per frame on desktop CPUs of the mid-2000s. At ECCV 2006, Herbert Bay (ETH Zürich) and colleagues offered a faster alternative in ["SURF: Speeded-Up Robust Features"](https://people.ee.ethz.ch/~surf/eccv06.pdf), built around two speedups.

First, instead of building a DoG scale pyramid, SURF detects keypoints using the *determinant of the Hessian matrix*, approximating second-order Gaussian derivatives with simple box filters evaluated in constant time over an integral image. Second, it halves the descriptor size to 64 dimensions by splitting the keypoint neighborhood into $4\times4$ subregions and summarizing Haar wavelet responses $d_x, d_y$ in each subregion with four sums—$(\sum d_x,\, \sum d_y,\, \sum|d_x|,\, \sum|d_y|)$—to form a $4\times4\times4=64$-dimensional vector (alongside an optional 128-dimensional SURF-128 variant).

SURF ran 3–7 times faster than SIFT, though relative accuracy varied with the detector, descriptor configuration, and test conditions—and Bay's method was likewise encumbered by an ETH Zürich patent. Where SIFT fell behind on speed, SURF struggled with both accuracy trade-offs and licensing restrictions, leaving an opening that ORB would soon fill.

> 🔗 **Borrowed.** Lowe's (1999/2004) DoG scale-space → Bay's (2006) Hessian integral image: two distinct engineering answers to scale invariance, trading DoG's theoretical elegance for the raw speed of box-filter approximations.

---

## 2.5 ORB — binary descriptor and release from the patent

What robotics needed was a feature pipeline orders of magnitude faster than SIFT, resilient to rotation and scale shifts, and completely free of patent restrictions. At ICCV 2011, Ethan Rublee, Vincent Rabaud, Kurt Konolige, and Gary Bradski—working at Willow Garage, the birthplace of ROS—delivered that tool in ["ORB: An Efficient Alternative to SIFT or SURF"](https://www.gwylab.com/download/ORB_2012.pdf).

Rather than inventing everything anew, ORB paired and upgraded two proven lightweight components:

**Detection.** For keypoint localization, ORB adopted [FAST](https://www.edwardrosten.com/work/rosten_2006_machine.pdf) (Features from Accelerated Segment Test, Rosten & Drummond 2006), which inspects a 16-pixel circle around each candidate pixel and flags a corner whenever a contiguous arc is noticeably brighter or darker than the center. More than 10 times faster than SIFT's DoG, FAST nonetheless fires along edges as well as true corners, so ORB ranks candidates with a Harris score to retain only the strongest corners.

**Descriptor.** For description, ORB built on [BRIEF](https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf) (Binary Robust Independent Elementary Features, Calonder et al. 2010), which compares intensities at pairs of points inside a patch to pack a 256-bit binary string by default. Matching bit strings uses Hamming distance—implemented on modern CPUs with a fast XOR followed by a bit-count—rather than floating-point Euclidean distance.

Because raw BRIEF breaks down under image rotation, Rublee and colleagues introduced **rBRIEF (rotated BRIEF)**, computing each FAST corner's orientation from its intensity centroid and rotating the sampling patch accordingly before running the binary tests:

$$\theta = \text{atan2}(m_{01},\, m_{10}), \quad m_{pq} = \sum_{x,y} x^p y^q I(x,y)$$

Running roughly 100 times faster than SIFT and completely unencumbered by patents, ORB entered OpenCV immediately and spread across robotics. When Mur-Artal et al. released [ORB-SLAM](https://arxiv.org/abs/1502.00956) in 2015, they named the entire system after the descriptor, and ORB-SLAM3 continued to rely on that same front end in 2021.

> 🔗 **Borrowed.** Calonder et al.'s (2010) BRIEF → Rublee et al.'s (2011) ORB: pairing a binary comparison descriptor with intensity-centroid orientation estimation preserved speed while adding rotation invariance.

---

## 2.6 Learned descriptors

Once ORB reached the practical peak of hand-crafted efficiency, researchers asked whether neural networks trained directly on data could outperform human-designed filters and bit tests. In 2016, Yi et al.'s [LIFT](https://arxiv.org/abs/1603.09114) (Learned Invariant Feature Transform, ECCV 2016) offered an early answer by replacing detection, orientation estimation, and description with three separately trained CNNs linked into a single pipeline.

Two years later, DeTone et al.'s [SuperPoint](https://arxiv.org/abs/1712.07629) (CVPRW 2018) unified keypoint detection and 256-dimensional description inside a single network trained with a self-supervised scheme called homographic adaptation. Pretrained on synthetic geometric shapes and adapted to real photographs, SuperPoint quickly became the benchmark learned local feature in SLAM research. More recently, DINOv2-based representations have entered place recognition through systems like [AnyLoc](https://arxiv.org/abs/2308.00688) (Keetha et al. 2023).

Even so, classical descriptors have not vanished as of 2026. On resource-constrained embedded hardware, ORB remains far faster than SuperPoint, and its failure modes are easier to predict and bound than those of learned models encountering unfamiliar out-of-domain scenes. Since its 2021 release, ORB-SLAM3 has stayed with ORB—meaning Moravec's 1977 corner intuition still steers robots in the 2020s.

---

## Still open

**Generalization limits of learned descriptors.** Learned descriptors such as SuperPoint, R2D2, and DISK routinely outperform classical detectors within their training distribution, yet behave erratically when moved to underwater, thermal, or severe low-light imagery. As of 2026, no consensus settles which family is more dependable across arbitrary environments.

**Failure modes of wide-baseline matching.** Point matchers built on Harris or ORB degrade sharply under large viewpoint changes, depending on scene geometry, rotation axis, and matching thresholds. Affine-covariant detectors such as ASIFT and MSER cover part of that gap without offering a complete fix. More recently, [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al. 2023) opened a different path by bypassing explicit feature matching altogether, though it remains too early to tell whether this marks the end of the descriptor problem or simply a detour around it.

---

Harris's corner intuition, Lowe's scale and rotation invariance, and Rublee's binary speed optimization assembled a remarkably durable front-end toolbox. Yet every technique in that toolbox operates on one image or matches a pair of images at a time. Stitching dozens, hundreds, or thousands of photographs into a single geometrically consistent 3D reconstruction required another layer altogether.

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
