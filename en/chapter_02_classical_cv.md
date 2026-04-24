# Ch.2 — The Classical CV Toolbox: Harris to SIFT, and on to ORB

Bundle adjustment requires "corresponding points" — the same physical location found independently in two or more images. The surveyor planted targets in the field by hand; computer vision had to hand that role off to an algorithm. Feature detection and description is the problem that started there.

In the late 1970s, Hans Moravec tried to locate salient points in the environment with a camera on the Stanford Cart project. The work was written up in his 1980 Stanford doctoral thesis, ["Obstacle Avoidance and Navigation in the Real World by a Seeing Robot Rover"](https://frc.ri.cmu.edu/~hpm/project.archive/robot.papers/1975.cart/1980.html.thesis/index.html). The intuition that texture-rich corners are good to track was there, but no mathematical definition. Eleven years later, Chris Harris and Mike Stephens formalized that intuition as eigenvalues of the autocorrelation matrix. Lucas and Kanade had laid down the frame for pixel tracking seven years earlier. Lowe absorbed both ideas and built a descriptor invariant to scale and rotation. Rublee did the same thing faster and without a patent. The SLAM front-end runs on top of this lineage.

---

## 2.1 The idea of a corner: from Moravec to Harris

A point where an image patch changes a lot under a small camera motion is called a "corner". Moravec's (1977) criterion was simple. If the Sum of Squared Differences (SSD) against neighbor pixels is large in every direction — up, down, left, right — the point counts as a corner.

Harris and Stephens replaced this with continuous differentiation at the 1988 Alvey Vision Conference in ["A Combined Corner and Edge Detector"](https://www.bmva.org/bmvc/1988/avc-88-023.html). For image $I$, shifting a window $W$ around point $(x,y)$ by $(\Delta x, \Delta y)$ and approximating the intensity change gives:

$$M = \sum_{(x,y) \in W} \begin{pmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{pmatrix}$$

The two eigenvalues $\lambda_1, \lambda_2$ of $M$ classify the point: both large indicates a corner, one large an edge, and both small a flat region. Harris avoided the eigenvalue decomposition altogether by using the score $R = \det(M) - k \cdot \text{tr}(M)^2$. $k$ is typically 0.04–0.06.

> 🔗 **Borrowed.** Harris's (1988) autocorrelation-matrix idea refined Moravec's (1977) SSD-based corner search through continuous differentiation. The prototype of the concept was in the Stanford Cart report.

In 1994, Jianbo Shi and Carlo Tomasi showed in ["Good Features to Track"](https://cecas.clemson.edu/~stb/klt/shi-tomasi-good-features-cvpr1994.pdf) (CVPR 1994) that using $\min(\lambda_1, \lambda_2)$ directly, in place of the Harris score, is more stable for optical flow tracking. This criterion is the Shi-Tomasi corner detector. OpenCV implemented it as the `goodFeaturesToTrack` function. Thirty years later, that function is still the same.

---

## 2.2 The archetype of tracking: Lucas-Kanade and KLT

Harris's matrix $M$ finds the point. Finding the same point again in the next frame is a separate problem. Bruce Lucas and Takeo Kanade, in the 1981 paper ["An Iterative Image Registration Technique"](https://www.ijcai.org/Proceedings/81-2/Papers/017.pdf), formulated inter-frame pixel motion as a minimization problem under the brightness constancy assumption.

Brightness constancy assumption: the intensity of pixel $(x,y)$ is the same before and after the motion.

$$I(x, y, t) = I(x + u, y + v, t + 1)$$

A Taylor expansion followed by linearization gives:

$$I_x u + I_y v + I_t = 0$$

One equation, two unknowns. Lucas-Kanade adds the assumption that pixels inside a $3\times3$ or $5\times5$ window move with the same $(u,v)$, producing an overdetermined system solved by least squares.

$$\begin{pmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{pmatrix} \begin{pmatrix} u \\ v \end{pmatrix} = -\begin{pmatrix} \sum I_x I_t \\ \sum I_y I_t \end{pmatrix}$$

The matrix on the left is the same structure matrix $M$ as Harris's. Corner detection and optical flow sit on the same math.

Tomasi and Kanade, in the 1991 tech report ["Detection and Tracking of Point Features"](https://cecas.clemson.edu/~stb/klt/tomasi-kanade-techreport-1991.pdf), gave a concrete implementation that selects tracking-window quality by the eigenvalue criterion and refines displacement through Newton-Raphson iteration. Bouguet (Intel, 2000) later added an image-pyramid-based coarse-to-fine strategy so the tracker would converge under large motion, and this combination settled into the KLT (Kanade-Lucas-Tomasi) tracker. Real-time VIO systems like [VINS-Mono](https://arxiv.org/abs/1708.03852) (2018) still run a front-end from this lineage. A least-squares tracker from 1981 runs inside the VIO of a smartphone drone forty-odd years later.

> 🔗 **Borrowed.** Lucas-Kanade (1981) → KLT tracker → Qin et al.'s VINS-Mono (2018): a 38-year-old optical flow survives unchanged as the feature-tracking backbone of real-time VIO.

---

## 2.3 SIFT — invariance and the patent

KLT fits the case of a single camera moving a little at a time. Connecting the same point across images taken by different cameras, on different days, is a different order of problem. A change in viewpoint alters the patch's shape, size, and orientation for the same point, and a plain pixel comparison no longer works. That is why a **descriptor** is needed.

David Lowe (UBC) presented the idea at ICCV 1999. The talk was titled "Object Recognition from Local Scale-Invariant Features", and the demo compared 128-dimensional vectors to match the same object across different photographs. Five years later, in 2004, the complete version, ["Distinctive Image Features from Scale-Invariant Keypoints"](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf), appeared in IJCV, and this is the paper cited today as SIFT. SIFT (Scale-Invariant Feature Transform) runs in two stages.

**Detection stage.** Compute DoG (Difference of Gaussians) at several scales and select local extrema as keypoints. DoG is an approximation of the Laplacian of Gaussian. With $L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)$ as the Gaussian-smoothed image:

$$D(x, y, \sigma) = L(x, y, k\sigma) - L(x, y, \sigma)$$

Here $k$ is the ratio between adjacent scales (typically $2^{1/s}$, where $s$ is the number of scales per octave). Searching for extrema across multiple octaves makes it possible to detect the same point under scale change.

**Descriptor stage.** A $16\times16$ window around the keypoint is divided into $4\times4$ blocks, and the 8-bin gradient-orientation histogram in each block is concatenated into a 128-dimensional vector. Since the patch is rotated relative to the keypoint's dominant gradient direction, rotation invariance is obtained as well.

The result was a 128-dimensional descriptor robust to scale, rotation, and partial affine deformation. That is why researchers had to use SIFT in the era before KITTI, before SLAM benchmarks existed.

Lowe filed a patent on SIFT in March 2000, and it was granted in March 2004 (US6711293B1, with priority from March 1999). The patent imposed licensing fees for commercial use, and until it expired in March 2020 it was one of the motivations for efforts to replace SIFT.

> 📜 **Prediction vs. outcome.** In "9 Conclusions" of the 2004 SIFT paper, Lowe listed the descriptor's possible extensions as "view matching for 3D reconstruction, motion tracking and segmentation, robot localization, image panorama assembly, epipolar calibration". Most of the directions landed — SfM, SLAM, panoramas, and early vision-based robot localization leaned on SIFT in the late 2000s. In the long-term correspondence problem, however, SIFT's position wobbled after CNNs arrived. After AlexNet in 2012, demand on the object-recognition side shifted to CNNs, and the local-descriptor slot for SLAM was gradually taken over by learned descriptors like SuperPoint and R2D2. The application domains were predicted correctly; the descriptor form diverted. `[partial hit]`

---

## 2.4 SURF — a speed–accuracy compromise

SIFT's 128-dimensional descriptor was accurate but slow. Hundreds of milliseconds per image on the desktop CPUs of the time. Not usable for real-time SLAM. Herbert Bay (ETH Zürich) presented ["SURF: Speeded-Up Robust Features"](https://people.ee.ethz.ch/~surf/eccv06.pdf) at ECCV 2006. Two ideas sit at the core.

Detect keypoints with the *determinant of the Hessian matrix* instead of DoG. Approximate the second Gaussian derivatives with box filters on an integral image to speed up computation. The descriptor is 64-dimensional, half of SIFT's. The neighborhood of the keypoint is split into $4\times4$ subregions, and in each subregion four values from Haar wavelet responses $d_x, d_y$, $(\sum d_x,\, \sum d_y,\, \sum|d_x|,\, \sum|d_y|)$, are concatenated into a $4\times4\times4=64$-dimensional vector. A 128-dimensional extension (SURF-128) exists, but the default is 64-dimensional.

SURF was 3–7 times faster than SIFT. But the accuracy gap between 128 and 64 dimensions remained, and Bay could not avoid a patent either (ETH Zürich patent). SIFT was edged out over speed; SURF was edged out over accuracy and the patent both. What solved both problems at once was ORB.

> 🔗 **Borrowed.** Lowe's (1999/2004) DoG scale-space → Bay's (2006) Hessian integral image: two answers for achieving scale invariance. DoG is theoretically elegant; the Hessian approximation is engineered to be fast.

---

## 2.5 ORB — binary descriptor and release from the patent

In 2011, Ethan Rublee (Willow Garage), Vincent Rabaud, Kurt Konolige, and Gary Bradski presented ["ORB: An Efficient Alternative to SIFT or SURF"](https://www.gwylab.com/download/ORB_2012.pdf) at ICCV. The title is direct. Willow Garage was also the birthplace of ROS. The motive to build a feature that robotics researchers could actually use is spelled out in the title.

ORB combines and improves two existing techniques.

**Detection.** [FAST](https://www.edwardrosten.com/work/rosten_2006_machine.pdf) (Features from Accelerated Segment Test, Rosten & Drummond 2006). Cycles through 16 points around a pixel and declares it a corner if there is a contiguous arc that is sufficiently brighter or darker. More than 10 times faster than SIFT's DoG. ORB adds a Harris score on top of FAST and keeps only the strong responses.

**Descriptor.** [BRIEF](https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf) (Binary Robust Independent Elementary Features, Calonder et al. 2010). Compares the intensities of randomly chosen point pairs in the patch around a keypoint to produce a bit string. 256 bits by default. Matching uses Hamming distance instead of Euclidean distance, so comparison is a single XOR.

BRIEF's weak point was the lack of rotation invariance. Rublee built **rBRIEF (rotated BRIEF)** by rotation-correcting the patch along the direction of the FAST corner's intensity centroid. With orientation estimation in place, BRIEF finally became a descriptor usable in practice.

$$\theta = \text{atan2}(m_{01},\, m_{10}), \quad m_{pq} = \sum_{x,y} x^p y^q I(x,y)$$

Computation was 100 times faster than SIFT, there was no patent, and it was integrated into OpenCV right away. [ORB-SLAM](https://arxiv.org/abs/1502.00956) (Mur-Artal et al. 2015), as the name says, was built on ORB, and the line continued through the trilogy. ORB-SLAM3 had still not changed the front-end as of 2021.

> 🔗 **Borrowed.** Calonder et al.'s (2010) BRIEF → Rublee et al.'s (2011) ORB: adding intensity-centroid-based orientation estimation to a binary descriptor secured rotation invariance.

---

## 2.6 Learned descriptors

If ORB is the practical peak, the next question is natural. Are learned rules better than hand-designed ones? Yi et al.'s 2016 [LIFT](https://arxiv.org/abs/1603.09114) (Learned Invariant Feature Transform, ECCV 2016) tried to replace the three stages — detection, orientation estimation, descriptor — with CNNs. Three separately trained networks wired into a pipeline.

In 2018, DeTone et al.'s [SuperPoint](https://arxiv.org/abs/1712.07629) (CVPRW 2018) trained keypoint detection and a 256-dimensional descriptor jointly, under a self-supervised scheme called homographic adaptation. Pretrained on synthetic data, adapted to real images. The first learned descriptor to catch attention in the SLAM community.

Even so, as of 2026, the traditional descriptors have not disappeared. ORB is faster than SuperPoint on embedded devices, and it behaves more predictably than learned descriptors, which generalize unstably on out-of-domain images. DINOv2-based features have entered place recognition through work like [AnyLoc](https://arxiv.org/abs/2308.00688) (Keetha et al. 2023), but ORB-SLAM3, since its 2021 release, still uses ORB. Moravec's 1977 intuition runs on robots in the 2020s.

---

## 2.7 🧭 Still open

**Generalization limits of learned descriptors.** SuperPoint, R2D2, DISK, and others beat the classical methods inside the training domain, but in new environments (underwater, thermal, low-light) they are inconsistent. There is no consensus on which side is better. The question is still open in 2026.

**Failure modes of wide-baseline matching.** Harris- or ORB-based matching degrades sharply once the camera viewpoint change exceeds 45 degrees. Affine-covariant detectors (ASIFT, MSER) patched part of the gap, but there is no complete solution. [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al. 2023) opened a path by bypassing matching itself, though it is still too early to judge whether this is the end of the descriptor problem or a detour around it.

---

Harris's intuition and Lowe's invariance laid the base, and Rublee's speed optimization dragged it onto the shop floor. The toolbox was complete. These techniques were each designed to work between one or two images. Connecting dozens or hundreds of images simultaneously, in a geometrically consistent way, needed another layer.

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
