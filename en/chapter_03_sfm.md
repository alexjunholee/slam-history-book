# Ch.3 — Structure from Motion: From Longuet-Higgins to COLMAP

While Harris and Lowe were sharpening how to pick out the "points worth looking at" inside a single image, a different lineage asked what could be known when those points were captured in two photographs at once. *Detecting* features and *reconstructing space* from features developed side by side through the same years, and only in the mid-2000s did they merge into one pipeline.

In 1981, H.C. Longuet-Higgins, a theoretical psychologist at Cambridge, published a three-page paper in *Nature*. The title was "[A Computer Algorithm for Reconstructing a Scene from Two Projections](https://cseweb.ucsd.edu/classes/fa01/cse291/hclh/SceneReconstruction.pdf)". He showed that from eight pairs of coordinates for the same points captured in two photographs, one could simultaneously solve for how the camera had moved and what shape the scene took in three dimensions. He was neither a roboticist nor a computer vision researcher. Structure from Motion (SfM) began in those three pages, and the mathematics became engineering only in 2016, when Johannes Schönberger released COLMAP.

---

## 3.1 Essential Matrix and the 8-point Algorithm

Longuet-Higgins started from a simple point. When two cameras capture the same point, an algebraic constraint holds between the pair of image coordinates. Once the coordinate system is normalized, this constraint collapses into a single matrix. He defined it as the **essential matrix** $\mathbf{E}$.

Let the two camera centers be $\mathbf{O}_1$ and $\mathbf{O}_2$, and let the corresponding points in normalized coordinates be $\mathbf{x}_1$ and $\mathbf{x}_2$. The constraint is:

$$\mathbf{x}_2^\top \mathbf{E} \mathbf{x}_1 = 0$$

$\mathbf{E}$ factors through the rotation $\mathbf{R}$ and translation $\mathbf{t}$ between the cameras as $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$, where $[\mathbf{t}]_\times$ is the skew-symmetric matrix of $\mathbf{t}$.

Once scale ambiguity is removed, the essential matrix has five degrees of freedom. But before the non-linear 5-point algorithm ([Nistér 2004](http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/2004-PAMI-David%20Nister-An%20Efficient%20Solution%20to%20the%20Five-Point%20Relative%20Pose%20Problem.pdf)) that solved it with five correspondences, the standard approach was to fix one of the nine matrix entries as unit scale, treat the remaining eight as unknowns, and solve a linear system from eight correspondences — before enforcing the rank-2 constraint and unit-scale constraint. This is the **8-point algorithm**. Longuet-Higgins himself gave a procedure that produced a unique solution from exactly eight points. The implementation was simple, and the computational cost was small.

The problem was numerical stability. When image coordinates run in the hundreds or thousands of pixels, the magnitudes of the coefficient matrix entries diverge sharply, and the SVD becomes unstable.

> 🔗 **Borrowed.** Hartley's 1997 normalized 8-point algorithm ([In Defense of the Eight-Point Algorithm](https://www.cse.unr.edu/~bebis/CS485/Handouts/hartley.pdf)) applied a linear transform to image coordinates so that their mean was zero and their average distance was $\sqrt{2}$, and then estimated the essential matrix. The geometry of Longuet-Higgins was left untouched; only the numerical conditioning was fixed. Every textbook afterward adopted this normalized version as the standard.

The fundamental matrix $\mathbf{F}$ generalizes the essential matrix. Even without knowing the camera intrinsics $\mathbf{K}$, the relation $\mathbf{x}_2^\top \mathbf{F} \mathbf{x}_1 = 0$ holds. With intrinsics $\mathbf{K}_1$, $\mathbf{K}_2$ for the two cameras, the relationship is $\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \mathbf{K}_1^{-1}$. For images from the same camera ($\mathbf{K}_1 = \mathbf{K}_2 = \mathbf{K}$) it simplifies to $\mathbf{F} = \mathbf{K}^{-\top} \mathbf{E} \mathbf{K}^{-1}$. In an SfM pipeline, when $\mathbf{K}$ is unknown $\mathbf{F}$ is estimated first; when $\mathbf{K}$ is known, $\mathbf{E}$ is solved directly.

---

## 3.2 Tomasi-Kanade Factorization

For ten years after 1981, SfM was studied mostly as the geometry between two photographs. Processing many photographs at once was a separate problem. Its outline came into view in 1992, when Carlo Tomasi and Takeo Kanade at CMU published the **[factorization method](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/TomasiC_Shape%20and%20motion%20from%20image%20streams%20under%20orthography.pdf)**.

The idea runs as follows. Given $F$ frames observing $P$ points, the image coordinates stack into a $2F \times P$ matrix $\mathbf{W}$. Each entry $w_{fp}$ is the coordinate of point $p$ in frame $f$. Under an orthographic (scaled orthographic) camera model, $\mathbf{W}$ is a rank-3 matrix. The original paper (Tomasi & Kanade 1992) started from exactly this assumption. Then:

$$\mathbf{W} = \mathbf{M} \mathbf{S}$$

where $\mathbf{M}$ is a $2F \times 3$ motion matrix and $\mathbf{S}$ is a $3 \times P$ structure matrix. Keeping only the top three singular values of $\mathbf{W}$ through SVD gives $\mathbf{M}$ and $\mathbf{S}$ at once.

The heart of the method was that a single SVD estimated the motion of every frame and the 3D position of every point together. The computational complexity was a light $O(F \cdot P)$, and the implementation was easy.

> 🔗 **Borrowed.** Nistér, Naroditsky, and Bergen's 2004 CVPR paper "Visual Odometry" is cited in the later literature as redirecting real-time ego-motion estimation into an applied branch of this lineage. Instead of using Tomasi-Kanade's batch factorization directly, the work moved toward solving the relative pose between frames inside a short window, and it stands as an early point of the shift that traded batch accuracy for latency.

The limitation sat in the orthographic/affine assumption. An affine camera ignores perspective distortion. The model holds only when the depth variation in the scene is small compared to the distance to the camera — that is, for distant, small objects. For close scenes, wide-angle lenses, or scenes with large foreground–background depth differences, the error grew. From the late 1990s, extensions to the perspective camera were attempted from several directions, and these efforts led to the rediscovery of bundle adjustment.

---

## 3.3 Hartley & Zisserman and the canonization

If Tomasi-Kanade's factorization framed the multiple-view problem, what remained was extension to the perspective camera and tying the scattered mathematics together in a single language.

In 2000, Richard Hartley and Andrew Zisserman's textbook *[Multiple View Geometry in Computer Vision](https://www.robots.ox.ac.uk/~vgg/hzbook/)* appeared. 680 pages. It consolidated the SfM mathematics scattered from 1981 through the 1990s into the language of projective geometry.

Hartley & Zisserman did more than arrange. They derived essential matrix, fundamental matrix, homography, camera calibration, and bundle adjustment all from a single projective-geometry framework. For the first time, it became clear that concepts that had run separately came from the same root.

Bundle adjustment got particular weight in this book. The reprojection-error minimization problem that Triggs et al. (1999) had formally introduced in Ch.1 was placed by Hartley & Zisserman inside the projective-geometry framework, and a *robust cost function* $\rho$ was put on it explicitly. To keep optimization from breaking on real data with outliers mixed in, the error was suppressed through Huber or Cauchy functions. The solver was Levenberg-Marquardt, and the sparsity of the Jacobian was exploited to cut computation.

Most SLAM and visual odometry (VO) papers in the early 2000s cited this textbook as their standard reference. With concept definitions unified through this one book, large-scale applications like Photo Tourism could focus on implementation without redefining the basics.

---

## 3.4 Photo Tourism and Bundler — Internet-scale SfM

In 2006, Noah Snavely, Steven Seitz, and Richard Szeliski published the SIGGRAPH paper "[Photo Tourism](https://doi.org/10.1145/1179352.1141964)". They gathered photographs of tourist sites uploaded to the internet (the Florence Duomo, the Trevi Fountain in Rome) and tried to reconstruct them in 3D.

The setting itself was a challenge. Cameras and weather varied, composition varied, and some of the images were unrelated indoor shots. This was not a systematically captured dataset but thousands of images uploaded in no particular order by thousands of people.

Snavely's pipeline ran in the following order. SIFT feature detection and matching found correspondences between image pairs. RANSAC with the fundamental matrix removed geometrically inconsistent matches. Starting from image pairs with high connectivity, cameras were added one at a time in incremental SfM. Each time a camera was added, bundle adjustment re-optimized the full set of poses and points.

The datasets reported in the paper included the Notre Dame Cathedral (597 registered out of 2,635 candidates), the Trevi Fountain in Rome (360 out of 466), Yosemite Half Dome (325 out of 1,882), the Great Wall (82 out of 120), and Trafalgar Square (278 out of 1,893), with an average reprojection error of about 1.5 pixels on 1,611×1,128 images. No prior attempt had reconstructed anything at this scale from uncontrolled internet images.

The implementation of this pipeline was Bundler. Snavely released it as open source, and it became the default starting point for SfM researchers.

<!-- DEMO: sfm_incremental.html -->

---

## 3.5 COLMAP — engineering maturity

> 📜 **Prediction vs. outcome.** The "Discussion and future work" section of Snavely et al. 2006 states "Ultimately, we wish to scale up our reconstruction algorithm to handle millions of photographs" and lists better image-registration ordering, lens-distortion modeling, repeated-structure handling, and disconnected-structure reconstruction as remaining problems. Scale-up was taken up by COLMAP (Schönberger 2016) and OpenSfM, reaching tens to hundreds of thousands of images, while real-time and online processing was answered separately by the SLAM lineage — not through incremental refinement, but through fixed-lag smoothers and loop closure. Of the items Snavely listed, scale-up was the one most clearly filled in. `[partial hit]`

In 2016, Johannes Schönberger and Jan-Michael Frahm published the CVPR paper "[Structure-from-Motion Revisited](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)". The "Revisited" in the title was modest, but the paper was a systematic redesign that bundled ten years of improvements since Bundler.

COLMAP differed from Bundler most in three places.

First, the order in which cameras were added. Bundler started from pairs with high connectivity, but had no systematic criterion for which pair to extend first. COLMAP automated the choice of initial image pair and the camera-registration order using triangulation angle, feature-track length, and visibility score. The stability of reconstruction rose substantially.

Second, the bundle-adjustment cadence. Running a full bundle adjustment after every camera addition is expensive. COLMAP alternated local bundle adjustment (optimizing only the recently added camera together with cameras that shared many points with it) and periodic global bundle adjustment.

Third, geometric verification. For each pair of matched feature points, COLMAP ran RANSAC with two models in parallel: fundamental matrix and homography. The fundamental matrix covered general non-planar scenes; the homography covered planar scenes or pure rotation. COLMAP compared the inlier counts of the two models to classify the scene type, and filtered out matches that fit neither. It held up better than Bundler on poor matches and planar-degeneracy situations.

> 🔗 **Borrowed.** COLMAP's incremental bundle adjustment strategy modularized Snavely's Bundler pipeline and added quality control at each stage. The core mathematics of the algorithm (essential matrix estimation, triangulation, Levenberg-Marquardt) came from the Hartley & Zisserman textbook. COLMAP's contribution sat not in new mathematics but in the systematization of engineering judgment.

COLMAP became the de facto standard not only for performance. The codebase was well organized and the documentation was adequate, and CUDA acceleration handled thousands of images within reasonable time. After NeRF appeared in 2020, every NeRF training codebase took COLMAP's output (camera poses + sparse point cloud) as input. 3D Gaussian Splatting did the same. More than an SfM tool, COLMAP became the entryway to 3D reconstruction research.

---

## 3.6 The split between SfM and SLAM

SfM and SLAM use the same mathematics yet solve different problems. The distinction came into sharp relief in the early 2000s.

SfM is *offline*. All images are gathered before processing, so there is no time constraint, and global bundle adjustment can be run multiple times with full access to the whole dataset. If a camera pose was wrong, one can go back and recompute.

SLAM is *online*. Sensor data streams in in real time, and the robot's current position has to come out on the spot. Past data cannot be referenced indefinitely, the computation grows as the map does, and when the robot loops back to a place it first visited, the accumulated drift has to be corrected.

The place where the two fields diverge most is loop closure. In SfM, global bundle adjustment cleans up every inconsistency. In SLAM, the moment a loop closes has to be detected, and the drift at that moment has to be corrected locally. The techniques for this (visual place recognition, pose graph optimization, covisibility-based local optimization) were problems unique to SLAM, with no counterpart in SfM.

Uncertainty propagation differed as well. SLAM tracks the uncertainty of the current pose in real time and updates it with each new observation. A probabilistic representation in the form of EKF or factor graph is needed. In SfM, covariance can be computed after optimization finishes, and real-time tracking is not required.

Davison's [MonoSLAM (2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf) called itself "real-time SfM". But the structure that kept the camera pose and landmarks together in an EKF state vector differed from SfM's global batch. Over the 2000s, the two fields split into independent lineages, each with its own problem setting.

---

## 3.7 🧭 Still open

**SfM with dynamic objects.** Every current SfM system, including COLMAP, assumes a static world. Bundle adjustment is solved on the premise that every point in the scene is stationary, so in scenes crowded with cars or pedestrians, contaminated matches distort the optimization. RANSAC filters some of them, but it is not a root fix. Research into dynamic SfM (segmentation integration, per-object independent motion estimation) is in progress, but as of 2026 there is no general-purpose implementation at COLMAP's level.

**The blurring boundary between SfM and SLAM.** In 2023, [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al.) took two images into a single pretrained network and produced a dense point map and camera poses at once. No feature matching, no RANSAC, and no bundle-adjustment initialization. Extended as [MASt3R](https://arxiv.org/abs/2406.09756) (2024), it handled tens of images. Each module of the traditional SfM pipeline is being replaced one at a time. If COLMAP was the entryway to NeRF and 3DGS, the DUSt3R line is trying to replace that entryway itself. Whether this paradigm will actually push COLMAP out, or win only in specific domains, is still unknown.

---

While SfM was refining precise offline reconstruction, a different kind of question was piling up elsewhere. Not photographs, but a moving robot. The images have not been gathered yet. A pose estimate has to come out right now. The question Randall Smith and Peter Cheeseman [posed in 1986](https://people.csail.mit.edu/brooks/idocs/Smith_Cheeseman.pdf), how to propagate uncertain spatial relations, grew under that pressure into a separate field called SLAM.
