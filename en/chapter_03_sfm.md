# Ch.3 — Structure from Motion: From Longuet-Higgins to COLMAP

While Harris and Lowe were refining how to detect salient points in one image, a different lineage asked what could be recovered when those points appeared in two images. Feature *detection* and spatial *reconstruction* developed side by side over the same period; only in the mid-2000s did they merge into one pipeline.

In 1981, H.C. Longuet-Higgins, a theoretical psychologist at Cambridge, published a three-page paper in *Nature*. The title was "[A Computer Algorithm for Reconstructing a Scene from Two Projections](https://cseweb.ucsd.edu/classes/fa01/cse291/hclh/SceneReconstruction.pdf)." He showed that eight coordinate pairs for the same points in two photographs were enough to solve simultaneously for camera motion and the scene's three-dimensional shape. He was neither a roboticist nor a computer vision researcher. Structure from Motion (SfM) began in those three pages, and the mathematics reached a widely used engineering system in 2016, when Johannes Schönberger released COLMAP.

---

## 3.1 Essential Matrix and the 8-point Algorithm

Longuet-Higgins began with a simple constraint. When two cameras capture the same point, an algebraic relation holds between the two image coordinates. Once the coordinate system is normalized, the relation can be written as a single matrix. He defined it as the **essential matrix** $\mathbf{E}$.

Let the two camera centers be $\mathbf{O}_1$ and $\mathbf{O}_2$, and let the corresponding points in normalized coordinates be $\mathbf{x}_1$ and $\mathbf{x}_2$. The constraint is:

$$\mathbf{x}_2^\top \mathbf{E} \mathbf{x}_1 = 0$$

$\mathbf{E}$ factors through the rotation $\mathbf{R}$ and translation $\mathbf{t}$ between the cameras as $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$, where $[\mathbf{t}]_\times$ is the skew-symmetric matrix of $\mathbf{t}$.

Once scale ambiguity is removed, the essential matrix has five degrees of freedom. Before the nonlinear 5-point algorithm ([Nistér 2004](http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/2004-PAMI-David%20Nister-An%20Efficient%20Solution%20to%20the%20Five-Point%20Relative%20Pose%20Problem.pdf)) solved it with five correspondences, the standard approach fixed one of the nine matrix entries as unit scale, treated the remaining eight as unknowns, and solved a linear system from eight correspondences. The rank-2 and unit-scale constraints were then enforced. This is the **8-point algorithm**. Longuet-Higgins himself gave a procedure that produced a unique solution from exactly eight points. The implementation was simple and computationally inexpensive.

Numerical stability was the problem. When image coordinates run into the hundreds or thousands of pixels, the entries of the coefficient matrix span sharply different scales, and the SVD becomes unstable.

> 🔗 **Borrowed.** Hartley's 1997 normalized 8-point algorithm ([In Defense of the Eight-Point Algorithm](https://www.cse.unr.edu/~bebis/CS485/Handouts/hartley.pdf)) applied a linear transform to image coordinates so that their mean was zero and their average distance was $\sqrt{2}$, and then estimated the fundamental matrix before transforming it back to the original coordinates. This serves a different purpose from normalizing coordinates with camera intrinsics. The geometry of Longuet-Higgins was left untouched; only the numerical conditioning was fixed. The normalization was widely adopted in later multiple-view-geometry texts and implementations.

The fundamental matrix $\mathbf{F}$ generalizes the essential matrix. Even without knowing the camera intrinsics $\mathbf{K}$, the relation $\mathbf{x}_2^\top \mathbf{F} \mathbf{x}_1 = 0$ holds. With intrinsics $\mathbf{K}_1$, $\mathbf{K}_2$ for the two cameras, the relationship is $\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \mathbf{K}_1^{-1}$. For images from the same camera ($\mathbf{K}_1 = \mathbf{K}_2 = \mathbf{K}$) it simplifies to $\mathbf{F} = \mathbf{K}^{-\top} \mathbf{E} \mathbf{K}^{-1}$. In an SfM pipeline, when $\mathbf{K}$ is unknown $\mathbf{F}$ is estimated first; when $\mathbf{K}$ is known, $\mathbf{E}$ is solved directly.

---

## 3.2 Tomasi-Kanade Factorization

For ten years after 1981, SfM was studied mostly as the geometry between two photographs. Processing many photographs at once was a separate problem. Carlo Tomasi and Takeo Kanade at CMU provided one answer in 1992 with the **[factorization method](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/TomasiC_Shape%20and%20motion%20from%20image%20streams%20under%20orthography.pdf)**.

Given $F$ frames observing $P$ points, the image coordinates stack into a $2F \times P$ matrix $\mathbf{W}$. After subtracting each frame's point centroid to remove translation, $\mathbf{W}$ has rank at most three under an orthographic (scaled orthographic) camera model. The original paper (Tomasi & Kanade 1992) started from this assumption. Then:

$$\mathbf{W} = \mathbf{M} \mathbf{S}$$

where $\mathbf{M}$ is a $2F \times 3$ motion matrix and $\mathbf{S}$ is a $3 \times P$ structure matrix. Keeping only the top three singular values of $\mathbf{W}$ through SVD gives a rank-3 factorization. Because $\mathbf{M}\mathbf{A}$ and $\mathbf{A}^{-1}\mathbf{S}$ produce the same $\mathbf{W}$, a metric-upgrade step then determines the invertible matrix $\mathbf{A}$ from orthogonality and equal-norm constraints on the camera rows.

The procedure estimated every frame's motion and every point's 3D position through one low-rank SVD followed by the metric upgrade. Its cost is dominated by the SVD of a $2F \times P$ matrix, and its structure is simpler to implement than iterative nonlinear bundle adjustment.

> 🔗 **Borrowed.** Later literature places Nistér, Naroditsky, and Bergen's 2004 CVPR paper "Visual Odometry" at the point where real-time ego-motion estimation became an applied branch of this lineage. Instead of using Tomasi-Kanade's batch factorization directly, the work solved relative pose between frames inside a short window, trading batch accuracy for lower latency.

The limitation was the orthographic/affine assumption. An affine camera ignores perspective distortion. The model holds only when depth variation in the scene is small relative to the distance from the camera, as with small, distant objects. Error grew for close scenes, wide-angle lenses, and scenes with large foreground–background depth differences. From the late 1990s, researchers pursued perspective-camera extensions from several directions, leading back to bundle adjustment.

---

## 3.3 Hartley & Zisserman and the canonization

Tomasi-Kanade's factorization framed the multiple-view problem. The remaining tasks were to extend it to perspective cameras and unify the scattered mathematics in one language.

In 2000, Richard Hartley and Andrew Zisserman's 680-page textbook *[Multiple View Geometry in Computer Vision](https://www.robots.ox.ac.uk/~vgg/hzbook/)* appeared. It consolidated the SfM mathematics scattered from 1981 through the 1990s into the language of projective geometry.

Hartley & Zisserman did more than compile earlier results. They brought the essential matrix, fundamental matrix, homography, camera calibration, and bundle adjustment into one projective-geometry framework. Their treatment showed within one text that concepts developed separately shared the same foundation.

Bundle adjustment received particular attention. Hartley & Zisserman placed the reprojection-error minimization problem reviewed in Ch.1 through the synthesis by Triggs et al. (1999) inside the projective-geometry framework and included an explicit *robust cost function* $\rho$. Huber or Cauchy losses downweighted outliers that would otherwise break optimization on real data. The solver was Levenberg-Marquardt, and the sparsity of the Jacobian reduced computation.

Most SLAM and visual odometry (VO) papers in the early 2000s cited this textbook as their standard reference. With the definitions unified in one source, large-scale applications such as Photo Tourism could focus on implementation rather than redefining the basics.

---

## 3.4 Photo Tourism and Bundler — Internet-scale SfM

In 2006, Noah Snavely, Steven Seitz, and Richard Szeliski published the SIGGRAPH paper "[Photo Tourism](https://doi.org/10.1145/1179352.1141964)." They gathered photographs of tourist sites uploaded to the internet (the Florence Duomo, the Trevi Fountain in Rome) and reconstructed them in 3D.

The data were uncontrolled. Cameras, weather, and composition varied, and some images were unrelated indoor shots. This was not a systematically captured dataset but thousands of images uploaded in no particular order by thousands of people.

Snavely's pipeline first used SIFT detection and matching to find correspondences between image pairs. RANSAC with the fundamental matrix removed geometrically inconsistent matches. Incremental SfM started from highly connected image pairs and added cameras one at a time. After each addition, bundle adjustment reoptimized the full set of poses and points.

The datasets reported in the paper included the Notre Dame Cathedral (597 registered out of 2,635 candidates), the Trevi Fountain in Rome (360 out of 466), Yosemite Half Dome (325 out of 1,882), the Great Wall (82 out of 120), and Trafalgar Square (278 out of 1,893), with an average reprojection error of about 1.5 pixels on 1,611×1,128 images. The work became a turning point in assembling hundreds of uncontrolled internet photographs into consistent reconstructions.

Bundler implemented this pipeline. Snavely released it as open source, and it became the default starting point for SfM researchers.

<!-- DEMO: sfm_incremental.html -->

---

## 3.5 COLMAP — engineering maturity

> 📜 **Prediction vs. outcome.** The "Discussion and future work" section of Snavely et al. 2006 states, "Ultimately, we wish to scale up our reconstruction algorithm to handle millions of photographs," and lists better image-registration ordering, lens-distortion modeling, repeated-structure handling, and disconnected-structure reconstruction as remaining problems. COLMAP (Schönberger 2016) and OpenSfM pursued scale, reaching tens to hundreds of thousands of images. The SLAM lineage addressed real-time and online processing separately through fixed-lag smoothers and loop closure, not through incremental refinement. This was progress toward scale, although the sizes cited here do not establish that the goal of millions of photographs was met.

In 2016, Johannes Schönberger and Jan-Michael Frahm published the CVPR paper "[Structure-from-Motion Revisited](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)." Despite the modest title, the paper systematically redesigned the pipeline around ten years of improvements since Bundler.

COLMAP differed from Bundler most in three respects.

The first change was the order in which cameras were added. Bundler started from pairs with high connectivity but had no systematic criterion for which pair to extend first. COLMAP automated the choice of initial image pair and camera-registration order using triangulation angle, feature-track length, and visibility score. Reconstruction became substantially more stable.

The second change was the bundle-adjustment cadence. Running a full bundle adjustment after every camera addition is expensive. COLMAP alternated local bundle adjustment (optimizing only the recently added camera together with cameras that shared many points with it) with periodic global bundle adjustment.

The third change was geometric verification. For each pair of matched feature points, COLMAP ran RANSAC with two models in parallel: the fundamental matrix and homography. The fundamental matrix covered general non-planar scenes; the homography covered planar scenes or pure rotation. COLMAP compared the two models' inlier counts to classify the scene and filtered out matches that fit neither. It was more robust than Bundler to poor matches and planar degeneracy.

> 🔗 **Borrowed.** COLMAP's incremental bundle adjustment strategy modularized Snavely's Bundler pipeline and added quality control at each stage. The core mathematics of the algorithm (essential matrix estimation, triangulation, Levenberg-Marquardt) came from the Hartley & Zisserman textbook. COLMAP's contribution was the systematization of engineering judgment rather than new mathematics.

COLMAP became widely used for reasons beyond performance. The codebase was well organized, the documentation was adequate, and CUDA acceleration supported large image collections. After NeRF appeared in 2020, many NeRF training codebases took COLMAP's output (camera poses + sparse point cloud) as input, and the reference 3D Gaussian Splatting implementation used the same preprocessing. COLMAP became a common front end for 3D reconstruction research.

---

## 3.6 The split between SfM and SLAM

SfM and SLAM use the same mathematics yet solve different problems. The distinction came into sharp relief in the early 2000s.

SfM is *offline*. All images are gathered before processing, so there is no time constraint, and global bundle adjustment can be run multiple times with full access to the whole dataset. If a camera pose is wrong, the system can go back and recompute it.

SLAM is *online*. Sensor data streams in real time, and the robot's current position must be available immediately. The system cannot retain and revisit past data indefinitely, computation grows with the map, and accumulated drift must be corrected when the robot returns to a previously visited place.

The fields differ in the constraints of online processing. SLAM must detect revisits during motion and correct accumulated drift, potentially across many poses connected by the loop. SfM and SLAM share tools such as bundle adjustment and image retrieval, but differ in when and which states must be updated.

Uncertainty propagation differed as well. SLAM tracks the uncertainty of the current pose in real time and updates it with each new observation. A probabilistic representation in the form of EKF or factor graph is needed. In SfM, covariance can be computed after optimization finishes, and real-time tracking is not required.

Davison's [MonoSLAM (2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf) called itself "real-time SfM." But its structure, which kept camera pose and landmarks together in an EKF state vector, differed from SfM's global batch optimization. Over the 2000s, the two fields split into independent lineages, each with its own problem setting.

---

## 3.7 🧭 Still open

**SfM with dynamic objects.** Mainstream general-purpose SfM pipelines, including COLMAP, assume a static world. Bundle adjustment is solved on the premise that scene points are stationary, so contaminated matches distort optimization around cars or pedestrians. RANSAC filters some of them, while Dynamic SfM research models segmentation or per-object motion. Among these public implementations, none has settled into a general-purpose tool with COLMAP's scope.

**The blurring boundary between SfM and SLAM.** In 2023, [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al.) took two images into a single pretrained network and produced a dense point map and camera poses at once. It needed no feature matching, RANSAC, or bundle-adjustment initialization. Extended as [MASt3R](https://arxiv.org/abs/2406.09756) (2024), it handled tens of images. Modules of the traditional SfM pipeline are now being replaced one at a time. COLMAP became the front end for NeRF and 3DGS; the DUSt3R line is trying to replace that front end. Whether it will displace COLMAP or prevail only in specific domains remains unknown.

---

While SfM refined precise offline reconstruction, another question grew urgent in robotics. A moving robot had to estimate its pose immediately from images that had not yet been gathered. Under that pressure, the question Randall Smith and Peter Cheeseman [posed in 1986](https://people.csail.mit.edu/brooks/idocs/Smith_Cheeseman.pdf), how to propagate uncertain spatial relations, grew into a separate field called SLAM.
