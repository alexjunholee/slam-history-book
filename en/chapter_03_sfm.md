# Chapter 3. SfM: From Longuet-Higgins to COLMAP

While Harris and Lowe were figuring out how to pick reliable points out of a single image, a parallel community was asking what geometry can recover once those points appear across two or more images. Feature *detection* and spatial *reconstruction* advanced along separate tracks for more than two decades before converging into a single pipeline in the mid-2000s.

In 1981, H.C. Longuet-Higgins—working not in a robotics department, but in the Laboratory of Experimental Psychology at the University of Sussex—published a three-page paper in *Nature* titled "[A Computer Algorithm for Reconstructing a Scene from Two Projections](https://cseweb.ucsd.edu/classes/fa01/cse291/hclh/SceneReconstruction.pdf)." In it, he showed that given just eight corresponding point pairs across two photographs, a simple linear algorithm could recover both camera motion and 3D scene structure. Photogrammetrists and vision scientists had studied motion perception long before 1981, so his paper did not invent Structure from Motion (SfM) out of thin air; nonetheless, it became the modern catalyst for two-view geometry. Over the next thirty-five years, that spark led through self-calibration, incremental SfM, Bundler, and large-scale reconstruction from internet photographs, culminating in 2016 when Johannes Schönberger assembled the accumulated craft into COLMAP, a robust general-purpose open-source pipeline.

---

## 3.1 Essential Matrix and the 8-point Algorithm

Longuet-Higgins started from a crisp geometric observation: whenever two calibrated cameras observe the same 3D point, their normalized image coordinates obey a single bilinear constraint. That constraint collapses into a single $3\times 3$ matrix, which he defined as the **essential matrix** $\mathbf{E}$.

If the two camera centers sit at $\mathbf{O}_1$ and $\mathbf{O}_2$ and the corresponding normalized image points are $\mathbf{x}_1$ and $\mathbf{x}_2$, they satisfy:

$$\mathbf{x}_2^\top \mathbf{E} \mathbf{x}_1 = 0$$

Inside this matrix, the relative rotation $\mathbf{R}$ and translation $\mathbf{t}$ between the two views factor cleanly as $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$, where $[\mathbf{t}]_\times$ is the skew-symmetric matrix of $\mathbf{t}$.

Once global scale is set aside, the essential matrix carries only five degrees of freedom. Yet solving directly from five correspondences requires handling nonlinear polynomial equations—a capability that had to wait for [Nistér's 2004](http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/2004-PAMI-David%20Nister-An%20Efficient%20Solution%20to%20the%20Five-Point%20Relative%20Pose%20Problem.pdf) 5-point algorithm. Longuet-Higgins bypassed that complexity by fixing one of the nine matrix entries as unit scale, treating the remaining eight as independent unknowns in a linear system solved from eight correspondences, and enforcing the rank-2 and unit-scale constraints afterward. His **8-point algorithm** delivered a unique solution directly from eight points, making it fast, lightweight, and remarkably easy to code.

That linear shortcut, however, suffered from poor numerical conditioning. Because raw image coordinates span hundreds or thousands of pixels while homogeneous coordinates append a trailing 1, the columns of the linear system vary by orders of magnitude, rendering the singular value decomposition (SVD) hypersensitive to noise. Fortunately, the cure required no change to the underlying geometry:

> 🔗 **Borrowed.** Hartley's 1997 normalized 8-point algorithm ([In Defense of the Eight-Point Algorithm](https://www.cse.unr.edu/~bebis/CS485/Handouts/hartley.pdf)) applied a simple linear transform to each image's coordinates—shifting their mean to zero and scaling their average distance from the origin to $\sqrt{2}$—before estimating the fundamental matrix and transforming the result back. Unlike normalizing by camera intrinsics, this was purely a numerical conditioning step that left Longuet-Higgins's geometry untouched while transforming its reliability, quickly becoming standard textbook practice.

When camera intrinsics $\mathbf{K}$ are unknown, the same epipolar geometry generalizes from the essential matrix to the **fundamental matrix** $\mathbf{F}$, which satisfies $\mathbf{x}_2^\top \mathbf{F} \mathbf{x}_1 = 0$ directly in pixel coordinates. Given intrinsic matrices $\mathbf{K}_1$ and $\mathbf{K}_2$ for the two views, the two matrices relate by $\mathbf{F} = \mathbf{K}_2^{-\top} \mathbf{E} \mathbf{K}_1^{-1}$—simplifying to $\mathbf{F} = \mathbf{K}^{-\top} \mathbf{E} \mathbf{K}^{-1}$ when both photographs come from the same camera ($\mathbf{K}_1 = \mathbf{K}_2 = \mathbf{K}$). Modern SfM pipelines exploit this duality automatically, solving directly for $\mathbf{E}$ when $\mathbf{K}$ is calibrated and falling back to $\mathbf{F}$ when it is not.

---

## 3.2 Tomasi-Kanade Factorization

For roughly a decade after 1981, SfM research centered on pairs of views rather than full image sequences. Solving for an entire stream of photographs simultaneously posed a different algebraic challenge—one that Carlo Tomasi and Takeo Kanade at CMU answered in 1992 with their **[factorization method](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/TomasiC_Shape%20and%20motion%20from%20image%20streams%20under%20orthography.pdf)**.

Suppose $F$ frames track the same $P$ points throughout a sequence. Stacking every 2D measurement produces a $2F \times P$ measurement matrix $\mathbf{W}$. Subtracting each frame's point centroid eliminates camera translation, and under an orthographic (or scaled orthographic) camera model—the starting point of Tomasi and Kanade's 1992 paper—the centered matrix $\mathbf{W}$ has rank at most three. It therefore factors cleanly into motion and structure:

$$\mathbf{W} = \mathbf{M} \mathbf{S}$$

In this decomposition, $\mathbf{M}$ is a $2F \times 3$ motion matrix and $\mathbf{S}$ is a $3 \times P$ structure matrix. Truncating an SVD of $\mathbf{W}$ to its top three singular values immediately produces a rank-3 factorization. Because any invertible $3\times 3$ matrix $\mathbf{A}$ yields an equally valid product $(\mathbf{M}\mathbf{A})(\mathbf{A}^{-1}\mathbf{S})$, a subsequent metric-upgrade step pins down $\mathbf{A}$ by enforcing orthogonality and equal-norm constraints on the camera rows.

With a single low-rank SVD followed by that metric upgrade, Tomasi-Kanade recovered every camera pose and every 3D point simultaneously—far simpler to implement than iterative nonlinear bundle adjustment, with runtime governed mainly by the SVD of a $2F \times P$ matrix. Later real-time systems adapted this multi-frame intuition in a leaner, incremental way:

> 🔗 **Borrowed.** Later literature often marks Nistér, Naroditsky, and Bergen's 2004 CVPR paper "Visual Odometry" as the moment real-time ego-motion estimation crystallized into an applied lineage. Rather than running Tomasi-Kanade's batch factorization over a full sequence, it solved relative pose frame-by-frame inside a short window, trading global batch accuracy for low latency.

Ultimately, Tomasi-Kanade's elegance was boxed in by its orthographic and affine camera model. Because an affine camera ignores perspective foreshortening, the factorization holds closely only when depth variation within the scene is tiny compared to the distance from the camera, such as distant, compact objects. On wide-angle lenses, close-up scenes, or environments with large foreground-to-background depth differences, reconstruction error climbed quickly. By the late 1990s, attempts to extend factorization to full perspective cameras pushed researchers back toward bundle adjustment.

---

## 3.3 Hartley & Zisserman and the canonization

While Tomasi-Kanade framed the multi-view problem, the field still lacked a unified mathematical grammar capable of handling full perspective geometry across arbitrary numbers of cameras.

That synthesis arrived in 2000 with Richard Hartley and Andrew Zisserman's 680-page textbook, *[Multiple View Geometry in Computer Vision](https://www.robots.ox.ac.uk/~vgg/hzbook/)*. Drawing together nearly two decades of results scattered from 1981 through the 1990s, the authors unified the essential matrix, fundamental matrix, homography, camera calibration, and bundle adjustment under the single framework of projective geometry. Concepts that had matured in separate papers suddenly appeared as facets of one coherent theory.

Bundle adjustment played a starring role in that synthesis. Building on the Triggs et al. (1999) formulation reviewed in Chapter 1, Hartley and Zisserman cast reprojection-error minimization inside projective geometry and paired it explicitly with a *robust cost function* $\rho$—such as Huber or Cauchy—to damp out mismatched outliers that would otherwise derail optimization on real photographs. Using Levenberg-Marquardt alongside the sparse structure of the Jacobian, they delivered a practical blueprint for large-scale solvers.

Throughout the early 2000s, nearly every SLAM and visual odometry (VO) paper cited Hartley & Zisserman as its foundational reference. With shared definitions established in one textbook, researchers building systems like Photo Tourism could focus on scale and engineering rather than re-deriving basic geometry.

---

## 3.4 Photo Tourism and Bundler — Internet-scale SfM

In 2006, Noah Snavely, Steven Seitz, and Richard Szeliski proved that this textbook geometry could survive the chaos of uncurated consumer photos. In their SIGGRAPH paper "[Photo Tourism](https://doi.org/10.1145/1179352.1141964)," they harvested tourist snapshots of landmarks such as the Florence Duomo and Rome's Trevi Fountain directly from the internet and reconstructed them in 3D.

Unlike lab datasets, those collections were completely uncontrolled: cameras, lenses, lighting, weather, and framing varied wildly, mixed in with unrelated indoor shots uploaded in no order by thousands of strangers. Snavely's pipeline first extracted and matched SIFT features across image pairs, then applied RANSAC with the fundamental matrix to prune geometrically inconsistent matches. Starting from a strongly connected seed pair, incremental SfM registered cameras one at a time, re-running bundle adjustment over all poses and 3D points after each addition.

The results demonstrated that uncontrolled internet photo collections could yield coherent 3D models: Notre Dame Cathedral registered 597 out of 2,635 candidate images, the Trevi Fountain in Rome 360 out of 466, Yosemite Half Dome 325 out of 1,882, the Great Wall 82 out of 120, and Trafalgar Square 278 out of 1,893, achieving an average reprojection error of roughly 1.5 pixels on 1,611×1,128 images. Snavely released the underlying pipeline as open-source software under the name **Bundler**, giving the SfM community a common baseline to build upon. You can step through how incremental SfM adds cameras and adjusts bundles in the interactive demo below:

<!-- DEMO: sfm_incremental.html -->

---

## 3.5 COLMAP — engineering maturity

> 📜 **Prediction vs. outcome.** In the "Discussion and future work" section of Snavely et al. (2006), the authors wrote, *"Ultimately, we wish to scale up our reconstruction algorithm to handle millions of photographs,"* flagging better registration ordering, lens-distortion modeling, repeated structures, and disconnected components as open hurdles. Ten years later, COLMAP (Schönberger 2016) and OpenSfM pushed robust open-source reconstruction to tens and hundreds of thousands of images, while SLAM pursued real-time online scale separately via fixed-lag smoothers and loop closure rather than offline incremental refinement. Even with that progress, the scale cited here reflects tens to hundreds of thousands of images rather than demonstrating that the million-image goal had been solved universally.

In 2016, Johannes Schönberger and Jan-Michael Frahm published "[Structure-from-Motion Revisited](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf)" at CVPR. Behind that modest title lay a ground-up engineering overhaul incorporating a decade of lessons learned since Bundler, packaged into **COLMAP**.

Compared with Bundler, COLMAP transformed incremental SfM in three decisive ways. First, it replaced ad-hoc camera ordering with principled next-best-view selection: instead of picking whichever pair had the most raw connections, COLMAP scored initial pairs and subsequent camera additions by triangulation angle, feature-track length, and spatial visibility distribution, dramatically stabilizing reconstruction. Second, it restructured the bundle-adjustment schedule: rather than paying for a full global bundle adjustment after every single registered image, COLMAP ran fast local bundle adjustment over the newly added camera and its closest neighbors, triggering global bundle adjustment only periodically. Third, it strengthened geometric verification by running RANSAC on every matched pair with two competing models in parallel—the fundamental matrix for general 3D scenes and a homography for planar scenes or pure camera rotation—and comparing their inlier counts to classify the geometry and reject false matches before reconstruction could drift.

> 🔗 **Borrowed.** COLMAP's incremental bundle-adjustment pipeline modularized Snavely's Bundler workflow and inserted strict quality checks at every stage. Its core mathematical primitives—essential matrix estimation, triangulation, and Levenberg-Marquardt—remained straight out of Hartley & Zisserman; COLMAP's breakthrough was turning hard-won engineering judgment into a reliable system.

Beyond raw accuracy, COLMAP conquered 3D vision through practical craftsmanship: clean code, thorough documentation, and CUDA acceleration suited to large image collections. When NeRF arrived in 2020 and 3D Gaussian Splatting followed, their reference training pipelines assumed COLMAP camera poses and sparse point clouds as standard preprocessing inputs, making COLMAP the de facto gateway for modern 3D reconstruction research.

---

## 3.6 The split between SfM and SLAM

Even though SfM and SLAM share projective geometry and nonlinear least squares, they solve fundamentally different operational problems—a divergence that sharpened unmistakably in the early 2000s.

SfM operates *offline*: every photograph is collected before reconstruction begins, leaving no per-frame deadline and allowing global bundle adjustment to revisit and repair any camera pose across the entire dataset. SLAM, by contrast, must run *online* on a moving platform: sensor data streams in continuously, the robot needs its current pose right away, past frames cannot be reprocessed endlessly as the map expands, and accumulated drift has to be corrected promptly whenever the robot closes a loop back to a previously visited place.

That temporal constraint dictates two very different system designs. Because SLAM must detect revisits on the fly and propagate loop-closure corrections back through long chains of poses, it also has to track the uncertainty of the robot's current pose in real time—requiring probabilistic representations such as an Extended Kalman Filter (EKF) or factor graph. In offline SfM, parameter covariances can simply be extracted once optimization finishes.

When Andrew Davison presented [MonoSLAM (2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf), he initially framed it as "real-time SfM." Yet keeping camera pose and 3D landmarks coupled inside a single EKF state vector stood worlds apart from SfM's offline batch optimization. Over the 2000s, the two communities grew into distinct traditions, each defined by its own latency constraints and benchmarks.

---

## Still open

**SfM with dynamic objects.** General-purpose SfM pipelines such as COLMAP still assume a rigidly static world. Because bundle adjustment expects every 3D point to stay put, feature matches on moving cars or pedestrians pull camera poses off course. RANSAC removes some of these contaminated correspondences, and Dynamic SfM methods introduce segmentation or per-object motion models, yet no public implementation has matched COLMAP's broad reliability on everyday dynamic scenes.

**The blurring boundary between SfM and SLAM.** In 2023, [DUSt3R](https://arxiv.org/abs/2312.14132) (Wang et al.) fed two images into a single pretrained transformer and directly predicted dense 3D point maps alongside camera poses—skipping feature matching, RANSAC, and bundle-adjustment initialization altogether. Its successor [MASt3R](https://arxiv.org/abs/2406.09756) (2024) extended the approach across tens of images. One by one, traditional SfM modules are being absorbed into learned networks, with DUSt3R-family models now vying to replace COLMAP as the front end for NeRF and 3DGS. Whether they will displace classical SfM completely or settle into complementary niches remains an open question.

---

While SfM perfected offline 3D reconstruction from completed image collections, robotics faced a far less forgiving reality: a moving robot had to estimate where it was *before* the journey ended, using noisy measurements as they arrived. Out of that pressure, the question Randall Smith and Peter Cheeseman [posed in 1986](https://people.csail.mit.edu/brooks/idocs/Smith_Cheeseman.pdf)—how to propagate uncertain spatial relationships step by step—grew into the discipline of SLAM.
