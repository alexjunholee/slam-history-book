# Chapter 1. Photogrammetry and Bundle Adjustment

The mathematical backbone of today's SLAM optimization backend did not originate in robotics; it was born in German surveying. In the early twentieth century, Carl Pulfrich's optical method for reading depth from glass photographic plates merged with Albrecht Meydenbauer's architectural surveying system to establish a unified photogrammetric tradition. Half a century later, Duane C. Brown cast that tradition into numerical equations in 1958, and in 1999 Bill Triggs, Philip McLauchlan, Richard Hartley, and Andrew Fitzgibbon translated the accumulated surveying literature into the language of computer vision. Rather than inventing bundle adjustment from scratch, Triggs et al. (1999) unlocked a century-old inheritance for a new field—combining Pulfrich's parallax geometry and Brown's (1958) reprojection formulation with the Levenberg-Marquardt nonlinear solver.

---

## 1.1 Early twentieth-century glass plates and stereophotogrammetry

In 1901, [Carl Pulfrich](https://en.wikipedia.org/wiki/Carl_Pulfrich) formally unveiled the **stereocomparator**, manufactured by the Zeiss optical works, at the Hamburg conference of natural scientists (building on a prototype stereoscopic rangefinder he had shown in Munich in 1899). The instrument worked by photographing the same scene from two camera viewpoints and measuring the coordinate difference between corresponding points on the glass plates. Its underlying geometry was straightforward: the parallax between two views is inversely proportional to depth. While that trigonometric relationship had been known since Greek antiquity, Pulfrich's achievement was to pair it with precision optical instrumentation.

A generation earlier, [Albrecht Meydenbauer](https://www.uni-marburg.de/de/fotomarburg/histfoto/gliederung/messbilder/messbildverfahren) had pioneered **architectural photogrammetry** for documenting historic buildings. After nearly falling to his death while measuring the exterior of Wetzlar Cathedral in 1858, he set out to replace risky manual tape-measuring with photographs, an effort that culminated in the founding of the Royal Prussian Photogrammetric Institute (Königlich Preussische Messbild-Anstalt) in 1885.

In the twentieth century, these two streams converged in aerial surveying. Using **aerotriangulation**, aircraft photographed terrain from above so that overlapping pairs of images could be turned into three-dimensional maps—all computed by human operators working with hand calculators. The geometric logic established in that era still underpins modern robotics:

> 🔗 **Borrowed.** Modern SLAM's stereo depth estimation runs on the same principle as Pulfrich's stereocomparator. Depth still comes from the baseline between two cameras and the resulting parallax; a digital pixel array has simply replaced the glass plate of 125 years ago.

---

## 1.2 1958, Brown, and numerical bundle adjustment

What Pulfrich and Meydenbauer achieved with optical instruments, [Duane C. Brown](https://digital.hagley.org/08206139_solution) moved into numerical equations. Working as a surveying engineer on the United States Air Force ballistic missile program, Brown had to jointly estimate satellite orbits and ground coordinates—a task that meant optimizing many camera viewpoints and many ground control points at the same time.

In his 1958 report "A Solution to the General Problem of Multiple Station Analytical Stereotriangulation" (RCA-MTP Data Reduction Technical Report No. 43, AFMTC-TR-58-8), Brown produced one of the foundational formulations of numerical **bundle adjustment (BA)**, alongside Helmut Schmid as a co-inventor of the same era.

At the center of Brown's formulation sits the **reprojection error**. Suppose a 3D point $X_j \in \mathbb{R}^3$ is projected into camera $i$ through its intrinsic matrix $K_i$ and extrinsic pose $[R_i | t_i]$, producing a predicted 2D coordinate $\pi(K_i, R_i, t_i, X_j)$. Bundle adjustment minimizes the sum of squared differences between those predicted projections and the actual 2D image coordinates $x_{ij} \in \mathbb{R}^2$ observed in each image:

$$E = \sum_{i,j} \| x_{ij} - \pi(K_i, R_i, t_i, X_j) \|^2$$

The name "bundle" refers to the bundle of light rays radiating from each camera center toward the observed 3D points. Adjusting the camera poses and point coordinates together bends and aligns those bundles until corresponding rays intersect cleanly in 3D space. Although Brown developed the method for military surveying, bundle adjustment continued to mature quietly within the photogrammetry literature for decades before crossing over into computer vision.

> 🔗 **Translation.** Triggs et al. (1999) synthesized photogrammetric bundle adjustment specifically for implementers in computer vision, bridging two communities that had been tackling the same optimization problem with different terminology and conventions. Notably, their historical account does not support the popular claim that military classification forced computer vision researchers to rediscover the method from scratch.

---

## 1.3 Levenberg and Marquardt — pioneers of nonlinear optimization

Brown supplied the objective function, but the numerical engine capable of solving it came from outside surveying altogether. Because reprojection-error minimization is a nonlinear least-squares problem with no closed-form analytical solution, it requires iterative numerical optimization.

Fourteen years before Brown's report, [Kenneth Levenberg](https://cs.uwaterloo.ca/~y328yu/classics/levenberg.pdf) published a 1944 paper introducing a damping parameter $\lambda$ to interpolate smoothly between Gauss-Newton and steepest descent. Increasing $\lambda$ steers the step toward steepest descent for safe, reliable progress; decreasing $\lambda$ hands control back to Gauss-Newton for rapid convergence near the solution. In its basic form, the update adds $\lambda \mathbf{I}$ to the linearized normal-equation matrix $\mathbf{J}^{\top}\mathbf{J}$, preventing singularity and stabilizing each step. Twenty years ahead of computer vision, the idea was independently rediscovered and formalized in 1963 by [Donald Marquardt](https://epubs.siam.org/doi/10.1137/0111030), settling into the literature as the **Levenberg-Marquardt (LM) algorithm**.

Even then, another thirty-five years would pass before LM became the standard bundle-adjustment solver in computer vision.

---

## 1.4 1999, Triggs et al. — a hundred years of inheritance integrated

The convergence of photogrammetric bundle adjustment and the LM solver arrived at the 1999 Vision Algorithms Workshop, where Bill Triggs, Philip McLauchlan, Richard Hartley, and Andrew Fitzgibbon presented ["Bundle Adjustment — A Modern Synthesis"](https://link.springer.com/chapter/10.1007/3-540-44480-7_21).

Their paper translated a century of bundle-adjustment theory scattered across surveying and aerial photogrammetry into the working vocabulary of computer vision, making two lasting contributions. First, they exposed the computational value of sparse BA: because each 3D point appears in only a subset of cameras, the Hessian has a predictable sparse block structure that can be exploited via the Schur complement trick to make joint camera-point optimization dramatically faster. Second, they gave an explicit treatment of gauge freedom—the inherent arbitrariness of choosing a global reference frame.

That synthesis quickly bore engineering fruit. Seven years later, Noah Snavely's [Photo Tourism (2006)](https://phototour.cs.washington.edu/Photo_Tourism.pdf) automatically reconstructed landmarks such as Notre-Dame and the Trevi Fountain from hundreds of uncurated internet photographs. Another decade on, Johannes Schönberger's [COLMAP (2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) packaged robust incremental structure from motion (SfM) into open-source software capable of handling tens to hundreds of thousands of images, turning a technique that had already scaled into the million-image range into a reproducible community tool. Without the shared language established by Triggs et al., that transition would have taken far longer.

---

## 1.5 Reprojection Error — the formation of the concept

The error function at the core of Triggs et al.'s synthesis had its own history of conceptual shifts before reaching its modern form.

In the early twentieth century, aerial triangulators typically measured error as a spatial distance in the 3D ground coordinate frame. Yet comparing points directly in 3D had a subtle flaw: errors from a misaligned lens or imperfect camera calibration could be absorbed into the ground-coordinate residual and vanish from diagnosis.

In his 1958 report, Brown shifted the comparison plane from 3D ground space to the 2D image plane, measuring the discrepancy between a projected 3D point and its observed pixel location directly in pixel units. On the image plane, calibration errors, lens distortion, and extrinsic pose errors all surface cleanly in a single residual. Just as important, the formulation rests on solid statistical ground: if pixel noise is modeled as an isotropic Gaussian, minimizing the sum of squared reprojection errors coincides exactly with maximum-likelihood estimation. Triggs et al. (1999) canonized that formulation for computer vision textbooks, and as of 2026 reprojection-error minimization remains the workhorse objective for visual observations in factor graph-based SLAM backends.

> 🔗 **Borrowed.** The standard visual-landmark observation model in SLAM, $z = \pi(K, T, p) + \epsilon$, descends directly from Brown's (1958) reprojection formula. A modern SLAM backend minimizing this residual with Gauss-Newton shares the exact mathematical skeleton of a 1958 aerial triangulation solver.

---

## 1.6 The skeleton of the SLAM backend — through 2026

Although the acronym "SLAM" only entered broad circulation through 1990s literature such as [Durrant-Whyte and Leonard's 1995 survey](https://ieeexplore.ieee.org/document/476131), the mathematics inside modern SLAM backends still bears Brown's 1958 imprint. ORB-SLAM3 jointly optimizes SE(3) camera poses and 3D landmark positions through g2o; LIO-SAM runs the LM algorithm over GTSAM's factor graph; and even DROID-SLAM, which predicts optical-flow updates using a GRU network, passes its estimates through a final bundle-adjustment layer built on the Schur complement trick.

Over the decades, Lie groups and factor graphs have replaced the matrix notation of 1999, and neural networks have taken over parts of feature matching and flow estimation. Still, the core optimization pattern endures: visual BA balances multi-view reprojection errors to refine poses and structure, while LiDAR systems swap in sensor-appropriate residuals such as point-to-point or point-to-plane distances. Pulfrich's glass plate has become a digital sensor and hand calculators have given way to GPUs, yet the underlying estimation problem remains remarkably intact.

That continuity cuts both ways. On one hand, SLAM inherits decades of mature optimization theory and battle-tested solvers; on the other, when real-world scenes violate bundle adjustment's classical assumptions—a static world, discrete point features, and Gaussian noise—that inherited machinery can break down unless those assumptions are explicitly re-examined.

---

> 📜 **Prediction vs. outcome.** Triggs et al. (1999) identified scaling BA to massive problems (thousands of cameras and millions of points) as the defining challenge ahead. Over the next twenty years, SfM systems reached that scale—from Snavely's Photo Tourism (2006) assembling hundreds of internet photos to COLMAP (2016) standardizing robust incremental SfM. Yet scaling required more than just enlarging the linear solver; it depended on an outer engineering layer of incremental BA, visibility-graph pruning, and vocabulary-tree loop closure.

---

## Still open

**Global optimum guarantees for nonlinear BA.** Because the LM algorithm is a local optimizer, a poor initial guess can trap it in the wrong 3D reconstruction. Geometric initialization tools such as the 5-point algorithm, PnP, and epipolar-geometry estimation help place the solver in the right basin, though these minimal solvers must be distinguished from the robust estimation wrappers around them—such as running RANSAC to reject outliers before applying iterative refinement. Convex-relaxation methods that mathematically certify a global optimum in large-scale scenes are an active area of research, yet they still cannot match the speed and scale required for real-time SLAM.

**The gap between photogrammetric accuracy and Visual SLAM.** Accuracy in aerial photogrammetry is tightly governed by controlled imaging conditions and product specifications: calibrated metric cameras, high-precision ground control points (GCPs), and unconstrained offline computation. Real-time Visual SLAM applies the same optimization machinery in GPS-denied settings with consumer cameras and strict per-frame latency budgets. Any meaningful comparison between the two disciplines must therefore separate image-coordinate residuals from ground-coordinate accuracy and explicitly state the range, resolution, control-point conditions, and evaluation criteria involved.

---

Bundle adjustment's core assumptions—a static world, point features, and Gaussian noise—begin to fray the moment a camera encounters moving objects; surveyors, after all, measured bridges rather than robot soccer fields. Even in a static scene, photogrammetry's legacy handed computer vision a decisive prerequisite question: before bundle adjustment can align anything, which pixels across a sequence of moving images should serve as the "corresponding points"?
