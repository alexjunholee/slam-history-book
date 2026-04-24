# Ch.1 — Photogrammetry and Bundle Adjustment: The 100 Years Before Triggs

The skeleton of today's SLAM optimization backend was born in German surveying. In the early twentieth century, Carl Pulfrich's method of hand-computing two-view triangulation on glass plates combined with Albrecht Meydenbauer's photogrammetric system to form a single surveying tradition. That tradition passed through Duane C. Brown's numerical formulation in 1958, and in 1999 Bill Triggs, Philip McLauchlan, Richard Hartley, and Andrew Fitzgibbon translated it into the language of computer vision. Bundle adjustment was not an invention in itself but the work of carrying a century-old surveying inheritance into a language the computer vision community could use. Triggs et al. (1999) inherited the parallax principle from Pulfrich's geometry and the reprojection formulation from Brown's (1958) military surveying. The solver skeleton came from Levenberg-Marquardt.

---

## 1. Early twentieth-century glass plates and stereophotogrammetry

In 1901, [Carl Pulfrich](https://en.wikipedia.org/wiki/Carl_Pulfrich) presented the **stereocomparator**, built by the Zeiss optical works, at the Hamburg conference of natural scientists (this was the formal unveiling, following a prototype stereoscopic rangefinder shown in Munich in 1899). The device photographed the same point from two camera viewpoints and computed distance by reading the coordinate difference on the glass plates. The principle was simple: the parallax between two views is inversely proportional to depth. The mathematics was Greek-era trigonometry; what was new was the precision of the optical instrument.

A generation earlier, [Albrecht Meydenbauer](https://de.wikipedia.org/wiki/Albrecht_Meydenbauer) had systematized **architectural photogrammetry** for the preservation of buildings. In 1858, after a fall while surveying the exterior of Wetzlar Cathedral, he conceived of using photographs in place of direct measurement. In 1885 he founded the Royal Prussian Photogrammetric Institute (Königlich Preussische Messbild-Anstalt).

These two streams joined into the tradition that carried into twentieth-century aerial surveying — aerotriangulation, in which a plane photographed the terrain from above and two-view photographs yielded three-dimensional maps. It was the age of hand calculators.

> 🔗 **Borrowed.** Modern SLAM's stereo depth estimation runs on the same principle as Pulfrich's stereocomparator. Depth comes from the baseline between two cameras and the parallax. The glass plate of 125 years ago has only become a pixel array.

---

## 2. 1958, Brown, and numerical bundle adjustment

What Pulfrich and Meydenbauer had solved with optical instruments, Brown moved into equations.

[Duane C. Brown](https://digital.hagley.org/08206139_solution) was a surveying engineer inside the United States Air Force ballistic missile development program. He worked on the problem of jointly estimating satellite orbits and ground coordinates — that is, simultaneously optimizing many camera viewpoints and many ground control points.

In his 1958 report "A Solution to the General Problem of Multiple Station Analytical Stereotriangulation" (RCA-MTP Data Reduction Technical Report No. 43, AFMTC-TR-58-8), Brown left one of the early documents that formulated **bundle adjustment (BA)** numerically (Helmut Schmid is named as a co-inventor from the same period).

The core is the **reprojection error**. Minimize the difference between the 2D image coordinate $x_{ij} \in \mathbb{R}^2$ observed in camera $i$ and the predicted coordinate $\pi(K_i, R_i, t_i, X_j)$ obtained by projecting the 3D point $X_j \in \mathbb{R}^3$ through the intrinsic matrix $K_i$ and extrinsic matrix $[R_i | t_i]$:

$$E = \sum_{i,j} \| x_{ij} - \pi(K_i, R_i, t_i, X_j) \|^2$$

The name "bundle" comes from the bundle of rays extending out from each camera center to the observed 3D points. Camera poses and point locations are adjusted together so that those rays meet at the 3D points. It took forty years for a technique that began in military and intelligence applications to be absorbed into academia.

> 🔗 **Borrowed.** Bundle techniques from the satellite geolocation field entered the computer vision community in the 1990s. During the years they were held under military classification, academia independently rediscovered the same problem. Triggs 1999 is the confluence point of those two streams.

---

## 3. Levenberg and Marquardt — pioneers of nonlinear optimization

Brown had the objective function to minimize in hand; the tool that would actually solve it came from somewhere else entirely.

Reprojection error minimization is a nonlinear least-squares problem. There is no analytical solution, so iterative numerical optimization is needed.

In 1944, [Kenneth Levenberg](https://cs.uwaterloo.ca/~y328yu/classics/levenberg.pdf) published a method that interpolated between Gauss-Newton and steepest descent with a damping parameter $\lambda$. Larger $\lambda$ moves toward steepest descent for safe convergence; smaller $\lambda$ uses the fast convergence of Gauss-Newton. The strategy is expressed by adding $\lambda \mathbf{I}$ to the objective function, improving numerical stability. It was twenty years ahead of computer vision. In 1963, [Donald Marquardt](https://epubs.siam.org/doi/10.1137/0111030) independently rediscovered the same idea and formulated it more explicitly. The name settled as the **Levenberg-Marquardt (LM) algorithm**.

It took about another thirty-five years for the LM algorithm to become the standard BA solver in computer vision, not because the technology was missing but because of the walls between fields.

---

## 4. 1999, Triggs et al. — a hundred years of inheritance integrated

Thirty-five years after Levenberg-Marquardt had readied the numerical tool, computer vision finally picked it up.

At the 1999 Vision Algorithms Workshop, Bill Triggs, Philip McLauchlan, Richard Hartley, and Andrew Fitzgibbon presented ["Bundle Adjustment — A Modern Synthesis"](https://link.springer.com/chapter/10.1007/3-540-44480-7_21).

What this paper did was translate and synthesize the BA theory scattered across twentieth-century surveying and aerial photogrammetry into the language of the computer vision community. Triggs et al. contributed two things. First, they made the structural properties of sparse BA explicit. Using the sparse block structure of the Hessian matrix (the Schur complement trick), joint camera-point optimization can be performed far more efficiently. Second, they treated gauge freedom (the arbitrariness of the reference frame) explicitly.

Seven years after this paper, Noah Snavely's [Photo Tourism (2006)](https://phototour.cs.washington.edu/Photo_Tourism.pdf) automatically reconstructed famous landmarks like Notre-Dame and the Trevi Fountain from hundreds of photographs scattered across the Internet. Ten years after that, Johannes Schönberger's [COLMAP (2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) open-sourced robust incremental structure from motion (SfM) at the scale of tens to hundreds of thousands of images, bringing a research stream that had already reached the million-image range into a tool anyone could reproduce. Without Triggs's language, that path would have been much slower.

---

## 5. Reprojection Error — the formation of the concept

If Triggs et al. described which error function was being minimized, it is worth tracing separately how that function itself settled into its current form.

There were two transitions before this error function took its present shape.

Early twentieth-century aerial triangulators measured error as "distance difference in the ground coordinate frame." Because the comparison was made directly in 3D space, a misaligned camera lens or poor calibration would dissolve into the ground coordinate residual and become invisible.

Brown moved the object of comparison to the image plane in his 1958 report. The method matches "the projected location of a 3D point in the image" to "the actual image observation" in pixel units. Calibration error, lens distortion, and extrinsic parameter error all surface together in one residual. It is also cleaner statistically. Camera image noise can be modeled as an isotropic Gaussian in pixel units, and under that model reprojection error minimization becomes maximum likelihood estimation.

Triggs et al. (1999) polished that formulation into the language of computer vision textbooks and standardized it. This reprojection error minimization is, as of 2026, the core measurement function of factor graph-based SLAM backends.

> 🔗 **Borrowed.** The observation model for a visual landmark in SLAM, $z = \pi(K, T, p) + \epsilon$, directly inherits Brown's (1958) reprojection formula. A SLAM backend that minimizes this with Gauss-Newton has the same mathematical structure as a 1958 aerial triangulation solver.

---

## 6. The skeleton of the SLAM backend — through 2026

[Durrant-Whyte and Leonard's 1995 survey](https://ieeexplore.ieee.org/document/476131) fixed the acronym "SLAM" as standard terminology, but the mathematics of that backend inherits Brown's 1958 reprojection formulation, traced in this chapter, almost unchanged. Look at today's SLAM optimization backends. ORB-SLAM3 jointly optimizes SE(3) poses and 3D landmark locations through g2o. LIO-SAM runs the LM algorithm on top of GTSAM's factor graph. DROID-SLAM gets its update direction from GRU-based optical flow, but the final bundle adjustment layer still uses the Schur complement trick.

Lie groups and factor graphs replaced the matrix notation of 1999, and neural networks took over descriptor computation, but the substance of the computation is unchanged. The reprojection error of points observed from multiple viewpoints is minimized to estimate camera poses and the map together. Pulfrich's glass plate has become a pixel array, and hand calculation has become the GPU — that is all.

This continuity is the field's strength and its weakness. Strength: a hundred years of convergence proofs and practical validation come along for free. Weakness: when BA's assumptions (static world, point features, Gaussian noise) break against the real environment, there is no alternative in hand.

---

> 📜 **Prediction vs. outcome.** Triggs et al. (1999) named scaling BA to large problems (thousands of cameras, millions of points) as the main challenge. That direction was achieved over the following twenty years. In 2006, Snavely's Photo Tourism reconstructed landmarks from hundreds of Internet photographs; in 2016, COLMAP standardized the robust incremental SfM implementation of that line. It was not, however, the "direct scaling" Triggs imagined. What arrived was an engineering layer — incremental BA and visibility graph pruning, with vocabulary tree loop closure on top. `[hit]`

---

## 🧭 Still open

**Global optimum guarantees for nonlinear BA.** The LM algorithm converges to a local minimum. With a bad initial value, it converges to the wrong structure. Methods for initialization (the 5-point algorithm, PnP, epipolar geometry estimation) appeared in turn, but these themselves depend internally on RANSAC and iterative optimization. Convex relaxation approaches that guarantee a global optimum in large-scale environments are being researched, but they are not yet practical at the speed and scale of real-time SLAM.

**The gap between photogrammetric accuracy and Visual SLAM.** Aerial photogrammetry standardly demands subpixel (below 0.1 pixel) accuracy. It has calibrated cameras and high-quality GCPs (ground control points), and the optimization runs offline. Real-time Visual SLAM uses the same formulation but operates under the constraints of GPS-denied environments, low-resolution cameras, and immediate estimation. The environments in which Visual SLAM systematically reaches the surveying field's accuracy standard (RMSE < 5 cm at 500 m range) are limited, and attempts to unify the accuracy standards of the two fields in a single framework are ongoing.

---

BA's assumptions (static world, point features, Gaussian noise) begin to break when the camera meets a moving object. Surveyors measured bridges, not robot soccer fields. The inheritance passed to computer vision in the form of a single question: which pixels in a moving image should serve as the "corresponding points" that bundle adjustment would later receive?
