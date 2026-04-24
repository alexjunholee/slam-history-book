# Ch.6 — The Graph SLAM Revolution

A basement corridor at Carnegie Mellon, 1997. Feng Lu and Evangelos Milios were wrestling with the problem of aligning multiple laser scans into a globally consistent map. EKF was the default choice, but the two of them took a different path. They modeled the relative measurements between poses directly as a graph, and ran least-squares optimization on that graph. The result was the kind of global consistency the Kalman family had never reached. Lu-Milios were not the sole founders of this direction. More than a decade earlier, [Chatila and Laumond (1985)](https://www.semanticscholar.org/paper/Position-referencing-and-consistent-world-modeling-Chatila-Laumond/c34a678e40a7d80cb3683f07fc837179fd9bf3ee) at LAAS had already discussed reference frames and consistent world models for mobile robots in the language of smoothing; in 1999, [Gutmann and Konolige](https://www.semanticscholar.org/paper/Incremental-mapping-of-large-cyclic-environments-Gutmann-Konolige/3c1bda51b8ca59f1836ed1b96c485d905804989a) applied pose graph matching to incremental mapping of large cyclic environments; and in the early 2000s Thrun's group formalized the approach as the *full SLAM* problem and put it on a commercial trajectory. [Folkesson and Christensen (2004)](http://www.hichristensen.net/hic-papers/folkesson-icra2004.pdf), Konolige, and Dellaert followed with formulations of their own. The reason Lu-Milios 1997 is the most cited today is that it presented a complete pipeline ("laser scan matching plus batch least-squares") in finished form, not that it opened the direction alone. If Smith-Cheeseman laid the mathematical foundation of probabilistic mapping and Davison proved the feasibility of real-time monocular SLAM, these parallel contributors were, nearly simultaneously, making several different moves that redefined SLAM as a graph inference problem. Klein and Murray's PTAM (2007) had split tracking and mapping to achieve real-time performance, but as hundreds of poses accumulated, the $O(N^2)$ update cost of the EKF backend became the bottleneck. The answer to that problem had been in preparation for a decade, in its own forms, in basement corridors at CMU and in labs at LAAS, Stanford, and KTH.

---

## 6.1 From Laser Scans to Pose Graphs: Lu-Milios 1997

Before [Lu & Milios 1997, "Globally Consistent Range Scan Alignment"](https://doi.org/10.1023/A:1008854305733) appeared, alignment of successive laser scans was often handled by stitching together local matches from the ICP (Iterative Closest Point) family. ICP aligned two scans well locally, but as drift accumulated the map twisted after tens of meters. When the robot came back to close a loop, the starting point and the map no longer matched.

The Lu-Milios idea was simple. Represent the robot's pose sequence $x_1, x_2, \ldots, x_n$ as nodes, and the relative measurement between each pose pair as an edge; then the map-building problem becomes an energy minimization on the graph. Each edge carries the relative transform $\hat{z}_{ij}$ between two poses and its uncertainty $\Omega_{ij}$. The full cost function is

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

where $h(x_i, x_j)$ computes the expected relative transform from the two poses, $z_{ij}$ is the actual measured relative transform, and $\Omega_{ij} = \Sigma_{ij}^{-1}$ is the information matrix, the inverse of the measurement uncertainty.

The core of this formulation is the natural inclusion of loop closures. When the robot later revisits the same place and obtains a new relative measurement, adding it to the graph as an edge makes the full optimization adjust all poses to reflect that constraint. In EKF, closing a loop was a heavy operation that updated the covariance at $O(N^2)$ cost. In a pose graph, adding a single edge is enough.

> 🔗 **Borrowed.** The Lu-Milios formulation of pose graph optimization rests on the nonlinear least-squares algorithms of [Levenberg (1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/) and [Marquardt (1963)](https://www.stat.cmu.edu/technometrics/70-79/VOL-14-03/v1403757.pdf). A numerical optimization technique developed decades earlier for nonlinear parameter estimation arrived at the backend of indoor laser mapping.

The Lu-Milios solution of the time was a batch linear system that solved for all poses at once. As the number of scans grew, the size of the linear system grew with it. So it was closer to a proof of concept than a field-ready system. It did, however, show two things clearly. Global consistency is achievable. And the tool is optimization, not a filter. In the same period Gutmann-Konolige emphasized incrementality, Folkesson-Christensen robustness of data association, and Thrun's group application at real-world scale — each carving a different facet of the same conclusion.

---

## 6.2 Discovering Sparsity: The Information Matrix and the Pose-Graph Extension

In the five years after the Lu-Milios idea was published, several groups pushed extensions in the same direction. The common discovery was the **sparsity** of the information matrix ($\Omega = \Sigma^{-1}$).

The covariance matrix $\Sigma$ of EKF-SLAM is dense. Every time the robot observes a new landmark, its correlation with every existing landmark is updated. With the robot pose marginalized and $n$ 2D landmarks, $\Sigma$ is a $2n \times 2n$ matrix and the update cost is $O(n^2)$. That is why real-time performance collapsed around 100 landmarks.

The information matrix of a pose graph is different. A nonzero term appears in the $(i,j)$ block of $\Omega$ only when poses $x_i$ and $x_j$ are in a direct measurement relation. Under continuous motion only nearby poses are connected by edges; distant poses are not connected directly. $\Omega$ has a banded sparse structure that reflects the graph topology. In a pure driving scenario without loop closures the structure is nearly exactly tridiagonal.

Sebastian Thrun's group's [Sparse Extended Information Filter (SEIF)](http://www.cs.cmu.edu/~thrun/papers/thrun.tr-seif02.pdf) and Edwin Olson's work started exploiting this sparsity explicitly. With a sparse linear solver, the computational cost could drop far below $O(n^2)$. The actual complexity depended on the graph structure, but in realistic scenarios where the robot moves within a bounded region, $O(n \log n)$ became reachable.

> 🔗 **Borrowed.** Thrun's sparse information filter (SEIF) and [Eustice's exactly sparse delayed-state filter](https://web.mit.edu/2.166/www/handouts/eustice_et_al_ieeetro_2006.pdf) showed that the sparsity of the information matrix was usable even in filter-based form. This sparsity insight frames the context that leads into Dellaert's factor graph formulation and the Bayes tree data structure.

At ICRA 2006, [Olson, Leonard, and Teller](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf) presented a method for optimizing pose graphs with stochastic gradient descent. There was no convergence guarantee. It still ran fast enough on graphs of hundreds of nodes, and Olson's implementation spread across the community afterward.

---

## 6.3 Factor Graphs and Square Root SAM

In 2006, Dellaert and his then-PhD-student Kaess published [Square Root SAM](https://doi.org/10.1177/0278364906072768), which rewrote the way SLAM backends were understood. Dellaert had been working on probabilistic graphical models at Georgia Tech. He saw SLAM as a Bayesian inference problem and judged that performing that inference on a factor graph was the most natural form.

In a **factor graph** (a bipartite graph that represents variables and constraints as nodes and edges), variable nodes are the robot poses and landmark positions, and factor nodes are observations or priors. A factor $f_k(x_{i_1}, x_{i_2}, \ldots)$ expresses a probabilistic constraint among the variables it connects. The full joint probability is

$$p(X) \propto \prod_k f_k(X_k)$$

and MAP estimation finds the $X^*$ that maximizes this probability. Under Gaussian factors, this becomes a nonlinear least-squares problem.

Dellaert's insight came from the structure of this least-squares problem. Applying QR decomposition to the Jacobian matrix $J$ leaves an upper triangular matrix $R$. $R^T R = J^T J = \Omega$, and $R$ is the "square root information matrix". The sparse structure of this $R$ is determined not by the Jacobian itself but by the variable elimination order and the factor graph topology. Choosing a good ordering (e.g., AMD, COLAMD) minimizes fill-in and yields a sparse $R$.

This formulation is numerically more stable than the EKF covariance update. The full map of landmarks and poses can be optimized together in a consistent way, and a loop closure is expressed as the addition of a new factor.

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM and iSAM2: Online Incremental Inference

Square Root SAM was a batch method. Redoing the full decomposition of $J^T J$ every time a new observation came in cost $O(n^3)$. That was not practical in an online robot system.

In 2008, [Kaess, Ranganathan, and Dellaert published **iSAM** (incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf), which approached this problem with Givens rotations. When a new variable and factor were added, instead of redoing the QR decomposition from scratch, only the new rows were appended and $R$ was updated via Givens rotations.

The intrinsic limit of iSAM1 was the relinearization schedule. The $R$ obtained by linearizing nonlinear factors is only a first-order approximation near the current estimate. As the robot moved and the estimate drifted from the linearization point, approximation error accumulated. iSAM1's response was **periodic full relinearization**. Every few dozen steps, the entire factor graph was relinearized from scratch and the QR decomposition was redone from scratch. Fill-in in $R$ caused by loop closures damaging the sparse structure was the visible symptom that triggered this schedule, but the root of the cost was the schedule itself — "periodically re-solve the whole thing". An algorithm that looked incremental reverted to a batch algorithm every cycle.

In 2012 [iSAM2](https://doi.org/10.1177/0278364911430419) solved this problem with a data structure called the Bayes tree. The Bayes tree is a tree built from the chordal Bayes net obtained by applying variable elimination to the factor graph. Its nodes are the cliques of the Bayes net, and its edges are the separators (shared variables between cliques). When a new factor is added, the cliques affected in the Bayes tree are identified, and only that subtree is turned back into a factor graph, relinearized, and reoptimized. The core is **fluid relinearization**. Only factors whose linearization error exceeds a threshold are selectively relinearized, and the effect propagates through Bayes tree separators only as far as needed. iSAM1's "everything, every cycle" schedule was replaced by "only the necessary factors, only the affected cliques". Even when a loop closure occurred, the set of connected cliques was often locally bounded, and full recomputation could be avoided.

> 🔗 **Borrowed.** The data-structural idea of the Bayes tree extends the junction tree (join tree) algorithm lineage in the probabilistic graphical models literature — the elimination-order and chordal-graph-based inference techniques covered in standard textbooks like Koller-Friedman's [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/) are representative. A line of technique from the AI inference community was transplanted into real-time robot SLAM.

iSAM2 was packaged as the [GTSAM (Georgia Tech Smoothing and Mapping)](https://gtsam.org) library. A C++ core with Python bindings on top. GTSAM development did not stop even after Dellaert later moved to Google. As of 2026 it is in effect the standard SLAM backend in areas ranging from autonomous driving and drones to robot arm calibration.

---

## 6.5 g2o: The Standard of the ROS Ecosystem

While the Georgia Tech group concentrated on refining the theory, Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, and Wolfram Burgard — at TUM (Munich) and Freiburg — concentrated on a practical open-source implementation. At ICRA 2011, they presented [g2o](https://doi.org/10.1109/ICRA.2011.5979949) (general graph optimization), designed around the principle of "handle any kind of graph optimization in a plug-in manner". The authorship itself was already hybrid. The Freiburg robotics tradition of Burgard and Grisetti, Strasdat's monocular SLAM experience, and the industrial engineering sensibility Konolige brought merged into a single framework.

The g2o design separates three concepts. Vertices (variable nodes) and edges (factors or constraints) form the graph, and a solver handles the sparse linear system. The user defines vertex types and the error function and Jacobian of edges, and g2o then runs the full optimization with Gauss-Newton or Levenberg-Marquardt. The sparse solver can be chosen among Cholmod, CSparse, and Eigen, or swapped out for an external library.

As ROS (Robot Operating System) established itself in the early 2010s as the standard platform for mobile robot research, g2o became the de facto SLAM backend standard. gmapping, Cartographer, ORB-SLAM, and LSD-SLAM adopted g2o or a g2o-like interface. A researcher starting a new pose graph SLAM implementation would consider g2o as the first option.

---

## 6.6 Why the Field Converged Here

Chatila-Laumond (1985), Lu-Milios (1997), Gutmann-Konolige (1999), Folkesson-Christensen (2004), Thrun's group, Dellaert (2006), and Kaess (2012) — several groups, each with their own tools, at different times, arrived at the same conclusion that the EKF backend alone was not enough.

The core of the shift was not a swap of algorithms but a shift in problem modeling. EKF-SLAM maintains the best estimate of the current state and its uncertainty while marginalizing away the past. In this filter paradigm, past poses disappear and accumulated error hides inside the current estimate. Closing a loop requires a heavy update on the current covariance.

Graph SLAM does not throw the past away. Poses, landmarks, and observations all remain in the graph, and a loop closure is expressed as the addition of a new edge. Reoptimization adjusts the full trajectory consistently (for lines that reformulate the graph as a time-continuous trajectory rather than discrete keyframes, see Ch.7c Continuous-Time SLAM). That past poses themselves remain revisable is the essential difference from filters.

The computational cost was also different. EKF's update cost is $O(N^2)$ (in the number of landmarks $N$), and information storage is $O(N^2)$. Graph methods, using sparse Cholesky (or QR) decomposition, can cut the complexity substantially. In a realistic scenario where the robot moves within a bounded region — a sparsely connected graph — updates at the level of $O(N \log N)$ are generally possible. In large-scale long-term SLAM, this gap is hard to close.

> 📜 **Prediction vs. outcome.** The limitation of the batch form presented by Dellaert's Square Root SAM (2006) led directly to the incremental direction in the same group. iSAM in 2008 handled it with Givens-rotation-based incremental updates, and iSAM2 in 2012 raised the efficiency even for loop closure situations using the Bayes tree. GTSAM, Ceres, and g2o all compete on the same structure. In that the three papers form a lineage that resolves the same problem statement in stages, this line is close to a trajectory realized almost as predicted. `[hit]`

The flexibility of marginalization played a part as well. When an old pose in the graph is marginalized, its information is preserved as a linking factor among the remaining variables. The filter threw information away; the graph can compress while retaining it. Engineering trade-offs like sliding-window optimization and keyframe selection come in here.

---

## 6.7 Nonlinearity and Robustness: The Layer of Practical Engineering

There is a gap between the theoretical elegance of graph optimization and the actual implementation. Closing that gap took up a good part of 2010s SLAM engineering.

The first problem is initial-value dependence. Gauss-Newton or LM optimization converges to a local minimum when the initial pose estimate is far from the true value. A wrong correspondence in a loop closure corrupts the initial values. That is why loop closure verification and outlier rejection became a core task of the pre-backend stage. The line that bypasses this local-minimum problem itself through convex relaxation (SDP) and solves it in a form that provably certifies global optimality is treated separately in Ch.6b (Certifiable SLAM).

Standard least-squares is fragile to outliers, as practice quickly revealed. Using a robust kernel like Huber or Cauchy cost reduces the influence of wrong matches. Both g2o and GTSAM make robust kernels selectable. Which kernel to use depends on environment and sensor characteristics, and as of 2026 this choice still rests on the engineer's experience.

The third problem is marginalization approximation. iSAM2's Bayes tree provides exact incremental inference, but as the variable count grows, the tree keeps growing. In real systems, old poses are marginalized to keep the tree size manageable. The fill-in generated during this marginalization can make the information matrix dense. How to truncate it, and how to approximate with a prior factor, separates implementation quality.

> 📜 **Prediction vs. outcome.** The "handle any graph optimization problem as a plug-in" generality that g2o claimed has been partly extended by systems that internally use complex geometric constraints like lines and planes (OpenVINS, the VINS-Fusion family, and so on) (for preintegration, the standard way to place IMU factors in the graph, see Ch.7b Preintegration). As of 2026, however, the g2o library itself puts more weight on interface stability and compatibility with existing users than on broad extension of the built-in factors, and new factor types are commonly stacked on from the user side via inheritance, forks, or wrappers. `[in progress]`

---

## 🧭 Still open

Which robust kernel to choose. Huber, Cauchy, Geman-McClure, DCS, and others are available, but there is no principled method for deciding in advance which kernel is optimal for a given environment and sensor. The choice still rests on the engineer's intuition and experience. There is research on optimizing the cost function itself in a learning-based way, but integrating it into an online incremental system is an unresolved problem.

Representing non-Gaussian situations inside a factor graph is still open. At present, factors in GTSAM and g2o nearly all assume Gaussian noise. Accurately representing situations like loop-closure mismatch probability and multi-hypothesis poses is hard both theoretically and computationally. Attempts like the max-mixture model exist, but there is no general solution.

The Bayes tree is efficient when the number of loop closures is small. In scenarios where a vehicle drives tens of kilometers over hours and generates thousands of loop closures, the tree structure becomes complex and memory efficiency drops. This bottleneck has been reported in GTSAM's actual application to autonomous driving data, and combining hierarchical tree management or submap partitioning is one of the current research directions.

---

Entering the 2010s, the backend debate quieted down. As g2o and GTSAM became de facto standards, researchers' attention moved to what to put on top of the backend. Rather than "how to close a loop", the question became "with which features, and from how far away, to recognize a loop". The front end was the new competitive stage.

One thread left unresolved here is whether the solutions g2o and GTSAM return are actually the global minimum. That question — certifiability — is the subject of Ch.6b, which can be read as a supplement before continuing to Ch.7. The main line proceeds to Ch.7 regardless.
