# Ch.6 — The Graph SLAM Revolution

In a basement corridor at Carnegie Mellon in 1997, Feng Lu and Evangelos Milios were trying to align multiple laser scans into a globally consistent map. The EKF was the default, but they modeled relative measurements between poses as a graph and ran least-squares optimization on it. This produced global consistency without a Kalman filter. Lu and Milios were not alone. More than a decade earlier, [Chatila and Laumond (1985)](https://www.semanticscholar.org/paper/Position-referencing-and-consistent-world-modeling-Chatila-Laumond/c34a678e40a7d80cb3683f07fc837179fd9bf3ee) at LAAS had discussed reference frames and consistent world models for mobile robots in the language of smoothing. In 1999, [Gutmann and Konolige](https://www.semanticscholar.org/paper/Incremental-mapping-of-large-cyclic-environments-Gutmann-Konolige/3c1bda51b8ca59f1836ed1b96c485d905804989a) applied pose graph matching to incremental mapping of large cyclic environments, and in the early 2000s Thrun's group formalized the approach as the *full SLAM* problem and put it on a commercial trajectory. [Folkesson and Christensen (2004)](http://www.hichristensen.net/hic-papers/folkesson-icra2004.pdf), Konolige, and Dellaert followed with formulations of their own. Lu-Milios 1997 remains the most cited because it presented a complete pipeline ("laser scan matching plus batch least-squares"), not because it opened the direction alone. If Smith and Cheeseman supplied the mathematical basis for probabilistic mapping and Davison demonstrated real-time monocular SLAM, these contributors made parallel moves that reframed SLAM as graph inference. EKF-SLAM in the 2000s met an $O(N^2)$ covariance-update bottleneck as landmark count grew, while Klein and Murray's PTAM (2007) separately demonstrated real-time optimization through a BA-based keyframe structure and split tracking and mapping. Laboratories at CMU, LAAS, Stanford, and KTH had already been developing graph-smoothing alternatives to filtering.

---

## 6.1 From Laser Scans to Pose Graphs: Lu-Milios 1997

Before [Lu & Milios 1997, "Globally Consistent Range Scan Alignment"](https://doi.org/10.1023/A:1008854305733) appeared, alignment of successive laser scans was often handled by stitching together local matches from the ICP (Iterative Closest Point) family. ICP aligned two scans well locally, but as drift accumulated the map twisted after tens of meters. When the robot came back to close a loop, the starting point and the map no longer matched.

Lu and Milios represented the robot's pose sequence $x_1, x_2, \ldots, x_n$ as nodes and each relative measurement between poses as an edge. Map building then becomes energy minimization on the graph. Each edge carries the relative transform $\hat{z}_{ij}$ between two poses and its uncertainty $\Omega_{ij}$. The full cost function is

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

where $h(x_i, x_j)$ computes the expected relative transform from the two poses, $z_{ij}$ is the actual measured relative transform, and $\Omega_{ij} = \Sigma_{ij}^{-1}$ is the information matrix, the inverse of the measurement uncertainty.

Loop closures fit this formulation naturally. When the robot revisits a place and obtains a new relative measurement, one edge adds the constraint to the graph, and full optimization adjusts every pose accordingly. An EKF updated the covariance at $O(N^2)$ cost to close a loop; a pose graph expresses the constraint with an additional edge, but still requires reoptimization to update its estimates.

> 🔗 **Borrowed.** The Lu-Milios formulation of pose graph optimization rests on the nonlinear least-squares algorithms of [Levenberg (1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/) and [Marquardt (1963)](https://www.stat.cmu.edu/technometrics/70-79/VOL-14-03/v1403757.pdf). A numerical optimization technique developed decades earlier for nonlinear parameter estimation arrived at the backend of indoor laser mapping.

The Lu-Milios solution was a batch linear system that solved for all poses at once, growing with the number of scans. It was closer to a proof of concept than a field-ready system, but it showed that optimization rather than filtering could provide global consistency. During the same period, Gutmann-Konolige emphasized incrementality, Folkesson-Christensen the robustness of data association, and Thrun's group application at real-world scale. Each developed a different part of the same conclusion.

---

## 6.2 Discovering Sparsity: The Information Matrix and the Pose-Graph Extension

In the five years after the Lu-Milios idea was published, several groups pushed extensions in the same direction. The common discovery was the **sparsity** of the information matrix ($\Omega = \Sigma^{-1}$).

The covariance matrix $\Sigma$ of EKF-SLAM is dense. Every time the robot observes a new landmark, its correlation with every existing landmark is updated. With the robot pose marginalized and $n$ 2D landmarks, $\Sigma$ is a $2n \times 2n$ matrix and the update cost is $O(n^2)$. That is why real-time performance collapsed around 100 landmarks.

The information matrix of a pose graph is different. A nonzero term appears in the $(i,j)$ block of $\Omega$ only when poses $x_i$ and $x_j$ are directly connected by a measurement. Under continuous motion, edges connect only nearby poses; distant poses have no direct connection. $\Omega$ therefore has a banded sparse structure that reflects the graph topology. For a trajectory without loop closures, the structure is nearly tridiagonal.

Sebastian Thrun's group, through the [Sparse Extended Information Filter (SEIF)](http://www.cs.cmu.edu/~thrun/papers/thrun.tr-seif02.pdf), and Edwin Olson began exploiting this sparsity explicitly. A sparse linear solver could reduce computation far below $O(n^2)$. Actual complexity depended on graph structure, but $O(n \log n)$ became possible in realistic scenarios where a robot moves within a bounded region.

> 🔗 **Borrowed.** Thrun's sparse information filter (SEIF) and [Eustice's exactly sparse delayed-state filter](https://web.mit.edu/2.166/www/handouts/eustice_et_al_ieeetro_2006.pdf) showed that information-matrix sparsity was usable even in filter form. This insight set the stage for Dellaert's factor graph formulation and the Bayes tree data structure.

At ICRA 2006, [Olson, Leonard, and Teller](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf) presented a stochastic-gradient method for optimizing pose graphs. It offered no convergence guarantee but ran fast enough on graphs of hundreds of nodes, and Olson's implementation spread through the community.

---

## 6.3 Factor Graphs and Square Root SAM

In 2006, Dellaert and his doctoral student Kaess published [Square Root SAM](https://doi.org/10.1177/0278364906072768), giving SLAM backends a new representation. Dellaert had worked on probabilistic graphical models at Georgia Tech. He treated SLAM as Bayesian inference and represented that inference on a factor graph.

In a **factor graph**, a bipartite graph with variable and factor nodes, the variable nodes are robot poses and landmark positions, while factor nodes represent observations or priors. A factor $f_k(x_{i_1}, x_{i_2}, \ldots)$ expresses a probabilistic constraint among the variables it connects. The full joint probability is

$$p(X) \propto \prod_k f_k(X_k)$$

and MAP estimation finds the $X^*$ that maximizes this probability. Under Gaussian factors, this becomes a nonlinear least-squares problem.

The least-squares structure supplied Dellaert's key observation. Applying QR decomposition to the Jacobian matrix $J$ leaves an upper-triangular matrix $R$. Because $R^T R = J^T J = \Omega$, $R$ is the "square root information matrix." Its sparsity depends not only on the Jacobian but also on variable-elimination order and factor-graph topology. A good ordering (e.g., AMD, COLAMD) minimizes fill-in and yields a sparse $R$.

This formulation is numerically more stable than the EKF covariance update. The full map of landmarks and poses can be optimized together in a consistent way, and a loop closure is expressed as the addition of a new factor.

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM and iSAM2: Online Incremental Inference

Square Root SAM was a batch method. Recomputing the full decomposition of $J^T J$ whenever a new observation arrived added substantial work. Dense factorization costs $O(n^3)$; sparse costs depend on connectivity and elimination order.

In 2008, [Kaess, Ranganathan, and Dellaert published **iSAM** (incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf), which updated the factorization with Givens rotations. When a new variable and factor were added, iSAM appended only the new rows and updated $R$ rather than recomputing the QR decomposition.

iSAM1's intrinsic limit was its relinearization schedule. The $R$ obtained by linearizing nonlinear factors is only a first-order approximation near the current estimate. As the robot moved away from the linearization point, approximation error accumulated. iSAM1 responded with **periodic full relinearization**: every few dozen steps, it relinearized the entire factor graph and recomputed the QR decomposition. Fill-in within $R$ after loop closures signaled when to run this batch step. An apparently incremental algorithm therefore reverted to batch processing every cycle.

In 2012 [iSAM2](https://doi.org/10.1177/0278364911430419) solved this problem with a data structure called the Bayes tree. The Bayes tree is a tree built from the chordal Bayes net obtained by applying variable elimination to the factor graph. Its nodes are the cliques of the Bayes net, and its edges are the separators (shared variables between cliques). When a new factor is added, the cliques affected in the Bayes tree are identified, and only that subtree is turned back into a factor graph, relinearized, and reoptimized. The core is **fluid relinearization**. Only factors whose linearization error exceeds a threshold are selectively relinearized, and the effect propagates through Bayes tree separators only as far as needed. iSAM1's "everything, every cycle" schedule was replaced by "only the necessary factors, only the affected cliques". Even when a loop closure occurred, the set of connected cliques was often locally bounded, and full recomputation could be avoided.

> 🔗 **Borrowed.** The Bayes tree extends the junction tree (join tree) lineage in probabilistic graphical models, represented by the elimination-order and chordal-graph inference techniques in textbooks such as Koller-Friedman's [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/). It brought a technique from AI inference into real-time robot SLAM.

iSAM2 was packaged as the [GTSAM (Georgia Tech Smoothing and Mapping)](https://gtsam.org) library, with a C++ core and Python bindings. GTSAM development continued while Dellaert held his Georgia Tech position and also worked with Google. Its public documentation and examples cover applications including autonomous driving, drones, and robot-arm calibration.

---

## 6.5 g2o: A General Graph Optimizer in the ROS Ecosystem

While the Georgia Tech group refined the theory, Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, and Wolfram Burgard at TUM (Munich) and Freiburg built a practical open-source implementation. At ICRA 2011, they presented [g2o](https://doi.org/10.1109/ICRA.2011.5979949) (general graph optimization), designed to "handle any kind of graph optimization in a plug-in manner." The author list bridged Burgard and Grisetti's Freiburg robotics tradition, Strasdat's monocular SLAM experience, and Konolige's industrial engineering perspective.

The g2o design separates three concepts. Vertices (variable nodes) and edges (factors or constraints) form the graph, and a solver handles the sparse linear system. The user defines vertex types and the error function and Jacobian of edges, and g2o then runs the full optimization with Gauss-Newton or Levenberg-Marquardt. The sparse solver can be chosen among Cholmod, CSparse, and Eigen, or swapped out for an external library.

As ROS (Robot Operating System) spread through mobile-robot research in the early 2010s, g2o became one of the leading implementations for graph-based SLAM. ORB-SLAM and LSD-SLAM adopted it, but ROS SLAM did not converge on one backend: gmapping belongs to the particle-filter line, and Cartographer uses Ceres. g2o was influential without being universal.

---

## 6.6 Why the Field Converged Here

Chatila-Laumond (1985), Lu-Milios (1997), Gutmann-Konolige (1999), Folkesson-Christensen (2004), Thrun's group, Dellaert (2006), and Kaess (2012) started from different problems and developed tools for maintaining and solving graph constraints.

The shift changed the model of the problem, not only the algorithm. EKF-SLAM maintains the best estimate of the current state and its uncertainty while marginalizing away the past. Past poses disappear, and accumulated error remains inside the current estimate. Closing a loop then requires a costly update to the current covariance.

Graph SLAM retains the past. Poses, landmarks, and observations remain in the graph, and a loop closure becomes a new edge. Reoptimization adjusts the full trajectory consistently (for methods that use a time-continuous trajectory rather than discrete keyframes, see Ch.7c Continuous-Time SLAM). Keeping past poses revisable is the essential difference from filters.

Costs differ as well. EKF's update cost is $O(N^2)$ (in the number of landmarks $N$), and information storage is $O(N^2)$. Graph methods can reduce that complexity substantially with sparse Cholesky (or QR) decomposition. Update cost depends on graph connectivity, fill-in during factorization, elimination order, and the region that must be recomputed. Motion within a bounded physical region alone does not guarantee $O(N \log N)$ updates.

> 📜 **Prediction vs. outcome.** The limits of Dellaert's batch Square Root SAM (2006) led the same group toward incremental methods. iSAM handled the problem in 2008 with Givens-rotation updates, and iSAM2 improved loop-closure efficiency in 2012 with the Bayes tree. GTSAM, Ceres, and g2o all handle nonlinear least squares, but differ in their solvers and incremental data structures. The three papers resolved one problem in stages, largely along the predicted path.

The flexibility of marginalization played a part as well. When an old pose in the graph is marginalized, its information is preserved as a linking factor among the remaining variables. Filters also remove past states while transferring their information to the current estimate. Both approaches must account for approximation and relinearization constraints introduced by this compression. Engineering trade-offs like sliding-window optimization and keyframe selection come in here.

---

## 6.7 Nonlinearity and Robustness: The Layer of Practical Engineering

Implementation is less tidy than the theory. Much of 2010s SLAM engineering went into closing that gap.

The first problem is dependence on the initial value. Gauss-Newton or LM optimization converges to a local minimum when the initial pose estimate is far from the truth. A wrong loop-closure correspondence corrupts that initialization, making verification and outlier rejection central pre-backend tasks. Ch.6b (Certifiable SLAM) treats convex-relaxation methods (SDP) that avoid this local-minimum problem and certify global optimality.

Standard least squares is fragile to outliers, as practice quickly revealed. Robust Huber or Cauchy losses reduce the influence of wrong matches, and both g2o and GTSAM make them selectable. The choice depends on the environment and sensor, and as of 2026 still rests on the engineer's experience.

The third problem is marginalization approximation. iSAM2's Bayes tree provides exact incremental inference, but the tree grows with the variable count. Real systems marginalize old poses to keep it manageable, and the resulting fill-in can make the information matrix dense. Implementation quality depends on how this fill-in is truncated and approximated with a prior factor.

> 📜 **Prediction vs. outcome.** g2o's generality lies in allowing users to define states as vertices and observation constraints as edges. New line, plane, inertial, or object constraints can be implemented through that interface; examples must be checked against the estimator and edge implementation actually used by each system. Ch.7b describes preintegration, the standard way to place IMU factors in the graph. As of 2026, however, g2o itself prioritizes interface stability and compatibility with existing users over broad expansion of its built-in factors. Users commonly add new factor types through inheritance, forks, or wrappers.

---

## 🧭 Still open

Which robust kernel to choose. Huber, Cauchy, Geman-McClure, DCS, and others are available, but no principled method decides in advance which kernel is optimal for a given environment and sensor. The choice still rests on engineering judgment. Researchers are learning cost functions themselves, but integration into an online incremental system remains unresolved.

Representing non-Gaussian uncertainty inside a factor graph remains open. The basic continuous optimization in GTSAM and g2o starts from Gaussian residual models, although robust kernels, max mixtures, and hybrid factors provide extensions. Accurately representing loop-closure mismatch probabilities and multi-hypothesis poses in real time still lacks a general solution.

The Bayes tree is most efficient when a new factor affects only local cliques. As loop-closure constraints become dense in a large map, the affected subtree and fill-in can grow, increasing computation and memory. Hierarchical management and submap partitioning are ways to control that growth.

---

By the early 2010s, the backend debate had quieted. As tools such as g2o and GTSAM became widely used, attention moved to the layers above them. The question changed from "how to close a loop" to "which features can recognize one, and from how far away?" The front end became the new focus of competition.

One question remained unresolved: do g2o and GTSAM actually return the global minimum? Ch.6b treats that question of certifiability before the front-end lineage resumes in Ch.7.
