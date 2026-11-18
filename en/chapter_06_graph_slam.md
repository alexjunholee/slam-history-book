# Chapter 6. Graph SLAM

In a basement corridor at Carnegie Mellon in 1997, Feng Lu and Evangelos Milios set out to stitch a sequence of laser scans into a single globally consistent map. Although the Extended Kalman Filter was the field's reigning default, they chose a different representation altogether—modeling the relative measurements between robot poses as edges in a graph and solving for all poses simultaneously with least squares. They achieved global consistency without a Kalman filter in sight.

Lu and Milios were part of a wider shift away from filtering. More than a decade earlier, [Chatila and Laumond (1985)](https://www.semanticscholar.org/paper/Position-referencing-and-consistent-world-modeling-Chatila-Laumond/c34a678e40a7d80cb3683f07fc837179fd9bf3ee) at LAAS had framed mobile-robot spatial consistency in the language of smoothing. Shortly after Lu and Milios, [Gutmann and Konolige (1999)](https://www.semanticscholar.org/paper/Incremental-mapping-of-large-cyclic-environments-Gutmann-Konolige/3c1bda51b8ca59f1836ed1b96c485d905804989a) applied pose-graph matching to incremental mapping of large cyclic buildings, Sebastian Thrun's group cast the approach as the *full SLAM* problem and carried it into commercial robotics in the early 2000s, and [Folkesson and Christensen (2004)](http://www.hichristensen.net/hic-papers/folkesson-icra2004.pdf), Konolige, and Frank Dellaert developed complementary formulations of their own. Lu and Milios (1997) became the canonical citation not because they worked in isolation, but because they delivered a complete, clean pipeline combining laser scan matching with batch least-squares optimization. Even as EKF-SLAM struggled with its $O(N^2)$ covariance bottleneck in the 2000s and Klein and Murray's PTAM (2007) proved that real-time bundle adjustment could power monocular tracking, laboratories at CMU, LAAS, Stanford, and KTH were forging a unified graph-inference backend for SLAM.

---

## 6.1 From Laser Scans to Pose Graphs: Lu-Milios 1997

Before [Lu and Milios's 1997 paper, "Globally Consistent Range Scan Alignment"](https://doi.org/10.1023/A:1008854305733), mobile robots typically assembled laser maps by chaining together pairwise matches from Iterative Closest Point (ICP). Locally, ICP aligned consecutive scans neatly; globally, tiny registration errors compounded until corridors visibly twisted over tens of meters. When the robot completed a loop and returned to its starting hallway, the two ends of the map refused to meet.

Lu and Milios broke that chain of local fixes by representing the robot's pose history $x_1, x_2, \ldots, x_n$ as nodes in a graph and each relative spatial measurement between poses as an edge. Each edge encodes a measured relative transform $\hat{z}_{ij}$ paired with its certainty $\Omega_{ij}$, turning global map alignment into minimizing total energy over the entire graph:

$$F = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} e_{ij}, \quad e_{ij} = z_{ij} - h(x_i, x_j)$$

In this cost function, $h(x_i, x_j)$ predicts the relative transform from the current pose estimates, $z_{ij}$ is the actual measured transform, and $\Omega_{ij} = \Sigma_{ij}^{-1}$ is the information matrix—the inverse of the measurement covariance.

Under this graph view, loop closures become remarkably clean. Whenever a robot revisits a known place and registers a new scan match, a single edge drops that constraint straight into the graph, and global optimization adjusts every pose along the loop together. Where an EKF had to touch a dense covariance matrix at $O(N^2)$ cost to close a loop, a pose graph expresses the loop constraint effortlessly in one edge—though it still has to re-solve the optimization to update its pose estimates.

> 🔗 **Borrowed.** The Lu-Milios formulation of pose graph optimization rests on the nonlinear least-squares algorithms of [Levenberg (1944)](https://www.ams.org/qam/1944-02-02/S0033-569X-1944-10666-0/) and [Marquardt (1963)](https://www.stat.cmu.edu/technometrics/70-79/VOL-14-03/v1403757.pdf). A numerical optimization technique developed decades earlier for nonlinear parameter estimation arrived at the backend of indoor laser mapping.

Lu and Milios solved a batch linear system over all poses simultaneously, whose cost grew with scan count. Their system was closer to a proof of concept than a deployed real-time navigator, yet it established a vital principle: optimization—not recursive filtering—offered the natural path to global consistency. Alongside it, Gutmann and Konolige pushed incrementality, Folkesson and Christensen tackled robust data association, and Thrun's group scaled the idea to real-world environments—each filling in another piece of the same shift.

---

## 6.2 Discovering Sparsity: The Information Matrix and the Pose-Graph Extension

In the five years following Lu and Milios's paper, multiple research groups converged on the same structural revelation: unlike the covariance matrix, the **information matrix** ($\Omega = \Sigma^{-1}$) is overwhelmingly **sparse**.

In EKF-SLAM, the covariance matrix $\Sigma$ is stubbornly dense. Observing a new landmark updates its statistical correlation with every landmark already stored; even after marginalizing out the robot pose, tracking $n$ 2D landmarks leaves a $2n \times 2n$ dense matrix costing $O(n^2)$ per update—the very ceiling that crippled real-time tracking around 100 landmarks.

A pose graph's information matrix behaves completely differently. An off-diagonal block $(i,j)$ of $\Omega$ is nonzero *only* when poses $x_i$ and $x_j$ share a direct measurement edge. Because a moving robot mostly observes nearby poses along its trajectory, distant poses have zero direct coupling. $\Omega$ therefore assumes a clean banded sparse structure mirroring the graph's topology—nearly tridiagonal for an open trajectory without loop closures.

Sebastian Thrun's group, through the [Sparse Extended Information Filter (SEIF)](http://www.cs.cmu.edu/~thrun/papers/thrun.tr-seif02.pdf), along with Edwin Olson, began exploiting that sparsity directly. Using sparse linear solvers pushed computation well below $O(n^2)$; while exact runtime depends on graph connectivity, $O(n \log n)$ scaling became achievable in realistic bounded environments.

> 🔗 **Borrowed.** Thrun's sparse information filter (SEIF) and [Eustice's exactly sparse delayed-state filter](https://web.mit.edu/2.166/www/handouts/eustice_et_al_ieeetro_2006.pdf) showed that information-matrix sparsity was usable even in filter form. This insight set the stage for Dellaert's factor graph formulation and the Bayes tree data structure.

At ICRA 2006, [Olson, Leonard, and Teller](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf) added another practical boost by introducing a stochastic-gradient optimizer for pose graphs. Although it lacked formal convergence guarantees, it chewed through graphs of hundreds of nodes at interactive speeds, and Olson's open code spread rapidly across the robotics community.

---

## 6.3 Factor Graphs and Square Root SAM

Later that same year, Frank Dellaert and his doctoral student Michael Kaess published [Square Root SAM](https://doi.org/10.1177/0278364906072768) (2006), giving SLAM backends what would become their standard mathematical language. Drawing on his background in probabilistic graphical models at Georgia Tech, Dellaert reframed SLAM as Bayesian inference over a **factor graph**.

A factor graph is a bipartite graph linking two kinds of nodes: *variable nodes* represent unknown robot poses and landmark coordinates, while *factor nodes* encode sensor observations or prior constraints. Each factor $f_k(x_{i_1}, x_{i_2}, \ldots)$ captures a local probabilistic constraint over the variables it touches, so the full joint probability decomposes cleanly into a product of factors:

$$p(X) \propto \prod_k f_k(X_k)$$

Finding the maximum a posteriori (MAP) state $X^*$ maximizes this joint likelihood—which, when factors carry Gaussian noise models, reduces directly to nonlinear least squares.

That least-squares structure unlocked Dellaert's key algebraic insight. Running QR factorization on the measurement Jacobian $J$ produces an upper-triangular matrix $R$ satisfying $R^T R = J^T J = \Omega$, earning $R$ the name "square root information matrix." Crucially, how sparse $R$ stays depends not just on the graph's topology, but on the order in which variables are eliminated: smart column-reordering heuristics such as AMD and COLAMD minimize fill-in and keep $R$ compact and fast to back-substitute.

Square Root SAM offered far better numerical conditioning than an EKF covariance update, optimized poses and landmarks jointly in one consistent framework, and treated loop closures effortlessly as newly inserted factors. The interactive visualization below illustrates how sparsity and variable elimination shape that matrix structure:

<!-- DEMO: factor_graph_sparse.html -->

---

## 6.4 iSAM and iSAM2: Online Incremental Inference

Square Root SAM laid a clean theoretical foundation, yet it remained a batch solver: every time a new sensor reading arrived, re-factorizing $J^T J$ from scratch repeated massive amounts of work—costing $O(n^3)$ for dense matrices and still growing significantly for sparse ones depending on graph connectivity and elimination order.

In 2008, [Kaess, Ranganathan, and Dellaert solved that online bottleneck with **iSAM** (incremental Smoothing and Mapping)](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf). Instead of rebuilding the entire QR decomposition at every step, iSAM appended the new rows for incoming factors and updated the upper-triangular matrix $R$ incrementally using Givens rotations.

Still, iSAM1 had a structural weak spot: its relinearization schedule. Because $R$ is built by linearizing nonlinear measurement models around a current estimate, drift away from that linearization point slowly erodes accuracy. iSAM1 compensated with **periodic full relinearization**—halting every few dozen steps to relinearize the entire factor graph and re-run QR decomposition from scratch, especially when loop closures triggered heavy fill-in inside $R$. In practice, an ostensibly incremental algorithm periodically fell back into batch mode.

In 2012, [iSAM2](https://doi.org/10.1177/0278364911430419) eliminated that batch fallback using a new data structure called the **Bayes tree**. Constructed from the chordal Bayes net produced by variable elimination, a Bayes tree groups tightly coupled variables into clique nodes linked by separator edges (their shared variables). When a new factor enters the graph, iSAM2 identifies only the cliques along the affected branch of the tree, converts that subtree back into a local factor graph, relinearizes, and re-optimizes—leaving the rest of the map untouched. Paired with **fluid relinearization**, which selectively updates only variables whose linearization drift crosses a threshold and propagates changes through separators no farther than necessary, iSAM2 replaced "everything, every cycle" with "only the affected cliques, only when needed." Even after a loop closure, affected cliques frequently remained bounded enough to avoid full global recomputation.

> 🔗 **Borrowed.** The Bayes tree extends the junction tree (join tree) lineage in probabilistic graphical models, represented by the elimination-order and chordal-graph inference techniques in textbooks such as Koller-Friedman's [*Probabilistic Graphical Models*](https://mitpress.mit.edu/9780262013192/probabilistic-graphical-models/). It brought a technique from AI inference into real-time robot SLAM.

Kaess and Dellaert packaged iSAM2 inside [GTSAM (Georgia Tech Smoothing and Mapping)](https://gtsam.org), an open-source C++ library with Python bindings whose development continued across Dellaert's work at Georgia Tech and Google, eventually powering autonomous vehicles, aerial drones, and manipulator calibration.

---

## 6.5 g2o: A General Graph Optimizer in the ROS Ecosystem

While the Georgia Tech group advanced incremental inference theory, a European collaboration across TUM (Munich) and Freiburg—Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, and Wolfram Burgard—built an extraordinarily versatile open-source solver. Presented at ICRA 2011, [g2o](https://doi.org/10.1109/ICRA.2011.5979949) (general graph optimization) aimed to "handle any kind of graph optimization in a plug-in manner," blending Burgard and Grisetti's Freiburg mobile-robotics heritage, Strasdat's monocular Visual SLAM insights, and Konolige's industrial software discipline.

The architecture of g2o cleanly separates three layers: *vertices* represent state variables, *edges* encode error functions and Jacobians, and a modular *linear solver* handles sparse matrix systems via pluggable backends such as Cholmod, CSparse, Eigen, or custom libraries. Users only specify vertex types and edge residuals, leaving g2o to drive Gauss-Newton or Levenberg-Marquardt optimization automatically.

Arriving just as ROS (Robot Operating System) swept through robotics labs in the early 2010s, g2o quickly became one of the primary engines behind graph SLAM, powering landmark systems such as ORB-SLAM and LSD-SLAM. Even so, the ROS ecosystem never standardized on a single solver: particle-filter mapping lived on in `gmapping`, and Google's Cartographer built on Ceres. g2o became enormously influential without claiming a monopoly.

---

## 6.6 Why the Field Converged Here

Tracing the arc from Chatila and Laumond (1985), Lu and Milios (1997), and Gutmann and Konolige (1999) through Folkesson and Christensen (2004), Thrun's group, Dellaert (2006), and Kaess (2012) reveals more than a series of faster linear-algebra tricks.

The real revolution was a shift in how SLAM modeled time and memory. EKF-SLAM tracks only the current state and its covariance, marginalizing out every past robot pose as soon as the robot moves on; past errors become frozen inside the current state, forcing an expensive $O(N^2)$ global covariance update whenever a loop closure discovers accumulated drift.

Graph SLAM refuses to forget the past. By keeping historical poses, landmarks, and observations inside the graph, a loop closure simply drops in a new edge, and re-optimization adjusts the entire trajectory consistently (→ Chapter 10 for time-continuous trajectories). Keeping past poses revisable is the fundamental divide separating graph optimization from filtering.

The computational contrast is equally sharp. While EKF-SLAM pays $O(N^2)$ in memory and update time for $N$ landmarks, sparse Cholesky or QR factorization cuts graph-update costs dramatically—depending on graph connectivity, variable-elimination order, fill-in, and the size of the updated region (since physical motion inside a bounded area does not automatically guarantee $O(N \log N)$ complexity by itself).

> 📜 **Prediction vs. outcome.** The limits of Dellaert's batch Square Root SAM (2006) led the same group toward incremental methods. iSAM handled the problem in 2008 with Givens-rotation updates, and iSAM2 improved loop-closure efficiency in 2012 with the Bayes tree. GTSAM, Ceres, and g2o all handle nonlinear least squares, but differ in their solvers and incremental data structures. The three papers resolved one problem in stages, largely along the predicted path.

Even marginalization proved cleaner in graph form. When real systems prune old poses to keep map size bounded, a graph converts their eliminated connections into a compact prior factor linking the remaining variables. Just as in filters, that compression introduces linearization approximations that require careful handling—motivating practical compromises such as sliding-window optimization and keyframe selection.

---

## 6.7 Nonlinearity and Robustness: The Layer of Practical Engineering

Moving from tidy factor-graph theory to messy real-world robots exposed a gap that consumed much of 2010s SLAM engineering.

First came **initialization sensitivity**. Because Gauss-Newton and Levenberg-Marquardt are local optimizers, starting far from the true trajectory sends them into twisted local minima—and a single false loop-closure match can yank the initial estimate off course, making front-end outlier rejection indispensable. Chapter 7 (Certifiable SLAM) examines how convex relaxations via semidefinite programming (SDP) tackle this local-minimum trap and certify global optimality.

Second came **fragility to outliers**. Standard squared error amplifies bad loop closures disproportionately, so both g2o and GTSAM let engineers swap in robust loss kernels such as Huber or Cauchy. Yet selecting and tuning the right kernel for a given sensor and environment remains largely an art guided by engineering experience even in 2026.

Third came **marginalization fill-in**. Although iSAM2's Bayes tree performs exact incremental inference, unbounded growth in variables forces deployed systems to marginalize out older poses—generating dense fill-in that must be carefully sparsified or approximated as prior factors without destabilizing the map.

> 📜 **Prediction vs. outcome.** g2o's generality lies in allowing users to define states as vertices and observation constraints as edges. New line, plane, inertial, or object constraints can be implemented through that interface; examples must be checked against the estimator and edge implementation actually used by each system. Chapter 9 describes preintegration, the standard way to place IMU factors in the graph. As of 2026, however, g2o itself prioritizes interface stability and compatibility with existing users over broad expansion of its built-in factors. Users commonly add new factor types through inheritance, forks, or wrappers.

---

## Still open

**Choosing a robust kernel.** While Huber, Cauchy, Geman-McClure, and DCS are all standard options, no principled theory prescribes in advance which robust loss will work best for a given sensor suite and environment; engineers still tune them by hand. Researchers are exploring learned cost functions, though integrating adaptive learned losses cleanly into online incremental backends remains unresolved.

**Non-Gaussian uncertainty in factor graphs.** Standard continuous optimization in GTSAM and g2o rests on Gaussian noise assumptions, extended piecemeal with robust kernels, max-mixtures, or hybrid factors. Capturing true multimodal pose hypotheses and loop-closure mismatch probabilities in real time without blowing up compute remains an open problem.

**Bayes tree growth in densely looped maps.** The Bayes tree shines when new constraints touch only local cliques. Once long-term operation packs a large map with dense, overlapping loop closures, affected subtrees and fill-in can expand rapidly, driving up memory and latency unless checked by hierarchical submapping or sparsification.

---

By the early 2010s, the debate between filters and graphs had largely settled: tools like g2o and GTSAM made optimization backends reliable enough that research momentum pivoted upward toward the front end. Instead of asking *how* to solve a loop closure, engineers began asking *which visual features* could recognize a revisited place across drastic viewpoint and lighting shifts.

Yet before leaving the backend behind, one nagging mathematical question still hung in the air: when g2o or GTSAM reports convergence, how can we know whether it actually found the true global minimum (→ Chapter 7)?
