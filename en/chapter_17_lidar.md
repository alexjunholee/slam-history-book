# Ch.17 — The LiDAR Parallel Universe: From LOAM to FAST-LIO

The lineage that runs from Ch.1 photogrammetry through Ch.16 Foundation 3D rests on one shared premise. The sensor is a camera. MonoSLAM, PTAM, ORB-SLAM, DSO, DUSt3R — these names all sit inside the tradition of reading the world through pixels. During the same period, inside the same robotics community, an entirely different lineage was growing. The LiDAR lineage built its own grammar on the bones of ICP, unrelated to the camera camp's keypoints, photometric consistency, or feature descriptors. The two lineages did not cite each other's papers, and their benchmarks and conferences were separate.

LOAM, the chapter's central system, sits on three pre-existing bones. Point-to-plane matching from Besl & McKay's 1992 ICP. The network-of-poses framing from Lu & Milios's 1997 work on globally consistent scan alignment. The real-time outdoor pressure carried over from the 2007 DARPA Urban Challenge. Ji Zhang's 2014 contribution was to split high-frequency odometry from low-frequency mapping on a spinning Velodyne, and that split became the default grammar of every LiDAR system that followed.

When Ji Zhang presented LOAM at RSS 2014, the Visual SLAM community paid little attention. That year the visual side was busy with ElasticFusion and LSD-SLAM. The LiDAR side was no different. LOAM shared no code with camera-based methods, and the research communities did not overlap. The two lineages ran for ten years under the shared name of robotics while barely looking at each other. LOAM stood on the old skeleton of [ICP (Besl·McKay, 1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf), and the factor graph of Graph SLAM had been standard on the visual side for a long while before it crossed over to LiDAR. The parallel universe matured without exchange.

---

## 17.1 LOAM: edges and planes, and the capture of KITTI

In 2014, Google's Waymo predecessor program was already driving on roads, and the aftershocks of the DARPA Urban Challenge had not yet faded. A Velodyne HDL-64E cost $75,000 per unit. Only groups at the scale of CMU, MIT, and Stanford could afford to take LiDAR as a research object. CMU Robotics Institute's Autonomous Mobile Robot Lab — Professor Sanjiv Singh's lab — was one of them.

Attempts to build maps with LiDAR existed before LOAM. [Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping" (Autonomous Robots)](https://doi.org/10.1023/A:1008854305733) placed 2D range scans as nodes, tied them together with relative scan-to-scan constraints as edges, and jointly optimized the full trajectory; this "network of poses" idea would later be traced back as the origin point of pose-graph SLAM (see Ch.6). For the matching step itself, alongside Besl·McKay's ICP, [Biber·Straßer 2003. "The Normal Distributions Transform" (IROS)](https://doi.org/10.1109/IROS.2003.1249285) had proposed NDT — distribution-based matching that aligns to per-cell Gaussian distributions — and Magnusson's later 3D extension lived alongside ICP as an alternative. All of this was 2D, or offline 3D. LOAM's share was real-time 3D.

Ji Zhang, under Singh's supervision, released [Zhang & Singh 2014. "LOAM: Lidar Odometry and Mapping in Real-time" (RSS)](https://www.roboticsproceedings.org/rss10/p07.pdf). He classified LiDAR points into two kinds of features. An **edge point** is a point with high smoothness $c$ (high curvature); a **planar point** is one with low $c$ (low curvature). Rather than registering the whole point set like ICP, LOAM matches only these two feature sets. Edge points are constrained point-to-line against edge lines in the neighboring scan, and planar points are constrained point-to-plane against local planes. Computational cost drops. Real-time feasibility opens up.

The algorithm is split into two stages. Lidar Odometry estimates the 6-DoF transform between scans at 10 Hz. Lidar Mapping, at a lower frequency (1 Hz), registers against the full map to correct the error. Separating high-frequency odometry from low-frequency mapping suppresses drift while keeping real-time performance. This two-tier structure becomes the default grammar of LiDAR SLAM that follows.

On the KITTI benchmark, LOAM took first place shortly after release and held it for years. To be precise, until visual-LiDAR fusion methods appeared. On sequence 00, Zhang reported a relative translation error of 0.78%. Compared with the contemporary best visual odometry at around 1%, the structural advantage of LiDAR is clear.

> 🔗 **Borrowed.** LOAM's feature-based point registration starts from Besl·McKay's (1992) ICP. The difference is that it selectively matches only edge and planar features rather than all points. Selective reuse of classical registration bought both speed and precision.

---

## 17.2 LeGO-LOAM: cut the ground first

LOAM's problem was that it did not treat the ground plane explicitly. In outdoor self-driving environments, a significant share of the point cloud is road surface. Lumping it in with edge/planar features produces matching noise.

At Stevens Institute of Technology's Robust Field Autonomy Lab, Tixiao Shan and his advisor Brendan Englot separated ground segmentation as the first stage in [Shan & Englot 2018. LeGO-LOAM](https://doi.org/10.1109/IROS.2018.8594299). The point cloud is projected onto a range image, the ground points are separated first, and the non-ground points are then re-clustered. Ground is used for roll and pitch estimation, clusters for yaw and translation. Two-stage optimization.

The result was computation savings compared with LOAM. Where the original LOAM struggled to run in real time on a Velodyne VLP-16, LeGO-LOAM runs on the same sensor even on embedded platforms (NVIDIA Jetson). Lightweighting has its price. In sparse-point environments or environments with irregular ground structure — sections the laser occludes, rough off-road terrain, building interiors — segmentation fails and odometry wavers.

But LeGO-LOAM's real contribution was less the lightweighting itself than the design principle "preprocess sensor input into structured modules, then run odometry." FAST-LIO and LIO-SAM pick up this principle later.

Around the same time as LeGO-LOAM, at the University of Bonn, Jens Behley and Cyrill Stachniss brought **surfels** (surface elements), rather than edge/plane features, to outdoor LiDAR. Their **SuMa**, in [Behley & Stachniss 2018. "Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments" (RSS)](http://www.roboticsproceedings.org/rss14/p16.pdf), summarized each point's neighborhood as a disk-shaped surfel and performed scan-to-model registration; the follow-up [Chen et al. 2019. "SuMa++" (IROS)](https://doi.org/10.1109/IROS40897.2019.8967704) combined semantic segmentation to filter moving objects at the surfel level. This is the moment the surfel representation that Kintinuous and ElasticFusion had used in the Kinect indoor RGB-D lineage (Ch.9) crossed over to outdoor Velodyne. Three branches — feature selection (LOAM), segmentation-first (LeGO-LOAM), surfel accumulation (SuMa) — were competing simultaneously around 2018.

---

## 17.3 FAST-LIO — tightly coupled LiDAR-IMU

LiDAR scan frequency sits at 10–20 Hz. Fast motion in between scans produces motion distortion in the point cloud. The sensor position at the end of a scan differs from the position at the start, and this is the main reason the LOAM family wavers on high-speed platforms.

The IMU runs at 100–400 Hz. That is enough to fill the gaps in LiDAR. But performance depends on how LiDAR and IMU are combined. Loosely coupled: estimate each independently and then fuse. Tightly coupled: handle both inside a single state estimator. The latter is theoretically superior but harder to implement.

At Hong Kong University (HKU)'s MaRS Lab, Wei Xu and his advisor Fu Zhang presented [**FAST-LIO**](https://arxiv.org/abs/2010.08196) in RA-L 2021. The paper came out of a drone-control lab. There was a concrete field motivation: LiDAR odometry had to hold up on UAVs with heavy rotor vibration and fast maneuvering. The tool they chose was the **iterated Extended Kalman Filter (iEKF)**. The iEKF re-linearizes at the current estimate iteratively during the measurement update. It is consistently better than a basic EKF — which linearizes only once — under fast nonlinear motion.

The following year they published **FAST-LIO2** ([Xu et al. 2022](https://doi.org/10.1109/TRO.2022.3141876)) in TRO, adding the ikd-Tree. Conventional kd-Trees carry heavy reconstruction cost every time a point is added. The ikd-Tree is an incremental variant that performs only partial reconstruction. Real-time nearest-neighbor search stays feasible even with millions of map points. Experiments showed consistent performance on UAVs, handheld rigs, and autonomous cars. Drift stayed low even in drone environments.

The next move in the FAST-LIO lineage was to remove motion distortion entirely. [He et al. 2023. "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry" (Advanced Intelligent Systems)](https://doi.org/10.1002/aisy.202200459), also from the MaRS Lab, updates the state every time a LiDAR point arrives, instead of gathering points into a scan and updating all at once. Point-by-point observation update. Rather than correcting intra-scan distortion via constant velocity or IMU interpolation, each point is fused at its own timestamp, erasing the window in which distortion could occur. Drift reductions compared with FAST-LIO2 were reported on high-agility platforms.

> 🔗 **Borrowed.** FAST-LIO's tightly coupled IMU integration transplants to LiDAR a mathematical framework first sorted out in the Visual-Inertial SLAM camp. IMU preintegration theory was completed in [Forster et al. 2016. "On-Manifold Preintegration" (TRO)](https://doi.org/10.1109/TRO.2016.2597321), and FAST-LIO reimplements that spirit in iEKF form.

---

## 17.4 LIO-SAM: the factor graph crosses over to LiDAR

At the same time, on the Visual SLAM side the factor graph was already standard. [GTSAM (Dellaert·Kaess, 2012)](https://gtsam.org/) had settled in as the backend of Visual-Inertial systems. Yet the LiDAR side was still in EKF variants or scan-matching. The main benefit of graph optimization — correcting the full trajectory after a loop closure — LiDAR systems were not using properly.

Tixiao Shan, after LeGO-LOAM, released [Shan et al. 2020. LIO-SAM](https://doi.org/10.1109/IROS45743.2020.9341176), which explicitly adopted GTSAM's factor graph as the backend of a LiDAR-IMU system. IMU preintegration factors, LiDAR odometry factors, GPS factors, and loop closure factors are integrated into a single graph. Each keyframe becomes a node, each sensor constraint an edge. Marginalization keeps the graph size under control.

> 🔗 **Borrowed.** LIO-SAM imports directly into a LiDAR system the GTSAM factor graph backend that Dellaert (from 2006 onward) had standardized on the Visual SLAM side.

LIO-SAM is stronger than FAST-LIO2 in drift-accumulation scenarios. It has loop closure. On the other hand, computation cost is higher, and without GPS or additional sensor input the factor graph's advantage shrinks. The two systems have different design goals. FAST-LIO2 targets top speed and precision in a real-time single-sensor configuration; LIO-SAM targets consistency in multi-sensor long-term mapping.

There was also a paradoxical return. For close to ten years after LOAM, LiDAR odometry had run toward feature selection, surfels, and neural descriptors — getting steadily more complex — yet in 2023 Bonn's [Vizzo et al. 2023. "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L)](https://doi.org/10.1109/LRA.2023.3236571) released the opposite direction. With no feature extraction and no learned descriptors, an adaptive threshold and a single point-to-point ICP with almost no tuning showed competitive odometry on KITTI. Keep It Small and Simple, as the name says. The authors' claim was close to historical revisionism: "LOAM did not arise because ICP was lacking; the engineering was." Behind the fact that a return to classical registration became possible after ten years lies practical progress at the level of GPUs and kd-Tree implementations.

---

## 17.5 Sensor price drop and diffusion: 2007–2024

In the history of LiDAR SLAM, sensor price is as important as any technical paper.

At the 2007 DARPA Urban Challenge, the Velodyne HDL-64E that top teams carried was $75,000 per unit. Unless you were an autonomous-driving research team or a defense project, the hardware was out of reach. In 2012 the HDL-32E was still around $30,000. By 2014, when LOAM came out, the VLP-16 had dropped to $7,999 — still a significant chunk of a research budget.

Over the following decade, the reversal happened. The Chinese startup Livox (part of DJI) released the Livox Mid-40 at $599 in 2019. Ouster brought 128-channel sensors into the low-thousands range. By 2023–2024, solid-state LiDARs from RoboSense, Innovusion, and Livox came in under $500. The price fell by more than 100x, and it took ten years.

Diffusion did not outrun algorithmic progress. Solid-state LiDARs, unlike spinning types, have a limited field of view (FoV). 70°×70° or narrower. It is not the 360° all-around scan that LOAM and FAST-LIO assumed. Existing algorithms do not just work out of the box. The spread of cheap sensors created new algorithmic research questions at the same time.

---

## 17.6 Why the Visual and LiDAR lineages split

Visual SLAM and LiDAR SLAM developed in the same period, yet the two communities did not exchange for a long time. The reason was not a single layer.

The first is the sensor itself. Cameras see texture and color; LiDAR sees range and geometry. While camera-based methods developed around keypoints, descriptors, and photometric consistency, LiDAR differentiated around edges, planes, and range images. The problem formulations themselves were different.

The conferences were also different. CVPR and ICCV were the main stages for camera-based methods; ICRA, IROS, and RSS were where LiDAR SLAM mostly appeared. The researcher populations did not overlap. During the early-to-mid 2010s, when Velodyne was shipping into Google and the autonomous-driving industry, the LiDAR SLAM researcher population was dense on the self-driving robotics side.

Place recognition methods diverged too. Cameras use visual appearance, as in DBoW2 and NetVLAD. LiDAR uses the structural features of a 3D point cloud, as in [Scan Context (Kim·Kim, 2018)](https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf) or [PointNetVLAD](https://arxiv.org/abs/1804.03492). Even for the same location, the signal being recognized is different.

The first signs of convergence showed up in the early 2020s. Papers on LiDAR-Camera fusion started appearing at CVPR, and Tixiao Shan's [LVI-SAM (2021)](https://arxiv.org/abs/2104.10831) was an attempt to bolt a visual-inertial subsystem onto LIO-SAM. The authors presented it as a tightly coupled factor graph, but the two subsystems (LIS and VIS) operate largely independently and help each other on failure, which means a fully unified single state estimate is still open.

---

## 17.7 Visual-LiDAR convergence attempts: 2024–2025

Starting in 2024, the mood shifted. As foundation models developed toward extracting features sensor-agnostically, attempts to handle camera and LiDAR in one frame increased. There are two branches.

One is multi-modal pretrained features. Align LiDAR and camera into the same embedding space. As [CLIP (Radford et al., 2021)](https://arxiv.org/abs/2103.00020) did for image-text alignment, approaches that use LiDAR-image contrastive learning. In 2023–2024 several groups are at the experimental stage.

The other is unified sensor abstraction. Integrate sensor outputs into geometric primitives or neural fields and then process them in a single backend. This side is still at the research-paper stage, and systems that have shown real-time operation are rare.

Neither direction has yet produced a single lineage that actually merges LiDAR SLAM and Visual SLAM. FAST-LIO2 and ORB-SLAM3 are still used independently.

---

## 17.8 Radar is outside this book's scope

Right next to the LiDAR parallel universe sits another parallel universe. Radar SLAM has matured as an independent subfield on two hardware branches — spinning radar (the Navtech CIR family) and SoC-based 4D mmWave radar — and on the fact that Doppler radial velocity can be measured directly, which enables correspondence-free odometry, and on radio-specific noise models for speckle, multipath, and receiver saturation. The lineage runs from Oxford's radar localisation in [Cen & Newman 2018](https://doi.org/10.1109/ICRA.2018.8460687) through Adolfsson·Magnusson's **CFEAR** and its successor **TBV-SLAM** to Burnett·Barfoot's continuous-time ICP, and dedicated datasets such as Oxford Radar RobotCar, Boreas, and MulRan form the benchmark base for this area. The practical motivations — penetration through bad weather and smoke — are clear, but the contact surface with the lineage this book has been tracking (photogrammetry → SfM → Visual SLAM → learning → 3D foundation) is thin. Radar is left as "a neighbor to merge in later," and this book does not write its separate history — see Handbook of SLAM (2026) Ch.9 for details.

---

## 📜 Prediction vs. outcome

> Zhang·Singh named two items as explicit future work in the Conclusion of the 2014 LOAM paper. First, introducing loop closure to correct drift. Second, combining the method with IMU output through a Kalman filter. Both directions were realized within the next ten years. IMU integration was settled by FAST-LIO (2021) and FAST-LIO2 (2022) with a tightly coupled iEKF, and loop closure was folded in by LIO-SAM (2020) with a factor graph backend. The path the authors sketched was implemented fairly accurately. Beyond those two axes, however, there was a problem that never appeared in the paper's Conclusion but kept surfacing in the field. Dynamic object handling. Real-time separation of moving pedestrians and vehicles from LiDAR points still leans mostly on deep-learning segmentation as of 2026, and a solution built into the SLAM algorithm itself is still absent. `[hit + in progress]`

---

## 🧭 Still open

**Full Visual+LiDAR fusion.** Even after LVI-SAM, no system has reached practical status that handles the two sensors tightly coupled inside a single state estimator. The scenario where the camera fails in fog or rain and LiDAR has to fill the gap is a clear need in autonomous driving. Algorithmic and sensor-calibration difficulty are still the barriers. Several groups in 2024–2025 are experimenting with transformer-based fusion, but consistent results have not arrived.

**Algorithms optimized for solid-state LiDAR.** LOAM and FAST-LIO all assume 360° spinning LiDAR. The solid-state products from Livox and RoboSense use non-repetitive scan patterns. They accumulate by hitting the same point many times. Feature extraction and motion distortion correction that fit these characteristics need their own research. Livox LOAM exists, but the level of generalization is weak.

**Dynamic object handling.** This problem sits at the same place in Zhang's 2014 prediction and in 2026. The static-world assumption is an old premise of SLAM, and LiDAR is no exception. Real-time separation of moving objects from the point cloud is currently handed off to segmentation networks as a workaround. Methods that handle it inside SLAM on a geometric basis are expensive and unstable in accuracy. Companies like Waymo and Argo AI run their own solutions, but no general public algorithm exists.

---

The LiDAR lineage matured without crossing the visual main track. The two lineages acquired their own languages, and translation between those languages is still in progress. Ch.18 steps back from both tracks to look at what was tried alongside them — lineages that neither camera nor LiDAR absorbed, and what their failure meant.
