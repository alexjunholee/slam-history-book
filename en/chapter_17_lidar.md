# Ch.17 — The LiDAR Parallel Universe: From LOAM to FAST-LIO

The lineage from Ch.1 photogrammetry through Ch.16 Foundation 3D shares one premise: the sensor is a camera. MonoSLAM, PTAM, ORB-SLAM, DSO, and DUSt3R all read the world through pixels. Over the same period, LiDAR SLAM developed separately around ICP and point-cloud registration rather than keypoints, photometric consistency, or feature descriptors. The two communities rarely cited each other, and they used different benchmarks and conferences.

LOAM combined three earlier elements: point-to-plane matching from Besl & McKay's 1992 ICP, the network-of-poses formulation from Lu & Milios's 1997 work on globally consistent scan alignment, and the demand for real-time outdoor operation after the 2007 DARPA Urban Challenge. Ji Zhang's 2014 contribution was to split high-frequency odometry from low-frequency mapping on a spinning Velodyne. That division became a standard LiDAR-system architecture.

When Ji Zhang presented LOAM at RSS 2014, it drew little attention from the Visual SLAM community, which was occupied with ElasticFusion and LSD-SLAM. LOAM shared no code with camera-based methods, and the researcher populations barely overlapped. Both lineages belonged to robotics but developed apart for roughly a decade. LOAM built on [ICP (Besl·McKay, 1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf), while the factor graph already standard in Graph SLAM crossed into LiDAR systems only later.

---

## 17.1 LOAM: edges and planes, and the capture of KITTI

In 2014, Google's Waymo predecessor program was already driving on roads, and the influence of the DARPA Urban Challenge remained strong. A Velodyne HDL-64E cost $75,000 per unit. LiDAR research was therefore concentrated in large, well-funded groups able to afford the hardware. CMU Robotics Institute's Autonomous Mobile Robot Lab, led by Professor Sanjiv Singh, was one of them.

Attempts to build maps with LiDAR existed before LOAM. [Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping" (Autonomous Robots)](https://doi.org/10.1023/A:1008854305733) placed 2D range scans as nodes, tied them together with relative scan-to-scan constraints as edges, and jointly optimized the full trajectory. This "network of poses" idea became an origin point of pose-graph SLAM (see Ch.6). Alongside Besl and McKay's ICP, [Biber and Straßer 2003. "The Normal Distributions Transform" (IROS)](https://doi.org/10.1109/IROS.2003.1249285) proposed NDT, a distribution-based method that aligns per-cell Gaussian distributions; Magnusson later extended it to 3D. These methods were either 2D or offline 3D. LOAM brought real-time 3D operation.

Ji Zhang, under Singh's supervision, released [Zhang & Singh 2014. "LOAM: Lidar Odometry and Mapping in Real-time" (RSS)](https://www.roboticsproceedings.org/rss10/p07.pdf). He classified LiDAR points into two kinds of features. An **edge point** is a point with high smoothness $c$ (high curvature); a **planar point** is one with low $c$ (low curvature). Rather than registering the whole point set like ICP, LOAM matches only these two feature sets. Edge points are constrained point-to-line against edge lines in the neighboring scan, and planar points are constrained point-to-plane against local planes. This selection lowers computational cost enough for real-time operation.

The algorithm is split into two stages. Lidar Odometry estimates the 6-DoF transform between scans at 10 Hz. Lidar Mapping, at a lower frequency (1 Hz), registers against the full map to correct the error. Separating high-frequency odometry from low-frequency mapping suppresses drift while retaining real-time performance. Later LiDAR SLAM systems widely adopted this two-tier structure.

LOAM entered the leading group in public KITTI comparisons after its release. The widely reported relative translation error is 0.78% on sequence 00 and 0.84% averaged over the listed sequences. These figures show its competitiveness, but they do not establish a universal advantage over visual odometry under different sensor and evaluation conditions.

> 🔗 **Borrowed.** LOAM's feature-based point registration starts from Besl·McKay's (1992) ICP. The difference is that it selectively matches only edge and planar features rather than all points. Selective reuse of classical registration bought both speed and precision.

---

## 17.2 LeGO-LOAM: cut the ground first

LOAM did not treat the ground plane explicitly. In outdoor driving environments, a significant share of the point cloud is road surface. Grouping it with other edge and planar features produces matching noise.

At Stevens Institute of Technology's Robust Field Autonomy Lab, Tixiao Shan and his advisor Brendan Englot separated ground segmentation as the first stage in [Shan & Englot 2018. LeGO-LOAM](https://doi.org/10.1109/IROS.2018.8594299). The point cloud is projected onto a range image, the ground points are separated first, and the non-ground points are then re-clustered. Ground is used for roll and pitch estimation, and clusters are used for yaw and translation. This is a two-stage optimization.

The result required less computation than LOAM. Where the original LOAM struggled to run in real time on a Velodyne VLP-16, LeGO-LOAM runs on the same sensor even on embedded NVIDIA Jetson platforms. The reduction came with a cost: segmentation can fail and odometry can degrade in sparse scans or environments with irregular ground, including occluded sections, rough off-road terrain, and building interiors.

Beyond its lower computational cost, LeGO-LOAM established a design pattern: preprocess the sensor input into structured components before running odometry. FAST-LIO and LIO-SAM later used related modular preprocessing.

Around the same time as LeGO-LOAM, Jens Behley and Cyrill Stachniss at the University of Bonn brought **surfels** (surface elements) to outdoor LiDAR instead of using edge and plane features. Their **SuMa**, described in [Behley & Stachniss 2018. "Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments" (RSS)](http://www.roboticsproceedings.org/rss14/p16.pdf), summarized each point's neighborhood as a disk-shaped surfel and performed scan-to-model registration. The follow-up [Chen et al. 2019. "SuMa++" (IROS)](https://doi.org/10.1109/IROS40897.2019.8967704) used semantic segmentation to filter moving objects at the surfel level. The surfel representation used by ElasticFusion in the indoor RGB-D lineage (Ch.9) had crossed into outdoor Velodyne systems. By 2018, feature selection (LOAM), segmentation-first processing (LeGO-LOAM), and surfel accumulation (SuMa) were competing approaches.

---

## 17.3 FAST-LIO — tightly coupled LiDAR-IMU

LiDAR scan frequency sits at 10–20 Hz. Fast motion between scans produces motion distortion in the point cloud. The sensor position at the end of a scan differs from its position at the start, which degrades the LOAM family on high-speed platforms.

An IMU runs at 100–400 Hz and can fill the gaps between LiDAR scans. Performance depends on how the two sensors are combined. A loosely coupled system estimates each independently and fuses the results; a tightly coupled system handles both inside one state estimator. The latter can use their cross-correlation but is harder to implement.

At Hong Kong University (HKU)'s MaRS Lab, Wei Xu and his advisor Fu Zhang presented [**FAST-LIO**](https://arxiv.org/abs/2010.08196) in RA-L 2021. Their drone-control work supplied a concrete field requirement: LiDAR odometry had to withstand heavy rotor vibration and fast UAV maneuvers. They used an **iterated Extended Kalman Filter (iEKF)**, which repeatedly re-linearizes at the current estimate during the measurement update. This can reduce measurement-model linearization error relative to a basic EKF that linearizes once, but the improvement depends on the initial estimate and the motion and observation conditions.

The following year they published **FAST-LIO2** ([Xu et al. 2022](https://doi.org/10.1109/TRO.2022.3141876)) in TRO, adding the ikd-Tree. Conventional kd-Trees carry heavy reconstruction cost every time a point is added. The ikd-Tree is an incremental variant that performs only partial reconstruction. Real-time nearest-neighbor search stays feasible even with millions of map points. Experiments showed consistent performance on UAVs, handheld rigs, and autonomous cars. Drift stayed low even in drone environments.

The next FAST-LIO system addressed motion distortion at the point level. [He et al. 2023. "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry" (Advanced Intelligent Systems)](https://doi.org/10.1002/aisy.202200459), also from the MaRS Lab, updates the state whenever a LiDAR point arrives instead of collecting a full scan before updating. Each point is fused at its own timestamp rather than correcting intra-scan distortion with a constant-velocity model or IMU interpolation. The authors reported lower drift than FAST-LIO2 on high-agility platforms.

> 🔗 **Borrowed.** FAST-LIO brings to LiDAR a tightly coupled IMU framework developed in Visual-Inertial SLAM. [Forster et al. 2016. "On-Manifold Preintegration" (TRO)](https://doi.org/10.1109/TRO.2016.2597321) established the preintegration formulation, while FAST-LIO implemented tightly coupled inertial estimation in iEKF form.

---

## 17.4 LIO-SAM: the factor graph crosses over to LiDAR

Factor graphs were already standard in Visual SLAM. [GTSAM (Dellaert·Kaess, 2012)](https://gtsam.org/) had become a common backend for Visual-Inertial systems, while LiDAR systems still relied mainly on EKF variants or scan matching. LiDAR pipelines therefore made less use of graph optimization's ability to correct the full trajectory after loop closure.

Tixiao Shan, after LeGO-LOAM, released [Shan et al. 2020. LIO-SAM](https://doi.org/10.1109/IROS45743.2020.9341176), which explicitly adopted GTSAM's factor graph as the backend of a LiDAR-IMU system. The [public implementation](https://github.com/TixiaoShan/LIO-SAM) maintains two graphs. The long-term mapping graph accumulates LiDAR odometry, GPS, and loop-closure constraints between keyframes. A separate IMU-preintegration graph estimates state and bias from inertial measurements and LiDAR odometry and is reset periodically to bound computation.

> 🔗 **Borrowed.** LIO-SAM imports directly into a LiDAR system the GTSAM factor graph backend that Dellaert (from 2006 onward) had standardized on the Visual SLAM side.

LIO-SAM handles accumulated drift better than FAST-LIO2 because it includes loop closure, but its computational cost is higher. Without GPS or another sensor, the factor graph offers less advantage. The two systems serve different design goals: FAST-LIO2 prioritizes speed and precision in a real-time single-sensor configuration, while LIO-SAM prioritizes consistency in multi-sensor long-term mapping.

For nearly ten years after LOAM, LiDAR odometry added feature selection, surfels, and neural descriptors. In 2023, [Vizzo et al. 2023. "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L)](https://doi.org/10.1109/LRA.2023.3236571) at Bonn took the opposite direction. An adaptive threshold and a point-to-point ICP, with no feature extraction or learned descriptors and little tuning, produced competitive odometry on KITTI. The name stood for Keep It Small and Simple. The result showed that classical registration remained competitive when motion compensation, sampling, and robust correspondence handling were combined effectively. It does not establish why LOAM arose historically.

---

## 17.5 Falling sensor prices and wider use: 2007–2024

Sensor price shaped LiDAR SLAM alongside its technical papers.

At the 2007 DARPA Urban Challenge, the Velodyne HDL-64E used by leading teams cost $75,000 per unit, putting it beyond most groups outside autonomous-driving and defense research. In 2012 the HDL-32E was still around $30,000. By 2014, when LOAM appeared, the VLP-16 had dropped to $7,999, still a significant share of a research budget.

The LiDAR market spread across a much wider range of prices over the following decade. In 2019, Livox (part of DJI) [announced a US retail price of $599 for the Mid-40](https://www.livoxtech.com/news/1). Ouster's 128-channel OS1-128 cost $18,000 that year; in 2020, Ouster [announced a $600 target price for the solid-state ES2 in 2024 series-production programs](https://investors.ouster.com/news-releases/news-release-details/ouster-announces-first-high-performance-true-solid-state-digital). Products and production targets in the hundreds of dollars marked a real shift, but they do not establish a uniform 100-fold decline across LiDAR: channel count, field of view, range, and retail-versus-volume pricing differ.

Wider availability did not eliminate algorithmic constraints. Solid-state LiDARs generally have a more limited field of view (FoV) than spinning sensors, and some use non-repetitive scan patterns. The original LOAM, designed around a 360° rotating scan, does not transfer unchanged. FAST-LIO and FAST-LIO2, by contrast, were designed to handle both mechanical and solid-state LiDARs and reported operation with small FoV and irregular sampling. Lower prices therefore changed the algorithmic questions rather than removing them.

---

## 17.6 Why the Visual and LiDAR lineages split

Visual SLAM and LiDAR SLAM developed in the same period, yet the two communities exchanged little for years. Several differences reinforced the split.

The sensors produced different measurements. Cameras capture texture and color; LiDAR measures range and geometry. Camera-based methods developed around keypoints, descriptors, and photometric consistency, while LiDAR methods used edges, planes, and range images. Their problem formulations differed accordingly.

The conferences differed as well. Camera-based methods appeared mainly at CVPR and ICCV, while LiDAR SLAM appeared mostly at ICRA, IROS, and RSS. The researcher populations overlapped little. During the early-to-mid 2010s, as Velodyne supplied Google and the autonomous-driving industry, LiDAR SLAM concentrated in self-driving robotics groups.

Place recognition methods diverged too. Cameras use visual appearance, as in DBoW2 and NetVLAD. LiDAR uses the structural features of a 3D point cloud, as in [Scan Context (Kim·Kim, 2018)](https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf) or [PointNetVLAD](https://arxiv.org/abs/1804.03492). Even for the same location, the signal being recognized is different.

The first signs of convergence appeared in the early 2020s, when LiDAR-camera fusion papers began reaching CVPR. Tixiao Shan's [LVI-SAM (2021)](https://arxiv.org/abs/2104.10831) added a visual-inertial subsystem to LIO-SAM. The authors presented a tightly coupled factor graph, but the LIS and VIS subsystems operate largely independently and support each other during failures. A fully unified state estimate remained open.

---

## 17.7 Visual-LiDAR convergence attempts: 2024–2025

From 2024, more work attempted to handle camera and LiDAR data in one representation as foundation models became less tied to a single sensor. Two approaches emerged.

One approach uses multimodal pretrained features to align LiDAR and camera data in the same embedding space. It adapts the contrastive-learning principle of [CLIP (Radford et al., 2021)](https://arxiv.org/abs/2103.00020) from image-text alignment to LiDAR-image pairs. In 2023–2024, this work remained experimental.

The other approach converts sensor outputs into shared geometric primitives or neural fields and processes them in one backend. This work also remained at the research-paper stage, with few demonstrations of real-time operation.

Neither approach had produced a common system lineage by 2026. FAST-LIO2 and ORB-SLAM3 were still used independently.

---

## 17.8 Radar is outside this book's scope

Radar SLAM developed as another independent subfield, divided between spinning radar such as the Navtech CIR family and SoC-based 4D mmWave radar. Direct Doppler radial-velocity measurements enable correspondence-free odometry, while radio-specific noise models address speckle, multipath, and receiver saturation. The lineage runs from Oxford's radar localisation in [Cen & Newman 2018](https://doi.org/10.1109/ICRA.2018.8460687), through Adolfsson and Magnusson's **CFEAR** and its successor **TBV-SLAM**, to Burnett and Barfoot's continuous-time ICP. Oxford Radar RobotCar, Boreas, and MulRan supply dedicated benchmarks. Radar's operation in bad weather and smoke offers a clear practical advantage, but it has little historical overlap with the photogrammetry → SfM → Visual SLAM → learning → 3D foundation lineage. Its separate history falls outside this book; Handbook of SLAM (2026) Ch.9 provides a technical account.

---

## 📜 Prediction vs. outcome

> Zhang and Singh named two items as explicit future work in the conclusion of the 2014 LOAM paper: loop closure to correct drift and fusion with IMU output through a Kalman filter. Both appeared within the next ten years. FAST-LIO (2021) and FAST-LIO2 (2022) integrated the IMU with a tightly coupled iEKF, while LIO-SAM (2020) added loop closure through a factor-graph backend. One field problem absent from the paper's conclusion remained: dynamic-object handling. As of 2026, real-time separation of moving pedestrians and vehicles from LiDAR points often used deep-learning segmentation. Geometric methods within SLAM also existed, but no general solution covered the range of dynamic environments.

---

## 🧭 Still open

**Full Visual+LiDAR fusion.** Even after LVI-SAM, no tightly coupled design that handles both sensors inside one state estimator has become a broadly accepted common architecture. Autonomous-driving systems need LiDAR to compensate when a camera weakens in fog or rain, but algorithm design and sensor calibration remain barriers. Transformer-based fusion in 2024–2025 remained at the research-prototype stage.

**Algorithms optimized for solid-state LiDAR.** The original LOAM assumed the scan-line structure of a 360° spinning LiDAR. Non-repetitive scans in some Livox products and the limited FoV of solid-state sensors change observability and motion distortion. FAST-LIO2's direct point-to-map formulation and Livox LOAM already address parts of this setting, but no single configuration covers the differing fields of view and scan patterns across sensor families.

**Dynamic object handling.** The static-world assumption remained in LiDAR SLAM from Zhang's 2014 work through 2026. Systems commonly hand real-time separation of moving objects to segmentation networks. Geometric approaches inside SLAM still face computational and stability costs. Production pipelines are mostly proprietary, while public research has not converged on one generally accepted solution.

---

The LiDAR and visual lineages matured with separate technical vocabularies, and their integration remained incomplete. Other approaches developed beside both of them, including biologically inspired, event-based, and semantic SLAM, without becoming the main line of either community.
