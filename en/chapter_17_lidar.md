# Chapter 21. LiDAR SLAM: From LOAM to FAST-LIO

Every lineage traced from Chapter 1 photogrammetry through Chapter 20 Foundation 3D shared one quiet premise: the sensor was a camera. MonoSLAM, PTAM, ORB-SLAM, DSO, and DUSt3R all interpreted the world through pixels. Over the same decades, however, an independent tradition grew up around laser rangefinders—built on Iterative Closest Point (ICP) and point-cloud registration rather than keypoints, photometric residuals, or visual descriptors. The two communities rarely cited one another, met at different conferences, and judged success on separate benchmarks.

Within that laser tradition, LOAM fused three strands: point-to-plane matching from Besl & McKay's 1992 ICP, the "network of poses" formulation from Lu & Milios's 1997 work on globally consistent scan alignment, and the urgent push for real-time outdoor autonomy sparked by the 2007 DARPA Urban Challenge. Ji Zhang's decisive 2014 insight was architectural: splitting high-frequency odometry from low-frequency mapping on a spinning Velodyne scanner—a two-tier template that became standard across LiDAR SLAM.

When Zhang presented LOAM at RSS 2014, the Visual SLAM community barely noticed, preoccupied as it was with ElasticFusion and LSD-SLAM. LOAM shared no codebase with camera-based pipelines, and the two researcher pools scarcely overlapped; both belonged to robotics yet evolved on parallel tracks for roughly a decade. LOAM rooted itself in [ICP (Besl·McKay, 1992)](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf), whereas the factor graphs already standard in Graph SLAM crossed into LiDAR systems only years later.

Radar SLAM branched along yet another independent path, split between spinning mechanically scanned sensors like the Navtech CIR series and single-chip 4D mmWave radars. Direct Doppler radial-velocity returns support correspondence-free odometry, paired with radio-specific noise models for speckle, multipath reflections, and receiver saturation. Its own arc runs from Oxford's radar localization in [Cen & Newman 2018](https://doi.org/10.1109/ICRA.2018.8460687), through Adolfsson and Magnusson's **CFEAR** and its successor **TBV-SLAM**, to Burnett and Barfoot's continuous-time ICP—supported by benchmarks including Oxford Radar RobotCar, Boreas, and MulRan. While radar's resilience in rain, fog, and smoke offers clear operational value, its history scarcely intersects the photogrammetry → SfM → Visual SLAM → learning → 3D foundation lineage covered here; a full technical treatment appears in Handbook of SLAM (2026) Ch.9.

---

## 21.1 LOAM: edges and planes, and the capture of KITTI

By 2014, Google's self-driving program—the precursor to Waymo—was already logging road miles, and momentum from the 2007 DARPA Urban Challenge remained strong. Because a Velodyne HDL-64E cost $75,000 per unit, 3D LiDAR research clustered inside well-funded labs able to field the hardware, among them Sanjiv Singh's Autonomous Mobile Robot Lab at Carnegie Mellon's Robotics Institute.

Range-based mapping itself predated LOAM by nearly two decades. [Lu & Milios 1997. "Globally Consistent Range Scan Alignment for Environment Mapping" (Autonomous Robots)](https://doi.org/10.1023/A:1008854305733) had modeled 2D laser scans as graph nodes linked by relative scan-to-scan edges and jointly optimized the trajectory—laying an early cornerstone of pose-graph SLAM (→ Chapter 6). Alongside Besl and McKay's ICP, [Biber and Straßer 2003. "The Normal Distributions Transform" (IROS)](https://doi.org/10.1109/IROS.2003.1249285) introduced NDT, aligning local Gaussian cell distributions in a formulation Magnusson later generalized to 3D. Yet those predecessors ran either in 2D or offline in 3D; LOAM delivered real-time 3D mapping on live vehicles.

In [Zhang & Singh 2014. "LOAM: Lidar Odometry and Mapping in Real-time" (RSS)](https://www.roboticsproceedings.org/rss10/p07.pdf), Ji Zhang—working under Singh—avoided registering every raw laser return. Instead, he scored local smoothness $c$ to separate sharp **edge points** (high curvature) from flat **planar points** (low curvature), matching only those sparse geometric classes: edge points against line segments and planar points against local surface patches in neighboring scans. Dropping the rest of the cloud slashed computation enough to run live.

To reconcile speed with global accuracy, Zhang split the pipeline into two asynchronous loops. **Lidar Odometry** runs at 10 Hz to track 6-DoF motion between consecutive sweeps, while **Lidar Mapping** runs at 1 Hz to register accumulated sweeps against the global point cloud and cancel drift. That division—fast frame-to-frame tracking paired with slower map-level refinement—quickly became canonical across LiDAR SLAM.

Once released, LOAM climbed near the top of the KITTI odometry leaderboard, posting a widely cited relative translation error of 0.78% on sequence 00 and 0.84% averaged across the evaluated sequences. Those numbers proved its strength on highway-scale scans, though differences in sensors and evaluation protocols preclude claiming any universal edge over visual odometry.

> 🔗 **Borrowed.** LOAM's geometric registration descends directly from Besl·McKay's (1992) ICP, but swaps brute-force all-point matching for selective edge-to-line and plane-to-plane alignment—a targeted refinement that bought both speed and accuracy.

---

## 21.2 LeGO-LOAM: cut the ground first

For all its success, LOAM treated every planar return alike—a blind spot on roads, where ground points dominate the scan and easily introduce noisy, ambiguous matches alongside building facades.

At Stevens Institute of Technology's Robust Field Autonomy Lab, Tixiao Shan and his advisor Brendan Englot tackled that bottleneck in [Shan & Englot 2018. LeGO-LOAM](https://doi.org/10.1109/IROS.2018.8594299). Projecting each 3D scan onto a 2D range image, LeGO-LOAM segments out the ground plane first, clusters the remaining above-ground objects, and runs a two-stage optimization: ground points constrain roll, pitch, and vertical translation, while object clusters solve for yaw and horizontal translation.

Cutting away raw ground clutter yielded sharp computational savings. Where original LOAM strained to keep up in real time on a lightweight 16-beam Velodyne VLP-16, LeGO-LOAM ran live on the same sensor even on an embedded NVIDIA Jetson. The trade-off was fragility whenever the ground assumption broke: in sparse scans, occluded alleys, rugged off-road terrain, or cluttered indoor rooms, segmentation errors could quickly degrade odometry. Still, LeGO-LOAM established an enduring pattern—structuring raw point clouds through modular preprocessing before optimization—that later systems like FAST-LIO and LIO-SAM built upon.

Around the same time, Jens Behley and Cyrill Stachniss at the University of Bonn bypassed edge and plane extraction altogether in favor of **surfels** (disk-shaped surface elements). Their **SuMa** system, presented in [Behley & Stachniss 2018. "Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments" (RSS)](http://www.roboticsproceedings.org/rss14/p16.pdf), summarized local neighborhoods as oriented surfels for scan-to-model registration, and [Chen et al. 2019. "SuMa++" (IROS)](https://doi.org/10.1109/IROS40897.2019.8967704) layered semantic segmentation on top to strip moving vehicles before surfel fusion. A surface representation popularized indoors by ElasticFusion (Chapter 12) had crossed into outdoor Velodyne mapping—leaving feature selection (LOAM), ground-first segmentation (LeGO-LOAM), and surfel accumulation (SuMa) competing side by side by 2018.

---

## 21.3 FAST-LIO — tightly coupled LiDAR-IMU

A spinning LiDAR typically sweeps at just 10–20 Hz. On a fast-moving vehicle or agile drone, the sensor translates and rotates significantly between the first and last laser firing of a single sweep, warping the point cloud with motion distortion that undermines LOAM-style scan matching.

An IMU, sampling at 100–400 Hz, fills that temporal gap. How tightly the two sensors are coupled, however, dictates accuracy under rapid motion: loosely coupled pipelines fuse separate inertial and laser estimates after the fact, whereas tightly coupled estimators place raw LiDAR returns and inertial measurements inside a single joint state—exploiting cross-correlations at the price of tighter mathematical integration.

At the University of Hong Kong (HKU)'s MaRS Lab, Wei Xu and his advisor Fu Zhang faced a demanding field testbed: quadrotors buffeted by rotor vibration and aggressive flight maneuvers. Their [**FAST-LIO**](https://arxiv.org/abs/2010.08196) (RA-L 2021) fused IMU measurements and LiDAR edge/plane residuals inside an **iterated Extended Kalman Filter (iEKF)**. By re-linearizing the measurement model repeatedly around each updated state estimate, an iEKF can cut linearization errors compared with a standard single-step EKF, though the realized gain depends on initialization quality and motion dynamics.

A year later, the same team pushed further in **FAST-LIO2** ([Xu et al. 2022](https://doi.org/10.1109/TRO.2022.3141876), TRO), dropping feature extraction to register raw points directly against a custom incremental data structure—the **ikd-Tree**. Because conventional kd-trees incur heavy rebuild overhead whenever new points arrive, the ikd-Tree restructures only affected branches incrementally, keeping nearest-neighbor queries real-time across millions of mapped points and delivering low drift across UAVs, handheld rigs, and ground vehicles alike.

The MaRS Lab then pushed temporal granularity to its limit in [He et al. 2023. "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry" (Advanced Intelligent Systems)](https://doi.org/10.1002/aisy.202200459). Rather than buffering a full sweep and deskewing intra-scan distortion afterward, Point-LIO updates the filter state at the exact timestamp of every arriving LiDAR point—cutting drift even further on high-agility platforms.

> 🔗 **Borrowed.** FAST-LIO imported into LiDAR SLAM the tightly coupled inertial fusion paradigm forged in Visual-Inertial SLAM. Whereas [Forster et al. 2016. "On-Manifold Preintegration" (TRO)](https://doi.org/10.1109/TRO.2016.2597321) formalized inertial factors for graph optimization, FAST-LIO realized tight LiDAR-inertial coupling inside an iterated EKF.

---

## 21.4 LIO-SAM: the factor graph crosses over to LiDAR

While Visual-Inertial SLAM had standardized around factor-graph backends like [GTSAM (Dellaert·Kaess, 2012)](https://gtsam.org/), most LiDAR pipelines through the late 2010s still relied on EKF filters or pairwise scan matching—leaving them less equipped to propagate global loop-closure corrections back through an entire trajectory.

In 2020, building on his LeGO-LOAM work, Tixiao Shan brought GTSAM squarely into LiDAR-inertial mapping with [Shan et al. 2020. LIO-SAM](https://doi.org/10.1109/IROS45743.2020.9341176). Its [public codebase](https://github.com/TixiaoShan/LIO-SAM) couples two factor graphs: a long-term mapping graph that binds keyframes with LiDAR odometry, GPS, and loop-closure factors, and a short-term IMU-preintegration graph that tracks high-rate state and sensor bias before resetting periodically to keep runtime bounded.

> 🔗 **Borrowed.** LIO-SAM carried the GTSAM factor-graph backend—cultivated by Dellaert in Visual SLAM from 2006 onward—directly into LiDAR-inertial mapping.

Choosing between FAST-LIO2 and LIO-SAM reflects a classic systems trade-off: LIO-SAM suppresses long-term drift more effectively when loop closures or GPS factors are available, at higher computational cost, whereas FAST-LIO2 excels in speed and raw precision for single-sensor real-time flight without external aids.

Meanwhile, after a decade in which LiDAR odometry accumulated edge/plane classifiers, surfels, and neural descriptors, [Vizzo et al. 2023. "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L)](https://doi.org/10.1109/LRA.2023.3236571) at Bonn challenged the premise that complexity was mandatory. True to its name ("Keep It Small and Simple"), KISS-ICP paired vanilla point-to-point ICP with an adaptive correspondence threshold, motion compensation, and clean voxel subsampling—requiring no feature extraction, no learned descriptors, and minimal tuning—yet matched modern baselines on KITTI. That result demonstrated how far careful engineering of classical registration could reach, though it reflects modern compute and motion compensation rather than explaining the historical conditions under which LOAM first emerged.

---

## 21.5 Falling sensor prices and wider use: 2007–2024

The trajectory of LiDAR SLAM was shaped as much by hardware economics as by estimation theory.

At the 2007 DARPA Urban Challenge, the flagship Velodyne HDL-64E cost $75,000, restricting 3D LiDAR experimentation largely to defense contractors and well-endowed autonomous-driving labs. By 2012 the 32-beam HDL-32E hovered near $30,000, and by LOAM's 2014 debut the 16-beam VLP-16 had fallen to $7,999—still a major line item for an academic lab.

Over the ensuing decade, pricing splintered across a far broader spectrum. In 2019, DJI-backed Livox [announced a US retail price of $599 for the Mid-40](https://www.livoxtech.com/news/1); that same year Ouster's 128-channel OS1-128 retailed at $18,000, and in 2020 Ouster [announced a $600 target price for its solid-state ES2 in 2024 series-production programs](https://investors.ouster.com/news-releases/news-release-details/ouster-announces-first-high-performance-true-solid-state-digital). While sub-thousand-dollar units marked a genuine inflection point, those announcements do not imply a uniform 100-fold price drop across all LiDARs, given wide differences in beam count, range, field of view, and retail versus automotive-volume pricing.

Cheaper sensors also brought fresh algorithmic headaches. Solid-state LiDARs typically offer narrower fields of view than 360° spinning turrets, and several scan in non-repetitive rosette patterns—conditions under which LOAM's ring-based edge/plane logic breaks down. By contrast, FAST-LIO and FAST-LIO2 were built to accommodate both spinning and solid-state sensors, operating reliably under narrow fields of view and irregular sampling. Falling hardware prices thus transformed the algorithmic problem rather than solving it.

---

## 21.6 Why the Visual and LiDAR lineages split

Despite pursuing the same spatial estimation goal over the same decades, Visual SLAM and LiDAR SLAM operated almost as separate disciplines. Several structural forces kept them apart:

Measurement physics drove divergent math. Cameras capture dense texture and appearance, nudging researchers toward keypoints, descriptors, and photometric residuals; LiDAR directly measures 3D range, favoring edges, planes, surfels, and range-image geometry.

Institutional venues reinforced the divide. Visual SLAM lived primarily at CVPR and ICCV, whereas LiDAR SLAM anchored ICRA, IROS, and RSS—and during the early-to-mid 2010s, as Velodyne hardware flowed into Google and autonomous-driving startups, LiDAR expertise concentrated heavily in self-driving teams.

Even loop closure evolved incompatible vocabularies. Camera systems matched appearance via DBoW2 or NetVLAD, while LiDAR systems encoded 3D structural signatures with descriptors such as [Scan Context (Kim·Kim, 2018)](https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf) and [PointNetVLAD](https://arxiv.org/abs/1804.03492): for the very same street corner, the two sensors were recognizing fundamentally different signals.

Early bridges finally emerged in the early 2020s as LiDAR-camera fusion reached top vision and robotics venues. Tixiao Shan's [LVI-SAM (2021)](https://arxiv.org/abs/2104.10831) grafted a visual-inertial subsystem onto LIO-SAM; although framed as a tightly coupled factor graph, its LiDAR-inertial (LIS) and visual-inertial (VIS) branches still run largely as semi-independent trackers that rescue each other upon failure, leaving a single seamlessly unified state estimator unfinished.

---

## 21.7 Visual-LiDAR convergence attempts: 2024–2025

From 2024 onward, sensor-agnostic foundation models renewed efforts to unite cameras and LiDAR inside one framework along two main paths. One line adapts the contrastive alignment of [CLIP (Radford et al., 2021)](https://arxiv.org/abs/2103.00020) from image-text pairs to LiDAR-camera pairs, mapping both modalities into a shared pretrained embedding space—work that remained largely experimental through 2023–2024. The other converts laser returns and camera pixels into common geometric primitives or neural fields handled by one backend, likewise remaining largely at the paper-prototype stage without widespread real-time deployment. By 2026, no unified mainstream lineage had displaced standalone specialists: FAST-LIO2 and ORB-SLAM3 continued to dominate their respective domains independently.

---

## Prediction vs. outcome

> In the conclusion of their 2014 LOAM paper, Zhang and Singh explicitly highlighted two items for future work: adding loop closure to bound long-term drift and fusing IMU measurements via a Kalman filter. Both materialized within a decade—VIO-style tightly coupled iEKF fusion in FAST-LIO (2021) and FAST-LIO2 (2022), and factor-graph loop closure in LIO-SAM (2020). What the 2014 conclusion left unmentioned was dynamic-object handling. By 2026, stripping moving pedestrians and vehicles from LiDAR scans in real time leaned heavily on deep-learning segmentation alongside geometric filters, with no single consensus solution across all dynamic scenes.

---

## Still open

**Full Visual+LiDAR fusion.** Even after LVI-SAM, no tightly coupled visual-LiDAR state estimator has emerged as a universal standard. Autonomous vehicles rely on LiDAR when rain or fog blinds cameras, yet cross-sensor calibration, asymmetric failure modes, and unified estimator design remain stumbling blocks—and 2024–2025 transformer-based fusion prototypes had yet to leave the lab.

**Algorithms optimized for solid-state LiDAR.** Classic LOAM baked in the ring structure of a 360° rotating scanner. Non-repetitive scan patterns (as in Livox units) and narrow fields of view alter motion distortion and geometric observability; while FAST-LIO2's direct point-to-map formulation and Livox LOAM tackle parts of that gap, no single framework spans the full diversity of solid-state optics and scan schedules.

**Dynamic object handling.** From Zhang's 2014 paper through 2026, mainstream LiDAR SLAM largely retained a static-world assumption. Real-time pipelines routinely offload moving-object masking to external segmentation networks while purely geometric in-loop filtering battles compute and stability limits; commercial autonomous-driving stacks remain proprietary, leaving open research without a settled standard.

---

Camera SLAM and LiDAR SLAM each matured into formidable engineering traditions, speaking separate technical dialects while converging only partially at their borders. Yet alongside both mainstreams ran quieter, more unorthodox lineages—biologically inspired cognitive maps, microsecond event cameras, object-centric semantic worlds—that never claimed the center of either community, yet left lasting marks on where SLAM is heading next.
