# Chapter 22. Lost Lineages

While camera and LiDAR researchers built their respective mainstreams, others struck out along very different paths—some beginning with RatSLAM in 2004, well before LOAM, and others unfolding across the 2010s. Though none became the dominant paradigm, they left lasting imprints on SLAM's trajectory.

Each lineage sprang from a distinct scientific or hardware root: Milford and Wyeth's RatSLAM (2004) drew on O'Keefe and Dostrovsky's 1971 hippocampal place-cell discovery through cognitive-map theory; Event SLAM grew out of the silicon-retina tradition sparked by Lichtsteiner, Posch, and Delbruck's 2008 DVS sensor at ETH Zürich INI; and Salas-Moreno et al.'s SLAM++ (2013) pushed 1990s object-level scene understanding straight into the SLAM state vector. Along the way, each hit a different engineering wall. Yet running into a scaling ceiling or losing adoption to a more practical rival is not the same as an intellectual failure—and several of these "lost" ideas later resurfaced in unexpected forms.

---

## 22.1 RatSLAM — place cell-based topological map

At ICRA 2004, [Milford et al. 2004](https://doi.org/10.1109/ROBOT.2004.1302555) bypassed metric landmark geometry entirely, modeling place recognition after biology. **RatSLAM** mimicked **place cells** and **head direction cells** in the rodent hippocampus using a **Continuous Attractor Network (CAN)**: a 2D sheet of neurons sustaining a localized "bump" of activity that shifts with velocity and rotation commands (path integration) and snaps back whenever incoming camera views match stored place templates.

> 🔗 **Borrowed.** [O'Keefe and Dostrovsky (1971)](https://pubmed.ncbi.nlm.nih.gov/5124915/) discovered hippocampal place cells in neuroscience, laying the foundation for cognitive-map theory. RatSLAM translated that biological mechanism into a full engineering pipeline, though few mainstream SLAM systems later adopted it directly.

Between 2004 and 2008, Michael Milford and Gordon Wyeth at the Queensland University of Technology (QUT) put RatSLAM through demanding road trials across suburban Brisbane using a roof-mounted camera. In [Milford & Wyeth 2008](https://doi.org/10.1109/TRO.2008.2004520) (IEEE T-RO), it closed loops across tens of thousands of images over a 66 km route at a time when contemporary metric SLAM struggled beyond a few hundred meters.

Yet RatSLAM hit a wall when pressed for precision and scale. CAN computation grew with the number of stored places, and its topological graph excelled at recognizing "I have been here before" without reliably delivering meter-accurate 3D coordinates—the very quantities autonomous driving and robotic manipulation demanded.

> 📜 **Prediction vs. outcome.** Concluding their 2008 T-RO paper, Milford and Wyeth pitched RatSLAM as "an alternative approach to vision-only SLAM," highlighting repeatable loop closure over long routes despite drift and visual ambiguity—an alternative rather than a wholesale replacement. Although RatSLAM held its own on specialized benchmarks, post-2012 graph SLAM and visual odometry surged ahead in metric accuracy and speed; topological maps persisted within place recognition, while RatSLAM's metric-topological hybrid faded as a central system lineage.

Even as the full algorithm receded, its geometry-free view of place recognition took root. In 2012, Milford's group spun that insight into [SeqSLAM](https://doi.org/10.1109/ICRA.2012.6224623), turning image-sequence matching into a staple of visual place-recognition benchmarks.

---

## 22.2 The engineering limits of biologically-inspired SLAM

RatSLAM was the fullest expression of bio-inspired SLAM, yet not an isolated one: from the mid-2000s into the early 2010s, researchers explored SLAM designs grounded in cognitive maps, entorhinal grid cells, and hippocampal replay, only to encounter the same divide.

Neuroscience models *how* biological brains represent space; whether those mechanisms serve an engineering specification is another matter altogether. Evolution tuned the rat hippocampus for foraging and survival rather than sub-meter metric accuracy, deterministic real-time latency, rapid zero-shot deployment, or verifiable error bounds. Cognitive models offered no guarantees on those fronts, leaving a wide gap between biological plausibility and robotic utility.

Curiously, the question reopened in the 2020s as large-scale foundation models formed internal spatial representations that invited comparison with cognitive maps—though whether that parallel runs deeper than metaphor remains unresolved.

---

## 22.3 Event SLAM — the gap between hardware and algorithm maturity

In 2008, Patrick Lichtsteiner, Christoph Posch, and Tobi Delbruck at the ETH Zürich Institute of Neuroinformatics (INI) unveiled the [Dynamic Vision Sensor (DVS)](https://doi.org/10.1109/JSSC.2007.914337) at ISSCC. Instead of reading out full frames on a global shutter, each pixel independently monitors logarithmic brightness changes and fires asynchronous positive (ON) or negative (OFF) events with microsecond timestamps—a camera with no frames at all.

> 🔗 **Borrowed.** Inspired by change detection in the biological retina, the DVS event camera (Lichtsteiner et al. 2008) handed SLAM researchers new hardware before algorithms existed to exploit it—touching off a decade-long effort to close the software gap.

On paper, event cameras looked made for agile robotics: microsecond temporal resolution, near-immunity to motion blur during violent maneuvers, high dynamic range (HDR) spanning dark tunnels and glaring sunlight, and a fraction of the power draw of frame cameras.

Pioneering pipelines soon followed. At ICRA 2014, [Weikersdorfer et al. 2014](https://doi.org/10.1109/ICRA.2014.6906882) demonstrated event-based 3D SLAM alongside parallel event flow and depth estimators, and between 2016 and 2018 Henri Rebecq in Davide Scaramuzza's RPG lab at the University of Zurich built key infrastructure in [EVO](https://doi.org/10.1109/LRA.2016.2645143) (RA-L 2017) and the simulator [ESIM](https://proceedings.mlr.press/v87/rebecq18a.html) (CoRL 2018).

Out in everyday environments, however, early deployments ran into twin constraints. First-generation DVS chips offered just 128×128 pixels—far below VGA—starving feature matching and mapping of spatial detail, while frame-based vision algorithms refused to map cleanly onto asynchronous event streams. From 2014 to 2018, event SLAM shone in controlled high-speed or low-texture setups without overtaking conventional visual-inertial odometry in general scenes. Still, the paradigm branched productively: [EventVLAD](https://ieeexplore.ieee.org/document/9635907/) (Lee & Kim, IROS 2021), for instance, reconstructed edge images from event streams and fed them to NetVLAD, achieving resilient place recognition amid abrupt lighting shifts and severe blur where standard VPR failed.

---

## 22.4 Semantic SLAM — the shrinking of the object-as-landmark path

Between 2017 and 2019, semantic SLAM filled the programs of CVPR, ECCV, and IROS. Driven by rapid gains in deep object detection and instance segmentation, researchers sought to build maps out of meaningful objects rather than anonymous points—yet real-world implementations lagged far behind the rhetoric.

The touchstone predecessor was [Salas-Moreno et al. 2013](https://doi.org/10.1109/CVPR.2013.178)'s **SLAM++**, built with Andrew Davison at Imperial College. Instead of storing thousands of points or patches, SLAM++ treated *objects*—chairs, desks, monitors matched via ICP against a catalog of known 3D CAD models—as the map's atomic landmarks, compressing map size while anchoring loop closure in recognizable entities.

> 🔗 **Borrowed.** SLAM++ fused computer-graphics scene graphs with model-based 3D object recognition. A decade later, that object-centric impulse resurfaced in LERF and LangSplat, substituting open-vocabulary language features for rigid CAD catalogs.

Subsequent systems such as [SemanticFusion](https://arxiv.org/abs/1609.05130) (McCormac et al., 2017, ICRA) and [MaskFusion](https://arxiv.org/abs/1804.09194) (Rünz et al., 2018, ISMAR) wove semantic masks into dense mapping and moving-object separation. Learned keypoint pipelines like [SuperPoint](https://arxiv.org/abs/1712.07629) (DeTone et al., 2018) emerged alongside them, though SuperPoint jointly learns a local keypoint detector and descriptor rather than semantic labels—a distinct problem despite sharing deep-learning roots. Across the semantic wave, proponents argued that injecting object understanding into geometry would make maps resilient to real-world change.

Through 2019, however, benchmark gains on autonomous-driving and general navigation still belonged overwhelmingly to purely geometric pipelines—ORB-SLAM2, VINS-Mono, and LIO-SAM. Semantic-first SLAM stumbled on two structural vulnerabilities: unlike geometric outliers, which RANSAC or robust kernels can shrug off, a misclassified semantic mask could corrupt the map directly; and object priors trained on fixed indoor categories broke down whenever robots encountered unfamiliar objects or outdoor clutter.

> 📜 **Prediction vs. outcome.** In the Conclusion of SLAM++, Salas-Moreno framed the system as "a first step toward a more generic SLAM method," envisioning extension to deformable object families and autonomous open-ended object discovery, while the introduction promised "large map compression" and "gains in efficiency and robustness." History bore out part of that bet: object-centric maps carved out niches in AR and robotic manipulation, confirming compression and efficiency gains indoors where repeated furniture abounds. Yet by 2026 mainstream geometric SLAM still rested on sparse points and keyframe graphs, autonomous open-set object discovery inside SLAM remained unrealized, and semantics instead thrived in dynamic masking and higher-level planning.

As the object-as-landmark path contracted, semantic information migrated upward to sit *above* the geometric map. [SuMa++](https://doi.org/10.1109/IROS40897.2019.8967704) (Chen et al., IROS 2019) draped semantic labels onto LiDAR surfels to filter dynamic cars; [Kimera](https://doi.org/10.1109/ICRA40945.2020.9196885) (Rosinol et al., ICRA 2020) coupled a metric-semantic mesh with a 3D scene graph; [Hydra](https://doi.org/10.15607/RSS.2022.XVIII.050) (Hughes et al., RSS 2022) turned that graph into a real-time hierarchy; and [ConceptGraphs](https://doi.org/10.1109/ICRA57147.2024.10610243) (Gu et al., ICRA 2024) alongside [Clio](https://doi.org/10.1109/LRA.2024.3451395) (Maggio et al., RA-L 2024) infused open-vocabulary foundation features—an active 2026 lineage examined further in [Chapter 19](chapter_15b_dynamic.md), [Chapter 20](chapter_16_foundation_3d.md), and [Chapter 23 §23.7](chapter_19_open_problems.md#237-the-return-of-semantic-representation-and-open-world).

---

## 22.5 The Manhattan-world assumption — scope and continued use as an auxiliary constraint

Another once-prominent branch—Manhattan-world SLAM—faded as general-purpose sensors matured.

Tracing back to [Coughlan & Yuille 1999](https://doi.org/10.1109/ICCV.1999.790349), Manhattan-world methods assumed that indoor walls, floors, and ceilings align with three mutually orthogonal axes (x, y, z). Parallel edges in the scene project to vanishing points governed by `v = K R d`, tying the camera rotation matrix R directly to scene direction d through intrinsic matrix K. Observing three orthogonal vanishing points pins down the columns of R directly, letting visual odometry curb orientation drift without an IMU or even feature matching.

In long rectangular corridors and office blocks, that structural prior cut rotational drift dramatically—only to break outdoors, along curved walls, or inside irregular industrial plants. As tightly coupled visual-inertial odometry matured after 2015, hard-coding architectural orthogonality became unnecessary risk; Manhattan-world constraints survive today as optional priors in specialized indoor mappers rather than a standalone SLAM lineage.

---

## 22.6 Rediscovery patterns of extinct lineages

Even lineages that fell out of the mainstream found second lives in altered form. RatSLAM's topological intuition reappeared in SeqSLAM and modern visual place recognition, while SLAM++'s object-level ambitions returned after 2022 as language-embedded NeRFs and Gaussian splats—most visibly in [LERF](https://arxiv.org/abs/2303.09553) (Kerr et al., 2023) and [LangSplat](https://arxiv.org/abs/2312.16084) (Qin et al., 2023).

Event-camera SLAM illustrates a different rhythm—one governed by hardware lag. Once event sensors exceeding 640×480 resolution reached commercial markets after 2022, high-speed drones and extreme-HDR scenes gave them clear niches, and between 2020 and 2024 the event-vision community around [Guillermo Gallego](https://arxiv.org/abs/1904.08405) at TU Berlin delivered competitive ego-motion and depth results.

Across all these cases, technical promise alone never settles adoption: ideas wait for silicon and software to mature together, and risk being overtaken if a simpler practical alternative consolidates first.

---

## Still open

**Biologically inspired SLAM.** Spatial embeddings learned by foundation models through massive self-supervision bear an intriguing resemblance to cognitive maps. Whether transformer internals implement anything genuinely analogous to hippocampal place cells remains unproven—making a RatSLAM revival via foundation models a tantalizing hypothesis rather than an established convergence.

**Event-camera SLAM adoption.** Although commercial high-resolution event sensors broadened the research community after 2022, by 2026 event SLAM had yet to settle on a canonical pipeline. Hybrid event-frame fusion, standard event representations, and unified real-world benchmarks were all still evolving, leaving mass adoption uncertain.

**The direction of the semantic-map concept.** After object-as-landmark SLAM cooled post-2017, semantics migrated into dynamic masking and high-level planning, and from 2023 onward LERF and language-grounded 3DGS fused open-vocabulary embeddings directly into dense visual maps. Whether semantics ultimately folds back into core SLAM estimation or remains an upper planning layer—and whether it can loosen SLAM's dependence on rock-solid geometry first—remains an open question.

Visual-LiDAR fusion, solid-state laser algorithms, dynamic environments, event-camera maturation, and the return of semantic mapping all point toward the same underlying fault lines; Chapter 23 draws these open threads together across the entire book.
