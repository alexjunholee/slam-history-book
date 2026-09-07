# Ch.18 — Failed Cases and Lost Lineages

While the camera and LiDAR communities developed their own methods, other robotics researchers followed different directions. Some of these lines began with RatSLAM in 2004, well before LOAM; others ran through the 2010s. They did not become mainstream, but they remained part of SLAM's history.

These approaches had identifiable sources. Milford and Wyeth's RatSLAM (2004) drew on O'Keefe and Dostrovsky's 1971 place-cell work through cognitive-map theory. Event SLAM inherited the silicon-retina line through Lichtsteiner, Posch, and Delbruck's 2008 DVS at ETH Zürich INI. Salas-Moreno et al.'s SLAM++ (2013) extended 1990s object-level scene understanding into the SLAM state. Each encountered a different engineering limit.

Some approaches accumulated papers and promising early results without entering the mainstream. They reached scaling limits or lost adoption to a more practical alternative, outcomes distinct from a failure of the underlying technique.

---

## 18.1 RatSLAM — place cell-based topological map

RatSLAM, presented by [Milford et al. 2004](https://doi.org/10.1109/ROBOT.2004.1302555) at ICRA 2004, approached place recognition through a biological model. It imitated the firing patterns of **place cells** and **head direction cells** in the rat hippocampus to form a place representation during exploration. The computational model was a **Continuous Attractor Network (CAN)**. Its neurons form a continuous activation 'bump' on a 2D grid, which moves according to the robot's velocity and rotation input (path integration). Visual input is compared with stored place representations and corrects the bump's position. RatSLAM alternates between propagation from motion and correction from visual matching.

> 🔗 **Borrowed.** The place-cell discovery by [O'Keefe and Dostrovsky (1971)](https://pubmed.ncbi.nlm.nih.gov/5124915/) began in neuroscience and led to the theory of cognitive maps. RatSLAM was an early complete implementation of that biological mechanism in an engineering system, but few later SLAM systems adopted it directly.

Milford and Gordon Wyeth, based at the Queensland University of Technology (QUT) robotics lab, repeatedly tested RatSLAM on suburban roads in Brisbane between 2004 and 2008. A roof-mounted camera supplied the image stream as the system recognized previously traveled routes and closed loops. The [Milford & Wyeth 2008](https://doi.org/10.1109/TRO.2008.2004520) IEEE T-RO paper reported tens of thousands of images over a 66 km route. Contemporary geometric SLAM systems often operated over only a few hundred meters, so RatSLAM had a much larger demonstrated range.

The system did not scale much further. CAN grew in computational complexity with the number of places, and its topological map could recognize a return without reliably producing meter-level metric positions. Autonomous driving and manipulation needed precise coordinates that this cognitive-map formulation did not supply.

> 📜 **Prediction vs. outcome.** In the conclusion of the 2008 T-RO paper, Milford and Wyeth described RatSLAM as "an alternative approach to vision-only SLAM" and reported repeatable, reliable loop closure on long routes with large accumulated error and visual ambiguity. They presented it as an alternative, not a replacement. RatSLAM remained competitive on specific benchmarks, but after 2012 graph-based SLAM and visual odometry moved ahead in accuracy and speed. Topological maps persisted in some place-recognition work, while RatSLAM's metric-topological integration did not continue as a major system lineage.

RatSLAM's algorithm saw limited adoption, but its geometry-free place representation entered the place-recognition literature. In 2012, [SeqSLAM](https://doi.org/10.1109/ICRA.2012.6224623) emerged from the same Milford group, and image-sequence-based recognition became one line of visual place-recognition benchmarks.

---

## 18.2 The engineering limits of biologically-inspired SLAM

RatSLAM was the most complete case of biologically inspired SLAM, but not the only one. From the mid-2000s to the early 2010s, researchers proposed SLAM variants based on cognitive maps, entorhinal grid cells, and hippocampal replay. They encountered similar problems.

Biological models describe *how* the brain represents space. Whether the same representation fits an engineering objective is a separate question. Evolution shaped the rat hippocampus for particular environments and behavior, which differ from a robot's operating conditions.

Engineering SLAM requires sub-meter position accuracy, real-time processing, rapid adaptation to new environments, and verifiable error bounds. Cognitive models did not guarantee these properties, leaving a substantial gap between neuroscience and robotics.

The question reopened in the 2020s because representations learned by foundation models invited comparison with cognitive maps. Whether the resemblance can be made precise, or amounts only to analogy, remains unknown.

---

## 18.3 Event SLAM — the gap between hardware and algorithm maturity

The [Dynamic Vision Sensor (DVS)](https://doi.org/10.1109/JSSC.2007.914337), developed by Patrick Lichtsteiner, Christoph Posch, and Tobi Delbruck at the ETH Zürich Institute of Neuroinformatics (INI), was first disclosed at ISSCC 2008. Each pixel independently compares logarithmic light-intensity change against a threshold and asynchronously outputs an event of positive (ON) or negative (OFF) polarity. With no global shutter, it records each pixel's firing time at microsecond resolution, producing a camera without frames.

> 🔗 **Borrowed.** The DVS event sensor (Lichtsteiner et al. 2008) was hardware inspired by the change-detection mechanism of the biological retina. Event SLAM started with this sensor in hand. Hardware ran ahead of algorithms, and closing that gap took ten years.

Event cameras offered μs-level temporal resolution, little blur during high-speed motion, high dynamic range (HDR) across tunnels and direct sunlight, and a fraction of the power consumption of conventional cameras.

At ICRA 2014, [Weikersdorfer et al. 2014](https://doi.org/10.1109/ICRA.2014.6906882) presented event-based 3D SLAM. The same year, other groups released event-based optical-flow and depth-estimation methods. Between 2016 and 2018, Henri Rebecq in Davide Scaramuzza's RPG lab at the University of Zurich released [EVO](https://doi.org/10.1109/LRA.2016.2645143) (RA-L 2017) and [ESIM](https://proceedings.mlr.press/v87/rebecq18a.html) (CoRL 2018), completing more of the event-SLAM pipeline.

Results in real environments remained limited. Early DVS sensors had 128×128 pixels rather than VGA resolution, a serious constraint for feature matching and map building. Existing frame-based algorithms also did not apply directly to event streams, so the field needed new methods as well as better hardware.

From 2014 to 2018, event SLAM produced good results in controlled environments and low-texture conditions, but did not outperform existing visual-inertial odometry in general environments.

The approach also spread beyond odometry. [EventVLAD](https://ieeexplore.ieee.org/document/9635907/) (Lee & Kim, IROS 2021) combined edge images reconstructed from event streams with NetVLAD descriptors, demonstrating place recognition under sudden illumination changes and motion blur, conditions that challenged frame-based VPR.

---

## 18.4 Semantic SLAM — the shrinking of the object-as-landmark path

From 2017 to 2019, semantic methods appeared throughout the programs of CVPR, ECCV, and IROS. Deep learning was improving instance segmentation and object detection, and researchers widely explored their integration with SLAM. Implementations progressed more slowly than the surrounding claims.

[Salas-Moreno et al. 2013](https://doi.org/10.1109/CVPR.2013.178)'s **SLAM++** was an early large-scale system in this lineage. Salas-Moreno and his advisor Andrew Davison at Imperial College used *objects* rather than points or patches as the map's basic unit. They stored predefined 3D models of chairs, desks, and monitors in a database, recognized them in RGB-D input through ICP (Iterative Closest Point) alignment, and placed them on the map. Tens of objects could replace thousands of points, reducing map size and grounding place recognition and loop closure in semantic entities.

> 🔗 **Borrowed.** SLAM++'s object-level representation combined scene graphs from graphics with model-based recognition from computer vision. The approach later reappeared in LERF (Language Embedded Radiance Field) and LangSplat in the 2020s, with language features replacing objects as the representation unit.

After SLAM++, [SemanticFusion](https://arxiv.org/abs/1609.05130) (McCormac et al., 2017, ICRA) and [MaskFusion](https://arxiv.org/abs/1804.09194) (Rünz et al., 2018, ISMAR) used semantic information in mapping and dynamic-object separation. [SuperPoint](https://arxiv.org/abs/1712.07629) (DeTone et al., 2018)-based systems appeared in the same period, but SuperPoint is not a semantic feature: it jointly learns a keypoint detector and local descriptor. Both lines used learning, but they addressed different problems and representation units. Semantic-SLAM papers argued that integrating semantic understanding with geometric pipelines could make maps more robust to environmental change.

Through 2019, traditional geometric pipelines such as ORB-SLAM2, VINS-Mono, and LIO-SAM produced the main gains on autonomous-driving benchmarks. Systems with deep semantic features remained competitive only in specific indoor environments and with fixed object classes. On new categories or unseen environments, semantic priors sometimes increased drift.

> 📜 **Prediction vs. outcome.** In the Conclusion of the SLAM++ paper, Salas-Moreno described his method as "a first step toward a more generic SLAM method," hoping it would extend to objects with low-dimensional shape variation, and ultimately to systems that segment and define object classes on their own. The paper's introduction added that an object-unit representation would bring "large map compression" and "gains in efficiency and robustness." The actual development partially hit the mark. Object-level maps found a place in AR and certain manipulation applications, and the compression and efficiency advantages were confirmed again in indoor environments with repeated objects. But mainstream geometric SLAM still retains sparse points and keyframe-based graphs as of 2026, and the stage where objects are segmented and defined autonomously has not been reached. Object-as-landmark adoption remained limited, while semantics found other roles in dynamic-region separation within SLAM and in downstream semantic mapping and task planning.

Semantic-first SLAM depended on accurate segmentation; a segmentation error could corrupt the map, whereas robust estimation let a geometric pipeline survive some incorrect matches. Generalization posed a second problem. Semantic priors trained on particular object classes did not transfer beyond those classes, while SLAM systems had to operate in a much wider range of environments.

The object-as-landmark path contracted, but semantic information continued at a different map layer. [SuMa++](https://doi.org/10.1109/IROS40897.2019.8967704) (Chen et al., IROS 2019) overlaid semantic classes on LiDAR point clouds to filter dynamic objects, and [Kimera](https://doi.org/10.1109/ICRA40945.2020.9196885) (Rosinol et al., ICRA 2020) combined a metric-semantic mesh with a 3D scene graph. [Hydra](https://doi.org/10.15607/RSS.2022.XVIII.050) (Hughes et al., RSS 2022) extended the graph into a real-time hierarchy. [ConceptGraphs](https://doi.org/10.1109/ICRA57147.2024.10610243) (Gu et al., ICRA 2024) and [Clio](https://doi.org/10.1109/LRA.2024.3451395) (Maggio et al., RA-L 2024) later added open-vocabulary foundation features. This lineage remained active in 2026 and reappears in [Ch.15b](chapter_15b_dynamic.md), [Ch.16](chapter_16_foundation_3d.md), and [Ch.19 §19.7](chapter_19_open_problems.md#197-the-return-of-semantic-representation-and-open-world).

---

## 18.5 The Manhattan-world assumption — scope and continued use as an auxiliary constraint

Another line of research used the Manhattan-world assumption and later receded.

Following the Manhattan-world concept of [Coughlan & Yuille 1999](https://doi.org/10.1109/ICCV.1999.790349), these methods treated indoor walls, floors, and ceilings as aligned with three orthogonal axes (x, y, z) of the world coordinate frame. Parallel image lines converge to vanishing points, each described by the relation `v = K R d` between the camera rotation matrix R and direction vector d (K is the camera intrinsic matrix). Three orthogonal vanishing points allow direct recovery of the three columns of R. Visual-odometry systems could use this geometric constraint to suppress drift without an IMU or feature matching.

The constraint reduced drift in long corridors and rectangular rooms but failed outdoors, around curved structures, and in irregular industrial environments where the assumption did not hold. A prior fitted tightly to one environment became a liability elsewhere. As general-purpose visual-inertial odometry matured after 2015, Manhattan-world methods receded. The assumption remains an auxiliary constraint in some indoor mapping tools rather than an independent research lineage.

---

## 18.6 Rediscovery patterns of extinct lineages

The discontinued lineages left different kinds of descendants. RatSLAM's topological-map idea carried into SeqSLAM and visual place recognition. SLAM++'s object-level map returned in another form after 2022 when NeRF and Gaussian splatting were combined with language features, as in [LERF](https://arxiv.org/abs/2303.09553) (Kerr et al., 2023) and [LangSplat](https://arxiv.org/abs/2312.16084) (Qin et al., 2023).

Event-camera SLAM followed a different path because early hardware remained limited. After 2022, event cameras above 640×480 reached the market, and high-speed drones and HDR environments supplied clear applications. Between 2020 and 2024, the event-vision community around [Guillermo Gallego](https://arxiv.org/abs/1904.08405) at TU Berlin reported competitive results in event-based depth and ego-motion estimation.

A promising idea can still wait years for suitable hardware and algorithms. Adoption also depends on whether a more practical alternative becomes established before both mature.

---

## 🧭 Still open

**Biologically inspired SLAM.** Spatial representations formed by foundation models through large-scale unsupervised learning have structural similarities to cognitive maps. Whether a transformer's internal representation implements anything comparable to a place cell remains unverified. A RatSLAM-like lineage returning through foundation models is therefore a hypothesis, not an observed convergence.

**Event-camera SLAM adoption.** Commercial high-resolution event cameras widened the research base after 2022, but event processing had not settled on a stable common framework by 2026. Integration with frame-based pipelines, event representations, real-world benchmarks, and evaluation standards were all still developing. Broad adoption remained uncertain.

**The direction of the semantic-map concept.** As interest in semantic SLAM cooled after 2017, semantic representation developed other roles in dynamic-region separation and downstream tasks. From 2023, LERF and language-based Gaussian-splatting systems combined language features with dense scene representations. It remained unclear whether semantics would become part of SLAM itself or stay downstream, and whether these representations could relax the usual requirement that geometry be reliable first.

Full visual-LiDAR fusion, solid-state sensor algorithms, dynamic-object handling, event-camera maturity, and the return of semantic maps all connect with unresolved problems from earlier chapters; Ch.19 brings those threads together.
