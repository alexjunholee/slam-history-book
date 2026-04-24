# Ch.18 — Failed Cases and Lost Lineages

Across the same decades the camera and LiDAR lineages were building their grammars, other people inside the robotics community were walking in different directions. Lineages that were neither the camera lineage nor the LiDAR one — some began as early as RatSLAM in 2004, well before LOAM, others ran in parallel through the 2010s. That they were not chosen does not mean they were absent from the history.

Each dead lineage traced here has a live ancestor. Milford & Wyeth's RatSLAM (2004) pulls from O'Keefe & Dostrovsky's 1971 place-cell work by way of cognitive-map theory. Event SLAM inherits the silicon-retina line through Lichtsteiner, Posch, & Delbruck's 2008 DVS at ETH Zürich INI. Salas-Moreno et al.'s SLAM++ (2013) extends 1990s object-level scene understanding into the SLAM state. The chapter traces where those inheritances hit walls.

The history of SLAM is not made up only of successful lineages. Every decade there were approaches that accumulated enough papers and early results yet failed to enter the mainstream — engineering scaling hit a wall, or a more practical alternative took the seat first. That was a different matter from technical failure.

---

## 18.1 RatSLAM — place cell-based topological map

RatSLAM, presented by [Milford et al. 2004](https://doi.org/10.1109/ROBOT.2004.1302555) at ICRA 2004, approached the place recognition problem in an entirely different way. It imitated the firing patterns of **place cells** and **head direction cells** in the rat hippocampus so that the robot would form a place representation naturally while exploring. The computational model was called a **Continuous Attractor Network (CAN)**. The neurons' activation state forms a continuous activation 'bump' on a 2D grid, and the bump moves along the grid according to the robot's velocity and rotation input (path integration). When visual input arrives, it is compared against stored place representations and the bump position is corrected. This loop — bump propagation from motion, correction from visual matching — is the core operating principle of RatSLAM.

> 🔗 **Borrowed.** The place cell discovery by [O'Keefe and Dostrovsky (1971)](https://pubmed.ncbi.nlm.nih.gov/5124915/) began in neuroscience and led to the theory of cognitive maps. RatSLAM was the first completed attempt to carry that biological mechanism into an engineering system. The door opened, but not many engineers walked through it.

Milford and Gordon Wyeth, based at the Queensland University of Technology (QUT) robotics lab, ran outdoor driving experiments repeatedly on suburban roads in Brisbane between 2004 and 2008. The test vehicle mounted a camera on the roof and drove through suburban residential areas. RatSLAM received that image stream, recognized paths it had already traveled, and closed loops. The [Milford & Wyeth 2008](https://doi.org/10.1109/TRO.2008.2004520) IEEE T-RO paper reported results from processing tens of thousands of images over a 66 km route. At the same time, geometric SLAM systems were struggling at the scale of a few hundred meters, so by the numbers alone RatSLAM was ahead.

The engineering scaling stopped there, however. CAN grew in computational complexity as the number of places increased. The deeper problem was precision. The topological map RatSLAM built could judge "I have been here before," but it could not reliably produce metric position estimates at the meter level. Autonomous driving and manipulation demanded exact coordinates. Cognitive maps did not fit that demand.

> 📜 **Prediction vs. outcome.** In the Conclusion of the 2008 T-RO paper Milford and Wyeth argued that RatSLAM was "an alternative approach to vision-only SLAM" and that it performed repeatable, reliable loop closure in environments — long routes, large accumulated error, visual ambiguity — that would challenge the contemporary state-of-the-art SLAM. The claim was a position as an alternative, not a replacement. The actual development was partially right on the claim itself. RatSLAM showed competitiveness on specific benchmarks. But in the overall flow of the field after 2012, graph-based SLAM and visual odometry pulled ahead on both accuracy and speed, and while topological maps still appear in some place recognition research, the original RatSLAM ambition of metric-topological integration did not carry forward in another form. `[partial hit + abandoned]`

What RatSLAM left behind was not the algorithm itself. The idea that "place representation is possible without geometry" seeped into the place recognition literature. In 2012 [SeqSLAM](https://doi.org/10.1109/ICRA.2012.6224623) came out of the same Milford group, and image-sequence-based place recognition became one axis of visual place recognition benchmarks. The lineage itself survived, only its form had changed.

---

## 18.2 The engineering limits of biologically-inspired SLAM

RatSLAM was the most complete case of biologically-inspired SLAM, but it was not alone. From the mid-2000s to the early 2010s, SLAM variants imitating cognitive maps, entorhinal grid cells, and hippocampal replay came out steadily. They all carried similar problems.

Biological models describe *how* the brain represents space, but *why* it does so in that way, and whether that way also fits engineering purposes, is a different question. The rat hippocampus is a structure that hundreds of millions of years of evolution shaped to match specific environments and behavioral patterns. The conditions a robot operates in are not the same.

Engineering SLAM requires sub-meter position estimation accuracy, real-time processing, fast adaptation to new environments, and verifiable error bounds. Cognitive models had trouble guaranteeing these conditions. Neuroscience and robotics can draw inspiration from each other, but the gap was not short.

Entering the 2020s, there was room for this discussion to reopen. Observations appeared that the way foundation models form large-scale representation on their own structurally resembles the emergent properties of place cells. Whether this is rediscovery or convergence along a different path is not yet known.

---

## 18.3 Event SLAM — the gap between hardware and algorithm maturity

The [Dynamic Vision Sensor (DVS)](https://doi.org/10.1109/JSSC.2007.914337), developed by Patrick Lichtsteiner, Christoph Posch, and Tobi Delbruck at the ETH Zürich Institute of Neuroinformatics (INI), was first disclosed at ISSCC 2008. Each pixel independently compares logarithmic light intensity change against a threshold and outputs an event of positive (ON) or negative (OFF) polarity asynchronously. Without a global shutter, the firing moment is recorded per pixel at microsecond resolution. It was a camera without frames.

> 🔗 **Borrowed.** The DVS event sensor (Lichtsteiner et al. 2008) was hardware inspired by the change-detection mechanism of the biological retina. Event SLAM started with this sensor in hand. Hardware ran ahead of algorithms, and closing that gap took ten years.

The advantages of the event camera were a list that looked good: μs-level temporal resolution, no blur in high-speed motion, high dynamic range (HDR) that handled both tunnels and direct sunlight, and power consumption at a fraction of conventional cameras. Numbers that read well in a paper.

At ICRA 2014 [Weikersdorfer et al. 2014](https://doi.org/10.1109/ICRA.2014.6906882) presented event-based 3D SLAM. The same year, other groups released event-based optical flow and depth estimation. Between 2016 and 2018, Henri Rebecq (Davide Scaramuzza's group, the RPG lab at the University of Zurich) released [EVO](https://doi.org/10.1109/LRA.2016.2645143) (RA-L 2017) and [ESIM](https://proceedings.mlr.press/v87/rebecq18a.html) (CoRL 2018), and the event SLAM pipeline took concrete shape.

In real environments the results fell short of expectations. The first problem was resolution. Early DVS sensors were 128×128 pixels, not comparable to existing VGA cameras, and for SLAM where feature matching and map building depend directly on resolution, this constraint was heavy. The second was the algorithm paradigm itself. Existing frame-based algorithms could not be applied to event streams as-is, and building a new approach took time.

From 2014 to 2018, event SLAM produced good results in controlled environments and low-texture conditions, but did not outperform existing visual-inertial odometry in general environments.

During that same period, the application scope of the event approach quietly spread outside odometry. [EventVLAD](https://ieeexplore.ieee.org/document/9635907/) (Lee & Kim, IROS 2021) bundled edge images reconstructed from event streams with NetVLAD descriptors, showing that place recognition was possible even under sudden illumination changes and motion blur. It was an attempt where event touched environments that frame-based VPR struggled with.

---

## 18.4 Semantic SLAM — the shrinking of the object-as-landmark path

From 2017 to 2019, "semantic" did not leave the session titles at CVPR, ECCV, and IROS. This was the period when deep learning was advancing quickly in instance segmentation and object detection, and the question of what happens when that semantic understanding is integrated into SLAM circulated widely. The question itself was not wrong, but the execution did not keep up with the discourse.

[Salas-Moreno et al. 2013](https://doi.org/10.1109/CVPR.2013.178)'s **SLAM++** was the first large-scale declaration of that lineage. The group of Salas-Moreno and his advisor Andrew Davison at Imperial College took *objects* as the basic unit of the map instead of points or patches. They stored pre-defined 3D object models — chair, desk, monitor — in a database, and during SLAM execution they recognized those objects in RGB-D input via ICP (Iterative Closest Point) alignment and placed them on the map. Representing the map with tens of objects instead of thousands of points could shrink map size and make place recognition and loop closure more semantically grounded.

> 🔗 **Borrowed.** SLAM++'s object-level representation combined the scene graph representation from graphics with model-based recognition from computer vision. That idea carried forward into LERF (Language Embedded Radiance Field) and LangSplat in the 2020s. Only the representation unit changed from object to language feature; the intuition that "the map should be semantic" survived.

After SLAM++, between 2017 and 2019, [SemanticFusion](https://arxiv.org/abs/1609.05130) (McCormac et al., 2017, ICRA), [MaskFusion](https://arxiv.org/abs/1804.09194) (Rünz et al., 2018, ISMAR), and [SuperPoint](https://arxiv.org/abs/1712.07629) (DeTone et al., 2018)-based feature lines were released in succession. The shared claim across that period ran: deep semantic features are more robust to environmental change than geometric features, and SLAM with semantic understanding integrated is the next stage.

The actual development was different. Through 2019 what pushed performance up on autonomous driving benchmarks was traditional geometric pipelines like ORB-SLAM2, VINS-Mono, and LIO-SAM. Systems integrating deep semantic features had competitiveness only in specific indoor environments and with fixed object classes. Cases appeared where semantic priors in new object categories or unseen environments actually increased drift.

> 📜 **Prediction vs. outcome.** In the Conclusion of the SLAM++ paper, Salas-Moreno described his method as "a first step toward a more generic SLAM method," hoping it would extend to objects with low-dimensional shape variation, and ultimately to systems that segment and define object classes on their own. The paper's introduction added that an object-unit representation would bring "large map compression" and "gains in efficiency and robustness." The actual development partially hit the mark. Object-level maps found a place in AR and certain manipulation applications, and the compression and efficiency advantages were confirmed again in indoor environments with repeated objects. But mainstream geometric SLAM still retains sparse points and keyframe-based graphs as of 2026, and the stage where objects are segmented and defined autonomously has not been reached. Semantic ended up settling not inside SLAM but in downstream tasks — semantic mapping, task planning. `[partial hit + diverted]`

Why did semantic-first SLAM fail to become mainstream. One cause was dependency. Semantic SLAM needed segmentation to be accurate, and when segmentation was wrong the whole map was polluted. A geometric pipeline survived partial failure of feature matching through robust estimation. The other cause was generalization. Semantic priors trained on specific object classes were useless outside those classes, and the environments SLAM had to enter were far wider than the world those priors assumed.

What shrank was the object-as-landmark path, not semantic itself. In the same period another path survived. [SuMa++](https://doi.org/10.1109/IROS40897.2019.8967704) (Chen et al., IROS 2019) overlaid semantic classes onto LiDAR point clouds to filter out dynamic objects, and [Kimera](https://doi.org/10.1109/ICRA40945.2020.9196885) (Rosinol et al., ICRA 2020) bundled metric-semantic mesh with a 3D scene graph. [Hydra](https://doi.org/10.15607/RSS.2022.XVIII.050) (Hughes et al., RSS 2022) extended that scene graph in real time and hierarchically, and by [ConceptGraphs](https://doi.org/10.1109/ICRA57147.2024.10610243) (Gu et al., ICRA 2024) and [Clio](https://doi.org/10.1109/LRA.2024.3451395) (Maggio et al., RA-L 2024), open-vocabulary foundation features were layered on top. Semantic survived by rising not to the landmark seat but to a higher layer of the map. This lineage is in progress as of 2026, and is picked up again in [Ch.15b](chapter_15b_dynamic.md) (the semantic return of dynamic-static separation), [Ch.16](chapter_16_foundation_3d.md) (foundation 3D and metric-semantic core), and [Ch.19 §19.7](chapter_19_open_problems.md#197-the-return-of-semantic-representation-and-open-world) (The return of semantic).

---

## 18.5 The Manhattan-world assumption — scope of application and extinction

Around the same time, another lineage was quietly tried and quietly disappeared. It was SLAM using the Manhattan-world assumption.

The assumption itself was simple. Inheriting the Manhattan world concept from [Coughlan & Yuille 1999](https://doi.org/10.1109/ICCV.1999.790349), indoor environments were viewed as mostly aligned to three orthogonal axes (x, y, z) of the world coordinate frame. Walls, floors, and ceilings make those directions. Bundles of parallel lines in the image converge to vanishing points, and each vanishing point is described by the relation `v = K R d` between the camera's rotation matrix R and direction vector d (K: camera intrinsic matrix). Finding three orthogonal vanishing points allows direct recovery of the three columns of R. It meant drift could be suppressed with geometric constraints alone, without an IMU or feature matching. Attempts to combine this idea with visual odometry appeared in this period.

In long corridors and rectangular rooms drift did decrease. The problem was outside. Once outdoors, in environments with round structures, or in irregular industrial environments, the Manhattan-world assumption itself did not hold. A prior tightly fit to an environment became a liability outside that environment. As general-purpose visual-inertial odometry matured after 2015, this lineage lost attention. It remains as an auxiliary constraint in some indoor mapping tools, but as an independent research lineage it is gone.

---

## 18.6 Rediscovery patterns of extinct lineages

What it means for a lineage to die differs by case. RatSLAM's topological map idea carried into SeqSLAM, and its descendants are alive in the visual place recognition field. The object-level map intuition of SLAM++ returned in a different form after 2022, as NeRF and Gaussian splatting combined with language. [LERF](https://arxiv.org/abs/2303.09553) (Kerr et al., 2023) and [LangSplat](https://arxiv.org/abs/2312.16084) (Qin et al., 2023) are such cases.

Event camera SLAM followed a different path. The algorithms were not stuck; the hardware had not come that far yet. After 2022, event cameras above 640×480 came to market, and the need in high-speed drones and HDR environments became clear. The event vision community centered around [Guillermo Gallego](https://arxiv.org/abs/1904.08405) (TU Berlin) produced competitive results in event-based depth estimation and ego-motion estimation between 2020 and 2024.

Inspiration can be good yet engineering takes time to catch up, and even when the sensor is new the algorithm has to be built separately. How long it takes to close that gap depends on algorithm maturity, on how fast the hardware reaches practical use, and on whether a better alternative takes the seat first in between.

---

## 🧭 Still open

**Biologically-inspired SLAM.** The way foundation models form spatial representations through large-scale unsupervised learning has properties structurally similar to cognitive maps. Reports that place-cell-like units have been observed inside transformers appeared in 2023–2024. Whether this is convergence or coincidence is not known. There is a possibility that the RatSLAM-class lineage returns under a different name inside the foundation model paradigm.

**Event camera SLAM going mainstream.** After 2022, as commercial high-resolution event cameras spread, the research base widened. But the algorithm paradigm for processing event data effectively has not yet settled on a stable common framework. Integration with frame-based pipelines, new event representations, and the diversification of real-world benchmarks together with the establishment of evaluation standards are all in progress simultaneously. Whether it becomes mainstream is still too early to judge as of 2026.

**The direction of the "semantic map" concept.** After the 2017 overheating of semantic SLAM cooled, semantic representation was pushed outside SLAM — to downstream tasks. In 2024–2025, LERF and Gaussian splatting combined language features with dense scene representation, and another form appeared. Whether it will lead to internalization or remain downstream again is not known. Whether the pattern that geometry has to be right first for semantic to be useful will repeat this time, or whether a change in representation itself will overturn that order, is the question. Among the things treated as "solved" as of 2026, the name someone in the future will add to this chapter is still among those not yet out.

The open threads from Ch.17 and Ch.18 — full visual-LiDAR fusion, algorithms for solid-state sensors, dynamic object handling, event camera maturity, the second life of semantic maps — do not stay in their own chapters. Ch.19 collects them alongside the unresolved items from every earlier chapter and puts them in the same room.
