# Ch.0 — SLAM Solved?

In 2026, you pick up a phone and an AR layer sticks to the wall. Indoor delivery robots tell the kitchen from the conference room without being handed a map. Give a few photos to a [DUSt3R](https://arxiv.org/abs/2312.14132)-family model and a 3D structure emerges in seconds. By now, these are products rather than demos, part of the background. SLAM often looks like a more-or-less solved problem.

---

Go back to 2003 and the scene is different. Andrew Davison, in a lab at Imperial College London, demonstrated real-time 3D tracking with one laptop and one webcam. The system, called [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf), ran at 30 Hz on a desktop, tracked about ten features per frame, and maintained a sparse map of a few dozen landmarks. It covered one desk in one room; when the camera left the desk, the map diverged. That scale and limitation were representative of real-time monocular SLAM at the time.

Those systems tracked far fewer features per frame than a phone AR session does today. How did the tracking systems of 2003 develop into the AR systems of 2026?

---

SLAM's history is not a single development curve. It traces four traditions that ran independently before colliding and absorbing one another. Photogrammetrists solved bundle adjustment by hand a century ago. Roboticists began treating maps in the language of probability with [Smith-Cheeseman](https://arxiv.org/abs/1304.3111)'s 1986 stochastic spatial-relations framework, and the name "SLAM" was attached to this problem setting nine years later, in [Durrant-Whyte & Leonard's 1995 survey](https://ieeexplore.ieee.org/document/476131). Computer vision researchers focused on real-time feature tracking. The deep learning community of the 2020s is trying to absorb all of it into a single network.

The book asks not "how" but "why this way." Was the replacement of EKF-based SLAM by graph-based SLAM a natural technical evolution, or a contingency decided by a few people? Was the split between feature-based and direct methods foreseen from the start? Why has deep learning been so slow to replace the geometry pipeline? Counterfactuals matter only when the alternatives actually existed. Here, they did.

---

Tracing that path needs tools. A list of years gives a chronicle; an explanation of techniques gives a textbook. This history uses two lenses: lineage and prediction. Where did an idea come from? How did the future researchers expected differ from what actually unfolded?

Four devices recur throughout the book and guide each chapter.

**Lineage openings** sit in the first paragraph or two of a chapter. They show, through names and years, which intellectual inheritance the chapter's protagonist took on. No idea in SLAM was born in a vacuum. Follow the lineage and the terrain of borrowing becomes visible.

**🔗 Borrowed boxes** are margin annotations that state in one or two sentences where a specific technique came from: "ORB-SLAM's structure here came from Strasdat 2011." Researchers cite their sources, but often leave the lineage implicit. The box makes it explicit.

**📜 Prediction vs. outcome boxes** contrast what the original paper's Conclusion, Future Work, or Summary section anticipated with what actually happened. In §12, "Summary and Recommendations," the [Triggs 1999](https://dblp.org/rec/conf/dagstuhl/TriggsMHF99.html) bundle adjustment (BA) synthesis made the exploitation of large-scale sparse structure a central recommendation. In the 2010s, [COLMAP](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) approached that problem from another direction, turning SfM over tens of thousands of images into an open-source production tool. The direction was right; the route was not. This device examines the gap between the future a researcher expected and the one that arrived.

**🧭 Still open** sits at the end of the chapter. It lists questions on that chapter's subject that remain unresolved as of 2026, drawing out the open problems hidden by the perception that SLAM is solved. Ch.19 gathers these items from every chapter and reassembles them by theme.

---

The book runs in six parts.

**Part 1: Prehistory** traces the tools that photogrammetry and classical computer vision built up before SLAM was born in robotics. Why is bundle adjustment still the skeleton of every optimization backend?

**Part 2: Classical SLAM** follows probabilistic mapping and the limits of the EKF through MonoSLAM, PTAM, graph-based SLAM, and optimality certification. How did joint mapping and estimation expand from filtering to optimization?

**Part 3: Maturity** covers ORB-SLAM, inertial preintegration, continuous-time estimation, direct methods, RGB-D, and place recognition. It traces how systems expanded beyond early real-time demonstrations to larger environments and more varied sensors.

**Part 4: Learning Fusion** covers monocular depth estimation, end-to-end SLAM, and hybrid methods that combine geometry with learning. Which computations did learning take over, and where did geometric constraints remain?

**Part 5: Representation** covers Neural Radiance Fields, 3D Gaussian Splatting, dynamic-scene representations, and 3D foundation models. Changes in how maps and scenes are represented also changed the relationship between tracking and reconstruction.

**Part 6: Dead Ends and Open Problems** pulls out the failed routes in SLAM's history and the structural unresolved problems still sitting behind today's perception that it is "solved."

---

A useful map needs boundaries. The relationship between foundation models and SLAM is treated as an open question arising from current research, without predicting a settled outcome. The book's material is what happened in the past and why. Nor does it set out to declare earlier research wrong. It asks what a choice meant under the constraints of its moment. Homogeneous coordinates, epipolar geometry, and EKF formulas are assumed knowledge. The task here is to trace lineage rather than explain concepts; choosing a camera or LiDAR belongs to another book.

For a systematic account of the equations, theorems, and proofs, see the [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release). Edited by Carlone, Kim, Barfoot, Cremers, and Dellaert and published by Cambridge University Press in 2026, its 18 chapters cover the current theory and systems of SLAM. The present history records the path to that point.

The five editors close the Handbook with the line *"If someone tells you 'SLAM is solved,' don't listen to them."* The tendency to treat SLAM as solved, noted at the opening of this chapter, is a phenomenon within the field rather than its consensus.

---

When Davison stood in front of his webcam in 2003, he did not know exactly what he was starting. That demo video is still on the internet: the shaky frame, the blinking landmark dots, and a sparse map containing only dozens of points. The history between that room and today's systems is the subject of this book.

The record starts well before MonoSLAM. Before the acronym "SLAM" settled in the 1990s, and even before Smith-Cheeseman expressed a probabilistic map in equations, photogrammetrists were already recovering 3D structure from cameras. The next chapter traces that prehistory.
