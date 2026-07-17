# Ch.0 — SLAM Solved?

In 2026, you pick up a phone and an AR layer sticks to the wall. Indoor delivery robots tell the kitchen from the conference room without being handed a map. Give a few photos to a [DUSt3R](https://arxiv.org/abs/2312.14132)-family model and a 3D structure emerges in seconds. By now, these are products rather than demos, part of the background. SLAM often looks like a more-or-less solved problem.

---

Go back to 2003 and the scene is different. Andrew Davison, in a lab at Imperial College London, demonstrated real-time 3D tracking with one laptop and one webcam. The system, called [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf), ran at 30 Hz on a desktop, tracked about ten features per frame, and maintained a sparse map of a few dozen landmarks. It covered one desk in one room; when the camera left the desk, the map diverged. That scale and limitation were representative of real-time monocular SLAM at the time.

Those systems tracked far fewer features per frame than a phone AR session does today. More striking than the difference between 2003 and 2026 is *the path* by which it narrowed.

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

**Part 2: The Birth of SLAM** follows the period in which robots began building their own maps, from Smith-Cheeseman's 1986 stochastic framework to Davison's MonoSLAM. The problem took shape in the late 1980s, and the acronym "SLAM" and a shared terminology settled through the Durrant-Whyte and Leonard line of work in the 1990s. How did EKF become the dominant paradigm, and why were its limits structural?

**Part 3: The Parallel Revolution** covers the decade from PTAM splitting mapping and camera tracking in 2007 through graph-based SLAM and loop closure, up to ORB-SLAM. This was the decade in which "real-time SLAM" became possible on a desktop.

**Part 4: Methodological Divergence** handles the split between feature-based and direct methods, the arrival of RGB-D, and the process by which place recognition broke off into its own subfield. How did different assumptions produce different ecosystems?

**Part 5: The Inflow of Learning** covers monocular depth estimation, end-to-end SLAM, Neural Radiance Fields, and 3D Gaussian Splatting. It follows how quickly deep learning absorbs the geometry pipeline and where that process meets resistance.

**Part 6: Dead Ends and Open Problems** pulls out the failed routes in SLAM's history and the structural unresolved problems still sitting behind today's perception that it is "solved."

---

A useful map needs boundaries. Questions such as whether foundation models will replace SLAM fall outside this book's scope; its material is what happened in the past and why. Nor does it set out to declare earlier research wrong. It asks what a choice meant under the constraints of its moment. Homogeneous coordinates, epipolar geometry, and EKF formulas are assumed knowledge. The task here is to trace lineage rather than explain concepts; choosing a camera or LiDAR belongs to another book.

For a systematic account of the equations, theorems, and proofs, see the [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release). Edited by Carlone, Kim, Barfoot, Cremers, and Dellaert and published by Cambridge University Press in 2026, its 18 chapters cover the current theory and systems of SLAM. This book records the path to that point.

The five editors close the Handbook with the line *"If someone tells you 'SLAM is solved,' don't listen to them."* The tendency to treat SLAM as solved, noted at the opening of this chapter, is a phenomenon within the field rather than its consensus.

---

When Davison stood in front of his webcam in 2003, he did not know exactly what he was starting. That demo video is still on the internet: the shaky frame, the blinking landmark dots, and a sparse map containing only dozens of points. The history between that room and today's systems is the subject of this book.

The record starts well before MonoSLAM. Before the acronym "SLAM" settled in the 1990s, and even before Smith-Cheeseman expressed a probabilistic map in equations, photogrammetrists were already recovering 3D structure from cameras. The next chapter traces that prehistory.
