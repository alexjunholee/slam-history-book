# Ch.0 — SLAM Solved?

In 2026, you pick up a phone and an AR layer sticks to the wall. Indoor delivery robots tell the kitchen from the conference room without being handed a map. Throw a few photos at a [DUSt3R](https://arxiv.org/abs/2312.14132)-family model and a 3D structure comes out in seconds. These are less demos than products by now, and largely background. So there is a mood that treats SLAM as a more-or-less solved problem.

---

Go back to 2003 and the scene is different. Andrew Davison, in a lab at Imperial College London, demonstrated real-time 3D tracking with one laptop and one webcam. The system, called [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf), ran at 30Hz on a desktop, watched about ten features per frame, and held a sparse map on the order of a few dozen landmarks. One desk in one room; when the camera left the desk, the map diverged. That was the state of the art.

The peak then was a few hundredths of the feature count a phone AR session tracks at any given moment today, and it took 23 years to close that gap. More striking than the gap itself is *which path* it was closed along.

---

SLAM's history is not a single development curve. It is the trace of four separate traditions running independently, then colliding and absorbing each other. Photogrammetrists solved bundle adjustment by hand a century ago. Roboticists began treating maps in the language of probability with [Smith-Cheeseman](https://arxiv.org/abs/1304.3111)'s 1986 stochastic spatial-relations framework, and the name "SLAM" was attached to this problem setting nine years later, in [Durrant-Whyte & Leonard's 1995 survey](https://ieeexplore.ieee.org/document/476131). Computer vision researchers were fixated on real-time feature tracking. And the 2020s deep learning community is trying to absorb all of it into a single network.

The question this book puts is not "how" but "why this way." Was the replacement of EKF-based SLAM by graph-based SLAM a natural technical evolution, or a contingency that a few people's choices decided? Was the split between feature-based and direct methods foreseen from the start? Why has deep learning been so slow to replace the geometry pipeline? Counterfactuals are worth asking only when the alternatives actually existed. This book shows that they did.

---

Tracing that path needs tools. Listing years gives you a chronicle; explaining techniques gives you a textbook. This book reads the history through two lenses: lineage and prediction. Where did an idea come from? And how did the future researchers saw from their vantage point diverge from the future that actually unfolded?

There are four repeating devices in this book. They can serve as guides as you read each chapter.

**Lineage openings** sit in the first paragraph or two of a chapter. They show, through names and years, which intellectual inheritance the chapter's protagonist took on. No idea in SLAM was born in a vacuum. Follow the lineage and the terrain of borrowing becomes visible.

**🔗 Borrowed boxes** are margin annotations that state in one or two sentences where a specific technique came from. Something like "ORB-SLAM's structure here came from Strasdat 2011." Researchers cite, but they often do not make the lineage explicit. The box does.

**📜 Prediction vs. outcome boxes** contrast what the original paper's Conclusion, Future Work, or Summary section pointed to against what actually happened. The place where [Triggs 1999](https://dblp.org/rec/conf/dagstuhl/TriggsMHF99.html)'s bundle adjustment (BA) synthesis paper, in §12 "Summary and Recommendations," left exploitation of large-scale sparse structure as its core guidance was filled, in the 2010s, from another angle: [COLMAP](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) turned tens-of-thousands-of-image SfM into an open-source production tool. The direction of the prediction was right, the route was not. The gap between the future a researcher saw from their moment and the future that arrived is what this device is about.

**🧭 Still open** sits at the end of the chapter. These are items on the chapter's subject that, as of 2026, remain unresolved. They pull out the open problems hiding inside the perception that SLAM is solved. Ch.19 harvests these items across all chapters and reassembles them by theme.

---

The book runs in six parts.

**Part 1: Prehistory** traces the tools that photogrammetry and classical computer vision built up before SLAM was born in robotics. Why is bundle adjustment still the skeleton of every optimization backend.

**Part 2: The Birth of SLAM** follows the period in which robots first started building their own maps, from Smith-Cheeseman's 1986 stochastic framework to Davison's MonoSLAM. The problem setting was fixed in 1986; the acronym "SLAM" and the standard terminology settled in the community with Durrant-Whyte and Leonard's 1995 survey. How EKF became the dominant paradigm as a tool, and why its limits were structural.

**Part 3: The Parallel Revolution** covers the decade from PTAM splitting mapping and camera tracking in 2007 through graph-based SLAM and loop closure, up to ORB-SLAM. The ten years in which "real-time SLAM" became possible on a desktop.

**Part 4: Methodological Divergence** handles the split between feature-based and direct methods, the arrival of RGB-D, and the process by which place recognition broke off into its own subfield. How different assumptions produced different ecosystems.

**Part 5: The Inflow of Learning** covers monocular depth estimation, end-to-end SLAM, Neural Radiance Fields, and 3D Gaussian Splatting. The speed at which deep learning absorbs the geometry pipeline, and the sources of friction along the way.

**Part 6: Dead Ends and Open Problems** pulls out the failed routes in SLAM's history and the structural unresolved problems still sitting behind today's perception that it is "solved."

---

Setting the scope is what turns the book into a map. Questions like whether foundation models will replace SLAM are not this book's concern. What happened in the past and why is the material. Arguing that some research was wrong is not the goal either. The closer goal is to show what a given choice meant under the constraints of its moment. Homogeneous coordinates, epipolar geometry, and EKF formulas are assumed known. This book's job is tracing lineage, not explaining concepts, and which camera or LiDAR to pick is a different book's subject.

If you want the equations, the theorems, and the proofs laid out systematically, there is the [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release). Edited by Carlone, Kim, Barfoot, Cremers, and Dellaert, published by Cambridge University Press in 2026, it covers the current theory and systems of SLAM in 18 chapters. This book records the path that led to that state.

One of the lines the five editors left jointly in that Handbook's Epilogue is *"If someone tells you 'SLAM is solved,' don't listen to them."* The "mood that treats it as solved," mentioned at the opening of this chapter, is an observed phenomenon inside the field, not a consensus of the field.

---

When Davison stood in front of his webcam in 2003, he did not know exactly what he was starting. That demo video is still on the internet. The shaky frame, the blinking landmark dots, the sparse map on the order of dozens of points. This book records what happened between there and here.

The record starts well before MonoSLAM. Before the acronym "SLAM" settled in the 1995 survey, even before Smith-Cheeseman wrote a probabilistic map down as equations, photogrammetrists were already recovering 3D structure from cameras. The next chapter traces that prehistory.
