# Chapter 0. Is SLAM Solved?

Pick up a smartphone in 2026 and an augmented-reality layer locks onto a wall as if painted there. Indoor delivery robots tell the kitchen from the conference room without ever being handed a floor plan. Feed a handful of photographs into a [DUSt3R](https://arxiv.org/abs/2312.14132)-family model, and a 3D scene emerges in seconds. Because these capabilities have disappeared into everyday products, it is tempting to assume that SLAM is essentially a solved problem.

---

Step back to 2003, and the picture looks very different. In a laboratory at Imperial College London, Andrew Davison demonstrated real-time 3D tracking using nothing more than a single laptop and a webcam. His system, [MonoSLAM](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf), ran at 30 Hz on a desktop computer, tracking roughly ten features per frame and maintaining a sparse map of a few dozen landmarks. Its entire world was a single desk in a single room; the moment the camera wandered past the desk's edge, the map could drift and diverge. That modest scale defined the frontier of real-time monocular SLAM at the time.

Those early systems tracked a tiny fraction of the features that a modern phone AR session handles effortlessly. How did the fragile tracking demos of 2003 grow into the spatial-computing infrastructure of 2026?

---

The answer does not follow a single straight line of progress. Instead, SLAM's history traces four distinct research traditions that matured independently before colliding and absorbing one another. Photogrammetrists were already solving bundle adjustment by hand more than a century ago. Roboticists began casting maps in the language of probability with [Smith and Cheeseman](https://arxiv.org/abs/1304.3111)'s 1986 framework for uncertain spatial relationships—a problem setting that acquired the name "SLAM" nine years later in [Durrant-Whyte and Leonard's 1995 survey](https://ieeexplore.ieee.org/document/476131). Meanwhile, computer vision researchers focused on finding and tracking visual features in real time, and in the 2020s the deep learning community began folding all of these geometric pieces into unified neural architectures.

Looking back across those converging paths raises real historical questions. Was the transition from EKF-based SLAM to graph-based SLAM an inevitable technical step, or a shift shaped by a handful of researchers' design bets? Did anyone foresee the divide between feature-based and direct methods from the beginning? And why has deep learning taken so long to displace the classical geometry pipeline? Counterfactual questions only make sense when genuine alternatives existed—and in SLAM, they did.

---

For a rigorous, textbook treatment of the underlying equations, theorems, and proofs, readers can turn to the [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release). Edited by Carlone, Kim, Barfoot, Cremers, and Dellaert and published by Cambridge University Press in 2026, its 18 chapters survey the current theory and practice of the field.

Fittingly, the five editors close the Handbook with a pointed warning: *"If someone tells you 'SLAM is solved,' don't listen to them."* Far from being the field's consensus, the notion that SLAM is finished usually comes from mistaking mature commercial demos for solved science.

---

When Davison stood in front of his webcam in 2003, he could hardly have known everything that demo would set in motion. You can still find the video online: the jittery frame, the blinking landmark dots, and a sparse map holding only a few dozen points. The path connecting that laboratory room to today's spatial systems is the story of this book.

Yet our story starts long before MonoSLAM. Decades before the acronym "SLAM" took hold in the 1990s—and even before Smith and Cheeseman wrote a probabilistic map down in equations—photogrammetrists were already reconstructing 3D worlds from photographs. The next chapter traces that prehistory.
