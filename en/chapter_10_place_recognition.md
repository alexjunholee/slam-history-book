# Ch.10 — The Parallel Line of Place Recognition: From FAB-MAP to NetVLAD, and on to AnyLoc

Around 2003, while Davison was proving real-time 3D tracking with a single webcam, Mark Cummins and Paul Newman at the Oxford Mobile Robotics Group were holding onto a different question. "How does a robot recognize a place it has visited before?" As long as visual odometry (VO) suffered from accumulated drift, no SLAM system could close the loop without an answer to that question. Place recognition developed through the 2000s in parallel with the other components of Visual SLAM, but along a lineage of its own. FAB-MAP transplanted Josef Sivic's bag-of-words (BoW) idea into robot space, DBoW2 made it practical, and NetVLAD broke through with learning. In 2023 AnyLoc pulled in features from a foundation model as-is.

While Ch.7 closed out the feature-based lineage with the ORB-SLAM trilogy, Ch.8 traced the direct lineage through DSO, and Ch.9 followed the arc of dense mapping from KinectFusion to BundleFusion, place recognition drew a line unlike any of them. It was neither tracking nor mapping, and derived from neither — an independent problem. Even so, all three lineages were incomplete without loop closure, and the "where have we seen this" judgment behind loop closure was supplied by place recognition.

---

## 10.1 Place recognition before BoW

To close a loop without GPS, whether indoors, in a tunnel, or in an urban canyon, a robot has to find, among thousands of candidate images, the one most similar to the current observation, and do so quickly. Pixel-level comparison is a linear scan, O(N), and once the image count passes tens of thousands, real time is out of reach.

In early-2000s computer vision, the first to touch this problem were Sivic and Zisserman. Published at ICCV 2003, ["Video Google"](https://www.robots.ox.ac.uk/~vgg/publications/2003/Sivic03/sivic03.pdf) applied TF-IDF from document retrieval to images. SIFT descriptors were clustered with k-means to form "visual words," and each image was represented as a frequency vector over those words. Retrieval, through an inverted index, became close to O(1). Place recognition researchers picked up the idea right away.

---

## 10.2 FAB-MAP — probabilistic BoW and the Chow-Liu tree (2008)

Mark Cummins and Paul Newman, at the Oxford Mobile Robotics Group, published [Cummins & Newman. FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance](https://doi.org/10.1177/0278364908090961) in 2008.

The core question behind FAB-MAP (**Fast Appearance-Based Mapping**) is: "Is this scene a place already in the database, or somewhere entirely new?" A plain similarity score cannot settle that. If dozens of corridors all look alike, the highest similarity score does not guarantee the right answer.

Cummins and Newman framed this as a Bayesian inference problem. Given an observation $z_t$ (the set of visual-word occurrences), they computed the probability that the current location is each database place $\ell_i$:

$$P(\ell_i \mid z_t) \propto P(z_t \mid \ell_i) P(\ell_i)$$

The hard part is $P(z_t \mid \ell_i)$. Assuming visual words are independent gives a naïve Bayes model, but in practice visual words are correlated. If the word "door" appears, the word "doorknob" tends to appear along with it. The independence assumption distorts the probability.

FAB-MAP modeled this correlation with a **Chow-Liu tree**. A Chow-Liu tree is a tree-structured graphical model that maximizes pairwise mutual information among words. The mutual information between two words $e_i, e_j$ is defined as

$$I(e_i; e_j) = \sum_{e_i, e_j} P(e_i, e_j) \log \frac{P(e_i, e_j)}{P(e_i)P(e_j)}$$

and the Chow-Liu algorithm uses this as an edge weight to build a maximum spanning tree. Factorizing the joint likelihood through this tree gives

$$P(z_t \mid \ell_i) = \prod_k P(z_t^k \mid z_t^{\text{pa}(k)}, \ell_i)$$

where $z_t^k \in \{0,1\}$ is the occurrence of the $k$-th word and $\text{pa}(k)$ is its parent in the tree. Compared with naïve Bayes (independence), this reflects co-occurrence patterns across words, and so lowers false positives in places that look visually alike, such as corridors. During training, the vocabulary and the tree are learned together from a large image set.

FAB-MAP also handled, explicitly, the possibility that the current location is a new place not in the database. Adding the "new place" hypothesis cut false positives. In loop closure, a false positive leads to catastrophic failure. That was the practical heart of the matter.

> 🔗 **Borrowed.** FAB-MAP's visual-word approach was transplanted directly from Sivic & Zisserman's "Video Google" (2003). The inverted-index logic of document retrieval was applied to a robot's memory of places.

In 2011 Cummins and Newman published [FAB-MAP 2.0](https://www.robots.ox.ac.uk/~mjc/Papers/cummins_newman_ijrr_fabmap2_2010_preprint.pdf). The goal was to push the processable map scale to around 1,000 km. They showed experimentally that it ran on a city-scale dataset.

---

## 10.3 DBoW2 — binary descriptors and the vocabulary tree (2012)

FAB-MAP was built on floating-point descriptors like SIFT. Around 2012 the SLAM community was moving toward faster binary descriptors, in particular BRIEF, ORB, and BRISK. Keeping a SIFT vocabulary as-is was a compute-cost problem.

In 2012 Dorian Gálvez-López and Juan D. Tardós (Universidad de Zaragoza) published [Gálvez-López & Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences](https://doi.org/10.1109/TRO.2012.2197158). **DBoW2** is a vocabulary tree that uses binary descriptors, with Hamming-distance comparisons that made word assignment tens of times faster than SIFT.

DBoW2's structure is a vocabulary tree built by hierarchical k-means. The BoW vector representing an image is a TF-IDF–weighted binary-word frequency vector. Each leaf node $w_i$ of a tree with branching factor $k$ and depth $d$ carries the TF-IDF weight

$$\eta_i = \frac{n_i}{n} \cdot \log \frac{N}{N_i}$$

where $n_i$ is the word count of $w_i$ in the image, $n$ is the total word count, $N$ is the number of database images, and $N_i$ is the number of images containing $w_i$. The similarity between two images $a$, $b$ is given by the L1-norm

$$s(\mathbf{v}_a, \mathbf{v}_b) = 1 - \frac{1}{2} \left\| \frac{\mathbf{v}_a}{|\mathbf{v}_a|} - \frac{\mathbf{v}_b}{|\mathbf{v}_b|} \right\|_1$$

Lookup runs in O(log N) through an inverted index.

> 🔗 **Borrowed.** DBoW2's vocabulary-tree concept traces its lineage to Nistér & Stewénius's 2006 ["Scalable Recognition with a Vocabulary Tree"](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/nister_stewenius_cvpr2006.pdf) (CVPR). DBoW2 transplanted that structure into the binary-descriptor world and tuned the weighting scheme for SLAM.

What mattered about DBoW2 was deployment more than algorithm. Released as open source, the library was adopted as the loop-closure module of ORB-SLAM (2015), and ORB-SLAM2 and ORB-SLAM3 used the same DBoW2. From 2015 through the mid-2020s, place recognition in the SLAM community was effectively DBoW2's job.

The Gálvez-López–Tardós partnership is also worth noting. Tardós was the figure who, together with Mur-Artal and Campos, later led the ORB-SLAM trilogy. DBoW2 was, in effect, the place-recognition layer prepared in advance for that project.

---

## 10.4 NetVLAD — CNN-based VPR (2016)

The BoW family had one fundamental limit. The vocabulary was trained against a specific descriptor and a specific environment. When lighting changed, or the season shifted, or the viewpoint moved far enough, the distribution of visual words shifted too, and a pre-trained vocabulary broke.

At CVPR 2016, Relja Arandjelović, Petr Gronat, Akihiko Torii, Tomáš Pajdla, and Josef Sivic published [NetVLAD: CNN Architecture for Weakly Supervised Place Recognition](https://doi.org/10.1109/CVPR.2016.572). Sivic, among the authors, is the same Sivic behind "Video Google" in 2003. The person who had introduced BoW to image retrieval at ICCV 2003 was a co-author, thirteen years later, on the paper that pushed past that approach's limits.

NetVLAD's idea was to make **VLAD (Vector of Locally Aggregated Descriptors)** aggregation differentiable.

VLAD is an aggregation scheme proposed in 2010 by [Jégou et al.](https://inria.hal.science/inria-00548637/file/jegou_compactimagerepresentation.pdf), which represents an entire image by accumulating how much each local descriptor contributes, as a "residual," to its nearest cluster center (visual word). The VLAD sub-vector for cluster center $k$ is

$$\mathbf{V}(k) = \sum_{\mathbf{x}_i : \text{NN}(\mathbf{x}_i)=k} (\mathbf{x}_i - \boldsymbol{\mu}_k)$$

and the full VLAD vector $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$ is the concatenation across all clusters, L2-normalized. With $K$ clusters and $D$-dimensional descriptors, the final vector is $KD$-dimensional. The VLAD vector carries much richer information than BoW's binary assignment.

> 🔗 **Borrowed.** NetVLAD's aggregation design inherits directly from VLAD in Jégou et al.'s "Aggregating Local Descriptors into a Compact Image Representation" (CVPR 2010). What NetVLAD did was swap VLAD's hard assignment for a soft assignment and make the whole pipeline trainable end-to-end.

The NetVLAD layer softens the nearest-neighbor assignment of classical VLAD into a softmax:

$$\bar{a}_k(\mathbf{x}_i) = \frac{e^{\mathbf{w}_k^\top \mathbf{x}_i + b_k}}{\sum_{k'} e^{\mathbf{w}_{k'}^\top \mathbf{x}_i + b_{k'}}}$$

Here $\mathbf{x}_i$ is a local feature extracted by the CNN, and $\mathbf{w}_k$ and $b_k$ are learnable parameters. Accumulating the NetVLAD vector with this soft assignment gives

$$\mathbf{V}(k) = \sum_i \bar{a}_k(\mathbf{x}_i)\,(\mathbf{x}_i - \boldsymbol{\mu}_k)$$

and the full vector $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$, after intra-normalization (L2 on each sub-vector) and a final L2-normalization, becomes the VPR descriptor. Unlike hard-assignment VLAD, the gradient back-propagates, so the CNN backbone can be trained end-to-end with it.

Training was also different. The authors used Google Street View Time Machine data, taking image pairs of the same place at different times as positive examples and images of different places as negatives, under a weakly supervised triplet loss. With just GPS positions, they could train without labels.

On the Pittsburgh 250k and Tokyo 24/7 benchmarks, NetVLAD was well ahead of the DBoW family and of earlier VLAD-based methods. It was much more robust across changes in lighting and season and had some tolerance to viewpoint differences. Still, NetVLAD was not immediately integrated into practical SLAM pipelines. Its inference speed and memory footprint were heavier than DBoW2, and the ORB-SLAM ecosystem had already been built around DBoW2.

---

## 10.5 Patch-NetVLAD, MixVPR, AnyLoc (2020–2023)

After NetVLAD, Visual Place Recognition (VPR) research scattered into improvements on generalization.

In 2021 Hausler et al. put out [Patch-NetVLAD](https://arxiv.org/abs/2103.01486). Instead of judging a place from a single global descriptor, as NetVLAD did, the image is split into patches and the NetVLAD representation of each patch is combined spatially. On Tokyo 24/7, it raised Recall@1 by about 10 percentage points over NetVLAD. Patch-level processing raised inference cost along with it.

In 2023 Ali-bey et al.'s [MixVPR](https://arxiv.org/abs/2303.02190) produced global features through Transformer-style feature mixing. The target was balance between being lightweight and performance. VPR papers from this period commonly took Mapillary Street Level Sequences (MSLS) and seasonal-change datasets such as Nordland as benchmarks. Extreme lighting and seasonal conditions emerged as a common barrier.

In 2023 Keetha et al.'s [AnyLoc: Towards Universal Visual Place Recognition](https://arxiv.org/abs/2308.00688) took a different route. Use DINOv2-based self-supervised features for place recognition, as-is, with no fine-tuning.

> 🔗 **Borrowed.** AnyLoc's feature extraction pulls pre-trained ViT representations from Oquab et al.'s [DINOv2](https://arxiv.org/abs/2304.07193) (Meta AI, 2023). AnyLoc layered VLAD aggregation on top. The BoW–VLAD lineage that began with FAB-MAP rejoins the story in the foundation-model era.

DINOv2 is a Vision Transformer (ViT) trained on large-scale internet images. It produces general-purpose features that are not biased to a particular city, a particular season, or a particular camera. What AnyLoc drew attention to is DINOv2's **facet** concept. Each attention head in a ViT outputs a query (Q), key (K), value (V) matrix and a final token (patch feature). Keetha et al. confirmed experimentally that, among these four kinds of facet, the value (V) facet gives the most semantically stable representation for place recognition. Q and K facets lean toward structural and geometric information, while the V facet concentrates more on semantics, which is useful for a consistent place representation across season and lighting. Keetha et al. showed that linking this V-facet representation to VLAD aggregation yields a single model that works across very diverse environments worldwide — indoor and outdoor, underground, aerial views. Across seven or more environments (Pittsburgh, Tokyo, indoor factories, underground parking garages, libraries), a single model was competitive with, or ahead of, earlier specialized methods.

Once generality came loose along one axis, the next branch moved toward crossing modality boundaries. Lee et al.'s [(LC)²](https://arxiv.org/abs/2304.08660) (RA-L 2023) projected camera imagery and LiDAR point clouds into a shared 2.5D depth image, attempting cross-modal retrieval in which a 2D query looks up places in a LiDAR map. The follow-up LC²++ took an LoRA-adapted global retrieval and chained MINIMA-based local matching and PnP onto it, connecting place identification all the way to 6-DoF pose recovery in a single pipeline. Such cross-modal evaluation became possible only because of datasets like Lee et al.'s [ViViD++](https://arxiv.org/abs/2204.06183) (RA-L 2022), which synchronize visible, thermal, event, LiDAR, inertial, and depth streams across indoor, outdoor, and underground settings.

---

## 10.6 Toward integrating place recognition and metric localization (2024–2025)

Place recognition research has run in parallel with the rest of SLAM's components since the early 2000s. ORB-SLAM embedded DBoW2, but the place-recognition module was a black box isolated from mapping and tracking. The input was an image, the output a loop-candidate ID.

Moving into 2024–2025, this boundary began to blur. Berton et al.'s [EigenPlaces](https://arxiv.org/abs/2308.10832) (2023) and Izquierdo & Civera's [SALAD](https://arxiv.org/abs/2311.15937) (2023 arXiv / CVPR 2024) explored pulling place-recognition descriptors directly into metric localization. Not stopping at "where have we seen this place," they tried to read 6-DoF pose out of the place-recognition representation itself.

Around 2024 there were also attempts to combine Gaussian-map representations with place recognition. It was a direction aligned with the rise of 3DGS (3D Gaussian Splatting) as a map representation.

> 📜 **Prediction vs. outcome.** In their 2011 FAB-MAP 2.0 paper, Cummins and Newman pushed the scale limit of place recognition, demonstrating appearance-only loop closure on a 1,000 km trajectory. Measured against the early FAB-MAP experiments that had run the Oxford campus and parts of the city, that was a two-digit-factor leap. Later city-scale experiments using DBoW2 and large vocabularies reproduced the same scale in practical SLAM. The scale problem was solved this way, but the failure mode Cummins and Newman left behind, vocabulary-based representations vulnerable to seasonal and lighting change, was crossed using a different tool that deep learning brought. `[diverted]`

> 📜 **Prediction vs. outcome.** In the introduction of the 2016 NetVLAD paper, Arandjelović et al. named three challenges for solving place recognition — a CNN architecture, enough training data, and an end-to-end training procedure — and laid out their contribution to each. The architecture and training procedure were answered directly by NetVLAD, but in the seven years that followed, a string of VPR papers targeted generalization across appearance conditions (season, lighting, viewpoint). In 2023, AnyLoc showed the possibility of a single multi-environment model with fine-tuning-free foundation-model features. Less a complete solution than the axis shifting from specialized models toward general-purpose ones. `[in progress]`

---

## 10.7 🧭 Still open

**Extreme season and lighting change.** The Nordland (Norwegian railway, summer vs. winter) and Oxford RobotCar (a year of seasonal change) datasets have reported the same barrier for over a decade. DINOv2-based methods have narrowed the gap, but no single model yet recognizes the same place at 99% accuracy between snow-covered winter and leaf-heavy summer. Place recognition in environments with heavy appearance change remains an open problem as of 2026.

**Integration of place recognition and metric localization.** In most SLAM pipelines today, place recognition answers only "where have we seen this," and actual pose estimation is handled by a separate PnP or descriptor-matching stage. Attempts to merge the two processes into a single representation appeared between 2023 and 2025, but no method has yet reached deployment-level precision and speed at the same time.

*Privacy of recognizable place representations.* The place representations a VPR system stores can be used, through reconstruction attacks, to recover the original images or the 3D structure. For commercial robots mapping the interiors of homes, hospitals, and offices, this becomes a real concern. A place-representation scheme that guarantees privacy without sacrificing performance does not yet exist.

---

The three lineages of Part 3 (the mature period) close this way. While ORB-SLAM standardized the feature-based pipeline, DSO completed the photometric theory, and the KinectFusion line laid out the possibilities and limits of dense mapping, place recognition sat somewhere different from all of them. It did not grow inside SLAM — it grew out of the image-retrieval problem in computer vision, and when SLAM needed loop closure, it took the supplier's seat. That distance turned into an advantage. When the deep-learning wave hit, place recognition absorbed the new tools faster than the rest of the SLAM pipeline did.

When AnyLoc appeared in 2023, Sivic's name was in the references, not the acknowledgements. The person who plugged BoW into image retrieval in 2003, and who was co-author on NetVLAD in 2016 when that approach was pushed past its limits. At the end of that lineage, AnyLoc pushed the door Sivic had opened over toward the foundation-model side.

Part 4 begins with a different kind of intrusion — not a place-recognition module borrowed from vision, but the geometry-first assumption at the core of every lineage in Part 3. The next chapter follows the crack from an unexpected direction: a graduate student at NYU, a depth-estimation CNN, and the slow unraveling of the premise that geometry must come from geometry.
