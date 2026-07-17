# Ch.10 — The Parallel Line of Place Recognition: From FAB-MAP to NetVLAD, and on to AnyLoc

Around 2003, while Davison was demonstrating real-time 3D tracking with a single webcam, Mark Cummins and Paul Newman at the Oxford Mobile Robotics Group were asking a different question: "How does a robot recognize a place it has visited before?" Because visual odometry (VO) accumulated drift, no SLAM system could close a loop without answering it. Place recognition developed through the 2000s alongside other components of Visual SLAM but followed its own lineage. FAB-MAP adapted Josef Sivic's bag-of-words (BoW) approach to robotics, DBoW2 made it practical, and NetVLAD introduced learning. In 2023, AnyLoc used foundation-model features without fine-tuning.

The ORB-SLAM, DSO, and KinectFusion lineages developed different approaches to tracking and mapping, but each required loop closure. Place recognition supplied the decision behind it: where has the system seen this scene before?

---

## 10.1 Place recognition before BoW

To close a loop without GPS, whether indoors, in a tunnel, or in an urban canyon, a robot must quickly find among thousands of candidates the image most similar to its current observation. Pixel-level comparison requires a linear scan, O(N), and becomes impractical in real time once the collection reaches tens of thousands of images.

Sivic and Zisserman were among the first computer vision researchers to address this problem in the early 2000s. Their ICCV 2003 paper ["Video Google"](https://www.robots.ox.ac.uk/~vgg/publications/2003/Sivic03/sivic03.pdf) applied TF-IDF from document retrieval to images. They clustered SIFT descriptors with k-means to form "visual words" and represented each image as a frequency vector over those words. An inverted index avoided scanning the full image collection by retrieving only the posting lists for visual words in the query, and place-recognition researchers quickly adopted the idea.

---

## 10.2 FAB-MAP — probabilistic BoW and the Chow-Liu tree (2008)

Mark Cummins and Paul Newman, at the Oxford Mobile Robotics Group, published [Cummins & Newman. FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance](https://doi.org/10.1177/0278364908090961) in 2008.

FAB-MAP (**Fast Appearance-Based Mapping**) asks whether a scene corresponds to a place already in the database or to an entirely new location. A simple similarity score cannot resolve this distinction: if dozens of corridors look alike, the highest score does not guarantee a correct match.

Cummins and Newman framed this as a Bayesian inference problem. Given an observation $z_t$ (the set of visual-word occurrences), they computed the probability that the current location is each database place $\ell_i$:

$$P(\ell_i \mid z_t) \propto P(z_t \mid \ell_i) P(\ell_i)$$

The hard part is $P(z_t \mid \ell_i)$. Assuming visual words are independent gives a naïve Bayes model, but in practice visual words are correlated. If the word "door" appears, the word "doorknob" tends to appear along with it. The independence assumption distorts the probability.

FAB-MAP modeled this correlation with a **Chow-Liu tree**. A Chow-Liu tree is a tree-structured graphical model that maximizes pairwise mutual information among words. The mutual information between two words $e_i, e_j$ is defined as

$$I(e_i; e_j) = \sum_{e_i, e_j} P(e_i, e_j) \log \frac{P(e_i, e_j)}{P(e_i)P(e_j)}$$

and the Chow-Liu algorithm uses this as an edge weight to build a maximum spanning tree. Factorizing the joint likelihood through this tree gives

$$P(z_t \mid \ell_i) = \prod_k P(z_t^k \mid z_t^{\text{pa}(k)}, \ell_i)$$

where $z_t^k \in \{0,1\}$ indicates the occurrence of the $k$-th word and $\text{pa}(k)$ is its parent in the tree. Unlike naïve Bayes, this factorization captures co-occurrence patterns across words and lowers false positives in visually similar places such as corridors. During training, the vocabulary and the tree are learned together from a large image set.

FAB-MAP also explicitly modeled the possibility that the current location was absent from the database. Adding this "new place" hypothesis reduced false positives, which can cause catastrophic failure in loop closure.

> 🔗 **Borrowed.** FAB-MAP's visual-word approach was transplanted directly from Sivic & Zisserman's "Video Google" (2003). The inverted-index logic of document retrieval was applied to a robot's memory of places.

In 2011, Cummins and Newman published [FAB-MAP 2.0](https://www.robots.ox.ac.uk/~mjc/Papers/cummins_newman_ijrr_fabmap2_2010_preprint.pdf). They sought to extend the processable map scale to around 1,000 km and demonstrated the system on a city-scale dataset.

---

## 10.3 DBoW2 — binary descriptors and the vocabulary tree (2012)

FAB-MAP used floating-point descriptors such as SIFT. Around 2012, the SLAM community was moving toward faster binary descriptors, particularly BRIEF, ORB, and BRISK. Retaining a SIFT vocabulary imposed a substantial computational cost.

In 2012, Dorian Gálvez-López and Juan D. Tardós of Universidad de Zaragoza published [Gálvez-López & Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences](https://doi.org/10.1109/TRO.2012.2197158). **DBoW2** uses a vocabulary tree of binary descriptors. Hamming-distance comparisons made word assignment tens of times faster than with SIFT.

DBoW2's structure is a vocabulary tree built by hierarchical k-means. The BoW vector representing an image is a TF-IDF–weighted binary-word frequency vector. Each leaf node $w_i$ of a tree with branching factor $k$ and depth $d$ carries the TF-IDF weight

$$\eta_i = \frac{n_i}{n} \cdot \log \frac{N}{N_i}$$

where $n_i$ is the word count of $w_i$ in the image, $n$ is the total word count, $N$ is the number of database images, and $N_i$ is the number of images containing $w_i$. The similarity between two images $a$, $b$ is given by the L1-norm

$$s(\mathbf{v}_a, \mathbf{v}_b) = 1 - \frac{1}{2} \left\| \frac{\mathbf{v}_a}{|\mathbf{v}_a|} - \frac{\mathbf{v}_b}{|\mathbf{v}_b|} \right\|_1$$

Lookup runs in O(log N) through an inverted index.

> 🔗 **Borrowed.** DBoW2's vocabulary-tree concept traces its lineage to Nistér & Stewénius's 2006 ["Scalable Recognition with a Vocabulary Tree"](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/nister_stewenius_cvpr2006.pdf) (CVPR). DBoW2 transplanted that structure into the binary-descriptor world and tuned the weighting scheme for SLAM.

DBoW2's influence came as much from deployment as from its algorithm. Released as an open-source library, it became the loop-closure module in ORB-SLAM (2015), ORB-SLAM2, and ORB-SLAM3. From 2015 through the mid-2020s, it served as the default place-recognition method across much of the SLAM community.

The Gálvez-López–Tardós collaboration also connected directly to later SLAM systems. Tardós subsequently led the ORB-SLAM trilogy with Mur-Artal and Campos, and DBoW2 supplied its place-recognition layer.

---

## 10.4 NetVLAD — CNN-based VPR (2016)

The BoW family had a fundamental limitation: its vocabulary was trained for a specific descriptor and environment. Large changes in lighting, season, or viewpoint shifted the distribution of visual words and could make a pretrained vocabulary fail.

At CVPR 2016, Relja Arandjelović, Petr Gronat, Akihiko Torii, Tomáš Pajdla, and Josef Sivic published [NetVLAD: CNN Architecture for Weakly Supervised Place Recognition](https://doi.org/10.1109/CVPR.2016.572). Sivic had co-authored "Video Google" in 2003, introducing BoW to image retrieval. Thirteen years later, he co-authored a method designed to move beyond its limitations.

NetVLAD made **VLAD (Vector of Locally Aggregated Descriptors)** aggregation differentiable.

VLAD is an aggregation scheme proposed in 2010 by [Jégou et al.](https://inria.hal.science/inria-00548637/file/jegou_compactimagerepresentation.pdf). It represents an entire image by accumulating the residual between each local descriptor and its nearest cluster center, or visual word. The VLAD sub-vector for cluster center $k$ is

$$\mathbf{V}(k) = \sum_{\mathbf{x}_i : \text{NN}(\mathbf{x}_i)=k} (\mathbf{x}_i - \boldsymbol{\mu}_k)$$

and the full VLAD vector $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$ is the concatenation across all clusters, L2-normalized. With $K$ clusters and $D$-dimensional descriptors, the final vector is $KD$-dimensional. The VLAD vector carries much richer information than BoW's binary assignment.

> 🔗 **Borrowed.** NetVLAD's aggregation design directly inherits VLAD from Jégou et al.'s "Aggregating Local Descriptors into a Compact Image Representation" (CVPR 2010). NetVLAD replaced VLAD's hard assignment with a soft assignment and made the full pipeline trainable end to end.

The NetVLAD layer softens the nearest-neighbor assignment of classical VLAD into a softmax:

$$\bar{a}_k(\mathbf{x}_i) = \frac{e^{\mathbf{w}_k^\top \mathbf{x}_i + b_k}}{\sum_{k'} e^{\mathbf{w}_{k'}^\top \mathbf{x}_i + b_{k'}}}$$

Here $\mathbf{x}_i$ is a local feature extracted by the CNN, and $\mathbf{w}_k$ and $b_k$ are learnable parameters. Accumulating the NetVLAD vector with this soft assignment gives

$$\mathbf{V}(k) = \sum_i \bar{a}_k(\mathbf{x}_i)\,(\mathbf{x}_i - \boldsymbol{\mu}_k)$$

After intra-normalization (L2 on each sub-vector) and a final L2 normalization, the full vector $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$ becomes the VPR descriptor. Unlike hard-assignment VLAD, the soft assignment permits gradients to propagate through the layer, so the CNN backbone can be trained end to end.

The training procedure also differed from earlier methods. Using Google Street View Time Machine data, the authors treated images of the same place at different times as positive pairs and images of different places as negatives under a weakly supervised triplet loss. GPS positions provided the supervision without manual labels.

On the Pittsburgh 250k and Tokyo 24/7 benchmarks, NetVLAD substantially outperformed the DBoW family and earlier VLAD-based methods. It was more robust to lighting and seasonal changes and tolerated some viewpoint differences. NetVLAD was not immediately integrated into practical SLAM pipelines, however, because its inference and memory costs exceeded those of DBoW2 and the ORB-SLAM ecosystem was already built around DBoW2.

---

## 10.5 Patch-NetVLAD, MixVPR, AnyLoc (2020–2023)

After NetVLAD, Visual Place Recognition (VPR) research increasingly focused on generalization.

In 2021, Hausler et al. introduced [Patch-NetVLAD](https://arxiv.org/abs/2103.01486). Rather than representing a place with a single global descriptor, it divides the image into patches and spatially combines their NetVLAD representations. On Tokyo 24/7, it improved Recall@1 by about 10 percentage points over NetVLAD, at the cost of more expensive inference.

In 2023, Ali-bey et al.'s [MixVPR](https://arxiv.org/abs/2303.02190) produced global features through Transformer-style feature mixing, seeking a balance between a lightweight design and high performance. VPR papers from this period commonly evaluated on Mapillary Street Level Sequences (MSLS) and seasonal-change datasets such as Nordland. Extreme lighting and seasonal changes remained common obstacles.

In 2023, Keetha et al.'s [AnyLoc: Towards Universal Visual Place Recognition](https://arxiv.org/abs/2308.00688) used self-supervised DINOv2 features for place recognition without fine-tuning.

> 🔗 **Borrowed.** AnyLoc uses pretrained ViT representations from Oquab et al.'s [DINOv2](https://arxiv.org/abs/2304.07193) (Meta AI, 2023) and applies VLAD aggregation to them. This connects the BoW–VLAD lineage that began with FAB-MAP to foundation-model features.

DINOv2 is a Vision Transformer (ViT) trained on large-scale internet imagery. It produces general-purpose features that are not tied to a particular city, season, or camera. AnyLoc emphasized DINOv2's **facets**. Each ViT attention head produces query (Q), key (K), and value (V) matrices as well as final patch features. Keetha et al. found experimentally that the value (V) facet provided the most semantically stable representation for place recognition. Q and K facets favored structural and geometric information, whereas the V facet emphasized semantics, supporting consistent place representations across seasonal and lighting changes. By applying VLAD aggregation to the V-facet representation, they produced a single model that operated across diverse indoor, outdoor, underground, and aerial environments. Across seven or more settings, including Pittsburgh, Tokyo, indoor factories, underground parking garages, and libraries, the model matched or surpassed earlier specialized methods.

Research then extended generalization across sensor modalities. Lee et al.'s [(LC)²](https://arxiv.org/abs/2304.08660) (RA-L 2023) projected camera imagery and LiDAR point clouds into a shared 2.5D depth image, enabling a 2D query to retrieve places from a LiDAR map. Datasets such as Lee et al.'s [ViViD++](https://arxiv.org/abs/2204.06183) (RA-L 2022) enabled this cross-modal evaluation by synchronizing visible, thermal, event, LiDAR, inertial, and depth streams across indoor, outdoor, and underground settings.

---

## 10.6 Toward integrating place recognition and metric localization (2024–2025)

Place-recognition research has developed alongside other SLAM components since the early 2000s. ORB-SLAM embedded DBoW2, but kept place recognition as a black box separate from mapping and tracking: an image entered, and a loop-candidate ID emerged.

By 2024–2025, this boundary had begun to blur. Berton et al.'s [EigenPlaces](https://arxiv.org/abs/2308.10832) (2023) and Izquierdo & Civera's [SALAD](https://arxiv.org/abs/2311.15937) (2023 arXiv / CVPR 2024) explored using place-recognition descriptors directly for metric localization. They sought to estimate a 6-DoF pose from the place representation itself, rather than stopping after identifying a previously seen location.

Around 2024, researchers also began combining Gaussian-map representations with place recognition, following the rise of 3DGS (3D Gaussian Splatting) as a map representation.

> 📜 **Prediction vs. outcome.** In their 2011 FAB-MAP 2.0 paper, Cummins and Newman extended the scale of place recognition by demonstrating appearance-only loop closure on a 1,000 km trajectory. Compared with early FAB-MAP experiments on the Oxford campus and parts of the city, this was an increase by a factor in the tens. Later city-scale experiments with DBoW2 and large vocabularies reproduced the scale in practical SLAM. These methods addressed scale, but deep learning supplied a different solution to the remaining vulnerability of vocabulary-based representations to seasonal and lighting changes.

> 📜 **Prediction vs. outcome.** In the introduction to the 2016 NetVLAD paper, Arandjelović et al. identified three challenges for place recognition: a CNN architecture, sufficient training data, and an end-to-end training procedure. NetVLAD directly addressed the architecture and training procedure, while VPR research over the next seven years focused on generalization across season, lighting, and viewpoint. In 2023, AnyLoc demonstrated a single multi-environment model using foundation-model features without fine-tuning. This marked a shift from specialized models toward general-purpose ones rather than a complete solution.

---

## 10.7 🧭 Still open

**Extreme seasonal and lighting change.** The Nordland dataset (a Norwegian railway in summer and winter) and Oxford RobotCar dataset (a year of seasonal change) have exposed the same limitation for more than a decade. DINOv2-based methods have narrowed the gap, but one model still does not maintain consistent precision and recall across conditions such as snow-covered winter and dense summer foliage. Place recognition under severe appearance changes remains an open problem as of 2026.

**Integration of place recognition and metric localization.** In most current SLAM pipelines, place recognition only identifies a previously seen location; a separate PnP or descriptor-matching stage estimates the pose. Methods introduced between 2023 and 2025 attempted to merge both processes into one representation, but none has yet achieved deployment-level accuracy and speed simultaneously.

*Privacy of recognizable place representations.* Reconstruction attacks can use stored VPR representations to recover original images or 3D structure. This poses a practical concern for commercial robots that map homes, hospitals, and offices. No place-representation scheme yet guarantees privacy without sacrificing performance.

---

By this point, three SLAM lineages had matured. ORB-SLAM standardized the feature-based pipeline, DSO developed the photometric formulation, and KinectFusion and its successors established the capabilities and limits of dense mapping. Place recognition differed because it grew from image retrieval in computer vision rather than from SLAM itself, later supplying the loop-closure component that SLAM required. This separation let place recognition adopt deep-learning methods faster than the rest of the SLAM pipeline.

When AnyLoc appeared in 2023, it cited Sivic's work. He had introduced BoW to image retrieval in 2003 and co-authored NetVLAD in 2016, when learned aggregation moved beyond some of BoW's limitations. AnyLoc extended that lineage by applying foundation-model features to place recognition.

A different challenge came from a graduate student at NYU. His depth-estimation CNN tested whether geometry had to be recovered only through geometric methods.
