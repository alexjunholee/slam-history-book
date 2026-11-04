# Ch.10 — Place Recognition: FAB-MAP에서 NetVLAD, AnyLoc까지

Oxford 모바일 로보틱스 그룹의 Mark Cummins와 Paul Newman은 로봇이 이전에 지나간 장소를 알아보는 문제를 연구했다. VO(visual odometry, 영상만으로 이동량을 추정하는 방법)에는 drift가 쌓이므로, SLAM 시스템이 loop를 닫으려면 이 문제를 풀어야 한다. feature 기반, direct, dense 방식 모두 loop closure가 필요했다. 지금 보는 장면을 전에 어디서 봤는지 판단하는 일은 place recognition(장소 인식)이 맡았고, 이 분야는 tracking, mapping과 따로 발전했다.

---

## 10.1 BoW 이전의 place recognition

GPS가 잡히지 않는 실내, 터널, 고층 건물 사이에서 로봇이 loop를 닫으려면 과거 이미지 수천 장 가운데 지금 관측과 비슷한 것을 빨리 찾아야 한다. 모든 이미지와 하나씩 비교하면 이미지 수에 비례해 시간이 드는 선형 탐색이 되어, 이미지가 수만 장을 넘으면 실시간으로 처리하기 어렵다.

컴퓨터 비전에서는 Oxford의 Sivic과 Zisserman이 비슷한 검색 문제를 다뤘다. 이들이 2003년 ICCV에서 발표한 ["Video Google"](https://www.robots.ox.ac.uk/~vgg/publications/2003/Sivic03/sivic03.pdf)은 동영상 속 물체를 찾으려고 문서 검색의 TF-IDF(단어 빈도에 희귀도 가중치를 곱한 점수)를 이미지에 적용했다. SIFT 서술자를 k-means로 군집화해 "visual word"를 만들고, 이미지를 그 단어들의 빈도 벡터로 표현했다. inverted index(단어마다 그 단어가 나온 이미지 목록을 저장한 색인)를 쓰면 질의에 나온 visual word의 목록만 조회하면 되므로, 전체 영상을 훑지 않아도 된다. place recognition 연구자들은 이 방법을 곧 가져다 썼다.

---

## 10.2 FAB-MAP — 확률적 BoW와 Chow-Liu 트리 (2008)

Cummins와 Newman은 2008년 [Cummins & Newman. FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance](https://doi.org/10.1177/0278364908090961)를 발표했다.

FAB-MAP(Fast Appearance-Based Mapping)은 현재 장면이 데이터베이스의 어느 장소에 해당하는지, 혹은 새 장소인지를 확률로 판단한다. 비슷해 보이는 장소가 여러 곳 있으면 유사도가 가장 높은 후보가 틀릴 수 있어서, 유사도 점수만으로는 부족하다.

Cummins와 Newman은 이를 베이즈 추론 문제로 세웠다. 관측 $z_t$(visual word의 발생 여부 집합)가 주어졌을 때, 현재 위치가 데이터베이스의 각 장소 $\ell_i$일 확률을 계산한다:

$$P(\ell_i \mid z_t) \propto P(z_t \mid \ell_i) P(\ell_i)$$

어려운 부분은 $P(z_t \mid \ell_i)$다. visual word들이 서로 독립이라고 가정하면 naïve Bayes가 된다. 실제 visual word들은 서로 상관이 있다. "문"이라는 word가 등장하면 "문손잡이"라는 word도 같이 등장하기 쉽다. 독립 가정은 확률 값을 왜곡한다.

FAB-MAP은 **Chow-Liu tree**로 이 상관을 모델링했다. Chow-Liu tree는 word 쌍의 mutual information 합이 최대가 되도록 만든 트리 구조 그래프 모델이다. 두 word $e_i, e_j$ 사이의 mutual information은

$$I(e_i; e_j) = \sum_{e_i, e_j} P(e_i, e_j) \log \frac{P(e_i, e_j)}{P(e_i)P(e_j)}$$

로 정의되고, Chow-Liu 알고리즘은 이를 엣지 가중치로 삼아 최대 스패닝 트리를 구성한다. 이 트리로 joint likelihood를 분해하면

$$P(z_t \mid \ell_i) = \prod_k P(z_t^k \mid z_t^{\text{pa}(k)}, \ell_i)$$

가 된다. 여기서 $z_t^k \in \{0,1\}$은 $k$번째 word의 발생 여부이고, $\text{pa}(k)$는 트리에서 $k$의 부모 노드다. 이 분해는 word들이 함께 나타나는 패턴을 반영하므로, 복도처럼 비슷해 보이는 장소에서 false positive(잘못된 일치)가 줄어든다. vocabulary와 tree는 큰 이미지 집합으로 미리 학습한다.

FAB-MAP은 지금 위치가 데이터베이스에 없는 새 장소일 가능성도 따로 계산한다. 이 "new place" 가설을 넣자 false positive가 줄었다. loop closure에서 false positive가 하나만 나와도 지도 전체가 망가질 수 있다.

> 🔗 **차용.** FAB-MAP의 visual word는 Sivic & Zisserman의 "Video Google"(2003)에서 가져왔다. FAB-MAP은 문서 검색에서 쓰던 방법을 로봇의 장소 인식에 적용했다.

2011년 Cummins와 Newman은 [FAB-MAP 2.0](https://www.robots.ox.ac.uk/~mjc/Papers/cummins_newman_ijrr_fabmap2_2010_preprint.pdf)을 발표했다. 논문은 1,000 km 길이의 경로에서 appearance(겉모습)만으로 loop closure를 검출했다.

---

## 10.3 DBoW2 — binary descriptor와 vocabulary tree (2012)

FAB-MAP은 SURF 같은 부동소수점 서술자를 썼다. 2012년 무렵 SLAM 연구자들은 더 빠른 binary descriptor(BRIEF, ORB, BRISK 등)로 옮겨 가고 있었다. 부동소수점 서술자의 vocabulary를 그대로 쓰면 계산이 무거웠다.

Dorian Gálvez-López와 Juan D. Tardós(Universidad de Zaragoza)는 2012년 [Gálvez-López & Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences](https://doi.org/10.1109/TRO.2012.2197158)를 발표했다. Ch.7에서 본 **DBoW2**는 binary descriptor로 만든 vocabulary tree다. 서술자를 Hamming distance로 비교하므로 SIFT·SURF를 쓸 때보다 word를 훨씬 빨리 배정할 수 있었다.

이미지를 나타내는 BoW 벡터는 binary word의 빈도에 TF-IDF 가중치를 곱한 벡터다. $k$분기 $d$깊이 트리의 각 리프 노드 $w_i$에 TF-IDF 가중치

$$\eta_i = \frac{n_i}{n} \cdot \log \frac{N}{N_i}$$

를 준다. 여기서 $n_i$는 해당 이미지에 이 word가 나온 횟수, $n$은 총 word 수, $N$은 데이터베이스 이미지 수, $N_i$는 $w_i$를 포함한 이미지 수다. 두 이미지 $a$, $b$의 유사도는 L1-norm

$$s(\mathbf{v}_a, \mathbf{v}_b) = 1 - \frac{1}{2} \left\| \frac{\mathbf{v}_a}{|\mathbf{v}_a|} - \frac{\mathbf{v}_b}{|\mathbf{v}_b|} \right\|_1$$

으로 계산한다. Inverted index의 조회 비용은 word 수와 각 목록의 길이에 따라 달라진다.

> 🔗 **차용.** DBoW2의 vocabulary tree 개념은 Nistér & Stewénius의 2006년 ["Scalable Recognition with a Vocabulary Tree"](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/nister_stewenius_cvpr2006.pdf)(CVPR)에서 왔다. DBoW2는 이 구조를 binary descriptor에 맞게 바꾸고 가중치를 SLAM에 맞게 조정했다.

DBoW2는 오픈소스 라이브러리로 공개되어 널리 쓰였다. ORB-SLAM(2015)이 loop closure에 DBoW2를 썼고, ORB-SLAM2와 ORB-SLAM3도 DBoW2를 썼다. 공저자 Tardós는 ORB-SLAM 세 편 모두의 저자이기도 하다.

---

## 10.4 NetVLAD — CNN 기반 VPR (2016)

BoW 방식의 vocabulary는 특정 서술자로 특정 환경에서 학습한 것이다. 조명, 계절, 시점이 크게 바뀌면 visual word의 분포가 달라져서 미리 학습한 vocabulary가 잘 맞지 않았다.

Relja Arandjelović, Petr Gronat, Akihiko Torii, Tomáš Pajdla, Josef Sivic은 2016년 CVPR에서 [NetVLAD: CNN Architecture for Weakly Supervised Place Recognition](https://doi.org/10.1109/CVPR.2016.572)을 발표했다. 저자 Sivic은 2003년 "Video Google"의 저자이기도 하다.

NetVLAD는 **VLAD(Vector of Locally Aggregated Descriptors)** aggregation(국소 서술자를 이미지 하나의 벡터로 모으는 방법)을 미분 가능하게 만들었다.

VLAD는 2010년 [Jégou et al.](https://inria.hal.science/inria-00548637/file/jegou_compactimagerepresentation.pdf)이 제안했다. 각 local descriptor와 가장 가까운 cluster center(visual word) 사이의 차이를 cluster별로 더해 이미지 전체를 표현한다. cluster center $k$의 VLAD 부분 벡터는

$$\mathbf{V}(k) = \sum_{\mathbf{x}_i : \text{NN}(\mathbf{x}_i)=k} (\mathbf{x}_i - \boldsymbol{\mu}_k)$$

이고, 전체 VLAD 벡터 $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$는 이를 모든 cluster에 대해 연결(concatenate)한 뒤 L2-normalize한 것이다. $K$ clusters, $D$차원 descriptor라면 최종 벡터는 $KD$차원이다. VLAD 벡터는 word마다 개수만 세는 BoW보다 정보를 많이 담는다.

> 🔗 **차용.** NetVLAD의 aggregation 설계는 Jégou et al.의 "Aggregating Local Descriptors into a Compact Image Representation"(CVPR 2010)의 VLAD를 이어받았다. NetVLAD는 VLAD의 hard assignment를 soft assignment로 바꿔 전체 파이프라인을 end-to-end로 학습할 수 있게 했다.

NetVLAD layer는 기존 VLAD의 nearest-neighbor 할당을 softmax로 완화한다:

$$\bar{a}_k(\mathbf{x}_i) = \frac{e^{\mathbf{w}_k^\top \mathbf{x}_i + b_k}}{\sum_{k'} e^{\mathbf{w}_{k'}^\top \mathbf{x}_i + b_{k'}}}$$

여기서 $\mathbf{x}_i$는 CNN에서 추출한 local feature, $\mathbf{w}_k$와 $b_k$는 학습 가능한 파라미터다. 이 soft 할당으로 NetVLAD 벡터를 누적하면

$$\mathbf{V}(k) = \sum_i \bar{a}_k(\mathbf{x}_i)\,(\mathbf{x}_i - \boldsymbol{\mu}_k)$$

이고, 전체 벡터 $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$를 intra-normalization(각 부분 벡터 L2 정규화) 후 전체를 다시 L2-normalize하면 최종 VPR(visual place recognition) descriptor가 된다. gradient가 역전파되므로 CNN backbone과 함께 end-to-end로 학습할 수 있다.

저자들은 Google Street View Time Machine(같은 장소를 여러 시기에 찍은 Street View 이미지) 데이터로 같은 장소의 다른 시점 이미지 쌍을 양성 예, 다른 장소를 음성 예로 삼는 weakly supervised triplet loss를 썼다. GPS 위치를 약한 감독 신호로 써서 사람이 장소 레이블을 달지 않아도 학습할 수 있었다.

Pittsburgh 250k와 Tokyo 24/7 벤치마크에서 NetVLAD는 기존 VLAD 기반 방법과 학습하지 않은 CNN 표현을 크게 앞섰다. 낮과 밤처럼 조건이 바뀌어도 더 잘 버텼다. NetVLAD가 실제 SLAM 파이프라인에 바로 들어가지는 않았다. 추론 속도와 메모리 부담이 DBoW2보다 컸다.

---

## 10.5 Patch-NetVLAD, MixVPR, AnyLoc (2020-2023)

NetVLAD 뒤로 VPR 연구자들은 조건과 환경이 바뀌어도 성능이 유지되는 방법을 찾았다. 이 시기 VPR 논문들은 Mapillary Street Level Sequences(MSLS)와 계절 변화가 큰 Nordland 같은 데이터셋으로 평가했다.

2021년 Hausler et al.은 [Patch-NetVLAD](https://arxiv.org/abs/2103.01486)를 내놓았다. Patch-NetVLAD는 NetVLAD의 global descriptor로 후보를 먼저 찾고, 이미지를 patch로 나눠 patch별 NetVLAD 표현을 공간적으로 맞춰 보며 후보의 순위를 다시 매긴다. Tokyo 24/7에서 Recall@1은 NetVLAD의 64.8%에서 86.0%로 올랐다.

2023년 Ali-bey et al.의 [MixVPR](https://arxiv.org/abs/2303.02190)는 사전 학습된 backbone의 feature map을 MLP로 섞어(Feature-Mixer) global feature를 만들었다. 파라미터 수는 NetVLAD의 절반이 안 되면서 Pitts250k, MSLS, Nordland에서 기존 방법보다 높은 Recall@1을 보고했다.

Keetha et al.의 [AnyLoc: Towards Universal Visual Place Recognition](https://arxiv.org/abs/2308.00688)(RA-L 2024, arXiv 공개는 2023년)은 DINOv2의 self-supervised feature를 fine-tuning 없이 그대로 place recognition에 썼다.

> 🔗 **차용.** AnyLoc의 feature 추출은 Oquab et al.의 [DINOv2](https://arxiv.org/abs/2304.07193)(Meta AI, 2023)에서 사전 학습한 ViT 표현을 가져와 VLAD로 모은다. BoW와 VLAD 시절의 aggregation을 foundation model(대규모 데이터로 미리 학습해 여러 작업에 쓰는 모델) feature에 다시 적용했다.

DINOv2는 대규모 인터넷 이미지로 학습된 Vision Transformer(ViT)다. 여러 환경에 쓸 수 있는 범용 feature를 낸다. Keetha et al.은 DINOv2의 **facet**을 비교했다. facet은 ViT layer 안의 query(Q), key(K), value(V), token(patch feature) 출력을 가리킨다. 실험에서는 DINOv2의 value facet이 평균적으로 place recognition에 가장 좋았다. 이 value facet을 비지도 VLAD로 모으자, 재학습이나 fine-tuning 없이 도시, 실외, 실내, 항공, 수중, 지하 환경에서 하나의 방법이 동작했다. 논문은 기존 방법보다 최대 4배 높은 Recall@1을 보고했다.

센서 종류가 다른 데이터 사이의 장소 인식도 연구됐다. Lee et al.의 [(LC)²](https://arxiv.org/abs/2304.08660) (RA-L 2023)는 카메라 영상에서 얻은 disparity image와 LiDAR range image를 둘 다 2.5D depth image로 보고 서로 맞췄다. 이 cross-modal 일치를 pose graph의 loop 제약으로 넣어, 미리 만든 point cloud 지도 없이 LiDAR 기반 위치 추정을 했다. 조명이 나쁜 환경의 평가에는 Lee et al.의 [ViViD++](https://arxiv.org/abs/2204.06183) (RA-L 2022) 같은 데이터셋을 쓸 수 있다. ViViD++는 RGB, thermal, event, depth 카메라와 관성 센서를 함께 달고 같은 장소를 조명 조건을 바꿔 가며 반복 기록했다.

---

## 10.6 place recognition과 metric localization의 통합 시도 (2023-2024)

place recognition 모듈은 SLAM의 다른 부분과 떨어져 동작해 왔다. ORB-SLAM의 DBoW2 모듈은 mapping·tracking과 상관없이 이미지를 받아 loop 후보 ID만 내놓는다.

2023–2024년에는 Berton et al.의 [EigenPlaces](https://arxiv.org/abs/2308.10832)(2023)와 Izquierdo & Civera의 [SALAD](https://arxiv.org/abs/2311.15937)(2023 arXiv / CVPR 2024)가 시점 변화에 강한 global descriptor와 local feature aggregation을 발전시켰다. 두 방법이 내놓는 것도 데이터베이스에서 찾은 이미지다. metric pose(미터 단위의 실제 자세)가 필요하면 검색된 후보와의 기하 검증이나 별도의 localization 단계가 뒤따라야 한다.

2024년에는 3DGS(3D Gaussian Splatting)로 만든 장면 표현에서 place recognition용 descriptor를 뽑으려는 시도도 나왔다.

---

## 10.7 🧭 아직 열린 것

**계절·조명 극변.** Nordland(노르웨이 철도, 여름-겨울)와 Oxford RobotCar(1년치 계절 변화) 데이터셋에서는 이 어려움이 10년 넘게 보고된다. DINOv2 기반 방법이 이 격차를 줄였다.

**Place recognition과 metric localization의 통합.** ORB-SLAM 계열 파이프라인은 place recognition으로 후보 장소를 찾고, 서술자 매칭으로 기하 대응을 구한 뒤 PnP 같은 방법으로 자세를 추정한다. 두 과정을 표현 하나로 합쳤을 때 정밀도와 속도를 센서·장면·배포 조건마다 함께 만족하는지 확인하는 일은 남은 과제다.

**인식 가능한 장소 표현의 프라이버시.** 위치 추정용 지도도 복원 공격의 대상이 된다. [Pittaluga 등(CVPR 2019)](https://arxiv.org/abs/1904.03303)은 서술자가 붙은 SfM point cloud에서 장면 이미지를 되살렸다. 이 결과대로라면 가정, 병원, 사무실의 실내 지도를 만드는 상업용 로봇도 같은 위험을 안는다. 같은 해 [Speciale 등(CVPR 2019)](https://arxiv.org/abs/1903.05572)은 3D 점을 무작위 방향의 직선으로 바꾼 line cloud를 지도로 쓰자고 했다. 2021년 [Chelani, Kahl, Sattler](https://arxiv.org/abs/2103.05086)는 line cloud에서도 점의 위치와 이미지 내용을 대략 되살릴 수 있음을 보였다. 선을 아주 성기게 남기면 이미지의 세부는 되살리지 못했다. 저자들은 프라이버시를 지키는 장면 표현을 더 연구해야 한다고 결론에 적었다.
