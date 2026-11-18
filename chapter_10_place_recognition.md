# 13장 장소 인식: FAB-MAP에서 AnyLoc까지

Oxford 모바일 로보틱스 그룹의 Mark Cummins와 Paul Newman이 파고든 질문은 단순하면서도 치명적이었다. "로봇이 방금 지나친 이 장소를 예전에 본 적이 있는가?" 영상만으로 이동량을 쌓아 가는 visual odometry에는 어쩔 수 없이 drift가 누적되므로, feature 기반이든 direct든 dense든 모든 SLAM 시스템이 큰 오차를 털어 내려면 결국 loop를 닫아야 한다. 지금 카메라에 비친 장면이 수천 장의 과거 기록 중 어디와 같은지 가려내는 이 역할은 **장소 인식**(place recognition)이 도맡았고, 흥미롭게도 이 분야는 tracking·mapping과 사뭇 다른 궤적을 그리며 독자적으로 발전했다. 특히 장소 인식은 손으로 깎아 만든 visual word 통계에서 출발해 딥러닝 기반 전역 서술자로 가장 먼저, 그리고 깊숙이 옮겨 간 영역이기도 하다.

---

## 13.1 BoW 이전의 place recognition

GPS 신호가 닿지 않는 실내나 터널, 고층 빌딩 숲에서 로봇이 루프를 닫으려면 과거에 찍어 둔 이미지 수천, 수만 장 가운데 현재 장면과 닮은 것을 눈 깜짝할 사이에 찾아내야 한다. 데이터베이스의 모든 이미지를 하나씩 꺼내 비교하는 선형 탐색으로는 이미지가 수만 장만 넘어가도 실시간 제한 시간을 훌쩍 넘겨 버린다.

돌파구는 SLAM 바깥, 컴퓨터 비전의 영상 검색 연구에서 왔다. 2003년 ICCV에서 Oxford의 Sivic과 Zisserman이 발표한 ["Video Google"](https://www.robots.ox.ac.uk/~vgg/publications/2003/Sivic03/sivic03.pdf)은 동영상 속에서 특정 물체가 나온 장면을 곧바로 찾아내기 위해 텍스트 검색의 고전인 TF-IDF(단어 빈도에 희귀도 가중치를 곱한 점수)를 이미지로 가져왔다. SIFT 서술자를 k-means로 군집화해 일종의 시각 단어인 "visual word" 사전을 만들고, 각 이미지를 이 단어들의 빈도 벡터로 요약한 것이다. 여기에 단어마다 그 단어가 등장한 이미지 목록을 미리 매달아 둔 inverted index를 얹으면, 질의 영상에 포함된 visual word 목록만 훑고도 후보 영상을 즉시 추려 낼 수 있다. 장소 인식 연구자들은 이 검색 엔진 구조를 지체 없이 로보틱스로 가져왔다.

---

## 13.2 FAB-MAP — 확률적 BoW와 Chow-Liu 트리 (2008)

하지만 영화 속 물체를 찾는 것과 로봇의 loop closure는 무게가 달랐다. 2008년 Cummins와 Newman은 [Cummins & Newman. FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance](https://doi.org/10.1177/0278364908090961)에서 단순한 유사도 점수만으로는 왜 충분하지 않은지를 분명히 짚었다. 비슷한 구조물이 반복되는 환경에서는 가장 높은 유사도 점수를 받은 후보조차 가짜일 수 있으며, SLAM에서 잘못된 loop closure(false positive)가 단 하나라도 그래프에 끼어들면 지도 전체가 산산조각 나기 때문이다.

두 사람은 유사도 순위 매기기 대신 엄밀한 베이즈 추론으로 문제를 다시 세웠다. 현재 관측 $z_t$(어떤 visual word들이 나타났는지에 대한 집합)가 주어졌을 때, 데이터베이스의 각 장소 $\ell_i$에 있을 사후 확률은 다음과 같다.

$$P(\ell_i \mid z_t) \propto P(z_t \mid \ell_i) P(\ell_i)$$

핵심 난관은 우도 $P(z_t \mid \ell_i)$를 어떻게 모델링하느냐였다. 모든 visual word가 서로 독립이라고 가정하면 간단한 naïve Bayes가 되지만, 현실의 시각 단어들은 강하게 얽혀 있다. 가령 "문"에 해당하는 단어가 보이면 "문손잡이"에 해당하는 단어도 덩달아 등장하기 쉽다. 이 상관관계를 무시하고 독립을 가정하면 흔한 구조물이 겹칠 때마다 확률이 과대평가되어 오탐이 속출한다.

FAB-MAP은 이 상관관계를 **Chow-Liu tree**로 포착했다. Chow-Liu tree는 visual word 쌍 $(e_i, e_j)$ 사이의 상호정보량(mutual information)

$$I(e_i; e_j) = \sum_{e_i, e_j} P(e_i, e_j) \log \frac{P(e_i, e_j)}{P(e_i)P(e_j)}$$

의 합이 최대가 되도록 짠 최대 스패닝 트리다. 이 트리 구조 위에서 joint likelihood를 분해하면 다음과 같은 연쇄 곱 형태가 된다.

$$P(z_t \mid \ell_i) = \prod_k P(z_t^k \mid z_t^{\text{pa}(k)}, \ell_i)$$

여기서 $z_t^k \in \{0,1\}$은 $k$번째 word의 출현 여부, $\text{pa}(k)$는 트리에서 $k$의 부모 노드다. 단어들이 함께 등장하는 공존 패턴을 확률 모델 안에 녹여 낸 덕분에 복도처럼 닮은 장소가 줄지어 있어도 false positive를 크게 걸러 낼 수 있었다(vocabulary와 tree는 대량의 별도 학습 영상에서 미리 구해 둔다). 여기에 FAB-MAP은 현재 위치가 기존 데이터베이스에 없는 전혀 새로운 장소일 확률까지 "new place" 가설로 명시적으로 계산해 넣음으로써 오탐의 틈을 한 번 더 막았다.

그 위력은 2011년 발표된 [FAB-MAP 2.0](https://www.robots.ox.ac.uk/~mjc/Papers/cummins_newman_ijrr_fabmap2_2010_preprint.pdf)에서 증명됐다. FAB-MAP 2.0은 무려 1,000 km에 달하는 도로 주행 경로에서 오직 영상의 겉모습(appearance)만으로 loop closure를 성공적으로 검출해 냈다.

> 🔗 **차용.** FAB-MAP이 visual word로 장면을 요약하는 뼈대는 Sivic & Zisserman의 "Video Google"(2003)에서 왔다. Cummins와 Newman은 문서 검색용 BoW 기법을 베이즈 확률과 Chow-Liu 트리 위에서 로봇 항법의 안전 기준에 맞게 재탄생시켰다.

---

## 13.3 DBoW2 — binary 서술자와 vocabulary tree (2012)

FAB-MAP이 활약하던 시기에도 SURF 같은 부동소수점 서술자는 여전히 무거웠다. 2012년 무렵 SLAM 진영이 BRIEF, ORB, BRISK처럼 비트 연산으로 매칭하는 훨씬 가벼운 binary 서술자로 이동하면서, 장소 인식 역시 부동소수점 vocabulary의 계산 굴레를 벗어날 필요가 생겼다.

스페인 Zaragoza 대학의 Dorian Gálvez-López와 Juan D. Tardós는 2012년 [Gálvez-López & Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences](https://doi.org/10.1109/TRO.2012.2197158)를 발표하며 **DBoW2**를 내놓았다. binary 서술자로 계층적 vocabulary tree를 직접 구축해 Hamming distance만으로 word를 배정할 수 있게 한 덕분에, SIFT·SURF 기반 방식보다 훨씬 빠른 속도로 장소를 검색할 수 있었다.

DBoW2는 $k$분기 $d$깊이 트리의 각 리프 노드 $w_i$에 TF-IDF 가중치

$$\eta_i = \frac{n_i}{n} \cdot \log \frac{N}{N_i}$$

를 곱해 BoW 벡터를 만든다. 여기서 $n_i$는 해당 이미지에 word $w_i$가 등장한 횟수, $n$은 이미지 내 전체 word 수, $N$은 데이터베이스 총 이미지 수, $N_i$는 $w_i$를 포함한 이미지 수다. 두 이미지 $a$와 $b$의 BoW 벡터 $\mathbf{v}_a, \mathbf{v}_b$ 사이의 유사도는 L1-norm 기반으로 다음과 같이 잰다.

$$s(\mathbf{v}_a, \mathbf{v}_b) = 1 - \frac{1}{2} \left\| \frac{\mathbf{v}_a}{|\mathbf{v}_a|} - \frac{\mathbf{v}_b}{|\mathbf{v}_b|} \right\|_1$$

이 계산은 inverted index 조회와 맞물려 매우 가볍게 돌아갔다(조회 비용은 word 수와 각 목록 길이에 비례한다). 오픈소스로 풀린 DBoW2는 곧바로 ORB-SLAM(2015), ORB-SLAM2, ORB-SLAM3의 loop closure 심장으로 자리 잡았고, 공저자 Tardós 역시 ORB-SLAM 세 편 모두에 이름을 올렸다.

> 🔗 **차용.** DBoW2가 채택한 계층적 vocabulary tree 구조는 Nistér & Stewénius의 2006년 ["Scalable Recognition with a Vocabulary Tree"](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/nister_stewenius_cvpr2006.pdf)(CVPR)에서 빌려 왔다. Gálvez-López와 Tardós는 이 트리를 binary 서술자의 Hamming 공간에 맞게 다듬고 가중치와 시간적 일관성 검사를 SLAM에 최적화했다.

---

## 13.4 NetVLAD — CNN 기반 VPR (2016)

BoW 방식은 빠르고 가벼웠지만 근본적인 약점이 있었다. 미리 학습해 둔 vocabulary가 특정 환경과 특정 수작업 서술자에 묶여 있어, 밤낮이 바뀌거나 계절이 변하고 카메라 시점이 크게 틀어지면 visual word 분포 자체가 무너져 버린 것이다.

2016년 CVPR에서 Relja Arandjelović, Petr Gronat, Akihiko Torii, Tomáš Pajdla, Josef Sivic이 발표한 [NetVLAD: CNN Architecture for Weakly Supervised Place Recognition](https://doi.org/10.1109/CVPR.2016.572)은 딥러닝으로 이 장벽을 넘었다. 공저자 Sivic이 13년 전 "Video Google"을 썼던 바로 그 연구자라는 점도 시대의 전환을 상징한다.

NetVLAD의 핵심은 국소 서술자들을 이미지 한 장의 고정 길이 벡터로 묶는 **VLAD**(Vector of Locally Aggregated Descriptors) 연산을 미분 가능하게 뜯어고친 데 있다. 2010년 [Jégou 등](https://inria.hal.science/inria-00548637/file/jegou_compactimagerepresentation.pdf)이 제안한 원래 VLAD는 각 국소 서술자 $\mathbf{x}_i$와 가장 가까운 군집 중심 $\boldsymbol{\mu}_k$ 사이의 잔차 벡터를 군집별로 누적해 단어 빈도만 세는 BoW보다 훨씬 풍부한 기하·외관 정보를 담았다.

$$\mathbf{V}(k) = \sum_{\mathbf{x}_i : \text{NN}(\mathbf{x}_i)=k} (\mathbf{x}_i - \boldsymbol{\mu}_k)$$

전체 VLAD 벡터 $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$는 군집별 부분 벡터를 모두 이어 붙인(concatenate) 뒤 L2 정규화한 $KD$차원 벡터다($K$개 군집, $D$차원 서술자).

문제는 가장 가까운 군집 하나를 고르는 $\text{NN}(\mathbf{x}_i)=k$ 연산이 불연속이라 기울기가 흐르지 않는다는 점이었다. NetVLAD layer는 이 hard assignment를 학습 가능한 파라미터 $\mathbf{w}_k, b_k$를 쓴 softmax 기반 soft assignment로 부드럽게 풀어냈다.

$$\bar{a}_k(\mathbf{x}_i) = \frac{e^{\mathbf{w}_k^\top \mathbf{x}_i + b_k}}{\sum_{k'} e^{\mathbf{w}_{k'}^\top \mathbf{x}_i + b_{k'}}}$$

CNN feature map에서 뽑은 국소 특징 $\mathbf{x}_i$에 이 가중치를 적용해 누적하면

$$\mathbf{V}(k) = \sum_i \bar{a}_k(\mathbf{x}_i)\,(\mathbf{x}_i - \boldsymbol{\mu}_k)$$

이 되고, 이어 $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$의 각 부분 벡터를 먼저 정규화(intra-normalization)한 뒤 전체 벡터를 다시 L2 정규화해 최종 VPR(visual place recognition) 서술자를 얻는다. 이제 전 과정이 미분 가능하므로 CNN backbone까지 한꺼번에 end-to-end로 학습시킬 수 있었다.

학습 데이터의 병목도 영리하게 우회했다. 저자들은 같은 거리를 서로 다른 해와 계절에 찍은 Google Street View Time Machine 이미지와 GPS 태그를 활용해, 가까운 위치의 다른 시점 사진은 양성 쌍으로, 먼 위치의 사진은 음성 쌍으로 묶는 weakly supervised triplet loss를 썼다. 사람이 일일이 정밀한 대응 레이블을 달아 주지 않아도 강건한 장소 표현을 학습할 수 있었던 비결이다.

Pittsburgh 250k와 Tokyo 24/7 벤치마크에서 NetVLAD는 기존 VLAD와 비학습 CNN 특징을 큰 격차로 따돌렸고, 특히 낮과 밤이 뒤바뀌는 가혹한 조건에서도 잘 버텼다. 다만 DBoW2에 비해 추론 속도가 무겁고 메모리 요구량이 컸기에, 당시 실시간 SLAM 파이프라인에 곧장 갈아 끼워지기보다는 주로 대규모 VPR과 후처리 재위치 추정 영역에서 표준으로 자리 잡았다.

> 🔗 **차용.** NetVLAD의 집계 계층은 Jégou 등이 CVPR 2010에 발표한 "Aggregating Local Descriptors into a Compact Image Representation"의 VLAD 설계를 그대로 계승하되, hard assignment를 softmax soft assignment로 치환해 종단간 역전파 학습의 길을 열었다.

---

## 13.5 Patch-NetVLAD, MixVPR, AnyLoc (2020-2023)

NetVLAD 이후 VPR 연구는 계절과 날씨, 시점 변화가 극단으로 치닫는 조건에서도 무너지지 않는 표현을 찾는 데 집중했고, 평가 무대도 Mapillary Street Level Sequences(MSLS)나 노르웨이 철도의 여름·겨울을 담은 Nordland 같은 데이터셋으로 옮겨 갔다.

2021년 Hausler 등의 [Patch-NetVLAD](https://arxiv.org/abs/2103.01486)는 global NetVLAD 서술자로 후보를 빠르게 추린 뒤, 이미지를 패치 단위로 쪼개 뽑은 다중 스케일 NetVLAD 특징들의 공간적 일치도를 따져 순위를 재정렬함으로써 Tokyo 24/7의 Recall@1을 NetVLAD의 64.8%에서 86.0%까지 끌어올렸다. 2023년 Ali-bey 등의 [MixVPR](https://arxiv.org/abs/2303.02190)는 사전 학습된 backbone의 feature map을 MLP 블록(Feature-Mixer)으로 통째로 섞어 전역 특징을 빚어냈고, NetVLAD의 절반도 안 되는 파라미터 수로 Pitts250k, MSLS, Nordland에서 기존 기록을 경신했다.

더 큰 전환점은 foundation model의 등판이었다. Keetha 등의 [AnyLoc: Towards Universal Visual Place Recognition](https://arxiv.org/abs/2308.00688)(arXiv 2023, RA-L 2024)은 대규모 인터넷 이미지로 자기지도 학습된 Vision Transformer(ViT)인 DINOv2의 특징을 아무런 fine-tuning 없이 VPR에 곧바로 투입했다. 연구진은 ViT 계층 내부의 **facet**—곧 query(Q), key(K), value(V), token(patch feature)—을 꼼꼼히 비교한 끝에 value facet이 장소 인식에서 평균적으로 가장 뛰어나다는 사실을 확인했고, 여기에 비지도 VLAD 집계를 결합하자 재학습 없이도 도시, 실외, 실내, 항공, 수중, 지하까지 단일 방법으로 작동하며 기존 대비 최대 4배 높은 Recall@1을 달성했다.

한편 카메라와 LiDAR처럼 서로 다른 센서 사이의 장벽을 허무는 교차 모달(cross-modal) 장소 인식도 열렸다. Lee 등의 [(LC)²](https://arxiv.org/abs/2304.08660)(RA-L 2023)는 스테레오 카메라에서 얻은 disparity 영상과 LiDAR range 영상을 똑같은 2.5D 깊이 이미지 공간으로 옮겨 서로 매칭함으로써, 사전 구축된 3D point cloud 지도 없이도 카메라–LiDAR 간 loop 제약을 걸어 위치 추정을 해냈다. 또 Lee 등이 만든 [ViViD++](https://arxiv.org/abs/2204.06183)(RA-L 2022)처럼 RGB, 열화상(thermal), event, depth 카메라와 관성 센서를 한 몸에 싣고 같은 경로를 낮과 밤을 바꿔 반복 기록한 데이터셋도 등장해 악천후·저조도 평가의 토대를 놓았다.

> 🔗 **차용.** AnyLoc의 특징 추출기는 Meta AI가 공개한 [DINOv2](https://arxiv.org/abs/2304.07193)(Oquab et al., 2023)다. 흥미롭게도 2000–2010년대 BoW와 VLAD 시절의 고전적 집계 기법이 2023년 foundation model의 범용 ViT 특징과 만나 '학습 없는 범용 VPR'로 다시 태어난 셈이다.

---

## 13.6 place recognition과 metric localization의 통합 시도 (2023-2024)

오랫동안 장소 인식은 SLAM의 나머지 단계와 분리된 별동대처럼 움직였다. ORB-SLAM 안의 DBoW2만 보더라도 mapping이나 tracking 상태를 살피는 대신 이미지를 받아 "이전에 본 몇 번 keyframe과 비슷하다"는 후보 ID만 뱉어 낼 뿐, 실제 기하 정합은 뒤따르는 RANSAC과 PnP에 넘겼다.

2023–2024년에는 Berton 등의 [EigenPlaces](https://arxiv.org/abs/2308.10832)(2023)와 Izquierdo·Civera의 [SALAD](https://arxiv.org/abs/2311.15937)(arXiv 2023 / CVPR 2024)가 시점 변화에 한층 강한 전역 서술자와 최적 수송(optimal transport) 기반 국소 특징 집계를 밀어 올렸다. 하지만 이들이 내놓는 결과 역시 데이터베이스 속 가장 닮은 이미지일 뿐이어서, 미터 단위의 정밀한 metric pose를 얻으려면 결국 기하 검증과 별도의 위치 추정 단계를 거쳐야 한다. 2024년에는 3D Gaussian Splatting(3DGS)으로 구축한 장면 표현 자체에서 곧바로 장소 인식용 서술자를 끌어내어 지도 표현과 재위치 추정을 하나로 엮으려는 시도도 고개를 들었다.

---

## 아직 열린 문제

**계절·조명 극변.** 여름과 겨울이 극단적으로 대비되는 Nordland 노르웨이 철도 데이터셋이나 사계절을 1년 내내 기록한 Oxford RobotCar는 10년 넘게 장소 인식 알고리즘을 괴롭혀 왔다. DINOv2 같은 foundation model이 그 간극을 크게 좁히긴 했으나, 눈보라가 치거나 식생이 완전히 뒤바뀐 환경에서도 오탐 없이 작동하는 일은 여전히 쉽지 않다.

**Place recognition과 metric localization의 통합.** 전통적인 ORB-SLAM 계열은 '장소 인식(후보 검색) → 서술자 매칭 → 기하적 자세 추정'이라는 다단계 파이프라인을 밟는다. 검색용 전역 표현과 정밀 자세 추정용 3D 기하 표현을 하나로 통합했을 때, 다양한 센서와 장면·배포 제약 조건 아래에서도 정밀도와 속도를 동시에 지켜 낼 수 있는지는 아직 검증이 진행 중이다.

**인식 가능한 장소 표현의 프라이버시.** 위치 추정을 위해 서버나 로봇에 저장하는 지도 자체가 사생활 유출의 통로가 될 수 있다는 경고도 무겁다. [Pittaluga 등(CVPR 2019)](https://arxiv.org/abs/1904.03303)은 서술자가 붙은 3D SfM point cloud만 가지고도 원래 그 장소를 찍었던 카메라 영상을 섬뜩할 만큼 선명하게 복원해 보임으로써, 가정·병원·사무실을 누비는 상용 로봇의 지도가 안고 있는 보안 위험을 드러냈다. 같은 해 [Speciale 등(CVPR 2019)](https://arxiv.org/abs/1903.05572)이 3D 점을 무작위 방향의 직선으로 치환한 line cloud 지도를 대안으로 내놓았으나, 2021년 [Chelani, Kahl, Sattler](https://arxiv.org/abs/2103.05086)은 그 line cloud에서조차 점 위치와 영상 내용을 대략 되살릴 수 있음을 증명했다(선을 극도로 성기게 남길 때만 세부가 가려졌다). 정확히 장소를 알아맞히면서도 원래 모습은 엿볼 수 없게 만드는 프라이버시 보존형 장면 표현은 여전히 풀어야 할 숙제다.

---

장소 인식이 신경망을 통해 "여기가 어디인가"라는 전역 기억을 다루기 시작했다면, 거의 같은 시기 딥러닝이 정조준한 또 하나의 고전 난제는 "단 한 장의 사진만 보고 각 픽셀이 얼마나 멀리 떨어져 있는가"를 읽어 내는 일이었다. 다음 장에서는 단일 이미지에서 깊이를 예측하며 단안 SLAM의 고질적 제약을 흔들어 놓은 단안 깊이 추정의 역사를 따라간다(→ 14장).