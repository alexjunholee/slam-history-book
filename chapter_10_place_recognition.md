# Ch.10 — Place Recognition의 평행선: FAB-MAP에서 NetVLAD까지, 그리고 AnyLoc까지

2003년 Davison이 웹캠 한 대로 실시간 3D 추적을 증명하던 무렵, Oxford 모바일 로보틱스 그룹의 Mark Cummins와 Paul Newman은 다른 질문을 붙잡고 있었다. "로봇이 이전에 지나간 장소를 어떻게 알아보는가?" VO(visual odometry)가 누적 drift에 시달리는 한, 이 질문에 답하지 못하면 어떤 SLAM 시스템도 루프를 닫을 수 없었다. Place recognition은 Visual SLAM의 나머지 구성 요소들과 평행하게, 그러나 독자적인 계보로 2000년대 내내 발전했다. FAB-MAP은 Josef Sivic의 BoW 아이디어를 로봇 공간으로 이식했고, DBoW2는 그것을 실용화했으며, NetVLAD는 학습으로 끊어냈다. 2023년 AnyLoc는 foundation model의 feature를 그대로 가져왔다.

Ch.7이 ORB-SLAM 삼부작으로 feature-based 계보를 완성하고, Ch.8이 DSO까지 direct 계보를 추적하고, Ch.9가 KinectFusion에서 BundleFusion까지 dense mapping의 궤도를 따라가는 동안, place recognition은 그 어느 계보와도 다른 선을 그었다. Tracking도 아니고 mapping도 아닌, 어느 쪽에서도 파생되지 않은 독립 문제였다. 그럼에도 세 계보 모두 루프 클로저 없이는 불완전했고, 그 루프 클로저의 "어디서 봤는가" 판단을 place recognition이 공급했다.

---

## 10.1 BoW 이전: 장소를 어떻게 묻는가

GPS가 없는 실내, 터널, 도심 협곡에서 로봇이 루프를 닫으려면 현재 관측과 과거 관측 사이의 유사도를 수천 장의 후보 이미지 중에서 빠르게 찾아야 한다. 픽셀 단위 비교는 선형 탐색이어서 O(N)이고, 이미지 수가 수만 장을 넘으면 실시간은 불가능하다.

2000년대 초 컴퓨터 비전에서 이 문제를 먼저 건드린 것은 Sivic과 Zisserman이었다. 2003년 ICCV에서 발표된 "Video Google"은 문서 검색의 TF-IDF를 이미지에 적용했다. SIFT 기술자를 k-means로 군집화해 "visual word"를 만들고, 이미지를 그 단어들의 빈도 벡터로 표현했다. 검색은 inverted index를 통해 O(1)에 가까워졌다. place recognition 연구자들은 이 아이디어를 곧바로 받아들였다.

---

## 10.2 FAB-MAP — 확률적 BoW와 Chow-Liu 트리 (2008)

Mark Cummins와 Paul Newman은 Oxford 모바일 로보틱스 그룹에서 2008년 [Cummins & Newman. FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance](https://doi.org/10.1177/0278364908090961)를 발표했다.

FAB-MAP(**Fast Appearance-Based Mapping**)의 핵심 질문은 "이 장면은 데이터베이스에 있는 장소인가, 아니면 전혀 새로운 곳인가?"다. 단순 유사도 점수로는 이 판단을 내릴 수 없다. 비슷해 보이는 복도가 수십 개라면 가장 높은 유사도가 정답을 보장하지 않는다.

Cummins와 Newman은 이를 베이즈 추론 문제로 구성했다. 관측 $z_t$(visual word의 발생 여부 집합)가 주어졌을 때, 현재 위치가 데이터베이스의 각 장소 $\ell_i$일 확률을 계산한다:

$$P(\ell_i \mid z_t) \propto P(z_t \mid \ell_i) P(\ell_i)$$

문제는 $P(z_t \mid \ell_i)$다. visual word들이 독립이라고 가정하면 naïve Bayes가 되지만, 실제로 visual word들은 상관된다. "문"이라는 word가 등장하면 "문손잡이"라는 word도 같이 등장하기 쉽다. 독립 가정은 확률 값을 왜곡한다.

FAB-MAP은 **Chow-Liu tree**를 사용해 이 상관을 모델링했다. Chow-Liu tree는 word 간의 pairwise mutual information을 최대화하는 트리 구조 그래픽 모델이다. 두 word $e_i, e_j$ 사이의 mutual information은

$$I(e_i; e_j) = \sum_{e_i, e_j} P(e_i, e_j) \log \frac{P(e_i, e_j)}{P(e_i)P(e_j)}$$

로 정의되고, Chow-Liu 알고리즘은 이를 엣지 가중치로 삼아 최대 스패닝 트리를 구성한다. 이 트리로 joint likelihood를 분해하면

$$P(z_t \mid \ell_i) = \prod_k P(z_t^k \mid z_t^{\text{pa}(k)}, \ell_i)$$

가 된다. 여기서 $z_t^k \in \{0,1\}$은 $k$번째 word의 발생 여부이고, $\text{pa}(k)$는 트리에서 $k$의 부모 노드다. 나이브 베이즈(독립 가정) 대비 word 간 공동 발생 패턴을 반영하므로, 복도처럼 시각적으로 유사한 장소들에서 false positive를 낮출 수 있다. 학습 단계에서 대규모 이미지 집합으로 vocabulary와 tree를 함께 훈련한다.

또한 FAB-MAP은 현재 위치가 데이터베이스에 없는 새 장소일 가능성을 명시적으로 다룬다. "new place" 가설을 넣자 false positive가 줄었다. Loop closure에서 false positive는 catastrophic failure로 이어진다. 실용적으로 핵심이었다.

> 🔗 **차용.** FAB-MAP의 visual word 방식은 Sivic & Zisserman의 "Video Google"(2003)에서 직접 이식되었다. 문서 검색의 inverted index 논리를 로봇의 장소 기억에 적용한 것이다.

2011년 Cummins와 Newman은 FAB-MAP 2.0을 발표했다. 처리 가능한 지도 규모를 1,000 km 수준으로 확장한 것이 목표였다. 실험적으로 도시 규모 데이터셋에서 작동함을 보였다.

---

## 10.3 DBoW2 — 이진 descriptor와 실용화 (2012)

FAB-MAP은 SIFT처럼 부동소수점 descriptor를 기반으로 했다. 2012년 무렵 SLAM 커뮤니티는 더 빠른 binary descriptor, 특히 BRIEF·ORB·BRISK 쪽으로 이동하고 있었다. SIFT vocabulary를 그대로 쓰는 것은 연산 비용이 문제였다.

Dorian Gálvez-López와 Juan D. Tardós(Universidad de Zaragoza)는 2012년 [Gálvez-López & Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences](https://doi.org/10.1109/TRO.2012.2197158)를 발표했다. **DBoW2**는 binary descriptor를 사용하는 vocabulary tree로, Hamming distance 기반 비교로 SIFT보다 수십 배 빠른 word 배정이 가능했다.

DBoW2의 구조는 계층적 k-means로 만든 vocabulary tree다. 이미지를 표현하는 BoW 벡터는 TF-IDF 가중치가 부여된 binary word 빈도 벡터다. $k$분기 $d$깊이 트리의 각 리프 노드 $w_i$에 TF-IDF 가중치

$$\eta_i = \frac{n_i}{n} \cdot \log \frac{N}{N_i}$$

를 부여한다. 여기서 $n_i$는 해당 이미지에서의 word 빈도, $n$은 총 word 수, $N$은 데이터베이스 이미지 수, $N_i$는 $w_i$를 포함한 이미지 수다. 두 이미지 $a$, $b$의 유사도는 L1-norm

$$s(\mathbf{v}_a, \mathbf{v}_b) = 1 - \frac{1}{2} \left\| \frac{\mathbf{v}_a}{|\mathbf{v}_a|} - \frac{\mathbf{v}_b}{|\mathbf{v}_b|} \right\|_1$$

으로 계산한다. 조회는 inverted index를 통해 O(log N)에 수행된다.

> 🔗 **차용.** DBoW2의 vocabulary tree 개념은 Nistér & Stewénius의 2006년 "Scalable Recognition with a Vocabulary Tree"(CVPR)에서 계보를 잇는다. DBoW2는 그 구조를 binary descriptor 세계로 이식하고, 가중치 체계를 SLAM에 맞게 조정했다.

DBoW2가 중요한 건 알고리즘보다 배포다. 오픈소스로 공개된 이 라이브러리는 ORB-SLAM(2015)의 loop closure 모듈로 채택되었고, ORB-SLAM2·ORB-SLAM3까지 같은 DBoW2를 썼다. 2015-2020년대 중반, SLAM 커뮤니티의 place recognition은 사실상 DBoW2가 담당했다.

Gálvez-López와 Tardós의 파트너십 역시 주목할 만하다. Tardós는 이후 Mur-Artal, Campos와 함께 ORB-SLAM 삼부작을 이끈 인물이다. DBoW2는 그 프로젝트의 place recognition 계층을 미리 준비한 셈이었다.

---

## 10.4 NetVLAD — 학습으로의 전환 (2016)

BoW 계열은 한 가지 근본 한계가 있었다. vocabulary는 특정 descriptor와 특정 환경에서 훈련된 것이었다. 조명이 바뀌거나 계절이 달라지거나 시점이 크게 달라지면 visual word의 분포가 달라지고, 미리 훈련된 vocabulary는 부정합을 일으켰다.

Relja Arandjelović, Petr Gronat, Akihiko Torii, Tomáš Pajdla, Josef Sivic는 2016년 CVPR에서 [NetVLAD: CNN Architecture for Weakly Supervised Place Recognition](https://doi.org/10.1109/CVPR.2016.572)을 발표했다. 저자 중 Sivic은 2003년 "Video Google"의 그 Sivic이다. 2003년 ICCV에서 BoW를 이미지 검색에 도입한 사람이, 13년 뒤 그 방식의 한계를 넘는 논문에 공동저자로 이름을 올렸다.

NetVLAD의 아이디어는 **VLAD(Vector of Locally Aggregated Descriptors)** aggregation을 미분 가능하게 만드는 것이었다.

VLAD는 2010년 Jégou et al.이 제안한 aggregation 방식으로, 각 local descriptor가 가장 가까운 cluster center(visual word)에 "잔차"로 얼마나 기여하는지를 누적해 이미지 전체를 표현한다. cluster center $k$에 대한 VLAD 부분 벡터는

$$\mathbf{V}(k) = \sum_{\mathbf{x}_i : \text{NN}(\mathbf{x}_i)=k} (\mathbf{x}_i - \boldsymbol{\mu}_k)$$

이고, 전체 VLAD 벡터 $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$는 이를 모든 cluster에 대해 연결(concatenate)한 뒤 L2-normalize한 것이다. $K$ clusters, $D$차원 descriptor라면 최종 벡터는 $KD$차원이다. VLAD 벡터는 BoW의 이진 할당보다 훨씬 풍부한 정보를 담는다.

> 🔗 **차용.** NetVLAD의 aggregation 설계는 Jégou et al.의 "Aggregating Local Descriptors into a Compact Image Representation"(CVPR 2010)에서 VLAD를 직접 계승했다. NetVLAD가 한 것은 VLAD의 hard assignment를 soft assignment로 바꾸고 전체 파이프라인을 end-to-end로 학습 가능하게 만든 것이다.

NetVLAD layer는 기존 VLAD의 nearest-neighbor 할당을 softmax로 완화한다:

$$\bar{a}_k(\mathbf{x}_i) = \frac{e^{\mathbf{w}_k^\top \mathbf{x}_i + b_k}}{\sum_{k'} e^{\mathbf{w}_{k'}^\top \mathbf{x}_i + b_{k'}}}$$

여기서 $\mathbf{x}_i$는 CNN에서 추출한 local feature, $\mathbf{w}_k$와 $b_k$는 학습 가능한 파라미터다. 이 soft 할당으로 NetVLAD 벡터를 누적하면

$$\mathbf{V}(k) = \sum_i \bar{a}_k(\mathbf{x}_i)\,(\mathbf{x}_i - \boldsymbol{\mu}_k)$$

이고, 전체 벡터 $\mathbf{V} = [\mathbf{V}(1)^\top, \ldots, \mathbf{V}(K)^\top]^\top$를 intra-normalization(각 부분 벡터 L2 정규화) 후 전체를 다시 L2-normalize하면 최종 VPR descriptor가 된다. hard assignment VLAD와 달리 gradient가 역전파되므로 CNN backbone과 함께 end-to-end 학습이 가능하다.

학습 방식도 달랐다. 저자들은 Google Street View Time Machine 데이터를 활용해 같은 장소의 다른 시점 이미지 쌍을 양성 예, 다른 장소를 음성 예로 삼는 weakly supervised triplet loss를 사용했다. GPS 위치만 있으면 레이블 없이 학습할 수 있었다.

Pittsburgh 250k, Tokyo 24/7 벤치마크에서 NetVLAD는 DBoW 계열과 이전 VLAD 기반 방법들을 큰 차이로 앞섰다. 조명·계절 조건 변화에 걸쳐 훨씬 강건했고, 시점 차에도 내성이 있었다. 그러나 실용 SLAM 파이프라인에 NetVLAD가 바로 통합되지는 않았다. 추론 속도와 메모리 요구가 DBoW2보다 무거웠고, 이미 ORB-SLAM 생태계가 DBoW2에 맞춰 구축되어 있었기 때문이다.

---

## 10.5 Patch-NetVLAD, MixVPR, AnyLoc — 일반화를 향한 시도 (2020-2023)

NetVLAD 이후 VPR(Visual Place Recognition) 연구는 일반화 성능 개선으로 흩어졌다.

2021년 Hausler et al.은 Patch-NetVLAD를 내놓았다. Global descriptor 하나로 장소를 판단하는 NetVLAD 대신, 이미지를 패치로 분할해 각 패치의 NetVLAD 표현을 공간적으로 결합하는 방식이다. Tokyo 24/7에서 NetVLAD 대비 Recall@1을 약 10% 포인트 올렸다. 패치 단위 처리로 추론 비용도 함께 늘었다.

2023년 Ali-bey et al.의 MixVPR는 Transformer-style feature mixing으로 global feature를 생성했다. 경량화와 성능 사이의 균형이 목표였다. 이 시기 VPR 논문들은 공통으로 Mapillary Street Level Sequences(MSLS)와 Nordland 같은 계절 변화 데이터셋을 벤치마크로 삼았다. 극단적 조명·계절 조건이 공통의 장벽으로 떠올랐다.

2023년 Keetha et al.의 [AnyLoc: Towards Universal Visual Place Recognition](https://arxiv.org/abs/2308.00688)은 다른 방향을 택했다. DINOv2 기반 self-supervised feature를 fine-tuning 없이 그대로 place recognition에 쓰는 것이다.

> 🔗 **차용.** AnyLoc의 feature 추출은 Oquab et al.의 DINOv2(Meta AI, 2023)에서 사전 학습된 ViT 표현을 가져온다. AnyLoc은 그 위에 VLAD aggregation을 얹었다. FAB-MAP에서 시작한 BoW-VLAD 계보가 foundation model 시대에 다시 합류한 형태다.

DINOv2는 대규모 인터넷 이미지로 학습된 Vision Transformer(ViT)다. 특정 도시, 특정 계절, 특정 카메라에 편향되지 않은 범용 feature를 생성한다. AnyLoc에서 주목한 것은 DINOv2의 **facet** 개념이다. ViT의 각 attention head는 query(Q), key(K), value(V) 행렬과 최종 token(patch feature)을 출력한다. Keetha et al.은 이 네 종류의 facet 중 value(V) facet이 place recognition에 가장 의미론적으로 안정된 표현을 제공함을 실험으로 확인했다. Q·K facet은 구조·기하 정보에, V facet은 의미론(semantics)에 더 집중되는 경향이 있어, 계절·조명에 걸친 일관된 장소 표현에 유리하다. Keetha et al.은 이 V facet 표현을 VLAD aggregation에 연결하면 세계 각지, 실내외, 지하, 항공 뷰 등 매우 다양한 환경에서 단일 모델이 동작함을 보였다. Pittsburgh, Tokyo, 실내 공장, 지하 주차장, 도서관 등 7개 이상의 환경에서 single-model이 이전 specialized 방법들과 경쟁하거나 앞섰다.

---

## 10.6 평행선의 수렴 (2024-2025)

Place recognition 연구는 2000년대 초부터 SLAM의 나머지 구성 요소와 평행하게 달려왔다. ORB-SLAM이 DBoW2를 내장했지만 place recognition 모듈은 mapping·tracking으로부터 격리된 블랙박스였다. 입력은 이미지, 출력은 루프 후보 ID였다.

2024-2025년 들어 이 경계가 흐려지기 시작했다. Berton et al.의 EigenPlaces(2023)와 Izquierdo et al.의 Salad(2023)는 place recognition descriptor를 metric localization에 직접 끌어들이는 방향을 탐구했다. "어디서 본 장소"를 찾는 데 그치지 않고 6-DoF pose를 place recognition 표현 자체에서 바로 뽑으려 했다.

Sphinx(2024)는 Gaussian map 표현과 place recognition을 결합했다. 3DGS(3D Gaussian Splatting)가 지도 표현으로 올라온 흐름과 맞물린 방향이었다.

> 📜 **예언 vs 실제.** Cummins와 Newman은 2011년 FAB-MAP 2.0 §6에서 "1,000 km 규모 place recognition이 목표"라고 적었다. Oxford 캠퍼스와 도심 일부를 달리며 얻은 데이터로 연구를 시작했으니, 1,000 km는 당시 기준으로 두 자릿수 배율의 도약이었다. 2015년경 DBoW2와 대형 vocabulary를 사용한 도시 규모 루프 클로저 실험들이 이 규모를 달성했다. 규모 문제는 예언대로 풀렸다. 그러나 Cummins와 Newman이 남긴 실패 모드 — 계절·조명 변화에 취약한 vocabulary 기반 표현 — 는 deep learning이 가져다준 다른 도구로 넘어섰다. `[기술변화]`

> 📜 **예언 vs 실제.** Arandjelović et al.은 2016년 NetVLAD §6에서 "end-to-end VPR의 일반화는 아직 열린 도전"이라 했다. 이후 7년간 수십 편의 VPR 논문들이 외관 조건 일반화를 목표로 쏟아졌다. 2023년 AnyLoc은 fine-tuning 없는 foundation model feature로 다환경 단일 모델의 가능성을 보였다. 완전한 해결은 아니다. "불가능"에서 "다루기 가능"으로 옮겨간 것에 가깝다. `[진행형]`

---

## 10.7 🧭 아직 열린 것

**계절·조명 극변.** Nordland(노르웨이 철도, 여름-겨울)와 Oxford RobotCar(1년치 계절 변화) 데이터셋에서 10년 넘게 같은 장벽이 보고된다. DINOv2 기반 방법들이 격차를 줄였지만, 눈이 쌓인 겨울과 나뭇잎이 무성한 여름 사이에서 동일 장소를 99% 정확도로 인식하는 단일 모델은 아직 없다. 외관 변화가 심한 환경에서의 장소 인식은 2026년 기준으로도 열린 문제다.

**Place recognition과 metric localization의 통합.** 현재 대부분의 SLAM 파이프라인에서 place recognition은 "어디서 봤는가"만 답하고, 실제 pose 추정은 별도의 PnP 또는 descriptor matching 단계가 처리한다. 두 과정을 하나의 표현으로 통합하려는 시도들이 2023-2025년에 등장했으나, 실용적 배포 수준의 정밀도와 속도를 동시에 달성한 방법은 아직 없다.

*인식 가능한 장소 표현의 프라이버시.* VPR 시스템이 저장하는 장소 표현은 복원 공격으로 원본 이미지나 3D 구조를 되살리는 데 쓰일 수 있다. 상업 로봇이 가정·병원·사무실 실내를 매핑할 때 이 문제는 현실이 된다. 성능 저하 없이 프라이버시를 보장하는 장소 표현 방식은 아직 없다.

---

3부(성숙기)의 세 계보는 이렇게 막을 내린다. ORB-SLAM은 feature-based 파이프라인을 표준화했고, DSO는 그 반대편에서 photometric 이론을 완성했으며, KinectFusion 계열은 dense mapping의 가능성과 한계를 함께 드러냈다. Place recognition은 이 세 계보와 달리 SLAM 내부에서 자란 것이 아니었다. 컴퓨터 비전의 이미지 검색 문제에서 자라났고, SLAM이 루프 클로저를 필요로 했을 때 공급자 자리를 맡았다. 그 거리가 오히려 유리했다. deep learning 물결이 닥쳤을 때, place recognition은 기존 SLAM 파이프라인보다 빠르게 새 도구를 흡수했다.

2023년 AnyLoc이 등장했을 때 Sivic의 이름은 감사의 글이 아니라 참고문헌에 있었다. 2003년 BoW를 이미지 검색에 꽂은 사람, 2016년 NetVLAD로 그 한계를 넘은 공동저자. 그 계보의 끝에서 AnyLoc은 Sivic이 연 문을 foundation model 쪽으로 밀어 넘겼다. 3부가 끝난다.
