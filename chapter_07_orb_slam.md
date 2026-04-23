# Ch.7 — Feature-based 계보: ORB-SLAM 삼부작

Ch.6의 graph SLAM 혁명은 포즈 그래프 최적화를 SLAM의 표준 언어로 굳혔다. Kümmerle(2011)의 g²o와 Kaess(2012)의 iSAM2는 대규모 지도에서 반복 최적화를 실현했고, loop closure의 비용을 현실적인 수준으로 낮췄다. 최적화 이론이 완성을 향해 달려가던 그 흐름이 3부의 출발선이다. 2부가 "어떻게 오차를 줄이는가"를 질문했다면, 3부는 그 질문에 이미 답이 나와 있다는 전제 위에서 시작한다. 남은 과제는 프론트엔드였다. 어떤 특징을 어떻게 뽑아 추적할 것인가.

Klein과 Murray가 2007년 PTAM으로 tracking과 mapping을 두 스레드로 분리했을 때, 그것은 실험실 데모였다. 넓이가 아니라 깊이가 입증된 아이디어였고, 소규모 실내 장면 이상에서는 무너졌다. Raúl Mur-Artal이 2015년 Zaragoza대학에서 그 구조를 가져올 때, 그는 세 가지를 함께 들고 왔다. Rublee(2011)의 ORB 디스크립터, Gálvez-López(2012)의 DBoW2 visual vocabulary, 그리고 Strasdat(2011)의 Essential graph 아이디어. PTAM이 빠른 프로토타입이었다면 ORB-SLAM은 10년짜리 표준이었다.

---

## 7.1 ORB-SLAM (2015): 설계의 삼각대

[Mur-Artal, Montiel & Tardós 2015. ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671)은 IEEE Transactions on Robotics에 실린 논문이다. 제목이 단순하다. ORB feature를 쓰는 SLAM. 그러나 논문을 뜯으면 선택 하나하나가 설계 판단이다.

시스템의 뼈대는 Tracking, Local Mapping, Loop Closing 세 스레드다. PTAM도 두 스레드(Tracking과 Mapping)였다. Mur-Artal은 Loop Closing이라는 세 번째 스레드를 추가했다. Loop Closing은 DBoW2로 장소를 인식하고, Essential graph를 통해 포즈 그래프를 최적화하며, 마지막으로 전역 Bundle Adjustment를 실행한다. 이 분리 덕분에 Tracking은 지도 수정을 기다리지 않고 실시간을 유지한다.

> 🔗 **차용.** PTAM(Klein & Murray, 2007)의 Tracking–Mapping 분리가 ORB-SLAM의 Tracking–LocalMapping으로 직접 이어졌다. Mur-Artal은 논문 §3에서 이 부채를 명시했다. ORB-SLAM은 두 스레드 구조를 세 스레드로 확장하며 루프 클로저를 독립 모듈로 격리했다.

Mur-Artal이 front-end에서 ORB(Oriented FAST and Rotated BRIEF) descriptor를 고른 데는 이유가 있었다. SIFT와 SURF는 특허 문제가 있었고, BRIEF는 빠르지만 회전에 취약했다. ORB는 FAST 키포인트에 회전 불변성을 덧붙인 것으로, Rublee et al.이 2011 ICCV에서 발표했다. 계산 비용이 SIFT 대비 두 자릿수 빠르고 binary 형태라 해밍 거리로 매칭한다. CPU에서 실시간이 된다.

ORB가 scale invariance를 얻는 방식은 image pyramid다. 원 이미지를 스케일 팩터 s(ORB-SLAM에서는 1.2)로 8단계 축소해 피라미드를 만들고, 각 레벨에서 독립적으로 FAST 키포인트를 검출한다. 키포인트의 방향(orientation)은 intensity centroid로 정의한다: 패치 내 픽셀 intensity의 1차 모멘트로 중심을 구하고, 이 방향각 θ를 BRIEF 비트 비교 쌍에 적용해 회전 불변 descriptor를 만든다. 결과는 256-bit binary vector다. 두 descriptor 사이의 유사도는 XOR 후 popcount, 즉 해밍 거리로 계산한다.

> 🔗 **차용.** [Rublee et al. 2011. ORB](https://doi.org/10.1109/ICCV.2011.6126544)의 descriptor가 시스템 이름 자체가 되었다. ORB는 Zaragoza 팀이 설계한 것이 아니다. Mur-Artal은 있는 도구를 가져다가 파이프라인을 조립했다. front-end의 선택이 10년간 시스템의 이름으로 불린 경우다.

키프레임 선택 정책이 PTAM과 다르다. PTAM은 키프레임을 공격적으로 추가했다. ORB-SLAM은 covisibility graph 기반으로 중복을 제거한다. **Covisibility graph**는 키프레임 사이의 공유 landmark 수를 엣지 가중치로 삼는 그래프다. 공유 landmark가 15개 이상인 키프레임 쌍이 연결된다. Local Mapping은 이 그래프를 이용해 local window를 선택하고, 그 안에서만 Bundle Adjustment를 수행한다.

KITTI 시퀀스 00(전체 4.5 km 루프)에서 ORB-SLAM은 1.2% translation drift를 기록했다. 당시 비교 대상이었던 PTAM은 루프를 닫지 못한다. scale 자체가 없다. ORB-SLAM이 같은 시퀀스에서 루프를 닫고 drift를 흡수한 것은 Essential graph와 DBoW2 덕분이다.

**Essential graph**는 covisibility graph의 부분 그래프다. 공유 landmark가 100개 이상인 엣지, spanning tree, 루프 클로저 엣지만 남긴다. 루프가 감지되면 이 그래프 전체를 포즈 그래프로 최적화한다. 수천 개의 키프레임이 있어도 Essential graph의 엣지는 sparse하다. 최적화가 수 초 안에 끝난다.

> 🔗 **차용.** Essential graph의 아이디어는 [Strasdat et al. 2011. Double Window Optimisation](https://doi.org/10.1109/ICCV.2011.6126517)의 계층적 최적화 구조에서 왔다. Strasdat는 local window와 global window를 분리해 최적화 비용을 낮췄다. Mur-Artal은 이를 Essential graph라는 sparse 포즈 그래프로 일반화했다.

루프 클로저의 장소 인식은 DBoW2가 담당한다. [Gálvez-López & Tardós 2012. DBoW2](https://doi.org/10.1109/TRO.2012.2197158)는 binary descriptor용 vocabulary tree다. ORB feature를 hierarchical k-means로 클러스터링해 트리 구조의 vocabulary를 만든다. 트리 분기 수 k와 깊이 L이 고정되면 leaf 노드(word) 수는 k^L이 된다. ORB-SLAM에서는 k=10, L=6으로 약 10^6 개의 word가 쓰인다. 각 word에는 TF-IDF(Term Frequency–Inverse Document Frequency) 가중치가 붙는다: 특정 word가 전체 키프레임 데이터베이스에 자주 등장할수록 낮은 IDF 가중치를 받아 discriminative한 word가 더 큰 영향력을 갖는다. 키프레임은 이 가중 BoW 벡터로 표현되고, inverted index에 저장된다. 새 프레임이 들어오면 vocabulary tree를 내려가 word를 결정하는 데 O(log(k^L))=O(L)이 걸리고, inverted index로 후보 키프레임을 바로 조회한다. 전체 지도를 순회하지 않는다.

Tracking 스레드는 매 프레임마다 현재 포즈를 추정한다. 이전 프레임 포즈를 초기값으로 feature matching을 수행한 뒤, **EPnP**(Efficient Perspective-n-Point)로 포즈 $\mathbf{T}_{cw} \in SE(3)$를 구한다. EPnP는 3D–2D correspondence $\{(\mathbf{X}_i, \mathbf{u}_i)\}$에서 reprojection error를 최소화한다:

$$\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \left\| \mathbf{u}_i - \pi(\mathbf{T}\mathbf{X}_i) \right\|^2$$

여기서 $\pi$는 카메라 투영 함수, $\mathbf{X}_i$는 맵 포인트의 월드 좌표, $\mathbf{u}_i$는 이미지 좌표다. 초기 추정 후 RANSAC으로 outlier를 제거하고, inlier만으로 g²o 기반 local bundle adjustment를 수행해 현재 키프레임과 covisibility graph 이웃 키프레임들의 포즈 및 맵 포인트를 동시에 최적화한다.

---

## 7.2 ORB-SLAM2 (2017): Scale Ambiguity를 넘어서

ORB-SLAM(2015)는 mono-only였다. 카메라 하나만으로는 scale을 알 수 없다. "이 복도가 10m인가 100m인가"를 이미지 픽셀에서 읽어낼 방법이 없다. Mur-Artal과 Tardós가 2016년에 작업을 시작한 것은 이 문제 때문이었다.

[Mur-Artal & Tardós 2017. ORB-SLAM2](https://doi.org/10.1109/TRO.2017.2705103)는 stereo와 RGB-D를 추가해 이 문제를 해결한다. stereo는 기선(baseline)을 알므로 depth를 직접 삼각측량한다. RGB-D는 depth 센서가 측정값을 준다. 두 경우 모두 scale이 생긴다.

구조는 mono와 동일한 세 스레드다. front-end만 센서 종류에 따라 달라진다. stereo는 rectified 이미지 쌍에서 ORB를 추출하고 좌우 매칭으로 depth를 구한다. 기선 근방의 특징점은 **stereo landmark**로, 멀리 있어 depth 추정이 불가능한 것은 **monocular landmark**로 분류한다. 이 혼합 방식이 stereo와 mono의 장점을 동시에 활용한다.

**Stereo 초기화**는 mono와 달리 첫 프레임부터 즉각 수행된다. mono 초기화는 두 프레임 사이의 Essential Matrix나 Homography를 통해 맵을 구성하고 scale 모호성이 남는다. Stereo는 첫 키프레임에서 좌우 이미지 간 수평 시차(disparity) $d$와 기선 $b$, 초점 거리 $f$로 depth를 계산한다:

$$Z = \frac{b \cdot f}{d}$$

depth $Z$가 임계값 $Z_{\max}=40b$ 이하인 특징점은 즉시 3D 맵 포인트로 등록된다. RGB-D 초기화도 동일한 원리다. depth 이미지에서 픽셀 $(u, v)$의 depth값 $Z$를 읽고, 역투영(back-projection)으로 3D 좌표를 얻는다. 두 경우 모두 scale이 고정되므로 첫 프레임 직후 Local BA를 바로 실행할 수 있다.

EuRoC MAV(Micro Aerial Vehicle) 데이터셋 Machine Hall 01 시퀀스에서 ORB-SLAM2(stereo)는 절대 translation 오차 0.035 m를 기록했다. 비교 시점의 VINS-Mono(0.052 m)보다 낮았다. KITTI 오도메트리에서도 ORB-SLAM2가 당시 published 방법 중 상위권이었다.

2017년 5월 논문이 IEEE TRO에 실리던 날, Mur-Artal과 Tardós는 GitHub에 소스를 함께 올렸다. Zaragoza 팀 둘이서 mono·stereo·RGB-D 세 모드를 단일 코드베이스로 공개한 것이다. 이후 GitHub star는 수천을 넘었고, ROS 래퍼가 커뮤니티에서 만들어졌다.

---

## 7.3 ORB-SLAM3 (2021): Atlas와 Visual-Inertial

2021년 IEEE Transactions on Robotics에 실린 [Campos et al. 2021. ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644)는 저자 목록이 달라진다. Mur-Artal이 아니라 Carlos Campos가 1저자다. Mur-Artal은 Tardós와 함께 공저자로 이름을 올렸다. Campos는 Zaragoza 대학에서 Tardós 지도 아래 박사 과정을 밟았다. 계보가 한 세대 내려온 것이다.

ORB-SLAM3의 핵심 확장은 두 가지다. **Atlas**(멀티맵)와 **Visual-Inertial** 모드.

Atlas는 여러 개의 분리된 지도를 동시에 유지하는 구조다. 추적이 실패하면 기존 지도를 닫고 새 지도를 시작하며, 나중에 같은 장소를 재방문했을 때 두 지도를 병합한다. ORB-SLAM과 ORB-SLAM2에서 추적 실패는 치명적이었다. 한번 잃으면 처음부터 다시 해야 했다. Campos는 이 점을 박사 과정 내내 가장 자주 겪은 한계로 지목했고, Atlas가 그 답이었다. ORB-SLAM3는 실패 후 재초기화하고 이전 지도를 기억한다.

Visual-Inertial(VI) 모드는 IMU 데이터를 통합한다. Campos는 Forster et al.이 2016년 IJRR에 정리한 [IMU Preintegration on Manifold](https://doi.org/10.1177/0278364916652421) 방식을 그대로 가져왔다. IMU는 빠른 모션에서 Visual SLAM이 잃기 쉬운 추적을 보완한다. VI-SLAM은 단안 카메라의 scale ambiguity도 해결한다. IMU의 가속도계 측정이 중력 방향과 함께 절대 scale을 제공한다.

Preintegration의 핵심은 키프레임 $i$와 $j$ 사이의 IMU 측정을 한 번만 적분해 놓는 것이다. 가속도계·자이로스코프 측정값을 $\tilde{\mathbf{a}}_t = \mathbf{a}_t + \mathbf{b}_a + \mathbf{n}_a$, $\tilde{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t + \mathbf{b}_g + \mathbf{n}_g$로 모델링하면(bias $\mathbf{b}$, noise $\mathbf{n}$), 두 키프레임 사이의 상대 회전·속도·위치 변화량을 다음과 같이 preintegration한다:

$$\Delta\mathbf{R}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp}\bigl((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\bigr)$$
$$\Delta\mathbf{v}_{ij} = \sum_{k=i}^{j-1} \Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t$$
$$\Delta\mathbf{p}_{ij} = \sum_{k=i}^{j-1}\!\left[\Delta\mathbf{v}_{ik}\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\right]$$

여기서 $\mathrm{Exp}(\cdot)$는 $\mathfrak{so}(3)$의 지수 사상이다. bias가 BA 중 갱신되면 전체 재적분 없이 1차 선형 근사로 보정한다. ORB-SLAM3는 이 preintegrated 항($\Delta\mathbf{R}$, $\Delta\mathbf{v}$, $\Delta\mathbf{p}$)을 factor graph의 inertial edge로 추가해 visual reprojection residual과 함께 최적화한다.

> 🔗 **차용.** Campos는 [Forster et al. 2016. IMU Preintegration on Manifold](https://doi.org/10.1177/0278364916652421)의 preintegration 공식을 ORB-SLAM3 Inertial 통합의 핵심으로 가져왔다. Forster의 수식은 연속 IMU 측정을 bias 보정과 함께 SE(3) 위에서 적분하는 방법을 제공한다. ORB-SLAM3는 이 공식을 factor graph 최적화에 연결했다.

EuRoC MAV 전체 11개 시퀀스 평균 RMSE ATE(절대 궤적 오차)에서 ORB-SLAM3(mono-inertial)는 0.016 m였다. 같은 조건에서 VINS-Mono는 0.043 m, Kimera는 0.022 m였다.

VI 모드와 Atlas가 결합하면 무인기나 핸드헬드 장치가 조명이 달라지거나 추적을 잃어도 이전 지도로 돌아올 수 있다. 버전 번호를 바꾼 것이 아니라 시스템의 성격이 달라진 것이다.

---

## 7.4 왜 2020년대에도 Baseline인가

2023년, 학회 논문들은 여전히 ORB-SLAM3를 비교 대상으로 표에 넣는다. 새 방법이 발표될 때 "ORB-SLAM3보다 얼마나 낫냐"가 기준선이다. 알고리즘이 2021년에 멈췄는데도 기준이 되는 이유가 있다.

강건성이 먼저다. ORB feature는 조명 변화에 어느 정도 내성이 있고, binary descriptor라 계산이 빠르며, 많은 수를 실시간으로 뽑아 추적 실패를 줄인다. learned feature가 특정 데이터셋에서는 더 정확하지만, 새로운 환경에서 무너지는 경우가 있다. ORB의 동작은 예측 가능하다.

재현성도 있다. 코드가 공개되어 있고, ROS 통합이 잘 되어 있으며, 수천 개의 실사용 사례가 문서화되어 있다. 실험실에서 새 시스템을 평가할 때 ORB-SLAM3를 돌려보는 것이 첫 번째 단계가 된 지 오래다. mono·stereo·RGB-D·IMU를 단일 코드베이스가 지원하기 때문에 "우리 방법 vs ORB-SLAM3(stereo)" 혹은 "우리 방법 vs ORB-SLAM3(mono-inertial)"을 나란히 비교할 수 있다. 하나의 baseline이 여러 설정을 커버한다.

마지막으로, learned alternative가 일관되게 능가하지 못한다. DROID-SLAM(Teed & Deng, 2021)은 여러 시퀀스에서 ORB-SLAM3를 이긴다. 그러나 GPU 메모리를 수 GB 사용하고, 초기화 시간이 길며, 실시간이 아닌 경우가 있어 임베디드 플랫폼에서는 돌아가지 않는다. ORB-SLAM3는 Raspberry Pi 4에서도 작동한다.

---

## 📜 예언 vs 실제

> 📜 **예언 vs 실제.** Mur-Artal은 2015년 ORB-SLAM 논문에서 RGB-D 및 stereo 카메라로의 확장, IMU 통합을 다음 목표로 꼽았다. 2017년 ORB-SLAM2가 stereo와 RGB-D를 추가했고, 2021년 ORB-SLAM3가 IMU를 통합했다. 같은 팀이 자기 Future Work를 순서대로 직접 이행하는 데 6년이 걸렸다. `[적중]`

> 📜 **예언 vs 실제.** Campos et al.은 2021년 ORB-SLAM3 논문에서 learned descriptor와 deep SLAM과의 통합을 다음 방향으로 제시했다. 2023-2025년 사이 SuperPoint와 LightGlue를 front-end로 사용하는 ORB-SLAM3 변형들이 연구되었다. 그러나 ORB-SLAM3 공식 GitHub 저장소의 main branch는 2026년 현재도 전통 ORB descriptor를 유지한다. 학술 실험에서는 통합이 이뤄졌지만, 공식 시스템은 바뀌지 않았다. `[진행형]`

---

## 🧭 아직 열린 것

Long-term map reuse. Atlas가 멀티맵 유지를 가능하게 했지만, 조명이 크게 달라진 환경에서 지도 병합은 여전히 실패한다. 아침에 만든 지도와 저녁에 재방문할 때의 장소를 같은 곳으로 인식하는 것이 목표인데, 외관 변화가 크면 DBoW2의 place recognition이 놓친다. seasonal change가 있는 outdoor 환경에서 장기 자율주행이 필요한 연구그룹들이 이 문제를 붙잡고 있다. 2024년 기준 완전한 해답은 없다.

Pure vision baseline의 자리. learned feature 기반 시스템들이 표준 benchmark에서 ORB-SLAM3를 이기기 시작했다. SuperPoint + SuperGlue 조합, LightGlue, 그리고 DINOv2 기반 feature들이 특정 시퀀스에서 더 낮은 오차를 보인다. 그러나 일반화 가능성은 다른 문제다. training distribution 밖의 환경에서 learned feature가 전통 ORB보다 나쁜 결과를 내는 경우가 보고된다. "일관되게 능가한다"는 주장을 하려면 아직 더 넓은 실험이 필요하다.

대규모 outdoor에서의 drift. ORB-SLAM3는 도심 주행이나 수 km 이상의 경로에서 LiDAR SLAM 대비 여전히 열세다. GPS-denied 환경에서 urban-scale localization을 순수 카메라로 달성하는 것은 2026년 기준 미해결이다. 시각 조건의 변화, 동적 객체, 텍스처 없는 구간이 복합되면 drift가 누적된다. LiDAR 측량 정밀도와의 격차는 좁혀지고 있으나 닫히지는 않았다.

---

ORB-SLAM 삼부작이 feature-based 계보의 표준을 세운 같은 시기, Newcombe와 Engel은 정반대의 선택을 하고 있었다. 특징점을 뽑지 않고 이미지 전체의 밝기 정보를 직접 쓰겠다는 것이었다. 두 계보는 2010년대 내내 나란히 발전했고, 서로를 비교 대상으로 삼으면서 각자의 한계를 드러냈다. ORB-SLAM3가 2021년 EuRoC에서 0.016 m를 기록하는 동안, DSO는 TUM 복도에서 ORB-SLAM2를 눌렀다. 같은 시간표, 다른 출발점이었다.
