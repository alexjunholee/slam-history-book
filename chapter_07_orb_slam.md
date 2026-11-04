# Ch.7 — Feature 기반 SLAM: ORB-SLAM 세 편

Ch.6에서 본 graph SLAM 연구를 거치며 pose graph optimization은 SLAM의 표준 방법이 됐다. Kümmerle 등(2011)의 g2o와 Kaess 등(2012)의 iSAM2 덕분에 큰 지도도 거듭 최적화할 수 있게 됐고, loop closure 비용도 감당할 만해졌다. backend가 갖춰지자 남은 과제는 어떤 특징을 어떻게 뽑아 추적하느냐였다.

PTAM은 Klein과 Murray가 2007년에 tracking과 mapping을 두 스레드로 나눈 시스템으로, 작은 실내 장면에서만 돌았다. 2015년 Zaragoza 대학의 Raúl Mur-Artal은 이 구조에 Rublee(2011)의 ORB 서술자, Gálvez-López와 Tardós(2012)의 DBoW2 장소 인식, Strasdat의 covisibility 기반 지역 지도와 scale drift를 고려한 loop closing을 더해 ORB-SLAM을 만들었다. ORB-SLAM은 그 뒤 10년 가까이 비교 기준으로 쓰였다.

---

## 7.1 ORB-SLAM (2015)

[Mur-Artal, Montiel & Tardós 2015. ORB-SLAM](https://doi.org/10.1109/TRO.2015.2463671)은 IEEE Transactions on Robotics에 실렸다. 이름 그대로 ORB feature를 쓰는 SLAM이다. 논문은 tracking, mapping, relocalization, loop closing에 모두 같은 feature를 쓴 것을 첫 기여로 꼽았다.

시스템은 Tracking, Local Mapping, Loop Closing 세 스레드로 이루어진다. PTAM은 Tracking과 Mapping 두 스레드였고, Mur-Artal은 Loop Closing이라는 세 번째 스레드를 추가했다. Loop Closing 스레드는 DBoW2로 장소를 알아보고, Essential graph로 포즈 그래프를 최적화한다. 스레드를 이렇게 나눈 덕분에 Tracking 스레드는 지도를 고치는 동안에도 실시간으로 돈다. loop closure 뒤 전역 Bundle Adjustment를 네 번째 스레드로 돌리는 것은 ORB-SLAM2부터다.

> 🔗 **차용.** PTAM(Klein & Murray, 2007)의 Tracking–Mapping 분리가 ORB-SLAM의 Tracking–Local Mapping으로 그대로 이어졌다. 논문 서론은 PTAM의 주요 아이디어를 바탕으로 삼았다고 밝힌다.

ORB-SLAM 논문은 frontend의 feature를 고른 기준을 밝혔다. 추출에 이미지 한 장당 33 ms보다 훨씬 적게 걸려야 해서 SIFT와 SURF(각각 약 300 ms), A-KAZE(약 100 ms)를 뺐고, 장소 인식에는 회전 불변성이 필요해서 BRIEF와 LDB를 뺐다. 그렇게 고른 ORB(Oriented FAST and Rotated BRIEF)는 Rublee et al.이 2011년 ICCV에서 발표한 서술자로, FAST keypoint에 방향을 붙이고 BRIEF를 그 방향에 맞춰 회전시켰다. 원 논문은 SIFT보다 약 100배 빠르다고 보고했고, SIFT와 SURF의 라이선스 제약이 없다는 점을 또 하나의 장점으로 들었다. binary 서술자라서 Hamming 거리로 매칭하고, GPU 없이 CPU만으로도 실시간으로 돈다.

ORB는 image pyramid로 크기 변화에 대응한다. ORB-SLAM은 원본 이미지를 1.2배씩 줄여 8단계 피라미드를 만들고, 단계마다 따로 FAST keypoint를 검출한다. keypoint의 방향(orientation)은 intensity centroid로 정한다. 패치 안 픽셀 밝기의 1차 모멘트로 밝기 중심을 구하고, keypoint에서 그 중심을 향하는 방향각 θ만큼 BRIEF의 비교 쌍을 돌려 회전에 불변인 서술자를 만든다. 결과는 256비트 binary vector다. 두 서술자의 거리는 XOR한 뒤 1인 비트 수를 세는 Hamming 거리로 계산한다.

> 🔗 **차용.** [Rublee et al. 2011. ORB](https://doi.org/10.1109/ICCV.2011.6126544)의 서술자 이름이 시스템 이름이 됐다. ORB를 만든 사람은 Willow Garage의 Rublee, Rabaud, Konolige, Bradski다. Mur-Artal은 이미 있던 도구를 가져와 파이프라인을 짰다.

keyframe을 다루는 방식도 PTAM과 다르다. ORB-SLAM은 keyframe을 넉넉히 넣은 뒤, covisibility graph를 보고 중복된 keyframe을 지운다. **Covisibility graph**는 두 keyframe이 함께 보는 landmark 수를 엣지 가중치로 삼는 그래프다. 함께 보는 landmark가 15개 이상인 keyframe 쌍이 연결된다. Local Mapping은 이 그래프로 지역 window를 고르고, 그 안에서만 bundle adjustment를 돌린다.

**Essential graph**는 covisibility graph의 부분 그래프로, 함께 보는 landmark가 100개 이상인 엣지와 spanning tree, loop closure 엣지만 남긴다. 루프가 감지되면 이 그래프 전체를 포즈 그래프로 최적화한다. keyframe이 수천 개여도 Essential graph의 엣지는 적어서 최적화가 몇 초 안에 끝난다.

> 🔗 **차용.** [Strasdat et al. 2011. Double Window Optimisation](https://doi.org/10.1109/ICCV.2011.6126517)은 PTAM의 frontend를 쓰되 covisibility graph에서 뽑은 지역 지도에서만 추적했다. 안쪽 window에서는 BA를, 바깥 window에서는 포즈 그래프 최적화를 돌렸다. ORB-SLAM 논문은 covisibility로 지역 지도를 고르는 생각을 이 논문에서, 7자유도(similarity) 포즈 그래프로 loop를 닫는 생각을 Strasdat 등의 2010년 scale drift 연구에서 가져왔다고 밝힌다. Essential graph는 ORB-SLAM이 새로 정의한 희소 포즈 그래프다.

loop closure에 필요한 장소 인식은 DBoW2가 맡는다. [Gálvez-López & Tardós 2012. DBoW2](https://doi.org/10.1109/TRO.2012.2197158)는 binary 서술자로 vocabulary tree를 만든 첫 방법이다. binary 서술자를 k-medians(k-means++ seeding)로 계층적으로 군집화해 트리 모양 vocabulary를 만든다. 논문은 FAST keypoint와 BRIEF 서술자로 실험했고, ORB-SLAM은 같은 방식으로 ORB 서술자 vocabulary를 만들었다. 트리의 분기 수 $k_w$와 깊이 $L_w$가 정해지면 leaf 노드(word) 수는 $k_w^{L_w}$다. DBoW2 논문은 $k_w=10$, $L_w=6$으로 단어 100만 개짜리 vocabulary를 썼고, ORB-SLAM도 같은 크기의 vocabulary를 쓴다. word마다 TF-IDF(Term Frequency–Inverse Document Frequency) 가중치가 붙는다. keyframe 데이터베이스 전체에 자주 나오는 word일수록 IDF 가중치가 낮아서, 장소를 잘 구별하는 word가 더 큰 영향을 준다. keyframe은 이 가중 BoW(bag of words) 벡터로 나타내고, word마다 그 word가 나온 keyframe 목록(inverted index)을 둔다. 새 프레임의 서술자는 vocabulary tree를 깊이 $L_w$만큼만 내려가면 word가 정해지고, 후보 keyframe은 inverted index에서 바로 찾으므로 지도 전체를 훑을 필요가 없다.

Tracking 스레드는 프레임마다 현재 포즈를 추정한다. 이전 프레임과 feature를 매칭한 뒤 motion-only bundle adjustment로 포즈 $\mathbf{T}_{cw} \in SE(3)$를 다듬는다. 3D–2D 대응 $\{(\mathbf{X}_i, \mathbf{u}_i)\}$로 세운 기본 reprojection 목적함수는 다음과 같다.

$$\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \left\| \mathbf{u}_i - \pi(\mathbf{T}\mathbf{X}_i) \right\|^2$$

여기서 $\pi$는 카메라 투영 함수, $\mathbf{X}_i$는 맵 포인트의 월드 좌표, $\mathbf{u}_i$는 이미지 좌표다. 실제 포즈 최적화는 robust loss와 관측 가중치를 쓰고, 맵 포인트를 고정한 채 현재 카메라 포즈만 바꾼다. 추적을 잃었을 때 재위치 추정(relocalization)은 EPnP와 RANSAC으로 초기 포즈 후보를 구한다. 이웃 keyframe과 맵 포인트를 함께 고치는 local BA는 Local Mapping 스레드가 맡는다.

KITTI 시퀀스 00(564×496 m 범위, keyframe 1,391개)에서 ORB-SLAM의 keyframe 궤적 RMSE는 6.68 m였다(논문 Table V, 다섯 번 실행한 값의 중앙값). 단안 시스템이라 절대 scale은 알 수 없어서, 참값과 비교할 때는 similarity 변환으로 궤적을 맞춘 뒤 오차를 쟀다. 이 시퀀스에서 루프를 닫아 누적 오차를 줄인 것은 DBoW2와 Essential graph였다.

---

## 7.2 ORB-SLAM2 (2017) — stereo/RGB-D

ORB-SLAM(2015)은 단안 카메라만 지원했다. 단안 영상으로 복원한 지도와 궤적은 어떤 배율(scale)을 곱해도 관측과 똑같이 맞아서, 실제 크기를 알 수 없다.

[Mur-Artal & Tardós 2017. ORB-SLAM2](https://doi.org/10.1109/TRO.2017.2705103)는 stereo와 RGB-D를 지원해 이 문제를 풀었다. stereo는 두 카메라 사이 거리(baseline)를 알기 때문에 depth를 바로 삼각측량하고, RGB-D는 센서가 깊이를 직접 잰다. 어느 쪽이든 scale을 알 수 있다.

구조는 mono와 같은 세 스레드이고, 여기에 전역 BA를 맡는 네 번째 스레드가 붙는다. 센서 종류에 따라 달라지는 것은 frontend의 입력 처리다. stereo는 rectified 이미지 쌍에서 ORB를 추출하고 좌우 매칭으로 depth를 구한다. 좌우 대응이 있는 특징점은 **stereo 관측**으로, 한쪽에서만 검출된 특징점은 **monocular 관측**으로 쓴다. 깊이를 구한 점은 다시 가까운 점과 먼 점으로 나눈다. 깊이가 baseline의 40배보다 작으면 가까운 점이다. 가까운 점은 translation을 정확히 잡아 주고, 먼 점은 회전 추정에 주로 기여한다.

**Stereo 초기화**는 첫 프레임에서 바로 끝난다. mono 초기화는 두 프레임 사이의 fundamental matrix나 homography로 지도를 만들기 때문에 scale을 알 수 없다. Stereo는 첫 keyframe에서 좌우 이미지의 수평 시차(disparity) $d$, baseline $b$, 초점 거리 $f$로 depth를 계산한다.

$$Z = \frac{b \cdot f}{d}$$

ORB-SLAM2는 첫 프레임을 keyframe으로 삼아 원점에 두고, 깊이를 구한 stereo 특징점 전부로 초기 지도를 만든다. RGB-D 초기화도 원리가 같다. depth 이미지에서 픽셀 $(u, v)$의 depth 값 $Z$를 읽고, 역투영(back-projection)으로 3D 좌표를 얻는다. 어느 쪽이든 처음부터 scale이 정해진다.

EuRoC MAV(Micro Aerial Vehicle) 데이터셋 Machine Hall 01 시퀀스에서 ORB-SLAM2(stereo)의 translation RMSE는 0.035 m였다(Table II). 논문은 같은 표에서 ORB-SLAM2가 Stereo LSD-SLAM보다 정확했다고 보고했다. 다만 V2_03 시퀀스에서는 심한 motion blur 때문에 추적을 일부 잃었다.

Mur-Artal과 Tardós는 mono, stereo, RGB-D 세 모드를 한 코드베이스로 GitHub에 공개했다. 저장소의 star는 수천 개를 넘었고, ROS 예제도 함께 들어 있었다.

---

## 7.3 ORB-SLAM3 (2021): Atlas와 Visual-Inertial

2021년 IEEE Transactions on Robotics에 실린 [Campos et al. 2021. ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644)는 저자가 바뀌었다. Carlos Campos와 Richard Elvira가 공동 1저자이고, Juan J. Gómez Rodríguez, J.M.M. Montiel, Juan D. Tardós가 함께 썼다. 모두 Zaragoza 대학 연구자이고, Mur-Artal은 저자 목록에 없다.

ORB-SLAM3는 **Atlas**(멀티맵)와 **Visual-Inertial** 모드를 추가했다.

Atlas는 서로 떨어진 지도 여러 개를 함께 유지한다. 추적이 실패하면 지금 지도를 닫고 새 지도를 시작하며, 나중에 같은 장소를 다시 지나면 두 지도를 합친다. ORB-SLAM과 ORB-SLAM2도 추적을 잃으면 기존 지도에서 재위치 추정을 시도했다. 그러나 재위치 추정에 실패한 뒤 새 지도를 시작하면 이전 지도와 함께 관리하거나 합치지 못했다. ORB-SLAM3는 Atlas 덕분에 추적에 실패해 다시 초기화해도 이전 지도를 버리지 않는다.

Visual-Inertial(VI) 모드는 IMU 데이터를 함께 쓴다. IMU 측정은 Forster 등의 preintegration 방식으로 처리한다. Forster 등은 이 방식을 RSS 2015에 발표하고, 2017년 IEEE TRO에 [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321)로 확장해 실었다. IMU는 카메라가 빠르게 움직여 영상만으로는 추적을 놓치기 쉬울 때 도움이 된다. 단안 카메라로는 알 수 없던 scale도 구할 수 있다. 가속도계는 미터 단위로 가속도를 재고, 중력 방향도 알려 주기 때문이다.

keyframe $i$와 $j$ 사이의 IMU 측정은 한 번만 적분해 둔다. 가속도계와 자이로스코프의 측정값을 $\tilde{\mathbf{a}}_t = \mathbf{a}_t + \mathbf{b}_a + \mathbf{n}_a$, $\tilde{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t + \mathbf{b}_g + \mathbf{n}_g$로 모델링하면(bias $\mathbf{b}$, noise $\mathbf{n}$), 두 keyframe 사이의 상대 회전, 속도, 위치 변화량은 다음과 같이 미리 적분(preintegration)한다.

$$\Delta\mathbf{R}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp}\bigl((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\bigr)$$
$$\Delta\mathbf{v}_{ij} = \sum_{k=i}^{j-1} \Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t$$
$$\Delta\mathbf{p}_{ij} = \sum_{k=i}^{j-1}\!\left[\Delta\mathbf{v}_{ik}\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}\,(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\right]$$

여기서 $\mathrm{Exp}(\cdot)$는 $\mathfrak{so}(3)$의 지수 사상이다. BA 중에 bias 추정값이 바뀌면 처음부터 다시 적분하지 않고 1차 선형 근사로 보정한다. ORB-SLAM3는 이 preintegrated 항($\Delta\mathbf{R}$, $\Delta\mathbf{v}$, $\Delta\mathbf{p}$)을 factor graph의 inertial edge로 넣어 visual reprojection residual과 함께 최적화한다.

> 🔗 **차용.** ORB-SLAM3의 관성 측정 처리는 Forster 등의 On-Manifold Preintegration(TRO 2017, 처음 발표는 RSS 2015)을 바탕으로 한다. Forster의 수식은 연속된 IMU 측정을 bias 보정과 함께 SO(3) manifold 위에서 적분한다. ORB-SLAM3는 이 결과를 factor graph 최적화에 넣었다.

EuRoC MAV 11개 시퀀스의 평균 ATE(절대 궤적 오차) RMSE에서 ORB-SLAM3(mono-inertial)는 0.043 m였다(Table II). 같은 표에서 VINS-Mono는 0.110 m, Kimera(stereo-inertial)는 0.119 m였다. 논문은 단일 세션 mono-inertial 기준으로 ORB-SLAM3가 VINS-Mono보다 2.6배 정확하다고 썼다.

---

## 7.4 2020년대에도 비교 기준으로 쓰이는 이유

2023년에도 학회 논문들은 ORB-SLAM3를 비교표에 넣었다.

ORB feature는 조명 변화에 어느 정도 강하다. binary 서술자라 계산이 빨라서 점을 많이 뽑아도 실시간으로 돌고, 그만큼 추적 실패가 줄어든다. learned feature는 특정 데이터셋에서 더 정확하지만, 학습 데이터와 다른 환경에서는 성능이 크게 떨어지기도 한다.

코드가 공개되어 있고 ROS와 함께 쓰기도 쉽다. 코드베이스 하나가 mono, stereo, RGB-D, IMU를 모두 지원하므로, 새 방법을 ORB-SLAM3(stereo)나 ORB-SLAM3(mono-inertial)과 설정별로 나란히 비교할 수 있다.

DROID-SLAM(Teed & Deng, NeurIPS 2021)은 여러 시퀀스에서 ORB-SLAM3보다 정확했다. 그러나 논문은 RTX 3090 GPU 두 장이면 실시간으로 돈다고 했고, EuRoC, TartanAir, ETH-3D 결과를 내려면 메모리 24 GB짜리 GPU가 필요하다고 적었다. 카메라가 빠르게 움직이는 TartanAir에서는 평균 8 fps로 실시간에 못 미쳤다. ORB-SLAM3는 CPU만으로 돌고, 사용자 보고에 따르면 ARM 임베디드 보드에서도 돌아간다.

---

## 📜 예언 vs 실제

> 📜 **예언 vs 실제.** Mur-Artal은 2015년 ORB-SLAM 논문 Section IX-C(Future Work)에서 먼저, 시차가 부족해 지도에 넣지 못한 먼 점(points at infinity)을 tracking에 써서 정확도를 높이자고 했다. 논문은 이런 점이 카메라 회전에 관해 정보가 많다고 적었다. 이어서 희소한 지도를 더 조밀한 복원으로 키우자고 했다. 잘 고른 keyframe은 포즈가 정확하고 covisibility 정보가 풍부해서 dense 지도의 초기값이자 골격이 될 수 있다고 봤다. 먼 점 활용은 VI-SLAM과 후속 연구에 일부 반영됐다. dense 복원은 2020년대 NeRF-SLAM과 Gaussian Splatting 연구가 희소한 keyframe에 dense 표현을 붙이는 방식으로 이어졌다. stereo, RGB-D, IMU 지원은 이 절에 없던 주제였고, ORB-SLAM2(2017)와 ORB-SLAM3(2021)에서 따로 다뤘다.

> 📜 **예언 vs 실제.** Campos 등은 2021년 ORB-SLAM3 논문 결론에서 ORB-SLAM3가 주로 실패하는 곳이 텍스처가 적은 환경이라고 적었다. 그리고 논문이 다룬 네 가지 data association 문제에 맞는 photometric 기법을 흥미로운 연구 방향으로 꼽았다. 저자들은 이 생각을 인체 내부 내시경 영상으로 지도를 만드는 데 시험하고 있었다. 2023–2025년에는 SuperPoint나 LightGlue 같은 learned frontend를 ORB-SLAM3에 붙이는 연구가 더 많이 나왔다. photometric 기법은 DSO, LDSO 같은 direct 방법에서 따로 발전했다. ORB-SLAM3 공식 저장소 main branch는 2026년 현재도 전통 ORB descriptor를 쓴다.

---

## 🧭 아직 열린 것

**지도의 장기 재사용.** Atlas로 지도 여러 개를 유지할 수는 있게 됐지만, 조명이 크게 달라지면 지도 병합이 실패할 수 있다. 아침에 지도를 만든 곳에 저녁에 다시 오면 같은 곳으로 알아봐야 하는데, 겉모습이 크게 바뀌면 DBoW2가 놓친다. 계절이 바뀌는 실외 환경에서는 이 문제가 장기 자율주행 연구의 과제로 남아 있다.

**전통 feature는 아직 기준선인가.** learned feature를 쓰는 시스템들이 표준 benchmark에서 ORB-SLAM3를 앞서기 시작했다. SuperPoint와 SuperGlue 조합, LightGlue, DINOv2 기반 feature가 특정 시퀀스에서 더 낮은 오차를 낸다. 학습 분포 밖의 환경에서 learned feature가 전통 ORB보다 나쁜 결과를 내는 경우가 보고된다.

**넓은 실외의 drift.** 도심 주행이나 수 km가 넘는 경로에서 ORB-SLAM3는 여전히 LiDAR SLAM보다 덜 정확하다. 조명과 날씨가 바뀌고, 물체가 움직이고, 텍스처 없는 구간이 겹치면 drift가 쌓인다.

---

ORB-SLAM 세 편이 feature 기반 SLAM의 표준이 되던 무렵, Newcombe와 Engel은 이미지의 밝기 값을 직접 쓰는 방법을 만들고 있었다. 두 방식은 2010년대 내내 함께 발전하며 서로를 비교 대상으로 삼았다. DSO 논문은 loop closure와 relocalization을 끈 단안 ORB-SLAM과 비교해 TUM monoVO와 ICL-NUIM에서 DSO가 더 정확했다고 보고했다(Ch.8 §3).
