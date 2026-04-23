# Ch.13 — Hybrid 승리: CodeSLAM에서 DROID-SLAM까지

Michael Bloesch가 2018년 CVPR에 CodeSLAM을 발표했을 때, 그의 소속은 Imperial College London의 Dyson Robotics Lab이었다. 지도교수는 Andrew Davison. 같은 연구실에서 2011년 Richard Newcombe가 DTAM을 만들었고, 같은 연구실에서 Jan Czarnowski가 2020년 DeepFactors를 내놓았고, Edgar Sucar와 Tristan Laidlow가 계보를 이었다. CodeSLAM이 한 편의 논문 이상인 이유가 거기에 있다. Davison이 2002년부터 쌓아온 "SLAM은 확률론적 추론이다"라는 신조와, 2010년대 중반 딥러닝이 가져온 "표현을 배울 수 있다"는 충동이 2018년 Bloesch의 논문에서 처음 실질적으로 만났다.

---

## 13.1 CodeSLAM — latent code가 지도를 만나다

전통적인 monocular SLAM에서 depth는 추정의 대상이었다. 수백 개의 sparse landmark이든, DTAM처럼 모든 픽셀이든, depth는 결국 최적화 변수였다. 그 변수 공간의 차원은 이미지 해상도에 비례했다. keyframe 한 장의 dense depth map은 640×480 해상도에서 307,200개의 독립 변수를 의미한다. 최적화는 무겁고, 초기화는 민감하고, prior를 넣기가 어렵다.

[Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271)의 착상은 간단했다. depth map 자체를 최적화하는 대신, depth map을 생성하는 저차원 잠재 벡터(**latent code**)를 최적화하자. Variational autoencoder(VAE)를 훈련해 실제 depth 분포를 학습시키면, 그 bottleneck latent space는 "사실적인 depth map"들이 사는 다양체를 근사한다. 최적화는 그 다양체 위에서만 움직인다. 변수가 수십만 개에서 수백 개로 줄어든다.

> 🔗 **차용.** CodeSLAM의 latent depth 표현은 [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114)에서 확립된 encoder-decoder 잠재 공간 구조를 차용했다. 학습 단계에서는 VAE 틀을 따르되, SLAM 추론 시에는 stochastic sampling 없이 **z**를 직접 MAP 최적화 변수로 다룬다. 생성 모델 연구자들이 이미지 합성을 위해 고안한 도구가, 10년 뒤 SLAM 최적화의 저차원 표현 공간으로 재등장했다.

구조는 이렇다. keyframe마다 VAE encoder가 이미지에서 latent code **z**를 추출한다. Decoder는 **z**에서 dense depth map을 재구성한다. Camera pose와 **z**는 jointly 최적화된다. photometric loss가 consistency를 강제하고, latent prior가 **z**를 사전 분포 근방에 머물도록 regularize한다.

수식으로 쓰면 objective는:

$$E(\mathbf{z}, T) = \sum_{i,j} \rho\bigl(I_j(\pi(T_{ij}, D_\mathbf{z}(u_i), u_i)) - I_i(u_i)\bigr) + \lambda \|\mathbf{z}\|^2$$

$D_\mathbf{z}$는 decoder, $\pi$는 projection, $\rho$는 robust cost, $T_{ij}$는 keyframe 간 상대 pose. latent prior 항 $\lambda\|\mathbf{z}\|^2$은 표준 정규 prior $p(\mathbf{z}) = \mathcal{N}(0, I)$의 negative log-likelihood에 해당하며, Gaussian prior 가정 아래 MAP inference에서 자연스럽게 등장하는 regularizer다.

> 🔗 **차용.** Factor graph(Dellaert & Kaess의 GTSAM)는 DeepFactors의 backend 골격을 제공했다. CodeSLAM이 joint optimization으로 처리한 pose-latent 결합 구조를 Czarnowski는 명시적 factor graph로 재정식화했다. Learning이 만든 잠재 변수가 전통적인 pose node 옆에 또 하나의 graph 변수로 편입된 것은 DeepFactors에 이르러서다. 두 세계의 인터페이스가 graph의 edge였다.

sparse 입력에서 geometry를 채워 넣는 능력이 기존 방법을 앞섰다. 그러나 CodeSLAM 자체는 실시간이 아니었다. VAE 추론과 최적화 루프가 느렸다. 논문은 그것을 솔직하게 밝혔다.

> 📜 **예언 vs 실제.** Bloesch는 CodeSLAM §6 "Future Work"에서 "실시간·대규모 환경으로의 확장, 다양한 센서 모달리티 적용"을 다음 과제로 꼽았다. DeepFactors(2020)가 실시간이라는 목표를 향해 한 발 나아갔으나 상용 배포 수준은 되지 못했다. "다양한 카메라에서 범용으로 작동"이라는 목표는 결국 다른 팀(Teed·Deng, Princeton)이 다른 방법으로 달성했다. `[진행형+기술변화]`

---

## 13.2 DeepFactors — 같은 연구실, 한 발 더

2020년, Jan Czarnowski도 Davison 지도 아래 Imperial Dyson Robotics Lab에서 [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2969036)를 발표했다. Czarnowski의 목표는 CodeSLAM의 아이디어를 실제 SLAM 파이프라인 안으로 끌어들이는 것이었다.

DeepFactors는 CodeSLAM의 factor graph + latent depth 구조를 유지하면서 tracking과 mapping을 명시적으로 분리하고, keyframe 선택 기준을 도입했다. GPU 최적화로 640×480에서 초당 수 프레임. 여전히 실용적 실시간(30Hz)에는 미치지 못했으나 방향을 보여주었다.

DeepFactors가 더 중요하게 증명한 것은 원칙이었다. learned representation은 factor graph의 한 노드로 들어갈 수 있고, geometry optimization은 그 latent space 위에서 작동할 수 있다. Czarnowski가 도달한 결론은 단순했다. end-to-end 교체가 아니라 파이프라인 일부를 학습 가능한 모듈로 바꾸는 것이 현실적 경로라는 것.

그 원칙은 2021년 Princeton에서 전혀 다른 방식으로 다시 나타났다.

---

## 13.3 RAFT — optical flow가 SLAM에게 준 도구

Zachary Teed와 Jia Deng(Princeton)은 2020년 ECCV에 [Recurrent All-Pairs Field Transforms(RAFT)](https://arxiv.org/abs/2003.12039)를 발표했다. RAFT는 SLAM 논문이 아니었다. optical flow 추정 논문이었다.

그러나 RAFT의 설계는 이후 DROID-SLAM의 핵심이 된다. 구조는 세 부분으로 나뉜다.

1. Feature encoder: CNN이 두 이미지에서 feature map 추출
2. Correlation volume: 모든 픽셀 쌍 간의 유사도를 4D volume으로 구성. 4-level pyramid
3. Update operator: Gated Recurrent Unit(GRU) 기반 반복 refinement. correlation volume을 lookup하며 flow field를 업데이트

이름에 들어간 all-pairs가 이 구조의 차별점을 요약한다. 특정 이웃 픽셀만 보는 것이 아니라 모든 후보 위치를 동시에 고려하고, 고정 해상도에서 flow field를 점진적으로 refinement한다. 기존 coarse-to-fine 방법(PWC-Net 등)과 달리 flow field를 단일 full-resolution으로 유지한 채 correlation pyramid를 lookup한다. KITTI, Sintel, FlyingThings3D에서 기존 방법을 5%-15% 앞섰다.

RAFT는 SLAM 계보의 조상이 아니다. 그러나 Teed는 같은 update operator 구조가 SLAM의 iterative bundle adjustment와 구조적으로 유사하다는 것을 알아챘다. flow field를 refinement하는 GRU가 pose와 depth를 refinement하는 최적화 스텝과 얼마나 다른가.

---

## 13.4 DROID-SLAM — update operator가 Bundle Adjustment를 만나다

2021년 NeurIPS, [Teed & Deng. DROID-SLAM](https://arxiv.org/abs/2108.10869). 제목의 DROID는 "Differentiable Recurrent Optimization-Inspired Design"의 약자다.

아키텍처를 따라가면 hybrid 설계의 의도가 드러난다.

Frontend는 RAFT와 동일한 구조다. CNN encoder가 feature map을 추출하고, all-pairs correlation volume을 구성하고, GRU update operator가 dense flow를 반복 추정한다. 차이는 한 쌍의 이미지 사이 flow가 아니라 keyframe graph의 모든 edge에서 동시에 flow를 추정한다는 것이다.

Backend는 Dense Bundle Adjustment(DBA)다. Pose와 inverse depth가 optimization 변수다. flow 추정이 제공하는 2D correspondence를 제약으로 사용해 pose-depth를 jointly 최적화한다. Schur complement trick으로 선형 시스템을 효율적으로 푼다.

연결고리는 **DBA layer**다. GRU가 추정한 flow와 uncertainty가 DBA에 입력된다. DBA가 pose·depth를 업데이트하면, 그 결과가 다음 GRU iteration의 reference를 갱신한다. 두 모듈이 loop로 연결된다.

> 🔗 **차용.** Dense BA라는 아이디어 자체는 10년 전으로 거슬러 올라간다. Newcombe의 DTAM(2011)은 모든 픽셀을 사용한 photometric bundle adjustment의 선구자였다. DROID-SLAM은 그 아이디어를 learned flow라는 더 강건한 입력과 결합했다. Newcombe와 Teed의 affiliations는 다르지만 논리적 계보는 이어진다.

> 🔗 **차용.** DROID-SLAM의 update operator는 같은 저자(Teed·Deng)의 RAFT에서 직접 이식했다. optical flow를 위해 설계된 all-pairs recurrent refinement가 bundle adjustment의 반복 최적화와 구조적으로 호환된다는 통찰이 핵심이었다. 같은 사람이 두 논문을 썼다는 사실이 이 차용을 가능하게 했다.

EuRoC MAV 데이터셋에서 DROID-SLAM은 당시 최고 수준인 ORB-SLAM3보다 낮은 RMSE ATE를 기록했다. TartanAir(합성)와 실제 실내외 시퀀스 양쪽에서. 특히 조명 변화와 texture 부족 상황에서 feature-based 방법보다 강건했다.

12장의 순수 end-to-end 접근이 왜 실패했는지 돌아보면 DROID-SLAM이 왜 달랐는지 드러난다. PoseNet은 geometry constraint 없이 pose를 직접 회귀했고, 일반화에 실패했다. Teed와 Deng은 역할을 나눴다. dense correspondence 추정은 학습에 맡기고, geometry 제약 강제는 BA가 담당했다. feature 추출·dense matching에서는 신경망이, consistency enforcement·uncertainty 전파에서는 geometry optimizer가 각자 강점을 살렸다. 인간이 설계한 feature를 학습된 feature로 교체했지만, optimization structure는 그대로였다. 2021년의 hybrid가 2015년의 end-to-end와 달랐던 지점이 거기에 있다.

---

## 13.5 Imperial Dyson Lab 계보도

CodeSLAM에서 DROID-SLAM까지의 흐름은 Imperial Dyson Robotics Lab의 인적 계보를 따라가면 제대로 보인다.

Andrew Davison은 2002년 MonoSLAM 이후 20년 동안 Imperial에서 SLAM 연구를 이끌었다. 제자와 협력자들이 차례로 분기점을 만들었다.

- **Richard Newcombe** (Davison 지도, Imperial): DTAM(2011), KinectFusion(2011). 이후 Oculus→Meta Reality Labs
- **Michael Bloesch** (Davison 지도, Imperial): CodeSLAM(2018), touch·inertial SLAM 연구
- **Jan Czarnowski** (Davison 지도, Imperial): DeepFactors(2020)
- **Edgar Sucar** (Davison 그룹, Imperial): iMAP(2021), 이후 NeRF-SLAM 계보로 연결
- **Tristan Laidlow** (Davison 그룹, Imperial): dense 3D reconstruction, 이후 neural implicit SLAM 계보로 연결

이 계보는 "학파"라는 단어가 과장이 아닌 경우 중 하나다. factor graph + uncertainty 철학이 단안 sparse에서 dense latent로, dense latent에서 implicit representation으로 모양을 바꾸며 이어졌다. Davison은 2020년 FutureMapping 논문 시리즈에서 "universal scene representation"이라는 방향을 직접 이름 붙여 공개했다. CodeSLAM과 DeepFactors는 그 방향의 첫 번째 실험들이었다.

Teed와 Deng은 이 계보 밖에 있다. Princeton, 독립적 경로. 그러나 DROID-SLAM이 채택한 dense BA의 논리적 전임자는 DTAM이고, DTAM은 Newcombe·Davison의 작품이다. 계보는 인적 연결 없이도 논리적으로 이어진다.

---

## 13.6 2023-2025 — DROID 이후의 확장

DROID-SLAM 이후 2년간 그 위에서 성장한 연구들이 나왔다.

GO-SLAM(Zhang et al. 2023)은 DROID의 tracking module을 유지하면서 mapping을 NeRF로 교체했다. dense photometric map이 생성된다. tracking은 classical BA, map은 implicit representation. hybrid의 두 번째 층이다.

NICER-SLAM(Zhu et al. 2023)은 RGB-only로 DROID style tracking과 neural implicit map을 결합했다. 다양한 실내 시퀀스에서 depth map 품질과 localization 모두 이전 방법을 앞섰다고 보고했다.

SplaTAM(Keetha et al. 2024)은 map representation을 3D Gaussian Splatting으로 교체했다. rendering 속도가 빨라 온라인 loop closure에 활용 가능하다. DROID 계보와 3DGS의 결합이다.

DPV-SLAM(2024)은 DROID-SLAM의 frontend feature를 DINOv2 기반 foundation visual feature로 교체했다. 일부 시나리오에서 domain 적응력이 올랐다.

패턴은 같다. tracking은 DROID 또는 유사한 recurrent dense flow 방식을 유지하고, map representation이나 feature를 교체한다. 2021년 Teed와 Deng이 내놓은 learned frontend + classical backend 프레임워크가 기반이 되었다.

> 📜 **예언 vs 실제.** Teed와 Deng은 DROID-SLAM §6에서 "foundation-scale visual representation과 통합될 것"을 미래 방향으로 명시했다. 2024년 DPV-SLAM과 몇몇 후속 연구들이 이 방향을 탐구하고 있다. DINO 계열 feature로 교체한 결과는 일부 시나리오에서 개선을 보였으나 범용 해법은 아직 없다. `[진행형]`

---

## 🧭 아직 열린 것

Learned prior의 분포 밖 일반화가 첫 번째 문제다. CodeSLAM과 DeepFactors의 VAE는 훈련 데이터의 depth 분포를 학습한다. 완전히 다른 환경(실외 open-world, 비균질 texture, 야간)에서는 learned prior가 오히려 최적화를 잘못된 방향으로 당길 수 있다. DROID-SLAM의 flow estimator도 훈련 도메인 밖에서 성능이 떨어진다. 2026년 현재, "어떤 환경에서도 작동하는 learned SLAM"은 아직 없다. TartanAir처럼 다양한 합성 데이터로 훈련하는 접근이 있으나 sim-to-real gap이 남는다.

실시간 제약도 여전하다. DROID-SLAM은 NVIDIA RTX 2080Ti 기준으로 평균 10-15 fps 수준이다. keyframe graph 크기에 따라 더 느려진다. dense BA가 병목이다. 모바일 로봇이나 AR/VR처럼 실시간(30Hz+), 저전력 배포가 필요한 응용에서는 2026년 현재도 실용적이지 않다. 경량화 시도들(keyframe 수 줄이기, approximate BA)이 있으나 성능 trade-off가 따른다.

Loop closure의 learned 통합도 미해결이다. DROID-SLAM은 loop closure를 명시적으로 다루지 않는다. keyframe graph가 sliding window 방식으로 유지되고, global consistency는 제한적이다. learned loop closure(12장의 place recognition 연구들)를 DROID의 factor graph에 통합하는 시도가 일부 있으나 아직 단일 시스템으로 수렴하지 않았다. 12장 NetVLAD 계보와 13장 DROID 계보가 만나는 지점이 아직 열려 있다.

---

여기까지 오면 물음 하나가 남는다. map representation을 점, 선, 평면으로 유지해야 하는가. DROID-SLAM의 inverse depth map은 2021년 기준 최선의 dense representation이었다. 그러나 2020년, NeRF(Neural Radiance Field)가 전혀 다른 가능성을 제시했다. 장면을 포인트나 메시가 아니라 연속 함수로 표현하면 어떤가. 렌더링이 미분 가능하다면 photometric consistency를 새로운 방식으로 강제할 수 있다.

그것은 4부(러닝 융합기)가 hybrid optimization으로 마무리되는 지점이기도 하고, 5부(표현의 혁명)가 시작하는 지점이기도 하다. 14장은 NeRF가 SLAM에 충돌하는 순간을 따라간다.
