# Ch.13 — Hybrid 승리: CodeSLAM에서 DROID-SLAM까지

Michael Bloesch가 2018년 CVPR에 CodeSLAM을 발표했을 때, 그의 소속은 Imperial College London의 Dyson Robotics Lab이었고 지도교수는 Andrew Davison이었다. 같은 연구실에서 2011년 Richard Newcombe가 DTAM을 만들었고, Jan Czarnowski가 2020년 DeepFactors를 내놓았으며, Edgar Sucar와 Tristan Laidlow가 계보를 이었다. Davison이 2002년부터 쌓아온 확률론적 SLAM 계보와 2010년대 중반 딥러닝의 "표현을 배울 수 있다"는 발상이 2018년 Bloesch의 논문에서 만났다.

---

## 13.1 CodeSLAM — latent code와 지도

전통적인 monocular SLAM에서 depth는 추정의 대상이었다. 수백 개의 sparse landmark이든, [DTAM](https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf)(Newcombe et al. 2011)처럼 모든 픽셀이든, depth는 결국 최적화 변수였다. 그 변수 공간의 차원은 이미지 해상도에 비례했다. keyframe 한 장의 dense depth map은 640×480 해상도에서 307,200개의 독립 변수를 의미한다. 최적화는 무겁고, 초기화는 민감하고, prior를 넣기가 어렵다.

[Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271)은 depth map 자체가 아니라, depth map을 생성하는 저차원 잠재 벡터(**latent code**)를 최적화했다. Variational autoencoder(VAE)를 훈련해 실제 depth 분포를 학습시키면, 그 bottleneck latent space는 "사실적인 depth map"들이 사는 다양체를 근사한다. 최적화는 그 다양체 위에서만 움직인다. 변수가 수십만 개에서 수백 개로 줄어든다.

> 🔗 **차용.** CodeSLAM의 latent depth 표현은 [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114)에서 확립된 encoder-decoder 잠재 공간 구조를 차용했다. 학습 단계에서는 VAE 틀을 따르되, SLAM 추론 시에는 stochastic sampling 없이 **z**를 직접 MAP 최적화 변수로 다룬다. 생성 모델의 잠재 공간 도구가 약 5년 뒤 SLAM 최적화의 저차원 표현으로 쓰였다.

keyframe마다 VAE encoder가 이미지에서 latent code **z**를 추출한다. Decoder는 **z**에서 dense depth map을 재구성한다. Camera pose와 **z**는 jointly 최적화된다. photometric loss가 consistency를 강제하고, latent prior가 **z**를 사전 분포 근방에 머물도록 regularize한다.

목적함수는 다음과 같다:

$$E(\mathbf{z}, T) = \sum_{i,j} \rho\bigl(I_j(\pi(T_{ij}, D_\mathbf{z}(u_i), u_i)) - I_i(u_i)\bigr) + \lambda \|\mathbf{z}\|^2$$

$D_\mathbf{z}$는 decoder, $\pi$는 projection, $\rho$는 robust cost, $T_{ij}$는 keyframe 간 상대 pose다. latent prior 항 $\lambda\|\mathbf{z}\|^2$은 표준 정규 prior $p(\mathbf{z}) = \mathcal{N}(0, I)$의 negative log-likelihood에 해당하며, Gaussian prior 가정 아래 MAP inference에서 자연스럽게 등장하는 regularizer다.

> 🔗 **차용.** Factor graph(Dellaert & Kaess의 [GTSAM](https://gtsam.org/tutorials/intro.html))는 DeepFactors의 backend 골격을 제공했다. CodeSLAM이 joint optimization으로 처리한 pose-latent 결합 구조를 Czarnowski는 명시적 factor graph로 재정식화했다. Learning이 만든 잠재 변수가 전통적인 pose node 옆에 또 하나의 graph 변수로 편입된 것은 DeepFactors에 이르러서다. 두 세계의 인터페이스가 graph의 edge였다.

sparse 입력에서 geometry를 채워 넣는 능력이 기존 방법을 앞섰다. 그러나 CodeSLAM 자체는 실시간이 아니었다. VAE 추론과 최적화 루프가 느렸고, 논문에도 이 한계가 명시됐다.

> 📜 **예언 vs 실제.** CodeSLAM은 compact learned representation을 dense SLAM 안에 들이는 가능성을 보였지만 속도와 규모 양쪽에서 여지를 남겼다. 이어진 DeepFactors(2020)가 같은 Imperial 그룹에서 실시간 쪽으로 한 발 더 나아갔으나 상용 배포 수준은 되지 못했고, monocular·stereo·RGB-D를 아우르는 범용 성능은 결국 다른 팀(Teed·Deng, Princeton)이 학습된 frontend + dense BA라는 다른 설계로 달성했다.

---

## 13.2 DeepFactors — Imperial Dyson Lab, factor graph 통합

2020년, Jan Czarnowski도 Davison 지도 아래 Imperial Dyson Robotics Lab에서 [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2969036)를 발표했다. Czarnowski의 목표는 CodeSLAM의 아이디어를 실제 SLAM 파이프라인 안으로 끌어들이는 것이었다.

DeepFactors는 CodeSLAM의 latent depth 표현과 joint optimization을 명시적인 factor graph로 확장하면서 tracking과 mapping을 명시적으로 분리하고, keyframe 선택 기준을 도입했다. NVIDIA GTX 1080 위에서 keyframe 대비 tracking은 약 250Hz로 돌았으나, network Jacobian 계산이 keyframe당 수백 밀리초를 차지해 전체 파이프라인의 병목이었다. 방향은 보여주었으되 상용 배포 수준의 실시간에는 미치지 못했다.

DeepFactors는 learned representation을 factor graph의 한 노드로 넣을 수 있고, geometry optimization이 그 latent space 위에서 작동할 수 있음을 보였다. Czarnowski는 파이프라인 일부를 학습 가능한 모듈로 바꾸는 경로를 택했다.

같은 시기 TU Munich의 Daniel Cremers 그룹도 같은 원칙에 도달했다. 출발점이 Imperial과 달랐을 뿐이다. Davison 계보가 CodeSLAM의 VAE latent 위에 factor graph를 쌓았다면, Cremers 그룹은 자신들이 2016년 내놓은 direct sparse odometry([DSO](https://arxiv.org/abs/1607.02565))를 뼈대로 두고 거기에 neural prediction을 주입했다. [Yang, Wang, Stückler, Cremers 2018. DVSO](https://arxiv.org/abs/1807.02570)는 단안 DSO에 neural depth를 "가상 스테레오"로 주입해 두 번째 카메라에 해당하는 관측을 만들었고, [Yang, von Stumberg, Wang, Cremers 2020. D3VO](https://arxiv.org/abs/2003.01060)는 self-supervised로 학습된 depth·pose·uncertainty 세 종류의 neural prediction을 DSO의 factor graph에 추가 factor로 넣었다. [Wimbauer et al. 2021. MonoRec](https://arxiv.org/abs/2011.11814)과 [Wimbauer et al. 2023. Behind the Scenes](https://arxiv.org/abs/2301.07668)는 같은 계보를 dynamic scene dense reconstruction과 single-view density field 쪽으로 이어갔다. 인적 계보는 Imperial 그룹과 분리되어 있지만, "neural prediction을 고전 optimization 구조 안으로 흡수한다"는 설계 원칙은 수렴했다.

그 원칙은 2021년 Princeton에서 또 다른 방식으로 다시 나타났다.

---

## 13.3 RAFT — recurrent optical flow

Zachary Teed와 Jia Deng(Princeton)은 2020년 ECCV에 [Recurrent All-Pairs Field Transforms(RAFT)](https://arxiv.org/abs/2003.12039)를 발표했다. RAFT는 SLAM 논문이 아니었다. optical flow 추정 논문이었다.

RAFT의 설계는 이후 DROID-SLAM에 들어갔다. 구조는 세 부분으로 나뉜다.

1. Feature encoder: CNN이 두 이미지에서 feature map을 추출한다.
2. Correlation volume: 모든 픽셀 쌍 간의 유사도를 4-level pyramid의 4D volume으로 구성한다.
3. Update operator: Gated Recurrent Unit(GRU)이 correlation volume을 lookup하며 flow field를 반복해서 refinement한다.

이름에 들어간 all-pairs가 이 구조의 차별점을 요약한다. 모든 후보 위치를 동시에 고려하고, 고정 해상도에서 flow field를 점진적으로 refinement한다. 기존 coarse-to-fine 방법(PWC-Net 등)과 달리 flow field를 원영상의 가로·세로 1/8인 단일 해상도로 유지한 채 correlation pyramid를 lookup하고, 최종 출력을 upsampling한다. RAFT 논문은 당시 공개 최고 결과와 비교해 KITTI의 F1-all 오류를 16%, Sintel final pass의 endpoint error를 30% 상대 감소시켰다고 보고했다.

RAFT가 처음 풀던 과제는 SLAM이 아니라 optical flow였다. 그러나 Teed는 같은 update operator 구조가 SLAM의 iterative bundle adjustment와 구조적으로 유사하다는 점을 DROID-SLAM 설계에 활용했다. flow field를 refinement하는 GRU가 pose와 depth를 refinement하는 최적화 스텝과 얼마나 다른가.

---

## 13.4 DROID-SLAM — update operator와 BA

2021년 NeurIPS, [Teed & Deng. DROID-SLAM](https://arxiv.org/abs/2108.10869). 제목의 DROID는 "Differentiable Recurrent Optimization-Inspired Design"의 약자다.

아키텍처를 따라가면 hybrid 설계의 의도가 드러난다.

Frontend는 RAFT와 동일한 구조다. CNN encoder가 feature map을 추출하고, all-pairs correlation volume을 구성하고, GRU update operator가 dense flow를 반복 추정한다. 차이는 keyframe graph의 모든 edge에서 동시에 flow를 추정한다는 점이다.

Backend는 Dense Bundle Adjustment(DBA)다. Pose와 inverse depth가 optimization 변수다. flow 추정이 제공하는 2D correspondence를 제약으로 사용해 pose-depth를 jointly 최적화한다. Schur complement trick으로 선형 시스템을 효율적으로 푼다.

**DBA layer**가 두 모듈을 연결한다. GRU가 추정한 flow와 uncertainty가 DBA에 입력된다. DBA가 pose·depth를 업데이트하면, 그 결과가 다음 GRU iteration의 reference를 갱신한다.

> 🔗 **차용.** Dense BA라는 아이디어 자체는 10년 전으로 거슬러 올라간다. Newcombe의 DTAM(2011)은 모든 픽셀을 사용한 photometric bundle adjustment의 선구자였다. DROID-SLAM은 그 아이디어를 learned flow라는 더 강건한 입력과 결합했다. Newcombe와 Teed의 affiliations는 다르지만 논리적 계보는 이어진다.

> 🔗 **차용.** DROID-SLAM의 update operator는 같은 저자(Teed·Deng)의 RAFT에서 직접 이식했다. optical flow를 위해 설계된 all-pairs recurrent refinement가 bundle adjustment의 반복 최적화와 구조적으로 호환된다는 통찰이 핵심이었다. 두 논문의 저자가 같아 차용 관계도 직접적이다.

EuRoC MAV 데이터셋에서 DROID-SLAM은 당시 최고 수준인 ORB-SLAM3보다 낮은 RMSE ATE를 기록했고, TartanAir(합성)와 실제 실내외 시퀀스 양쪽에서 특히 조명 변화와 texture 부족 상황에 feature-based 방법보다 강건했다. Teed가 이후 Handbook 회고에서 EuRoC V1_02 시퀀스에 대해 보고한 수치가 인상적이다. frontend만 돌렸을 때 16.5cm이던 ATE가 global optimization을 거치면 1.2cm로 떨어졌다. learned correspondence가 공급한 제약 위에서 고전 BA가 한 자릿수 cm까지 수렴하는 장면이다.

PoseNet은 geometry constraint 없이 pose를 직접 회귀했고 일반화에 실패했다. Teed와 Deng은 역할을 나눴다. dense correspondence 추정은 학습에 맡기고, geometry 제약 강제는 BA가 담당했다. feature 추출·dense matching에서는 신경망이, consistency enforcement·uncertainty 전파에서는 geometry optimizer가 각자 강점을 살렸다. 인간이 설계한 feature를 학습된 feature로 교체하면서도 optimization structure는 그대로 유지됐다. 이 지점에서 2021년의 hybrid는 2015년의 end-to-end와 갈라졌다.

---

## 13.5 Imperial Dyson Lab 계보도

CodeSLAM에서 DROID-SLAM까지의 흐름은 Imperial Dyson Robotics Lab의 인적 계보를 따라가면 제대로 보인다.

Andrew Davison은 2002년 MonoSLAM 이후 20년 동안 Imperial에서 SLAM 연구를 이끌었다. 제자와 협력자들이 차례로 분기점을 만들었다.

- **Richard Newcombe** (Davison 지도, Imperial): DTAM(2011), [KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf)(2011). 이후 Oculus→Meta Reality Labs
- **Michael Bloesch** (Davison 지도, Imperial): CodeSLAM(2018), touch·inertial SLAM 연구
- **Jan Czarnowski** (Davison 지도, Imperial): DeepFactors(2020)
- **Edgar Sucar** (Davison 그룹, Imperial): [iMAP](https://arxiv.org/abs/2103.12352)(2021), 이후 NeRF-SLAM 계보로 연결
- **Tristan Laidlow** (Davison 그룹, Imperial): dense 3D reconstruction, 이후 neural implicit SLAM 계보로 연결

이 계보는 "학파"라는 단어가 과장이 아닌 경우 중 하나다. factor graph + uncertainty 철학이 단안 sparse에서 dense latent로, dense latent에서 implicit representation으로 모양을 바꾸며 이어졌다. Davison은 [FutureMapping](https://arxiv.org/abs/1803.11288)(2018)과 [FutureMapping 2](https://arxiv.org/abs/1910.14139)(2019, Ortiz 공저)에서 Spatial AI 시스템이 갖춰야 할 계산 구조와 표현을 직접 지도 위에 스케치했다. 다양한 geometric·semantic 표현을 하나의 확률 그래프 위에 묶자는 주장이었다. CodeSLAM과 DeepFactors는 그 스케치의 첫 번째 실험들이었다.

Teed와 Deng은 Princeton에서 독립적인 경로를 걸었다. 그러나 DROID-SLAM이 채택한 dense BA의 논리적 전임자는 DTAM이고, DTAM은 Newcombe·Davison의 작품이다. 계보는 인적 연결을 건너뛰고도 논리로 이어진다.

---

## 13.6 2023-2025 — DROID 이후의 확장

DROID-SLAM 이후 몇 년간 여러 후속 연구가 나왔다.

[GO-SLAM](https://arxiv.org/abs/2309.02436)(Zhang et al. 2023)은 DROID-SLAM의 tracking을 확장해 online loop closing과 full bundle adjustment를 얹고, mapping은 Instant-NGP 계열 neural implicit 표현(multi-resolution hash encoding)으로 돌렸다. tracking은 DROID 계열 dense flow + BA, map은 implicit representation. hybrid의 두 번째 층이다.

[NICER-SLAM](https://arxiv.org/abs/2302.03594)(Zhu et al. 2023)은 다른 길을 갔다. tracking과 mapping을 하나의 hierarchical neural implicit representation 위에서 동시에 풀었다. RGB-only dense SLAM이라는 목표는 공유하지만 경로가 다르다. DROID 계보의 외곽에서 같은 문제에 부딪히는 방식이다.

[SplaTAM](https://arxiv.org/abs/2312.02126)(Keetha et al. 2024)은 map representation을 3D Gaussian Splatting으로 바꾸고, tracking도 silhouette-guided differentiable rendering 기반으로 다시 짰다. 3DGS 계보와의 결합이지 DROID 계보의 직접 확장은 아니다.

[DPV-SLAM](https://arxiv.org/abs/2408.01654)(Lipson, Teed, Deng 2024)은 DROID-SLAM과 같은 Princeton 그룹에서 나왔다. [DPVO](https://github.com/princeton-vl/DPVO)(Deep Patch Visual Odometry)를 기반으로, 근접 기반 loop closure와 CUDA block-sparse BA를 추가해 DROID-SLAM 대비 약 2.5배 빠르고 메모리 footprint가 작은 시스템을 만들었다. 핵심은 patch 기반 sparse 표현 + 효율적 loop closure다.

DROID 계보 바깥에서는 Naver Labs가 열어놓은 [DUSt3R](https://arxiv.org/abs/2312.14132)(Wang et al. 2023) 계보에서 2024-2025년 확장이 이어졌다. DUSt3R가 두 이미지에서 pointmap을 직접 출력해 SfM의 절차 자체를 재정의한 뒤, 같은 Revaud 그룹이 [Cabon et al. 2025. MUSt3R](https://arxiv.org/abs/2503.01661)에서 symmetric multi-view 확장과 working memory를 도입해 이미지 쌍 단위였던 구조를 다수 프레임으로 늘렸다. offline SfM과 online VO/SLAM을 같은 네트워크로 처리할 수 있게 한 시도다. DROID 계열 도구들도 이 생태계 안에서 재활용된다. [Li et al. 2024. MegaSAM](https://arxiv.org/abs/2412.04463)은 DROID-SLAM의 differentiable dense BA를 dynamic scene과 uncalibrated 영상 쪽으로 밀어붙여 camera intrinsic까지 inference 도중 공동 최적화했다. NVIDIA의 [Huang et al. 2025. ViPE](https://arxiv.org/abs/2508.10934)는 DROID-SLAM의 dense flow network와 cuvslam의 sparse point, monocular depth network까지 세 종류의 제약을 하나의 dense BA로 결합해 유튜브 규모의 wild video annotation 파이프라인으로 산업화했다. learned frontend + classical backend라는 2021년 DROID의 설계가 2025년에는 calibration-free와 dynamic scene이라는 더 어려운 조건 위에서 반복되고 있다.

확장은 단일 경로를 따르지 않았다. GO-SLAM처럼 DROID tracking 위에 neural map을 얹는 길, DPV-SLAM처럼 patch odometry로 가볍게 재설계하는 길, NICER-SLAM이나 SplaTAM처럼 implicit/splatting 표현 위에서 tracking을 새로 쓰는 길, MegaSAM·ViPE처럼 DROID의 dense BA를 uncalibrated·dynamic 조건으로 밀어붙이는 길이 동시에 진행됐다. 2021년 Teed와 Deng의 learned frontend + classical backend는 여러 후속 시스템의 출발점이 됐지만, NICER-SLAM과 SplaTAM은 같은 문제를 별도 표현에서 풀어간 경로였다.

> 📜 **예언 vs 실제.** DROID-SLAM은 differentiable BA를 end-to-end 학습과 결합한 hybrid의 기준점을 세웠다. 같은 그룹이 3년 뒤 내놓은 DPV-SLAM은 그 기준점을 efficiency 쪽으로 이어받았다. 반면 GO-SLAM·NICER-SLAM·SplaTAM 계열은 map representation을 implicit 혹은 Gaussian splatting으로 갈아끼우는 쪽으로 갈라져 나갔다. "learned frontend + classical backend"라는 DROID의 설계가 여러 갈래로 변주되는 중이며, 어느 갈래가 범용 해법이 될지는 2026년 현재 아직 결론이 나지 않았다.

---

## 🧭 아직 열린 것

Learned prior의 분포 밖 일반화가 첫 번째 문제다. CodeSLAM과 DeepFactors의 VAE는 훈련 데이터의 depth 분포를 학습한다. 완전히 다른 환경(실외 open-world, 비균질 texture, 야간)에서는 learned prior가 오히려 최적화를 잘못된 방향으로 당길 수 있다. DROID-SLAM의 flow estimator도 훈련 도메인 밖에서 성능이 떨어진다. 2026년 현재, "어떤 환경에서도 작동하는 learned SLAM"은 아직 없다. TartanAir처럼 다양한 합성 데이터로 훈련하는 접근이 있으나 sim-to-real gap이 남는다.

실시간 제약도 여전하다. DROID-SLAM은 NVIDIA RTX 2080Ti 기준으로 평균 10-15 fps 수준이다. keyframe graph 크기에 따라 더 느려진다. dense BA가 병목이다. 모바일 로봇이나 AR/VR처럼 실시간(30Hz+), 저전력 배포가 필요한 응용에서는 2026년 현재도 실용적이지 않다. 경량화 시도들(keyframe 수 줄이기, approximate BA)이 있으나 성능 trade-off가 따른다.

Loop closure의 learned 통합도 미해결이다. DROID-SLAM은 loop closure를 명시적으로 다루지 않는다. Teed는 이후 Handbook 회고에서 "DROID-SLAM doesn't include any relocalization module, so large loops with lots of drift cannot be closed"고 명시했다. 국소 frontend와 달리 backend는 전체 keyframe에 걸쳐 global BA를 수행한다. 다만 별도의 relocalization이 없어 drift가 큰 루프에서 필요한 대응을 복구하는 데 한계가 있다. learned loop closure(10장의 place recognition 연구들)를 DROID의 factor graph에 통합하는 시도가 일부 있으나 아직 단일 시스템으로 수렴하지 않았다. 10장 NetVLAD 계보와 13장 DROID 계보가 만나는 지점이 아직 열려 있다.

---

DROID-SLAM의 inverse depth map은 2021년의 강한 dense representation이었다. 그러나 2020년 [NeRF](https://arxiv.org/abs/2003.08934)(Neural Radiance Field)는 장면을 포인트나 메시가 아니라 연속 함수로 표현하는 다른 가능성을 제시했다. 렌더링이 미분 가능하다면 photometric consistency를 새로운 방식으로 강제할 수 있다.

2021년 Imperial College의 Edgar Sucar는 이를 rendering이 아니라 SLAM의 문제로 다뤘다. MLP가 TSDF voxel grid를 완전히 대체할 수 있는가. 14개월 동안 개발한 iMAP이 그 질문에 대한 답이었다.
