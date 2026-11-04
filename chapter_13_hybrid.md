# Ch.13 — 학습과 기하의 결합: CodeSLAM에서 DROID-SLAM까지

Michael Bloesch는 2018년 CVPR에 CodeSLAM을 발표했다. 그때 그는 Imperial College London Dyson Robotics Lab의 Dyson Research Fellow로 Andrew Davison 그룹에 있었다. 2011년 Richard Newcombe가 DTAM을 만든 곳도 Davison 그룹이다. CodeSLAM은 Davison 그룹의 확률적 SLAM에 딥러닝으로 배운 표현을 넣었다.

---

## 13.1 CodeSLAM — latent code와 지도

기존 monocular SLAM은 depth를 직접 추정했다. sparse landmark 수백 개를 쓰든 [DTAM](https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf)(Newcombe et al. 2011)처럼 모든 픽셀을 쓰든 depth가 최적화 변수였다. dense 방식이면 변수 수가 이미지 해상도에 비례한다. 640×480 keyframe 한 장의 dense depth map에는 독립 변수가 307,200개 있다. 변수가 이만큼 많으면 최적화가 무겁고 초기값에 민감하며, prior를 넣기도 어렵다.

[Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271)은 depth map 대신 depth map을 만들어 내는 저차원 잠재 벡터(**latent code**)를 최적화했다. 이미지를 조건으로 받는 variational autoencoder(VAE)를 학습하면, latent code 공간의 점 하나가 그럴듯한 depth map 하나에 대응한다. 최적화는 이 공간 안에서만 움직이므로 keyframe당 변수가 수십만 개에서 128개로 줄어든다.

> 🔗 **차용.** CodeSLAM의 latent depth 표현은 [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114)의 encoder-decoder 잠재 공간 구조를 썼다. 학습은 VAE 방식을 따른다. SLAM에서 쓸 때는 sampling을 하지 않고 **z**를 최적화 변수로 다룬다.

keyframe마다 decoder가 그 이미지의 feature(U-Net으로 뽑은 것)와 latent code **z**로 dense depth map을 만든다. **z**는 0에서, 자세는 identity에서 시작해 함께 최적화한다. 시야가 겹치는 두 프레임 사이에서 밝기 차이(photometric residual)와 depth 차이(geometric residual)를 함께 줄인다.

목적함수는 다음과 같다:

$$E(\mathbf{z}, T) = \sum_{(A,B)} \Bigl[ L_p\bigl(I_A[u] - I_B[w(u, \mathbf{z}_A, T_A^B)]\bigr) + L_g\bigl(D_A[u] - D_B[w(u, \mathbf{z}_A, T_A^B)]\bigr) \Bigr]$$

$I$는 영상, $D$는 decoder가 이미지와 code로 만든 depth, $T_A^B$는 두 keyframe 사이의 상대 pose다. $w$는 keyframe $A$의 픽셀 $u$를 code $\mathbf{z}_A$로 만든 depth와 $T_A^B$로 $B$에 옮기는 warping이다. $L_p$, $L_g$는 Huber 가중치를 주고 잘못된 대응, 가려진 픽셀, 심하게 기운 면의 오차를 줄이는 손실 함수다. 원 논문은 식 (13)과 (14)에서 두 residual을 따로 정의한다. 위 식은 둘을 합쳐 쓴 것이다. CodeSLAM 논문은 code에 따로 prior 항을 두지 않았다. code를 0 근처에 묶어 두는 zero-code prior factor는 DeepFactors(13.2)에서 들어왔다.

CodeSLAM은 적은 변수로 dense depth를 추정했다. 속도는 keyframe 4개 구성에서 지도 갱신 5 Hz였다. 저자들은 keyframe을 너무 자주 추가하지 않으면 실시간에 충분하다고 적었다. TensorFlow로 구현한 image warping을 더 빠르게 만드는 일은 향후 과제로 남겼다.

> 📜 **예언 vs 실제.** CodeSLAM 논문은 이 구성 요소로 완전한 실시간 keyframe 기반 SLAM을 만들고, 실제 영상으로도 학습하겠다고 맺었다. 같은 Imperial 그룹은 2020년 DeepFactors를 실시간 시스템으로 내놓았다.

---

## 13.2 DeepFactors — factor graph 통합

2020년 Davison의 박사과정 학생 Jan Czarnowski는 [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2965415)를 IEEE RA-L에 발표했다.

DeepFactors는 CodeSLAM의 latent depth 표현과 결합 최적화를 factor graph로 옮겼다. tracking과 mapping을 나누고 keyframe을 추가하는 기준도 두었다. NVIDIA GTX 1080 한 장에서 새 영상을 최신 keyframe에 맞추는 tracking은 약 250 Hz로 돌았다. 새 keyframe을 만들 때마다 신경망을 돌려 첫 code와 Jacobian을 구했는데, 이 계산에 약 340 ms가 걸렸다. 그중 forward pass는 16 ms였고 나머지는 TensorFlow로 Jacobian을 구하는 데 들었다. 저자들은 탐색하듯 움직일 때 시스템이 실시간으로 돌았고, 이때는 보통 photometric 오차와 재투영 오차만 켠다고 적었다.

> 🔗 **차용.** DeepFactors의 mapping은 Dellaert 연구실의 factor graph 라이브러리 [GTSAM](https://gtsam.org/tutorials/intro.html)과 그 증분 최적화 알고리즘 iSAM2(Kaess et al.)를 쓴다. CodeSLAM이 함께 최적화하던 자세와 latent code를 Czarnowski는 factor graph의 변수로 다시 세웠다. VAE가 학습 중 code를 평균 0인 Gaussian 근처로 모으므로, 최적화 중에도 zero-code prior factor로 code를 그 근처에 묶어 둔다.

같은 시기 TU Munich의 Daniel Cremers 그룹은 2016년에 내놓은 direct sparse odometry([DSO](https://arxiv.org/abs/1607.02565))에 신경망 예측을 더했다. [Yang, Wang, Stückler, Cremers 2018. DVSO](https://arxiv.org/abs/1807.02570)는 신경망이 예측한 depth를 단안 DSO에 "가상 stereo" 관측으로 넣었다. [Yang, von Stumberg, Wang, Cremers 2020. D3VO](https://arxiv.org/abs/2003.01060)는 self-supervised로 학습한 depth, pose, uncertainty 예측을 DSO의 최적화에 factor로 더했다. [Wimbauer et al. 2021. MonoRec](https://arxiv.org/abs/2011.11814)과 [Wimbauer et al. 2023. Behind the Scenes](https://arxiv.org/abs/2301.07668)는 이 연구를 동적 장면의 dense 재구성과, 이미지 한 장으로 density field를 추정하는 문제로 넓혔다. Imperial과 TUM은 따로 연구하면서 둘 다 신경망 예측을 기존 최적화 구조에 넣었다. 2021년 Princeton에서 나온 DROID-SLAM도 이렇게 설계했다.

---

## 13.3 RAFT — recurrent optical flow

Zachary Teed와 Jia Deng(Princeton)은 2020년 ECCV에 [Recurrent All-Pairs Field Transforms(RAFT)](https://arxiv.org/abs/2003.12039)를 발표했다. RAFT는 optical flow 추정 논문이다. 이 설계가 뒤에 DROID-SLAM에 들어갔다. 구조는 다음과 같다.

1. Feature encoder: CNN이 두 이미지에서 feature map을 추출한다.
2. Correlation volume: 모든 픽셀 쌍 간의 유사도를 4-level pyramid의 4D volume으로 구성한다.
3. Update operator: Gated Recurrent Unit(GRU)이 correlation volume을 lookup하며 flow field를 반복해서 고친다.

이름의 all-pairs는 모든 픽셀 쌍의 유사도를 미리 계산해 둔다는 뜻이다. PWC-Net 같은 coarse-to-fine 방법은 해상도를 바꿔 가며 flow를 추정한다. RAFT는 flow field를 원영상 가로·세로의 1/8인 해상도 하나로 유지한 채 correlation pyramid를 lookup하고, 마지막에 출력을 upsampling한다. RAFT 논문은 KITTI의 F1-all 오류를 5.10%로 낮췄다고 보고했다. 논문이 당시 공개된 최고 결과로 든 6.10%보다 16% 낮다. Sintel final pass의 endpoint error는 30% 낮췄다.

DROID-SLAM 논문은 RAFT의 반복 갱신 구조를 이어받았다. 반복할 때마다 고치는 대상은 두 프레임 사이의 flow에서 여러 프레임의 카메라 자세와 depth로 바뀌었다. 각 갱신은 미분 가능한 Dense Bundle Adjustment(DBA) layer가 계산한다.

---

## 13.4 DROID-SLAM — update operator와 BA

[Teed & Deng. DROID-SLAM](https://arxiv.org/abs/2108.10869)은 2021년 NeurIPS에서 발표됐다. DROID는 "Differentiable Recurrent Optimization-Inspired Design"의 약자다.

frontend는 RAFT와 구조가 같다. CNN encoder가 feature map을 뽑으면 all-pairs correlation volume을 만들고, GRU update operator가 dense flow를 반복 추정한다. DROID-SLAM은 이 추정을 keyframe graph의 모든 edge에서 동시에 한다.

DBA의 최적화 변수는 자세와 inverse depth다. flow 추정으로 얻은 2D 대응을 제약으로 삼아 자세와 depth를 함께 최적화한다. 선형 시스템은 Schur complement로 빠르게 푼다.

**DBA layer**가 GRU와 최적화를 잇는다. GRU가 추정한 flow 수정량과 신뢰도 가중치가 DBA에 들어간다. DBA가 pose와 depth를 갱신하면, 그 결과로 다음 GRU 반복의 기준이 바뀐다.

> 🔗 **차용.** 모든 픽셀로 자세와 depth를 추정한다는 생각은 Newcombe의 DTAM(2011)에도 있었다. DTAM은 tracking과 depth 추정을 번갈아 했다. DROID-SLAM은 학습한 flow를 입력으로 dense BA를 한다.

EuRoC 단안 실험에서 DROID-SLAM의 ATE는 ORB-SLAM3가 성공한 10개 시퀀스만 놓고 비교해도 43% 낮았다. 스테레오 입력에서는 71% 낮았다. 학습은 합성 데이터셋 TartanAir로만 했는데, TartanAir·EuRoC·TUM-RGBD에서 추적에 실패한 시퀀스가 하나도 없었다. ETH-3D RGB-D에서는 32개 중 30개를 추적했고, 다음으로 좋은 방법은 19개였다. Teed와 Deng 등이 쓴 SLAM Handbook 13장은 EuRoC V1_02 시퀀스에서 global optimization이 RMSE ATE를 16.5 cm에서 1.2 cm로 줄였다고 적는다.

PoseNet은 기하 제약 없이 자세를 바로 회귀했고 일반화가 잘 되지 않았다. DROID-SLAM에서는 신경망이 feature 추출과 dense correspondence를 맡고, BA가 기하 제약과 일관성을 맡는다.

---

## 13.5 Imperial Dyson Robotics Lab의 연구자들

Andrew Davison은 2003년 Oxford에서 MonoSLAM을 발표했고, 2005년 Imperial College로 옮겨 Robot Vision Group을 세웠다. 2014년에는 Dyson Robotics Lab을 세워 소장을 맡았다. 이 그룹에서 연구한 학생과 연구원은 다음과 같다.

- **Richard Newcombe** (Davison 지도, Imperial): DTAM(2011), [KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf)(2011). 이후 Oculus→Meta Reality Labs
- **Michael Bloesch** (Dyson Research Fellow, Imperial, 2017–2019): CodeSLAM(2018). 그 전에는 ETH Zurich에서 박사과정을 하며 visual-inertial odometry(ROVIO)를 연구했고, 그 뒤 DeepMind로 옮김
- **Jan Czarnowski** (Davison 지도, Imperial): DeepFactors(2020)
- **Edgar Sucar** (Davison 그룹, Imperial): [iMAP](https://arxiv.org/abs/2103.12352)(2021). neural implicit SLAM의 초기 연구
- **Tristan Laidlow** (Davison 그룹, Imperial): DeepFactors(2020) 공저

이 그룹은 확률적 추정과 불확실성을 중시하면서 단안 sparse 지도에서 dense latent 표현으로, 다시 implicit 표현으로 옮겨 갔다. Davison은 [FutureMapping](https://arxiv.org/abs/1803.11288)(2018)과 [FutureMapping 2](https://arxiv.org/abs/1910.14139)(2019, Ortiz 공저)에서 Spatial AI 시스템에 필요한 계산 구조와 표현을 제안했다. 여러 기하·의미 표현을 확률 그래프 하나로 묶자는 주장이다. CodeSLAM과 DeepFactors는 이 생각을 실험한 초기 연구다.

---

## 13.6 2023-2025 — DROID-SLAM 이후

University of Bologna의 [GO-SLAM](https://arxiv.org/abs/2309.02436)(Zhang et al., ICCV 2023)은 DROID-SLAM의 tracking에 online loop closing과 full bundle adjustment를 더했다. 지도는 Instant-NGP처럼 multi-resolution hash encoding을 쓰는 neural implicit 표현으로 만들었다.

[NICER-SLAM](https://arxiv.org/abs/2302.03594)(Zhu et al., arXiv 2023 / 3DV 2024)은 카메라 자세와 계층적 neural implicit 지도를 함께 최적화했다. RGB만 쓰는 dense SLAM을 목표로 한 점은 DROID-SLAM과 같다. 구조는 DROID-SLAM을 따르지 않고, 단안 기하 단서와 optical flow를 추가 감독 신호로 쓴다.

CMU의 [SplaTAM](https://arxiv.org/abs/2312.02126)(Keetha et al., CVPR 2024)은 RGB-D 입력으로 지도를 3D Gaussian Splatting으로 만들었다. tracking도 silhouette mask를 쓰는 differentiable rendering으로 새로 짰다. DROID-SLAM과는 별개의 시스템이다.

[DPV-SLAM](https://arxiv.org/abs/2408.01654)(Lipson, Teed, Deng, ECCV 2024)은 DROID-SLAM과 같은 Princeton 그룹에서 나왔다. [DPVO](https://github.com/princeton-vl/DPVO)(Deep Patch Visual Odometry)에 근접 기반 loop closure와 고전적 loop closure를 더했다. 근접 기반 loop closure는 추정한 카메라 위치가 가까운 프레임끼리 잇고, CUDA로 짠 block-sparse BA로 푼다. 고전적 loop closure는 image retrieval과 pose graph 최적화를 쓴다. 저자들은 EuRoC와 TartanAir에서 DROID-SLAM과 비슷한 정확도를 내면서 EuRoC 기준 2.5배 빠르고 VRAM은 4분의 1만 쓴다고 보고했다. GPU 한 장으로 돈다.

DROID-SLAM과 별개로 Naver Labs Europe의 [DUSt3R](https://arxiv.org/abs/2312.14132)(Wang et al., CVPR 2024)에서 출발한 연구가 2024–2025년에 이어졌다. DUSt3R는 이미지 두 장에서 pointmap(픽셀마다 3D 점을 담은 지도)을 바로 출력한다. 같은 그룹(Cabon, Revaud 등)은 [Cabon et al. 2025. MUSt3R](https://arxiv.org/abs/2503.01661)에서 DUSt3R 구조를 대칭형으로 고치고 여러 층의 memory를 더했다. 그래서 이미지 쌍 단위였던 구조가 여러 프레임을 한 좌표계에서 처리하게 됐다. 저자들은 offline SfM과 online visual SLAM에 같은 네트워크를 쓸 수 있다고 밝혔다.

DROID-SLAM의 구성 요소는 다른 시스템에서도 다시 쓰였다. Google DeepMind 등의 [Li et al. 2025. MegaSaM](https://arxiv.org/abs/2412.04463)(CVPR 2025)은 DROID-SLAM 방식의 differentiable dense BA를 동적 장면이 담긴 일상 영상에 적용했다. 초점 거리를 모르는 영상도 다루도록 focal length까지 함께 최적화했다. NVIDIA의 [Huang et al. 2025. ViPE](https://arxiv.org/abs/2508.10934)는 DROID-SLAM의 dense flow 제약, cuVSLAM으로 추적한 sparse point, 단안 metric depth network가 낸 depth를 dense BA 하나에 함께 넣었다. NVIDIA는 이 파이프라인으로 인터넷 동영상 약 10만 개를 포함한 대규모 영상에 카메라 자세와 depth를 달았다.

> 📜 **예언 vs 실제.** DROID-SLAM은 미분 가능한 BA와 end-to-end 학습을 묶었다. 2026년 현재 학습한 frontend와 고전적 backend를 묶는 설계는 여러 형태로 쓰인다. 어느 형태가 범용 해법이 될지는 정해지지 않았다.

---

## 🧭 아직 열린 것

**학습한 prior의 일반화.** CodeSLAM과 DeepFactors의 VAE는 훈련 데이터의 depth 분포를 학습한다. 실외처럼 훈련 데이터와 크게 다른 환경에서는 learned prior가 최적화를 잘못된 방향으로 끌고 갈 수 있다. DROID-SLAM의 flow estimator도 훈련 도메인 밖에서 성능이 떨어진다. TartanAir처럼 다양한 합성 데이터로 학습하는 방법이 있지만 합성과 실제 사이의 차이(sim-to-real gap)가 남는다.

**실시간 제약.** DROID-SLAM 논문은 RTX 3090 두 장이면 실시간으로 돈다고 적었다. 한 장은 tracking과 local BA를, 다른 한 장은 global BA와 loop closure를 맡았다. EuRoC에서는 해상도를 320×512로 줄이고 프레임을 하나씩 건너뛰어 평균 20 fps를 냈다. 카메라가 빠르게 움직이는 TartanAir에서는 평균 8 fps로 실시간에 못 미쳤다. DPV-SLAM은 GPU 한 장에서 돌도록 구조를 바꿨다. 모바일 로봇이나 AR/VR처럼 저전력 기기에서 돌려야 하는 응용에는 여전히 계산량이 부담이다.

**Loop closure.** DROID-SLAM은 카메라가 지도로 만든 곳에 돌아오면 frame graph에 먼 거리 edge를 더해 loop를 닫는다. backend는 추정한 자세와 depth로 모든 프레임 쌍의 optical flow를 계산하고, flow가 기준보다 작은 쌍만 잇는다. 그래서 drift가 크지 않을 때만 loop가 닫힌다. Teed와 Deng 등이 쓴 SLAM Handbook 13장은 "DROID-SLAM doesn't include any relocalization module, so large loops with lots of drift cannot be closed"라고 적는다. DPV-SLAM은 이 자리에 image retrieval 기반의 고전적 loop closure를 따로 붙였다. Ch.10의 학습 기반 place recognition을 이런 dense BA 시스템과 한 구조로 묶는 일은 남은 과제다.

---

DROID-SLAM은 keyframe마다 inverse depth map으로 장면을 표현했다. 2020년에 나온 [NeRF](https://arxiv.org/abs/2003.08934)(Neural Radiance Field)는 장면을 연속 함수로 표현한다. 렌더링이 미분 가능하므로, 렌더링한 영상과 실제 영상의 차이를 줄이는 방향으로 장면과 자세를 최적화할 수 있다.

2021년 Imperial College의 Edgar Sucar는 이 표현을 SLAM에 가져와, TSDF voxel grid 대신 MLP(multilayer perceptron, 다층 퍼셉트론) 하나로 장면을 표현하는 iMAP을 만들었다.
