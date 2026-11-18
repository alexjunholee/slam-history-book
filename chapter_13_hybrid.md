# 16장 학습과 기하의 결합: CodeSLAM에서 DROID-SLAM까지

PoseNet과 SfMLearner가 신경망 하나로 SLAM의 여러 단계를 통째로 대신하려던 사이, 런던 임페리얼 칼리지의 Andrew Davison 연구실에서는 다른 질문을 던졌다. 기하 최적화의 틀을 버리는 대신, 그 안에 딥러닝이 배운 압축 표현을 집어넣으면 어떨까. 2018년 CVPR에 나온 CodeSLAM이 그 답이었다. 발표자 Michael Bloesch는 당시 Imperial College London Dyson Robotics Lab의 Dyson Research Fellow로 Davison 그룹에 몸담고 있었고, 그곳은 2011년 Richard Newcombe가 DTAM을 내놓은 바로 그 연구실이었다.

---

## 16.1 CodeSLAM — latent code와 지도

CodeSLAM 이전의 단안 SLAM은 깊이를 직접 최적화 변수로 잡았다. sparse landmark 수백 개를 추적하든 [DTAM](https://www.doc.ic.ac.uk/~ajd/Publications/newcombe_etal_iccv2011.pdf)(Newcombe et al. 2011)처럼 모든 픽셀을 밀집(dense)하게 다루든 마찬가지였다. 특히 dense 방식에서는 변수 개수가 해상도를 그대로 따라갔다. 640×480 keyframe 한 장만 해도 독립 변수가 307,200개에 달하니 최적화가 무겁고 초기값에 민감했으며, 장면에 관한 사전 지식(prior)을 넣기도 까다로웠다.

[Bloesch et al. 2018. CodeSLAM](https://doi.org/10.1109/CVPR.2018.00271)은 이 병목을 표현의 차원을 낮추는 방식으로 풀었다. 픽셀마다 깊이를 따로 최적화하는 대신, depth map을 생성하는 저차원 잠재 벡터(**latent code**)를 변수로 삼은 것이다. 이미지를 조건으로 받는 variational autoencoder(VAE)를 학습시켜 두면, 잠재 공간 안의 점 하나가 그럴듯한 depth map 하나에 대응한다. 덕분에 최적화는 이 저차원 공간 안에서만 움직이며 keyframe당 변수 수를 수십만 개에서 단 128개로 줄였다.

실제 추론에서는 keyframe마다 U-Net으로 뽑은 이미지 feature와 latent code **z**를 decoder에 넣어 dense depth map을 복원한다. **z**는 0에서, 카메라 자세는 단위 행렬(identity)에서 출발해 두 변수를 함께 최적화하며, 시야가 겹치는 두 프레임 사이의 밝기 차이(photometric residual)와 깊이 차이(geometric residual)를 동시에 줄여 나간다.

두 오차를 합친 목적함수는 다음과 같다.

$$E(\mathbf{z}, T) = \sum_{(A,B)} \Bigl[ L_p\bigl(I_A[u] - I_B[w(u, \mathbf{z}_A, T_A^B)]\bigr) + L_g\bigl(D_A[u] - D_B[w(u, \mathbf{z}_A, T_A^B)]\bigr) \Bigr]$$

여기서 $I$는 영상, $D$는 decoder가 이미지와 code로 만든 depth, $T_A^B$는 두 keyframe 사이의 상대 자세다. $w$는 keyframe $A$의 픽셀 $u$를 code $\mathbf{z}_A$로 복원한 깊이와 $T_A^B$를 이용해 $B$의 영상 평면으로 옮기는 warping 연산이고, $L_p$와 $L_g$는 Huber 가중치를 적용해 잘못된 대응이나 가려진 픽셀, 심하게 기운 표면의 영향을 누그러뜨리는 손실 함수다. 원 논문은 식 (13)과 (14)에서 두 residual을 따로 정의하지만, 위 식은 둘을 나란히 묶어 쓴 형태다. CodeSLAM 논문 자체에는 code에 거는 별도 prior 항이 없었고, code를 0 근처로 묶어 두는 zero-code prior factor는 뒤이어 나온 DeepFactors(16.2)에서 들어왔다.

변수를 128개로 압축한 덕분에 CodeSLAM은 적은 자유도만으로 dense depth를 추정할 수 있었다. 속도는 keyframe 4개 구성에서 지도 갱신 5 Hz 수준이었는데, 저자들은 keyframe을 너무 자주 추가하지 않으면 실시간 운용이 가능하다고 보았고 TensorFlow로 짠 image warping의 속도를 끌어올리는 일은 후속 과제로 남겼다.

> 📜 **예언 vs 실제.** CodeSLAM 논문은 이 구성 요소로 완전한 실시간 keyframe 기반 SLAM을 만들고, 실제 영상으로도 학습하겠다고 맺었다. 같은 Imperial 그룹은 2020년 DeepFactors를 실시간 시스템으로 내놓았다.

> 🔗 **차용.** CodeSLAM의 latent depth 표현은 [Kingma & Welling 2013. VAE](https://arxiv.org/abs/1312.6114)의 encoder-decoder 잠재 공간 구조를 썼다. 학습은 VAE 방식을 따르지만, SLAM에 투입할 때는 sampling을 하지 않고 **z**를 직접 최적화 변수로 다룬다.

---

## 16.2 DeepFactors — factor graph 통합

2년 뒤인 2020년, Davison의 박사과정 학생 Jan Czarnowski는 [Czarnowski et al. 2020. DeepFactors](https://doi.org/10.1109/LRA.2020.2965415)를 IEEE RA-L에 발표했다. DeepFactors는 CodeSLAM이 제안한 latent depth 표현과 결합 최적화를 본격적인 factor graph 체계로 옮겨 심은 시스템이다. tracking과 mapping 스레드를 분리하고 keyframe 삽입 기준을 세우자 실시간성이 살아났다. NVIDIA GTX 1080 한 장에서 새 프레임을 최신 keyframe에 정합하는 tracking은 약 250 Hz로 돌아갔다. 반면 새 keyframe을 추가할 때마다 신경망으로 초기 code와 Jacobian을 계산하는 데는 약 340 ms가 걸렸는데, 정작 forward pass는 16 ms에 불과했고 나머지 시간은 대부분 TensorFlow에서 Jacobian을 구하는 데 쓰였다. 저자들은 탐색하듯 움직이는 환경에서 실시간 동작을 확인했으며, 이때는 대체로 photometric 오차와 재투영 오차만 켠 채 운용했다고 적었다.

비슷한 시기 뮌헨 공과대학(TU Munich)의 Daniel Cremers 그룹도 2016년에 내놓은 direct sparse odometry([DSO](https://arxiv.org/abs/1607.02565))에 신경망 예측을 접목하기 시작했다. 출발점은 [Yang, Wang, Stückler, Cremers 2018. DVSO](https://arxiv.org/abs/1807.02570)로, 신경망이 예측한 깊이를 단안 DSO에 "가상 stereo" 관측처럼 넣어 주었다. 이어진 [Yang, von Stumberg, Wang, Cremers 2020. D3VO](https://arxiv.org/abs/2003.01060)는 자기지도 학습으로 얻은 깊이·자세·불확실성 예측까지 DSO 최적화의 factor로 합쳤고, [Wimbauer et al. 2021. MonoRec](https://arxiv.org/abs/2011.11814)과 [Wimbauer et al. 2023. Behind the Scenes](https://arxiv.org/abs/2301.07668)는 이 계보를 동적 장면의 dense 재구성과 단일 영상 기반 density field 추정으로까지 넓혔다. Imperial과 TUM은 각자 독자적인 경로를 걸었지만, 신경망 예측을 기존 최적화 프레임워크 안에 factor로 녹여 넣는다는 점에서는 같은 방향을 보고 있었다. 2021년 Princeton에서 등장한 DROID-SLAM 역시 이 흐름 위에서 탄생했다.

> 🔗 **차용.** DeepFactors의 mapping은 Dellaert 연구실의 factor graph 라이브러리 [GTSAM](https://gtsam.org/tutorials/intro.html)과 증분 최적화 알고리즘 iSAM2(Kaess et al.) 위에서 돌아간다. CodeSLAM이 한데 묶어 최적화하던 자세와 latent code를 Czarnowski는 factor graph의 변수 노드로 다시 세웠다. VAE 학습 과정에서 code가 평균 0인 Gaussian 근처로 모이는 성질 덕분에, 최적화에서도 zero-code prior factor를 두어 code를 그 근처에 붙들어 둔다.

---

## 16.3 RAFT — recurrent optical flow

DROID-SLAM의 직접적인 출발점은 SLAM이 아니라 optical flow 논문이었다. Zachary Teed와 Jia Deng(Princeton)이 2020년 ECCV에 발표한 [Recurrent All-Pairs Field Transforms(RAFT)](https://arxiv.org/abs/2003.12039)는 두 영상 사이의 픽셀 움직임을 추정하는 네트워크로, 크게 세 단계로 나뉜다.

1. Feature encoder: CNN이 두 이미지에서 feature map을 추출한다.
2. Correlation volume: 모든 픽셀 쌍 사이의 유사도로 4-level pyramid의 4D volume을 만든다.
3. Update operator: Gated Recurrent Unit(GRU)이 correlation volume을 lookup하며 flow field를 반복해서 고친다.

이름에 붙은 all-pairs는 두 이미지의 모든 픽셀 쌍에 대한 유사도를 미리 계산해 둔다는 뜻이다. PWC-Net 같은 기존 coarse-to-fine 방식이 해상도를 단계적으로 바꿔 가며 flow를 추정했다면, RAFT는 flow field를 원영상 가로·세로의 1/8 해상도 하나로 고정해 둔 채 correlation pyramid를 조회하며 반복적으로 값을 다듬고 마지막에만 upsampling한다. 이 단순하면서도 집요한 반복 구조 덕분에 RAFT는 KITTI의 F1-all 오류를 당시 최고 기록(6.10%)보다 16% 낮춘 5.10%로 끌어내렸고, Sintel final pass의 endpoint error도 30% 줄였다.

Teed와 Deng은 바로 이 반복 갱신 아이디어를 SLAM으로 가져왔다. 두 프레임 사이의 optical flow를 고치던 GRU 루프가 여러 프레임의 카메라 자세와 깊이를 함께 고치는 루프로 확장되었고, 각 반복의 기하 계산은 미분 가능한 Dense Bundle Adjustment(DBA) layer가 맡았다.

---

## 16.4 DROID-SLAM — update operator와 BA

2021년 NeurIPS에 발표된 [Teed & Deng. DROID-SLAM](https://arxiv.org/abs/2108.10869)에서 DROID는 "Differentiable Recurrent Optimization-Inspired Design"의 약자다. 이름 그대로 미분 가능한 최적화 루프를 신경망 구조 안에 심었다는 선언이었다.

frontend는 RAFT의 골격을 그대로 이어받는다. CNN encoder가 feature map을 뽑으면 all-pairs correlation volume을 세우고, GRU update operator가 dense flow를 반복 추정한다. 다만 두 장의 이미지에만 매달리던 RAFT와 달리, DROID-SLAM은 이 추정을 keyframe graph의 모든 edge 위에서 동시에 돌린다.

이때 backend에 해당하는 **DBA layer**가 GRU와 기하 최적화를 하나로 엮는다. GRU가 내놓은 flow 수정량과 신뢰도 가중치가 DBA에 들어가면, DBA는 이를 제약 조건 삼아 카메라 자세와 inverse depth를 Schur complement로 빠르게 푼다. 이어 DBA가 자세와 깊이를 새로 고쳐 놓으면, 그 갱신된 기하 상태가 다음 GRU 반복에서 correlation volume을 조회하는 기준점이 된다. 학습된 대응 추정과 엄밀한 3D 최적화가 매 반복마다 서로를 교정하는 셈이다.

성능 차이는 벤치마크에서 선명하게 드러났다. EuRoC 단안 실험에서 DROID-SLAM의 ATE는 ORB-SLAM3가 추적을 끝까지 유지한 10개 시퀀스만 골라 비교해도 43% 낮았고, 스테레오 입력에서는 격차가 71%까지 벌어졌다. 더 놀라운 점은 일반화였다. 학습은 합성 데이터셋 TartanAir만으로 마쳤는데도 TartanAir·EuRoC·TUM-RGBD에서 추적이 끊긴 시퀀스가 단 하나도 없었으며, 까다로운 ETH-3D RGB-D에서는 32개 시퀀스 중 30개를 완주해 차선책(19개)을 크게 앞섰다. Teed와 Deng 등이 집필한 *The SLAM Handbook* 13장은 EuRoC V1_02 시퀀스에서 global optimization이 RMSE ATE를 16.5 cm에서 1.2 cm까지 줄였다고 기록한다.

PoseNet이 기하 제약 없이 자세를 직접 회귀했다가 학습 장면을 벗어나면 무너졌던 것과 달리, DROID-SLAM은 역할을 명확히 갈랐다. 신경망은 feature 추출과 dense correspondence에만 집중하고, 기하 제약과 전역 일관성은 bundle adjustment가 책임진다.

> 🔗 **차용.** 영상의 모든 픽셀을 동원해 자세와 깊이를 동시에 추정한다는 발상은 Newcombe의 DTAM(2011)으로 거슬러 올라간다. DTAM은 tracking과 depth 추정을 번갈아 풀었고, DROID-SLAM은 신경망이 예측한 flow와 신뢰도를 입력받아 dense BA를 수행한다.

---

## 16.5 Imperial Dyson Robotics Lab의 연구자들

앞에서 살펴본 CodeSLAM, DeepFactors, 그리고 다음 장에서 다룰 iMAP까지, 학습과 기하를 결합한 초기 계보의 상당 부분이 한 연구실에서 나왔다. Andrew Davison은 2003년 Oxford에서 MonoSLAM을 발표한 뒤 2005년 Imperial College로 옮겨 Robot Vision Group을 꾸렸고, 2014년에는 Dyson Robotics Lab을 세워 소장을 맡았다. 이 연구실을 거쳐 간 주요 연구자와 작품은 다음과 같다.

- **Richard Newcombe** (Davison 지도, Imperial): DTAM(2011), [KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf)(2011). 이후 Oculus→Meta Reality Labs
- **Michael Bloesch** (Dyson Research Fellow, Imperial, 2017–2019): CodeSLAM(2018). 그 전에는 ETH Zurich에서 박사과정을 하며 visual-inertial odometry(ROVIO)를 연구했고, 그 뒤 DeepMind로 옮김
- **Jan Czarnowski** (Davison 지도, Imperial): DeepFactors(2020)
- **Edgar Sucar** (Davison 그룹, Imperial): [iMAP](https://arxiv.org/abs/2103.12352)(2021). neural implicit SLAM의 초기 연구
- **Tristan Laidlow** (Davison 그룹, Imperial): DeepFactors(2020) 공저

이 연구실의 궤적에는 일관된 철학이 흐른다. 확률적 추정과 불확실성 모델링을 놓지 않으면서도, 단안 sparse 지도에서 dense latent 표현으로, 다시 implicit 표현으로 지도의 그릇을 옮겨 왔다. Davison이 [FutureMapping](https://arxiv.org/abs/1803.11288)(2018)과 [FutureMapping 2](https://arxiv.org/abs/1910.14139)(2019, Ortiz 공저)에서 제시한 Spatial AI의 청사진—여러 기하·의미 표현을 하나의 확률 그래프 안으로 묶자는 구상—에서 CodeSLAM과 DeepFactors는 첫 실험실 무대였다.

---

## 16.6 2023-2025 — DROID-SLAM 이후

DROID-SLAM이 보여 준 미분 가능 DBA 구조는 2023년 이후 여러 방향으로 갈라지며 재사용됐다. 한쪽에서는 DROID-SLAM의 정밀한 tracking 위에 새로운 3D 표현 지도를 얹기 시작했다. University of Bologna의 [GO-SLAM](https://arxiv.org/abs/2309.02436)(Zhang et al., ICCV 2023)은 DROID-SLAM의 tracking에 online loop closing과 full bundle adjustment를 보태고, 지도는 Instant-NGP 스타일의 multi-resolution hash encoding 기반 neural implicit 표현으로 바꿨다. 같은 해 나온 [NICER-SLAM](https://arxiv.org/abs/2302.03594)(Zhu et al., arXiv 2023 / 3DV 2024)은 DROID-SLAM 구조를 그대로 따르지는 않았지만 RGB만으로 카메라 자세와 계층적 neural implicit 지도를 함께 최적화한다는 목표를 공유하며 단안 기하 단서와 optical flow를 추가 감독 신호로 삼았고, CMU의 [SplaTAM](https://arxiv.org/abs/2312.02126)(Keetha et al., CVPR 2024)은 아예 별개의 시스템으로 RGB-D 지도를 3D Gaussian Splatting으로 옮기고 silhouette mask 기반 differentiable rendering tracking을 새로 짰다.

Princeton 그룹 자신은 DROID-SLAM의 무거운 계산량을 덜어내는 데 집중했다. [DPV-SLAM](https://arxiv.org/abs/2408.01654)(Lipson, Teed, Deng, ECCV 2024)은 전작 [DPVO](https://github.com/princeton-vl/DPVO)(Deep Patch Visual Odometry)에 근접 기반 loop closure와 고전적 loop closure를 결합한 시스템이다. 근접 기반 loop closure는 추정한 카메라 위치가 가까운 프레임들을 이어 CUDA로 구현한 block-sparse BA로 풀고, 먼 루프는 image retrieval과 포즈 그래프 최적화로 닫는다. 그 결과 EuRoC와 TartanAir에서 DROID-SLAM과 맞먹는 정확도를 유지하면서도 EuRoC 기준 2.5배 빠르고 GPU 메모리도 5~7 GB에 그쳐 GPU 한 장으로 충분히 돌아갔다.

한편 DROID-SLAM과 별개로, 이미지 쌍에서 3D pointmap(픽셀마다 3D 좌표를 담은 지도)을 곧장 출력하는 Naver Labs Europe의 [DUSt3R](https://arxiv.org/abs/2312.14132)(Wang et al., CVPR 2024) 계열도 2024–2025년 빠르게 확장됐다. 같은 연구소(Cabon, Revaud 등)는 [Cabon et al. 2025. MUSt3R](https://arxiv.org/abs/2503.01661)에서 DUSt3R 구조를 대칭형으로 고치고 다층 memory를 붙여 쌍 단위 추론을 다중 프레임 좌표계로 넓혔으며, 오프라인 SfM과 온라인 visual SLAM을 같은 네트워크 하나로 다룰 수 있음을 보였다.

그러는 사이 DROID-SLAM의 differentiable dense BA 엔진 자체는 범용 영상 처리 파이프라인의 핵심 부품으로 자리를 잡았다. Google DeepMind 등의 [Li et al. 2025. MegaSaM](https://arxiv.org/abs/2412.04463)(CVPR 2025)은 이 DBA를 동적 물체가 뒤섞인 일상 영상에 투입하면서 초점 거리(focal length)까지 함께 최적화해 비보정 영상을 풀어냈다. NVIDIA의 [Huang et al. 2025. ViPE](https://arxiv.org/abs/2508.10934)는 한 걸음 더 나아가 DROID-SLAM의 dense flow 제약, cuVSLAM의 sparse point 추적, 단안 metric depth 네트워크 출력을 단일 dense BA에 묶어 인터넷 동영상 약 10만 개를 포함한 대규모 영상에 카메라 자세와 깊이를 달아 주었다.

> 📜 **예언 vs 실제.** DROID-SLAM은 미분 가능한 BA와 end-to-end 학습을 묶었다. 2026년 현재 학습한 frontend와 고전적 backend를 결합하는 설계는 여러 변형으로 널리 쓰이지만, 어느 조합이 범용 표준이 될지는 아직 정해지지 않았다.

---

## 아직 열린 문제

**학습한 prior의 일반화.** CodeSLAM과 DeepFactors의 VAE는 훈련 데이터의 깊이 분포를 외운다. 실외처럼 학습 분포와 동떨어진 환경에서는 오히려 learned prior가 최적화를 그릇된 방향으로 끌어당길 수 있고, DROID-SLAM의 flow estimator 역시 훈련 도메인 밖에서는 신뢰도가 흔들린다. TartanAir처럼 다양한 합성 데이터로 방어하는 전략이 쓰이지만 합성과 실제 사이의 간극(sim-to-real gap)은 여전히 남아 있다.

**실시간 제약.** DROID-SLAM 원 논문은 실시간 구동을 위해 RTX 3090 두 장—한 장은 tracking과 local BA, 다른 한 장은 global BA와 loop closure—을 요구했다. 그마저도 EuRoC에서는 해상도를 320×512로 낮추고 프레임을 건너뛰어 평균 20 fps를 냈을 뿐, 카메라 움직임이 거친 TartanAir에서는 평균 8 fps에 머물렀다. DPV-SLAM이 GPU 한 장 수준으로 문턱을 낮추긴 했으나, 모바일 로봇이나 AR/VR 기기 같은 저전력 환경에 올리기에는 여전히 계산량이 무겁다.

**Loop closure.** DROID-SLAM은 카메라가 지나온 곳으로 돌아오면 frame graph에 원거리 edge를 추가해 루프를 닫는다. 이때 backend는 현재 추정한 자세와 깊이로 프레임 쌍 사이의 optical flow를 계산해 기준치 이하인 쌍만 연결하므로, drift가 작을 때만 루프가 닫힌다. Teed와 Deng 등이 *The SLAM Handbook* 13장에서 *"DROID-SLAM doesn't include any relocalization module, so large loops with lots of drift cannot be closed"*라고 짚었듯, 큰 drift가 쌓인 뒤 재위치 추정을 하기는 어렵다. DPV-SLAM이 이 자리에 image retrieval 기반 고전 loop closure를 따로 붙인 것도 그 때문이다. 학습 기반 place recognition을 이런 dense BA 구조 안에 매끄럽게 통합하는 일은 아직 숙제로 남아 있다.

---

DROID-SLAM은 keyframe마다 inverse depth map을 얹어 장면을 기록했다. 그런데 비슷한 시기, 장면을 아예 하나의 연속 함수로 통째로 학습하는 전혀 다른 길이 열리고 있었다. 2020년 등장한 [NeRF](https://arxiv.org/abs/2003.08934)(Neural Radiance Field)는 미분 가능한 렌더링을 통해 렌더링 결과와 실제 영상의 차이를 줄이면서 장면과 카메라 자세를 함께 최적화할 수 있는 가능성을 보여 주었다. 이듬해인 2021년, Imperial College의 Edgar Sucar는 이 연속 표현을 SLAM으로 끌어들여 TSDF voxel grid 대신 단 하나의 MLP(multilayer perceptron, 다층 퍼셉트론)로 방 하나를 기억하는 iMAP을 내놓았다.
