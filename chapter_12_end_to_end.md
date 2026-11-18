# 15장 종단간 학습: PoseNet에서 SfMLearner까지

단안 깊이 예측이 CNN의 날개를 달고 날아오르던 바로 그 시기인 2015년부터 2018년 사이, 한 무리의 연구자들은 훨씬 더 급진적인 질문을 던졌다. 특징점을 뽑고, 매칭하고, RANSAC을 돌리고, 비선형 최적화를 푸는 복잡한 SLAM 파이프라인을 전부 치워 버리고, 입력 영상에서 카메라 자세까지 단 하나의 신경망으로 종단간(end-to-end) 학습할 수는 없을까?

그 첫 포문을 연 것은 2015년 케임브리지 대학교 박사과정생 Alex Kendall이었다. Roberto Cipolla의 지도를 받던 Kendall은 ImageNet과 Places 데이터셋으로 사전 학습된 GoogLeNet을 가져와, 케임브리지 시내를 걸으며 찍은 영상과 SfM으로 만든 자세 레이블로 미세 조정(fine-tuning)했다. 그렇게 사진 한 장을 넣으면 곧바로 6-DoF 카메라 자세를 뱉어 내는 신경망 [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336)이 산티아고에서 열린 ICCV 2015에 등장했다. 이후 몇 년간 특징 추출, 매칭, 최적화, 심지어 지도 관리까지 통째로 CNN이나 RNN 안에 밀어 넣으려는 야심 찬 논문들이 봇물처럼 쏟아졌다.

---

## 15.1 PoseNet

[AlexNet(Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks)의 대성공 이후, 이미지 분류로 훈련된 깊은 CNN이 단순한 텍스처를 넘어 고수준 시각 표현을 포착한다는 사실이 상식이 됐다. Kendall은 바로 그 분류망의 표현력을 기하학적 카메라 자세 추정으로 옮겨 심었다.

연구팀은 King's College, Street, Old Hospital, Shop Façade, St Mary's Church 등 케임브리지 시내 야외 풍경을 직접 촬영해 Cambridge Landmarks 데이터셋을 구축했다. 여기서 PoseNet은 특징점 추출이나 RANSAC, 3D 지도 조회 없이 프레임당 약 5 ms 만에 자세를 내놓으며, 장면에 따라 1.5~3.7 m의 위치 오차와 5~8.5° 수준의 방향 오차를 기록했다(원 논문 §5).

뒤이어 오차를 줄이고 불확실성을 붙이려는 변주들이 잇따랐다. [Bayesian PoseNet(Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909)은 Monte Carlo Dropout을 도입해 추론 때 여러 번 샘플링한 분산으로 자세 불확실성을 쟀고, [Walch 등의 LSTM PoseNet](https://arxiv.org/abs/1611.07890)(arXiv 2016, ICCV 2017)은 이미지 특징 맵의 공간적 맥락을 LSTM으로 훑어 회귀 안정성을 높였다. [Kendall과 Cipolla의 2017년 후속 연구](https://arxiv.org/abs/1704.00390)는 위치 오차와 회전 오차 사이의 애매한 가중치 균형 대신, 3D 점을 정답 자세와 예측 자세로 각각 투영한 픽셀 차이를 직접 줄이는 재투영 손실(reprojection loss)을 제안했다.

하지만 순수한 3D 기하 기반 위치 추정과 나란히 놓아 보면 격차가 뚜렷했다. Kendall과 Cipolla가 2017년 논문 표에 직접 정리한 대로, SIFT 특징점을 3D 점군에 정합하는 [Active Search(Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf)는 같은 Cambridge 장면에서 0.12~0.85 m의 정밀도를 낸 반면, PoseNet 계열의 오차는 대체로 1~3 m 대에 머물렀다. 훗날 Sattler 등(2019)은 이 현상을 면밀히 분석한 끝에, 이미지 한 장에서 절대 자세를 바로 회귀하는 방식(absolute pose regression)이 실제로는 3D 기하를 복원하기보다 DenseVLAD 같은 이미지 검색 기법에 더 가까운 동작을 보인다고 진단했다.

> 🔗 **차용.** PoseNet의 뼈대는 [GoogLeNet(Inception, Szegedy et al., CVPR 2015)](https://arxiv.org/abs/1409.4842)이다. 마지막 분류 head를 떼어 내고 그 자리에 위치 $(x, y, z)$와 쿼터니언 네 성분을 합친 7차원 회귀 head를 붙인 구조다.

---

## 15.2 DeepVO

단일 이미지로 절대 위치를 찍어 내는 대신, 연속된 영상 프레임 사이의 상대 운동을 신경망으로 학습하려는 흐름도 곧바로 뒤따랐다. 2017년 ICRA에서 옥스퍼드 대학의 Sen Wang 등은 [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429)를 발표했다. FlowNet에서 빌려 온 CNN으로 연속한 두 프레임 사이의 optical flow 특징을 뽑은 뒤, 이를 LSTM 순환 신경망에 통과시켜 시간적 맥락을 통합하고 곧바로 6-DoF 상대 움직임을 출력하게 만든 RCNN 구조였다.

KITTI 시퀀스 00, 02, 08, 09로 학습하고 03–07과 10에서 시험한 결과, DeepVO는 스테레오 기반 고전 기법인 VISO2-S보다는 오차가 컸지만 단안 고전 기법인 VISO2-M보다는 대체로 나은 궤적을 그렸다(고속 주행 구간의 이동 오차만 VISO2-M보다 조금 컸다). 그러나 데이터 분포를 벗어나는 순간의 취약성도 적나라하게 드러났다. 시퀀스 00–10 전체로 학습시킨 모델조차 시퀀스 12에서 위치 오차가 크게 치솟았는데, 저자들은 학습 데이터 가운데 시속 60 km를 넘긴 구간이 시퀀스 01 하나뿐이었던 반면 시퀀스 12는 시속 50 km에서 약 90 km까지 달렸기 때문이라고 짚었다.

같은 해 CVPR에 등장한 UC 버클리 Tinghui Zhou(Google 인턴 기간 연구)의 [Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813)는 한 발 더 나아가 정답 자세 레이블마저 없앴다. 카메라의 움직임(ego-motion)을 예측하는 망과 깊이를 예측하는 망을 나란히 세워 두고, 예측된 깊이와 상대 자세로 이웃 프레임을 현재 프레임 시점에 투영(warping)했을 때 실제 이미지와 밝기가 얼마나 일치하는지(photometric reprojection loss)를 자기지도 학습 신호로 삼은 것이다. KITTI 시퀀스 09와 10을 5프레임짜리 짧은 조각으로 잘라 평가했을 때, SfMLearner는 똑같이 5프레임만 본 단기 ORB-SLAM(short)보다 작은 오차를 냈다. 물론 시퀀스 전체를 훑으며 loop closure와 재위치 추정까지 동원하는 ORB-SLAM(full)의 전역 정밀도에는 미치지 못했다.

> 🔗 **차용.** DeepVO는 KITTI의 GPS/IMU 실측치를 정답 레이블로 삼고, 프레임 간 움직임 특징 추출 설계를 [FlowNet(Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852)의 optical flow CNN에서 그대로 가져왔다.

> 🔗 **차용.** SfMLearner와 [DSO(Engel et al. 2018)](https://arxiv.org/abs/1607.02565)는 기하학적으로 대응시킨 영상 간 밝기 차이를 줄인다는 동일한 원리 위에 서 있다. 다만 DSO가 그 밝기 차이로 매 순간 카메라 자세와 점 깊이를 직접 최적화하는 반면, SfMLearner는 같은 잔차를 손실 함수 삼아 깊이와 운동을 예측하는 신경망의 가중치를 학습시킨다는 점이 달랐다.

---

## 15.3 기하 제약, 일반화, 불확실성

2019–2020년 무렵 종단간 학습 열풍을 되돌아본 연구자들 앞에는 세 가지 구조적 한계가 뚜렷이 드러나 있었다(앞서 언급한 [Sattler et al.(2019)](https://arxiv.org/abs/1903.07504)의 분석은 단일 이미지 절대 자세 회귀를 정조준한 것이며, 연속 영상의 상대 운동을 배우는 learned odometry에 그대로 대입할 수는 없지만, 두 진영 모두 아래의 근본 질문에서 자유롭지 못했다).

**기하 제약(inductive bias)의 부재.** 고전 SLAM은 epipolar 제약, 강체 운동(rigid body motion), 투영 불변성 같은 물리·기하 법칙을 수식과 알고리즘 뼈대에 직접 새겨 넣었다. 반면 순수 CNN 회귀 망은 그 모든 3D 기하 법칙을 오직 데이터로부터 밑바닥에서 다시 배워야 했다. 게다가 ImageNet 분류로 사전 학습된 표현은 본래 3차원 거리 감각과 거리가 멀어서, 설령 망이 자세를 꽤 잘 맞히더라도 그것이 3D 기하 구조를 이해한 결과인지 아니면 특정 조명·색감·질감의 조합을 외워 버린 것인지 가려내기 어려웠다.

**학습 분포 밖 일반화의 어려움.** PoseNet의 좌표 출력은 태생적으로 학습한 특정 도시·건물의 좌표계에 묶여 있다. 상대 운동만 출력하는 learned odometry 역시 카메라 렌즈(초점거리), 주행 속도, 환경이 달라지면 성능이 흔들려 새로 검증해야 했다. CoRL 2020의 [TartanVO](https://proceedings.mlr.press/v155/wang21h.html)가 카메라 내장 파라미터를 입력으로 넣어 데이터셋 간 일반화를 핵심 과제로 끌어올린 것도 이 때문이었다(물론 특징점 검출이나 조명이 크게 바뀌면 고전 ORB-SLAM 역시 추적을 잃기는 마찬가지다).

**검증하기 어려운 불확실성 추정.** 자율주행 경로 계획이나 장애물 회피 시스템에 위치를 넘기려면, 추정값과 함께 그 값이 얼마나 믿을 만한지 알려 주는 불확실성 정보가 필수적이다. EKF가 공분산을 전파하고 factor graph가 추정치 주변의 정보 행렬을 뽑아내듯(물론 이 역시 선형화 근사에 의존하지만), Bayesian PoseNet은 dropout으로 불확실성을 쟀고 원 논문은 이를 위치 오차 가늠과 학습 장면 내외부 판별에 활용했다. 그러나 신경망이 뱉어 내는 분산 값이 학습 분포를 벗어난(out-of-distribution) 낯선 입력에서도 실제 기하적 오차와 비례하도록 보정(calibration)되는지는 별도의 까다로운 검증 문제로 남았다.

---

## 15.4 그 뒤

흥미로운 점은 이 시기의 선구적 시도들이 이후 전혀 다른 영역으로 뻗어 나가며 큰 유산을 남겼다는 사실이다. Kendall은 2017년 케임브리지에서 박사학위를 마친 뒤 그해 자율주행 스타트업 Wayve를 창업했고, 이후 [그의 연구 행보](https://alexgkendall.com/)는 end-to-end 자율주행 모방 학습과 대규모 주행 월드 모델(GAIA-1)로 이어졌다.

SLAM 본진 안에서는 순수 회귀 대신 학습과 기하를 섞는 절충안이 두각을 나타냈다. 2017년 뮌헨 공대(TUM)의 Tateno, Tombari, Laina, Navab가 내놓은 [CNN-SLAM](https://arxiv.org/abs/1704.03489)은 CNN이 예측한 조밀한 깊이를 direct monocular SLAM의 깊이 측정과 융합해, 기하로 깊이를 잡기 힘든 저질감 벽면에서는 신경망 예측에 기대고 기하가 뚜렷한 곳에서는 SLAM 측정을 우선시하며 단안 스케일까지 복원해 냈다(TUM RGB-D와 ICL-NUIM에서 검증). SfMLearner가 닦은 photometric self-supervision 기법은 MonoDepth2(Godard et al., ICCV 2019) 등 단안 깊이 추정 연구의 표준 학습 틀이 되었고, DeepVO와 같은 옥스퍼드 그룹에서 나온 [VINet(Clark et al., AAAI 2017)](https://arxiv.org/abs/1701.08376)은 LSTM으로 영상과 IMU를 함께 묶어 종단간 VIO의 가능성을 시험했다. 한편 2021년 등장한 DROID-SLAM(Teed & Deng, NeurIPS 2021)은 미분 가능한 기하 최적화 구조를 유지하되 정답 자세와 깊이로 계산한 optical flow를 지도 학습 신호로 삼아 전혀 다른 돌파구를 열었다.

> 📜 **예언 vs 실제.** Zhou 등(2017)은 SfMLearner 논문 맺음말에서 장면 내 동적 물체와 가림(occlusion)을 명시적으로 모델링하지 못한 점, 카메라 내부 파라미터를 미리 알아야 하는 점, 그리고 단일 depth map이 복잡한 3D 장면을 지나치게 단순화한다는 점을 한계로 짚었다. 이 가운데 동적 물체 문제는 이듬해 정적 영역과 동적 영역을 갈라 깊이·optical flow·카메라 운동을 함께 학습한 [GeoNet(Yin & Shi 2018)](https://arxiv.org/abs/1803.02276) 같은 후속 self-supervised 연구들이 곧바로 이어받아 일부 해결책을 내놓았다.

> 📜 **예언 vs 실제.** Kendall 등(2015) 역시 PoseNet 논문에서 다중 시점 기하 기반 학습 데이터 확충, 확률적 불확실성 모델링, 단일 모델이 외울 수 있는 장면 규모의 확장을 후속 과제로 제시했다. 곧이어 Bayesian PoseNet(2016)이 확률적 불확실성 추정을 구현하며 한 조각을 채웠지만, 절대 자세 회귀 자체의 정밀도와 새로운 장면으로의 일반화 한계는 그 뒤로도 오래 풀리지 않는 숙제로 남았다.

---

## 15.5 기하와 학습의 분담

2020년을 전후해 SLAM 연구의 큰 흐름은 일종의 역할 분담으로 수렴하기 시작했다. 억지로 신경망에 3차원 강체 변환과 투영 기하까지 다 외우게 하는 대신, 확실히 알고 있는 기하 계산과 비선형 최적화는 알고리즘에 맡기고 신경망은 텍스처 없는 영역의 깊이 사전 지식이나 까다로운 영상 대응을 읽어 내는 데 집중시키는 전략이다.

CodeSLAM과 DROID-SLAM은 바로 이 분업 철학 위에서 태어났다. 둘 다 PoseNet처럼 이미지에서 곧바로 자세 숫자를 회귀하지 않는다. 대신 영상 사이의 대응 관계와 잠재 표현을 신경망이 공급하면, 그 위에 기하 제약을 걸어 자세를 푸는 최적화 엔진이 실제 추정치를 도출한다.

물론 어느 쪽이 절대적으로 우월하다고만 단정할 수는 없었다. PoseNet은 SIFT 기반 기하 정합이 대응점을 찾지 못해 주저앉는 극단적인 시점 차이나 해 질 녘·비·안개·밤·motion blur 속에서도 어떻게든 근사 자세를 내놓았다는 장점이 있었다. 하지만 Sattler 등(2019)이 확인했듯 정밀한 3차원 위치 추정에서는 기하 구조 기반 방법을 따라잡기 어려웠고, 결국 고전 기하 최적화의 뼈대 위에 학습된 지각 모듈을 끼워 넣는 하이브리드 설계가 주류로 올라섰다.

> 🔗 **차용.** CodeSLAM(Bloesch et al., CVPR 2018)과 DROID-SLAM(Teed & Deng, NeurIPS 2021)이 공유하는 핵심 전략은 최적화 엔진의 보존이다. 둘 다 bundle adjustment나 비선형 최적화 틀을 버리지 않고 그대로 둔 채, 학습된 신경망에게 CodeSLAM에서는 저차원 depth 표현을, DROID-SLAM에서는 조밀한 대응점과 반복 갱신 방향을 맡겼다.

---

## 아직 열린 문제

**어떤 기하 제약을 어디에 넣을지.** 기하 계산을 알고리즘으로 푼다고 하더라도, rigid body motion이나 epipolar constraint 같은 제약을 아키텍처의 어느 층위에서 얼마나 강하게 코드로 강제할지는 아직 고정된 답이 없다. 최근 3DGS 기반 SLAM 시스템들은 미분 가능한 렌더링과 기하 표현 자체를 학습·최적화 루프 안에 직접 녹여 넣는 새로운 배합을 시험하고 있다.

**Learned uncertainty의 calibration.** Bayesian PoseNet 이후에도 딥러닝 모델이 출력하는 불확실성 추정치가 실제 위치 오차의 크기와 일관되게 맞물리는지, 특히 훈련 때 한 번도 본 적 없는 낯선 환경에서도 믿고 쓸 수 있는지 보증하는 일은 여전히 난제로 남아 있다.

**End-to-end의 범위.** 2015년 PoseNet이 말하던 end-to-end는 이미지 한 장에서 곧장 자세를 뽑아내는 단일 블랙박스 회귀였다. 그러나 대규모 foundation model과 미분 가능한 최적화 계층이 공존하는 2023년 이후에는 SLAM의 어느 모듈까지를 학습으로 대체하고 어느 뼈대를 해석적 알고리즘으로 남길지에 대한 경계선이 새롭게 그어지고 있다.

---

순수 종단간 회귀가 부딪힌 기하 일반화의 벽은 결국 학습과 기하를 결합하는 새로운 설계를 불러냈다. 2018년 Imperial College London의 Dyson Robotics Lab에서 나온 CodeSLAM은 VAE로 배운 저차원 잠재 공간 안에 조밀한 깊이를 압축해 확률적 최적화에 태웠고, 몇 년 뒤 프린스턴에서 나온 DROID-SLAM은 미분 가능한 Dense Bundle Adjustment를 신경망 반복 루프 안에 통째로 집어넣었다. 다음 장에서는 기하학과 딥러닝이 본격적으로 손을 맞잡은 이 하이브리드 시대의 계보를 따라간다(→ 16장).