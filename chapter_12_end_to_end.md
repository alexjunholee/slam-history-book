# Ch.12 — End-to-end 좌절

11장에서 단안 카메라 하나로 depth를 복원하는 일이 가능해졌다. Eigen의 망은 픽셀에서 metric depth를 꺼냈고, SfMLearner는 라벨 없이도 기하학적 supervision을 만들어냈다. 학습이 형태를 본다는 것이 증명된 순간이었다. 그렇다면 더 나아갈 수 있지 않을까. pose 추정도, 루프 클로저도—SLAM 전체를 하나의 망으로 끝낼 수 있지 않을까. 2015년부터 2018년 사이, 이 물음은 답을 찾지 못했다.

2015년, Cambridge Computer Laboratory 박사과정 학생 Alex Kendall은 Roberto Cipolla 지도교수 아래 Google Street View 이미지로 학습한 신경망에 사진 한 장을 넣고 6-DoF pose를 출력하는 연구를 완성했다. [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336)이라 명명한 이 논문은 Santiago de Chile에서 열린 ICCV에서 즉각 반향을 일으켰다. SLAM의 30년짜리 방정식—특징 추출, 매칭, 최적화, 지도 관리—을 단일 CNN으로 압축할 수 있다면? 이 물음은 2015년부터 2018년까지 수십 편의 논문을 낳았다. 그리고 거의 예외 없이, 같은 결론을 반복했다.

---

## 12.1 PoseNet

PoseNet이 물려받은 것은 [AlexNet(Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks)이었다. Kendall은 ImageNet에서 분류 task로 학습된 깊은 CNN이 고수준 시각 표현을 형성한다는 사실을 확인하고, 그 feature hierarchy를 pose estimation으로 전용했다.

> 🔗 **차용.** PoseNet의 backbone은 [GoogleNet(Inception, Szegedy et al. 2014)](https://arxiv.org/abs/1409.4842) 구조다. Classification head를 제거하고 7차원 회귀 head(x, y, z, quaternion 4개)를 붙인 것이 전부다. ImageNet 학습으로 얻은 feature hierarchy를 localization에 이식한 직접 차용.

Kendall이 직접 수집한 Cambridge Landmarks 데이터셋—킹스 칼리지 예배당, 거리, 옛 병원 등 여러 야외 장면—에서 PoseNet은 장면에 따라 위치 오차 2 m 안팎, 방향 오차 5-8° 수준을 기록했다(원 논문 §5 기준). 2015년 기준으로는 인상적인 수치였다. GPU 한 장으로 5 ms 이내에 답이 나왔다. 특징 추출도, RANSAC도, 지도 조회도 없었다.

논문은 즉각적인 후속을 촉발했다. [Bayesian PoseNet(Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909)은 Monte Carlo Dropout으로 자세 불확실성을 추정하려 했다. LSTM PoseNet은 시퀀스 정보를 통합했다. Geometric loss를 추가한 변형이 등장했다. Kendall 자신도 2017년에 재귀 구조와 photometric loss를 결합한 버전을 냈다.

그러나 비교 기준이 올라갈수록 격차가 드러났다. 같은 장면에서 [Active Search(Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf)나 DenseVLAD는 위치 오차 0.2 m 수준을 달성했다. PoseNet 계열은 수 미터 오차를 좀처럼 넘지 못했다. 이미지 한 장에서 절대 자세를 회귀하는 접근에는 원론적 한계가 있었다.

---

## 12.2 DeepVO

PoseNet의 한계 중 하나가 단일 이미지 입력이라면, 시퀀스를 입력하면 어떨까. Sen Wang(에든버러 Heriot-Watt)과 공저자들은 2017년 [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429)를 ICRA에 발표했다. FlowNet에서 영향을 받은 CNN으로 연속 프레임 쌍의 optical flow feature를 추출하고, LSTM으로 시간 맥락을 축적해 VO를 직접 출력하는 구조였다.

> 🔗 **차용.** DeepVO의 훈련 라벨은 KITTI의 GPS/IMU ground truth다. feature 추출 설계는 [FlowNet(Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852)의 optical flow CNN 구조에서 직접 차용했다. "deep VO"가 고전 센서 측정과 이전 딥러닝 연구 양쪽에 동시에 기대는 방식.

LSTM이 temporal modeling을 맡으면서 drift 억제를 기대했다. KITTI 시퀀스 일부에서 DVO-SLAM이나 VISO2-M 대비 낮은 drift를 보이는 결과가 논문에 실렸다. 하지만 조건이 있었다. 훈련 시퀀스와 비슷한 주행 패턴, 비슷한 조명 조건, 비슷한 도시 풍경, 비슷한 속도 프로파일. 조건이 어긋나면 LSTM이 축적한 "맥락"은 오히려 편향이 되었다.

Tinghui Zhou(UC Berkeley)가 같은 해에 발표한 [Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813)는 다른 각도로 접근했다. 자기지도(self-supervised) 학습으로 depth와 ego-motion을 동시에 추정하되, photometric reprojection loss를 학습 신호로 썼다. 라벨 없이 학습 가능하다는 점이 강점이었다.

> 🔗 **차용.** SfMLearner의 photometric loss는 고전 direct SLAM이 사용하는 intensity residual과 수학적으로 동일하다. [DSO(Engel et al. 2018)](https://arxiv.org/abs/1607.02565)의 photometric 원리를 미분가능 학습 프레임워크로 옮겼다. 이 계보는 살아남았다—SfMLearner의 self-supervision 아이디어는 MonoDepth2를 거쳐 결국 DROID-SLAM의 전제 조건 중 하나가 된다.

다만 SfMLearner 단독 VO는 KITTI 공식 리더보드에서 ORB-SLAM의 절반에도 미치지 못하는 성능으로 마무리되었다.

---

## 12.3 실패 원인 세 가지

2019년에서 2020년 사이, 이 분야의 논문들이 공통된 자기비판을 시작했다. Sudeep Pillai(MIT, 이후 TRI)는 2019년 발표에서 end-to-end 접근의 구조적 한계를 체계화했다. 원인은 크게 세 가지였다.

**첫 번째: inductive bias의 부재.** 고전 SLAM은 수십 년에 걸쳐 축적된 기하학적 제약을 알고리즘 구조 안에 새겨 넣었다. epipolar constraint, rigid body motion 가정, scale invariance, 공간의 연속성. CNN은 이것들을 데이터로부터 새로 배워야 했다. ImageNet의 고양이와 자동차 사진이 3D 공간의 metric geometry를 가르쳐주지는 않는다. 회귀 망이 pose를 맞추는 것처럼 보여도, 실제로 그것이 3D 공간을 이해해서인지 아니면 특정 조명·색상·질감 조합을 외워서인지 구별하기 어려웠다.

**두 번째: 일반화 실패.** 훈련 집합 밖으로 나가면 성능이 급락했다. Cambridge Landmarks로 학습한 PoseNet은 Oxford 거리에서 쓸 수 없었다. KITTI로 학습한 DeepVO는 레이더가 없는 다른 차량 데이터셋에서 drift가 기하급수로 커졌다. 고전 ORB-SLAM은 특징 검출에 실패하거나 조명이 극단적으로 변하면 추적을 잃었지만, 그 실패가 예측 가능했고 재초기화할 수 있었다. end-to-end는 조용히 틀렸다. 얼마나 틀렸는지 모르는 채로.

**세 번째: 불확실성 정량화의 부재.** SLAM이 단순한 pose 추정기로 끝나지 않는 이유는 downstream 시스템—경로 계획, 장애물 회피—이 위치 추정의 공분산을 요구하기 때문이다. EKF와 factor graph는 공분산을 자연스럽게 전파한다. Bayesian PoseNet이 dropout으로 분산을 추정하려 했지만, 그 분산이 실제 위치 오차와 calibrated 관계를 맺는지 검증하기 어려웠다. 특히 훈련 분포 밖 입력에서 Bayesian PoseNet은 오히려 자신만만한 틀린 답을 냈다. 틀린 것보다 자신만만하게 틀리는 것이 로봇 시스템에는 더 위험하다.

---

## 12.4 반성의 기록

Kendall은 이 실패를 외면하지 않았다. 박사학위를 마친 2019년, 그는 Wayve로 자리를 옮겨 자율주행용 imitation learning과 world model 연구로 방향을 틀었다. 학습 기반 localization을 포기한 것이 아니라, "이미지 한 장에서 절대 pose를 회귀한다"는 문제 정의가 틀렸다고 판단한 것이었다.

Federico Tombari 그룹(TU Munich, 이후 Google)도 같은 시기에 [CNN-SLAM(Tateno et al. 2017)](https://arxiv.org/abs/1704.03489)을 시도했다. CNN이 예측한 dense depth를 직접(direct) monocular SLAM의 깊이 측정과 융합하려는 접근이었다. 학습 부분이 dense depth에 국한되었다는 점에서 완전한 end-to-end는 아니었지만, "CNN이 단안 SLAM의 스케일·저텍스처 문제를 해결해 줄 수 있지 않을까"라는 기대의 한 갈래였다. 결과는 장면에 따라 들쭉날쭉했고, 정확도가 일관되게 앞서지 못했다.

> 📜 **예언 vs 실제.** Kendall은 PoseNet 논문(2015)에서 불확실성 추정, temporal 정보 통합, 더 넓은 규모의 장면으로의 확장을 다음 과제로 꼽았다. 세 방향 모두 실행되었다—Bayesian PoseNet(2016), LSTM PoseNet(2016), 복수의 outdoor 확장 실험들. 그러나 각 시도가 새 벽에 부딪혔고, 연구자들은 결국 이 접근법 전체를 포기했다. 예언이 합리적이었어도 플랫폼 자체가 틀렸으면 소용없다. `[무산]`

일부 시도는 다른 방향으로 살아남았다. SfMLearner의 photometric self-supervision은 MonoDepth2(Godard 2019), 나아가 DROID-SLAM(Teed & Deng 2021)의 훈련 전략 안에 흡수되었다. DeepVO가 보여준 LSTM 기반 temporal modeling은 시각-관성 학습 연구에서 변형된 형태로 재등장했다. 아이디어는 사라진 게 아니라 용도가 바뀌었다.

> 📜 **예언 vs 실제.** Zhou는 SfMLearner 논문(2017)에서 dynamic object 처리와 photometric noise에 대한 강건성을 남은 과제로 제시했다. [GeoNet(Yin & Shi 2018)](https://arxiv.org/abs/1803.02276)을 비롯한 후속 self-supervised 연구들이 부분적으로 이 방향을 밀었다. 그러나 self-supervised VO 단독으로 SLAM을 대체하는 경로는 주류에 합류하지 못했다. photometric self-supervision 자체는 계보를 이어갔지만, end-to-end VO라는 목표는 분야가 기각했다. `[기술변화]`

---

## 12.5 교훈의 정착

2020년을 전후해 이 분야는 하나의 합의에 도달했다. "geometry는 알고리즘, learning은 feature와 prior"—대략 이런 방향으로.

> 🔗 **차용.** 이 원칙의 실천은 13장에서 다루는 CodeSLAM(Bloesch 2018)과 DROID-SLAM(Teed & Deng 2021)에서 구체화된다. 두 시스템 모두 factor graph 또는 bundle adjustment라는 기하학적 뼈대를 유지하고, 학습 부분은 feature 추출이나 depth prior 형성에 국한한다. PoseNet이 버린 뼈대가 사실 포기할 수 없는 것이었다는 확인이다.

고전 파이프라인이 학습 기반 대안에 일관되게 우월한 것이 아니었다. ORB-SLAM도 textureless 환경에서, 야간에서, 비에서 자주 실패했다. 문제는 고전 SLAM의 견고함이 아니라 end-to-end의 오류가 더 불투명하고 더 예측 불가능하다는 데 있었다.

실패는 데이터셋이나 아키텍처의 문제가 아니었다. 이미지에서 바로 pose로 직결하는 경로에 30년짜리 기하학 지식이 통째로 빠져 있었다.

---

## 🧭 아직 열린 것

**어떤 inductive bias를 어떻게 주입할 것인가.** "geometry는 알고리즘으로"라는 원칙은 맞지만, 어떤 기하학을 어느 수준에서 코드화해야 하는지는 여전히 개방 질문이다. rigid body motion인가, epipolar constraint인가. foundation model 시대에 이 경계는 다시 흐려지고 있다. GaussianSLAM이나 3DGS 기반 시스템이 geometry를 학습 표현 안에 녹이는 방식을 실험하고 있다.

**Learned uncertainty의 calibration.** Bayesian PoseNet의 실패 이후에도 이 문제는 해결되지 않았다. 딥러닝 기반 uncertainty estimate가 실제 오차와 얼마나 calibrated 관계를 가지는지—특히 out-of-distribution 입력에서—는 2026년 기준으로도 열려 있다. 자율주행이 이 질문에 실용적 압력을 가하고 있다.

**"End-to-end"의 의미 재정의.** PoseNet이 정의한 end-to-end(이미지→pose, 학습만으로)는 실패했다. 그러나 foundation model이 등장한 2023년 이후 end-to-end의 의미가 바뀌고 있다. SLAM의 어느 모듈을 학습으로 채우고 어느 모듈을 알고리즘으로 유지할 것인가—이 분할선 자체가 재협상 중이다.

"geometry는 알고리즘으로, learning은 feature로"라는 원칙이 이 시기에 굳어졌다. 2018년, Andrew Davison의 연구실에서 그 원칙의 첫 실질적 구현이 나왔다. 장소는 Kensington, Imperial College London. 이름은 CodeSLAM이었다.
