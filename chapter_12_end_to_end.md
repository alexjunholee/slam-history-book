# Ch.12 — End-to-end 좌절

단안 카메라 하나로 depth를 복원하는 일이 가능해졌다. Eigen의 망은 픽셀에서 metric depth를 꺼냈고, SfMLearner는 라벨 없이도 기하학적 supervision을 만들어냈다. 그렇다면 pose 추정과 루프 클로저를 포함한 SLAM 전체를 하나의 망으로 끝낼 수 있을까. 2015년부터 2018년 사이, 이 물음은 답을 찾지 못했다.

2015년, Cambridge Computer Laboratory 박사과정 학생 Alex Kendall은 Roberto Cipolla 지도교수 아래 Google Street View 이미지로 학습한 신경망에 사진 한 장을 넣고 6-DoF pose를 출력하는 연구를 완성했다. [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336)이라 명명한 이 논문은 Santiago de Chile에서 열린 ICCV에서 발표됐다. SLAM의 30년짜리 방정식(특징 추출, 매칭, 최적화, 지도 관리)을 단일 CNN으로 압축할 수 있다면? 이 물음은 2015년부터 2018년까지 수십 편의 논문을 낳았고, 후속 연구들은 비슷한 한계를 반복해서 보고했다.

---

## 12.1 PoseNet

PoseNet이 물려받은 것은 [AlexNet(Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks)이었다. Kendall은 ImageNet에서 분류 task로 학습된 깊은 CNN이 고수준 시각 표현을 형성한다는 사실을 확인하고, 그 feature hierarchy를 pose estimation으로 전용했다.

> 🔗 **차용.** PoseNet의 backbone은 [GoogleNet(Inception, Szegedy et al. 2014)](https://arxiv.org/abs/1409.4842) 구조다. Classification head를 제거하고 7차원 회귀 head(x, y, z, quaternion 4개)를 붙인 것이 전부다. ImageNet 학습으로 얻은 feature hierarchy를 localization에 직접 이식했다.

Kendall이 직접 수집한 Cambridge Landmarks 데이터셋(킹스 칼리지 예배당, 거리, 옛 병원 등 여러 야외 장면)에서 PoseNet은 장면에 따라 위치 오차 2 m 안팎, 방향 오차 5-8° 수준을 기록했다(원 논문 §5 기준). 2015년 기준으로는 인상적인 수치였다. GPU 한 장으로 5 ms 이내에 답이 나왔다. 특징 추출도, RANSAC도, 지도 조회도 없었다.

후속 연구는 서로 다른 부분을 바꾸었다. [Bayesian PoseNet(Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909)은 Monte Carlo Dropout으로 자세 불확실성을 추정했다. [Walch 등의 LSTM PoseNet](https://arxiv.org/abs/1611.07890)은 2016년 처음 공개되어 ICCV 2017에 발표됐으며, 시간에 따른 영상 시퀀스가 아니라 한 영상에서 추출한 특징의 공간적 관계를 LSTM으로 처리했다. [Kendall과 Cipolla의 2017년 연구](https://arxiv.org/abs/1704.00390)는 자세 회귀의 손실 함수를 다뤘다. 그중 재투영 손실은 3D점을 정답 자세와 예측 자세로 투영한 영상 좌표의 차이를 이용하며, 밝기 차이를 비교하는 photometric loss와는 다르다.

그러나 비교 기준이 올라갈수록 격차가 드러났다. 같은 장면에서 [Active Search(Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf)나 DenseVLAD는 위치 오차 0.2 m 수준을 달성했다. PoseNet 계열은 수 미터 오차에서 좀처럼 벗어나지 못했다. 이미지 한 장에서 절대 자세를 회귀하는 접근에는 원론적 한계가 있었다.

---

## 12.2 DeepVO

PoseNet의 한계 중 하나가 단일 이미지 입력이라면, 시퀀스를 입력하면 어떨까. Sen Wang(에든버러 Heriot-Watt)과 공저자들은 2017년 [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429)를 ICRA에 발표했다. FlowNet에서 영향을 받은 CNN으로 연속 프레임 쌍의 optical flow feature를 추출하고, LSTM으로 시간 맥락을 축적해 VO를 직접 출력하는 구조였다.

> 🔗 **차용.** DeepVO의 훈련 라벨은 KITTI의 GPS/IMU ground truth다. feature 추출 설계는 [FlowNet(Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852)의 optical flow CNN 구조에서 직접 차용했다. "deep VO"는 고전 센서 측정과 이전 딥러닝 연구 양쪽에 동시에 기댔다.

LSTM이 temporal modeling을 맡으면서 drift 억제를 기대했다. KITTI 시퀀스 일부에서 DVO-SLAM이나 VISO2-M 대비 낮은 drift를 보이는 결과가 논문에 실렸다. 하지만 조건이 있었다. 훈련 시퀀스와 비슷한 주행 패턴, 비슷한 조명 조건, 비슷한 도시 풍경, 비슷한 속도 프로파일. 조건이 어긋나면 LSTM이 축적한 "맥락"은 오히려 편향이 되었다.

Tinghui Zhou(UC Berkeley)가 같은 해에 발표한 [Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813)는 다른 각도로 접근했다. 자기지도(self-supervised) 학습으로 depth와 ego-motion을 동시에 추정하되, photometric reprojection loss를 학습 신호로 썼다. 라벨 없이 학습 가능하다는 점이 강점이었다.

> 🔗 **차용.** SfMLearner와 [DSO(Engel et al. 2018)](https://arxiv.org/abs/1607.02565)는 기하적으로 대응시킨 영상의 밝기 차이를 이용한다는 원리를 공유한다. 다만 DSO는 이를 자세와 지도의 직접 최적화에 쓰고, SfMLearner는 깊이와 카메라 운동을 예측하는 망의 학습 신호로 쓴다. SfMLearner의 자기지도 접근은 MonoDepth2 같은 단안 깊이 연구로 이어졌다. 미분 가능한 기하를 사용하는 DROID-SLAM은 pose와 optical flow의 정답으로 학습하므로 이 자기지도 계열과 구별한다.

다만 SfMLearner의 짧은 구간 VO 평가는 장기 궤적과 루프 클로저까지 포함한 ORB-SLAM의 시스템 성능과 구별해서 읽어야 한다.

---

## 12.3 기하 제약, 일반화, 불확실성

2019–2020년 무렵 end-to-end pose regression과 learned odometry에는 기하 제약, 훈련 분포 밖 일반화, 불확실성 추정이라는 서로 다른 문제가 남았다. 가령 [Sattler et al.(2019)](https://arxiv.org/abs/1903.07504)은 absolute pose regression의 한계를 분석했다. 이 결과는 시간에 따른 운동을 추정하는 learned odometry의 문제와 구별해서 읽어야 한다.

**첫 번째: inductive bias의 부재.** 고전 SLAM은 수십 년에 걸쳐 축적된 기하학적 제약을 알고리즘 구조 안에 새겨 넣었다. epipolar constraint, rigid body motion 가정, scale invariance, 공간의 연속성. CNN은 이것들을 데이터로부터 새로 배워야 했다. ImageNet의 고양이와 자동차 사진이 3D 공간의 metric geometry를 가르쳐주지는 않는다. 회귀 망이 pose를 맞추는 것처럼 보여도, 실제로 그것이 3D 공간을 이해해서인지 아니면 특정 조명·색상·질감 조합을 외워서인지 구별하기 어려웠다.

**두 번째: 훈련 분포 밖 일반화.** 특정 장면에서 학습한 PoseNet은 보지 못한 장면의 절대 좌표를 그대로 알려 줄 수 없다. 상대 운동을 추정하는 learned odometry도 카메라·주행 방식·영상 분포가 달라지면 별도의 검증이 필요하다. CoRL 2020의 [TartanVO](https://proceedings.mlr.press/v155/wang21h.html)는 데이터셋을 넘는 일반화를 주요 설계 문제로 다뤘다. 고전 ORB-SLAM 역시 특징 검출이나 조명 조건이 달라지면 추적을 잃는다. 실패 감지와 복구 절차는 기하 기반 시스템과 학습 기반 시스템 모두에 필요하다.

**세 번째: 불확실성 추정의 검증.** 경로 계획과 장애물 회피에는 위치 추정뿐 아니라 그 오차를 판단할 정보도 필요하다. EKF는 공분산을 갱신하고, factor graph에서는 추정치 주변의 불확실성을 계산할 수 있지만, 이 값들도 모델과 근사의 영향을 받는다. Bayesian PoseNet은 dropout을 이용해 불확실성을 추정했고, 원 논문은 이를 위치 오차 추정과 학습 장면의 포함 여부 판별에 활용했다. 다만 이 평가 결과만으로 임의의 훈련 분포 밖 입력에서 불확실성이 실제 오차와 잘 맞는다고 보장할 수는 없다.

---

## 12.4 반성의 기록

Kendall은 2017년 Cambridge에서 박사학위를 받았고 같은 해 Wayve를 공동 창업했다. [이후 연구](https://alexgkendall.com/)에는 자율주행용 imitation learning과 world model이 포함됐다. 카메라 자세를 추정하던 연구에서 장면을 이해하고 주행 행동을 학습하는 문제로 범위가 넓어졌다.

Tateno·Tombari·Laina·Navab의 [CNN-SLAM(2017)](https://arxiv.org/abs/1704.03489)은 CNN이 예측한 dense depth를 직접(direct) monocular SLAM의 깊이 측정과 융합했다. 영상의 질감이 부족한 영역에서는 예측 깊이에 더 의존하고, 기하 추정이 가능한 곳에서는 SLAM 측정을 활용했다. 예측 깊이로 절대 스케일을 추정하는 방법도 제시했다. 두 벤치마크에서 평가한 이 방식은 학습과 기하를 결합한 당시의 사례였다.

> 📜 **예언 vs 실제.** PoseNet 논문(2015)은 다중 시점 기하로 얻는 학습 자료, 확률적 확장, 한 모델이 학습할 수 있는 장면 규모의 한계를 후속 과제로 제시했다. Bayesian PoseNet(2016)은 그중 확률적 자세 추정과 불확실성 평가를 다뤘다. LSTM PoseNet의 공간 특징 처리는 시간 정보 통합의 실현으로 분류할 수 없으며, 이 후속 연구들만으로 원 논문의 모든 과제가 해결됐다고 볼 수는 없다. Absolute pose regression의 정확도와 일반화는 이후에도 검증 과제로 남았다.

일부 시도는 다른 방향으로 살아남았다. SfMLearner의 photometric self-supervision은 MonoDepth2(Godard 2019) 같은 후속 단안 깊이 연구로 이어졌다. DROID-SLAM(Teed & Deng 2021)은 미분 가능한 기하 구조를 활용하지만, pose와 optical flow의 감독 신호로 학습한다. DeepVO가 보여준 LSTM 기반 temporal modeling은 시각-관성 학습 연구에서 변형된 형태로 재등장했다. 아이디어의 용도가 바뀌었을 뿐이다.

> 📜 **예언 vs 실제.** Zhou는 SfMLearner 논문(2017)에서 dynamic object 처리와 photometric noise에 대한 강건성을 남은 과제로 제시했다. [GeoNet(Yin & Shi 2018)](https://arxiv.org/abs/1803.02276)을 비롯한 후속 self-supervised 연구들이 부분적으로 이 방향을 밀었다. GeoNet은 정적 영역과 동적 영역을 나누어 다루며 깊이·optical flow·카메라 운동을 함께 학습했다.

---

## 12.5 교훈의 정착

2020년을 전후해 연구의 무게중심은 "geometry는 알고리즘, learning은 feature와 prior"라는 방향으로 옮겨갔다.

> 🔗 **차용.** 이 원칙의 실천은 13장에서 다루는 CodeSLAM(Bloesch 2018)과 DROID-SLAM(Teed & Deng 2021)에서 구체화된다. 두 시스템 모두 factor graph 또는 bundle adjustment라는 기하학적 뼈대를 유지하고, 학습 부분은 CodeSLAM에서 depth 표현을, DROID-SLAM에서 dense correspondence와 반복 update를 맡는다. PoseNet이 버린 뼈대가 사실 포기할 수 없는 것이었다는 확인이다.

고전 파이프라인이 학습 기반 대안에 일관되게 우월한 것은 아니다. PoseNet은 특징점 기반 정합이 실패하는 일부 조명 변화와 motion blur 조건에서 동작을 보였다. 반대로 Sattler et al.(2019)의 평가는 당시 absolute pose regression이 3D 구조 기반 위치 추정의 정확도에 미치지 못함을 보였다. 어느 방법이 유리한지는 입력 조건과 평가 과제에 따라 달랐다.

이 비교에서 중요한 구별은 이미지에서 pose를 직접 회귀하는 것과, 영상의 대응 관계에 기하 제약을 적용해 pose를 구하는 것이었다. CodeSLAM과 DROID-SLAM은 후자의 계산에 학습된 표현을 결합했다.

---

## 🧭 아직 열린 것

**어떤 inductive bias를 어떻게 주입할 것인가.** "geometry는 알고리즘으로"라는 원칙은 맞지만, 어떤 기하학을 어느 수준에서 코드화해야 하는지는 여전히 열린 질문이다. rigid body motion인가, epipolar constraint인가. foundation model 시대에 이 경계는 다시 흐려지고 있다. GaussianSLAM이나 3DGS 기반 시스템이 geometry를 학습 표현 안에 녹이는 방식을 실험하고 있다.

**Learned uncertainty의 calibration.** Bayesian PoseNet의 불확실성 추정 시도 이후에도 이 문제는 해결되지 않았다. 딥러닝 기반 uncertainty estimate가 실제 오차와 얼마나 calibrated 관계를 가지는지(특히 out-of-distribution 입력에서)는 2026년 기준으로도 열려 있다. 자율주행이 이 질문에 실용적 압력을 가하고 있다.

**"End-to-end"의 의미 재정의.** PoseNet은 end-to-end를 이미지에서 pose로 직접 회귀하는 형태로 구현했다. 그러나 foundation model이 등장한 2023년 이후 end-to-end의 의미가 바뀌고 있다. SLAM의 어느 모듈을 학습으로 채우고 어느 모듈을 알고리즘으로 유지할 것인가. 이 분할선 자체가 재협상 중이다.

"geometry는 알고리즘으로, learning은 feature로"라는 원칙이 이 시기에 굳어졌다. 2018년 Kensington의 Imperial College London, Andrew Davison 연구실에서 그 원칙을 구현한 CodeSLAM이 나왔다.
