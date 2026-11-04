# Ch.12 — End-to-end 학습의 한계: PoseNet, DeepVO, SfMLearner

Ch.11에서 본 단안 depth 예측 연구와 같은 시기인 2015년부터 2018년 사이에, 연구자들은 자세 추정까지 포함한 SLAM의 여러 단계를 신경망 하나로 풀려고 했다. end-to-end는 입력에서 출력까지 중간 단계를 따로 설계하지 않고 망 하나로 학습한다는 뜻이다.

2015년 Cambridge 대학 박사과정 학생 Alex Kendall은 Roberto Cipolla의 지도로, 사진 한 장을 넣으면 6-DoF pose를 출력하는 신경망을 만들었다. ImageNet이나 Places로 미리 학습한 GoogLeNet을 가져와, Cambridge 시내에서 찍은 영상과 SfM으로 만든 자세 라벨로 다시 학습(fine-tuning)했다. 이 [Kendall et al. 2015. PoseNet](https://doi.org/10.1109/ICCV.2015.336)은 Santiago de Chile에서 열린 ICCV에서 발표됐다. 그 뒤 몇 년 동안 feature 추출, 매칭, 최적화, 지도 관리를 CNN 하나로 대신하려는 논문이 여럿 나왔다.

---

## 12.1 PoseNet

[AlexNet(Krizhevsky et al. 2012)](https://papers.nips.cc/paper/4824-imagenet-classification-with-deep-convolutional-neural-networks) 이후, ImageNet 분류로 학습한 깊은 CNN이 고수준 시각 표현을 배운다는 사실이 널리 알려졌다. Kendall은 이 표현을 자세 추정에 옮겨 썼다.

> 🔗 **차용.** PoseNet의 backbone은 [GoogLeNet(Inception, Szegedy et al., CVPR 2015)](https://arxiv.org/abs/1409.4842) 구조다. 분류 head를 떼고 7차원 회귀 head(위치 x, y, z와 quaternion 네 성분)를 붙였다.

Kendall 등은 Cambridge Landmarks 데이터셋을 직접 찍어 만들었다. King's College, Street, Old Hospital, Shop Façade, St Mary's Church 같은 야외 장면이 들어 있다. 여기서 PoseNet의 위치 오차는 장면에 따라 1.5~3.7 m, 방향 오차는 5~8.5° 정도였다(원 논문 §5). 계산은 프레임당 약 5 ms였고, feature 추출이나 RANSAC, 지도 조회 없이 자세가 나왔다.

후속 연구는 저마다 다른 부분을 고쳤다. [Bayesian PoseNet(Kendall & Cipolla 2016)](https://arxiv.org/abs/1509.05909)은 Monte Carlo Dropout으로 자세 불확실성을 추정했다. [Walch 등의 LSTM PoseNet](https://arxiv.org/abs/1611.07890)은 2016년 arXiv에 먼저 올라왔고 ICCV 2017에 실렸다. 이미지 한 장에서 뽑은 feature의 공간 관계를 LSTM으로 처리한 연구다. [Kendall과 Cipolla의 2017년 연구](https://arxiv.org/abs/1704.00390)는 자세 회귀의 손실 함수를 다뤘다. 그중 재투영 손실은 3D 점을 정답 자세와 예측 자세로 각각 투영한 영상 좌표의 차이를 쓴다.

3D 구조를 쓰는 방법과는 차이가 컸다. Kendall과 Cipolla가 2017년 논문에 정리한 표를 보면, SIFT 특징점을 3D 점과 맞추는 [Active Search(Sattler et al. 2012)](https://www.graphics.rwth-aachen.de/media/papers/sattler_eccv12_preprint_1.pdf)는 같은 Cambridge 장면에서 위치 오차가 0.12~0.85 m였다. PoseNet과 그 후속 모델의 오차는 대체로 1~3 m였다. Sattler et al.(2019)은 absolute pose regression의 정확도가 3D 구조 기반 방법보다 DenseVLAD 같은 이미지 검색 방법에 더 가깝다고 분석했다.

---

## 12.2 DeepVO

이미지 한 장 대신 연속 영상을 넣는 방법도 나왔다. Oxford의 Sen Wang 등은 2017년 ICRA에 [Wang et al. 2017. DeepVO](https://arxiv.org/abs/1709.08429)를 발표했다. visual odometry(VO)는 연속 영상에서 카메라가 움직인 양을 추정하는 일이다. DeepVO는 미리 학습한 FlowNet을 바탕으로 만든 CNN으로 연속 두 프레임에서 optical flow feature를 뽑았다. 그다음 LSTM으로 시간 정보를 모아 VO를 바로 출력했다.

> 🔗 **차용.** DeepVO의 훈련 라벨은 KITTI의 GPS/IMU ground truth다. feature 추출 설계는 [FlowNet(Dosovitskiy et al. 2015)](https://arxiv.org/abs/1504.06852)의 optical flow CNN 구조에서 가져왔다.

저자들은 KITTI 시퀀스 00, 02, 08, 09로 학습하고 03–07, 10으로 시험했다. DeepVO의 오차는 스테레오 VISO2-S보다 컸고, 단안 VISO2-M보다는 대체로 작았다. 고속 구간의 이동 오차만 VISO2-M보다 조금 컸다. 00–10 전체로 학습한 모델은 시퀀스 12에서 위치 오차가 크게 났다. 저자들은 학습 데이터에 시속 60 km를 넘는 구간이 시퀀스 01밖에 없었다는 점을 첫 번째 원인으로 들었다. 시퀀스 12의 속도는 시속 50 km에서 약 90 km까지였다.

같은 해 CVPR에는 Tinghui Zhou(UC Berkeley)가 Google 인턴 기간에 주로 진행한 [Zhou et al. 2017. SfMLearner](https://arxiv.org/abs/1704.07813)가 나왔다. 정답 라벨 없이 영상 자체에서 학습 신호를 얻는 자기지도(self-supervised) 학습으로 depth와 ego-motion(카메라 자체의 운동)을 함께 추정했다. 학습 신호는 photometric reprojection loss였다. 한 프레임을 예측한 depth와 운동으로 다른 프레임 시점에 옮겨 그린 뒤, 실제 영상과 밝기를 비교한 값이다.

> 🔗 **차용.** SfMLearner와 [DSO(Engel et al. 2018)](https://arxiv.org/abs/1607.02565)는 기하적으로 대응시킨 영상의 밝기 차이를 이용한다는 원리를 공유한다. DSO는 이 밝기 차이로 자세와 지도를 직접 최적화한다. SfMLearner는 같은 신호로 깊이와 카메라 운동을 예측하는 망을 학습한다.

SfMLearner의 자세 평가는 KITTI 시퀀스 09, 10에서 5프레임짜리 짧은 구간 단위로 했다. 같은 5프레임만 입력받는 ORB-SLAM(short)보다는 오차가 작았다. 시퀀스 전체 프레임을 써서 loop closure와 재위치 추정(re-localization)까지 하는 ORB-SLAM(full)에는 미치지 못했다.

---

## 12.3 기하 제약, 일반화, 불확실성

2019–2020년 무렵 end-to-end pose regression과 학습 기반 odometry에는 아래 문제가 남아 있었다. [Sattler et al.(2019)](https://arxiv.org/abs/1903.07504)이 분석한 것은 이미지 한 장에서 절대 자세를 회귀하는 absolute pose regression이다. 연속 영상에서 상대 운동을 추정하는 learned odometry에는 이 분석을 그대로 옮길 수 없다.

**기하 제약(inductive bias)이 없다.** 고전 SLAM은 epipolar constraint, rigid body motion 가정, scale invariance, 공간의 연속성 같은 기하학적 제약을 알고리즘 구조에 담았다. CNN은 이 제약을 데이터에서 새로 배워야 했다. ImageNet 분류로 미리 학습해 얻은 표현은 3D metric geometry와 거리가 멀다. 회귀 망이 자세를 맞혀도 그 결과가 3D 구조를 반영한 것인지 특정 조명·색상·질감 조합을 기억한 것인지 가려내기 어려웠다.

**학습 분포 밖에서 일반화하기 어렵다.** PoseNet의 출력은 학습한 장면의 좌표계에 묶여 있다. 상대 운동을 추정하는 learned odometry도 카메라나 주행 방식, 영상 분포가 바뀌면 다시 검증해야 한다. CoRL 2020의 [TartanVO](https://proceedings.mlr.press/v155/wang21h.html)는 데이터셋을 넘는 일반화를 주요 설계 문제로 다뤘다. 고전 ORB-SLAM 역시 특징 검출이나 조명 조건이 달라지면 추적을 잃는다.

**불확실성 추정을 검증하기 어렵다.** 경로 계획과 장애물 회피에는 추정한 위치와 함께 그 오차가 얼마나 될지 판단할 정보가 필요하다. EKF는 공분산을 갱신하고, factor graph에서는 추정치 주변의 불확실성을 계산한다. 이 값들도 모델과 근사에 따라 달라진다. Bayesian PoseNet은 dropout으로 불확실성을 추정했다. 원 논문은 이 값을 위치 오차 추정과, 입력이 학습 장면에 속하는지 판별하는 데 썼다. 훈련 분포 밖 입력에서 이 불확실성이 실제 오차와 맞는지는 따로 검증해야 한다.

---

## 12.4 그 뒤

Kendall은 2017년 Cambridge에 박사학위 논문을 냈고, 같은 해 Wayve를 세웠다. [이후 논문](https://alexgkendall.com/)에는 자율주행용 imitation learning과 world model(GAIA-1)이 있다.

TUM의 Tateno·Tombari·Laina·Navab는 [CNN-SLAM(2017)](https://arxiv.org/abs/1704.03489)에서 CNN이 예측한 dense depth를 direct monocular SLAM의 깊이 측정과 합쳤다. 단안 SLAM이 깊이를 잘 잡지 못하는 저질감 영역에서는 예측 깊이를 더 믿고, 기하로 깊이가 잡히는 곳에서는 SLAM 측정을 썼다. 예측 깊이로 절대 스케일을 잡는 방법도 내놓았다. 평가는 TUM RGB-D와 ICL-NUIM에서 했다. 학습과 기하를 합친 초기 시스템이다.

> 📜 **예언 vs 실제.** PoseNet 논문(2015)은 다중 시점 기하로 얻는 학습 자료, 확률적 확장, 한 모델이 학습할 수 있는 장면 규모의 한계를 후속 과제로 제시했다. Bayesian PoseNet(2016)은 그중 확률적 자세 추정과 불확실성 평가를 다뤘다. absolute pose regression의 정확도와 일반화는 그 뒤에도 과제로 남았다.

이 시기의 아이디어 일부는 다른 용도로 쓰였다. SfMLearner의 photometric self-supervision은 MonoDepth2(Godard et al., ICCV 2019) 같은 후속 단안 깊이 연구로 이어졌다. DROID-SLAM(Teed & Deng, NeurIPS 2021)도 미분 가능한 기하를 쓰지만, 정답 pose와 정답 depth로 만든 flow를 손실로 써서 지도 학습한다. DeepVO와 같은 Oxford 그룹의 [VINet(Clark et al., AAAI 2017)](https://arxiv.org/abs/1701.08376)은 LSTM으로 영상과 IMU를 함께 처리해 visual-inertial odometry를 학습했다.

> 📜 **예언 vs 실제.** Zhou 등은 SfMLearner 논문(2017) 맺음말에서 장면 속 움직임과 가림(occlusion)을 명시적으로 모델링하지 않는다는 점, 카메라 내부 파라미터를 알아야 한다는 점, depth map이 3D 장면을 단순화한 표현이라는 점을 남은 과제로 들었다. 장면 속 움직임 문제는 [GeoNet(Yin & Shi 2018)](https://arxiv.org/abs/1803.02276) 같은 후속 self-supervised 연구가 일부 다뤘다. GeoNet은 정적 영역과 동적 영역을 나누어 깊이, optical flow, 카메라 운동을 함께 학습했다.

---

## 12.5 기하와 학습의 분담

2020년 무렵에는 기하 계산을 알고리즘에 맡기고 학습은 feature와 prior에 쓰는 연구가 나왔다.

> 🔗 **차용.** Ch.13에서 다루는 CodeSLAM(Bloesch et al., CVPR 2018)과 DROID-SLAM(Teed & Deng, NeurIPS 2021)이 이런 방식이다. 둘 다 최적화 또는 bundle adjustment를 그대로 두고, 학습은 CodeSLAM에서 depth 표현을, DROID-SLAM에서 dense correspondence와 반복 갱신을 맡는다.

CodeSLAM과 DROID-SLAM은 이미지에서 자세를 바로 회귀하지 않는다. 영상 사이의 대응 관계에 기하 제약을 걸어 자세를 구하고, 그 과정에 학습한 표현을 더했다.

어느 쪽이 나은지는 조건에 따라 달랐다. PoseNet 논문은 SIFT 기반 위치 추정이 크게 실패하는 넓은 시점 차이에서도 자세를 냈다고 보고했다. 해 질 녘, 비, 안개, 밤, motion blur 영상도 시험했다. Sattler et al.(2019)의 평가에서는 absolute pose regression이 3D 구조 기반 위치 추정만큼 정확하지 않았다.

---

## 🧭 아직 열린 것

**어떤 기하 제약을 어디에 넣을지.** 기하를 알고리즘으로 푼다고 해도, rigid body motion이나 epipolar constraint 같은 제약을 어느 수준에서 코드로 넣을지는 아직 정해지지 않았다. 3DGS 기반 시스템들은 geometry를 학습 표현 안에 넣는 방식을 시험하고 있다.

**Learned uncertainty의 calibration.** Bayesian PoseNet 이후에도 딥러닝이 내는 불확실성이 실제 오차와 얼마나 맞는지, 특히 학습 분포 밖 입력에서 맞는지 확인하는 일은 남은 과제다. 자율주행에서 특히 중요한 문제다.

**End-to-end의 범위.** PoseNet의 end-to-end는 이미지에서 자세를 바로 회귀하는 것이었다. foundation model이 나온 2023년 이후로는 SLAM의 어느 모듈을 학습으로 대신하고 어느 모듈을 알고리즘으로 둘지가 다시 논의되고 있다. 알고리즘과 학습의 경계가 흐려지고 있다고 본다.

Ch.13은 CodeSLAM에서 시작한다.
