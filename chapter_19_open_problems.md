# Ch.19 — 오늘의 지도와 내일의 공란

Ch.0은 2026년의 풍경을 이렇게 묘사했다. 핸드폰을 들면 AR 레이어가 벽에 달라붙는다. 실내 배송 로봇은 지도를 받지 않고도 주방과 회의실을 구분한다. DUSt3R 계열 모델에 사진 몇 장을 던지면 수 초 안에 3D 구조가 나온다. 그 묘사는 정확하다. 그리고 그 묘사는 이 책의 전제를 뒷받침하는 동시에 무너뜨린다.

풀린 것은 2003년의 문제다. 2003년의 SLAM이 설정한 가정들, 정적인 장면, 안정된 조명, 제한된 공간, 단안 카메라의 기하학. 그 가정들 위에서 EKF가 작동했고, graph SLAM이 루프를 닫았으며, ORB-SLAM이 keyframe을 관리했다. 각 답은 진짜 답이고, 각 가정은 진지하게 선택된 단순화였다.

18개 챕터의 마지막 절에는 동일한 표시가 남아 있다. 아직 열린 것들. 창작이 아니다. 수확이다. 각 챕터가 풀었다고 선언하는 자리 바로 옆에 꽂아둔 깃발들을 한 자리에 펼쳐놓는 것이다.

---

## 19.1 조명과 환경 변화: 카메라가 감당하지 못하는 현실

Visual SLAM이 실외로 나온 순간부터 따라다닌 문제가 있다. 카메라의 측광 모델이 감당하지 못하는 조건은 현장에서 항상 먼저 도착한다.

Ch.2의 SuperPoint, R2D2, DISK 같은 learned descriptor는 훈련 도메인에서 ORB를 능가한다. 그러나 underwater, thermal, low-light 환경에서는 일관성이 없다. 어느 쪽이 더 강건하다는 합의는 2026년에도 없다 (Ch.2 §2.7 참조). Ch.5가 기록한 저조도·동적 환경 추적 실패는 2007년 PTAM이 "Small AR Workspaces"라고 스스로 범위를 제한했던 이유와 같다. 2026년 대부분의 feature-based SLAM이 암묵적으로 유지하는 가정도 마찬가지다 (Ch.5 §🧭 참조).

Direct method 계보에서 이 문제는 더 구조적이다. Ch.8이 정리했듯, direct tracking의 근본 전제(장면의 밝기 분포가 프레임 간 보존된다)는 자동 노출 카메라, 강한 역광, 터널-야외 전환에서 즉각 붕괴한다. VI-DSO의 IMU 보조가 부분적으로 완화하지만, 조명 모델 자체를 동적으로 추정하는 완전한 해법은 아직 나오지 않았다 (Ch.8 §🧭 참조). Place recognition 계보에서도 같은 장벽이 10년 넘게 같은 자리에 있다. Nordland와 Oxford RobotCar 데이터셋에서 보고되는 계절·조명 극변 문제는 DINOv2 기반 방법들이 격차를 줄였어도, 눈이 쌓인 겨울과 나뭇잎이 무성한 여름을 99% 정확도로 연결하는 단일 모델은 없다 (Ch.10 §10.7 참조).

ORB-SLAM의 장기 지도 재사용도 같은 경계에 막혀 있다. Atlas가 멀티맵 유지를 가능하게 했지만, 아침에 만든 지도와 저녁에 재방문할 때 장소를 같은 곳으로 인식하는 일이 조명 변화 앞에서 실패한다 (Ch.7 §🧭 참조). 이 문제가 Ch.2·5·7·8·10을 거쳐 반복 등장한다는 사실 자체가 하나의 데이터다. 해결된 것이 아니라 각 계보가 각자의 언어로 같은 장벽을 보고한 것이다.

---

## 19.2 동적 세계 가정: 가장 오래된 단순화의 한계

정적 세계 가정은 SLAM의 가장 오래된 단순화다. 그리고 이 가정에 가장 많은 챕터가 각자의 깃발을 꽂았다.

Ch.3의 SfM 계보에서 동적 물체 문제는 COLMAP을 포함한 모든 현행 시스템의 공통 취약점이다. 자동차나 보행자가 많은 장면에서 RANSAC이 일부를 걸러내지만, COLMAP 수준의 범용성을 가진 Dynamic SfM 구현체는 2026년 기준 없다 (Ch.3 §3.7 참조). Ch.9의 KinectFusion부터 BundleFusion까지는 모두 정적 장면을 전제로 설계됐다. 사람이 걸어 다니는 공간을 dense하게 재구성하려면 실시간 semantic segmentation과 dense SLAM의 결합이 필요한데, DynaSLAM·MaskFusion 등이 시도했으나 계산 비용과 robustness 모두에서 실용 배포 수준에 미치지 못한다 (Ch.9 §🧭 참조).

Ch.11의 monocular depth 계보에서는 자동차·사람이 움직이는 장면의 photometric consistency 위반이 핵심이다. self-supervised 방법들이 moving object를 masking으로 우회하는데, 이는 문제를 피하는 것이지 푸는 것이 아니다 (Ch.11 §🧭 참조). Ch.15의 3D Gaussian Splatting SLAM은 2025년 기준으로 정적 세계 가정을 유지한다. 4DGS와 Deformable 3DGS가 시간 차원을 Gaussian에 추가하는 방향을 탐색 중이지만, SLAM 설정에서 동적 객체를 표현하고 추적하는 통합된 방식은 아직 없다 (Ch.15 §🧭 참조). Ch.17의 LiDAR SLAM도 면제되지 않는다. Zhang의 2014년 원 논문이 예견한 동적 물체 처리 문제는 2026년에도 같은 자리에 있다. 점군에서 실시간으로 동적 물체를 분리하는 geometry 기반 방법은 연산 비용이 높고 정확도가 일관되지 않으며, Waymo·Argo AI의 자체 솔루션은 공개된 일반 알고리즘이 아니다 (Ch.17 §🧭 참조).

동적 세계 문제가 다섯 챕터에서 반복 등장한다는 것은 "해결되지 않았다"가 아니라 "해결을 기다리는 올바른 방법이 아직 제안되지 않았다"는 의미일 수 있다.

---

## 19.3 Scale과 표현 메모리: 크기가 달라지면 문제가 달라진다

SLAM 시스템이 방 한 칸에서 건물로, 건물에서 도시로 확장될 때마다 같은 질문이 새로운 형태로 돌아왔다.

Ch.5의 monocular scale 문제는 1980년대 SfM 이론에서 이미 증명된 기하학적 사실이다. IMU나 depth sensor를 추가하면 우회할 수 있지만, 순수 단안 카메라로 metric scale을 안정적으로 유지하는 방법은 Ch.5에서 처음 제기된 이래 형태를 바꿔 돌아온다 (Ch.5 §🧭 참조). Ch.11에서는 같은 질문이 monocular depth 계보의 언어로 등장한다. Metric3D v2와 Depth Anything v2가 camera intrinsic 조건부로 metric depth를 내놓기 시작했지만, "intrinsic을 모르는 상황"(스마트폰 수백 종, CCTV, 역사 아카이브 사진, 위성 이미지)은 흔하다. 카메라 독립적 metric depth는 foundation model 규모에서도 쉽지 않다 (Ch.11 §🧭 참조).

Ch.9의 TSDF 계보에서 메모리 문제는 표현의 한계로 드러났다. Voxblox의 해시 구조, OctoMap의 octree 압축이 비용을 줄였지만, 건물 층 단위·도시 블록 단위의 dense 표현은 여전히 수십 기가바이트다. 어떤 해상도를 어느 영역에서 유지할지를 자동으로 결정하는 adaptive resolution map은 범용 해법이 없다 (Ch.9 §🧭 참조). Ch.14의 NeRF-SLAM도 같은 천장에 막혔다. Instant-NGP가 렌더링 속도를 수백 배 끌어올렸어도 도시 규모 NeRF-SLAM은 개방형 문제다 (Ch.14 §🧭 참조). Ch.15의 Gaussian Splatting에서는 Gaussian의 수가 장면 크기에 따라 선형으로 증가한다. 실내에서 수십만 개로 충분하던 것이 outdoor 도시 구역에서는 수천만 개로 늘어난다. Compact 3DGS 계열이 압축 방향을 탐색 중이지만 합의된 방법은 없다 (Ch.15 §🧭 참조).

Ch.16의 foundation 3D 계보에서 이 문제는 transformer 아키텍처의 물리적 한계로 재정의된다. DUSt3R·VGGT의 transformer는 이미지 수에 quadratic하게 메모리를 요구한다. 100장은 현실적이지만 1,000장, 10,000장은 다른 문제다. Spann3R의 incremental 방식이 부분적 답이지만 대규모 outdoor 처리는 미해결이다 (Ch.16 §🧭 참조). 표현이 바뀌어도 크기의 장벽은 같은 자리에 있다.

---

## 19.4 학습 기반의 신뢰 문제: 조용한 실패와 calibration의 부재

EKF의 inconsistency를 Ch.4에서 Julier·Uhlmann이 증명한 이래, SLAM 시스템이 "자신이 어디 있는지 모른다는 것을 얼마나 정확하게 아는가"는 이 분야의 핵심 질문으로 남아 있다.

Ch.4의 비가우시안 불확실성 문제는 EKF의 가장 근본적인 가정과 닿아 있다. 현실의 센서 오류는 다중 모드이거나 heavy-tail 분포를 갖는 경우가 많다. Stein particle, normalizing flow, 학습 기반 uncertainty estimation이 시도되고 있으나 실시간 SLAM에서 검증된 형태는 제한적이다 (Ch.4 §4.8 참조). Ch.6의 graph SLAM 계보에서 robust cost function의 선택은 여전히 엔지니어의 직관에 의존한다. Huber, Cauchy, Geman-McClure 중 주어진 환경과 센서에 어느 kernel이 최적인지를 사전에 결정하는 원칙적 방법이 없다 (Ch.6 §🧭 참조).

학습 기반 방법에서 이 문제는 더 날카로운 형태로 돌아온다. Ch.12가 기록한 Bayesian PoseNet의 실패 이후에도, 딥러닝 기반 uncertainty estimate가 실제 오차와 얼마나 calibrated 관계를 가지는지는 out-of-distribution 입력에서 특히 2026년에도 열려 있다 (Ch.12 §🧭 참조). Ch.13의 DROID-SLAM 계보가 보여주는 것처럼, learned prior는 훈련 도메인 밖에서 silently degrade한다. Geometric 방법의 실패는 명시적이다. 행렬이 발산하거나 tracking이 끊긴다. Learned method의 실패는 조용하고 그럴듯하다. TartanAir처럼 다양한 합성 데이터로 훈련하는 접근이 있으나 sim-to-real gap이 남는다 (Ch.13 §🧭 참조).

Ch.16의 foundation 3D 계보에서 loop closure의 재정의는 이 문제의 확장이다. 고전 SLAM에서 loop closure는 누적 오차를 교정하는 메커니즘이다. DUSt3R 계열에서 "이전에 방문한 장소"를 어떻게 표현하고, pointmap 기반 지도에서 교정을 어떻게 propagate하는가—MASt3R-SLAM이 기존 방식으로 처리하지만 이것이 원리적 해법인지는 알 수 없다 (Ch.16 §🧭 참조). 자율주행과 의료 로봇이 SLAM을 채택할 때 calibrated uncertainty는 선택이 아니다. 이 문제는 아직 선택으로 취급되고 있다.

---

## 19.5 센서 융합과 새 모달리티: 통합의 미완

Visual SLAM과 LiDAR SLAM은 같은 시기에 같은 문제를 다른 언어로 풀었다. 두 계보가 실질적으로 합쳐진 적은 없다.

Ch.17이 기록하듯, LVI-SAM이 LIO-SAM에 visual odometry를 결합했지만 이것은 두 시스템을 loosely coupled 방식으로 연결한 수준이었다. 안개·강우에서 카메라가 실패하고 LiDAR가 보완해야 하는 시나리오는 자율주행에서 명확하게 요구되지만, tightly coupled 융합의 알고리즘과 센서 캘리브레이션 난이도가 여전히 장벽이다. 2024-2025년 transformer 기반 융합 실험이 진행 중이지만 일관된 결과가 없다 (Ch.17 §🧭 참조). Solid-state LiDAR의 보급이 가져온 알고리즘 공백도 같은 층위다. LOAM·FAST-LIO는 360° spinning LiDAR를 전제한다. Livox·RoboSense의 비반복 스캔 패턴에 맞는 feature extraction과 motion distortion 보정은 별도 연구가 필요하고, 일반화 수준은 미흡하다 (Ch.17 §🧭 참조).

Ch.2의 wide-baseline 매칭 문제는 모달리티 융합의 다른 각도다. 시점 변화가 45도를 넘으면 Harris·ORB 기반 매칭 성능이 급격히 떨어진다. DUSt3R가 matching 자체를 회피하는 방향으로 돌파구를 열었지만, 이것이 descriptor 문제의 종말인지 우회인지는 아직 판단하기 이르다 (Ch.2 §2.7 참조). Ch.10의 place recognition과 metric localization의 통합 문제는 파이프라인 수준의 단절이다. 현재 대부분의 SLAM에서 place recognition은 "어디서 봤는가"만 답하고 실제 pose 추정은 별도 단계가 처리한다. 두 과정을 하나의 표현으로 통합하는 시도들이 2023-2025년에 등장했으나 정밀도와 속도를 동시에 달성한 방법은 없다 (Ch.10 §10.7 참조).

Ch.18의 event camera 문제는 모달리티가 새로울 때 알고리즘이 얼마나 뒤따르는지를 보여주는 사례다. 2022년 이후 상업용 고해상도 event camera가 보급되었지만 event 데이터를 처리하는 안정적인 공통 프레임워크는 없다. frame 기반 pipeline과의 통합, 새로운 event representation, real-world benchmark의 다양화가 동시에 진행 중이다 (Ch.18 §🧭 참조). 센서가 먼저 오고, 알고리즘이 뒤따른다. 그 간격이 얼마나 걸리는지는 Kinect의 역사가 한 번 보여줬다.

---

## 19.6 열린 질문의 구조

이 책이 추적한 18개 챕터의 열린 것들을 모아보면 패턴이 있다.

어떤 문제는 처음 제기된 챕터의 언어 그대로 남아 있다. Ch.5의 monocular scale ambiguity는 SfM 이론에서 이미 증명된 기하학적 사실이고, 2026년에도 같은 정식화로 남아 있다. 어떤 문제는 20년 동안 형태를 바꾸면서 되돌아왔다. 동적 세계 가정은 Ch.3의 SfM 언어로, Ch.9의 dense SLAM 언어로, Ch.15의 Gaussian 언어로, Ch.17의 LiDAR 언어로 각각 다르게 기록되었다. 그리고 어떤 문제는 2026년에야 비로소 문제라는 이름을 얻었다. Foundation 3D 계보에서 loop closure를 어떻게 재정의할 것인지, learned uncertainty를 어떻게 calibrate할 것인지는 그 이름을 얻은 지 몇 년 되지 않았다.

Ch.0은 SLAM이 풀렸다고 여겨지는 시대를 묘사했다. 그 묘사는 정확하다. SLAM의 역사는 새로운 것을 쌓는 역사가 아니라 언제 무엇을 놓아줘야 하는지 배우는 역사다. 어떤 가정을 놓아주는 순간, 이전에 닫혔던 문제가 새로운 형태로 돌아온다. EKF의 선형 가정을 놓자 particle filter가 왔다. Sparse feature를 놓자 dense method가 왔다. Geometric prior를 놓자 learned prior가 왔다. 각 전환은 이전 방법을 폐기한 것이 아니다. 새로운 가정 체계로 넘어간 것이다.

2026년에 풀렸다고 여기는 것도 대부분 이 순환의 한 지점이다. 지도에 다시 공란이 생기는 시점은 우리가 지금 확신하는 가정이 바뀌는 순간이다.
