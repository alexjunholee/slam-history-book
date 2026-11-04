# Ch.7b — IMU Preintegration: IMU 측정을 factor graph의 제약으로

2009년 시드니 대학교 ACFR(Australian Centre for Field Robotics)의 Todd Lupton과 Salah Sukkarieh는 빠르게 움직이는 플랫폼에서 얻은 IMU 측정을 추정기에 넣는 문제를 다뤘다. IMU는 초당 수백 번 측정값을 내지만 keyframe은 초당 몇 개뿐이다. 그 사이에 쌓이는 측정 수십·수백 개를 한 묶음으로 다룰 방법이 필요했다. Lupton과 Sukkarieh가 IROS 2009에 낸 논문에서 preintegration(사전 적분)이 시작됐다. 6년 뒤인 2015년 RSS에서 Christian Forster는 Luca Carlone, Frank Dellaert, Davide Scaramuzza와 함께 이 방법을 SO(3) manifold 위에서 다시 정식화했다. Ch.7의 ORB-SLAM3, Ch.8의 VI-DSO, Ch.17의 LIO-SAM은 이 수식을 따로 유도하지 않고 Forster의 논문을 인용한다. FAST-LIO는 다른 방식을 쓴다. IMU 원측정으로 상태를 그대로 전파하고 iterated filter로 갱신한다.

---

## 7b.1 MEMS IMU의 보급

preintegration이 필요해진 것은 IMU가 싸진 뒤였다. 관성항법은 1950년대 항공우주 분야에서 시작됐다. 잠수함이나 미사일에 쓰던 ring laser gyro는 수만 달러짜리 장비여서 로봇 연구자들이 쓸 일이 없었다. 이 사정을 바꾼 것이 MEMS(Micro-Electro-Mechanical Systems, 반도체 공정으로 만든 초소형 기계 소자)다. Analog Devices의 ADXL 가속도계와 InvenSense의 MPU 시리즈가 나오면서 6축 IMU 가격이 몇 달러로 내려갔다. 2007년 첫 iPhone에 가속도계가 들어갔고, 2010년 iPhone 4에 자이로스코프가 들어갔다. 스마트폰이 수십억 대 팔리며 단가가 떨어지던 무렵, Visual SLAM 연구자들은 단안 카메라의 scale 문제(Ch.5 🧭)를 본격적으로 고민하고 있었다.

측정 모델은 간단하다. 가속도계는 중력이 섞인 specific force(비력) $\tilde{\mathbf{a}} = \mathbf{R}_w^b(\mathbf{a}^w - \mathbf{g}^w) + \mathbf{b}^a + \boldsymbol{\eta}^a$를 재고, 자이로스코프는 각속도 $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_b^b + \mathbf{b}^g + \boldsymbol{\eta}^g$를 잰다. 여기서 $\mathbf{b}$는 bias, $\boldsymbol{\eta}$는 white noise다. bias는 시간에 따라 천천히 변하므로 random walk로 모델링한다. bias는 온도와 전원 상태에 따라서도 조금씩 달라져 다루기 까다로웠다. MEMS 잡음은 고주파 성분이 크다. 가속도계 측정에는 늘 중력이 섞여 있다. 그래서 IMU를 쓰는 시스템은 중력 방향을 추정해 world frame을 그 방향에 맞추는 경우가 많다.

---

## 7b.2 첫 시도: Lupton & Sukkarieh (2009 / 2012)

어려움은 측정 주기의 차이에서 왔다. Ch.6에서 본 iSAM2는 keyframe마다 pose를 노드로 둔다. 그런데 IMU는 keyframe 사이에 측정을 수십 번 낸다. 측정마다 노드를 만들면 그래프가 감당할 수 없이 커지고, 측정을 버리면 정보를 잃는다.

Lupton과 Sukkarieh는 IROS 2009 논문 [Efficient Integration of Inertial Observations into Visual SLAM without Initialization](https://doi.org/10.1109/IROS.2009.5354267)에서 이 문제에 답했다. 2012년 IEEE TRO에는 확장판 [Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions](https://doi.org/10.1109/TRO.2011.2170332)이 실렸다. 두 사람은 keyframe $i$와 $j$ 사이의 IMU 측정을 *한 번만* 적분해 상대 증분을 만들어 두었다. 이 증분은 초기 자세나 속도를 몰라도 계산할 수 있다. 증분을 제약 하나로 쓰면 IMU 원측정을 하나하나 추정기에 넣지 않아도 된다. "pre-integration"이라는 이름이 이 논문에서 나왔다.

구현에는 걸림돌이 있었다. 먼저 회전을 Euler angle로 나타냈다. Euler angle에는 특이점(gimbal lock)이 있고, 회전이 이루는 manifold 구조를 반영하지 못한다. 더 큰 문제는 bias였다. 최적화를 한 번 돌릴 때마다 bias 추정값이 바뀌고, bias가 바뀌면 증분도 달라진다. 증분을 매번 다시 적분하면 keyframe마다 측정 수백 개를 다시 처리해야 한다. Lupton과 Sukkarieh는 bias가 바뀐 만큼 증분을 1차 근사로 고치는 방법을 함께 내놓았다. 6년 뒤 Forster 등은 이 보정을 SO(3) 위에서 다시 정식화했다.

---

## 7b.3 Forster 등의 on-manifold preintegration (2015 / 2017)

2015년 RSS에서 University of Zurich(UZH)의 박사과정생 Christian Forster가 [IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf)을 발표했다. 공저자는 지도교수 Davide Scaramuzza(UZH)와 Georgia Tech의 Luca Carlone(이후 MIT), Frank Dellaert(GTSAM을 만든 사람)였다. 2017년 IEEE TRO에는 확장판 [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321)가 실렸다. 저자들은 GTSAM의 iSAM2 위에 VIO를 구현했다. 실험에는 VI-Sensor로 기록한 430 m 실내 궤적을 썼고, 센서를 Google Tango 기기에 붙여 사무실 건물 둘레를 걸은 실외 궤적도 썼다.

Forster 등은 먼저 $\Delta\mathbf{R}_{ij}$를 SO(3)의 상대 회전으로 엄밀하게 정의했다. 이어 $\Delta\mathbf{v}_{ij}, \Delta\mathbf{p}_{ij}$를 *중력과 초기 상태에 독립*이 되도록 다시 정의했다. 이 양은 물리적인 속도·위치 변화량과 다르다. 상태에 의존하지 않도록 수학적으로 구성한 값이다. 그래서 한 번 계산한 preintegration 값으로 IMU factor를 여러 번 평가할 수 있다. residual에는 양 끝의 pose와 속도, 중력, bias 보정이 함께 들어간다. 다음으로 잡음 항을 지수 사상 바깥으로 빼내는 right Jacobian 기법을 써서 공분산 $\boldsymbol{\Sigma}_{ij}$를 해석적으로 전파했다.

마지막은 **bias 1차 Jacobian 보정**이다. 최적화를 반복하는 동안 bias가 조금 바뀌면, 미리 계산해 둔 편미분으로 증분을 1차 근사만큼 고친다. keyframe 사이를 처음 적분할 때 편미분을 함께 계산해 두면, bias가 조금 바뀌어도 다시 적분하지 않고 factor를 갱신할 수 있다. 계산이 이만큼 줄어서 IMU factor를 실시간 최적화에 넣기 쉬워졌다.

저자들은 preintegration 구현을 GTSAM 4.0에 공개했다. 그 뒤 여러 시스템이 수식을 직접 구현하지 않고 GTSAM의 `ImuFactor`를 `#include`해 썼다.

> 🔗 **공통 수학.** Forster의 on-manifold preintegration은 SO(3)의 지수 사상과 right Jacobian으로 회전의 불확실성을 다룬다. [Barfoot 2017. *State Estimation for Robotics*](https://doi.org/10.1017/9781316671528)는 이런 Lie group 계산을 상태 추정 전반에 걸쳐 정리했다. 이 책은 Forster의 2015년 논문보다 2년 뒤에 나왔다. 두 작업은 같은 수학을 쓴다.

> 🔗 **차용.** bias 1차 보정은 [Lupton & Sukkarieh 2012](https://doi.org/10.1109/TRO.2011.2170332)가 먼저 제시했다. Forster et al.은 자신들의 보정이 "similar to the bias correction in [Lupton & Sukkarieh] but operates directly on SO(3)"라고 적었다.

---

## 7b.4 실제 VIO 시스템들

2007–2022년에 나온 VIO 시스템은 필터 기반, 최적화 기반, direct 방식으로 나눠 볼 수 있다.

필터 기반 VIO는 Forster보다 먼저 나왔다. 2007년 University of Minnesota의 Anastasios Mourikis와 Stergios Roumeliotis가 ICRA에 낸 [MSCKF(Multi-State Constraint Kalman Filter)](https://doi.org/10.1109/ROBOT.2007.364024)에서 시작했다. MSCKF는 stochastic cloning(과거 상태를 복제해 필터에 남기는 기법)으로 과거 카메라 포즈 여러 개를 필터 상태에 남겨 둔다. 관측한 3D 점은 상태에 넣지 않는다. 대신 선형화한 관측식을 점 Jacobian의 left null space로 투영해, 점 위치 오차가 관측식에 끼치는 영향을 없앤다. preintegration 없이 EKF로 visual-inertial 추정을 실시간으로 돌린 초기 사례다. 2021년 화성에서 비행한 NASA JPL의 Ingenuity 헬리콥터는 MAVeN이라는 EKF로 위치를 추정했다. MAVeN도 base image를 찍은 시점의 위치와 자세를 복제 상태(clone state)로 필터에 넣는다. University of Delaware의 Guoquan Huang 그룹은 2020년 MSCKF 기반 [OpenVINS](https://doi.org/10.1109/ICRA40945.2020.9196524)를 오픈소스로 공개했다.

최적화 기반의 대표는 HKUST의 Tong Qin, Peiliang Li, Shaojie Shen이 2018년 TRO에 낸 [VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729)다. VINS-Mono는 preintegration한 IMU 측정을 sliding window tightly-coupled BA에 factor로 넣는다. 초기화 단계에서 scale과 중력 방향을 따로 추정하는 절차도 정리했다. 코드가 공개되어 있어 2019–2022년 학회 논문들이 VIO 비교 기준으로 썼다. Ch.7의 ORB-SLAM3 논문 표를 보면, EuRoC 11개 시퀀스의 단안-관성 평균 ATE가 ORB-SLAM3는 0.043 m, VINS-Mono는 0.110 m였다.

direct 방식으로는 Ch.8에서 다룬 VI-DSO(ICRA 2018)와 [DM-VIO(RA-L 2022)](https://doi.org/10.1109/LRA.2021.3140129)가 있다. TUM Cremers 그룹은 VI-DSO에서 DSO의 photometric BA에 inertial factor를 더했다. DM-VIO에서는 *delayed marginalization*을 더했다. IMU 초기화가 끝나기 전에 marginalize하면 잘못된 scale이 prior에 굳는다. DM-VIO는 주 그래프 옆에 marginalization을 일정 keyframe 수만큼 늦게 하는 두 번째 factor graph를 둔다. IMU 초기화가 끝나면 이 그래프에 IMU factor를 넣고 다시 marginalize해, 주 시스템에 쓸 새 prior를 만든다.

---

## 7b.5 Observability

Minnesota의 Hesch, Kottas, Bowman, Roumeliotis는 2014년 TRO 논문에서 외부 기준이 없는 visual-inertial 시스템에 관측할 수 없는 방향이 **넷** 있음을 보였다. 전역 위치 3자유도와 중력축 둘레의 회전(yaw) 1자유도다. 이 네 방향으로 상태를 옮겨도 IMU와 카메라 측정값은 그대로다. GPS를 더하면 위치를, 자기장이나 외부 기준점을 더하면 yaw를 구할 수 있다.

*roll과 pitch는 관측된다*. 가속도계가 중력 방향을 재므로 수평을 알 수 있다. IMU를 붙이면 Ch.5에서 본 단안 카메라의 scale도 관측할 수 있다. 다만 아래처럼 운동이 충분해야 한다.

더 까다로운 것은 degenerate motion(관측 정보가 부족해지는 운동)이다. 전역 yaw는 어떻게 움직여도 관측할 수 없다. 여기에 시차나 가속이 부족하면 깊이·scale·bias까지 구별하기 어려워질 수 있다. 드론이 제자리 비행을 하거나 자동차가 일정한 속도로 직진할 때가 그렇다. 이륙, 제동, 선회는 정보를 더하지만, 어떤 동작 하나가 scale 추정을 보장하지는 않는다.

> 📜 **예언 vs 실제.** Forster et al.의 TRO 논문 결론(§IX)은 앞으로 할 연구를 따로 꼽지 않았다. 대신 preintegration과 structureless vision factor의 코드를 GTSAM 4.0에 공개한다고 밝혔다. 그 뒤 카메라–IMU 시간 offset은 Kalibr, VINS-Mono, OpenVINS가 추정 대상에 넣으면서 흔히 함께 추정하는 값이 됐다. 비동기 센서 문제에는 Le Gentil의 GP 연속시간 preintegration이 한 가지 답을 내놓았다.

---

## 7b.6 연속시간 preintegration

2021년 RSS에서 UTS(University of Technology Sydney)의 Cédric Le Gentil과 Teresa Vidal-Calleja가 [Continuous Integration over SO(3) for IMU Preintegration](https://roboticsproceedings.org/rss17/p078.pdf)을 발표했다. UTS는 Lupton이 연구한 ACFR과 같은 시드니에 있다.

Forster의 preintegration은 이산적이다. IMU 측정 사이에서는 값이 일정하다고 보고 Euler 적분을 한다. LiDAR나 event camera처럼 측정 시각이 IMU와 맞지 않는 센서가 섞이면, IMU 표본 사이 시각의 상태를 계산해야 한다. 측정 시각이 어긋난다고 구간 상수 근사가 깨지지는 않지만, 시간 보간이 얼마나 정확한지가 중요해진다. Le Gentil은 각속도를 **Gaussian Process**(GP)로 모델링해 연속 함수로 다뤘다. 아무 시각 $\tau$에서나 상태를 계산할 수 있으므로 비동기 측정을 그대로 넣을 수 있다. 논문은 IMU bias와 센서 사이 시간 차이를 보정하는 방법도 함께 냈다.

이 연구는 B-spline, STEAM, GPMP 같은 연속시간 추정 연구와 이어진다(Ch.7c).

---

## 7b.7 가져온 것들

> 🔗 **차용.** IMU factor를 factor graph에서 평가하고 최적화하는 구조는 Ch.6에서 본 [Dellaert의 GTSAM](https://gtsam.org/)을 그대로 따른다. `ImuFactor`는 GTSAM의 `NoiseModelFactor` 인터페이스를 구현하므로, visual reprojection factor와 함께 하나의 `Values` 객체로 최적화된다.

> 🔗 **차용.** bias를 random walk로 다루는 방식은 Ch.4에서 본 Kalman filter의 상태 전파 관습에서 왔다. 항법 분야는 Lupton 이전부터 bias를 상태에 넣고 process noise를 작게 주는 모델을 썼다. preintegration에서는 이 모델이 bias random walk factor가 됐다.

---

## 🧭 아직 열린 것

**관측 가능성을 실시간으로 감지하기.** 4차원 null space와 degenerate motion은 이론으로 정리됐다. Huang, Mourikis, Roumeliotis가 제안한 FEJ(First-Estimate Jacobian)는 선형화할 때 null space를 보존한다. 실행 중에 degenerate 구간의 시작과 끝을 알아내 제어 루프에 알려 주는 일은 남은 과제다.

**Preintegration과 continuous-time의 통합.** Forster의 이산 증분과 Le Gentil의 GP 연속 표현은 같은 문제를 다른 수학으로 푼다. LiDAR, event camera, 일반 카메라를 섞어 쓸 때 어느 표현을 기본으로 삼을지는 정해지지 않았다. B-spline 연속시간 BA는 Ch.7c에서 다룬다.

**학습 기반 IMU bias 모델.** bias random walk 가정은 navigation-grade IMU에서는 맞지만, consumer MEMS에서는 온도 hysteresis와 전원 과도 현상 때문에 어긋난다. TLIO와 RoNIN 같은 연구는 IMU만 쓰는 odometry를 신경망으로 학습했다. 최근에는 conditional diffusion으로 bias 분포를 모델링하려는 시도도 나왔다. 이런 학습 모델을 Forster의 factor에 어떻게 넣을지는 아직 정해지지 않았다.

---

Le Gentil의 연속시간 preintegration은 더 넓은 흐름의 한 갈래다. Ch.7c는 궤적 자체를 시간의 연속 함수로 두는 방법을 다룬다.
