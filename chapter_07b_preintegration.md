# Ch.7b — 흔들리는 센서에서 제약식으로: IMU Preintegration의 발명

2009년 시드니의 ACFR(Australian Centre for Field Robotics)에서 박사과정생 Todd Lupton과 지도교수 Salah Sukkarieh는 빠른 기동 중 IMU 측정을 factor graph에 묶는 문제를 다뤘다. 드론이 빠르게 기동할 때 IMU는 200Hz로 측정값을 내놓지만, factor graph에 이를 전부 넣기는 어려웠다. 키프레임은 초당 몇 개뿐인데, 그 사이 수십·수백 개 IMU 측정을 어떻게 한 묶음으로 만들 것인가. Lupton이 IROS에 낸 답이 preintegration의 씨앗이었다. 6년 뒤 2015년 RSS, Christian Forster가 Davide Scaramuzza·Luca Carlone·Frank Dellaert와 함께 그 씨앗을 SO(3) 매니폴드 위로 옮겼을 때 IMU는 factor graph의 일등시민이 되었다. Ch.7의 ORB-SLAM3, Ch.8의 VI-DSO, Ch.17의 LIO-SAM은 이 수식을 "Forster 2016을 썼다"로 처리했다. FAST-LIO는 raw IMU 전파와 iterated filter update를 쓰는 다른 경로다.

---

## 7b.1 MEMS와 센서의 민주화

Preintegration이 필요해진 이유는 IMU가 싸졌기 때문이다. 스트랩다운 관성항법의 뿌리는 1950년대 항공우주에 있다. 잠수함·미사일의 ring laser gyro는 수만 달러 장비였고 로봇공학 커뮤니티가 쓸 일은 없었다. 흐름을 바꾼 것은 MEMS(Micro-Electro-Mechanical Systems)였다. Analog Devices의 ADXL, InvenSense의 MPU 시리즈가 6축 IMU를 수 달러로 끌어내렸다. iPhone에 IMU가 들어간 것이 2007년, 2010년대 초반에는 연구용 드론·핸드헬드 장비가 당연히 MEMS IMU를 달았다. 스마트폰 수십억 대가 단가를 떨어뜨리는 시점과 Visual SLAM이 monocular scale ambiguity(Ch.5 §🧭)를 진지하게 고민하는 시점이 겹쳤다.

측정 모델은 단순하다. 가속도계는 중력이 섞인 specific force $\tilde{\mathbf{a}} = \mathbf{R}_w^b(\mathbf{a}^w - \mathbf{g}^w) + \mathbf{b}^a + \boldsymbol{\eta}^a$를, 자이로스코프는 angular velocity $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_b^b + \mathbf{b}^g + \boldsymbol{\eta}^g$를 준다. 여기서 $\mathbf{b}$는 bias, $\boldsymbol{\eta}$는 white noise다. 중력이 항상 섞이고, bias는 시간에 따라 천천히 떠다니며(random walk), MEMS 노이즈는 고주파다. IMU를 쓸 때에는 중력 방향을 추정하고 world frame을 그 방향에 정렬하는 경우가 많다. IMU는 온도·전원 상태마다 bias가 조금씩 달라지는 까다로운 동반자였다.

---

## 7b.2 첫 시도 — Lupton & Sukkarieh (2009 / 2012)

문제는 factor graph의 시간 축이었다. Ch.6가 정리한 Kaess의 iSAM2는 키프레임 단위의 pose를 노드로 삼는다. 그런데 IMU는 키프레임 사이에 수십 번 측정을 던진다. 이 측정을 전부 노드로 만들면 그래프가 폭발하고, 버리면 정보가 사라진다.

Lupton과 Sukkarieh의 [Visual-Inertial-Aided Navigation for High-Dynamic Motion (IROS 2009, TRO 2012)](https://doi.org/10.1109/TRO.2011.2170332)이 내놓은 답은 우회였다. 키프레임 $i$에서 $j$ 사이의 IMU 측정을 *한 번만* 수치 적분해 상대 증분을 만들어 놓자. 그 증분을 하나의 factor로 삼으면 IMU 원측정은 그래프에 들어갈 필요가 없다. "pre-integration"이라는 이름이 여기서 나왔다.

아이디어는 맞았지만 구현에 두 장애물이 있었다. 회전 표현이 Euler angle이었다(gimbal lock이 있고 매니폴드가 아니다). 더 큰 문제는 bias였다. BA 한 번 돌 때마다 bias 추정치가 바뀌고, bias가 바뀌면 증분도 달라진다. 증분을 매번 재적분하면 키프레임당 수백 개 측정을 다시 처리해야 한다. Lupton도 이를 줄이기 위한 1차 편향 보정을 제안했다. 6년 뒤 Forster는 회전의 매니폴드 구조를 다루면서 그 보정을 SO(3) 위에서 정식화했다.

---

## 7b.3 변곡점 — Forster-Carlone (2015 / 2017)

2015년 RSS, ETH Zürich의 박사과정생 Christian Forster가 Scaramuzza(UZH), Carlone(Georgia Tech, 이후 MIT), Dellaert(Georgia Tech, GTSAM의 창시자)와 함께 [IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf)을 냈다. 2017년 IEEE TRO에 확장판 [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321)가 실렸다. UZH의 민첩한 드론 실험, Georgia Tech의 GTSAM factor graph 언어, Carlone의 최적화 이론이 한 논문에서 만났다.

재정의가 셋이었다. $\Delta\mathbf{R}_{ij}$를 SO(3) 매니폴드 위 상대 회전으로 엄밀히 정의하고, $\Delta\mathbf{v}_{ij}, \Delta\mathbf{p}_{ij}$를 *중력과 초기 상태에 독립*이 되도록 재정의한 것이 첫째였다. 이 양들은 물리적 증분이 아니라 수학적으로 state-independent하게 만들어진 양이다. 사전적분량을 다시 계산하지 않고도 IMU factor를 평가할 수 있지만, 잔차에는 양 끝 pose와 velocity뿐 아니라 중력과 편향 보정도 들어간다. 둘째, noise를 지수사상 끝으로 밀어내는 right Jacobian trick으로 공분산 $\boldsymbol{\Sigma}_{ij}$를 해석적으로 propagate했다.

셋째는 **Bias 1차 Jacobian 선형 보정**이었다. BA 반복 중 bias가 조금 바뀌었을 때 증분 전체를 재적분하지 말고 precomputed 편미분으로 1차 근사 보정하자는 것이다. Lupton의 Euclidean 선형화와 같은 아이디어지만 SO(3) 위에서 작동한다. 키프레임 사이를 처음 적분할 때 편미분을 함께 계산해 두면, 선형화점이나 bias가 조금 바뀔 때 측정값 전체를 다시 적분하지 않고 1차 보정으로 factor를 갱신할 수 있다. 이 계산 절감 덕분에 IMU factor를 실시간 최적화 안에서 다루기 쉬워졌다.

GTSAM에 Forster의 구현이 레퍼런스로 올라갔다. 후속 시스템들은 수식을 다시 쓰지 않고 `ImuFactor`를 `#include`했다.

> 🔗 **공통 언어.** Forster의 manifold preintegration은 SO(3)의 지수사상과 right Jacobian으로 회전 불확실성을 다룬다. [Barfoot 2017. *State Estimation for Robotics*](https://doi.org/10.1017/9781316671528)는 뒤이어 이 Lie group 계산을 상태 추정의 공통 체계로 정리했다. 2017년 책이 2015년 논문의 원전인 것은 아니지만, 두 작업은 같은 수학적 언어를 보여 준다. Forster의 기여는 Lupton의 preintegration 아이디어를 Euler angle이 아닌 SO(3) 위에서 전개한 데 있다.

> 🔗 **차용.** Bias 1차 Jacobian 아이디어 자체는 [Lupton & Sukkarieh 2012](https://doi.org/10.1109/TRO.2011.2170332)가 먼저 제시했다. Forster et al. TRO 2016 §VIII-B는 이 부채를 명시하며 "we follow [Lupton-Sukkarieh] but operate directly on SO(3)"라 적었다. Euclidean 근사를 매니폴드 위로 옮기자 같은 수학이 실시간이 되었다.

---

## 7b.4 실무 VIO 3파의 정립

Forster 공식이 자리 잡자 2017-2022년 사이 VIO 시스템이 세 갈래로 뻗었다.

첫 갈래는 필터 계열이고 뿌리는 Forster보다 앞서 있다. 2007년 UC Riverside의 Anastasios Mourikis와 Stergios Roumeliotis가 ICRA에 낸 [MSCKF(Multi-State Constraint Kalman Filter)](https://doi.org/10.1109/ROBOT.2007.364024)가 출발점이었다. stochastic cloning으로 과거 여러 카메라 포즈를 필터 상태에 유지한다. 관측한 3D 점은 상태에 추가하지 않고, 선형화한 관측식을 점 Jacobian의 left null space로 투영해 점 위치 오차에 대한 의존성을 제거한다. preintegration 없이 EKF 뼈대로 Visual-Inertial을 실시간으로 돌린 초기에 큰 영향을 준 사례였다. 2021년 화성에서 NASA JPL의 Mars helicopter Ingenuity가 돌린 추정기가 MSCKF 계열이었다. University of Delaware의 Guoquan Huang 그룹이 2020년 [OpenVINS](https://doi.org/10.1109/ICRA40945.2020.9196524)로 오픈소스화했다.

두 번째 갈래는 최적화 계열이다. HKUST의 Shaojie Shen 그룹과 박사과정생 Tong Qin이 2018년 TRO에 낸 [VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729)가 대표작이다. Forster 공식을 그대로 받아 sliding-window tightly-coupled BA 안에 IMU factor로 심고, 초기화 단계에서 scale과 gravity 방향을 분리 추정하는 절차를 정리했다. 코드가 공개되어 2019-2022년 학회의 VIO baseline이 됐다. Ch.7의 ORB-SLAM3가 EuRoC 11개 시퀀스 평균 ATE 0.043m로 보고될 때 같은 표에서 VINS-Mono는 0.110m였다.

세 번째 갈래는 direct 계열이다. Ch.8에서 다룬 VI-DSO(2018), Basalt(2019), [DM-VIO(2022)](https://doi.org/10.1109/LRA.2021.3140129)가 여기 속한다. TUM Cremers 그룹이 DSO의 photometric BA 위에 Forster의 inertial factor를 얹은 적층 구조였다. DM-VIO는 *delayed marginalization*을 더했다. IMU 초기화가 수렴하기 전에 섣불리 marginalize하면 잘못된 prior가 고정돼 장기 drift를 유발하는데, 두 marginalization prior를 병렬로 유지하다 gravity와 scale이 관측된 뒤 최종 prior로 합치는 방식이다.

---

## 7b.5 Observability — 무엇을 못 보는가

Huang 그룹이 2010년대 초부터 정리한 분석에 따르면, 단서 없는 visual-inertial 시스템의 null space는 **4차원**, 즉 3차원 global position과 1차원 yaw-around-gravity다. 절대 좌표와 중력 축 회전은 IMU와 카메라만으로는 영원히 알 수 없다. GPS를 더하면 position이, 자기장이나 외부 anchor를 더하면 yaw가 복원된다. 순수 VIO는 이 4차원 부분공간을 구조적으로 볼 수 없다.

*roll과 pitch는 보인다*. 가속도계가 중력으로 수평을 읽기 때문이다. IMU를 붙이면 Ch.5가 지적한 monocular scale ambiguity도 해결된다.

더 까다로운 쪽은 degenerate motion이다. 전역 yaw의 비관측성은 운동에 관계없이 남으며, 시차나 관성 운동의 변화가 부족하면 깊이·scale·편향을 추가로 구별하기 어려워질 수 있다. 드론의 hover나 자동차의 일정 속도 직진에서는 이런 운동 자극 부족을 살펴야 한다. 이륙·제동·선회가 정보를 더할 수 있지만, 특정 동작 하나가 scale 추정을 보장하지는 않는다.

> 📜 **예언 vs 실제.** Forster et al.은 2017년 TRO §IX에서 세 방향을 꼽았다. time-synchronization과 online extrinsic calibration의 통합, long-term operation에서 bias random walk 가정 검증, event camera·rolling shutter 같은 비동기 센서로의 확장. 2026년 시점에서 첫 번째는 VINS-Mono·Kalibr·OpenVINS가 시간 offset을 상태 변수로 올리며 표준화됐고, 두 번째는 navigation-grade IMU에서는 맞지만 consumer MEMS에서는 온도·전원 변동이 여전히 남았으며, 세 번째는 Le Gentil의 GP 연속시간 preintegration이 답의 한 갈래가 되었다. 예언은 대체로 적중했으나 저자들이 그린 단일 확장이 아니라 세 갈래로 분화했다.

---

## 7b.6 Continuous-time 분기

2021년 RSS, 시드니의 UTS(University of Technology Sydney)에서 Cédric Le Gentil과 지도교수 Teresa Vidal-Calleja가 [Continuous Integration over SO(3) for IMU Preintegration](https://roboticsproceedings.org/rss17/p075.pdf)을 냈다. 같은 시드니였다. Lupton의 ACFR에서 몇 km 떨어진 곳에서 같은 문제를 다른 각도로 다시 본 셈이다.

Forster의 preintegration은 discrete하다. IMU 측정 사이를 piecewise-constant로 가정하고 Euler integration한다. LiDAR·event camera처럼 비동기 센서가 섞이면 IMU 표본 사이 시각의 상태를 평가해야 한다. 비동기성 자체가 구간 상수 근사를 깨뜨리는 것은 아니지만, 시간 보간의 정확도가 중요해진다. Le Gentil의 답은 IMU를 **Gaussian Process**로 모델링해 angular velocity를 연속 함수로 본 것이다. 임의의 시간 $\tau$에서 상태를 평가할 수 있으니 비동기 측정이 자연스럽게 들어온다.

이 방향은 B-spline·STEAM·GPMP 계보와 만난다.

---

## 7b.7 차용의 지형

> 🔗 **차용.** Factor graph 위에서 IMU factor를 평가·최적화하는 골격은 Ch.6에서 정리한 [Dellaert의 GTSAM](https://gtsam.org/) 전통 그대로다. Forster의 `ImuFactor`는 GTSAM의 `NoiseModelFactor` 인터페이스에 꽂혀 visual reprojection factor와 나란히 하나의 `Values` 객체로 최적화되었다. 소프트웨어 구조의 상속이었다.

> 🔗 **차용.** Bias를 random walk로 다루는 방식은 Ch.4의 Kalman filter state propagation 관습에서 왔다. Lupton 이전부터 항법 커뮤니티가 "bias를 상태에 포함하고 process noise를 작게 주는" 모델을 썼고, preintegration 시대에는 이것이 bias random walk factor로 재해석됐다.

---

## 🧭 아직 열린 것

**Visual-inertial observability의 실시간 감지.** 4차원 null space와 degenerate motion 표는 이론적으로 정리됐지만, 실제 시스템이 "지금 내가 degenerate 구간에 있다"를 판단하는 메커니즘은 미완성이다. Hesch·Li·Huang 계열의 FEJ(First-Estimate Jacobian)가 선형화 시점의 null space를 보존하지만, 런타임에 degenerate 조건의 시작·종료를 포착해 제어 루프에 피드백하는 널리 합의된 방법은 2026년 기준 없다. 드론 제어와 VIO 추정이 계산 자원을 공유할 때 이 공백은 추정 실패를 제어기가 늦게 알아차리는 위험으로 이어진다.

**Preintegration과 continuous-time의 통합.** Forster의 이산 증분과 Le Gentil의 GP 연속표현은 같은 문제를 다른 수학 언어로 푼다. LiDAR·event·frame 카메라를 섞어 쓸 때 어떤 표현을 밑바닥에 깔 것인가는 아직 엔지니어링 선택의 문제다. B-spline 연속시간 BA가 부분적 답을 내놓았지만 배포 시스템 다수는 여전히 Forster의 이산 factor를 쓴다.

**학습 기반 IMU bias 모델.** Bias random walk 가정은 navigation-grade IMU에서는 맞지만, consumer MEMS에서는 온도 hysteresis와 전원 과도 현상 때문에 어긋난다. TLIO·RoNIN 계열이 LSTM·Transformer로 IMU-only odometry의 bias를 학습했고, 최근에는 conditional diffusion으로 bias 분포 자체를 모델링하는 시도가 나왔다. 이 접근을 Forster factor 안에 어떻게 넣을지, 학습 모델을 결합한 뒤에도 preintegration의 수학적 우아함이 어디까지 유지될지는 다음 질문이다.

---

Lupton이 시드니에서 시작한 아이디어가 6년 동안 Euler angle의 벽에 갇혀 있었고, Forster가 SO(3)로 옮겨 bias Jacobian의 자물쇠를 풀었고, Le Gentil이 다시 시드니에서 연속시간으로 가지를 쳤다. 세 세대의 작업이 ORB-SLAM3, VI-DSO, LIO-SAM의 한 줄 뒤에 쌓여 있다.
