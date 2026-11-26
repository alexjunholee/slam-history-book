# 9장 IMU 사전적분

2009년 시드니 대학교 ACFR(Australian Centre for Field Robotics)의 Todd Lupton과 Salah Sukkarieh는 빠르게 기동하는 플랫폼에서 쏟아지는 IMU 측정을 SLAM 추정기에 어떻게 넣을지 고민하고 있었다. IMU는 1초에 수백 번씩 데이터를 내보내지만, 영상 keyframe은 1초에 몇 장 남짓이다. 그 사이에 쌓이는 수십, 수백 개의 관성 측정을 하나의 제약으로 압축해 다룰 방법이 필요했다. 두 사람이 IROS 2009에 발표한 논문이 바로 preintegration(사전적분)의 출발점이었다. 6년 뒤인 2015년 RSS에서 Christian Forster는 Luca Carlone, Frank Dellaert, Davide Scaramuzza와 손잡고 이 아이디어를 SO(3) manifold 위에서 엄밀하게 다시 세웠다. 그래서 오늘날 ORB-SLAM3, VI-DSO, LIO-SAM 같은 대표 시스템들은 수식을 새로 유도하지 않고 Forster의 논문을 그대로 인용한다(→ 8장, 11장, 21장). 반면 FAST-LIO는 preintegration 대신 IMU 원측정으로 상태를 곧장 전파한 뒤 iterated filter로 갱신하는 다른 길을 택했다.

---

## 9.1 MEMS IMU의 보급

preintegration 이론이 현실적인 필요로 떠오른 배경에는 IMU의 극적인 가격 하락이 있다. 관성항법 자체는 1950년대 항공우주 분야에서 시작됐지만, 잠수함이나 미사일에 들어가던 ring laser gyro는 수만 달러를 호가해 로봇 연구자가 쉽게 만질 수 있는 장비가 아니었다. 이 장벽을 허문 것이 반도체 공정으로 만든 초소형 기계 소자 MEMS(Micro-Electro-Mechanical Systems)였다. Analog Devices의 ADXL 가속도계와 InvenSense의 MPU 시리즈가 보급되면서 6축 IMU 가격이 몇 달러 수준으로 떨어졌고, 2007년 첫 iPhone에 가속도계가, 2010년 iPhone 4에 자이로스코프가 차례로 탑재됐다. 스마트폰이 수십억 대씩 팔리며 센서 단가가 급락하던 바로 그 무렵, Visual SLAM 연구자들은 단안 카메라가 풀지 못하던 scale 모호성(→ 5장 「아직 열린 문제」)을 해결할 현실적인 짝을 찾고 있었다.

MEMS IMU의 측정 모델 자체는 간결하다. 가속도계는 중력이 섞인 비력(specific force) $\tilde{\mathbf{a}} = \mathbf{R}_w^b(\mathbf{a}^w - \mathbf{g}^w) + \mathbf{b}^a + \boldsymbol{\eta}^a$를 재고, 자이로스코프는 각속도 $\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_b^b + \mathbf{b}^g + \boldsymbol{\eta}^g$를 잰다. 여기서 $\mathbf{b}$는 bias, $\boldsymbol{\eta}$는 white noise다. 수식은 간단해 보이지만 실제 MEMS 소자는 다루기가 만만치 않다. 고주파 잡음이 큰 데다, bias는 시간에 따라 서서히 흐르기 때문에 random walk로 모델링해야 하고, 온도와 전원 상태에 따라서도 조금씩 요동친다. 게다가 가속도계 측정에는 언제나 중력이 함께 실려 있으므로, IMU를 쓰는 시스템은 중력 방향을 함께 추정해 world frame을 중력축에 맞춰 정렬한다.

---

## 9.2 첫 시도: Lupton & Sukkarieh (2009 / 2012)

이론적 난관은 카메라와 IMU 사이의 극심한 측정 주기 차이에서 비롯됐다. iSAM2 같은 그래프 기반 추정기는 keyframe마다 pose 노드를 하나씩 세운다. 그런데 두 keyframe 사이에도 IMU는 수십 번씩 측정을 뱉어 낸다. 관성 측정마다 노드를 새로 세우면 그래프가 감당할 수 없을 만큼 비대해지고, 그렇다고 중간 측정을 버리면 귀중한 동역학 정보를 잃는다.

Lupton과 Sukkarieh는 IROS 2009 논문 [Efficient Integration of Inertial Observations into Visual SLAM without Initialization](https://doi.org/10.1109/IROS.2009.5354267)에서 이 딜레마를 정면으로 풀었고, 2012년 IEEE TRO에 확장판 [Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions](https://doi.org/10.1109/TRO.2011.2170332)을 실었다. 핵심 발상은 keyframe $i$와 $j$ 사이의 IMU 측정을 *단 한 번만* 적분해 상대 증분을 미리 구해 두는 것이었다. 이 증분은 로봇의 초기 자세나 속도를 전혀 모르는 상태에서도 계산할 수 있어, 이를 하나의 factor로 묶어 두면 수많은 IMU 원측정을 일일이 추정기에 넣지 않고도 관성 정보를 고스란히 살릴 수 있다. "pre-integration"이라는 이름도 바로 이 논문에서 태어났다.

하지만 초기 구현에는 두 가지 걸림돌이 남아 있었다. 첫째, 회전을 Euler angle로 표현한 탓에 gimbal lock 같은 특이점이 생겼고 회전 공간이 지닌 manifold 구조도 제대로 반영하지 못했다. 둘째는 더 근본적인 문제인 bias였다. 최적화를 한 바퀴 돌 때마다 bias 추정값이 조금씩 바뀌는데, 그때마다 keyframe 사이의 IMU 측정 수백 개를 처음부터 다시 적분하면 실시간성이 무너진다. Lupton과 Sukkarieh는 이를 피하려고 bias 변화량에 맞춰 증분을 1차 근사로 고치는 방법을 함께 제안했고, 6년 뒤 Forster 팀이 바로 이 보정을 SO(3) 위에서 말끔하게 재정식화했다.

---

## 9.3 Forster 등의 on-manifold preintegration (2015 / 2017)

2015년 RSS에서 취리히 대학교(UZH) 박사과정생 Christian Forster는 지도교수 Davide Scaramuzza, 그리고 Georgia Tech의 Luca Carlone(이후 MIT)과 GTSAM 개발자 Frank Dellaert와 함께 [IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation](https://www.roboticsproceedings.org/rss11/p06.pdf)을 발표했다. 2년 뒤인 2017년에는 IEEE TRO에 확장판 [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://doi.org/10.1109/TRO.2016.2597321)가 실렸다. 저자들은 GTSAM의 iSAM2 위에 VIO를 구현한 뒤, VI-Sensor로 기록한 430 m 실내 궤적과 Google Tango 기기를 들고 사무실 건물 둘레를 걸은 실외 궤적으로 성능을 검증했다.

Forster 등의 핵심 해법은 세 단계로 요약된다. 먼저 $\Delta\mathbf{R}_{ij}$를 SO(3) 위의 상대 회전으로 엄밀하게 정의했다. 이어 $\Delta\mathbf{v}_{ij}$와 $\Delta\mathbf{p}_{ij}$를 *중력과 초기 상태에 완전히 독립*이 되도록 재정의했다. 이렇게 만든 양은 실제 물리적 속도 변화량이나 위치 변화량과 그대로 일치하지는 않는다. 하지만 이 preintegration 값을 고정해 두면, 추정 상태가 바뀔 때마다 적분을 새로 할 필요 없이 IMU factor를 몇 번이고 다시 평가할 수 있다(양 끝의 pose·속도·중력·bias 보정은 residual 식 안에서 결합된다). 그다음 잡음 항을 지수 사상 바깥으로 끌어내는 right Jacobian 기법을 써서 공분산 $\boldsymbol{\Sigma}_{ij}$를 해석적으로 전파했다.

마지막 퍼즐은 **bias 1차 Jacobian 보정**이었다. keyframe 사이를 처음 적분할 때 bias에 대한 편미분을 미리 계산해 두면, 최적화 도중 bias 추정값이 조금 변하더라도 적분을 되풀이하지 않고 1차 근사로 증분을 즉시 고쳐 factor를 갱신할 수 있다. 이 계산 절감 덕분에 IMU factor는 비로소 대규모 실시간 그래프 최적화에 자연스럽게 녹아들었다.

저자들은 이 preintegration 구현을 GTSAM 4.0에 오픈소스로 공개했다. 이후 수많은 VIO와 LIO 시스템이 수식을 밑바닥부터 다시 짜는 대신 GTSAM의 `ImuFactor`를 `#include`해 쓰기 시작했다.

> 🔗 **공통 수학.** Forster의 on-manifold preintegration은 SO(3)의 지수 사상과 right Jacobian을 축으로 회전의 불확실성을 다룬다. [Barfoot 2017. *State Estimation for Robotics*](https://doi.org/10.1017/9781316671528)는 바로 이 Lie group 계산을 로봇 상태 추정 전반의 공통 언어로 엮어 낸 책이다. 출간 시점은 Forster의 2015년 논문보다 2년 늦지만, 두 작업은 같은 수학적 토대 위에 서 있다.

> 🔗 **차용.** bias 1차 보정의 원형은 [Lupton & Sukkarieh 2012](https://doi.org/10.1109/TRO.2011.2170332)에서 나왔다. Forster 등도 자신들의 보정이 Lupton과 Sukkarieh의 방식과 닮았으되 SO(3) 위에서 직접 작동한다는 점을 논문에서 명시했다("similar to the bias correction in [Lupton & Sukkarieh] but operates directly on SO(3)").

---

## 9.4 실제 VIO 시스템

2007년부터 2022년 사이에 등장한 VIO 시스템은 크게 필터 기반, 최적화 기반, 그리고 direct 방식이라는 세 줄기로 나뉜다. 흥미롭게도 필터 기반 VIO는 Forster의 논문보다 훨씬 먼저 출발했다. 2007년 University of Minnesota의 Anastasios Mourikis와 Stergios Roumeliotis가 ICRA에 발표한 [MSCKF(Multi-State Constraint Kalman Filter)](https://doi.org/10.1109/ROBOT.2007.364024)는 stochastic cloning(과거 상태를 복제해 필터 안에 남겨 두는 기법)으로 이전 카메라 자세 여러 개를 상태 벡터에 유지했다. 반면 관측된 3D 점은 상태에 넣지 않고, 선형화한 관측식을 점 Jacobian의 left null space로 투영해 점 위치 오차가 상태 추정에 스며들지 않게 소거했다. preintegration 없이도 EKF만으로 실시간 VIO를 구현한 이 선구적 설계는 2021년 화성 하늘을 난 NASA JPL의 Ingenuity 헬리콥터에도 이어졌다. Ingenuity의 항법 필터 MAVeN 역시 base image 시점의 위치와 자세를 clone state로 유지하는 EKF 구조였다. 또 2020년에는 University of Delaware의 Guoquan Huang 그룹이 MSCKF 기반 오픈소스 시스템 [OpenVINS](https://doi.org/10.1109/ICRA40945.2020.9196524)를 공개해 연구자들이 널리 쓰게 됐다.

최적화 기반 VIO의 간판은 HKUST의 Tong Qin, Peiliang Li, Shaojie Shen이 2018년 TRO에 발표한 [VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729)였다. VINS-Mono는 preintegration한 IMU 측정을 sliding window 방식의 tightly-coupled BA에 factor로 넣고, 초기화 단계에서 scale과 중력 방향을 체계적으로 분리 추정하는 절차까지 완비했다. 코드까지 깔끔하게 공개되자 2019–2022년 학회 논문들이 앞다투어 VINS-Mono를 비교 기준으로 삼았다.

한편 direct 방식에서는 TUM Cremers 그룹이 VI-DSO(ICRA 2018)와 [DM-VIO(RA-L 2022)](https://doi.org/10.1109/LRA.2021.3140129)로 계보를 이었다. VI-DSO는 DSO의 photometric BA에 inertial factor를 결합했고, DM-VIO는 여기에 *delayed marginalization*을 보탰다. IMU 초기화가 채 끝나기 전에 옛 keyframe을 marginalize하면 틀린 scale이 prior에 영구히 박혀 버린다. DM-VIO는 주 그래프 옆에 marginalization을 일정 keyframe만큼 늦춰 처리하는 보조 factor graph를 하나 더 두어 이 문제를 피했다. IMU 초기화가 안정된 뒤에야 보조 그래프에 IMU factor를 넣어 다시 marginalize하고, 그렇게 정제된 새 prior를 주 시스템에 넘기는 방식이다.

---

## 9.5 Observability

VIO가 아무리 정교해져도 관측 정보 자체가 부족하면 해가 하나로 모이지 않는다. 2014년 Minnesota의 Hesch, Kottas, Bowman, Roumeliotis는 TRO 논문에서 외부 기준이 없는 visual-inertial 시스템에는 원리적으로 관측할 수 없는 방향이 **넷** 존재한다는 사실을 증명했다. 전역 위치 3자유도와 중력축을 중심으로 한 회전(yaw) 1자유도다. 세상 전체를 이 네 방향으로 평행이동하거나 회전시켜도 카메라 영상과 IMU 측정값은 털끝만큼도 달라지지 않는다. 전역 위치를 잡으려면 GPS가, yaw를 잡으려면 지자기 센서나 외부 랜드마크가 추가로 필요하다.

이와 달리 *roll과 pitch는 언제나 관측된다*. 가속도계가 늘 중력 벡터의 방향을 재어 수평 기준을 잡아 주기 때문이다. 단안 카메라의 scale 역시 IMU를 붙이면 관측 가능해지지만, 여기에는 운동이 충분해야 한다는 단서가 따른다.

실무에서 더 까다로운 대목은 바로 **degenerate motion**(관측 정보가 빈약해지는 운동)이다. 전역 yaw가 원천적으로 비관측인 상태에서 시차나 가속 변화까지 부족해지면 깊이·scale·bias가 서로 얽혀 구분되지 않는다. 드론이 제자리에 떠서 호버링하거나 자동차가 일정한 속도로 반듯하게 직진할 때가 대표적인 함정이다. 이륙·제동·선회 같은 가감속 기동이 정보를 불어넣어 주기는 하지만, 특정 동작 하나만으로 모든 상황에서 scale 추정이 보장되지는 않는다.

> 📜 **예언 vs 실제.** Forster 등(2017)은 TRO 논문 결론(§IX)에서 장황한 미래 과제를 늘어놓는 대신 preintegration과 structureless vision factor 코드를 GTSAM 4.0에 공개한다고 적는 것으로 갈음했다. 이후 실제 현장에서 카메라와 IMU 사이의 시간 offset은 Kalibr, VINS-Mono, OpenVINS가 앞다투어 추정 변수에 포함시키면서 표준적인 보정 항목으로 굳어졌고, 센서 시각이 아예 어긋나는 비동기 문제는 Le Gentil의 GP 연속시간 preintegration이라는 새로운 해법으로 이어졌다.

---

## 9.6 연속시간 preintegration

2021년 RSS에서 UTS(University of Technology Sydney)의 Cédric Le Gentil과 Teresa Vidal-Calleja는 [Continuous Integration over SO(3) for IMU Preintegration](https://roboticsproceedings.org/rss17/p078.pdf)을 발표했다. UTS가 Lupton의 연구실 ACFR과 같은 시드니에 자리 잡고 있다는 점도 묘한 우연이다.

Forster의 preintegration은 본질적으로 이산적이다. 인접한 IMU 표본 사이에서는 값이 일정하다고 보고 구간 적분을 쌓아 간다. 그런데 측정 시각이 IMU와 엇갈리는 LiDAR나 event camera 같은 센서가 들어오면 IMU 표본과 표본 사이의 임의 시각 $\tau$에서의 상태가 필요해진다. 시각이 어긋난다고 구간 상수 근사가 당장 무너지는 것은 아니지만, 시간 보간의 정밀도가 시스템 전체의 성능을 좌우한다. Le Gentil은 각속도를 **Gaussian Process**(GP)로 모델링해 연속 함수로 다루면서, 임의의 시각 $\tau$에서도 상태를 즉시 뽑아낼 수 있게 했다. 이를 바탕으로 비동기 센서 측정을 매끄럽게 융합할 뿐 아니라 IMU bias와 센서 간 시간 차이까지 함께 보정하는 방법을 제시했다.

이 시도는 곧 B-spline, STEAM, GPMP로 이어지는 더 넓은 연속시간 궤적 추정 연구와 맞닿는다(10장).

---

## 9.7 가져온 것

> 🔗 **차용.** IMU factor를 factor graph 안에서 평가하고 최적화하는 골격은 [Dellaert의 GTSAM](https://gtsam.org/) 설계를 그대로 따른다. `ImuFactor`가 GTSAM의 `NoiseModelFactor` 인터페이스를 구현하기 때문에, 개발자는 visual reprojection factor와 inertial factor를 하나의 `Values` 객체 안에 나란히 넣고 함께 최적화할 수 있다.

> 🔗 **차용.** bias를 random walk로 모델링하는 관행은 오래된 Kalman filter 상태 전파에서 물려받았다. 항법 공학에서는 Lupton 이전부터 bias를 상태 변수에 넣고 작은 process noise를 주는 방식을 써 왔고, preintegration에서는 이 관습이 bias random walk factor라는 그래프 노드로 옷을 갈아입었다.

---

## 아직 열린 문제

**관측 가능성을 실시간으로 감지하기.** 4차원 null space와 degenerate motion의 수학은 이미 정립됐다. Huang, Mourikis, Roumeliotis가 제안한 FEJ(First-Estimate Jacobian)처럼 선형화 과정에서 null space를 훼손하지 않는 기법도 자리 잡았다. 하지만 실제 운용 중에 지금 막 관측 정보가 사라지는 구간에 진입했는지, 언제 빠져나왔는지를 실시간으로 진단해 제어 루프에 경고해 주는 일은 여전히 미완으로 남아 있다.

**Preintegration과 continuous-time의 통합.** Forster의 이산 증분과 Le Gentil의 GP 연속 표현은 본질적으로 같은 관성 적분 문제를 서로 다른 수학으로 푼다. LiDAR, event camera, 일반 카메라가 뒤섞인 이종 센서 시스템에서 어떤 표현을 기본 뼈대로 삼아야 가장 효율적인지는 아직 합의되지 않았다(→ 10장).

**학습 기반 IMU bias 모델.** bias가 순한 random walk를 따른다는 가정은 고가의 navigation-grade IMU에서는 꽤 잘 맞지만, 값싼 consumer MEMS에서는 온도에 따른 이력 현상(hysteresis)과 전원 과도 현상 때문에 종종 어긋난다. TLIO와 RoNIN 같은 연구는 신경망으로 IMU만 쓰는 odometry를 학습했고, 최근에는 conditional diffusion으로 bias 분포를 직접 모델링하려는 시도까지 등장했다. 이렇게 학습된 복잡한 오차 모델을 Forster 방식의 factor graph에 어떻게 녹여 낼지는 아직 풀리지 않은 문제다.

---

Le Gentil의 연속시간 preintegration에서 보듯, 문제는 관성 센서에 그치지 않는다. 카메라의 셔터가 행마다 차례로 열리거나 LiDAR가 회전하며 점을 찍을 때, 이산적인 keyframe이라는 전제 자체가 흔들리기 시작한다. 다음 장에서는 아예 궤적을 시간의 연속 함수로 세우는 연속시간 궤적 추정(→ 10장)을 살펴본다.