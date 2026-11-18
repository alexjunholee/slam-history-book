# 10장 연속시간 궤적 추정

preintegration은 수백 개의 IMU 측정을 이산적인 두 keyframe 사이의 상대 factor 하나로 압축해 냈다. 하지만 이 영리한 압축에는 keyframe이라는 단위가 또렷하게 존재한다는 전제가 깔려 있다. 양 끝의 시각이 명확히 찍히는 global shutter 카메라라면 문제될 것이 없다. 그러나 현실의 모든 센서가 그렇게 깔끔하게 한 순간을 포착하지는 않는다.

소비자용 rolling shutter 카메라는 이미지 한 장 안에서도 위쪽 행과 아래쪽 행이 서로 다른 시각에 찍힌다. 회전하는 spinning LiDAR가 한 바퀴를 도는 사이에도 차량은 계속 앞으로 나아간다. 게다가 IMU, 카메라, LiDAR는 제각기 다른 주기로 데이터를 쏟아 낸다. 2012년 ICRA에서 [Paul Furgale, Timothy Barfoot, Gabe Sibley](https://doi.org/10.1109/ICRA.2012.6225005)는 아예 로봇의 자세를 시간 $t$의 연속 함수로 세워 이 모든 비동기 측정을 하나의 확률적 batch 추정 문제로 엮었다. "Continuous-time batch estimation using temporal basis functions"라는 제목의 이 논문에서 세 사람은 유한한 개수의 시간 기저 함수(temporal basis function) 계수를 상태 변수로 추정했다.

이 발상은 이후 rolling shutter 카메라, spinning LiDAR, event camera처럼 한 프레임이나 한 scan 안에서도 측정 시각이 계속 변하는 센서들을 다루는 표준 도구로 번져 나갔다. 이산 keyframe을 중심에 둔 visual-inertial preintegration(→ 9장)과 나란히, 시간을 연속으로 모델링하는 또 하나의 축이 세워진 것이다.

---

## 10.1 Discrete-time의 한계

preintegration이 해결한 것은 IMU가 카메라보다 훨씬 자주 측정값을 낸다는 주기 차이였다. 하지만 실제 다중 센서 SLAM에서 시간과 얽힌 골칫거리는 거기서 그치지 않는다.

고전적인 이산 SLAM은 추정기가 정한 keyframe 시각에 맞추어 상태 노드를 세운다. 그러나 **Rolling shutter** 카메라는 한 프레임을 위에서 아래로 훑으며 읽어 들이기 때문에 카메라가 빠르게 회전하면 첫 행과 마지막 행이 서로 다른 자세에서 노광된다. DSO나 LSD-SLAM처럼 프레임당 자세를 하나만 두는 모델은 이 왜곡을 담아내지 못한다(→ 11장). **Spinning LiDAR의 motion distortion**도 같은 뿌리에서 나온다. 예를 들어 Velodyne HDL-64E가 10 Hz로 회전할 때 차량이 10 m/s로 달리면, 한 scan을 채우는 100 ms 동안 차가 1 m를 이동해 점마다 찍힌 자세가 달라진다. LOAM은 odometry 루프 안에서 이 왜곡을 간접적으로 펴 주었지만, 더 일반적인 해법은 각 점의 측정 시각에 해당하는 자세를 직접 조회할 수 있는 연속 궤적이다.

아예 프레임이라는 개념이 없는 센서도 있다. **Event camera**(DVS)는 픽셀 밝기가 변할 때마다 μs 단위로 비동기 이벤트를 뿜어 낸다. Mueggler, Gallego, Scaramuzza가 [RSS 2015 논문](https://www.roboticsproceedings.org/rss11/p36.pdf)에서 궤적을 SE(3) 위의 cubic B-spline으로 세워 event 기반 궤적 추정을 정식화한 것도 프레임 단위가 통하지 않았기 때문이다. 여기에 200 Hz IMU, 20 Hz 카메라, 10 Hz LiDAR처럼 **주기가 다른 여러 센서**를 함께 묶는 상황까지 겹치면 이산 그래프는 한계에 부딪힌다. 측정이 들어오는 시각마다 상태 노드를 세우면 그래프가 감당할 수 없이 커지기 때문이다.

이 모든 문제의 본질은 같다. 측정 시각 $t_i$를 추정기가 아니라 센서가 결정한다는 점이다. 연속시간 표현은 측정 시각, 상태 추정 시각, 궤적 조회 시각을 서로 분리해 이 얽힘을 풀어낸다.

---

## 10.2 Parametric spline

2012년 Furgale·Barfoot·Sibley가 연속 궤적을 세우기 위해 꺼내 든 도구는 B-spline이었다. 궤적을 기저 함수의 합 $\mathbf{p}(t) = \sum_k \Psi_k(t)\,\mathbf{c}_k$로 표현하고, 제어점 계수 $\mathbf{c}_k$를 최적화 변수로 잡는 방식이다. B-spline의 가장 큰 무기는 local support(국소 지지) 성질이다. 임의의 시각 $t$에서 0이 아닌 값을 갖는 기저 함수가 보통 4개 안팎으로 제한되므로, 어느 시각 $t_i$의 자세든 상수 시간 비용으로 조회할 수 있을 뿐 아니라 factor graph의 희소성(sparsity)도 고스란히 지켜진다.

다만 제어점 간격을 사람이 경험적으로 골라야 한다는 부담이 있었다. 간격이 너무 좁으면 잡음에 과적합되고, 너무 넓으면 급격한 기동을 따라가지 못한다. 게다가 유클리드 공간의 선형 B-spline을 그대로 적용하면 보간된 자세가 SE(3) 다양체를 벗어나 버린다.

이 걸림돌은 이듬해 BMVC 2013에서 해결됐다. [Steven Lovegrove, Alonso Patron-Perez, Gabe Sibley](https://doi.org/10.5244/C.27.93)는 "Spline Fusion"이라는 이름으로 cumulative B-spline을 제안했다. 기저 함수를 누적 곱 형태인 $T(t) = \prod_k \exp\bigl(\tilde\Psi_k(t) \log(T_k T_{k-1}^{-1})\bigr) \cdot T_0$로 다시 쓰면 각 항이 지수 사상을 통해 Lie group 안에 안전하게 머문다. 이 우아한 공식은 곧 rolling shutter, event camera, VIO 연구의 단골 도구가 됐다. Basalt와 Mueggler의 event 궤적 추정뿐 아니라 [Kerl·Stückler·Cremers의 ICCV 2015 dense rolling shutter RGB-D 추적](https://doi.org/10.1109/ICCV.2015.261)도 모두 이 cumulative B-spline 위에 세워졌다.

parametric spline은 계산이 가볍고 구현이 단순해 실시간 VIO와 event 시스템에서 오래 사랑받았다. 다만 별도의 motion prior를 얹지 않는 한 관측이 드문 구간의 매끄러움은 기저 함수와 제어점 배치에 맡겨야 한다(물론 계수나 미분에 prior를 추가할 수는 있다). 바로 이 지점에서 궤적의 사전 분포 자체를 출발점으로 삼는 Gaussian Process(GP) 방식과 갈라진다(→ §10.3).

> 🔗 **차용.** B-spline의 수학적 기초는 [de Boor (1978) *A Practical Guide to Splines*](https://link.springer.com/book/10.1007/978-1-4612-6333-3)에 집대성되어 있다. Furgale 등은 이 고전적인 스플라인 보간을 로봇 자세 궤적으로 옮겨 와 제어점 계수를 상태 추정 변수로 삼았다.

---

## 10.3 SDE 기반 GP와 STEAM

2014년 토론토 대학의 Barfoot 그룹은 spline과는 결이 다른 비모수적 해법을 들고 나왔다. [Barfoot, Tong, Särkkä의 RSS 2014 논문 "Batch Continuous-Time Trajectory Estimation as Exactly Sparse Gaussian Process Regression"](https://www.roboticsproceedings.org/rss10/p01.pdf)은 궤적 전체를 하나의 Gaussian Process(가우시안 과정)로 모델링했다. 커널 $\mathcal{K}(t, t')$로 궤적의 사전 분포를 깔아 두고, 센서 관측이 들어오면 조건부 가우시안 분포로 사후 궤적(posterior)을 구하는 접근이다.

원래 GP 회귀에는 치명적인 약점이 있다. 관측 수 $N$이 늘어나면 $N \times N$ 커널 행렬 $K$의 역행렬을 구하는 데 $O(N^3)$ 비용이 들어 실시간 SLAM에 쓸 수 없다는 점이다. Barfoot, Tong, Särkkä는 특정 조건을 만족하는 커널 부류가 이 병목을 완벽히 우회한다는 사실을 짚어 냈다. 궤적이 선형 시불변 확률 미분 방정식(linear time-invariant SDE) $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + L\mathbf{w}(t)$의 해로 정의되면, 정밀도 행렬 $K^{-1}$이 정확히 block-tridiagonal 형태를 띤다. 이를 factor graph 언어로 번역하면 인접한 상태 노드 사이에만 binary factor가 걸리고 멀리 떨어진 노드 사이에는 연결이 없는 희소한 사슬 구조가 된다.

이 이론은 곧 **STEAM**(Simultaneous Trajectory Estimation and Mapping)이라는 구체적인 프레임워크로 열매를 맺었다. 2015년 IROS에서 [Sean Anderson과 Barfoot이 발표한 "Full STEAM Ahead"](https://doi.org/10.1109/IROS.2015.7353368)는 SE(3) 위에서 constant-velocity prior를 세워 STEAM을 정식화했다. 상태는 자세와 body-centric 속도로 이루어지며, 속도는 백색 잡음의 적분만큼 흔들리고 자세는 그 속도를 따라 흘러간다. 같은 해 Anderson, Barfoot, Tong, Särkkä는 이 틀을 비선형 운동 모델로까지 넓힌 저널 논문 ["Batch nonlinear continuous-time trajectory estimation as exactly sparse Gaussian process regression"](https://doi.org/10.1007/s10514-015-9455-y)을 *Autonomous Robots*에 실었다.

STEAM의 또 다른 장점은 GP 보간이다. 몇 개의 제어점(control pose)만 상태 노드로 두고, 그 사이의 임의 시각 자세는 GP posterior mean으로 즉시 보간해 꺼내 쓸 수 있다. 예를 들어 spinning LiDAR 한 scan에 점 10,000개가 각기 다른 타임스탬프를 달고 들어와도 제어점은 scan당 하나만 세우면 된다(물론 10,000개 점 각각의 residual을 계산해 더하는 일은 여전히 필요하다).

[STEAM 오픈소스](https://github.com/utiasASRL/steam) 저장소는 2015년에 문을 열었고 가장 많은 커밋을 남긴 주역은 Sean Anderson이었다. 한편 Dellaert 그룹의 GTSAM에도 2026년에 GP motion prior가 공식 통합됐다.

> 🔗 **차용.** SDE와 GP의 등가 관계는 [Särkkä 2013 *Bayesian Filtering and Smoothing*](https://users.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf)에 정리되어 있었다. Rasmussen–Williams의 표준 GP 교과서가 닫힌 꼴의 커널을 주로 다루는 것과 달리, 실시간 SLAM에는 역행렬의 희소성이 절실했다. Barfoot 그룹은 Särkkä의 SDE 표현을 빌려와 GP motion prior를 곧장 희소 factor graph로 번역하는 다리를 놓았다.

---

## 10.4 Lie group 위의 continuous-time

parametric spline을 쓰든 nonparametric GP를 쓰든, 실제 로봇 궤적을 다루려면 결국 SE(3) 다양체 위에서 계산해야 한다. 유클리드 공간의 보간 공식은 회전 군의 제약을 지키지 못하기 때문에, 대개 접공간(tangent space)에서 선형 연산을 수행한 뒤 지수 사상으로 SE(3)에 되돌리는 정교한 장치가 필요하다.

B-spline 진영에서는 뮌헨 공대(TUM)의 Sommer, Usenko, Schubert, Demmel, Cremers가 ["Efficient Derivative Computation for Cumulative B-Splines on Lie Groups"(CVPR 2020)](https://arxiv.org/abs/1911.08860)을 통해 계산 병목을 뚫었다. Lie group cumulative B-spline의 시간 미분을 점화식으로 재유도해, spline 차수 $k$에 대해 $O(k^2)$번 들던 행렬 연산을 $O(k)$번으로 떨어뜨린 것이다. SO(3) spline에서는 제어점에 대한 미분 Jacobian까지 해석적으로 구해 내어 camera-IMU calibration 실험에서 눈에 띄는 속도 향상을 입증했다.

GP 진영에서는 Anderson과 Barfoot이 "local variable" 전략으로 해답을 내놓았다. 각 제어 자세 $T_k$ 근방에서 국소 섭동 $\xi_k(t) = \log(T(t)\,T_k^{-1})$를 정의하고, SE(3) 전체가 아닌 이 국소 접공간 위에서 유클리드 GP를 돌리는 방법이다. 인접한 두 제어점 구간을 건너갈 때는 adjoint 행렬로 좌표를 이어 주는데, 이 다양체 변환의 수학적 골격은 앞 장에서 본 on-manifold preintegration과 정확히 맞닿아 있다(→ 9장).

결국 spline과 GP의 차이는 motion prior를 다루는 태도에서 가장 선명하게 드러난다. spline은 기저 함수와 제어점 간격, 혹은 별도로 얹은 정규화 항이 빈 구간의 형태를 정한다. 반면 GP는 SDE에서 직접 유도한 constant-velocity나 white-jerk 같은 물리적 사전 분포가 내장되어 있어 관측이 뜸한 구간을 확률적으로 메워 준다. 두 접근을 결합하려는 시도(Johnson et al. 2020)도 있었지만, 실제 현장에서는 구현의 단순함과 물리적 사전 분포의 엄밀함 사이에서 응용 목적에 맞게 한쪽을 택하는 편이다.

> 🔗 **차용.** [Anderson과 Barfoot의 IROS 2015 논문](https://doi.org/10.1109/IROS.2015.7353368)은 Lie group의 국소 변수에 GP를 얹는 실용적 해법을 정립했다. 두 제어점 사이 구간에서만 국소 GP를 평가하고 경계를 넘을 때 adjoint로 변환을 잇는 이 기법은 이후 수많은 연속시간 LiDAR·VIO 시스템의 표준 패턴이 됐다.

---

## 10.5 응용: LiDAR, VIO, event camera

초기 연속시간 궤적 추정은 주로 오프라인 센서 calibration에 머물렀다. 그러나 2022년 무렵에 이르러서는 실시간 주행과 고속 비전이라는 거친 현장으로 무대를 넓혔다.

**LiDAR motion distortion.** MINES ParisTech와 Kitware의 Pierre Dellenbach 등이 발표한 [CT-ICP](https://arxiv.org/abs/2109.12979)(arXiv 2021, ICRA 2022)는 복잡한 spline 대신 scan마다 시작 자세와 끝 자세 두 개만 두고 그 사이를 보간하는 영리한 타협을 택했다(회전은 slerp, 위치는 선형 보간). 앞 scan의 끝 자세와 다음 scan의 시작 자세를 굳이 딱 붙이지 않고 느슨한 근접 제약만 걸어 두는 이 간결한 연속시간 모델은 KITTI, NCLT, Newer College 등 7개 데이터셋에서 위력을 발휘했고, KITTI odometry 리더보드에서 코드 공개 방법 중 1위(평균 상대 이동 오차 0.59%)를 차지했다.

같은 해 토론토 그룹의 Keenan Burnett, Yuchen Wu 등은 ["Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?"](https://arxiv.org/abs/2203.10174)(RSS 2022)에서 radar 단독, LiDAR 단독, 그리고 radar-to-LiDAR 지도 정합을 나란히 비교하며 세 파이프라인의 odometry 모두에 Anderson–Barfoot의 GP motion prior 기반 연속시간 ICP를 심었다. 이어 Wu 등은 ["Picking up Speed"](https://doi.org/10.1109/LRA.2022.3226068)(RA-L 2023)에서 점마다 Doppler 속도까지 함께 뱉어 내는 Aeva FMCW LiDAR에 이 틀을 적용하고 [STEAM-ICP](https://github.com/utiasASRL/steam_icp) 코드를 공개했는데, 센서가 재는 Doppler 속도가 STEAM 상태 벡터의 속도 변수와 정확히 맞물리는 묘미를 보여 주었다.

**Rolling shutter VIO.** [Kerl 등의 rolling shutter RGB-D 추적](https://doi.org/10.1109/ICCV.2015.261)(ICCV 2015)이 B-spline 궤적으로 이미지 행별 노출 시각의 자세를 찾아낸 데 이어, [Ctrl-VIO](https://doi.org/10.1109/LRA.2022.3202349)(RA-L 2022)는 Basalt의 spline 모듈을 개조해 rolling shutter VIO를 구현했다. global shutter 가정이 깨지는 고속 기동에서도 rolling shutter 왜곡을 우회하지 않고 모델 내부에서 직접 풀어낸 사례다.

**Event camera.** 2010년대에 센서의 난해함 때문에 고전하던 event camera 연구(→ 22장) 역시 2020년대에 접어들어 연속시간 궤적을 발판 삼아 활로를 열었다. 비동기 이벤트의 μs 단위 타임스탬프를 B-spline이나 GP에 그대로 질의해 순간 자세를 얻고, event와 영상 사이의 일관성으로 residual을 세우는 event SLAM이 잇따라 등장했다.

> 🔗 **차용.** CT-ICP는 [Besl·McKay 1992 ICP](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)에서 물려받은 point-to-plane 정합 목적함수에 scan 내부의 연속 시간 보간을 얹어 고속 주행의 왜곡을 흡수했다.

---

## 예측과 실제

> 📜 **예언 vs 실제.** Furgale·Barfoot·Sibley(2012)는 고속 IMU와 sweeping laser가 이산 상태 공간을 부풀리는 문제를 겨냥하며 rolling-shutter 보간 사례를 함께 언급했지만, 정작 논문에서 직접 검증한 것은 camera–IMU calibration이었다. 그 뒤 Barfoot·Tong·Särkkä(2014)가 SDE 기반 sparse GP로 이론을 가다듬고, rolling-shutter VIO와 event SLAM이 2012년 논문의 Future Work와 별개의 줄기에서 연속시간 궤적을 실전 도구로 끌어올렸다. 그럼에도 주류 visual SLAM 시장은 여전히 이산 keyframe 기반인 ORB-SLAM이 차지했고, 연속시간 궤적은 주로 LiDAR·event·rolling shutter처럼 시간 분해능이 핵심인 영역에서 힘을 발휘했다.

---

## 아직 열린 문제

**학습 기반 연속시간 prior.** SDE에서 끌어온 motion prior는 constant-velocity나 white-jerk 같은 매끈한 물리 가정에 기댄다. 하지만 실제 도심 주행이나 보행, 드론 비행 궤적은 이런 이상적인 가정을 수시로 어긴다. 2023–2024년에는 neural SDE나 neural ODE로 실제 데이터에서 motion prior를 학습해 연속시간 factor graph에 넣으려는 시도가 나타났지만, 실시간 추정을 지탱하는 희소 블록 구조를 잃지 않으면서 학습된 prior를 결합하는 일은 아직 숙제로 남아 있다.

**VIO와 continuous-time의 통합.** keyframe 기반 VIO에서는 preintegration이 사실상의 표준이다. 연속시간 궤적이 preintegration을 완전히 대체할 수 있을지, 아니면 둘의 장점을 섞는 편이 나을지는 아직 결론이 나지 않았다. Le Gentil, Vidal-Calleja, Huang의 [GP 기반 preintegration](https://doi.org/10.1109/LRA.2020.2970940)(RA-L 2020)이 두 진영을 이으려 했으나, 실제 현장에서 가장 널리 쓰이는 ORB-SLAM3나 VINS-Fusion은 여전히 이산 preintegration에 머물러 있다.

**임베디드 장치용 온라인 sliding window.** STEAM이나 B-spline 기반 추정기는 시간이 흘러 제어점이 누적될수록 계산 부담이 커진다. 과거 제어점을 marginalization으로 덜어 내면서도 연속시간 사후 분포의 일관성을 해치지 않고 임베디드 보드 위에서 가볍게 돌리는 기법은 아직 다듬어야 할 부분이 많다.

---

Le Gentil 등의 GP 기반 preintegration이 보여 주듯, 연속시간 표현과 이산 preintegration은 배타적인 도구가 아니라 서로 맞물릴 수 있다. 지금까지 2부에서는 특징점 추적, 관성 사전적분, 연속시간 궤적, 그리고 certifiable backend에 이르기까지 SLAM의 뼈대를 이루는 기하적 추정 엔진을 살펴보았다. 이제 3부에서는 시선을 돌려, 아예 특징점을 뽑아 추상화하는 과정을 건너뛰고 카메라가 찍은 픽셀 밝기와 깊이 측정 자체를 지도로 빚어낸 직접법(direct method)과 조밀 재구성(dense reconstruction)의 흐름(→ 11장)으로 넘어간다.