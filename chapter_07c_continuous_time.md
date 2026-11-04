# Ch.7c — Continuous-Time Trajectory: 궤적을 시간의 연속 함수로

Ch.7b의 preintegration은 IMU 측정을 이산 keyframe 사이의 상대 factor 하나로 압축했다. 이 압축은 keyframe이라는 단위가 있어야 성립한다. 두 keyframe 사이에 들어온 관성 측정 100개를 factor 하나로 묶으려면 factor 양 끝의 시각이 분명해야 한다. 셔터가 이미지 전체에 한 번에 열리고 닫히는 카메라라면 이 가정에 문제가 없다. 그렇지 않은 센서도 있다.

rolling shutter 카메라로 찍은 이미지 한 장은 행마다 다른 시각의 자세로 투영된다. spinning LiDAR가 한 바퀴를 도는 사이에도 차량은 움직인다. IMU와 카메라도 서로 다른 주기로 측정값을 낸다. 2012년 ICRA에서 [Paul Furgale, Timothy Barfoot, Gabe Sibley](https://doi.org/10.1109/ICRA.2012.6225005)는 자세를 시간 t의 함수로 두고 이 센서들의 측정을 하나의 확률적 batch 추정 문제로 묶었다. 논문 제목은 "Continuous-time batch estimation using temporal basis functions"다. 세 사람은 유한한 개수의 temporal basis function(시간 기저 함수) 계수를 상태로 추정했다.

그 뒤 continuous-time trajectory는 rolling shutter 카메라, spinning LiDAR, event camera를 다루는 데 널리 쓰였다. 이 센서들은 한 프레임이나 한 scan 안에서도 측정 시각이 다르다. 앞 장의 visual-inertial 방법은 대부분 이산 keyframe을 바탕으로 했다.

---

## 7c.1 Discrete-time의 한계

Ch.7b의 preintegration은 IMU가 카메라보다 자주 측정한다는 문제를 다뤘다. 시간을 다루는 데에는 문제가 더 있다.

**Rolling shutter.** 소비자용 CMOS 카메라는 한 프레임을 위에서 아래로 일정 시간에 걸쳐 읽는다. 카메라가 빠르게 움직이면 첫 행과 마지막 행은 서로 다른 자세에서 찍힌다. Ch.8의 DSO와 LSD-SLAM은 프레임마다 자세가 하나라고 가정하므로 이 왜곡을 모델에 넣지 않았다. continuous-time 궤적을 쓰면 행마다 실제 노출 시각의 자세를 구할 수 있다.

**Spinning LiDAR의 motion distortion.** Ch.17에서 다루는 Velodyne HDL-64E는 기본 설정에서 10 Hz로 한 바퀴를 돈다. 차량이 10 m/s로 달리면 한 scan을 얻는 100 ms 동안 1 m를 이동한다. 그래서 한 scan의 점들은 저마다 다른 자세에서 찍힌다. LOAM은 이 왜곡을 odometry 루프 안에서 간접적으로 보정했다. 더 일반적인 방법은 점마다 그 점이 찍힌 순간의 자세를 구할 수 있는 궤적 표현이다.

**Event camera.** Ch.18에서 다루는 DVS는 픽셀마다 μs 단위로 비동기 이벤트를 낸다. 이벤트에는 프레임이 없다. Mueggler, Gallego, Scaramuzza는 [RSS 2015 논문](https://www.roboticsproceedings.org/rss11/p36.pdf)에서 궤적을 SE(3) 위의 cubic B-spline으로 표현해 event 기반 궤적 추정을 정식화했다.

**주기가 다른 여러 센서.** 고속 IMU를 측정 주기가 다른 여러 센서와 함께 쓸 때도 문제가 생긴다. 한 시스템에 200 Hz IMU, 20 Hz 카메라, 10 Hz LiDAR가 들어온다고 하자. 측정 시각마다 이산 상태 노드를 두면 상태 수가 측정 수만큼 늘고, factor graph가 지나치게 커진다.

이 문제들에서 측정 시각 $t_i$는 센서가 정한다. 관측은 아무 때나 들어오고, 추정기는 그 시각의 자세를 알아야 한다. continuous-time 표현은 측정 시각과 추정 시각, 조회 시각을 서로 떼어 내 이 문제를 다룬다.

---

## 7c.2 Parametric spline

Furgale·Barfoot·Sibley가 2012년에 고른 도구는 B-spline이었다. 궤적을 basis function의 합 $\mathbf{p}(t) = \sum_k \Psi_k(t)\,\mathbf{c}_k$로 쓰고, 계수 $\mathbf{c}_k$를 최적화 변수로 둔다. B-spline은 local support(국소 지지) 성질이 있다. 한 시점 t에서 0이 아닌 basis는 몇 개(보통 4개)뿐이다. 그래서 아무 시각 $t_i$의 자세든 일정한 비용으로 구할 수 있고, factor graph의 sparsity도 유지된다.

> 🔗 **차용.** B-spline의 수학은 [de Boor (1978) *A Practical Guide to Splines*](https://link.springer.com/book/10.1007/978-1-4612-6333-3)에 정리되어 있다. Furgale은 이를 로봇 자세 궤적에 쓰고, 계수를 최적화 변수로 두었다.

계수 간격을 좁게 잡으면 과적합이 생긴다. 넓게 잡으면 빠른 움직임을 놓친다. 간격은 경험으로 골랐다. 또 선형 B-spline을 SE(3)에 그대로 적용하면 보간 결과가 SE(3)를 벗어난다.

2013년 BMVC에서 [Steven Lovegrove, Alonso Patron-Perez, Gabe Sibley](https://doi.org/10.5244/C.27.93)는 "Spline Fusion"이라는 이름으로 cumulative B-spline을 제안했다. basis를 누적 곱 꼴 $T(t) = \prod_k \exp\bigl(\tilde\Psi_k(t) \log(T_k T_{k-1}^{-1})\bigr) \cdot T_0$로 다시 쓰면 각 인자가 Lie group 안에 머문다. 이 꼴은 그 뒤 rolling shutter, event camera, VIO 논문에서 널리 쓰였다. Basalt와 Mueggler의 event 궤적 추정, [Kerl·Stückler·Cremers의 ICCV 2015 dense rolling shutter RGB-D 추적](https://doi.org/10.1109/ICCV.2015.261)이 cumulative B-spline을 사용했다.

parametric spline은 계산이 가볍고 코드가 간단해서 실시간 VIO와 event 시스템에서 꾸준히 쓰였다. motion prior를 따로 두지 않으면, 관측이 드문 구간이 얼마나 매끄러운지는 basis와 제어점이 정한다. 계수나 미분에 prior를 더할 수도 있다. 다음 절의 GP 방식은 궤적의 사전 분포에서 출발한다.

---

## 7c.3 SDE 기반 GP와 STEAM

2014년 Toronto의 Barfoot 그룹은 다른 방식을 내놓았다. [Barfoot, Tong, Särkkä의 RSS 2014 논문 "Batch Continuous-Time Trajectory Estimation as Exactly Sparse Gaussian Process Regression"](https://www.roboticsproceedings.org/rss10/p01.pdf)은 궤적을 Gaussian process(가우시안 과정)로 둔다. 궤적의 사전 분포는 kernel $\mathcal{K}(t, t')$로 주어진다. 관측이 들어오면 posterior를 조건부 Gaussian으로 구한다.

GP를 그대로 쓰면 문제가 있다. 관측 수 $N$이 크면 kernel matrix $K$의 역행렬을 구하는 비용이 $O(N^3)$이다. Barfoot, Tong, Särkkä는 이 비용을 피할 수 있는 kernel 부류를 보였다. 궤적이 linear time-invariant stochastic differential equation(선형 시불변 확률 미분 방정식) $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + L\mathbf{w}(t)$의 해로 정의된다고 하자. 그러면 kernel $K$의 역행렬 $K^{-1}$은 block-tridiagonal이 된다. factor graph로 보면 연속한 상태 노드 사이에만 binary factor가 있고, 멀리 떨어진 노드 사이에는 factor가 없다.

> 🔗 **차용.** SDE와 GP의 관계는 [Särkkä 2013 *Bayesian Filtering and Smoothing*](https://users.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf)에 정리되어 있다. Barfoot 그룹은 이 관계를 SLAM에 가져와, SDE에서 유도한 GP motion prior를 factor graph로 표현했다. Rasmussen-Williams의 GP 교과서는 kernel을 닫힌 꼴로 쓴다. 실시간 SLAM에는 희소한 역행렬이 필요하다. Särkkä의 SDE 표현으로 이 둘을 이을 수 있었다.

이 연구는 **STEAM**(Simultaneous Trajectory Estimation and Mapping)으로 이어졌다. 2015년 IROS에서 [Sean Anderson과 Barfoot의 "Full STEAM Ahead"](https://doi.org/10.1109/IROS.2015.7353368)는 SE(3) 위의 constant-velocity prior로 STEAM을 정식화했다. 상태는 자세와 body-centric 속도로 이루어진다. 속도는 white noise를 적분한 만큼 변하고, 자세는 그 속도를 따라 움직인다. 같은 해 Anderson, Barfoot, Tong, Särkkä는 비선형 운동 모델로 넓힌 저널 논문 ["Batch nonlinear continuous-time trajectory estimation as exactly sparse Gaussian process regression"](https://doi.org/10.1007/s10514-015-9455-y)을 *Autonomous Robots*에 실었다.

STEAM에서는 GP 보간도 쓸 수 있다. 제어점(control pose)을 몇 개만 두고, 제어점 사이 아무 시각의 자세를 posterior mean으로 구한다. spinning LiDAR의 한 scan 안에서 점 10,000개가 각자 다른 시각에 찍혀도 제어점은 scan당 하나만 둔다. 그래서 추정할 상태 수는 관측 수보다 훨씬 적다. 다만 관측마다 residual을 계산해 더하는 일은 여전히 해야 한다.

[STEAM 오픈소스](https://github.com/utiasASRL/steam) 저장소는 2015년에 시작됐고, 커밋을 가장 많이 남긴 사람은 Sean Anderson이다. Dellaert 그룹의 GTSAM에는 2026년에 GP motion prior가 들어갔다.

---

## 7c.4 Lie group 위의 continuous-time

parametric 방식이든 nonparametric 방식이든 SLAM에서는 SE(3) 위의 궤적이 필요하다. Euclidean 공간의 spline이나 GP를 SE(3)로 옮기려면 손을 더 써야 한다. 흔히 tangent space에서 선형 보간한 뒤 exponential map으로 SE(3)에 되돌린다.

B-spline 쪽에서는 TUM의 Sommer, Usenko, Schubert, Demmel, Cremers가 Lie group cumulative B-spline의 시간 미분을 점화식으로 다시 유도했다. 논문은 ["Efficient Derivative Computation for Cumulative B-Splines on Lie Groups"(CVPR 2020)](https://arxiv.org/abs/1911.08860)이다. 기존 방식은 spline 차수 $k$에 대해 $O(k^2)$번의 행렬 연산이 필요했는데, 이 방식은 $O(k)$번으로 줄였다. SO(3) spline에서는 제어점으로 미분한 Jacobian도 해석적으로 구했다. 논문은 camera-IMU calibration 실험에서 계산 시간이 줄어드는 것을 보였다.

GP 쪽에서는 Anderson과 Barfoot이 "local variable" 방식을 썼다. 각 제어 자세 $T_k$ 근처에서 local perturbation $\xi_k(t) = \log(T(t)\,T_k^{-1})$를 정의하고, 이 변수에 GP를 둔다. SE(3) 전체에서 GP를 직접 정의하기는 어렵다. 제어점 근처의 tangent space에서는 Euclidean GP를 쓸 수 있다. 제어점 사이를 넘어갈 때 adjoint가 쓰이는데, 그 수학은 Ch.7b preintegration의 on-manifold 논의와 같다.

> 🔗 **차용.** [Anderson과 Barfoot의 IROS 2015 논문](https://doi.org/10.1109/IROS.2015.7353368)은 Lie group의 local variable에 GP를 적용하는 방법을 정리했다. 이웃한 두 제어점 사이에서만 GP를 쓰고, 제어점을 넘어갈 때 adjoint로 보정한다. 이 방법은 그 뒤 여러 continuous-time LiDAR·VIO 논문에서 쓰였다.

여기서 비교한 spline 구현과 GP 구현은 motion prior를 두는 방식이 다르다. spline에서는 계수를 직접 추정할 수도 있고, 계수나 미분에 별도 사전 분포를 줄 수도 있다. GP에는 SDE에서 유도한 constant-velocity나 white-jerk 같은 사전 분포가 들어 있어서, 관측이 드문 구간을 prior가 채운다. spline에서는 이웃한 관측과 basis가 그 구간의 모양을 정한다. 둘을 결합하려는 시도(Johnson et al. 2020)도 있었다. 실무에서는 응용에 따라 한쪽을 고른다.

---

## 7c.5 응용: LiDAR, VIO, event camera

continuous-time 궤적은 처음에는 calibration에 주로 쓰였다. 2022년 무렵에는 다음 분야에서 자주 쓰였다.

**LiDAR motion distortion.** MINES ParisTech와 Kitware의 Pierre Dellenbach 등은 [CT-ICP](https://arxiv.org/abs/2109.12979)(arXiv 2021, ICRA 2022)에서 scan마다 시작 자세와 끝 자세를 두고 그 사이를 보간했다. 회전은 slerp로, 위치는 선형으로 보간한다. 앞 scan의 끝 자세와 다음 scan의 시작 자세는 같지 않아도 되고, 둘이 가깝도록 하는 제약을 둔다. 간단한 continuous-time 모델이다. 논문은 KITTI, NCLT, Newer College를 포함한 일곱 개 데이터셋에서 실험했다. KITTI odometry leaderboard에서는 코드를 공개한 방법 가운데 1위(평균 상대 이동 오차 0.59%)라고 보고했다.

> 🔗 **차용.** CT-ICP는 [Besl·McKay 1992 ICP](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)에서 이어진 ICP의 point-to-plane 목적함수에 scan 안의 보간을 더했다.

같은 해 Toronto 그룹의 Keenan Burnett, Yuchen Wu 등은 ["Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?"](https://arxiv.org/abs/2203.10174)(RSS 2022)에서 radar만 쓰는 위치 추정, LiDAR만 쓰는 위치 추정, radar scan을 LiDAR 지도에 맞추는 위치 추정을 비교했다. 세 파이프라인의 odometry는 모두 Anderson과 Barfoot의 GP motion prior를 쓰는 continuous-time ICP였다. 같은 그룹의 Wu 등은 ["Picking up Speed"](https://doi.org/10.1109/LRA.2022.3226068)(RA-L 2023)에서 점마다 Doppler 속도를 함께 내는 Aeva FMCW LiDAR에 이 방법을 적용했고, [STEAM-ICP](https://github.com/utiasASRL/steam_icp) 코드를 공개했다. 이 속도 측정은 STEAM의 속도 상태와 바로 대응한다.

**Rolling shutter VIO.** [Kerl 등의 rolling shutter RGB-D 추적](https://doi.org/10.1109/ICCV.2015.261)(ICCV 2015)은 이미지의 행마다 그 행이 찍힌 시각의 자세를 B-spline 궤적에서 구했다. [Ctrl-VIO](https://doi.org/10.1109/LRA.2022.3202349)(RA-L 2022)는 B-spline 궤적으로 같은 방식을 rolling shutter VIO에 썼다. 공개 저장소 README는 spline 모듈을 Basalt에서 가져와 고쳤다고 밝힌다. global shutter를 가정하는 VIO와 달리 rolling shutter를 모델 안에서 처리한다.

**Event camera.** Ch.18에서 2010년대 event camera 연구의 어려움을 다룬다. 2020년대에는 continuous-time 궤적을 쓰는 event SLAM이 여럿 나왔다. 이 시스템들은 이벤트마다 μs 단위 타임스탬프로 B-spline이나 GP에서 그 순간의 자세를 구하고, event와 이미지의 일관성으로 residual을 계산한다.

---

## 📜 예언 vs 실제

> 📜 **예언 vs 실제.** Furgale·Barfoot·Sibley의 2012년 ICRA 논문은 고속 IMU와 sweeping laser가 discrete-time state를 비대하게 만드는 문제를 출발점으로 삼았고, 기존 rolling-shutter 보간 연구도 관련 사례로 언급했다. 논문이 직접 검증한 것은 camera–IMU calibration이었다. 이후 Barfoot·Tong·Särkkä(2014)가 SDE 기반 sparse GP를 정식화했다. rolling-shutter VIO와 event SLAM에서도 continuous-time trajectory가 쓰였다. 이 둘은 2012년 논문의 Future Work와는 따로 이어진 연구다. 그동안 visual SLAM에서는 이산 keyframe 기반 ORB-SLAM이 주류가 됐고, continuous-time 궤적은 주로 특수 센서에 쓰였다.

---

## 🧭 아직 열린 것

**학습 기반 continuous-time prior.** SDE에서 얻는 motion prior에는 constant-velocity나 white-jerk 같은 물리 가정이 들어 있다. 실제 주행·보행·UAV 궤적은 이 가정을 어기는 경우가 많다. 2023–2024년에는 neural SDE나 neural ODE로 데이터 기반 prior를 학습해 continuous-time factor graph에 넣으려는 시도가 나왔다. 실시간으로 도는 희소 구조를 유지하면서 학습한 prior를 쓰는 일은 남은 과제다.

**VIO와 continuous-time의 통합.** Ch.7b의 preintegration은 keyframe 기반 VIO의 사실상 표준이다. continuous-time 궤적이 preintegration을 대신할 수 있는지, 둘을 섞는 편이 나은지는 아직 가려지지 않았다. Le Gentil, Vidal-Calleja, Huang의 [GP 기반 preintegration](https://doi.org/10.1109/LRA.2020.2970940)(RA-L 2020)이 둘을 이으려 했다. 그러나 ORB-SLAM3나 VINS-Fusion처럼 널리 쓰이는 시스템은 여전히 이산 preintegration을 쓴다.

**임베디드 장치용 온라인 sliding window.** STEAM과 B-spline 기반 시스템은 제어점이 쌓이면 최적화가 느려진다. marginalization으로 과거 제어점을 제거하면서 continuous-time posterior의 일관성을 유지하기는 까다롭다.

---

continuous-time 표현과 Ch.7b의 이산 preintegration은 함께 쓸 수 있다. 앞에서 본 Le Gentil 등의 GP 기반 preintegration이 그런 예다. Ch.8은 다시 카메라로 돌아가 direct photometric 방법과 DSO, VI-DSO를 다룬다.
