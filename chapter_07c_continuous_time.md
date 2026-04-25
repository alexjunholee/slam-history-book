# Ch.7c — 시간이 매끈하게 흘러야 할 때: Continuous-Time Trajectory

Ch.7b가 정리한 preintegration은 IMU 측정을 이산 키프레임 사이의 relative factor로 압축하는 공학이었다. 그 압축은 "키프레임"이라는 단위를 전제로 성립한다. 두 키프레임 사이에 100번 들어온 관성 샘플이 하나의 factor로 접히려면, factor의 끝점이 둘 다 명확한 시각을 가져야 한다. 카메라 셔터가 글로벌하게 한 번 열리고 닫히는 시스템에서는 그 가정이 무해하다. 문제가 생기는 자리는 따로 있었다.

2012년 Toronto에서 [Paul Furgale·Timothy Barfoot·Gabe Sibley](https://asrl.utias.utoronto.ca/~tdb/bib/furgale_iros12.pdf)가 IROS 논문에서 질문을 정식화했다. rolling shutter로 찍은 한 장의 이미지에서 각 행은 다른 시각의 자세로 투영된다. spinning LiDAR가 한 바퀴를 도는 사이에 차량은 수 미터를 달린다. IMU는 1 kHz로 쏟아지는데 카메라는 30 Hz다. 이 센서들을 하나의 최적화로 묶는 가장 자연스러운 방법은 자세를 "프레임"이 아니라 "시간 t의 함수"로 두는 것이었다. Furgale·Barfoot·Sibley는 B-spline을 골랐고, 그 선택이 continuous-time trajectory estimation이라는 갈래의 공식 출발점이 됐다.

10년 뒤, Handbook은 이 갈래를 manifold와 나란히 SLAM의 "기본 도구 2개" 중 하나로 배치한다. 우리 책의 이전 장들이 한 번도 이 도구를 꺼내지 않은 것은 Visual-Inertial의 문법이 대부분 discrete keyframe에서 완성됐기 때문이다. 이 장은 그 공백을 메운다.

---

## 7c.1 Discrete-time의 한계

Ch.7b의 preintegration이 해결한 것은 "IMU가 카메라보다 빠르다"는 단일 축이었다. 해결하지 못하는 축은 네 개가 더 있다.

첫째, rolling shutter. consumer CMOS 카메라는 한 프레임을 위에서 아래로 수십 ms에 걸쳐 읽는다. 빠르게 움직이는 카메라에서 첫 행과 마지막 행은 서로 다른 자세에서 찍힌다. Ch.8의 DSO·LSD-SLAM이 photometric consistency를 가정할 때 이 왜곡은 모델 밖에 있었다. Cremers 그룹이 2019년 [Basalt](https://arxiv.org/abs/1904.06504)에 B-spline 궤적을 올린 이유가 여기에 있다.

둘째, spinning LiDAR motion distortion. Ch.17에서 보았듯 Velodyne HDL-64E는 10 Hz로 한 바퀴를 돈다. 그 100 ms 사이에 차량이 10 m/s로 달리면 한 scan 안의 점들이 1 m씩 다른 자세에서 찍힌다. LOAM은 이 왜곡을 odometry 루프 안에서 간접 보정했지만, 원리적 해법은 "각 점이 찍힌 순간의 자세"를 질의할 수 있는 궤적 표현이었다.

셋째, event camera. Ch.18이 기록한 DVS는 픽셀마다 μs 단위로 비동기 이벤트를 쏟는다. 이벤트에는 "프레임"이 없다. [Mueggler et al. 2015](https://arxiv.org/abs/1502.00796)가 SE(3) B-spline 궤적 위에서 event SLAM을 정식화한 것은 다른 선택지가 없어서였다.

넷째, high-rate IMU를 이질 주파수 센서 여럿과 동시 결합하는 일. 한 시스템에 200 Hz IMU, 20 Hz 카메라, 10 Hz LiDAR가 들어오면, 이산 상태 노드를 모든 측정 시각마다 두는 것은 현실적이지 않다. 상태의 수가 측정의 수를 따라가는 순간 factor graph는 부풀어 오른다.

네 문제의 공통 구조는 같다. 측정 시각 $t_i$가 제어되지 않는다. 관측은 아무 때나 들어오고, 추정기는 그 시각의 자세를 알아야 한다. "측정 시각 / 추정 시각 / 질의 시각"의 분리가 continuous-time representation의 본질적 이점이다.

---

## 7c.2 Parametric spline: Furgale 계보

Furgale·Barfoot·Sibley가 2012년에 고른 도구는 B-spline이었다. 궤적을 basis function의 합 $\mathbf{p}(t) = \sum_k \Psi_k(t)\,\mathbf{c}_k$로 쓰고, 계수 $\mathbf{c}_k$를 최적화 변수로 둔다. B-spline의 핵심은 local support다. 한 시점 t에서 0이 아닌 basis는 소수(보통 4개)뿐이고, 나머지는 정확히 0이다. 임의 시각 $t_i$의 자세를 질의하는 비용이 상수고, factor graph sparsity가 그대로 유지된다.

> 🔗 **차용.** B-spline의 수학적 뼈대는 [de Boor (1978) *A Practical Guide to Splines*](https://link.springer.com/book/10.1007/978-1-4612-6333-3)의 고전이다. Furgale이 한 일은 그 뼈대를 SE(3) 위로 끌어올리고, 계수를 factor graph의 변수 노드로 배치한 것이다. 수치해석 교과서의 도구가 SLAM 최적화로 이식된 경로다.

형식의 약점은 명확했다. 계수 간격을 좁게 잡으면 과적합하고, 넓게 잡으면 빠른 움직임을 놓친다. 간격 선택이 경험 의존이었다. 그리고 linear B-spline을 SE(3)에 그대로 얹으면 보간 결과가 매니폴드를 벗어난다.

2013년 Oxford의 [Steven Lovegrove et al.](https://www.roboticsproceedings.org/rss09/p11.html)가 cumulative B-spline을 제안했다. basis를 합이 아니라 누적 곱 형태 — $T(t) = \prod_k \exp\bigl(\tilde\Psi_k(t) \log(T_k T_{k-1}^{-1})\bigr) \cdot T_0$ — 로 재배치하면 각 인자가 Lie group에 닫혀 있다. 이 형식은 이후 rolling-shutter·event camera·VIO 논문의 기본어가 됐다. Basalt, [Mueggler event SLAM](https://arxiv.org/abs/1502.00796), [Kerl et al. 2015 dense rolling shutter VO](https://doi.org/10.1109/ICCV.2015.172)가 모두 cumulative B-spline 위에 섰다.

Parametric spline은 계산이 가볍고 코드가 단순하다는 이점 때문에 실시간 VIO와 event 시스템에서 꾸준히 쓰였다. 대신 궤적에 대한 사전 분포(motion prior)를 자연스럽게 얹는 방법이 없었다. 관측이 드문 구간에서 spline은 매끈하긴 하지만 근거 없이 매끈했다. 그 공백을 다른 갈래가 메운다.

---

## 7c.3 SDE 기반 GP: Barfoot 계보와 STEAM

같은 2014년, 토론토의 Barfoot 그룹이 두 번째 갈래를 열었다. [Barfoot, Tong, Särkkä 2014 "Batch Continuous-Time Trajectory Estimation as Exactly Sparse Gaussian Process Regression"](https://www.roboticsproceedings.org/rss10/p01.pdf)은 제목이 그대로 주장이었다. 궤적을 basis 합이 아니라 Gaussian process로 두겠다. 궤적의 사전 분포는 kernel $\mathcal{K}(t, t')$로 주어지고, 관측이 들어오면 posterior를 조건부 Gaussian으로 닫는다.

GP의 순수 형태는 문제가 하나 있다. 관측 수 $N$이 크면 kernel matrix $K$의 역행렬 비용이 $O(N^3)$이다. Barfoot·Tong·Särkkä가 보인 것은 이 비용을 회피할 수 있는 kernel의 가족이 있다는 것이었다. 궤적이 linear time-invariant stochastic differential equation $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + L\mathbf{w}(t)$의 해로 정의될 때, 그 kernel $K$의 역행렬 $K^{-1}$이 block-tridiagonal 구조를 가진다. factor graph로 읽으면 연속한 상태 노드 사이에만 binary factor가 있고, 멀리 떨어진 노드 사이에는 factor가 없다.

> 🔗 **차용.** "GP posterior를 factor graph의 prior로 재해석한다"는 틀은 [Särkkä 2013 *Bayesian Filtering and Smoothing*](https://users.aalto.fi/~ssarkka/pub/cup_book_online_20131111.pdf)이 정리한 SDE-GP 연결을 Barfoot 그룹이 SLAM으로 끌어온 것이다. Rasmussen-Williams의 GP 교과서는 kernel을 닫힌 형식으로 쓰지만, 실시간 SLAM은 sparse inverse를 원한다. Särkkä의 SDE 표현이 그 다리였다.

실무 귀결이 **STEAM** (Simultaneous Trajectory Estimation and Mapping)이다. 2015년 RSS에서 [Sean Anderson·Barfoot 2015 "Full STEAM Ahead"](https://www.roboticsproceedings.org/rss11/p45.pdf)가 constant-velocity prior 기반 STEAM을 공식화했다. 상태를 자세 $\mathbf{p}(t)$와 속도 $\mathbf{v}(t)$로 augment하고, 속도의 white noise 적분으로 자세가 따라가는 구조다. Anderson은 같은 해 sparsity 증명을 tightened 형태로 완성했고, 그 증명이 이후 Barfoot 그룹의 모든 continuous-time 논문의 backbone이 됐다.

STEAM의 두 번째 이점이 GP interpolation이었다. 제어점(control pose)을 소수만 두고, 제어점 사이 임의 시각의 자세를 posterior mean으로 질의할 수 있다. spinning LiDAR의 한 scan 안에서 10,000개의 점이 각자 다른 시각에 찍혀도, 제어점은 scan 당 하나만 둔다. 계산량이 관측 수가 아니라 제어점 수에 비례한다.

2019년 Tang·Barfoot의 [STEAM 오픈소스](https://github.com/utiasASRL/steam)가 공개되면서 학계·산업계에서 직접 쓸 수 있는 라이브러리가 됐다. 같은 해 Dellaert 그룹의 GTSAM에도 GP continuous-time factor가 contrib로 들어간다. 두 경로의 수렴이었다.

---

## 7c.4 Lie group 위의 continuous-time

Parametric이든 nonparametric이든 SLAM은 SE(3) 위의 궤적을 원한다. Euclidean 상의 spline·GP를 SE(3)로 끌어올리는 일은 기술적으로 간단하지 않다. tangent space에 기대서 선형 보간을 한 뒤 exponential map으로 매니폴드에 얹는 방식이 통용된다.

B-spline 쪽에서는 [Sommer, Demmel et al. 2020 "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups"](https://arxiv.org/abs/1911.08860)가 SE(3) cumulative spline의 Jacobian을 닫힌 형식으로 정리했다. CVPR에 실린 이 논문은 rolling-shutter VIO·event camera·visual-inertial 시스템에서 실시간 미분이 가능한 B-spline 궤적의 표준 공식을 제공했다. Basalt와 Cremers 그룹 후속 작업이 이 정리 위에 섰다.

GP 쪽에서는 Anderson·Barfoot이 "local variable" 구도를 제안했다. 각 제어 자세 $T_k$ 근처에서 local perturbation $\xi_k(t) = \log(T(t)\,T_k^{-1})$를 정의하고, 그 위에서 GP를 운용한다. 전역 매니폴드 위에서 직접 GP를 정의하는 것은 어렵지만, 각 제어점 근방의 tangent space에서는 euclidean GP가 성립한다. 제어점 사이를 건너뛸 때 adjoint가 등장하는데, 그 수학적 근거는 Ch.7b preintegration의 on-manifold 논의와 같다. 두 도구가 같은 Lie group 문법을 공유한다는 사실이 2015년 이후 분명해졌다.

> 🔗 **차용.** GP를 Lie group local variable로 이식한 경로는 [Anderson-Barfoot 2015 ICRA](https://doi.org/10.1109/ICRA.2015.7138984)가 처음 체계화했다. 이들이 쓴 트릭 — "연속한 두 제어점 사이에서만 GP를 돌리고, 제어점 사이를 건너뛸 때 adjoint로 보정" — 은 이후 continuous-time LiDAR·VIO 논문이 모두 물려받는다.

spline과 GP의 실질적 차이는 motion prior의 유무다. spline은 계수를 직접 추정하고 사전 분포가 없다. GP는 SDE에서 유도된 사전 분포가 constant-velocity 혹은 white-jerk 등으로 내장돼 있다. 관측이 드문 구간에서 GP는 prior가 채우고, spline은 인접 관측이 채운다. 둘을 결합하려는 시도(Johnson et al. 2020)도 있었지만, 실무에선 application에 따라 한쪽을 고른다.

---

## 7c.5 응용으로 내려온 계보: LiDAR와 VIO

이론이 응용으로 내려오는 데 10년이 걸렸다. 2022년을 기점으로 continuous-time이 세 현장에서 사실상 표준이 된다.

첫째, LiDAR motion distortion. Paris의 [Pierre Dellenbach et al. 2022 "CT-ICP"](https://arxiv.org/abs/2109.12979)는 각 scan을 "시작 자세"와 "끝 자세" 두 개로 파라미터화하고 그 사이를 선형 보간했다. 간단한 continuous-time 모델이지만, KITTI·NCLT·Newer College 벤치마크에서 기존 LOAM·FAST-LIO의 정확도를 상회했다. 같은 해 Toronto의 [Keenan Burnett et al. 2022 "Are We Ready for Radar to Replace Lidar?"](https://arxiv.org/abs/2206.05432)와 [STEAM-ICP](https://github.com/utiasASRL/steam_icp)가 GP 기반 continuous-time을 Aeva FMCW LiDAR에 적용했다. Aeva 센서가 각 점마다 도플러 속도를 함께 출력하는데, 이 속도는 STEAM의 속도 상태와 직접 대응한다. continuous-time 표현이 아니었다면 쓸 방법이 없는 정보였다.

둘째, rolling-shutter VIO. Basalt·[Cremers 그룹 rolling-shutter VO](https://doi.org/10.1109/CVPR.2016.71)·[OKVIS](https://doi.org/10.1177/0278364914554813) 후속작들이 이미지 각 행의 찍힌 시각을 B-spline 궤적에 질의한다. 글로벌 셔터를 가정하고 우회하는 기존 VIO와 달리 rolling shutter 자체를 모델 안에서 처리한다.

셋째, event camera. Ch.18이 기록한 2010년대의 좌절 이후, 2020년대의 event SLAM은 거의 모두 continuous-time 궤적 위에 섰다. 각 이벤트의 μs 타임스탬프를 B-spline 혹은 GP에 질의해 그 순간의 자세를 얻고, event-image consistency로 residual을 계산한다. event가 "프레임이 없는 센서"라는 사실과 continuous-time이 "프레임 가정이 필요 없는 표현"이라는 사실이 자연스럽게 맞물렸다.

> 🔗 **차용.** CT-ICP는 [Besl·McKay 1992 ICP](https://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_icp.pdf)의 point-to-plane 목적함수에 scan 내부 continuous-time linear 보간을 얹은 조합이다. 고전 registration과 Furgale의 continuous-time 정신이 30년의 간격을 두고 한 시스템에서 만났다.

---

## 📜 예언 vs 실제

> Furgale·Barfoot·Sibley가 2012년 IROS 논문 Future Work에 적은 기대는 두 갈래였다. 하나는 "continuous-time 표현이 rolling shutter와 IMU 고속 샘플링을 통합하는 자연스러운 언어가 될 것"이라는 전망이었고, 다른 하나는 "sparse factor graph 호환성을 증명하는 후속 작업"이었다. 두 기대 모두 10년 안에 들어맞았다. Barfoot·Tong·Särkkä 2014가 sparse GP 증명을 닫았고, 2020년대 rolling-shutter VIO와 event SLAM은 cumulative B-spline을 기본어로 쓴다. 다만 저자들이 예측하지 않은 전개가 하나 있다. 2012년 당시에는 "discrete keyframe 기반 ORB-SLAM이 주류가 되고, continuous-time은 특수 센서용"이라는 분업이 암묵적으로 상정됐다. 실제로는 반대 방향의 밀어올림도 나왔다. Burnett이 FMCW LiDAR의 도플러 속도를 쓰는 STEAM-ICP를 내놓으면서, continuous-time이 특수 센서를 다루는 부록이 아니라 센서의 능력을 끌어내는 능동적 표현이 됐다. `[적중+확장]`

---

## 🔗 차용 (요약)

위에 산재한 세 상자 외에, 이 장이 기댄 다른 계보를 한 번 더 모은다.

Särkkä의 SDE-GP 교과서가 없었다면 Barfoot·Tong·Särkkä 2014는 수식의 앵커가 없었을 것이다. de Boor의 1978년 spline 고전이 없었다면 Furgale 2012는 basis function을 처음부터 쌓아야 했다. Anderson·Barfoot 2015의 local variable 기법이 없었다면 GP를 Lie group으로 이식하는 작업은 더 오래 걸렸을 것이다. continuous-time trajectory estimation은 수치해석·확률 이론·Lie group 미분기하의 세 줄기가 SLAM이라는 좁은 지점으로 모여든 자리다.

---

## 🧭 아직 열린 것

**Learning-based continuous-time prior.** SDE가 주는 motion prior는 constant-velocity나 white-jerk 같은 물리 가정을 내장한다. 실제 주행·보행·UAV 궤적은 이 가정을 어기는 경우가 많다. 2023-2024년 neural SDE나 neural ODE로 데이터 기반 prior를 학습해 continuous-time factor graph에 꽂으려는 시도들이 나왔다. 아직 실시간 sparse 구조를 유지한 채 learned prior를 얹은 시스템은 검증 단계다.

**VIO와 continuous-time의 통합.** Ch.7b의 preintegration은 keyframe 기반 VIO의 사실상 표준으로 남아 있다. continuous-time 궤적이 preintegration을 대체할 수 있는지, 혹은 두 도구가 공존하는 하이브리드가 더 나은지는 2026년 기준 결론이 없다. Le Gentil의 [GP-augmented preintegration 계보](https://arxiv.org/abs/2007.04144)가 한 다리를 놓으려 시도 중이지만, ORB-SLAM3·VINS-Fusion 수준의 배포 시스템에서는 여전히 discrete-time preintegration이 주력이다.

**Edge deployment를 위한 online sliding window.** STEAM과 B-spline 기반 시스템은 제어점 수가 누적되면 최적화가 느려진다. marginalization으로 과거 제어점을 제거하면서 continuous-time posterior의 일관성을 유지하는 문제는 기술적으로 까다롭다. 자동차·드론 같은 임베디드 플랫폼에서 continuous-time SLAM을 10년짜리 표준으로 밀어올리려면 이 공백이 먼저 메워져야 한다.

---

Ch.7b가 이산 시간의 효율을 끝까지 짜낸 공학이었다면, 이 장은 그 바깥 — "시간이 매끈하게 흘러야 할 때" — 에서 자란 갈래의 계보였다. 두 도구는 경쟁하지 않는다. 하나의 SLAM 시스템에 IMU preintegration factor와 continuous-time LiDAR factor가 나란히 들어가는 구성이 2024년 이후 꾸준히 보고되고 있다. 다음 장은 이 모든 문법이 어떤 시각 계보의 꼭짓점에서 맞닥뜨리는지를 다룬다.
