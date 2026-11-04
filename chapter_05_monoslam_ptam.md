# Ch.5 — MonoSLAM → PTAM: 카메라 한 대로 하는 실시간 SLAM과 tracking·mapping 분리

EKF-SLAM은 로봇 상태와 불확실성을 함께 추정하는 지도 작성 방법이었지만, 공분산 행렬이 landmark 수 $N$에 대해 $O(N^2)$로 커져 지도를 키우기 어려웠다. Davison과 Klein은 이 문제를 서로 다른 방식으로 다뤘다.

2003년 Oxford의 Davison은 손에 들고 움직이는 카메라의 위치와 주변 지도를 데스크톱 PC에서 실시간으로 추정했다. Smith, Self, Cheeseman의 확률적 공간 관계 수학과 Leonard·Durrant-Whyte의 EKF-SLAM 틀을 가져왔지만, 센서는 카메라 하나뿐이었다. IMU도 스테레오 카메라도 레이저도 없이 Shi-Tomasi의 특징 검출 연산자와 Kalman 필터의 예측-갱신 루프만 썼다.

4년 뒤인 2007년, 같은 Oxford의 Klein과 Murray는 tracking과 mapping을 두 스레드로 나눈 시스템을 내놓았다. 그 뒤 10년 동안 나온 Visual SLAM 시스템 상당수가 이 구조를 따랐다.

---

## 1. 2003년의 데모

2003년 ICCV에서 Davison은 [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654)를 발표했다. 단안 카메라 하나로 SLAM을 실시간으로 돌린 결과였다.

Davison이 논문에서 지적했듯 당시 SLAM 연구에서 카메라를 주 센서로 쓴 경우는 드물었고, 주로 레이저 센서를 썼다. 2D 레이저 스캐너는 거리를 직접 쟀고, 스테레오 카메라는 두 영상의 시차로 픽셀마다 깊이를 구했다. 단안 카메라로 3D 구조를 추정하려면 적어도 두 프레임이 필요했고, 처음 추정한 깊이의 불확실성이 EKF 상태 벡터 전체로 퍼졌다. 이론으로는 풀 수 있는 문제였지만, 실시간으로 돌리기는 어려웠다.

MonoSLAM은 단안 카메라만 썼으므로 추가 하드웨어도, 스테레오 캘리브레이션도 필요 없었다.

---

## 2. EKF로 짠 단안 SLAM과 그 한계

2007년 IEEE PAMI에 실린 [MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049)은 Davison, Ian Reid, Nicholas Molton, Olivier Stasse가 함께 쓴 논문으로, ICCV 2003 데모를 완성한 형태였다.

MonoSLAM의 상태 벡터는 [Smith-Self-Cheeseman(1988)](https://arxiv.org/abs/1304.3111)과 [Leonard-Durrant-Whyte(1991)](https://ieeexplore.ieee.org/document/174711/)의 정식화(Ch.4)를 단안 카메라에 그대로 옮겼다. 카메라 상태 $\mathbf{x}_v \in \mathbb{R}^{13}$(위치 3, 사원수 방향 4, 속도 3, 각속도 3)과 landmark 위치 $\mathbf{y}_i \in \mathbb{R}^3$를 벡터 하나 $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$에 담았다. $(13+3N)\times(13+3N)$ 공분산 행렬 $\mathbf{P}$는 프레임마다 예측-갱신 루프로 유지했다. 예측 단계에서는 카메라 운동 모델 $f$의 Jacobian $\mathbf{F}$로 공분산을 전파했고($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$), 갱신 단계에서는 투영 함수의 Jacobian $\mathbf{H}_i$로 Kalman gain을 계산해 상태와 공분산을 갱신했다. 수식은 Ch.4 §4.3과 같다. 다른 점은 상태 벡터에 카메라의 속도와 각속도가 들어간 것이다. 손에 든 카메라가 어떻게 움직일지 예측하려면 운동 모델이 필요했다.

공분산 행렬은 모든 landmark 사이의 상관관계를 담고 있었다. 공분산 갱신 $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$에서 비용이 가장 큰 부분은 $(13+3N)^2$ 크기 행렬 곱셈이고, 이 비용은 $N$에 대해 $O(N^2)$로 는다. PAMI 논문은 이 때문에 실시간으로 유지할 수 있는 feature 수가 제한된다고 적었다. 자기 시스템에서는 30 Hz 기준 "around 100"개였다.

Davison은 논문 결론에서 더 넓은 공간으로 가려면 sub-map 전략이 "certainly seems appropriate"라고 적으면서도, 시각 특징 지도를 의미 있는 하위 블록으로 깔끔하게 나누는 방법은 아직 분명하지 않다고 덧붙였다.

MonoSLAM은 [Shi-Tomasi(1994)](https://doi.org/10.1109/CVPR.1994.323794)의 연산자로 눈에 띄는 영상 영역을 찾아 landmark로 삼았다. "Good Features to Track"은 추적하기 좋은 점을 고르는 기준이었다. MonoSLAM은 질 좋은 특징으로 이루어진 희소한 지도를 유지하는 쪽을 택했고, PAMI 논문에 따르면 광각 렌즈에서 프레임마다 특징 12개 정도가 보이도록 지도를 관리했다.

> 🔗 **차용.** Shi-Tomasi(1994)의 코너 기준은 PTAM보다 앞서 MonoSLAM에서 이미 쓰였다. PTAM은 추적용 코너를 FAST-10으로 검출하고, 지도에 넣을 점을 고를 때 Shi-Tomasi 점수를 썼다.

---

## 3. 2007년, 같은 해

2007년 ISMAR에서 Klein과 Murray는 [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852)를 발표했다. 같은 해 PAMI에는 MonoSLAM의 정식 논문이 실렸다.

Klein은 Cambridge에서 Tom Drummond의 지도로 박사학위를 받았고, PTAM을 발표할 때는 Oxford에서 David Murray와 함께 일하는 박사후 연구원이었다. Murray는 Davison의 Oxford 박사과정 지도교수이기도 했다. PTAM 논문은 MonoSLAM이 질 좋은 특징의 희소한 지도를 쓴다고 짚고, 자신들은 질은 낮아도 훨씬 조밀한 지도를 쓰며 EKF 대신 tracking과 mapping을 나누는 구조를 택했다고 적었다.

---

## 4. Tracking과 Mapping을 나누다

PTAM은 Tracking(카메라 pose 추적)과 Mapping(3D 지도 작성)을 두 스레드로 나눠 병렬로 돌렸다.

EKF에서는 이 둘이 한 루프 안에 섞여 있었다. 프레임마다 예측-갱신을 한 번씩 돌리면서, 카메라가 움직이면 상태를 예측하고 이미지에서 landmark를 찾으면 상태를 갱신했다.

PTAM의 Tracking 스레드는 프레임마다 카메라 pose만 추정한다. 지도에 있는 3D 점들을 현재 이미지에 투영하고 실제 관측과 매칭해서 pose를 실시간으로 계산한다. Mapping 스레드는 새 keyframe이 들어오면 가장 새 keyframe과 가까운 keyframe 4개, 모두 5개에 대해 local bundle adjustment를 돌리고, 시간이 남으면 전체 지도에 global bundle adjustment를 돌린다. Tracking 스레드가 따로 돌기 때문에 Mapping이 느려져도 추적은 멈추지 않았다.

Mapping 스레드의 bundle adjustment는 keyframe 집합 $\mathcal{K}$와 3D 점 집합 $\mathcal{P}$에 대해 reprojection error의 합을 최소화했다:
$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$
$\mathbf{T}_k \in SE(3)$는 keyframe $k$의 pose, $\mathbf{p}_j \in \mathbb{R}^3$는 3D 점, $\pi$는 카메라 투영 함수, $\mathbf{z}_{kj}$는 keyframe $k$에서 점 $j$를 관측한 픽셀 좌표, $\mathbf{\Sigma}_{kj}$는 측정 공분산, $\rho$는 robust kernel로, PTAM은 Tukey biweight 함수를 썼다. 논문은 이 최적화를 풀기 위해 Hartley·Zisserman 교재 Appendix 6.6에 실린 Levenberg–Marquardt bundle adjustment를 "almost textbook" 그대로 구현했다고 적었다.

> 🔗 **차용.** PTAM의 Mapping 스레드가 돌리는 bundle adjustment는 사진측량에서 온 방법이다. PTAM 논문이 구현 근거로 든 것은 Hartley·Zisserman의 *Multiple View Geometry*(2판) Appendix 6.6이고, [Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"](https://doi.org/10.1007/3-540-44480-7_21)는 인용하지 않았다.

Mapping 스레드가 bundle adjustment를 비동기로 돌리자, 지도에 넣을 수 있는 landmark 수가 EKF의 $O(N^2)$ 제약에서 벗어났다. 논문에 실린 예에서 PTAM의 지도에는 점 2000~6000개와 keyframe 40~120개가 들어갔고, 점 11000개와 keyframe 280개까지 시험했다. 다만 keyframe이 100개를 넘으면 global bundle adjustment가 새 keyframe 추가 속도를 따라가지 못했다. feature 100개 안팎을 다루던 MonoSLAM과는 규모가 달랐다.

초기 지도를 만드는 방법도 달랐다. 초기화 단계에서 사용자가 카메라를 부드럽게 옆으로 옮기면, PTAM은 [Nistér 2004](https://doi.org/10.1109/TPAMI.2004.17)의 5-point 알고리즘 계열로 essential matrix를 추정하고 첫 keyframe 쌍에서 초기 3D 구조를 복원했다.

Essential matrix $\mathbf{E}$는 두 카메라 좌표계 사이의 기하 관계를 담은 $3\times 3$ 행렬로, 대응점 쌍 $(\mathbf{p}, \mathbf{p}')$에 대해 ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$을 만족한다. $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$($\mathbf{t}_\times$는 이동 벡터의 반대칭 행렬, $\mathbf{R}$은 회전)로 쓸 수 있어서 자유도가 5다. 대응점 5쌍이면 문제를 정할 수 있지만 해가 하나로 정해지지 않고, 복소수 해까지 치면 후보가 최대 10개 나온다. Nistér는 이 5-point 연립방정식을 빠르게 풀어 RANSAC 루프 안에서 실시간으로 돌릴 수 있게 했다. PTAM은 초기화 때 이 solver를 RANSAC과 함께 써서 첫 두 keyframe 사이의 상대 pose를 추정하고, 초기 3D 점군을 삼각측량으로 복원했다.

> 🔗 **차용.** PTAM의 5-point essential matrix 초기화는 Nistér(2004) "An Efficient Solution to the Five-Point Relative Pose Problem"에서 시작한 minimal solver(필요한 최소 개수의 대응점으로 해를 구하는 solver) 연구를 따른다. PTAM 논문이 직접 인용한 것은 그 후속인 Stewénius·Engels·Nistér(2006, ISPRS)다.

> 🔀 **구조적 평행.** EKF-SLAM의 submap과 PTAM의 keyframe 지도는 모두 계산량을 관리하려고 전역 지도를 골라낸 지역 단위로 다룬다. 그러나 PTAM 논문이 submap 연구를 설계 근거로 인용한 흔적은 찾지 못했다. PTAM 논문이 keyframe을 쓰는 이유로 든 것은 매 프레임을 처리하지 않아도 되므로 mapping을 실시간으로 돌릴 필요가 없다는 점이다. 뒤이은 ORB-SLAM은 여기에 covisibility graph를 더했다.

---

## 5. 새 구조가 퍼지다

PTAM은 AR(증강현실) 작업 공간을 대상으로 설계됐다. 논문 제목에도 "Small AR Workspaces"가 들어 있다. Tracking 스레드가 안정적이고 실시간으로 확실히 돌았기 때문에 AR 응용에 바로 쓸 수 있었다.

상용 AR SDK에도 SLAM이 들어갔다. Metaio(독일 AR 스타트업, 2015년 Apple이 인수)는 2012년 SDK에 SLAM을 넣었다. 회사 연구진의 [발표 자료](https://jsturm.de/publications/data/sturm2014ismar.pdf)(ISMAR 2014)는 이 SDK의 sparse SLAM이 local·global bundle adjustment와 relocalization을 갖췄다고 적었다.

2015년 IEEE Transactions on Robotics에 Raúl Mur-Artal, J.M.M. Montiel, Juan D. Tardós가 발표한 [ORB-SLAM](https://arxiv.org/abs/1502.00956)은 PTAM의 구조를 이어받았다. 특징은 patch에서 ORB 서술자로 바꾸고, keyframe 관리에는 covisibility graph를 쓰고, loop closure를 새로 더했다.

2018년 IEEE Transactions on Robotics에 실린 Qin, Li, Shen의 [VINS-Mono](https://arxiv.org/abs/1708.03852)도 sliding window 최적화와 loop closure를 서로 다른 스레드에서 돌린다. tracking과 mapping을 나누는 구조가 VIO까지 퍼진 예다.

---

## 6. Davison과 Klein·Murray의 관점

PAMI 논문은 표준 full-covariance EKF를 고른 이유로 목표를 들었다. 제한된 공간 안에서 오래, 반복해서 위치를 추정하는 것이 목표였다. 논문은 이때 같은 특징을 되풀이해 관측하므로, 복도를 따라 탐사하다 가끔 루프를 닫는 로봇 지도 작성과 관측 패턴이 다르다고 적었다. EKF는 상태의 불확실성을 공분산 행렬로 명시해서 관리했다. landmark 하나하나가 얼마나 불확실한지, landmark끼리 공분산으로 어떻게 얽혀 있는지를 수식이 계속 추적했다. 이 관점에서 bundle adjustment는 불확실성 표현을 일부 포기하고 확장성을 얻는 최소자승 최적화였다.

PTAM은 그 손해를 받아들였다. AR 응용에서 중요한 것은 카메라 pose를 실시간으로 추적하는 일이었고, 지도의 불확실성을 센티미터 단위로 추적할 필요는 없었다. bundle adjustment로 지도를 주기적으로 다듬으면 충분했다.

그 뒤 SLAM 연구는 PTAM 쪽을 따랐다. 2010년대에는 graph 기반 최적화와 bundle adjustment가 주류가 됐다. Davison의 연구실은 factor graph 기반 추정을 거쳐 Gaussian Belief Propagation(GBP)과 Robot Web 연구로 옮겨 갔다. [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release) Ch.18 "The Computational Structure of Spatial AI Systems"에서 Davison은 현재 기하 추정의 핵심 기법이 EKF와 BA 같은 Gaussian 기반 기법이라고 쓰고, 그 연장으로 GBP를 소개한다. 같은 장은 MonoSLAM과 PTAM을 실험해 볼 수 있는 오픈소스 연구 시스템의 예로 든다.

---

## 📜 예언 vs 실제

> **Davison 2007 PAMI MonoSLAM**: Davison은 결론에서 더 넓은 실내·실외 환경, 더 빠른 움직임, 가림과 조명 변화가 있는 복잡한 장면을 다음 과제로 꼽았다. 구체적인 수단으로는 sub-map 전략과 전체 해상도에서 100 Hz로 찍는 IEEE 1394 CMOS 카메라를 들었고, 희소한 지도를 표면 같은 "higher-order entities"의 dense 표현으로 넓힐 수 있다고도 적었다.
>
> SLAM Handbook 15.5절은 지금의 SLAM 시스템이 환경 변화와 움직임에 이미 꽤 강건하다고 보면서도, 여러 극한 조건에 함께 효율적이고 강건한 SLAM은 열린 문제로 남겼다. DynaSLAM 같은 시도는 Ch.15b에서 다룬다. IMU 결합은 Davison이 결론에서 꼽지 않았고(관련 연구는 본문에서 인용), 2010년대 Visual-Inertial Odometry(VIO) 연구에서 본격적으로 다뤄졌다.

> **Klein & Murray 2007 PTAM**: Klein과 Murray는 §8(Failure modes / Mapping inadequacies)에서 코너 기반 추적이 모션 블러에 약하다는 점, 점 구름 위주의 지도가 장면의 기하를 이해하지 못한다는 점, 그리고 시스템이 "not designed to close large loops in the SLAM sense"라는 점을 한계로 들었다. 큰 루프를 닫지 못하는 이유로는 tracking의 M-estimator가 지도의 불확실성을 알지 못한다는 점을 적었다.
>
> 2015년 ORB-SLAM은 이 한계를 직접 겨냥했다. [DBoW2](http://doriangalvez.com/papers/GalvezTRO12.pdf)로 이미지 외관을 비교해 loop closure를 하고, covisibility graph로 keyframe을 관리했다. 외관 기반 loop closure는 ORB-SLAM이 PTAM 구조에 더한 기능이다. 모션 블러와 점 구름 지도 문제는 ORB-SLAM에서도 그대로 남았다.

---

## 🧭 아직 열린 것

**단안 카메라의 scale 복원.** MonoSLAM부터 PTAM까지 단안 카메라 시스템은 모두 scale을 알 수 없다. 장면 전체를 같은 비율로 키우거나 줄여도 이미지는 똑같이 찍히기 때문이다. IMU를 더하면 중력 방향과 가속도계 값으로 scale을 관측할 수 있다. IMU 없는 순수 단안 시스템의 scale 복원은 2026년에도 풀리지 않았다. [MiDaS](https://arxiv.org/abs/1907.01341)나 [Depth Anything](https://arxiv.org/abs/2401.10891) 같은 학습 기반 monocular depth 추정은 이미지 한 장에서 상대 깊이를 구하지만, 이것을 미터 단위로 바꾸려면 지면 가정이나 크기를 아는 물체 같은 외부 기준이 필요하다. 미터 단위 깊이를 바로 낸다는 모델도 나왔지만, 학습하지 않은 데이터셋에서 scale을 틀리는 경우가 보고됐다(Ch.11 §🧭 참조).

**어떤 환경에서나 도는 VO 시스템.** MonoSLAM의 실험은 실내의 작은 공간에 머물렀다. PTAM은 스스로 "Small AR Workspaces"로 범위를 좁혔다. 그 뒤 ORB-SLAM2가 실내, 실외, RGB-D를 함께 다루려 했지만 조명이 극단적으로 바뀌거나 텍스처가 적은 공간에서는 지금도 추적이 끊긴다. 카메라, LiDAR, IMU를 함께 쓰면 이 가운데 일부를 넘을 수 있다.

**어두운 곳과 움직이는 장면의 특징점 추적.** MonoSLAM은 조명이 충분하고 장면이 정지해 있어야 돌았다. 2007년의 PTAM도 마찬가지였다. 2026년에도 feature 기반 SLAM 시스템 대부분은 이 두 가정을 말없이 깔고 있다. 어두운 곳에서는 ORB feature가 잘 검출되지 않는다. 움직이는 사람이 많은 장면에서는 움직이는 점을 정지한 점으로 잘못 분류한다. 학습 기반 optical flow나 semantic segmentation으로 이 문제를 피하려는 연구가 이어지고 있다.

---

PTAM의 tracking·mapping 분리로도 풀리지 않은 문제가 있었다. keyframe이 쌓일수록 누적 오차가 커져서, 루프를 돌아 같은 곳에 오면 지도가 크게 어긋났다. 이 문제를 푸는 그래프 최적화의 틀은 PTAM보다 10년 앞선 1997년, 캐나다 York University의 Feng Lu와 Evangelos Milios가 레이저 스캔 정합 문제를 다루면서 이미 내놓았다.
