# Ch.4 — Smith-Cheeseman과 EKF-SLAM의 흥망

사진측량, SfM, bundle adjustment는 촬영을 마친 뒤 모든 이미지를 한꺼번에 오프라인으로 처리할 여유가 있다고 보았다. Hartley-Zisserman의 기하, RANSAC(무작위로 뽑은 표본으로 모델을 세워 이상치를 걸러 내는 방법), Levenberg-Marquardt는 장면을 정밀하게 재는 도구였다. 움직이는 로봇이 *지금* 어디에 있는지 답하는 일은 이 도구들이 겨냥한 문제 밖에 있었다. 로봇이 지도를 만들면서 자기 위치를 추정하는 문제를 확률로 다루기 시작한 것은 1980년대 중반이다.

Durrant-Whyte와 Bailey의 2006년 튜토리얼은 그 출발점을 1986년 샌프란시스코에서 열린 IEEE Robotics and Automation 학회로 잡는다. 지도 작성과 위치 추정에 추정 이론을 쓰려던 Peter Cheeseman, Jim Crowley, Hugh Durrant-Whyte 등이 학회 기간 내내 일관된 지도를 어떻게 만들지 논의했다. 그 뒤 몇 년 사이에 SRI International의 Randall Smith와 Cheeseman, 그리고 Durrant-Whyte가 landmark 사이의 관계와 기하 불확실성을 통계로 다루는 논문을 냈다. Oxford에서 Durrant-Whyte와 그의 박사과정 학생 John Leonard는 이 수학을 로봇이 지도를 만들면서 동시에 자기 위치를 추정하는 문제에 적용했다.

---

## 4.1 불확실한 공간 관계의 수학 — Smith, Self, Cheeseman (1988)

1986년 SRI International의 Randall Smith, Matthew Self, Peter Cheeseman은 로봇이 여러 장소를 거치며 측정값을 쌓을 때 오차가 어떻게 전파되는지를 수식으로 쓰려 했다. 이 연구는 UAI 1986에서 발표됐고, 1988년 논문집에 ["Estimating Uncertain Spatial Relationships in Robotics"](https://arxiv.org/abs/1304.3111)로 실렸다. 질문은 분명했다. 로봇이 A에서 B를 측정하고 B에서 C를 측정했을 때, A에서 C까지의 불확실성은 어떻게 계산할까?

[Kalman 필터](https://doi.org/10.1115/1.3662552)는 1960년에 나와 레이더 추적, 탄도 계산, 위성 궤도 추정에 쓰이고 있었다. Smith, Self, Cheeseman은 Kalman의 공분산 전파 방정식을 공간 변환을 이어 붙이는 연산(composition)에 맞게 다시 썼다. 로봇 pose $\mathbf{x}_r$과 landmark 위치 $\mathbf{m}_i$를 state vector 하나에 담고, 전체의 joint covariance $\mathbf{P}$를 함께 유지한다.

$$\mathbf{x} = [\mathbf{x}_r^\top,\ \mathbf{m}_1^\top,\ \ldots,\ \mathbf{m}_N^\top]^\top$$

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rm} \\ \mathbf{P}_{mr} & \mathbf{P}_{mm} \end{bmatrix}$$

비대각 블록 $\mathbf{P}_{rm}$은 로봇 위치의 불확실성과 landmark 위치의 불확실성이 서로 얼마나 얽혀 있는지 나타낸다. 이 상관관계를 버리지 않고 함께 추적해야 추정이 일관된다. 튜토리얼에 따르면 이 논문은, 로봇이 상대 관측을 쌓아 가면 로봇 위치 추정의 공통 오차 때문에 landmark 추정치들이 모두 서로 상관된다는 것을 보였다. 그러면 지도 전체와 로봇 pose를 한 state에 담아 관측마다 갱신해야 하고, 계산량은 landmark 수의 제곱으로 는다. 계산량이 이렇게 큰 데다 지도 오차가 수렴하는지도 알려지지 않아서, 한동안 연구자들은 landmark 사이의 상관관계를 줄이거나 아예 없애는 근사를 썼다. 1995년 무렵 연구자들은 결합 문제를 하나의 추정 문제로 풀면 실제로 수렴하며, 지우려던 상관관계가 오히려 문제의 핵심이라는 것을 깨달았다. 이 논문의 틀은 그 뒤 EKF-SLAM의 기본 형태가 됐다.

> 🔗 **차용.** Smith-Self-Cheeseman(1988)의 공간관계 수학은 Kalman(1960)의 공분산 전파를 그대로 이어받았다. 움직이는 물체 하나를 추적하던 기법을 로봇과 지도 요소 전체를 함께 추적하는 데 썼다.

---

## 4.2 "SLAM"이라는 이름

Smith-Self-Cheeseman의 1988년 논문에는 "SLAM"이라는 단어가 없다. 1991년 Leonard와 Durrant-Whyte는 IROS에 ["Simultaneous Map Building and Localization for an Autonomous Mobile Robot"](https://doi.org/10.1109/IROS.1991.174711)을 발표했다. 지도 작성과 위치 추정을 따로 떼어 풀 수 없고 동시에(simultaneously) 해야 한다는 생각을 제목에 담은 초기 논문으로 자주 인용된다. 그 무렵 연구자들은 같은 문제를 CML(concurrent mapping and localization) 등 여러 이름으로 불렀다.

Durrant-Whyte와 Bailey의 튜토리얼에 따르면 "SLAM"이라는 약어는 1995년 International Symposium on Robotics Research(ISRR)에서 발표된 이동 로봇 서베이 논문(Durrant-Whyte, Rye, Nebot)에 처음 나왔다. SLAM 문제의 구조와 수렴 결과도 이 논문에서 처음 소개됐다. Durrant-Whyte는 같은 해 Oxford에서 Sydney 대학으로 옮겼고, MIT Sea Grant의 AUV 연구실에서 일하던 Leonard는 이듬해 MIT 교수가 됐다. "Simultaneous Localization and Mapping"을 줄인 이 이름은 그 뒤 10년 사이에 분야 전체가 쓰는 이름이 됐다.

> 🔗 **병행 계보.** [Bar-Shalom의 다중 표적 추적](https://archive.org/details/trackingdataasso0000bars)(multi-target tracking, 1988년 단행본)과 초기 SLAM은 모르는 상태 여러 개의 불확실성과 data association(어느 관측이 어느 대상에서 왔는지 정하는 일)을 함께 다룬다는 점에서 수학이 겹친다. 표적과 추적기의 관계를 landmark와 로봇 pose에 대어 보면 두 문제를 이해하기 쉽다. 그래도 지금 확인되는 인용만으로는 Leonard와 Durrant-Whyte가 이 대응을 직접 가져왔다고 말할 근거가 부족하다.

---

## 4.3 EKF-SLAM의 공식

SLAM에는 Extended Kalman Filter(EKF)가 먼저 쓰였다. EKF는 SLAM 이전부터 비선형 시스템 추정에 쓰이던 필터로, 예측(predict)과 갱신(update) 두 단계를 되풀이한다.

predict 단계에서는 로봇이 움직이면 모션 모델 $f(\cdot)$로 state를 예측하고, Jacobian $\mathbf{F}$로 공분산을 전파한다.

$$\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, \mathbf{u})$$
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$$

update 단계에서는 센서 측정값 $\mathbf{z}$가 오면 관측 모델 $h(\cdot)$의 Jacobian $\mathbf{H}$로 Kalman gain $\mathbf{K}$를 계산해 state와 공분산을 갱신한다.

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top(\mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

EKF-SLAM은 이 두 단계를 반복한다. 구조는 단순하지만 지도가 커지면 계산량이 감당할 수 없이 늘어난다.

원인은 state 차원이다. 6-DoF pose에 3D landmark $N$개를 담으면 state vector 차원은 $6 + 3N$이고, 공분산 행렬의 원소는 $(6+3N)^2$개, 곧 $O(N^2)$개다. 관측 차원이 고정돼 있어도 update 한 번에 전체 공분산을 갱신하는 데 $O(N^2)$ 비용이 든다. Kalman gain은 상태–관측 공분산에 $\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$의 역행렬을 곱해 계산한다. $\mathbf{S}$의 크기는 관측 차원으로 정해지므로 이 역행렬 계산은 작다. 계산량은 공분산 전체를 갱신하는 데서 커진다. landmark가 100개면 공분산은 $306 \times 306$, 약 9.4만 원소이고 1,000개면 $3006 \times 3006$, 약 900만 원소다.

---

## 4.4 지도를 키우기 어렵다

Ch.0에서 본 [Davison의 ICCV 2003 논문](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)은 full-covariance EKF-SLAM의 update 비용이 feature 수 $N$의 제곱에 비례하며, 2.2 GHz Pentium에서 feature가 100개에 가까워지면 update 시간이 빠르게 늘어난다고 적었다. 그보다 넓히려면 따로 발표된 SLAM 근사 기법을 써야 했다.

EKF는 결합 분포 전체를 한 덩어리로 유지하므로, 새 landmark를 더하면 그 landmark와 기존 모든 landmark 사이의 상관관계가 바로 생긴다.

2000년대 중반까지 연구자들이 주로 시도한 해법은 submap이었다. 전체 지도를 서로 겹치는 작은 영역으로 나누고, 영역마다 따로 EKF를 돌린 뒤 영역 사이를 별도의 구조로 잇는다. [Chong과 Kleeman(1999)](http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/integrated1/chong_feature_map.pdf)은 초음파 센서 배열로 큰 실내 환경의 지도를 만들면서 이런 방식을 썼다. 그러나 submap 경계에서 정보가 빠지고, loop closure를 처리하기 어렵고, 구현이 복잡해서 실제 시스템에는 드물게 쓰였다.

> 🔗 **차용.** Chong-Kleeman(1999)의 submap 분할과 현대 SLAM의 local window 최적화는 한 번에 계산하는 범위를 줄인다는 목적이 같다. ORB-SLAM의 local map과 VINS-Mono의 sliding window도 목적이 같다. 다만 지도를 나누는 방식과 상태를 주변화(marginalization)하는 방식이 달라서, 앞의 방법을 직접 이어받았다고 보기는 어렵다.

---

## 4.5 Consistency 문제: Julier-Uhlmann의 반례

EKF-SLAM의 더 근본적인 결함은 2001년 ICRA에서 드러났다. Simon Julier와 Jeffrey Uhlmann은 full-covariance SLAM이 일관되지 않은 지도를 낸다는 것을 보였다. 가장 단순한 경우는 프로세스 잡음 없이 멈춰 있는 로봇이 거리·방위 센서로 landmark를 재는 경우다. 이때도 각도 불확실성이 0이 아니면 지도가 반드시 비일관된다는 것을 증명했다. 움직이는 로봇에도 같은 결론이 적용되는 듯하다는 것은 시뮬레이션으로 보였고, 이 비일관성은 관측 갱신이 수백 번 쌓인 뒤에야 드러난다고 덧붙였다. 그들이 ICRA 2001에 낸 논문 제목은 ["A Counter Example to the Theory of Simultaneous Localization and Map Building"](https://doi.org/10.1109/ROBOT.2001.933280)이었다.

이 논문을 인용하는 후속 문헌은 대개 EKF-SLAM이 시간이 지날수록 *overconfident*해진다고 요약한다. 실제 추정 오차는 커지는데 필터가 계산한 공분산은 실제보다 작은 값으로 수렴한다. 이런 상태를 inconsistency(비일관성)라 한다.

원인은 선형화 오차(linearization error)다. EKF는 비선형 모션 모델과 관측 모델을 1차 Taylor 전개로 근사한다. 이 근사 오차가 단계마다 쌓이면 공분산이 실제 오차보다 작아진다. 필터가 자기 위치를 지나치게 확신하면 그 뒤에 들어오는 측정값을 덜 반영하고, 오차는 고쳐지지 않은 채 쌓인다.

2007년 [Shoudong Huang과 Gamini Dissanayake](https://doi.org/10.1109/TRO.2007.903811)는 이 비일관성의 원인을 더 자세히 분석했다. 이들은 매번 현재 추정치에서 계산한 Jacobian들이 서로 지켜야 할 기본 관계를 어기는 것을 주된 원인으로 들었다. 로봇에서 landmark까지의 상대 위치만 재면서 로봇과 landmark의 절대 위치 추정치를 갱신하는 방식도 원인으로 꼽았다. 극단적인 경우에는 로봇 방향각 추정의 분산이 실제와 달리 0까지 줄어들 수 있다. 그 뒤 나온 observability 분석은 이 논문을 바탕으로, 선형화하는 시점에 따라 관측 가능한 자유도가 달라지고 필터가 관측할 수 없는 방향에 근거 없는 정보를 넣는다고 설명했다.

> 📜 **예언 vs 실제.** Julier와 Uhlmann의 2001년 반례 뒤로 일관된 추정을 하려는 필터 설계가 이어졌다. Unscented Kalman Filter(UKF), Invariant EKF, robust covariance 같은 필터 변형이 10년 가까이 나왔다. 2026년에 돌아보면 최적화 기반 추정도 이 문제를 다루는 주요 방법이 됐다. [iSAM](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)(Kaess et al., TRO 2008), [g2o](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf)(Kümmerle et al., ICRA 2011), GTSAM이 널리 쓰였고, 필터 기반 VIO도 함께 발전했다. 반복 최적화는 남겨 둔 상태를 다시 선형화할 수 있다. 그래도 일관성을 지키려면 관측할 수 없는 방향과 주변화 과정의 선형화를 함께 관리해야 한다.

---

## 4.6 FastSLAM — 문제를 쪼개다

2002년 AAAI에서 Michael Montemerlo, Sebastian Thrun, Daphne Koller, Ben Wegbreit가 발표한 [FastSLAM](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf)은 EKF-SLAM의 $O(N^2)$ 비용을 다른 방식으로 줄였다.

FastSLAM은 Rao-Blackwellization을 썼다. 로봇 경로 $x_{0:t}$를 알면 landmark들의 위치 추정은 *서로 독립*이 된다. 그래서 경로는 particle filter로 표현하고(particle 하나가 가능한 경로 하나), particle마다 landmark별로 작은 EKF를 따로 돌린다.

landmark마다 EKF가 따로 있으니 $N \times N$ 공분산 전체를 유지할 필요가 없다. particle이 $K$개, landmark가 $N$개면 resampling 때 particle마다 landmark 추정을 복사해야 해서 단순하게 짜면 한 단계에 $O(KN)$이 든다. 논문은 landmark 추정을 트리에 담아 바뀐 부분만 새로 만들게 해서 이 비용을 $O(K \log N)$으로 줄였다. 논문은 particle 수를 100개 정도로 고정해도 여러 상황에서 잘 동작한다고 보고했다.

논문의 시뮬레이션에서 FastSLAM은 landmark 5만 개짜리 지도를 particle 100개로 만들었다. 실제 로봇 실험은 NASA 화성 탐사 연구용 작은 실험장에서 SICK 레이저를 단 Pioneer 로봇으로 했고, particle 10개로 만든 지도의 평균 오차는 8.3 cm였다. 그러나 지도가 커지면 particle depletion이 문제가 됐다. 대부분의 particle이 틀린 경로를 나타내게 되고, 실제로 쓸모 있는 표본 수(effective sample size)가 급격히 준다. loop closure 때 경로 가중치를 다시 맞추기도 어렵다. particle 수를 늘려도 넓은 환경에서 drift가 쌓이는 문제는 풀리지 않았다.

[FastSLAM 2.0](https://www.ijcai.org/Proceedings/03/Papers/165.pdf)(Montemerlo et al., IJCAI 2003)이 proposal distribution을 개선했지만 필터로 푸는 한 지도 크기의 한계는 남았다. 이 한계는 그 뒤 그래프 최적화가 넘었다.

---

## 4.7 EKF의 퇴장

2005년 이후 그래프 기반 방법이 실제로 쓸 만해지면서 EKF-SLAM은 주력에서 물러났다. [Feng Lu와 Evangelos Milios가 1997년에 내놓은 그래프 아이디어](https://doi.org/10.1023/A:1008854305733)에 [Olson, Leonard, Teller(2006)](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf)의 빠른 solver가 더해졌고, iSAM(2008)과 iSAM2는 희소 행렬 분해를 증분으로 갱신해, 그래프 방법도 새 관측을 바로바로 반영할 수 있게 했다.

loop closure는 로봇이 전에 왔던 곳에 다시 왔을 때 지도의 오차를 바로잡는 단계다. EKF는 이때 전체 공분산 행렬을 갱신해야 해서 비용이 $O(N^2)$다. 그래프 최적화는 pose graph에 엣지 하나를 더하고 희소 행렬을 다시 분해하므로 비용이 훨씬 낮다.

2010년 무렵부터 새 SLAM 시스템의 backend는 주로 그래프 최적화를 썼다. 필터는 visual-inertial odometry 쪽에서 계속 쓰였다(Ch.7b 참조).

> 📜 **예언 vs 실제.** Bailey와 Durrant-Whyte는 [2006년 IEEE Robotics & Automation Magazine 튜토리얼 2부](https://www.doc.ic.ac.uk/~ajd/Robotics/RoboticsResources/SLAMTutorial2.pdf)에서 계산량을 줄이는 방법으로 갱신 분할, information form의 희소화, submap을 소개했다. Information form은 EKF의 공분산 대신 그 역행렬(information matrix)을 쓰는 형태다. 큰 지도에서는 이 행렬의 비대각 원소가 대부분 0에 가깝다. Thrun 등은 이 원소들을 0으로 만드는 희소화를 내놓았지만, Eustice 등이 이 방법이 일관되지 않다는 것을 보였다. 튜토리얼은 지난 로봇 pose를 모두 state에 남겨 두면 information matrix가 근사 없이 희소해진다는 점도 짚었다. 2010년대에 주로 쓰인 것은 지난 pose를 남겨 두는 factor graph와 반복 최적화였다.

---

## 4.8 🧭 아직 열린 것

**필터와 최적화가 함께 쓰인다.** EKF는 backend 주력에서 물러난 뒤에도 계속 쓰인다. 2026년에도 자율주행 시스템 일부는 필터를 쓴다. 최적화 기반 SLAM은 반복해서 수렴시켜야 하므로 정해진 시간 안에 결과를 낸다고 보장하기 어려울 때가 있다. 값싼 임베디드 시스템에서는 sparse EKF나 UKF를 다시 쓰기도 한다.

**비가우시안 불확실성.** EKF의 가장 근본적인 가정은 불확실성이 가우시안 분포를 따른다는 것이다. 실제 센서 오차는 봉우리가 여럿인(multimodal) 분포이거나 꼬리가 두꺼운(heavy-tail) 분포일 때가 많다. 서로 다른 장소가 같아 보여 위치 가설이 여럿 생기는 perceptual aliasing 상황에서는 가우시안 하나로 불확실성을 제대로 나타낼 수 없다. Particle filter는 이론상 비가우시안 분포를 나타내지만 state 차원이 높으면 쓰기 어렵다. Stein particle, normalizing flow, 학습 기반 uncertainty estimation이 시도되고 있다.

---

Ch.0에서 보았듯 Davison은 2003년에 바로 이 EKF로 카메라 한 대만 써서 실시간 SLAM을 해 보였다.
