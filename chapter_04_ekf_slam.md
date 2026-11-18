# 4장 EKF-SLAM

사진측량과 SfM, bundle adjustment의 세계는 본질적으로 여유가 있었다. 촬영을 모두 마친 뒤 모든 이미지를 책상 위에 펼쳐 놓고 오프라인으로 계산하면 그만이었기 때문이다. Hartley와 Zisserman의 사영기하, RANSAC, Levenberg-Marquardt는 정지된 장면을 정교하게 측량하는 데 더할 나위 없는 도구였지만, 당장 눈앞에서 바퀴를 굴리는 로봇이 *"내가 지금 어디에 있는가"*에 즉답해야 하는 문제와는 결이 달랐다. 로봇이 미지의 공간을 누비며 지도를 그리는 동시에 자기 위치를 추정하는 문제를 확률론의 언어로 풀어내기 시작한 것은 1980년대 중반에 이르러서다.

Durrant-Whyte와 Bailey는 2006년 튜토리얼에서 그 역사적 기점을 1986년 샌프란시스코에서 열린 IEEE Robotics and Automation 학회(ICRA)로 짚는다. 당시 지도 작성과 위치 추정에 통계적 추정 이론을 접목하려던 Peter Cheeseman, Jim Crowley, Hugh Durrant-Whyte 등은 학회 기간 내내 복도와 강연장을 오가며 일관된 확률적 지도를 어떻게 만들 것인가를 두고 열띤 토론을 벌였다. 그 직후 몇 년 사이 SRI International의 Randall Smith와 Cheeseman, 그리고 Durrant-Whyte가 landmark 사이의 기하적 관계와 불확실성을 통계로 정식화한 논문들을 잇달아 내놓았고, Oxford에서는 Durrant-Whyte와 그의 박사과정 학생 John Leonard가 이 수학을 실제 자율 이동 로봇에 얹어 오늘날 우리가 아는 EKF-SLAM의 원형을 빚어냈다.

---

## 4.1 불확실한 공간 관계의 수학 — Smith, Self, Cheeseman (1988)

출발선은 소박하지만 날카로운 질문 하나였다. 로봇이 A 지점에서 B 지점까지의 상대 거리를 재고, 다시 B에서 C까지의 상대 거리를 쟀을 때, A에서 C에 이르는 위치와 그 오차의 크기는 어떻게 계산해야 할까? 1986년 SRI International의 Randall Smith, Matthew Self, Peter Cheeseman은 로봇이 이동하며 쌓은 불확실한 측정값들을 일관되게 합성하는 수학을 세웠고, UAI 1986 발표를 거쳐 1988년 논문집에 기념비적 논문 ["Estimating Uncertain Spatial Relationships in Robotics"](https://arxiv.org/abs/1304.3111)을 실었다.

이들이 꺼내 든 무기는 1960년 세상에 나와 레이더 추적과 탄도 계산, 위성 궤도 추정의 표준으로 자리 잡고 있던 [Kalman 필터](https://doi.org/10.1115/1.3662552)였다. Smith, Self, Cheeseman은 Kalman 필터의 공분산 전파 방정식을 공간 변환의 합성(composition)에 맞게 다시 짰다. 핵심은 로봇의 pose $\mathbf{x}_r$과 지도의 모든 landmark 위치 $\mathbf{m}_i$를 하나의 거대한 state vector로 묶고, 전체의 결합 공분산 행렬 $\mathbf{P}$를 통째로 유지하는 데 있었다.

$$\mathbf{x} = [\mathbf{x}_r^\top,\ \mathbf{m}_1^\top,\ \ldots,\ \mathbf{m}_N^\top]^\top$$

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rm} \\ \mathbf{P}_{mr} & \mathbf{P}_{mm} \end{bmatrix}$$

여기서 결정적인 역할을 하는 것은 대각선 바깥에 자리한 비대각 블록 $\mathbf{P}_{rm}$이다. 이 블록은 로봇 자신의 위치 불확실성과 각 landmark의 위치 불확실성이 서로 얼마나 단단히 얽혀 있는지를 기록한다. 로봇이 자기가 어디 있는지 정확히 모른 채 관측한 landmark들은 필연적으로 로봇 위치 추정의 공통 오차를 나눠 갖게 되므로, 모든 landmark 추정치는 서로 상관될 수밖에 없다. Smith-Self-Cheeseman 논문은 이 상관관계를 버리지 않고 끝까지 함께 추적해야만 지도가 모순 없이 수렴한다는 사실을 꿰뚫어 보았다.

하지만 이 통찰이 곧장 받아들여진 것은 아니다. 모든 landmark를 한 state에 담아 관측마다 갱신하면 계산량이 landmark 수의 제곱($O(N^2)$)으로 불어나는 데다, 당시에는 지도 오차가 정말 수렴하는지조차 수학적으로 증명되지 않았다. 그래서 한동안 연구자들은 landmark 사이의 상관관계를 인위적으로 줄이거나 아예 지워 버리는 근사법에 매달렸다. 그러다 1995년 무렵에 이르러서야 연구자들은 결합 문제를 쪼개지 않고 하나로 풀 때 비로소 추정이 수렴하며, 그동안 귀찮은 짐처럼 여겨 지우려 들었던 상관관계야말로 SLAM 문제의 본질이라는 사실을 깨달았다. 이 깨달음과 함께 Smith-Self-Cheeseman의 정식화는 EKF-SLAM의 표준 교과서가 됐다.

> 🔗 **차용.** Smith-Self-Cheeseman(1988)의 공간 관계 수학은 Kalman(1960)의 공분산 전파 방정식을 그대로 이어받았다. 원래 하늘을 나는 단일 표적을 쫓기 위해 고안된 필터링 수학이, 로봇 자신과 그 주변 환경 전체를 동시에 추적하는 공간 추정 이론으로 탈바꿈한 순간이었다.

---

## 4.2 "SLAM"이라는 이름

흥미롭게도 Smith, Self, Cheeseman의 1988년 논문 어디에도 "SLAM"이라는 단어는 나오지 않는다. 이 문제가 지금의 이름을 얻기까지는 몇 해가 더 필요했다. 1991년 John Leonard와 Hugh Durrant-Whyte는 IROS에 ["Simultaneous Map Building and Localization for an Autonomous Mobile Robot"](https://doi.org/10.1109/IROS.1991.174711)을 발표하며, 지도 작성과 위치 추정을 별개의 모듈로 떼어 놓고는 풀 수 없으며 반드시 *동시에* 다뤄야 한다는 핵심을 제목에 못 박았다. 당시 학계에서는 같은 문제를 CML(concurrent mapping and localization) 등 여러 이름으로 부르며 혼용하고 있었다.

우리가 익히 아는 "SLAM"이라는 약어가 공식 지면에 처음 등장한 것은 Durrant-Whyte와 Bailey의 튜토리얼에 따르면 1995년 International Symposium on Robotics Research(ISRR)에서 발표된 이동 로봇 서베이 논문(Durrant-Whyte, Rye, Nebot)에서다. SLAM 문제의 수학적 구조와 수렴 성질 역시 이 자리에서 처음으로 체계화됐다. 같은 해 Durrant-Whyte는 Oxford를 떠나 Sydney 대학으로 적을 옮겼고, MIT Sea Grant의 AUV 연구실에서 일하던 Leonard는 이듬해 MIT 교수진에 합류했다. 두 대륙을 거점으로 뻗어 나간 "Simultaneous Localization and Mapping"이라는 이름은 그 뒤 십 년 안에 분야 전체를 아우르는 고유명사로 굳어졌다.

> 🔗 **병행 계보.** [Bar-Shalom의 다중 표적 추적](https://archive.org/details/trackingdataasso0000bars)(multi-target tracking, 1988년 단행본)과 초기 SLAM은 미지의 상태 여러 개의 불확실성과 data association(어느 관측이 어느 대상에서 왔는지 가려내는 일)을 함께 푼다는 점에서 수학 구조가 맞닿아 있다. 움직이는 표적과 추적기의 구도를 정지한 landmark와 움직이는 로봇에 대입해 보면 두 문제의 닮은 꼴이 선명히 드러난다. 다만 현재 확인되는 인용 기록만으로는 Leonard와 Durrant-Whyte가 다중 표적 추적의 공식을 직접 끌어왔다고 단정하기는 어렵다.

---

## 4.3 EKF-SLAM의 공식

초창기 SLAM의 주무기는 단연 확장 칼만 필터(EKF)였다. 이미 항공우주와 제어공학에서 비선형 시스템 추정의 표준으로 검증된 EKF는 로봇이 움직일 때 상태를 예측하는 **predict** 단계와, 센서 관측이 들어왔을 때 추정을 바로잡는 **update** 단계를 끊임없이 교대하며 돈다.

먼저 로봇에 제어 입력 $\mathbf{u}$가 들어가면 비선형 모션 모델 $f(\cdot)$로 다음 상태를 예측하고, 그 선형 근사인 Jacobian $\mathbf{F}$로 공분산을 넓힌다.

$$\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, \mathbf{u})$$
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$$

이어 센서가 측정값 $\mathbf{z}$를 내놓으면 관측 모델 $h(\cdot)$의 Jacobian $\mathbf{H}$로 Kalman gain $\mathbf{K}$를 계산해 상태 벡터와 공분산 행렬을 한꺼번에 갱신한다.

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top(\mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

수식은 다섯 줄로 깔끔하게 떨어지지만, 이 단순함 뒤에는 가혹한 계산 비용이 숨어 있었다. 원인은 상태 벡터의 차원이다. 6-DoF 로봇 pose에 3D landmark $N$개를 얹으면 상태 벡터 길이는 $6 + 3N$이 되고, 공분산 행렬 $\mathbf{P}$의 원소 수는 $(6 + 3N)^2$개, 즉 $O(N^2)$으로 불어난다. 센서 관측의 차원이 아무리 작아도(그래서 $\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$의 역행렬 계산이 아무리 가벼워도), update 한 번이 끝날 때마다 행렬 $\mathbf{P}$ 전체를 훑어 고쳐야 하므로 계산량은 무조건 $O(N^2)$을 피할 수 없다. landmark가 100개일 때 $306 \times 306$(약 9.4만 원소)이던 행렬은 landmark가 1,000개로 늘면 단숨에 $3006 \times 3006$(약 900만 원소)짜리 거구로 비대해진다.

---

## 4.4 지도를 키우기 어렵다

이 이차적 증가의 벽은 곧 실전 시스템의 발목을 잡았다. [Davison의 ICCV 2003 논문](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)은 full-covariance EKF-SLAM의 갱신 비용이 특징점 수 $N$의 제곱에 비례해 치솟기 때문에, 당시 2.2 GHz Pentium PC에서도 특징점이 100개 근처에 다다르면 갱신 시간이 급격히 늘어나 실시간 구동이 버거워진다고 솔직히 털어놓았다. 그 이상의 지도를 다루려면 필터의 원형을 훼손하는 근사 기법에 기댈 수밖에 없었다.

EKF는 로봇과 모든 landmark의 결합 분포를 단 하나의 덩어리로 붙들고 있기에, 새 landmark 하나가 지도에 들어오는 순간 기존 모든 landmark와의 상관관계가 연쇄적으로 살아난다. 2000년대 중반까지 연구자들이 이 병목을 뚫으려고 주로 매달린 해법은 **submap** 분할이었다. 거대한 환경을 서로 조금씩 겹치는 작은 구역들로 쪼갠 뒤 구역마다 독립된 작은 EKF를 돌리고, 구역 사이의 연결만 별도 그래프로 관리하는 방식이다. [Chong과 Kleeman(1999)](http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/integrated1/chong_feature_map.pdf)이 초음파 센서 배열로 넓은 실내 지도를 그리며 이 전략을 선보였다. 하지만 submap의 경계를 넘나들 때마다 정보가 누실되고 loop closure를 매끄럽게 처리하기가 까다로운 데다 구현마저 복잡해서, 실제 현장 시스템에 널리 뿌리내리지는 못했다.

> 🔗 **차용.** Chong과 Kleeman(1999)의 submap 분할과 현대 SLAM의 local window 최적화는 한 번의 계산에 올리는 변수 규모를 통제한다는 문제의식을 공유한다. ORB-SLAM의 local map이나 VINS-Mono의 sliding window 역시 같은 고민의 산물이다. 다만 지도를 쪼개는 기준과 과거 상태를 주변화(marginalization)하는 수학적 절차가 판이해, 앞의 submap 기법을 직접 계승했다고 보기는 어렵다.

---

## 4.5 Consistency 문제: Julier-Uhlmann의 반례

계산량보다 더 뼈아픈 결함은 2001년 ICRA에서 터져 나왔다. Simon Julier와 Jeffrey Uhlmann은 논문 ["A Counter Example to the Theory of Simultaneous Localization and Map Building"](https://doi.org/10.1109/ROBOT.2001.933280)에서, 계산량을 전혀 아끼지 않은 이상적인 full-covariance EKF-SLAM조차 수학적으로 일관되지 않은(inconsistent) 지도를 만들어 낸다는 사실을 증명했다. 이들이 든 가장 단순한 반례는 충격적이었다. 프로세스 잡음 없이 제자리에 꼼짝 않고 서 있는 로봇이 거리·방위 센서로 landmark를 반복 관측하는 상황에서도, 각도 불확실성이 0이 아닌 한 지도는 *반드시* 일관성을 잃는다는 것이다. 두 사람은 움직이는 로봇에서도 똑같은 현상이 벌어지며, 이 비일관성이 관측 갱신이 수백 번 누적된 뒤에야 수면 위로 드러난다고 경고했다.

후속 문헌들이 이 논문을 인용하며 즐겨 쓰는 표현은 EKF-SLAM이 시간이 흐를수록 **과신(overconfident)**에 빠진다는 것이다. 실제 로봇과 지도의 오차는 점점 커지는데도 필터가 스스로 계산해 낸 공분산 행렬은 실제 오차보다 훨씬 작은 값으로 쪼그라든다.

원인은 EKF가 품고 있는 1차 Taylor 근사, 즉 **선형화 오차(linearization error)**에 있었다. 매 스텝 현재의 불완전한 추정치 근처에서 모션 모델과 관측 모델을 직선으로 잘라 근사하다 보니, 그 미세한 근사 오차가 갱신을 거듭하며 차곡차곡 쌓여 공분산을 실제보다 좁게 오그라뜨린다. 한번 자기 위치를 지나치게 확신하게 된 필터는 그다음부터 들어오는 새 센서 측정값을 불신해 거의 반영하지 않고, 결국 틀린 추정치에 스스로를 가둬 버린다.

2007년 [Shoudong Huang과 Gamini Dissanayake](https://doi.org/10.1109/TRO.2007.903811)는 이 병리의 해부도를 한층 정밀하게 그려 냈다. 이들은 매 단계 현재 추정값에서 새로 계산한 Jacobian들이 원래 시스템이 지켜야 할 기하적 항등 관계를 어기는 데 주목했다. 로봇이 landmark까지의 상대 위치만 재면서 로봇과 landmark의 절대 좌표를 동시에 갱신하는 구조에서는, 극단적인 경우 로봇 방향각 추정의 분산이 물리적으로 불가능한 0까지 수직 낙하하기도 했다. 이후 이어진 observability(가관측성) 분석 연구들은 이 논문을 발판 삼아, 선형화 기준점이 프레임마다 흔들리면 원래 관측 불가능한 전역 방향(yaw) 자유도가 갑자기 관측 가능한 것처럼 둔갑해 필터가 근거 없는 정보를 주입한다는 사실까지 밝혀냈다.

> 📜 **예언 vs 실제.** Julier와 Uhlmann의 2001년 반례 이후 약 10년간 연구자들은 필터 자체를 고쳐 일관성을 되살리려 애썼다. Unscented Kalman Filter(UKF), Invariant EKF, robust covariance 같은 정교한 변형들이 줄지어 등장했다. 하지만 2026년의 시점에서 되돌아보면 이 문제의 주류 해법은 필터 밖, 즉 반복 최적화 기반 추정에서 나왔다. [iSAM](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)(Kaess et al., TRO 2008), [g2o](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf)(Kümmerle et al., ICRA 2011), GTSAM 같은 그래프 최적화 도구들이 지배적 위치를 차지했고, 필터 기반 VIO 역시 선형화 지점을 고정하는 기법 등을 얹어 나란히 발전했다. 반복 최적화는 과거 상태를 버리지 않으므로 언제든 더 정확한 해 근처에서 다시 선형화할 수 있다. 물론 최적화 기반이라도 상태를 주변화(marginalization)할 때의 선형화와 가관측성 관리는 여전히 세심하게 다뤄야 하는 과제로 남아 있다.

---

## 4.6 FastSLAM — 문제를 쪼개다

$O(N^2)$의 벽을 넘으려는 또 하나의 독창적인 시도는 2002년 AAAI에서 나왔다. Michael Montemerlo, Sebastian Thrun, Daphne Koller, Ben Wegbreit가 발표한 [FastSLAM](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf)은 베이즈 네트워크의 조건부 독립성, 즉 **Rao-Blackwellization**을 정면으로 활용했다. 핵심 통찰은 놀라울 만큼 명쾌했다. 로봇이 지나온 *전체 경로* $x_{0:t}$를 알고 있다고 가정하면, 각 landmark의 위치를 추정하는 문제는 놀랍게도 *서로 완전히 독립*이 된다는 사실이다.

FastSLAM은 이 성질을 이용해 문제를 둘로 갈랐다. 로봇의 경로는 particle filter로 추적하고(각 particle이 하나의 가능한 경로 가설을 품는다), 각각의 particle 안에서는 $N$개의 landmark마다 아주 작은 $3\times 3$(2D라면 $2\times 2$) EKF를 따로따로 돌렸다. landmark들이 서로 얽혀 거대한 $N \times N$ 공분산 행렬을 이룰 이유가 사라진 것이다.

물론 particle이 $K$개, landmark가 $N$개일 때 particle을 resampling할 때마다 각 particle이 거느린 landmark 지도 전체를 복사하면 한 스텝에 $O(KN)$ 비용이 들 수 있었다. 연구진은 landmark 추정치들을 이진 트리 자료구조에 담아 resampling 시 변경된 경로의 가지에만 새 노드를 할당함으로써 스텝당 비용을 $O(K \log N)$까지 극적으로 낮췄다. 게다가 particle 수를 100개 안팎으로 고정해도 여러 환경에서 거뜬히 동작함을 보였다.

시뮬레이션에서 FastSLAM은 particle 100개만으로 landmark 5만 개짜리 지도를 거뜬히 소화했고, NASA 화성 탐사 모의 실험장에서 SICK 레이저를 단 Pioneer 로봇으로 수행한 실측 실험에서는 particle 10개만으로 평균 오차 8.3 cm라는 인상적인 기록을 남겼다. 그러나 환경이 더 넓어지고 루프가 길어지자 particle filter 특유의 **particle depletion(입자 고갈)**이라는 복병이 고개를 들었다. resampling을 거듭할수록 다양했던 경로 가설들이 빠르게 사라져 결국 대다수 particle이 미세하게 틀린 경로 하나를 복제하게 되고, 실제로 유효한 표본 수(effective sample size)가 바닥나면서 막상 loop closure 순간에 경로 가중치를 재조정할 여력이 남지 않았던 것이다. [FastSLAM 2.0](https://www.ijcai.org/Proceedings/03/Papers/165.pdf)(Montemerlo et al., IJCAI 2003)이 최신 관측까지 반영한 proposal distribution으로 한계를 밀어붙였지만, 필터라는 틀 안에서 거대 환경의 drift를 막는 데는 구조적 한계가 분명했다.

---

## 4.7 EKF의 퇴장

결국 EKF-SLAM을 주류의 자리에서 내려오게 만든 것은 2005년 이후 실전 성능을 증명한 그래프 기반 최적화였다. 1997년 [Feng Lu와 Evangelos Milios가 던진 포즈 그래프의 아이디어](https://doi.org/10.1023/A:1008854305733) 위에 [Olson, Leonard, Teller(2006)](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf)의 빠른 해법이 얹혔고, 뒤이어 등장한 iSAM(2008)과 iSAM2는 희소 행렬 분해를 증분 방식으로 갱신해 그래프 최적화에서도 실시간 응답이 가능함을 입증했다.

결정적 승부처는 loop closure였다. 로봇이 한 바퀴를 돌아 출발점으로 되돌아왔을 때, EKF는 거대한 전체 공분산 행렬을 $O(N^2)$ 비용으로 다시 뜯어고쳐야 했을 뿐 아니라 이미 쌓인 선형화 오차 때문에 과거 궤적을 깔끔히 펴내지도 못했다. 반면 포즈 그래프는 그래프에 loop closure 엣지 하나를 추가하고 희소 행렬을 다시 푸는 것만으로 전체 궤적을 훨씬 가볍고 정확하게 바로잡았다. 2010년 무렵을 지나며 새로운 SLAM 시스템의 backend는 급속히 그래프 최적화로 재편됐고, 필터 계열은 상태 크기를 작게 묶어 둘 수 있는 visual-inertial odometry(VIO) 영역에서 제2의 전성기를 맞았다(→ 9장).

> 📜 **예언 vs 실제.** Bailey와 Durrant-Whyte는 [2006년 *IEEE Robotics & Automation Magazine* 튜토리얼 2부](https://www.doc.ic.ac.uk/~ajd/Robotics/RoboticsResources/SLAMTutorial2.pdf)에서 EKF의 계산 병목을 돌파할 대안으로 갱신 분할, information form(정보 행렬)의 희소화, 그리고 submap을 꼽았다. 공분산의 역행렬인 information matrix는 큰 지도에서 비대각 성분이 대부분 0에 가까워 보였기에, Thrun 등은 작은 항을 0으로 잘라 내는 근사 희소화를 시도했다. 그러나 곧 Eustice 등이 이 근사 역시 비일관성을 부른다는 점을 지적했다. 같은 튜토리얼은 지나간 로봇 pose를 모두 state에 남겨 두면 information matrix가 어떤 근사 없이도 본래부터 희소해진다는 중요한 사실을 함께 짚었는데, 2010년대 SLAM의 표준으로 살아남은 것은 바로 이 "과거 pose를 남기는 factor graph와 반복 최적화"였다.

---

## 아직 열린 문제

**필터와 최적화가 함께 쓰인다.** EKF는 대규모 지도 작성의 주력에서는 물러났지만 완전히 사라진 것은 아니다. 2026년 현재에도 일부 자율주행 스택과 저전력 임베디드 플랫폼은 여전히 필터를 고수한다. 반복 최적화 기반 SLAM은 수렴 조건에 따라 매 프레임 실행 시간이 들쭉날쭉해 엄격한 실시간 마감(hard real-time)을 보장하기 어려운 경우가 있는 반면, sparse EKF나 UKF는 연산량이 예측 가능하고 가볍기 때문이다.

**비가우시안 불확실성.** EKF의 가장 뿌리 깊은 가정은 모든 불확실성이 단봉(unimodal) 가우시안 분포를 따른다는 것이다. 그러나 현실의 센서 오차는 꼬리가 두껍거나(heavy-tail) 여러 개의 봉우리가 솟은(multimodal) 형태를 띠는 일이 잦다. 특히 닮은 복도가 반복돼 로봇이 "여긴지 저긴지" 여러 위치 가설을 동시에 품어야 하는 perceptual aliasing 상황에서는 단일 가우시안이 무력하다. 이론상 비가우시안 분포를 다룰 수 있는 particle filter는 차원이 조금만 높아져도 계산이 폭발한다. 이 간극을 메우기 위해 Stein particle, normalizing flow, 학습 기반 불확실성 추정 같은 새로운 시도들이 계속되고 있다.

---

그래프 최적화가 EKF를 밀어내기 직전인 2003년, 바로 이 고전적인 EKF를 들고나와 카메라 단 한 대만으로 실시간 3D SLAM을 성공시켜 세상을 놀라게 한 연구자가 있었다. Andrew Davison의 MonoSLAM과 그 뒤를 이어 단안 SLAM의 설계 문법을 새로 쓴 PTAM의 이야기다(→ 5장).
