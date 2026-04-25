# Ch.4 — Smith-Cheeseman과 EKF-SLAM의 흥망

1부에서 다룬 photogrammetry·SfM·bundle adjustment는 한 가지를 전제했다. 카메라는 정지해 있거나, 촬영 후 오프라인으로 모든 이미지를 한꺼번에 처리할 여유가 있다는 것. Hartley-Zisserman의 기하학, RANSAC의 강건 추정, Levenberg-Marquardt의 반복 최적화 — 이 도구들은 세상을 측정하는 법을 알았지만, 움직이는 로봇이 *지금 이 순간* 어디에 있는지는 묻지 않았다. 2부는 그 질문에서 시작한다. 지도를 만들면서 동시에 자신의 위치를 아는 것, 불확실성이 쌓이는 와중에도 추정을 포기하지 않는 것. 확률론적 지도의 문제가 열린 것은 SRI International의 작은 메모에서였다.

Randall Smith와 Peter Cheeseman은 1986년 로봇이 공간 속에서 무언가를 측정할 때 그 측정값이 얼마나 불확실한지를 수학적으로 다루려 했다. SRI International에서 나온 그들의 아이디어는 Kalman(1960)의 필터 수학을 이어받되, 단일 상태 추정이 아닌 *공간관계의 네트워크* 전체에 불확실성을 전파하는 방향으로 확장했다. 그로부터 수년 뒤 Sydney에서 Hugh Durrant-Whyte가, MIT에서 John Leonard가 이 수학에 "로봇이 지도를 만들면서 동시에 자신의 위치를 추정한다"는 문제 정식을 결합했다. "SLAM"이라는 약어는 그 접합의 산물이다.

---

## 4.1 불확실 공간관계의 수학 — Smith, Self, Cheeseman (1988)

1986년 SRI International의 Randall Smith와 Peter Cheeseman은 로봇이 여러 장소를 거쳐 측정값을 누적할 때 오차가 어떻게 전파되는지를 수식으로 잡으려 했다. 그 작업 노트가 1988년 논문 ["Estimating Uncertain Spatial Relationships in Robotics"](https://arxiv.org/abs/1304.3111)으로 나왔다. 질문 자체는 명료했다. 로봇이 A에서 B를 측정하고 B에서 C를 측정했을 때, A에서 C까지의 불확실성은 어떻게 계산되는가?

[Kalman 필터](https://www.cs.unc.edu/~welch/kalman/kalmanPaper.html)는 이미 있었다. 레이더 추적, 탄도 계산, 위성 궤도 보정에 1960년부터 쓰였다. Smith와 Cheeseman이 한 일은 Kalman의 공분산 전파 방정식을 공간 변환의 합성(composition)에 맞게 재공식화한 것이다. 로봇 pose $\mathbf{x}_r$과 landmark 위치 $\mathbf{m}_i$를 하나의 state vector에 담고, 그 전체의 joint covariance $\mathbf{P}$를 유지한다.

$$\mathbf{x} = [\mathbf{x}_r^\top,\ \mathbf{m}_1^\top,\ \ldots,\ \mathbf{m}_N^\top]^\top$$

$$\mathbf{P} = \begin{bmatrix} \mathbf{P}_{rr} & \mathbf{P}_{rm} \\ \mathbf{P}_{mr} & \mathbf{P}_{mm} \end{bmatrix}$$

off-diagonal 블록 $\mathbf{P}_{rm}$이 핵심이었다. 로봇 위치 불확실성과 landmark 위치 불확실성이 *상관되어* 있다는 것, 그 상관관계를 추적해야 일관된 추정이 가능하다는 것. 논문은 이것을 명시적으로 증명했고, SLAM 분야 전체가 이 출발점에 섰다.

> 🔗 **차용.** Smith-Cheeseman(1988)의 공간관계 수학은 Kalman(1960)의 공분산 전파를 직접 계승한다. 단일 이동 물체를 추적하던 기법이 로봇과 지도 요소 전체를 동시에 추적하는 틀로 바뀌었다.

---

## 4.2 "SLAM"이라는 이름의 정착

Smith-Cheeseman의 1988년 논문에는 "SLAM"이라는 단어가 없다. Oxford에서 Sydney로 옮긴 Hugh Durrant-Whyte와 MIT의 John Leonard가 1990년대 초 각자의 연구실에서 같은 문제를 다른 이름으로 부르고 있었다. 두 그룹이 서로를 인용하기 시작하면서 공통 용어가 필요해졌고, "SLAM"은 그렇게 수렴해 굳었다. 정확히 어느 문서에서 처음 쓰였는지는 연구자마다 기억이 다르다. 공식 선점 논문은 없다.

Leonard와 Durrant-Whyte의 1991년 논문 ["Simultaneous Map Building and Localization for an Autonomous Mobile Robot"](https://doi.org/10.1109/IROS.1991.174711)이 이 문제를 로봇공학 메인스트림에서 제목으로 명시한 초기 사례로 자주 인용된다. "Mapping"과 "Localization"이 분리 불가능하게 얽혀 있다는 것, 그것을 동시에(simultaneously) 해야 한다는 것, 이 직관이 약어 이전에 있었다.

"Simultaneous Localization and Mapping", 줄여서 SLAM. 이후 10년간 이 이름이 분야 전체를 수렴시키는 구심이 된다.

> 🔗 **차용.** [Bar-Shalom의 다중 표적 추적](https://archive.org/details/trackingdataasso0000bars)(multi-target tracking, 1988년 단행본으로 정리됨)은 여러 물체의 state를 동시에 추정하는 프레임워크를 제공했다. Leonard와 Durrant-Whyte는 이 프레임워크에서 "표적 위치"를 "landmark 위치"로, "추적기 위치"를 "로봇 pose"로 대응시켰다고 볼 수 있다. 레이더 기술이 로봇 실내 매핑으로 번역된 사례다.

---

## 4.3 EKF-SLAM의 공식

Extended Kalman Filter(EKF)가 SLAM에 적용된 것은 선택이라기보다 자연스러운 수렴이었다. 1988년 이전부터 비선형 시스템 추정에 사용되던 EKF는 predict-update 두 단계로 작동한다.

predict 단계: 로봇이 움직이면 모션 모델 $f(\cdot)$로 state를 예측하고, Jacobian $\mathbf{F}$로 공분산을 전파.

$$\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}, \mathbf{u})$$
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$$

update 단계: 센서 측정값 $\mathbf{z}$가 오면 관측 모델 $h(\cdot)$의 Jacobian $\mathbf{H}$로 Kalman gain $\mathbf{K}$를 계산해 state와 공분산을 갱신.

$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top(\mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R})^{-1}$$
$$\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - h(\hat{\mathbf{x}}^-))$$
$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

이 두 단계의 반복이 EKF-SLAM의 전부다. 구조는 단순하지만 그 단순함이 처음부터 확장성의 천장을 안고 있었다.

문제는 state 차원이다. 6DOF pose에 3D landmark $N$개를 담으면 state vector 차원은 $6 + 3N$, 공분산 행렬은 $(6+3N)^2$ 원소의 $O(N^2)$ 구조. update 한 번에 Kalman gain 계산($\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$의 역행렬)과 공분산 갱신 모두 $O(N^2)$ 비용이다. landmark 100개면 $306 \times 306 \approx 9.4$만 원소, 1,000개면 $3006 \times 3006 \approx 900$만 원소. 2000년대 초 일반 PC로 실시간을 유지할 수 있는 landmark 수는 수십에서 백 단위가 한계였다.

[Andrew Davison의 MonoSLAM(2003)](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf)이 실시간 시연에서 landmark 수십 개 수준에 갇힌 것은 우연이 아니었다. EKF-SLAM의 $O(N^2)$ 벽이 그 숫자를 결정했다.

---

## 4.4 확장성의 벽

2003년 ICCV에서 Davison이 웹캠 하나로 실시간 3D 추적을 시연했을 때, 수십 개 수준의 feature로 책상 하나 크기의 공간을 매핑했다. 당시 상업용 SLAM 시스템이 없던 환경에서 실시간 단안 추적은 드문 시연이었다. 문제는 그 한계가 알고리즘의 한계가 아니라 공분산 행렬의 크기에서 온다는 점이었다.

100 landmarks에서 covariance 행렬은 $306 \times 306$ (6DOF pose + 3D landmark 100개 기준, state 차원 $6 + 3 \times 100 = 306$). 1,000개면 $3006 \times 3006$. 매 시간 단계마다 이것을 역행렬 연산과 함께 갱신해야 한다. 더불어 EKF는 joint 분포 전체를 한 덩어리로 유지하기 때문에, 새 landmark가 추가되면 기존 모든 landmark와의 cross-correlation이 즉시 생성된다. 지도가 커질수록 update 비용이 기하급수적으로 증가한다.

2000년대 중반까지 시도된 해법은 submap이었다. 전체 지도를 겹쳐지는 소영역으로 나누고, 각 submap에서만 EKF를 돌린 뒤 submap 사이를 별도 연결 구조로 잇는다. [Chong과 Kleeman(1999)](http://www.cs.cmu.edu/afs/cs/Web/People/motionplanning/papers/sbp_papers/integrated1/chong_feature_map.pdf)이 초기 형태를 제안했다. 그러나 submap 경계에서의 정보 손실과 루프 클로저의 어려움, 그리고 구현 복잡도가 submap 접근을 실용화하는 데 마찰을 일으켰다.

> 🔗 **차용.** Chong-Kleeman(1999)의 submap 분할 아이디어는 이후 현대 SLAM의 local window 최적화로 계승된다. ORB-SLAM의 local map, VINS-Mono의 sliding window가 conceptually 같은 원리 위에 있다. 단지 구현 도구가 EKF에서 bundle adjustment로 바뀌었을 뿐이다.

---

## 4.5 Consistency 문제: Julier-Uhlmann의 반례

EKF-SLAM의 더 깊은 결함은 2001년 ICRA에서 터졌다. Simon Julier와 Jeffrey Uhlmann이 EKF 기반 SLAM의 거동을 수치 실험으로 분석하며 필터가 자기 자신을 너무 믿는다는 것을 보였다. 그들이 IEEE ICRA에 낸 논문 제목은 ["A Counter Example to the Theory of Simultaneous Localization and Map Building"](https://doi.org/10.1109/ROBOT.2001.933257)이었다. 도발적이었고, 내용도 그랬다.

2차 문헌들이 이 논문을 인용하며 요약하는 핵심은, EKF-SLAM이 asymptotically *overconfident*하다는 것이다. 즉, 실제 추정 오류는 커지는데 필터가 계산하는 공분산(불확실성)은 실제보다 작게 수렴한다. 이것이 inconsistency다.

원인은 linearization error에 있다. EKF는 비선형 모션 모델과 관측 모델을 일차 Taylor 전개로 근사한다. 이 근사 오류가 매 단계 누적되면 공분산이 실제 오류를 과소 평가하기 시작한다. 로봇이 "나는 여기 있다"고 과도하게 확신하면, 이후 measurements를 필터가 덜 신뢰하게 되어 오류가 교정되지 않고 쌓인다.

2007년 [Shoudong Huang과 Gamini Dissanayake](https://doi.org/10.1109/TRO.2007.903811)는 이 inconsistency의 원인을 더 정밀하게 해부했다. 논문의 핵심 진단은 두 가지였다. 현재 상태 추정치에서 평가된 Jacobian들 사이의 기본 제약(constraint)이 무너지는 것이 EKF-SLAM 비일관성의 주된 원인이고, 그 결과 로봇 방향각(yaw)의 분산이 실제로는 유지되어야 하는데도 잘못 0으로 수렴할 수 있다는 것이었다. 선형화 시점에 따라 시스템의 관측 가능한 자유도가 달라지고, 관측 불가능한 방향에 필터가 임의의 정보를 주입하게 된다는 이후 observability 기반 계열의 해석은 이 논문에서 출발한다.

> 📜 **예언 vs 실제.** Julier와 Uhlmann의 2001년 반례 이후, consistent estimation을 달성하려는 필터 설계 시도가 이어졌다. Unscented Kalman Filter(UKF), Invariant EKF, robust covariance 등 필터 계열의 변형들이 10년 가까이 제안됐다. 그러나 2026년 시점에서 되돌아보면 이 문제의 실용적 해법은 *필터가 아닌 최적화*였다. [iSAM](https://www.cs.cmu.edu/~kaess/pub/Kaess08tro.pdf)(Kaess et al., 2008), [g2o](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf)(Kümmerle et al., 2011), GTSAM이 filter를 사실상 대체했다. Jacobian linearization을 current estimate에 고정하지 않고 반복 최적화로 갱신하는 방식은 inconsistency를 구조적으로 회피한다. 반례가 요구한 "새 필터"의 자리를 결국 필터가 아닌 구조가 채웠다. `[무산]`

---

## 4.6 FastSLAM — 분할통치

EKF-SLAM의 $O(N^2)$ 벽을 다른 방식으로 공격한 것이 [FastSLAM](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf)이다. Michael Montemerlo, Sebastian Thrun(Stanford), Daphne Koller, Ben Wegbreit가 2002년 AAAI에서 발표했다.

핵심 관찰은 Rao-Blackwellization이다. 로봇 경로 $x_{0:t}$가 주어지면 각 landmark의 위치 추정이 *서로 독립*이 된다. 따라서 경로를 particle filter로 표현하고(각 particle이 하나의 가능한 경로를 대표), 각 particle마다 별도의 landmark EKF를 독립적으로 운용할 수 있다.

particle $K$개, landmark $N$개면 per-step 복잡도는 $O(K \log N)$으로, EKF-SLAM의 $O(N^2)$와 달리 $N$에 대해 준선형(sublinear)으로 증가한다(KD-tree 기반 landmark 탐색 사용 시). landmark 수가 많아져도 per-particle EKF는 서로 독립이라 $N \times N$ 전체 공분산을 유지할 필요가 없다. $K$는 수십~수백 수준으로 고정되므로 실질적 이득이 컸다.

FastSLAM은 작동했다. 실내 환경에서 수백 개 landmark까지 실시간을 유지했고, 기술 이전도 빨랐다. 그러나 문제들이 쌓였다. particle depletion: 지도가 커지면 대부분의 particle이 불량 경로를 대표하게 되고, effective sample 수가 급감한다. 루프 클로저 상황에서 경로 가중치 재조정이 어렵다. 무엇보다 particle 수를 늘려도 large-scale 환경에서 드리프트가 축적되는 문제는 해결되지 않았다.

[FastSLAM 2.0](https://www.ijcai.org/Proceedings/03/Papers/165.pdf)(Montemerlo et al. 2003)이 proposal distribution을 개선했지만, 방법론이 filter 패러다임 안에 갇혀 있는 한 확장성의 천장이 있었다. 그 천장을 결국 피해 간 방법은 filter 계열이 아니었다.

---

## 4.7 EKF의 퇴장

그래프 기반 접근이 2005년 이후 빠르게 현실화되면서 EKF-SLAM은 주력에서 물러났다. [Feng Lu와 Evangelos Milios의 1997년 그래프 아이디어](https://doi.org/10.1023/A:1008854305733)가 [Olson-Leonard-Teller(2006)](https://april.eecs.umich.edu/pdfs/olson2006icra.pdf)의 efficient solver와, 이후 g2o·GTSAM·iSAM2의 실시간 인수분해 기법과 결합하자, EKF의 장점이었던 "incremental update"는 더 이상 차별점이 아니었다.

루프 클로저에서 차이가 드러났다. 로봇이 출발점으로 돌아왔을 때 지도 오류를 수정하는 것. EKF는 이 순간 전체 공분산 행렬을 업데이트해야 한다. 비용이 $O(N^2)$. 그래프 최적화는 pose 그래프에 새 엣지 하나를 추가하고 sparse 행렬을 재분해한다. Sparse 구조를 쓰면 비용이 훨씬 낮다.

2010년경을 기점으로 새로운 SLAM 시스템에서 backend로 EKF를 선택하는 경우는 드물어졌다. 특수 제약(매우 제한된 연산 자원, real-time filter 요구)이 있는 경우에만 잔존했다.

> 📜 **예언 vs 실제.** Durrant-Whyte와 Bailey의 [2006년 IEEE Robotics & Automation Magazine 튜토리얼](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf)은 SLAM의 확장성 문제를 논하며 submap 분해와 information filter가 대규모 환경에서의 해법이 될 것으로 전망했다. Information filter(EKF의 역공분산 형태)는 sparse information matrix를 이용해 landmark가 늘어도 연산이 느려지지 않을 것으로 기대됐다. 실제 전개는 달랐다. Information filter 계열(SEIF 등)은 sparsity를 강제로 유지하는 과정에서 marginalization error가 생겼다. Submap은 일부 시스템에 흡수되었으나 주류 해법이 되지 못했다. 2010년대를 지배한 것은 factor graph + iterative 최적화였다. `[기술변화]`

---

## 4.8 🧭 아직 열린 것

Filter vs Optimization의 공존. EKF가 backend 주력에서 물러났다고 해서 사라진 것은 아니다. 2026년 기준으로 자율주행 일부 구현은 여전히 필터 기반을 선호한다. 최적화 기반 SLAM은 반복 수렴이 필요하고, 실시간 보장이 어려운 경우가 있다. 저비용 임베디드 시스템에서 sparse EKF나 UKF가 재등장하는 사례가 있다. "필터는 죽었다"는 선언은 정확하지 않다. 용도와 제약에 따라 공존한다.

비가우시안 불확실성. EKF의 가장 근본적인 가정은 불확실성이 가우시안 분포를 따른다는 것이다. 현실의 센서 오류는 다중 모드(멀티모달)이거나 heavy-tail 분포를 갖는 경우가 많다. 특히 대칭성이 없는 perceptual aliasing(서로 다른 장소가 같아 보이는 것) 상황에서 단일 가우시안은 실제 불확실성을 심각하게 단순화한다. Particle filter는 이론상 비가우시안을 표현하지만 고차원 state에서는 비실용적이다. Stein particle, normalizing flow, 학습 기반 uncertainty estimation이 시도되고 있으나, 2026년 기준으로 이것이 실시간 SLAM에서 검증된 형태는 제한적이다.

---

EKF-SLAM이 100 landmark 수준의 실시간 천장에 부딪히는 동안, Imperial College의 Andrew Davison은 그 한정된 숫자로 다른 무언가를 증명하고 있었다. 카메라 한 대만 들고 별도 센서 없이 실시간으로. 숫자의 한계는 그대로였지만 그것을 다루는 방식이 달라졌다.
