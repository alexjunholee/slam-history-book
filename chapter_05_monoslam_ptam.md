# 5장 단안 실시간 SLAM: MonoSLAM과 PTAM

4장에서 보았듯 EKF-SLAM은 로봇 상태와 불확실성을 함께 추적하는 명쾌한 틀이었지만, landmark 수 $N$에 대해 공분산 행렬이 $O(N^2)$으로 부풀어 오르는 한계 때문에 지도를 크게 키우기 어려웠다. 2000년대 중반, 같은 Oxford 대학의 울타리 안에서 Andrew Davison과 Georg Klein 두 연구자는 바로 이 벽을 마주하고 단안 카메라 실시간 SLAM을 정반대의 해법으로 풀어냈다.

2003년 Oxford의 Davison은 손에 쥔 웹캠 한 대만으로 카메라의 6자유도 궤적과 주변 지도를 데스크톱 PC 위에서 실시간으로 그려 보였다. Smith-Self-Cheeseman의 확률적 공간 관계 수학과 Leonard·Durrant-Whyte의 EKF-SLAM 골격을 그대로 가져왔지만, 무거운 레이저 스캐너도, 스테레오 카메라도, IMU도 없이 오직 Shi-Tomasi 코너 검출기와 Kalman 필터의 예측-갱신 루프만으로 이룬 성과였다. 그리고 불과 4년 뒤인 2007년, 같은 Oxford 연구실에서 Georg Klein과 David Murray는 필터 자체를 과감히 내려놓고 tracking과 mapping을 두 개의 독립 스레드로 쪼갠 **PTAM**을 내놓았다. 이후 10년 넘게 쏟아진 Visual SLAM 시스템의 아키텍처는 대부분 Klein과 Murray가 그날 그린 청사진을 따랐다.

---

## 5.1 2003년의 데모

Davison이 ICCV 2003에서 [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654)를 발표할 당시, SLAM 학계에서 카메라를 주 센서로 삼는 일은 이단에 가까웠다. Davison 자신이 논문에서 짚었듯, 당시 로봇공학자들은 거의 예외 없이 레이저 센서에 의존했다. 2D 레이저 스캐너는 거리를 곧바로 쟀고, 스테레오 카메라는 두 렌즈 사이의 시차로 픽셀 깊이를 즉각 뽑아낼 수 있었다. 반면 단안 카메라는 본질적으로 *방위(bearing)*만 줄 뿐 *깊이*를 주지 못한다. 3D 구조를 세우려면 최소한 서로 다른 두 시점의 프레임이 있어야 하는 데다, 처음 추정한 깊이의 막대한 불확실성이 EKF 상태 벡터 전체로 번져 나갔다. 그래서 이론상 가능하다는 것은 알았어도 30 Hz 실시간으로 돌리는 일은 난공불락으로 여겨졌다.

MonoSLAM은 바로 그 장벽을 뚫었다. 추가 하드웨어도, 까다로운 스테레오 캘리브레이션도 없이 값싼 카메라 한 대만으로 방 안의 3차원 구조를 실시간으로 빚어낸 첫 시연이었다.

---

## 5.2 EKF로 짠 단안 SLAM과 그 한계

2007년 *IEEE TPAMI*에 게재된 [MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049)은 Davison이 Ian Reid, Nicholas Molton, Olivier Stasse와 함께 ICCV 2003의 프로토타입을 완전한 이론과 시스템으로 다듬어 낸 결정판이다. 이들은 [Smith-Self-Cheeseman(1988)](https://arxiv.org/abs/1304.3111)과 [Leonard-Durrant-Whyte(1991)](https://ieeexplore.ieee.org/document/174711/)의 EKF-SLAM 정식화(→ §4.3)를 단안 카메라에 맞춰 정교하게 이식했다. 상태 벡터 $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$ 안에는 카메라 상태 $\mathbf{x}_v \in \mathbb{R}^{13}$(위치 3, 사원수 방향 4, 선속도 3, 각속도 3)과 3D landmark 좌표 $\mathbf{y}_i \in \mathbb{R}^3$이 함께 들어갔고, $(13+3N)\times(13+3N)$ 크기의 공분산 행렬 $\mathbf{P}$가 매 프레임 예측-갱신 루프를 돌며 갱신됐다. 손에 쥔 카메라에는 바퀴 오도메트리가 없어 다음 움직임을 가늠하려면 속도 예측이 필수였기에, 상태 벡터에 선속도와 각속도를 얹은 정속도(constant velocity) 운동 모델이 핵심 역할을 했다.

예측 단계에서는 운동 모델 $f$의 Jacobian $\mathbf{F}$로 공분산을 전파하고($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$), 갱신 단계에서는 카메라 투영 함수의 Jacobian $\mathbf{H}_i$로 Kalman gain을 구해 상태와 공분산을 다듬었다. 모든 landmark 간의 상관관계가 공분산 행렬 $\mathbf{P}$ 안에 빠짐없이 유지된 덕에 불확실성 전파는 수학적으로 빈틈이 없었지만, 대가는 가혹했다. 공분산 갱신 $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$의 핵심 연산인 $(13+3N)^2$ 행렬 곱셈이 특징점 수 $N$의 제곱으로 치솟은 것이다. PAMI 논문은 이 때문에 30 Hz 실시간성을 지키려면 지도 전체에 유지할 수 있는 특징점이 "around 100"개로 묶일 수밖에 없다고 명시했다.

더 넓은 공간으로 나아가려면 어떻게 해야 할까? Davison은 논문 결론에서 sub-map 전략이 "certainly seems appropriate"하다고 적으면서도, 시각적 특징으로 이루어진 지도를 의미 있는 하위 구역으로 깔끔히 나누는 방법은 아직 모호하다고 솔직히 덧붙였다. 특징이 100개 남짓으로 제한된 조건에서 최선의 성능을 짜내기 위해, MonoSLAM은 [Shi-Tomasi(1994)](https://doi.org/10.1109/CVPR.1994.323794)의 "Good Features to Track" 기준으로 추적성이 뛰어난 특징점만 소수 정예로 골라 담았다. 논문에 따르면 광각 렌즈를 쓴 환경에서도 프레임당 고작 12개 안팎의 특징점이 화면에 들어오도록 깐깐하게 지도를 관리했다.

> 🔗 **차용.** Shi-Tomasi(1994)의 코너 점수는 PTAM보다 앞서 MonoSLAM에서 먼저 시각 SLAM의 핵심 선별 기준으로 자리 잡았다. 뒤이은 PTAM 역시 빠른 검출은 FAST-10에 맡기되, 실제 지도에 영구 편입할 점은 Shi-Tomasi 점수로 가려내는 절충을 택했다.

---

## 5.3 2007년, 같은 해

2007년은 단안 Visual SLAM 역사에서 가장 극적인 해였다. 한쪽에서는 MonoSLAM의 집대성인 PAMI 논문이 출간돼 EKF 기반 단안 SLAM의 정점을 찍었고, 같은 해 열린 증강현실 학회 ISMAR에서는 Georg Klein과 David Murray가 전혀 다른 패러다임의 논문 [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852)(**PTAM**)를 발표했다.

두 시스템의 인연은 놀라울 만큼 가까웠다. Klein은 Cambridge에서 Tom Drummond의 지도로 박사학위를 마친 뒤 Oxford로 건너와 David Murray 연구실의 박사후연구원으로 일하고 있었는데, 공교롭게도 바로 그 Murray가 몇 해 전 Davison의 Oxford 박사과정 지도교수였다. 스승이 같은 연구실의 두 세대가 필터와 최적화라는 정반대의 해법을 내놓은 셈이다. Klein과 Murray는 PTAM 논문에서 MonoSLAM이 소수의 고품질 특징점으로 희소한 지도를 유지하는 전략을 직접 언급했다. 이어 자신들은 개별 특징의 품질에 덜 까다로운 대신 훨씬 조밀한 지도를 쌓고, EKF를 쓰지 않고 tracking과 mapping을 물리적으로 분리하는 길을 택했다고 선언했다.

---

## 5.4 Tracking과 Mapping을 나누다

PTAM의 발상은 단순하지만 혁명적이었다. EKF-SLAM에서는 카메라의 현재 자세를 쫓는 일(tracking)과 3D 지도를 빚고 고치는 일(mapping)이 매 프레임 단 하나의 예측-갱신 루프 속에 묶여 있었다. 그러다 보니 지도가 조금만 커져도 매 프레임 돌아가는 갱신 루프가 느려지며 전체 시스템이 멈춰 섰다. PTAM은 이 두 일을 **두 개의 병렬 스레드**로 완전히 갈라놓았다.

**Tracking 스레드**는 오직 실시간 카메라 pose 추정에만 전념한다. 이미 만들어진 지도의 3D 점들을 현재 프레임에 투영하고 관측된 코너와 짝지어 매 프레임 빠르게 자세만 푼다. 한편 **Mapping 스레드**는 한 박자 느긋하게 백그라운드에서 돈다. 새 keyframe이 들어올 때마다 가장 최근 keyframe과 그에 가까운 4개, 총 5개를 묶어 local bundle adjustment를 돌리고, CPU에 여유가 나면 지도 전체를 대상으로 global bundle adjustment를 수행한다. Mapping 계산이 다소 밀리더라도 Tracking 스레드는 지체 없이 30 Hz로 계속 굴러가므로, 실시간 추적이 끊기지 않는다.

Mapping 스레드가 푸는 최적화는 keyframe 집합 $\mathcal{K}$와 3D 점 집합 $\mathcal{P}$에 걸쳐 robust reprojection error를 최소화하는 문제였다.

$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$

여기서 $\mathbf{T}_k \in SE(3)$는 keyframe $k$의 자세, $\mathbf{p}_j \in \mathbb{R}^3$는 3D 점 좌표, $\pi$는 카메라 투영 함수, $\mathbf{z}_{kj}$는 관측된 픽셀 좌표, $\mathbf{\Sigma}_{kj}$는 관측 공분산이며, $\rho$는 오매칭의 영향력을 깎아 내리는 Tukey biweight robust kernel이다. Klein과 Murray는 이 비선형 최적화를 풀기 위해 Hartley와 Zisserman 교과서(*Multiple View Geometry* 2판) Appendix 6.6에 실린 Levenberg–Marquardt bundle adjustment를 "almost textbook" 수준으로 곧이곧대로 구현했다고 밝혔다.

이 비동기 분리 구조 덕분에 단안 SLAM의 지도 규모는 단숨에 두 자릿수에서 수천 단위로 뛰어올랐다. PTAM 논문 시연에서 지도는 무난히 점 2,000~6,000개와 keyframe 40~120개를 품었고, 시스템은 최대 점 11,000개와 keyframe 280개까지도 돌아갔다(다만 keyframe이 100개를 넘어서면 global bundle adjustment가 새 keyframe 유입 속도를 따라잡지 못하는 한계는 남았다). 특징점 100개 안팎을 아슬아슬하게 유지하던 MonoSLAM과는 체급 자체가 달라졌다.

초기 지도를 세우는 방식에서도 결별했다. 사용자가 카메라를 옆으로 부드럽게 움직이면 PTAM은 [Nistér(2004)](https://doi.org/10.1109/TPAMI.2004.17)의 5-point 알고리즘 계열을 이용해 첫 두 keyframe 사이의 essential matrix $\mathbf{E}$를 추정하고 초기 3D 점군을 삼각측량으로 복원했다. essential matrix $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$은 대응점 쌍 $(\mathbf{p}, \mathbf{p}')$에 대해 ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$을 만족하며 자유도가 5다. 최소 개수인 5쌍만으로 풀면 복소수 해를 포함해 최대 10개의 후보가 나오는데, Nistér가 이 5차 다항식 연립방정식을 초고속으로 푸는 대수적 해법을 열어 준 덕분에 RANSAC 루프 안에서 실시간 초기화가 가능해졌다.

> 🔗 **차용.** PTAM의 5-point 초기화는 Nistér(2004) "An Efficient Solution to the Five-Point Relative Pose Problem"에서 꽃피운 minimal solver 계보에 서 있다. PTAM 논문이 구현의 바탕으로 직접 인용한 것은 그 후속 연구인 Stewénius·Engels·Nistér(2006, ISPRS)였다.

> 🔀 **구조적 평행.** EKF-SLAM의 submap과 PTAM의 keyframe 지도는 계산량을 통제하려고 전체 지도를 지역 단위로 쪼개어 다룬다는 점에서 닮았다. 그러나 PTAM 논문이 submap 문헌을 설계 근거로 삼은 흔적은 보이지 않는다. Klein과 Murray는 모든 프레임을 지도에 다 넣지는 않고 mapping을 실시간 제약에서 해방하려는 실용적 계산 전략으로 keyframe을 택했고, 이 선택은 훗날 ORB-SLAM의 covisibility graph로 진화한다.

> 🔗 **차용.** PTAM의 Mapping 스레드가 돌리는 bundle adjustment는 1장에서 다룬 사진측량의 유산이다. 다만 흥미롭게도 PTAM은 [Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"](https://doi.org/10.1007/3-540-44480-7_21)가 아니라 Hartley·Zisserman 교과서 Appendix 6.6을 직접적인 구현 청사진으로 삼았다.

---

## 5.5 새 구조가 퍼지다

논문 제목에 박힌 "Small AR Workspaces"라는 문구처럼 PTAM은 원래 책상 위 증강현실(AR)을 겨냥해 태어났다. Tracking 스레드가 흔들림 없이 실시간 궤적을 뱉어 주니 가상 물체를 실제 책상 위에 올려놓는 데 손색이 없었다. 이 실용성 덕분에 tracking과 mapping을 나누는 2-스레드 아키텍처는 연구실 담장을 넘어 순식간에 산업계와 후속 SLAM 전반으로 번져 나갔다.

상용 AR SDK들도 이 구조를 빠르게 흡수했다. 독일 AR 스타트업 Metaio(2015년 Apple에 인수)는 2012년 자사 SDK에 SLAM을 탑재했는데, 연구진의 [ISMAR 2014 발표 자료](https://jsturm.de/publications/data/sturm2014ismar.pdf)를 보면 이 상용 엔진 역시 local·global bundle adjustment와 relocalization을 갖춘 전형적인 PTAM형 sparse SLAM이었다. 2015년 Raúl Mur-Artal, J.M.M. Montiel, Juan D. Tardós가 발표한 [ORB-SLAM](https://arxiv.org/abs/1502.00956)(TRO 2015)은 PTAM의 두 스레드 골격을 고스란히 물려받은 뒤, 이미지 패치를 ORB 서술자로 갈아 끼우고 keyframe 관리를 covisibility graph로 체계화하며 loop closure 스레드를 덧붙여 완성도를 끌어올렸다. 2018년 Qin, Li, Shen의 [VINS-Mono](https://arxiv.org/abs/1708.03852)(TRO 2018) 역시 sliding window 최적화와 loop closure를 별도 스레드로 분리했는데, 이를 보면 PTAM의 병렬 구조가 VIO 영역까지 표준으로 자리 잡았음을 알 수 있다.

---

## 5.6 Davison과 Klein·Murray의 관점

그렇다면 2007년 두 논문의 차이는 단순한 성능 우열의 문제였을까? 속내를 들여다보면 두 팀은 처음부터 지향하는 문제 정의 자체가 달랐다.

Davison이 PAMI 논문에서 full-covariance EKF를 끝까지 고수한 까닭은 명확했다. 그가 겨냥한 시나리오는 로봇이 긴 복도를 따라 탐사하는 경우가 아니라, 제한된 작업 공간 안에서 카메라가 같은 특징점을 수없이 반복 관측하며 오래 머무는 상황이었다. EKF는 모든 landmark가 얼마나 불확실한지, 또 landmark끼리 어떤 상관관계로 얽혀 있는지를 공분산 행렬로 정직하게 추적한다. Davison의 시각에서 보면 bundle adjustment는 바로 그 정교한 불확실성 모델링을 일부 포기하는 대신 확장성을 사 오는 타협안이었다.

반대로 PTAM은 그 타협을 기꺼이 감수했다. AR 응용에서 가장 절실한 것은 카메라의 현재 pose를 밀림 없이 실시간으로 추적하는 일이지, 지도 점 하나하나의 공분산을 엄밀하게 유지하는 일이 아니었기 때문이다. 지도는 그저 주기적인 bundle adjustment로 적당히 깎아 다듬기만 해도 충분했다.

역사의 저울은 결국 PTAM 쪽으로 기울었다. 2010년대 SLAM의 대세는 그래프 최적화와 bundle adjustment가 장악했다. 흥미로운 점은 그 이후 Davison 연구실의 행보다. Davison 역시 factor graph 기반 추정을 거쳐 분산 확률 추론인 Gaussian Belief Propagation(GBP)과 Robot Web 연구로 시선을 넓혀 갔다. [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release) Ch.18 "The Computational Structure of Spatial AI Systems"에서 Davison은 오늘날 기하 추정의 양대 기둥으로 EKF와 BA 같은 Gaussian 기반 기법을 꼽는다. 이어 그 자연스러운 확장으로 GBP를 소개하며, MonoSLAM과 PTAM을 손에 잡히는 오픈소스 연구 유산으로 나란히 회고한다.

---

## 예측과 실제

> **Davison 2007 PAMI MonoSLAM**: Davison은 결론에서 더 넓은 실내외 공간, 더 빠른 카메라 움직임, 가림과 조명 변화가 얽힌 복잡한 장면을 다음 과제로 지목했다. 구체적인 돌파구로는 sub-map 전략과 100 Hz 전 해상도 IEEE 1394 CMOS 카메라 도입을 꼽았고, 희소한 점 지도를 넘어 표면(surface) 같은 "higher-order entities"의 조밀한 표현으로 나아갈 수 있다고 내다봤다.
>
> 훗날 SLAM Handbook 15.5절은 현대 SLAM이 환경 변화와 빠른 움직임에 꽤 강건해졌다고 평가하면서도, 여러 극한 조건을 동시에 버텨 내는 효율적이고 강건한 SLAM은 여전히 열린 문제로 남겨 두었다(→ 19장, 23장). 한편 Davison이 결론에서 주요 수단으로 꼽지 않았던 IMU 결합은 본문에서 관련 연구로만 언급됐으나, 2010년대 Visual-Inertial Odometry(VIO) 붐의 핵심 엔진이 됐다.

> **Klein & Murray 2007 PTAM**: Klein과 Murray는 §8(Failure modes / Mapping inadequacies)에서 코너 기반 추적이 모션 블러에 취약하다는 점, 점 구름 위주 지도가 장면의 기하학적 구조를 이해하지 못한다는 점, 그리고 무엇보다 시스템이 "not designed to close large loops in the SLAM sense"라는 점을 솔직히 인정했다. 큰 루프를 닫지 못하는 이유로는 tracking의 M-estimator가 지도의 전역 불확실성을 알지 못한다는 점을 짚었다.
>
> 2015년 ORB-SLAM은 정확히 그 지점을 파고들었다. [DBoW2](http://doriangalvez.com/papers/GalvezTRO12.pdf) 외관 기반 장소 인식으로 큰 루프를 닫고 covisibility graph로 keyframe을 엮어 PTAM의 공간 제약을 부쉈다. 다만 모션 블러에 대한 취약성과 단순 점 구름 지도의 한계는 ORB-SLAM에서도 고스란히 숙제로 남았다.

---

## 아직 열린 문제

**단안 카메라의 scale 복원.** MonoSLAM부터 PTAM까지 순수 단안 카메라 시스템에는 피할 수 없는 기하학적 맹점이 있다. 장면 전체와 카메라 궤적을 똑같은 비율로 두 배 키우거나 절반으로 줄여도 카메라에 맺히는 영상은 한 픽셀도 달라지지 않는다는 **scale 모호성**이다. IMU를 결합하면 중력 가속도라는 물리적 자가 들어와 scale을 관측할 수 있지만, IMU 없는 순수 단안 시스템의 절대 scale 복원은 2026년에도 미해결이다. [MiDaS](https://arxiv.org/abs/1907.01341)나 [Depth Anything](https://arxiv.org/abs/2401.10891) 같은 학습 기반 단안 깊이 추정이 한 장의 사진에서 놀라운 상대 깊이를 뽑아내지만, 이를 미터 단위로 바꾸려면 지면 높이나 기지 물체 크기 같은 외부 닻이 필요하다. 미터 단위 깊이를 직접 예측한다는 모델들조차 훈련 분포를 벗어난 데이터셋에서는 scale이 어긋나는 사례가 여전히 보고된다(→ 14장 「아직 열린 문제」).

**어떤 환경에서나 도는 VO 시스템.** MonoSLAM은 작은 실내 책상 주변에 머물렀고, PTAM은 제목에서부터 "Small AR Workspaces"로 발을 뺐다. 훗날 ORB-SLAM2가 실내·실외·RGB-D를 아우르려 했지만, 조명이 급변하거나 하얀 벽처럼 텍스처가 메마른 복도에서는 지금도 추적이 맥없이 끊어진다. 카메라와 LiDAR, IMU를 함께 엮는 멀티센서 융합이 그 빈틈을 상당 부분 메워 가고 있을 뿐이다.

**어두운 곳과 움직이는 장면의 특징점 추적.** MonoSLAM도 PTAM도 밝기가 충분하고 세상이 정지해 있다는 전제 위에서 돌았다. 놀랍게도 2026년 오늘날의 특징점 기반 SLAM 대부분이 여전히 이 두 가정을 은연중에 공유한다. 어둠 속에서는 ORB 특징점이 잡히지 않고, 행인이 오가는 거리에서는 움직이는 사람 위의 점을 고정된 벽처럼 오인해 지도를 망친다. 학습 기반 optical flow나 semantic segmentation을 끌어와 이 오래된 가정을 깨려는 연구가 지금도 진행 중이다.

---

PTAM이 tracking과 mapping을 떼어 내 실시간 최적화의 물꼬를 텄지만, keyframe이 쌓이고 동선이 길어질수록 서서히 커지는 누적 오차(drift)를 루프 전체에서 한꺼번에 바로잡는 문제는 여전히 남아 있었다. 사실 그 문제를 푸는 그래프 최적화의 수학적 청사진은 PTAM보다 꼭 10년 앞선 1997년, 캐나다 York University의 두 연구자가 레이저 스캔 정합 문제를 다루며 이미 세상에 내놓아 둔 상태였다(→ 6장).
