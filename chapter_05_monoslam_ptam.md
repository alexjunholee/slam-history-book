# Ch.5 — MonoSLAM → PTAM: 실시간의 몽상과 분리 혁명

앞 챕터는 EKF-SLAM이 어떻게 확률론적으로 일관된 지도 구축 방법을 완성했는지, 그리고 그 공분산 행렬이 landmark 수 $N$에 대해 $O(N^2)$로 커지는 구조적 벽에 부딪혔는지를 보였다. 이론이 틀려서가 아니라, 설계가 그렇게 생겼을 뿐이다. Davison과 Klein은 여기서부터 각자 다른 방향으로 걸었다.

2003년, Davison은 Imperial College 실험실에서 웹캠 한 대를 노트북에 꽂았다. 1988년 Smith와 Cheeseman이 세운 확률 공간관계 수학, 그 위에 Leonard와 Durrant-Whyte가 얹은 EKF-SLAM 틀을 그대로 가져왔지만 센서는 카메라 하나뿐이었다. IMU도 스테레오도 레이저도 없는 상태에서 Shi-Tomasi 1994 코너 검출기와 Kalman 예측-갱신 루프만 붙여 실시간으로 돌렸다. 당시 기준으로 무모한 조합이었다.

4년 뒤 2007년, Oxford의 Klein과 Murray가 같은 해에 다른 답을 냈다. tracking과 mapping을 두 스레드로 쪼갠 것이다. 그 분리가 이후 10년 Visual SLAM의 뼈대가 되었다.

---

## 1. 2003년의 데모

2003년 ICCV에서 Davison이 공개한 [Real-Time Simultaneous Localisation and Mapping with a Single Camera](https://doi.org/10.1109/ICCV.2003.1238654)는 장내를 술렁이게 했다. 결과가 놀라웠기 때문이 아니다. *그것이 가능하다는 것*이 충격이었다.

당시 SLAM 분야의 주류는 레이저 센서였다. LiDAR는 2D 거리를 직접 제공했고, 스테레오 카메라는 픽셀 수준에서 깊이를 복원했다. 단안 카메라는 깊이 정보 자체가 없었다. 단안으로 3D 구조를 추정하려면 최소 두 프레임이 필요했고, 초기 깊이 추정의 불확실성이 EKF 상태 벡터 전체로 전파되었다. 이론적으로 가능했지만 실시간으로 돌린다는 것은 별개의 문제였다.

Davison이 단안을 고른 것은 실용적 제약 때문이었다. IMU는 추가 하드웨어였고, 스테레오는 캘리브레이션 부담이 있었다. 그가 원한 것은 "카메라 하나로 증명하는 것"이었다. 증명에 성공하면 나머지는 얹을 수 있었다. 그 논리는 맞았다. 틀린 것은 EKF가 그 "나머지"를 실제로 수용할 수 있는 구조인지였다.

---

## 2. EKF의 아름다움과 벽

2007년 IEEE PAMI에 실린 [MonoSLAM](https://doi.org/10.1109/TPAMI.2007.1049)는 Davison, Ian Reid, Nicholas Molton, Olivier Stasse의 공동 저자로, ICCV 2003 데모의 완성된 논문 형태였다.

MonoSLAM의 상태 벡터는 [Smith-Cheeseman(1988)](https://arxiv.org/abs/1304.3111)과 [Leonard-Durrant-Whyte(1991)](https://ieeexplore.ieee.org/document/174711/)의 정식(Ch.4)을 단안 카메라에 직접 이식했다. 카메라 상태 $\mathbf{x}_v \in \mathbb{R}^{13}$ — 위치 3, 사원수 방향 4, 속도 3, 각속도 3 — 과 landmark 집합 $\mathbf{y}_i \in \mathbb{R}^3$를 하나의 벡터 $\mathbf{x} = (\mathbf{x}_v^\top, \mathbf{y}_1^\top, \ldots, \mathbf{y}_N^\top)^\top \in \mathbb{R}^{13+3N}$에 담고, 그 전체 공분산 $(13+3N)\times(13+3N)$ 행렬 $\mathbf{P}$를 매 프레임 predict-update 루프로 유지했다. predict 단계에서는 카메라 운동 모델 $f$의 자코비안 $\mathbf{F}$로 공분산을 전파했고($\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$), update 단계에서는 투영 함수의 자코비안 $\mathbf{H}_i$로 칼만 이득을 계산해 상태와 공분산을 갱신했다. EKF predict-update 수식 자체는 Ch.4 §4.3의 것과 동일하다. 달라진 것은 상태 벡터 안에 카메라 속도·각속도가 함께 들어간 점이었다(이동 물체인 카메라의 동역학 모델이 필요했기 때문이다).

공분산 갱신 $(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}^-$의 지배 비용은 $(13+3N)^2$ 행렬 곱셈으로, landmark 수 $N$에 대해 $O(N^2)$였다. 논문 §III은 30 Hz 실시간 처리에서 유지 가능한 feature 수의 상한이 "약 100개" 수준이라고 명시한다.

> 🔗 **차용.** MonoSLAM의 EKF 상태 벡터 구조는 Smith-Cheeseman-Durrant-Whyte(1988-1991)의 확률적 공간관계 표현을 단안 카메라에 직접 이식한 것이다. Kalman 필터 자체는 1960년부터 있었지만, 로봇 pose와 landmark를 같은 벡터에 넣는 "augmented state vector" 관행이 확립된 것은 Leonard-Durrant-Whyte 1991의 스타일이었다.

이 숫자는 시스템 한계를 드러냈다. Davison은 이를 알고 있었다. Davison은 논문에서 sub-mapping 전략으로의 확장을 향후 방향으로 제시했다. 그러나 EKF 내부에서 계층적 구조를 만드는 것은 근본적으로 어려웠다. 공분산 행렬이 모든 landmark 간 상관관계를 빠짐없이 담고 있었기 때문이다.

[Shi-Tomasi(1994)](https://doi.org/10.1109/CVPR.1994.323794) 코너가 MonoSLAM의 시각 특징으로 선택된 것도 이 맥락에서 읽힌다. "Good Features to Track"의 선택 기준은 추적하기 좋은 점을 고르는 것이었다. 애초에 추적이 실패할 가능성이 낮은 코너만 상태 벡터에 넣으면 EKF의 갱신이 더 안정적이었다. PAMI 논문은 광각 렌즈에서 매 프레임 약 12개의 특징이 안정적으로 보이도록 map management를 구성한다고 명시한다. 이 한정된 수의 특징이 모두 잘 추적되는 한, EKF는 돌아갔다.

> 🔗 **차용.** PTAM이 아니라 MonoSLAM에서 이미 Shi-Tomasi 1994의 코너 검출기가 쓰였다. "좋은 특징을 선택해서 추적한다"는 설계 철학은 Shi-Tomasi → MonoSLAM → PTAM의 직접적인 계보다.

---

## 3. 2007년, 같은 해

그 한정된 숫자가 EKF의 천장을 드러냈다. Oxford에서 그 천장을 올려다보던 사람이 Klein이었다.

2007년 ISMAR에 Klein과 Murray가 [Parallel Tracking and Mapping for Small AR Workspaces](https://doi.org/10.1109/ISMAR.2007.4538852)를 올렸다. 같은 해 PAMI에는 Davison의 MonoSLAM 정식판이 실렸다. 두 논문이 한 해에 나온 건 우연이 아니었다.

Klein은 당시 Murray 그룹 박사과정이었다. Murray 그룹은 Oxford Active Vision Laboratory의 직계였고, 몇 년 전까지 Davison이 박사과정 학생으로 있던 바로 그 방이었다. Murray는 Davison의 지도교수였다. Klein이 MonoSLAM을 보지 않았을 수 없다. 그가 본 건 EKF가 아니라, 단안 카메라가 실시간으로 돈다는 사실 그 자체였다.

가능성은 확인됐다. 남은 건 "어떻게 확장할 것인가"였다. Klein은 EKF를 버리기로 했다.

---

## 4. 분리

PTAM의 핵심 아이디어는 하나였다. Tracking(카메라 pose 추적)과 Mapping(3D 지도 구축)을 분리해서 두 개의 병렬 스레드로 실행한다.

EKF에서 이 둘은 같은 루프 안에 섞여 있었다. 매 프레임마다 예측-갱신 한 사이클을 돌리면서, 카메라가 움직이면 상태를 예측하고 이미지에서 landmark를 찾으면 다시 갱신했다.

PTAM은 이것을 풀었다. Tracking 스레드는 매 프레임 카메라 pose를 추정하는 일만 한다. 현재 keyframe 집합에서 보이는 3D 점들의 2D 투영과 실제 관측을 매칭해서 pose를 실시간으로 계산한다. Mapping 스레드는 새 keyframe이 추가될 때마다 bundle adjustment를 실행한다. Tracking 스레드가 독립적으로 돌아가기 때문에 Mapping이 느려져도 무방했다.

Mapping 스레드의 bundle adjustment는 keyframe 집합 $\mathcal{K}$와 3D 점 집합 $\mathcal{P}$에 대해 재투영 오차의 합을 최소화했다:
$$\min_{\{\mathbf{T}_k\}, \{\mathbf{p}_j\}} \sum_{k \in \mathcal{K}} \sum_{j \in \mathcal{P}_k} \rho\!\left(\left\|\mathbf{z}_{kj} - \pi(\mathbf{T}_k,\, \mathbf{p}_j)\right\|^2_{\mathbf{\Sigma}_{kj}}\right)$$
여기서 $\mathbf{T}_k \in SE(3)$는 keyframe $k$의 pose, $\mathbf{p}_j \in \mathbb{R}^3$는 3D 점, $\pi$는 카메라 투영 함수, $\mathbf{z}_{kj}$는 keyframe $k$에서 점 $j$의 관측 픽셀 좌표, $\mathbf{\Sigma}_{kj}$는 측정 공분산, $\rho$는 Huber 함수 등의 robust kernel이다. Mapping 스레드는 이 최적화를 Levenberg–Marquardt로 반복해서 풀었다. 비동기로 돌기 때문에 Tracking 스레드의 실시간성에 영향을 주지 않았다.

> 🔗 **차용.** PTAM의 Mapping 스레드에서 실행되는 bundle adjustment는 [Triggs et al. 1999 "Bundle Adjustment — A Modern Synthesis"](https://doi.org/10.1007/3-540-44480-7_21)의 직접 적용이다. 1부에서 다룬 사진측량의 100년 전통이 SLAM backend에 처음으로 제대로 자리를 잡은 지점이 여기다. EKF에서는 공분산 행렬의 크기 제약 때문에 전체 BA가 불가능했다. 스레드 분리로 그 제약이 사라졌다.

이 분리는 단순해 보이지만 결과는 달랐다. Mapping 스레드가 비동기로 bundle adjustment를 실행하기 때문에, 지도에 들어갈 수 있는 landmark 수가 EKF의 $O(N^2)$ 제약을 벗어났다. PTAM이 사용한 keyframe의 수는 수백 개였다. 각 keyframe에는 수백 개의 patch feature가 있었다. MonoSLAM의 수십 landmark 규모와는 다른 세계였다.

초기 맵 구축 방법도 달랐다. PTAM은 사용자가 카메라를 천천히 움직이는 초기화 단계에서 [Nistér 2004](https://doi.org/10.1109/TPAMI.2004.17)의 5-point 알고리즘 계열(PTAM 논문은 그 후속인 Stewénius·Engels·Nistér 2006을 인용)로 essential matrix를 추정하고, 첫 keyframe 쌍에서 초기 3D 구조를 복원했다. 이것 역시 차용이었다.

Essential matrix $\mathbf{E}$는 두 카메라 좌표계 사이의 순수 기하관계를 담는 $3\times 3$ 행렬로, 대응점 쌍 $(\mathbf{p}, \mathbf{p}')$에 대해 ${\mathbf{p}'}^\top \mathbf{E}\, \mathbf{p} = 0$을 만족한다. $\mathbf{E}$는 내부적으로 $\mathbf{E} = \mathbf{t}_\times \mathbf{R}$ ($\mathbf{t}_\times$는 병진의 반대칭 행렬, $\mathbf{R}$은 회전)으로 분해되므로 자유도가 5이다. 따라서 최소 5쌍의 대응점으로 유일해(최대 10개 실수 해)를 구할 수 있다. Nistér의 기여는 이 5-point 연립방정식을 Gröbner basis를 이용해 효율적으로 풀어 RANSAC 루프 안에서 실시간으로 돌릴 수 있게 한 것이다. PTAM은 이 solver를 초기화 단계에서 RANSAC과 함께 사용해 첫 두 keyframe 사이의 상대 pose를 추정하고 초기 3D 점군을 삼각측량으로 복원했다.

> 🔗 **차용.** PTAM의 5-point essential matrix 초기화는 David Nistér 2004 "An Efficient Solution to the Five-Point Relative Pose Problem"이 열어 놓은 minimal-solver 계보를 따른다(PTAM 논문은 그 후속 Stewénius·Engels·Nistér 2006 ISPRS를 직접 인용). 5-point solver는 단안 카메라의 초기 맵 구축에 필요한 최소 대응쌍을 사용하는 minimal solver였고, PTAM은 이 솔버를 RANSAC 루프에 태워 초기 두 keyframe의 상대 pose를 실시간에 가깝게 추정했다.

> 🔗 **차용.** PTAM의 keyframe 구조는 Leonard-Durrant-Whyte의 submap 아이디어에서 맥이 닿는다. "전체 맵을 한 번에 최적화하기 어려우면 지역 단위로 나눈다"는 발상이 PTAM에서는 keyframe 집합으로 표현되었다. 후속 ORB-SLAM의 covisibility graph는 이 keyframe 관리를 더 정교하게 만든 버전이다.

---

## 5. 새 아키텍처의 확산

PTAM은 AR(증강현실) 워크스페이스를 대상으로 설계되었다. 논문 제목에도 "Small AR Workspaces"가 명시되어 있다. Tracking 스레드의 재현성이 좋았고, 실시간성이 확실했기 때문에 AR 응용에 바로 쓸 수 있었다.

상업적 흡수는 빠르게 일어났다. 2010년대 초 Metaio(독일 AR 스타트업, 2015년 Apple에 인수)와 Qualcomm의 Vuforia SDK는 PTAM과 유사한 tracking/mapping 분리 구조를 채용했다. 소비자 스마트폰에서 처음으로 안정적인 planar AR이 돌아갔다.

학계에서의 영향은 더 직접적이었다. 2015년 Raul Mur-Artal, J.M.M. Montiel, Juan D. Tardós가 발표한 [ORB-SLAM](https://arxiv.org/abs/1502.00956)은 PTAM의 구조를 계승했다. 특징점은 patch에서 ORB 디스크립터로 바꾸고, keyframe 관리는 covisibility graph로 정교화했으며, loop closure를 새로 얹었다. PTAM이 없었으면 ORB-SLAM의 설계도가 달랐을 것이다.

2018년 Qin, Li, Shen의 [VINS-Mono](https://arxiv.org/abs/1708.03852) 역시 sliding window 최적화 + loop closure의 이중 스레드 구조를 갖는다. tracking/mapping 분리의 계보가 VIO로 확장된 사례다.

---

## 6. Davison vs Klein & Murray — 관점 비교

2007년에 두 논문이 나왔다. MonoSLAM PAMI는 2003년 데모의 완성판이었다. PTAM은 같은 해에 MonoSLAM의 한계를 돌파하는 새 구조로 나왔다.

MonoSLAM이 EKF를 붙들고 있었던 이유는 확률론적 일관성(consistency)에 있었다. EKF는 상태의 불확실성을 공분산 행렬로 명시적으로 관리했다. 지도의 각 landmark가 얼마나 불확실한지, landmark 간 공분산이 어떻게 연결되는지를 수학이 추적했다. 이 관점에서 bundle adjustment는 최소자승 최적화였고, 불확실성 표현을 줄이는 대신 확장성을 얻는 거래로 읽혔다.

Klein & Murray는 그 대가를 기꺼이 치렀다. AR 응용에서 중요한 것은 카메라 pose의 실시간 추적이었다. 지도의 불확실성을 센티미터 단위로 추적할 필요는 없었다. Bundle adjustment로 지도를 주기적으로 refine하면 충분했다.

이후 SLAM 분야의 방향은 이 거래 쪽으로 기울었다. 2010년대 이후 graph-based 최적화와 bundle adjustment가 주류가 되었고, EKF-SLAM은 계산 자원이 극도로 제한된 응용 외에서는 대부분 전면에서 물러났다. 다만 MonoSLAM이 붙들었던 확률론적 관심사가 사라진 것은 아니었다. Davison의 lab은 PTAM 계보로 건너뛰는 대신 이후 몇 단계에 걸쳐 방향을 바꾸어 factor graph 기반의 추정, 그리고 Gaussian Belief Propagation(GBP)·Robot Web 쪽으로 옮겨갔다. 23년 뒤 SLAM Handbook Ch.18에서 Davison은 같은 흐름을 EKF→BA→factor graph→GBP로 이어지는 representation 변경의 연속으로 해석한다. 본인이 MonoSLAM을 직접 호명해 평가하는 대목은 없고, 각 표현 교체가 시스템 전체의 재설계를 유발한다는 일반 원리로 치환해 서술한다.

---

## 📜 예언 vs 실제

> **Davison 2007 PAMI MonoSLAM**: Davison은 Conclusion에서 더 큰 실내·실외 환경, 더 빠른 움직임, 가림·조명 변화가 있는 복잡한 장면을 다음 과제로 꼽았다. 구체 수단으로 sub-map 전략과 100 Hz 이상의 고프레임률 CMOS 카메라를 거론했고, sparse map을 "higher-order entities"(표면 등)의 dense 표현으로 확장할 여지도 함께 언급했다.
>
> 이 예측들의 운명은 각기 달랐다. Sub-map 아이디어는 PTAM의 keyframe 구조와 ORB-SLAM의 covisibility graph를 거쳐 부분적으로 흡수되었다. 그러나 EKF를 유지하면서 계층적 확장을 달성한 시스템은 나오지 않았다 — 계층화는 BA 기반 아키텍처 전환과 함께 왔다. 고프레임률 카메라는 2010년대 이벤트 카메라 연구에서 다른 경로로 구체화되었다. 동적 장면 강건성은 2026년 기준 여전히 열려 있다. DynaSLAM, FlowSLAM 등 여러 시도가 있었지만 "기본 파이프라인에 포함된 해법"은 아직 없다. IMU 통합은 Davison이 Future Work에서 직접 지목하진 않았지만(관련 연구는 논문 본문에서 참조) 2010년대 Visual-Inertial Odometry(VIO) 연구 붐이 맡은 방향이다. 확률론적 일관성이라는 관심사 자체는 폐기되지 않고 factor graph·GBP 쪽으로 옮겨갔다 — Davison 본인은 23년 뒤 Handbook Ch.18에서 이 이동을 "representation 변경의 연속"으로 묘사하며 MonoSLAM을 계보의 한 단계로 재배치한다. `[진행형]`

> **Klein & Murray 2007 PTAM**: Klein과 Murray는 §8(Failure modes / Mapping inadequacies)에서 시스템의 한계로 corner-기반 추적의 모션 블러 취약성, point cloud 중심의 지도가 가진 기하 이해 부족, 그리고 "not designed to close large loops in the SLAM sense"를 열거했다. 즉, 큰 루프의 전역 일관성 확보가 PTAM의 설계 범위 밖임을 분명히 했다.
>
> 2015년 ORB-SLAM은 이 한계들을 정면으로 겨냥했다. [DBoW2](http://doriangalvez.com/papers/GalvezTRO12.pdf) 기반 appearance loop closure와 covisibility graph 기반 keyframe 관리가 얹혔고, 특징은 patch 대신 ORB descriptor로 교체됐다. PTAM이 "우리 문제가 아니다"라고 선을 그은 곳에서 ORB-SLAM이 지도 확장을 이어 받은 구도다. Klein & Murray 자신이 명시적으로 "appearance-based loop closure가 답"이라고 적은 것은 아니지만, 한계 지점의 지적이 후속 계보의 출발점으로 정확히 맞았다. `[한계 지점 적중]`

---

## 🧭 아직 열린 것

**Monocular scale 복원.** MonoSLAM부터 PTAM까지, 단안 카메라 시스템은 모두 scale ambiguity를 안고 있다. 이미지 한 장에서 절대 거리를 알 수 없다는 것은 기하학적 사실이다. IMU를 추가하면 중력 방향과 가속도계 판독값으로 scale이 observability를 갖는다. 그러나 IMU 없는 순수 monocular 시스템에서 scale 복원은 2026년에도 근본적으로 해결되지 않았다. 학습 기반 monocular depth estimation([MiDaS](https://arxiv.org/abs/1907.01341), [Depth Anything](https://arxiv.org/abs/2401.10891))이 단일 이미지에서 상대적 깊이를 추정하지만, 이것을 metric scale로 변환하려면 여전히 외부 참조(지면 가정, 사전 알려진 물체 크기 등)가 필요하다.

**단일 VO 시스템의 환경 범용성.** MonoSLAM은 실내 데스크탑 환경만 다루었다. PTAM은 "Small AR Workspaces"라고 스스로 범위를 제한했다. 이후 ORB-SLAM2가 실내·실외·RGB-D를 아우르려 했지만, 조명 변화가 극단적인 환경이나 low-texture 공간에서는 여전히 tracking failure가 발생한다. 단일 파이프라인이 실내 복도, 야외 도심, 야간 환경, 텍스처 없는 흰 벽 전부를 견고하게 처리하는 시스템은 2026년 기준 아직 없다. Multi-modal fusion(카메라 + LiDAR + IMU)이 일부 커버하지만, 카메라 단독 시스템의 범용성은 여전히 미결이다.

**저조도·동적 환경에서의 특징점 추적.** MonoSLAM이 요구했던 것은 충분한 조명과 정적인 장면이었다. 2007년의 PTAM도 마찬가지였다. 2026년 현재 이 두 가정은 여전히 대부분의 feature-based SLAM 시스템에서 암묵적으로 유지된다. 저조도에서 ORB feature는 검출 자체가 실패하고, 움직이는 사람이 많은 장면에서는 dynamic point가 static point로 잘못 분류된다. 이 문제를 학습 기반 optical flow나 semantic segmentation으로 우회하는 시도가 있지만, 실시간 범용 해법으로 자리잡은 시스템은 아직 없다.

---

PTAM이 확립한 tracking/mapping 분리는 한 가지를 해결하지 못했다. keyframe이 쌓일수록 누적 오차가 loop에서 폭발했다. 그 답은 PTAM과 같은 해에 나온 것이 아니었다. 1997년, CMU 지하 복도에서 Feng Lu와 Evangelos Milios가 레이저 스캔 문제를 붙들고 있던 바로 그 시점에 이미 형태를 갖추고 있었다.
